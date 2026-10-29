/**
 * @file imu_core.c
 * @brief The IMU producer state machine: filter, calibration, heading, turn
 *        rate, terrain, hump integration, collision, faults, and events.
 *
 * This file includes no Pico SDK or RTOS header, so the host tests exercise
 * exactly the code that runs on the board.
 */
#include "imu_core.h"

#include <math.h>
#include <string.h>

#include "imu_config.h"
#include "imu_hal.h"
#include "imu_lsm303.h"
#include "imu_snapshot.h"

#define IMU_PI              3.14159265358979323846f
#define IMU_RAD_TO_DEG      (180.0f / IMU_PI)
#define IMU_DEG_TO_RAD      (IMU_PI / 180.0f)

/** Saturation for the consecutive-sample counters. */
#define COUNT_CAP           1000u

/** Largest event value that still fits the signed 16-bit payload. */
#define EVENT_VALUE_MAX     32700

/** Fault detail codes carried in IMU_EVT_SENSOR_FAULT. */
#define FAULT_DETAIL_BUS    1u
#define FAULT_DETAIL_INVALID 2u

/** Bounded reader retries for the two-word seqlock. */
#define SNAPSHOT_READ_ATTEMPTS 8

/**
 * Optional test hook, invoked between the two snapshot word loads.
 *
 * The host build defines IMU_TEST_SNAPSHOT_HOOK to force a publish in the
 * middle of a read, which proves the reader retries instead of returning a
 * torn pair. A production build compiles the hook away.
 */
#ifdef IMU_TEST_SNAPSHOT_HOOK
void imu_test_snapshot_hook(void);
#define IMU_SNAPSHOT_READ_HOOK() imu_test_snapshot_hook()
#else
#define IMU_SNAPSHOT_READ_HOOK() do { } while (0)
#endif

typedef enum
{
    IMU_PHASE_SELFTEST = 0,
    IMU_PHASE_CALIBRATING,
    IMU_PHASE_RUNNING,
    IMU_PHASE_FAULT
} imu_phase_t;

/**
 * Private episode phase. Never exposed: the public terrain enum stays
 * INIT / LEVEL / ON_HUMP / SENSOR_FAULT.
 *
 * It records how the episode began, so a pitch-led ascent can hold ON_HUMP
 * across a broad, flat crest instead of closing after the level dwell and
 * opening a second episode on the descent.
 */
typedef enum
{
    HUMP_LEAD_NONE = 0,     /**< Pose-led, e.g. a roll-only one-wheel case.  */
    HUMP_LEAD_UP,           /**< Began nose-up: needs a descent to close.   */
    HUMP_LEAD_DOWN          /**< Began nose-down: may close once level.     */
} hump_lead_t;

typedef struct
{
    imu_phase_t   phase;
    imu_terrain_t terrain;

    /* Self-test staging. */
    bool     needs_configure;
    uint8_t  selftest_wait;
    uint16_t selftest_cycles;
    bool     selftest_passed;

    /* Stationary calibration. */
    bool     calibrated;
    float    accel_bias_g[IMU_AXIS_COUNT];
    float    gravity_gain;              /**< 1.0 until a window accepts one.    */
    uint32_t cal_start_ms;
    uint32_t cal_count;
    float    cal_mean[IMU_AXIS_COUNT];
    float    cal_mag_mean;
    float    cal_mag_m2;
    float    cal_min_mag;
    float    cal_max_mag;
    uint32_t last_reg_check_ms;

    /* Low-pass filter state. */
    bool  filt_init;
    float accel_filt_g[IMU_AXIS_COUNT];
    float mag_filt_gauss[IMU_AXIS_COUNT];

    /* Derived motion. */
    float    pitch_deg;
    float    roll_deg;
    float    tilt_deg;
    float    heading_deg;
    float    turn_rate_dps;
    bool     heading_valid;
    float    heading_prev_deg;
    bool     heading_prev_valid;
    uint32_t last_ms;
    bool     last_ms_valid;

    /* Terrain and hump. One ON_HUMP state, so one counter per direction of the
     * hysteresis: entry while LEVEL, exit while ON_HUMP. */
    uint16_t hump_enter_cnt;
    uint16_t hump_exit_cnt;

    /* Private episode phase (RT-0027). Never exposed in the public enum. */
    hump_lead_t lead;
    bool        descent_seen;
    uint16_t    descent_cnt;
    float    hump_mm;
    float    hump_peak_mm;
    uint16_t highest_peak_mm;

    /* Terrain pause from the Manager recovery gate (RT-0023). */
    bool     terrain_pause;
    bool     terrain_resume_reset;

    /* Collision. */
    bool    collision_latched;
    uint8_t collision_cnt;
    uint8_t collision_release_cnt;

    /* Fault handling and run odometer. */
    uint32_t retry_at_ms;
    uint32_t run_distance_mm;

    /* Encoder input, consumed once per cycle. */
    bool                enc_pending;
    imu_encoder_input_t enc;

    /* Event queue. */
    imu_event_t queue[IMU_CFG_EVENT_QUEUE_LEN];
    uint8_t     q_head;
    uint8_t     q_count;

    /* Published validity and packet. */
    bool           accel_valid;
    bool           mag_valid;
    uint8_t        sequence;

    /**
     * Two-word snapshot, guarded by a seqlock.
     *
     * Both words are volatile so the compiler keeps them in order relative to
     * the sequence counter. The counter is odd while publisher() is writing
     * and even when the pair is stable, so a reader that sees a change retries
     * and never returns a word-1 sample with a word-2 sample from another
     * cycle.
     */
    volatile uint32_t snapshot_word[2];
    volatile uint32_t snapshot_seq;
    imu_diag_t diag;
} imu_state_t;

static imu_state_t g;

/**
 * Axis remap. This is the only place the mounting orientation lives.
 * Body axes are X right, Y forward, Z up.
 */
static const imu_axis_map_t ACCEL_MAP = IMU_CFG_ACCEL_AXIS_MAP;
static const imu_axis_map_t MAG_MAP   = IMU_CFG_MAG_AXIS_MAP;

/** Gravity expected during calibration, in milli-g, in body axes. */
static const float EXPECTED_GRAVITY_MG[IMU_AXIS_COUNT] = IMU_CFG_CAL_GRAVITY_BODY_MG;

/* ---- Small helpers ----------------------------------------------------- */

static float
norm3 (const float v[IMU_AXIS_COUNT])
{
    return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static bool
finite3 (const float v[IMU_AXIS_COUNT])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static bool
normalize3 (float v[IMU_AXIS_COUNT])
{
    float const n = norm3(v);

    if (!isfinite(n) || !(n > 0.0f))
    {
        return false;
    }

    v[0] /= n;
    v[1] /= n;
    v[2] /= n;

    return true;
}

static float
clampf (float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }

    return (v > hi) ? hi : v;
}

/** Wrap an angle difference into -180..180. */
static float
wrap180 (float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }

    while (deg < -180.0f)
    {
        deg += 360.0f;
    }

    return deg;
}

static void
remap3 (const imu_axis_map_t * map, const float in[IMU_AXIS_COUNT],
        float out[IMU_AXIS_COUNT])
{
    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        int const src = (int) map->src[a];
        int const use = ((src >= 0) && (src < IMU_AXIS_COUNT)) ? src : a;

        out[a] = ((float) map->sign[a]) * in[use];
    }
}

static void
low_pass3 (float alpha, float state[IMU_AXIS_COUNT],
           const float in[IMU_AXIS_COUNT])
{
    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        state[a] += alpha * (in[a] - state[a]);
    }
}

/**
 * Pitch in degrees, positive nose-up.
 *
 * Gravity's reaction in the body frame is (0, sin(pitch), cos(pitch)) * 1 g
 * for a pure pitch, so the forward axis reads a positive +Y component when
 * the nose is up.
 */
static float
pitch_from_accel (const float accel_g[IMU_AXIS_COUNT])
{
    float const horizontal = sqrtf((accel_g[0] * accel_g[0]) +
                                   (accel_g[2] * accel_g[2]));

    return atan2f(accel_g[1], horizontal) * IMU_RAD_TO_DEG;
}

/**
 * Roll in degrees, positive when the right side of the chassis drops.
 *
 * Body +X is right and +Z up, so a positive rotation about the forward axis
 * drives the measured +X component negative: a negative reading means the right
 * side is up, which is what one wheel on a hump produces.
 */
static float
roll_from_accel (const float accel_g[IMU_AXIS_COUNT])
{
    float const horizontal = sqrtf((accel_g[1] * accel_g[1]) +
                                   (accel_g[2] * accel_g[2]));

    return atan2f(accel_g[0], horizontal) * IMU_RAD_TO_DEG;
}

/**
 * Non-directional resultant tilt in degrees, relative to the flat reference.
 *
 * This is the ON_HUMP classifier signal. It cannot tell a front-to-back ramp
 * from a one-wheel pose, which is exactly why it catches both, where pitch
 * alone misses the one-wheel case and roll alone would miss the ramp. Pitch
 * stays the only input to height integration; tilt never contributes height.
 */
static float
tilt_from_accel (const float accel_g[IMU_AXIS_COUNT])
{
    float const horizontal = sqrtf((accel_g[0] * accel_g[0]) +
                                   (accel_g[1] * accel_g[1]));

    return atan2f(horizontal, accel_g[2]) * IMU_RAD_TO_DEG;
}

/** Attitude from one acceleration vector: pitch, roll and resultant tilt. */
static void
set_attitude (const float accel_g[IMU_AXIS_COUNT])
{
    g.pitch_deg = pitch_from_accel(accel_g);
    g.roll_deg  = roll_from_accel(accel_g);
    g.tilt_deg  = tilt_from_accel(accel_g);
}

/**
 * Tilt-compensated heading in degrees, clockwise from magnetic north, where
 * north is the direction of the body +Y axis projected onto the horizontal
 * plane. Only the direction of the magnetic vector matters, so no hard or
 * soft iron calibration is implied by this function.
 */
static bool
heading_from_vectors (const float accel_g[IMU_AXIS_COUNT],
                      const float mag_gauss[IMU_AXIS_COUNT],
                      float * heading_deg)
{
    float up[IMU_AXIS_COUNT] = { accel_g[0], accel_g[1], accel_g[2] };

    if (!normalize3(up))
    {
        return false;
    }

    float east[IMU_AXIS_COUNT] =
    {
        (mag_gauss[1] * up[2]) - (mag_gauss[2] * up[1]),
        (mag_gauss[2] * up[0]) - (mag_gauss[0] * up[2]),
        (mag_gauss[0] * up[1]) - (mag_gauss[1] * up[0])
    };

    /* A magnetic vector parallel to gravity has no horizontal component. */
    if (!normalize3(east))
    {
        return false;
    }

    float const north[IMU_AXIS_COUNT] =
    {
        (up[1] * east[2]) - (up[2] * east[1]),
        (up[2] * east[0]) - (up[0] * east[2]),
        (up[0] * east[1]) - (up[1] * east[0])
    };

    /* Body +Y is forward, so project it on the north/east pair. */
    float const fwd_east  = east[1];
    float const fwd_north = north[1];

    if (!isfinite(fwd_east) || !isfinite(fwd_north))
    {
        return false;
    }

    float h = atan2f(fwd_east, fwd_north) * IMU_RAD_TO_DEG;

    if (h < 0.0f)
    {
        h += 360.0f;
    }

    if (h >= 360.0f)
    {
        h -= 360.0f;
    }

    *heading_deg = h;

    return true;
}

/* ---- Event queue ------------------------------------------------------- */

static void
queue_remove_at (uint8_t offset)
{
    for (uint8_t i = offset; (uint8_t) (i + 1u) < g.q_count; i++)
    {
        uint8_t const dst = (uint8_t) ((g.q_head + i) % IMU_CFG_EVENT_QUEUE_LEN);
        uint8_t const src =
            (uint8_t) ((g.q_head + i + 1u) % IMU_CFG_EVENT_QUEUE_LEN);

        g.queue[dst] = g.queue[src];
    }

    g.q_count--;
}

/**
 * Queue one event.
 *
 * When the queue is full a non-fault event is dropped and counted. A fault
 * event evicts the newest non-fault event instead, because the latched fault
 * must reach the consumer; the snapshot still carries it either way.
 */
static void
post_event (uint8_t id, uint8_t detail, int16_t value)
{
    g.diag.events_posted++;

    if (g.q_count >= IMU_CFG_EVENT_QUEUE_LEN)
    {
        if (IMU_EVT_SENSOR_FAULT != id)
        {
            g.diag.events_dropped++;
            return;
        }

        int32_t victim = -1;

        for (int32_t i = (int32_t) g.q_count - 1; i >= 0; i--)
        {
            uint8_t const at = (uint8_t)
                ((g.q_head + (uint8_t) i) % IMU_CFG_EVENT_QUEUE_LEN);

            if (IMU_EVT_SENSOR_FAULT != g.queue[at].id)
            {
                victim = i;
                break;
            }
        }

        if (victim < 0)
        {
            g.diag.events_dropped++;
            return;
        }

        queue_remove_at((uint8_t) victim);
    }

    uint8_t const at = (uint8_t) ((g.q_head + g.q_count) % IMU_CFG_EVENT_QUEUE_LEN);

    g.queue[at].id     = id;
    g.queue[at].seq    = g.sequence;
    g.queue[at].detail = detail;
    g.queue[at].value  = value;
    g.q_count++;

    if (g.q_count > g.diag.queue_high_water)
    {
        g.diag.queue_high_water = g.q_count;
    }
}

/* ---- State transitions ------------------------------------------------- */

static void
set_terrain (imu_terrain_t terrain)
{
    g.terrain        = terrain;
    g.hump_enter_cnt = 0u;
    g.hump_exit_cnt  = 0u;
    g.lead           = HUMP_LEAD_NONE;
    g.descent_seen   = false;
    g.descent_cnt    = 0u;
}

static void
enter_fault (uint8_t detail, uint32_t now_ms)
{
    if (IMU_TERRAIN_SENSOR_FAULT != g.terrain)
    {
        g.terrain = IMU_TERRAIN_SENSOR_FAULT;
        post_event(IMU_EVT_SENSOR_FAULT, detail, 0);
    }

    g.phase          = IMU_PHASE_FAULT;
    g.retry_at_ms    = now_ms + IMU_CFG_FAULT_RETRY_MS;
    g.accel_valid    = false;
    g.mag_valid      = false;
    g.calibrated     = false;
    g.filt_init      = false;
    g.heading_valid  = false;
    g.last_ms_valid  = false;
    g.hump_mm        = 0.0f;
    g.hump_peak_mm   = 0.0f;
    g.hump_enter_cnt = 0u;
    g.hump_exit_cnt  = 0u;
    g.collision_latched     = false;
    g.collision_cnt         = 0u;
    g.collision_release_cnt = 0u;
}

static void
begin_selftest (void)
{
    g.phase            = IMU_PHASE_SELFTEST;
    g.terrain          = IMU_TERRAIN_INIT;
    g.needs_configure  = true;
    g.selftest_wait    = (uint8_t) IMU_CFG_SELFTEST_WAIT_CYCLES;
    g.selftest_cycles  = 0u;
    g.selftest_passed  = false;
    g.calibrated       = false;
    g.gravity_gain     = 1.0f;      /* re-measure after every recovery */
    g.filt_init        = false;
    g.terrain_pause    = false;
    g.terrain_resume_reset = false;
    g.heading_valid    = false;
    g.last_ms_valid    = false;
    g.heading_prev_deg = 0.0f;
    g.hump_mm          = 0.0f;
    g.hump_peak_mm     = 0.0f;
    g.hump_enter_cnt   = 0u;
    g.hump_exit_cnt    = 0u;
}

/* ---- Calibration ------------------------------------------------------- */

static void
calibration_reset (uint32_t now_ms)
{
    g.cal_start_ms  = now_ms;
    g.cal_count     = 0u;
    g.cal_mag_mean  = 0.0f;
    g.cal_mag_m2    = 0.0f;
    g.cal_min_mag   = 0.0f;
    g.cal_max_mag   = 0.0f;

    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        g.cal_mean[a] = 0.0f;
    }
}

/**
 * Accumulate the stationary window, then accept or reject it.
 *
 * The window is judged in *corrected* units: the mean magnitude gives a scalar
 * gravity gain of 1 g / mean, and the corrected band and standard deviation are
 * then required to look like a stationary robot at 1 g. A sensor whose full
 * scale disagrees with the configured one is corrected rather than rejected,
 * but only inside IMU_CFG_CAL_GAIN_MIN..MAX, so a dead, stuck or clipped sensor
 * still fails.
 */
static void
calibration_update (uint32_t now_ms, const float accel_g[IMU_AXIS_COUNT])
{
    float const magnitude = norm3(accel_g);

    g.cal_count++;

    float const count = (float) g.cal_count;

    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        g.cal_mean[a] += (accel_g[a] - g.cal_mean[a]) / count;
    }

    /* Welford running variance of the magnitude. */
    float const delta = magnitude - g.cal_mag_mean;

    g.cal_mag_mean += delta / count;
    g.cal_mag_m2   += delta * (magnitude - g.cal_mag_mean);

    /* Raw span, judged once the gain is known. */
    if (1u == g.cal_count)
    {
        g.cal_min_mag = magnitude;
        g.cal_max_mag = magnitude;
    }
    else
    {
        if (magnitude < g.cal_min_mag)
        {
            g.cal_min_mag = magnitude;
        }

        if (magnitude > g.cal_max_mag)
        {
            g.cal_max_mag = magnitude;
        }
    }

    if ((now_ms - g.cal_start_ms) < IMU_CFG_CAL_WINDOW_MS)
    {
        return;
    }

    if (g.cal_count < 2u)
    {
        g.diag.calibration_rejects++;
        calibration_reset(now_ms);
        return;
    }

    float const std_dev = sqrtf(g.cal_mag_m2 / (count - 1.0f));
    float const gain = (g.cal_mag_mean > 0.0f) ? (1.0f / g.cal_mag_mean) : 0.0f;

    bool const gain_ok = (gain >= IMU_CFG_CAL_GAIN_MIN) &&
                         (gain <= IMU_CFG_CAL_GAIN_MAX);
    bool const std_ok  = ((std_dev * gain) <= IMU_CFG_CAL_MAX_STD_G);
    bool const band_ok = ((g.cal_min_mag * gain) >= IMU_CFG_CAL_MIN_G) &&
                         ((g.cal_max_mag * gain) <= IMU_CFG_CAL_MAX_G);

    if (gain_ok && std_ok && band_ok)
    {
        g.gravity_gain = gain;

        /* The bias is the corrected stationary mean minus the expected
         * gravity, so a mounting tilt is absorbed here as well. */
        for (int a = 0; a < IMU_AXIS_COUNT; a++)
        {
            g.accel_bias_g[a] = (g.cal_mean[a] * gain) -
                                (EXPECTED_GRAVITY_MG[a] / IMU_CFG_MG_PER_G);
        }

        g.calibrated = true;
        g.phase      = IMU_PHASE_RUNNING;
        g.filt_init  = false;       /* restart the filter on corrected data */
        set_terrain(IMU_TERRAIN_LEVEL);

        post_event(IMU_EVT_CALIBRATED, 0u,
                   (int16_t) lroundf(clampf(g.cal_mag_mean * gain *
                                            IMU_CFG_MG_PER_G, 0.0f,
                                            (float) EVENT_VALUE_MAX)));
        return;
    }

    g.diag.calibration_rejects++;
    calibration_reset(now_ms);
}

/* ---- Terrain ----------------------------------------------------------- */

static void
integrate_hump (float distance_delta_mm)
{
    float const limit = (float) IMU_CFG_HUMP_MAX_MM;

    g.hump_mm += distance_delta_mm * sinf(g.pitch_deg * IMU_DEG_TO_RAD);
    g.hump_mm = clampf(g.hump_mm, -limit, limit);

    if (g.hump_mm > g.hump_peak_mm)
    {
        g.hump_peak_mm = g.hump_mm;
    }
}

/** Begin one hump episode: exactly one HUMP_START for the whole episode. */
static void
start_episode (void)
{
    set_terrain(IMU_TERRAIN_ON_HUMP);   /* also clears the private phase */

    g.hump_mm      = 0.0f;
    g.hump_peak_mm = 0.0f;

    if (g.pitch_deg >= IMU_CFG_HUMP_LEAD_DEG)
    {
        g.lead = HUMP_LEAD_UP;
    }
    else if (g.pitch_deg <= -IMU_CFG_HUMP_LEAD_DEG)
    {
        g.lead = HUMP_LEAD_DOWN;
    }
    else
    {
        g.lead = HUMP_LEAD_NONE;
    }

    /* Only a pitch-led ascent has to earn its exit with a descent; a pose-led
     * or nose-down start may close on the level dwell, so it cannot stick. */
    g.descent_seen = (HUMP_LEAD_UP != g.lead);

    post_event(IMU_EVT_HUMP_START, 0u, 0);
}

/** Post exactly one HUMP_END and keep the peak only if it reached the floor. */
static void
end_episode (void)
{
    float const peak = clampf(g.hump_peak_mm, 0.0f, (float) IMU_CFG_HUMP_MAX_MM);
    uint16_t const peak_mm = (uint16_t) lroundf(peak);
    uint8_t detail = IMU_EVT_DETAIL_REJECTED;

    if (peak_mm >= IMU_CFG_HUMP_MIN_MM)
    {
        detail = IMU_EVT_DETAIL_RETAINED;

        if (peak_mm > g.highest_peak_mm)
        {
            g.highest_peak_mm = peak_mm;
        }
    }

    int32_t const event_value =
        (peak_mm > (uint16_t) EVENT_VALUE_MAX) ? EVENT_VALUE_MAX : (int32_t) peak_mm;

    post_event(IMU_EVT_HUMP_END, detail, (int16_t) event_value);

    g.hump_mm      = 0.0f;
    g.hump_peak_mm = 0.0f;
    set_terrain(IMU_TERRAIN_LEVEL);
}

/**
 * Classify the terrain from the resultant tilt.
 *
 * One ON_HUMP state replaced the directional CLIMBING/CREST/DESCENDING trio: an
 * episode starts from LEVEL after three samples at or above the enter
 * threshold, survives the crest and the descent without any state change, and
 * ends after three samples at or below the exit threshold. The gap between the
 * thresholds is the hysteresis that stops a chassis parked near the boundary
 * from chattering.
 *
 * Tilt decides that a hump is present. Pitch alone still decides how much
 * height the robot climbs, so a roll-only pose can start an episode without
 * manufacturing height.
 */
static void
terrain_update (float distance_delta_mm)
{
    bool const above = (g.tilt_deg >= IMU_CFG_TILT_ENTER_DEG);
    bool const below = (fabsf(g.tilt_deg) <= IMU_CFG_TILT_EXIT_DEG);

    if (above)
    {
        if (g.hump_enter_cnt < COUNT_CAP)
        {
            g.hump_enter_cnt++;
        }
    }
    else
    {
        g.hump_enter_cnt = 0u;
    }

    if (below)
    {
        if (g.hump_exit_cnt < COUNT_CAP)
        {
            g.hump_exit_cnt++;
        }
    }
    else
    {
        g.hump_exit_cnt = 0u;
    }

    if (IMU_TERRAIN_LEVEL == g.terrain)
    {
        if (g.hump_enter_cnt >= IMU_CFG_TERRAIN_SAMPLES)
        {
            start_episode();
        }
        else
        {
            /* No episode yet, so there is no height to hold. */
            g.hump_mm      = 0.0f;
            g.hump_peak_mm = 0.0f;
        }

        return;
    }

    if (IMU_TERRAIN_ON_HUMP != g.terrain)
    {
        return;
    }

    /* Forward-only integration, and pitch only. */
    integrate_hump(distance_delta_mm);

    /* A qualified descent is clearly nose-down for the dwell. */
    if (g.pitch_deg <= -IMU_CFG_TILT_ENTER_DEG)
    {
        if (g.descent_cnt < COUNT_CAP)
        {
            g.descent_cnt++;
        }

        if (g.descent_cnt >= IMU_CFG_TERRAIN_SAMPLES)
        {
            g.descent_seen = true;
        }
    }
    else
    {
        g.descent_cnt = 0u;
    }

    /* A pitch-led ascent holds ON_HUMP across a broad, flat crest until the
     * descent has actually been seen, so one hump stays one episode. */
    if ((g.hump_exit_cnt >= IMU_CFG_TERRAIN_SAMPLES) && g.descent_seen)
    {
        end_episode();
    }
}

/* ---- Collision --------------------------------------------------------- */

static void
collision_update (float accel_magnitude_g)
{
    float const residual = fabsf(accel_magnitude_g - 1.0f);

    g.diag.collision_residual_g = residual;

    if (!g.collision_latched)
    {
        if (residual >= IMU_CFG_COLLISION_RESIDUAL_G)
        {
            if (g.collision_cnt < 255u)
            {
                g.collision_cnt++;
            }

            if (g.collision_cnt >= IMU_CFG_COLLISION_SAMPLES)
            {
                g.collision_latched     = true;
                g.collision_release_cnt = 0u;

                int32_t const value = (int32_t) lroundf(residual * IMU_CFG_MG_PER_G);

                post_event(IMU_EVT_COLLISION, 0u,
                           (int16_t) ((value > EVENT_VALUE_MAX) ? EVENT_VALUE_MAX
                                                                : value));
            }
        }
        else
        {
            g.collision_cnt = 0u;
        }

        return;
    }

    if (residual < IMU_CFG_COLLISION_RELEASE_G)
    {
        if (g.collision_release_cnt < 255u)
        {
            g.collision_release_cnt++;
        }

        if (g.collision_release_cnt >= IMU_CFG_COLLISION_RELEASE_SAMPLES)
        {
            g.collision_latched     = false;
            g.collision_cnt         = 0u;
            g.collision_release_cnt = 0u;
        }
    }
    else
    {
        g.collision_release_cnt = 0u;
    }
}

/* ---- Diagnostics ------------------------------------------------------- */

/**
 * Encoder cross-check, qualitative only.
 *
 * Heading increases clockwise, so a right turn means the left wheel travelled
 * further. Agreement requires both signs to match. A disagreement is recorded
 * but never raises an IMU fault: track width and counts-to-angle are not
 * calibrated yet.
 */
static imu_crosscheck_t
turn_crosscheck (void)
{
    if (fabsf(g.turn_rate_dps) < IMU_CFG_CROSSCHECK_MIN_DPS)
    {
        return IMU_CROSSCHECK_UNKNOWN;
    }

    int32_t const diff = ((int32_t) g.diag.encoder_left_delta) -
                         ((int32_t) g.diag.encoder_right_delta);

    if (0 == diff)
    {
        return IMU_CROSSCHECK_UNKNOWN;
    }

    bool const imu_right = (g.turn_rate_dps > 0.0f);
    bool const enc_right = (diff > 0);

    return (imu_right == enc_right) ? IMU_CROSSCHECK_AGREE
                                    : IMU_CROSSCHECK_DISAGREE;
}

static void
update_diag (const float * accel_corrected, const float * mag_gauss)
{
    g.diag.terrain      = g.terrain;
    g.diag.pitch_deg    = g.pitch_deg;
    g.diag.roll_deg     = g.roll_deg;
    g.diag.tilt_deg     = g.tilt_deg;
    g.diag.heading_deg  = g.heading_deg;
    g.diag.turn_rate_dps = g.turn_rate_dps;
    g.diag.hump_mm      = g.hump_mm;
    g.diag.hump_peak_mm = (int16_t) lroundf(clampf(g.hump_peak_mm, 0.0f,
                                                   (float) EVENT_VALUE_MAX));
    g.diag.highest_hump_mm = g.highest_peak_mm;
    g.diag.turn_crosscheck = turn_crosscheck();
    g.diag.sequence     = g.sequence;
    g.diag.run_distance_mm = g.run_distance_mm;
    g.diag.gravity_gain = g.gravity_gain;

    if (NULL != accel_corrected)
    {
        /* accel_raw_g and accel_counts are recorded before the gravity gain is
         * applied, so the diagnostic always shows the physical reading. */
        for (int a = 0; a < IMU_AXIS_COUNT; a++)
        {
            g.diag.accel_filt_g[a] = g.filt_init ? g.accel_filt_g[a]
                                                 : accel_corrected[a];
            g.diag.accel_bias_g[a] = g.accel_bias_g[a];
        }
    }

    if (NULL != mag_gauss)
    {
        for (int a = 0; a < IMU_AXIS_COUNT; a++)
        {
            g.diag.mag_raw_gauss[a]  = mag_gauss[a];
            g.diag.mag_filt_gauss[a] = g.filt_init ? g.mag_filt_gauss[a]
                                                   : mag_gauss[a];
        }
    }
}

/* ---- Publish ----------------------------------------------------------- */

static void
publish (void)
{
    imu_snapshot_fields_t fields;

    memset(&fields, 0, sizeof(fields));

    if (g.heading_valid && isfinite(g.heading_deg))
    {
        fields.heading_ddeg = (uint16_t) lroundf(clampf(g.heading_deg,
                                                        0.0f, 359.9f) * 10.0f);

        if (fields.heading_ddeg > 3599u)
        {
            fields.heading_ddeg = 3599u;
        }
    }

    /* The pack helper clamps pitch and turn rate to the 10-bit field range. */
    fields.pitch_ddeg     = (int16_t) lroundf(clampf(g.pitch_deg,
                                                     -1000.0f, 1000.0f) * 10.0f);
    fields.turn_rate_dps  = (int16_t) lroundf(clampf(g.turn_rate_dps,
                                                     -1000.0f, 1000.0f));
    fields.highest_hump_mm = g.highest_peak_mm;
    fields.terrain         = g.terrain;
    fields.accel_valid     = g.accel_valid;
    fields.mag_valid       = g.mag_valid;
    fields.calibrated      = g.calibrated;
    fields.selftest_passed = g.selftest_passed;
    fields.fault_latched   = (IMU_TERRAIN_SENSOR_FAULT == g.terrain);

    g.sequence++;
    fields.sequence = g.sequence;

    /* Seqlock: odd while the pair is being written, even once it is stable.
     * The words are volatile, so the stores stay in this order. */
    imu_snapshot_t packed;

    imu_snapshot_pack(&fields, &packed);

    g.snapshot_seq = g.snapshot_seq + 1u;
    g.snapshot_word[0] = packed.word[0];
    g.snapshot_word[1] = packed.word[1];
    g.snapshot_seq = g.snapshot_seq + 1u;

    g.diag.sequence = g.sequence;
}

/* ---- One cycle --------------------------------------------------------- */

static void
consume_encoder_input (float * distance_delta_mm)
{
    *distance_delta_mm = 0.0f;

    if (!g.enc_pending)
    {
        g.diag.missed_distance++;
        return;
    }

    *distance_delta_mm = (float) g.enc.distance_delta_mm;

    g.diag.distance_delta_mm  = g.enc.distance_delta_mm;
    g.diag.encoder_left_delta = g.enc.encoder_left_delta;
    g.diag.encoder_right_delta = g.enc.encoder_right_delta;

    int32_t const total = ((int32_t) g.run_distance_mm) +
                          ((int32_t) g.enc.distance_delta_mm);

    g.run_distance_mm = (uint32_t) ((total < 0) ? 0 : total);
    g.enc_pending = false;
}

/**
 * Read the accelerometer configuration register back once a second.
 *
 * The start-up path already verifies the write, but a live readback lets the
 * bench prove which full scale the physical board is running, independently of
 * anything this firmware assumes.
 */
static void
refresh_config_readback (uint32_t now_ms)
{
    if ((now_ms - g.last_reg_check_ms) < IMU_CFG_REG_CHECK_PERIOD_MS)
    {
        return;
    }

    g.last_reg_check_ms = now_ms;

    uint8_t value = 0u;

    if (IMU_LSM303_OK == imu_lsm303_read_ctrl4(&value))
    {
        g.diag.accel_ctrl4       = value;
        g.diag.accel_ctrl4_valid = true;
    }
    else
    {
        g.diag.accel_ctrl4_valid = false;
    }
}

static void
run_cycle (uint32_t now_ms)
{
    g.diag.cycles++;
    g.diag.drdy_seen = imu_hal_drdy_asserted();
    if (!g.diag.drdy_seen)
    {
        g.diag.missed_drdy++;
    }

    float distance_delta_mm = 0.0f;

    consume_encoder_input(&distance_delta_mm);

    /* Retry the bus while faulted. Nothing else runs in this state. */
    if (IMU_PHASE_FAULT == g.phase)
    {
        if ((int32_t) (now_ms - g.retry_at_ms) >= 0)
        {
            uint8_t detail = 0u;

            g.retry_at_ms = now_ms + IMU_CFG_FAULT_RETRY_MS;

            if (IMU_LSM303_OK == imu_lsm303_configure(&detail))
            {
                /* Recovered: repeat the self-test and the stationary
                 * calibration before returning to LEVEL. */
                g.diag.recoveries++;
                begin_selftest();
                g.needs_configure = false;
            }
        }

        update_diag(NULL, NULL);
        publish();
        return;
    }

    refresh_config_readback(now_ms);

    if (g.needs_configure)
    {
        uint8_t detail = 0u;
        imu_lsm303_status_t const status = imu_lsm303_configure(&detail);

        if (IMU_LSM303_OK != status)
        {
            if (IMU_LSM303_ERR_BUS == status)
            {
                g.diag.i2c_errors++;
            }

            /* Report the self-test result before the fault state latches. */
            post_event(IMU_EVT_SELFTEST,
                       (0u != detail) ? detail : IMU_LSM303_ST_ACCEL_ID, 0);
            enter_fault((0u != detail) ? detail : FAULT_DETAIL_BUS, now_ms);
            update_diag(NULL, NULL);
            publish();
            return;
        }

        g.needs_configure = false;
        g.selftest_wait   = (uint8_t) IMU_CFG_SELFTEST_WAIT_CYCLES;
        g.selftest_cycles = 0u;
    }

    imu_sample_t sample;

    if (IMU_LSM303_OK != imu_lsm303_read(&sample))
    {
        g.diag.i2c_errors++;
        enter_fault(FAULT_DETAIL_BUS, now_ms);
        update_diag(NULL, NULL);
        publish();
        return;
    }

    float accel_sensor[IMU_AXIS_COUNT];
    float mag_sensor[IMU_AXIS_COUNT];
    float accel_g[IMU_AXIS_COUNT];
    float mag_gauss[IMU_AXIS_COUNT];

    imu_lsm303_accel_to_g(sample.accel_raw, accel_sensor);
    imu_lsm303_mag_to_gauss(sample.mag_raw, mag_sensor);
    remap3(&ACCEL_MAP, accel_sensor, accel_g);
    remap3(&MAG_MAP, mag_sensor, mag_gauss);

    /* Record the physical reading first, then apply the scalar gravity gain.
     * Everything below - validity, bias, pitch, heading, the collision
     * residual and the terrain logic - works in calibrated physical g. */
    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        g.diag.accel_raw_g[a]     = accel_g[a];
        g.diag.accel_counts[a]    = sample.accel_raw[a];
        g.diag.accel_counts_mg[a] = ((float) sample.accel_raw[a]) *
                                    IMU_CFG_ACCEL_MG_PER_LSB;
        accel_g[a] *= g.gravity_gain;
    }

    float const accel_norm = norm3(accel_g);
    float const mag_norm   = norm3(mag_gauss);

    bool const valid = finite3(accel_g) && finite3(mag_gauss) &&
                       (accel_norm >= IMU_CFG_ACCEL_MIN_G) &&
                       (accel_norm <= IMU_CFG_ACCEL_MAX_G) &&
                       (mag_norm >= IMU_CFG_MAG_MIN_GAUSS) &&
                       (mag_norm <= IMU_CFG_MAG_MAX_GAUSS);

    if (!valid)
    {
        g.diag.invalid_samples++;
        g.accel_valid = false;
        g.mag_valid   = false;
        enter_fault(FAULT_DETAIL_INVALID, now_ms);
        update_diag(accel_g, mag_gauss);
        publish();
        return;
    }

    g.accel_valid = true;
    g.mag_valid   = true;

    /* Self-test stage: identity and readback already passed, so this only
     * waits for one plausible live sample. */
    if (IMU_PHASE_SELFTEST == g.phase)
    {
        g.selftest_cycles++;

        if (g.selftest_wait > 0u)
        {
            g.selftest_wait--;
        }

        if (0u == g.selftest_wait)
        {
            g.selftest_passed = true;
            post_event(IMU_EVT_SELFTEST, 0u,
                       (int16_t) lroundf(clampf(accel_norm * IMU_CFG_MG_PER_G,
                                                0.0f, (float) EVENT_VALUE_MAX)));
            g.phase = IMU_PHASE_CALIBRATING;
            calibration_reset(now_ms);
        }
        else if (g.selftest_cycles > IMU_CFG_SELFTEST_MAX_CYCLES)
        {
            g.selftest_passed = false;
            g.diag.self_test_failures++;
            post_event(IMU_EVT_SELFTEST, IMU_LSM303_ST_DATA, 0);
            enter_fault(IMU_LSM303_ST_DATA, now_ms);
        }

        set_attitude(accel_g);
        update_diag(accel_g, mag_gauss);
        publish();
        return;
    }

    if (IMU_PHASE_CALIBRATING == g.phase)
    {
        set_attitude(accel_g);
        calibration_update(now_ms, accel_g);
        update_diag(accel_g, mag_gauss);
        publish();
        return;
    }

    /* ---- RUNNING: correct, filter, derive, classify ------------------- */

    float accel_corrected[IMU_AXIS_COUNT];

    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        accel_corrected[a] = accel_g[a] - g.accel_bias_g[a];
    }

    if (!g.filt_init)
    {
        for (int a = 0; a < IMU_AXIS_COUNT; a++)
        {
            g.accel_filt_g[a]    = accel_corrected[a];
            g.mag_filt_gauss[a]  = mag_gauss[a];
        }

        g.filt_init = true;
    }
    else
    {
        low_pass3(IMU_CFG_LPF_ALPHA, g.accel_filt_g, accel_corrected);
        low_pass3(IMU_CFG_LPF_ALPHA, g.mag_filt_gauss, mag_gauss);
    }

    set_attitude(g.accel_filt_g);

    float heading = 0.0f;

    if (!heading_from_vectors(g.accel_filt_g, g.mag_filt_gauss, &heading))
    {
        g.diag.invalid_samples++;
        g.accel_valid = false;
        g.mag_valid   = false;
        enter_fault(FAULT_DETAIL_INVALID, now_ms);
        update_diag(accel_g, mag_gauss);
        publish();
        return;
    }

    g.heading_deg   = heading;
    g.heading_valid = true;

    uint32_t interval_ms = IMU_CFG_PERIOD_MS;

    if (g.last_ms_valid)
    {
        uint32_t const dt = now_ms - g.last_ms;

        if ((dt > 0u) && (dt <= 1000u))
        {
            interval_ms = dt;
        }
    }

    if (g.heading_prev_valid && (interval_ms > 0u))
    {
        float const delta = wrap180(heading - g.heading_prev_deg);

        g.turn_rate_dps = (delta * 1000.0f) / (float) interval_ms;
    }
    else
    {
        g.turn_rate_dps = 0.0f;
    }

    g.heading_prev_deg = heading;
    g.heading_prev_valid = true;
    g.last_ms       = now_ms;
    g.last_ms_valid = true;

    if (g.terrain_pause)
    {
        /* Reverse recovery freezes the classifier: state, dwell counters,
         * current and peak height, and hump events all hold. Sensor sampling,
         * health checks, collision and the diagnostics keep running. */
        g.terrain_resume_reset = true;
    }
    else
    {
        if (g.terrain_resume_reset)
        {
            /* Resume: forward samples must re-qualify before any transition is
             * accepted, so a reverse cannot leave a half-counted dwell behind. */
            g.hump_enter_cnt = 0u;
            g.hump_exit_cnt  = 0u;
            g.terrain_resume_reset = false;
        }

        terrain_update(distance_delta_mm);
    }

    collision_update(norm3(g.accel_filt_g));

    update_diag(accel_g, mag_gauss);
    publish();
}

/* ---- Public API -------------------------------------------------------- */

void
imu_init (uint32_t now_ms)
{
    memset(&g, 0, sizeof(g));

    g.phase           = IMU_PHASE_SELFTEST;
    g.terrain         = IMU_TERRAIN_INIT;
    g.needs_configure = true;
    g.selftest_wait   = (uint8_t) IMU_CFG_SELFTEST_WAIT_CYCLES;
    g.last_ms         = now_ms;
    g.cal_start_ms    = now_ms;
    g.gravity_gain    = 1.0f;       /* unity until a window accepts a value */
    g.diag.terrain    = IMU_TERRAIN_INIT;
    g.snapshot_word[0] = 0u;
    g.snapshot_word[1] = 0u;
    g.snapshot_seq     = 0u;
}

void
imu_step (uint32_t now_ms)
{
    run_cycle(now_ms);
}

void
imu_publish_encoder_input (const imu_encoder_input_t * input)
{
    if (NULL == input)
    {
        return;
    }

    g.enc         = *input;
    g.enc_pending = true;

    /* The pause travels with the same hand-off as the distance, so a recovery
     * can never freeze one and forget the other. */
    g.terrain_pause = input->terrain_pause;
}

void
imu_begin_run (void)
{
    g.highest_peak_mm      = 0u;
    g.run_distance_mm      = 0u;
    g.diag.highest_hump_mm = 0u;
    g.diag.run_distance_mm = 0u;
}

bool
imu_snapshot_read (imu_snapshot_t * out)
{
    if (NULL == out)
    {
        return false;
    }

    /* Two-word seqlock. Read the counter, copy both words, read the counter
     * again; a change means a publish landed in between, so retry. The caller
     * keeps its previous copy when this returns false. */
    for (int attempt = 0; attempt < SNAPSHOT_READ_ATTEMPTS; attempt++)
    {
        uint32_t const seq_before = g.snapshot_seq;

        if (0u != (seq_before & 1u))
        {
            continue;                       /* publish in progress */
        }

        uint32_t const w0 = g.snapshot_word[0];

        IMU_SNAPSHOT_READ_HOOK();

        uint32_t const w1 = g.snapshot_word[1];

        if (g.snapshot_seq == seq_before)
        {
            out->word[0] = w0;
            out->word[1] = w1;

            return true;
        }
    }

    g.diag.snapshot_failures++;

    return false;
}

bool
imu_event_pop (imu_event_t * out)
{
    if ((NULL == out) || (0u == g.q_count))
    {
        return false;
    }

    *out = g.queue[g.q_head];

    g.q_head = (uint8_t) ((g.q_head + 1u) % IMU_CFG_EVENT_QUEUE_LEN);
    g.q_count--;
    g.diag.events_popped++;

    return true;
}

void
imu_diag_read (imu_diag_t * out)
{
    if (NULL == out)
    {
        return;
    }

    *out = g.diag;
}

const char *
imu_terrain_name (imu_terrain_t terrain)
{
    switch (terrain)
    {
        case IMU_TERRAIN_INIT:          return "INIT";
        case IMU_TERRAIN_LEVEL:         return "LEVEL";
        case IMU_TERRAIN_ON_HUMP:       return "ON_HUMP";
        case IMU_TERRAIN_SENSOR_FAULT:  return "SENSOR_FAULT";
        default:                        return "?";
    }
}

const char *
imu_event_name (uint8_t id)
{
    switch (id)
    {
        case IMU_EVT_SELFTEST:      return "SELFTEST";
        case IMU_EVT_CALIBRATED:    return "CALIBRATED";
        case IMU_EVT_HUMP_START:    return "HUMP_START";
        case IMU_EVT_HUMP_END:      return "HUMP_END";
        case IMU_EVT_COLLISION:     return "COLLISION";
        case IMU_EVT_SENSOR_FAULT:  return "SENSOR_FAULT";
        default:                    return "?";
    }
}
