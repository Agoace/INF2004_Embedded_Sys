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
    uint32_t cal_start_ms;
    uint32_t cal_count;
    float    cal_mean[IMU_AXIS_COUNT];
    float    cal_mag_mean;
    float    cal_mag_m2;
    bool     cal_in_range;

    /* Low-pass filter state. */
    bool  filt_init;
    float accel_filt_g[IMU_AXIS_COUNT];
    float mag_filt_gauss[IMU_AXIS_COUNT];

    /* Derived motion. */
    float    pitch_deg;
    float    heading_deg;
    float    turn_rate_dps;
    bool     heading_valid;
    float    heading_prev_deg;
    bool     heading_prev_valid;
    uint32_t last_ms;
    bool     last_ms_valid;

    /* Terrain and hump. */
    uint16_t level_cnt;
    uint16_t climb_cnt;
    uint16_t descend_cnt;
    float    hump_mm;
    float    hump_peak_mm;
    uint16_t highest_peak_mm;

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
    g.terrain   = terrain;
    g.level_cnt = 0u;
    g.climb_cnt = 0u;
    g.descend_cnt = 0u;
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
    g.level_cnt      = 0u;
    g.climb_cnt      = 0u;
    g.descend_cnt    = 0u;
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
    g.filt_init        = false;
    g.heading_valid    = false;
    g.last_ms_valid    = false;
    g.heading_prev_deg = 0.0f;
    g.hump_mm          = 0.0f;
    g.hump_peak_mm     = 0.0f;
    g.level_cnt        = 0u;
    g.climb_cnt        = 0u;
    g.descend_cnt      = 0u;
}

/* ---- Calibration ------------------------------------------------------- */

static void
calibration_reset (uint32_t now_ms)
{
    g.cal_start_ms  = now_ms;
    g.cal_count     = 0u;
    g.cal_mag_mean  = 0.0f;
    g.cal_mag_m2    = 0.0f;
    g.cal_in_range  = true;

    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        g.cal_mean[a] = 0.0f;
    }
}

/**
 * Accumulate the stationary window and decide once it is long enough.
 *
 * The window is accepted only when every sample magnitude stayed inside the
 * configured band and the sample standard deviation of the magnitude is small
 * enough, so a robot that is being carried is never mistaken for a calibrated
 * one.
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

    if ((magnitude < IMU_CFG_CAL_MIN_G) || (magnitude > IMU_CFG_CAL_MAX_G))
    {
        g.cal_in_range = false;
    }

    if ((now_ms - g.cal_start_ms) < IMU_CFG_CAL_WINDOW_MS)
    {
        return;
    }

    float std_dev = 0.0f;

    if (g.cal_count > 1u)
    {
        std_dev = sqrtf(g.cal_mag_m2 / (count - 1.0f));
    }

    if (g.cal_in_range && (g.cal_count > 1u) &&
        (std_dev <= IMU_CFG_CAL_MAX_STD_G))
    {
        for (int a = 0; a < IMU_AXIS_COUNT; a++)
        {
            g.accel_bias_g[a] = g.cal_mean[a] -
                                (EXPECTED_GRAVITY_MG[a] / IMU_CFG_MG_PER_G);
        }

        g.calibrated = true;
        g.phase      = IMU_PHASE_RUNNING;
        g.filt_init  = false;       /* restart the filter on corrected data */
        set_terrain(IMU_TERRAIN_LEVEL);

        post_event(IMU_EVT_CALIBRATED, 0u,
                   (int16_t) lroundf(clampf(g.cal_mag_mean * IMU_CFG_MG_PER_G,
                                            0.0f, (float) EVENT_VALUE_MAX)));
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

static void
start_episode (imu_terrain_t terrain)
{
    g.hump_mm      = 0.0f;
    g.hump_peak_mm = 0.0f;
    set_terrain(terrain);
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

static void
terrain_update (float distance_delta_mm)
{
    float const pitch = g.pitch_deg;

    if (fabsf(pitch) <= IMU_CFG_LEVEL_DEG)
    {
        if (g.level_cnt < COUNT_CAP)
        {
            g.level_cnt++;
        }
    }
    else
    {
        g.level_cnt = 0u;
    }

    if (pitch >= IMU_CFG_CLIMB_DEG)
    {
        if (g.climb_cnt < COUNT_CAP)
        {
            g.climb_cnt++;
        }
    }
    else
    {
        g.climb_cnt = 0u;
    }

    if (pitch <= IMU_CFG_DESCEND_DEG)
    {
        if (g.descend_cnt < COUNT_CAP)
        {
            g.descend_cnt++;
        }
    }
    else
    {
        g.descend_cnt = 0u;
    }

    switch (g.terrain)
    {
        case IMU_TERRAIN_LEVEL:
        {
            if (g.climb_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                start_episode(IMU_TERRAIN_CLIMBING);
            }
            else if (g.descend_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                start_episode(IMU_TERRAIN_DESCENDING);
            }
            else
            {
                g.hump_mm      = 0.0f;
                g.hump_peak_mm = 0.0f;
            }

            break;
        }

        case IMU_TERRAIN_CLIMBING:
        {
            integrate_hump(distance_delta_mm);

            if (g.level_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                /* CREST needs its own fresh dwell before the episode ends. */
                set_terrain(IMU_TERRAIN_CREST);
            }
            else if (g.descend_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                set_terrain(IMU_TERRAIN_DESCENDING);
            }

            break;
        }

        case IMU_TERRAIN_CREST:
        {
            if (g.descend_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                set_terrain(IMU_TERRAIN_DESCENDING);
            }
            else if (g.level_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                /* Level without a descent: the hump is over. */
                end_episode();
            }

            break;
        }

        case IMU_TERRAIN_DESCENDING:
        {
            integrate_hump(distance_delta_mm);

            if (g.level_cnt >= IMU_CFG_TERRAIN_SAMPLES)
            {
                end_episode();
            }

            break;
        }

        default:
        {
            break;
        }
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
update_diag (const float * accel_g, const float * mag_gauss)
{
    g.diag.terrain      = g.terrain;
    g.diag.pitch_deg    = g.pitch_deg;
    g.diag.heading_deg  = g.heading_deg;
    g.diag.turn_rate_dps = g.turn_rate_dps;
    g.diag.hump_mm      = g.hump_mm;
    g.diag.hump_peak_mm = (int16_t) lroundf(clampf(g.hump_peak_mm, 0.0f,
                                                   (float) EVENT_VALUE_MAX));
    g.diag.highest_hump_mm = g.highest_peak_mm;
    g.diag.turn_crosscheck = turn_crosscheck();
    g.diag.sequence     = g.sequence;
    g.diag.run_distance_mm = g.run_distance_mm;

    if (NULL != accel_g)
    {
        for (int a = 0; a < IMU_AXIS_COUNT; a++)
        {
            g.diag.accel_raw_g[a]    = accel_g[a];
            g.diag.accel_filt_g[a]   = g.filt_init ? g.accel_filt_g[a] : accel_g[a];
            g.diag.accel_bias_g[a]   = g.accel_bias_g[a];
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

        g.pitch_deg = pitch_from_accel(accel_g);
        update_diag(accel_g, mag_gauss);
        publish();
        return;
    }

    if (IMU_PHASE_CALIBRATING == g.phase)
    {
        g.pitch_deg = pitch_from_accel(accel_g);
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

    g.pitch_deg = pitch_from_accel(g.accel_filt_g);

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

    terrain_update(distance_delta_mm);
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
        case IMU_TERRAIN_CLIMBING:      return "CLIMBING";
        case IMU_TERRAIN_CREST:         return "CREST";
        case IMU_TERRAIN_DESCENDING:    return "DESCENDING";
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
