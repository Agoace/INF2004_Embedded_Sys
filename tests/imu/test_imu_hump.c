/**
 * @file test_imu_hump.c
 * @brief Host tests for the integrated moving height profile (RT-0021).
 *
 * The production IMU core, snapshot codec, LSM303 driver and encoder feed all
 * compile unchanged; only the hardware seam from imu_hal.h is replaced, by a
 * fake LSM303DLHC register file. The feed is driven with the same signed
 * per-wheel count deltas that motion_consume_travel() hands the firmware.
 *
 *   cc -std=c11 -Wall -Wextra -Wshadow -Isrc/imu -Isrc/motion -Isrc/common \
 *      tests/imu/test_imu_hump.c src/imu/imu_core.c src/imu/imu_snapshot.c \
 *      src/imu/imu_lsm303.c src/imu/imu_feed.c -lm -o /tmp/t_hump && /tmp/t_hump
 *
 * Covered (RT-0021): 65 mm / 2535 CPR conversion, fractional carry, signed
 * wheel averaging, flat travel, climb and descend peak with return, stationary
 * zero height, the +3/-3 entry against the 2 degree level band, and reverse
 * recovery isolation.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_config.h"
#include "imu_core.h"
#include "imu_feed.h"
#include "imu_hal.h"
#include "imu_snapshot.h"
#include "imu_types.h"
#include "motion_config.h"

#define TEST_PI 3.14159265358979323846f

/* ---- Fake LSM303DLHC --------------------------------------------------- */

#define ACCEL_ID_REG  0x0Fu
#define ACCEL_DATA    0x28u
#define MAG_ID_REG    0x0Au
#define MAG_DATA      0x03u

static uint8_t g_accel[256];
static uint8_t g_mag[256];
static float   g_scale = 1.0f;      /* artificial board gain error */
static bool    g_bus_ok = true;

static uint8_t *
regs_for (uint8_t addr7)
{
    if (IMU_CFG_ACCEL_ADDR == addr7)
    {
        return g_accel;
    }

    if (IMU_CFG_MAG_ADDR == addr7)
    {
        return g_mag;
    }

    return NULL;
}

bool
imu_hal_bus_init (void)
{
    return true;
}

bool
imu_hal_i2c_write_reg (uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t * const regs = regs_for(addr7);

    if ((!g_bus_ok) || (NULL == regs))
    {
        return false;
    }

    regs[reg & 0x7Fu] = value;

    return true;
}

bool
imu_hal_i2c_read_reg (uint8_t addr7, uint8_t reg, uint8_t * dst, size_t len)
{
    uint8_t * const regs = regs_for(addr7);

    if ((!g_bus_ok) || (NULL == regs) || (NULL == dst))
    {
        return false;
    }

    bool const auto_increment = (0u != (reg & 0x80u));
    uint8_t index = (uint8_t) (reg & 0x7Fu);

    for (size_t i = 0; i < len; i++)
    {
        dst[i] = regs[index];

        if (auto_increment)
        {
            index++;
        }
    }

    return true;
}

uint32_t
imu_hal_now_ms (void)
{
    return 0u;
}

bool
imu_hal_drdy_asserted (void)
{
    return true;
}

static void
put_be16 (uint8_t * regs, uint8_t reg, int16_t value)
{
    uint16_t const bits = (uint16_t) value;

    regs[reg]      = (uint8_t) (bits >> 8);
    regs[reg + 1u] = (uint8_t) (bits & 0xFFu);
}

static void
fake_reset (void)
{
    memset(g_accel, 0, sizeof(g_accel));
    memset(g_mag, 0, sizeof(g_mag));

    g_accel[ACCEL_ID_REG] = 0x33u;
    g_mag[MAG_ID_REG]     = 0x48u;

    g_scale  = 1.0f;
    g_bus_ok = true;
}

/** Accelerometer in milli-g, through the injected board gain error. */
static void
fake_set_accel_mg (int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    int16_t const values[3] = { x_mg, y_mg, z_mg };

    for (int a = 0; a < 3; a++)
    {
        int32_t counts = (int32_t) lroundf((((float) values[a]) * g_scale) /
                                           IMU_CFG_ACCEL_MG_PER_LSB);

        if (counts > 2047)
        {
            counts = 2047;
        }

        if (counts < -2048)
        {
            counts = -2048;
        }

        uint16_t const bits = (uint16_t) ((int16_t) (counts * 16));

        g_accel[ACCEL_DATA + (2 * a)]      = (uint8_t) (bits & 0xFFu);
        g_accel[ACCEL_DATA + (2 * a) + 1u] = (uint8_t) (bits >> 8);
    }
}

/**
 * Static chassis attitude facing magnetic north.
 *
 * Built from the inverse of the reporting formulas, so the angles requested
 * here are the angles the core reports. Roll follows the core's convention: a
 * negative roll is the right side dropping, which is the pose the bench
 * recorded for one wheel on a hump.
 */
static void
fake_set_attitude (float pitch_deg, float roll_deg)
{
    float const p = pitch_deg * (TEST_PI / 180.0f);
    float const r = roll_deg * (TEST_PI / 180.0f);

    float const ax = 1000.0f * sinf(r);
    float const ay = 1000.0f * sinf(p);
    float const az = sqrtf((1000.0f * 1000.0f) - (ax * ax) - (ay * ay));

    fake_set_accel_mg((int16_t) lroundf(ax), (int16_t) lroundf(ay),
                      (int16_t) lroundf(az));
    put_be16(g_mag, MAG_DATA, 0);
    put_be16(g_mag, (uint8_t) (MAG_DATA + 2u), 0);
    put_be16(g_mag, (uint8_t) (MAG_DATA + 4u), 3000);
}

/** Pitch-only attitude, the common case. */
static void
fake_set_pitch (float pitch_deg)
{
    fake_set_attitude(pitch_deg, 0.0f);
}

/* ---- Harness ----------------------------------------------------------- */

static int      g_failures;
static uint32_t g_now;
static imu_feed_t g_feed;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(actual, expect, tol)                                      \
    do                                                                       \
    {                                                                        \
        double const a_ = (double) (actual);                                 \
        double const e_ = (double) (expect);                                 \
        if (fabs(a_ - e_) > (double) (tol))                                  \
        {                                                                    \
            printf("  FAIL %s:%d: %s = %.3f, expected %.3f +/- %.3f\n",      \
                   __FILE__, __LINE__, #actual, a_, e_, (double) (tol));     \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static imu_terrain_t
terrain_now (void)
{
    imu_diag_t diag;

    imu_diag_read(&diag);

    return diag.terrain;
}

static imu_snapshot_fields_t
fields_now (void)
{
    imu_snapshot_t snapshot;
    imu_snapshot_fields_t fields;

    memset(&fields, 0, sizeof(fields));
    (void) imu_snapshot_read(&snapshot);
    imu_snapshot_unpack(&snapshot, &fields);

    return fields;
}

/** One IMU cycle with the given signed per-wheel count deltas fed first. */
static void
feed_step (int16_t d_left, int16_t d_right, bool reverse)
{
    imu_encoder_input_t input;

    (void) imu_feed_step(&g_feed, d_left, d_right, reverse, &input);
    imu_publish_encoder_input(&input);

    g_now += IMU_CFG_PERIOD_MS;
    imu_step(g_now);
}

static void
run_steps (int count, int16_t d_left, int16_t d_right)
{
    for (int i = 0; i < count; i++)
    {
        feed_step(d_left, d_right, false);
    }
}

/** Boot through self-test and the flat stationary calibration. */
static void
boot_level (void)
{
    fake_reset();
    imu_feed_init(&g_feed);
    g_now = 1000u;
    imu_init(g_now);
    fake_set_pitch(0.0f);
    run_steps(280, 0, 0);
}

static bool
run_until_terrain (imu_terrain_t want, int max_steps)
{
    for (int i = 0; i < max_steps; i++)
    {
        if (terrain_now() == want)
        {
            return true;
        }

        feed_step(0, 0, false);
    }

    return (terrain_now() == want);
}

/* ---- Feed conversion --------------------------------------------------- */

static void
test_conversion (void)
{
    puts("feed: 65 mm wheel, 2535 counts per revolution");

    imu_feed_t feed;
    imu_encoder_input_t input;

    imu_feed_init(&feed);
    CHECK(204 == imu_feed_step(&feed, (int16_t) MOTION_COUNTS_PER_REV,
                               (int16_t) MOTION_COUNTS_PER_REV, false, &input));
    CHECK_NEAR(input.distance_delta_mm, 204.0, 0.5);

    /* Half a revolution raises the same numbers to the halfway point. */
    imu_feed_init(&feed);
    CHECK(102 == imu_feed_step(&feed, 1268, 1267, false, &input));

    /* The raw deltas are always reported verbatim. */
    CHECK(input.encoder_left_delta == 1268);
    CHECK(input.encoder_right_delta == 1267);

    /* Nothing moves, nothing is emitted. */
    imu_feed_init(&feed);
    CHECK(0 == imu_feed_step(&feed, 0, 0, false, &input));
    CHECK(0 == input.distance_delta_mm);
}

static void
test_fractional_carry (void)
{
    puts("feed: sub-millimetre remainder survives every cycle");

    imu_feed_t feed;
    imu_encoder_input_t input;

    /* One count on each wheel is 0.081 mm, so a single cycle must emit 0. */
    imu_feed_init(&feed);
    CHECK(0 == imu_feed_step(&feed, 1, 1, false, &input));

    /* Thirteen of them carry past one millimetre. */
    int16_t total = 0;

    for (int i = 0; i < 12; i++)
    {
        total = (int16_t) (total + imu_feed_step(&feed, 1, 1, false, &input));
    }

    printf("  13 x 1 count -> %d mm\n", (int) total);
    CHECK(1 == total);

    /* A long slow run stays faithful: 1000 counts is 80.55 mm. */
    imu_feed_init(&feed);
    int32_t sum = 0;

    for (int i = 0; i < 1000; i++)
    {
        sum += imu_feed_step(&feed, 1, 1, false, &input);
    }

    printf("  1000 x 1 count -> %ld mm (exact 80.55)\n", (long) sum);
    CHECK_NEAR(sum, 80.0, 1.0);
}

static void
test_signed_average (void)
{
    puts("feed: the average is signed");

    imu_feed_t feed;
    imu_encoder_input_t input;

    /* dL=100, dR=-60 averages to 20 counts forward. */
    imu_feed_init(&feed);
    CHECK(1 == imu_feed_step(&feed, 100, -60, false, &input));

    /* Both wheels backwards average -80 counts, which is -6.44 mm. */
    imu_feed_init(&feed);
    CHECK(-6 == imu_feed_step(&feed, -100, -60, false, &input));

    /* Mirror of the forward case, so the sign handling is symmetric. */
    imu_feed_init(&feed);
    CHECK(-1 == imu_feed_step(&feed, -100, 60, false, &input));
    CHECK(-1 == input.distance_delta_mm);

    /* Equal and opposite wheels cancel: a spin in place travels nowhere. */
    imu_feed_init(&feed);
    CHECK(0 == imu_feed_step(&feed, 200, -200, false, &input));
    CHECK(0 == input.distance_delta_mm);
}

static void
test_reverse_gate (void)
{
    puts("feed: reverse publishes zero distance, keeps deltas, pauses terrain");

    imu_feed_t feed;
    imu_encoder_input_t input;

    imu_feed_init(&feed);

    memset(&input, 0, sizeof(input));
    (void) imu_feed_step(&feed, 500, 500, false, &input);
    CHECK(!input.terrain_pause);        /* forward: classifier runs */

    memset(&input, 0, sizeof(input));
    CHECK(0 == imu_feed_step(&feed, -300, -300, true, &input));
    CHECK(0 == input.distance_delta_mm);
    CHECK(-300 == input.encoder_left_delta);
    CHECK(-300 == input.encoder_right_delta);
    CHECK(input.terrain_pause);         /* reverse: classifier frozen */

    /* The carry is dropped, so a resume cannot start from a reverse offset. */
    memset(&input, 0, sizeof(input));
    CHECK(0 == imu_feed_step(&feed, 0, 0, false, &input));
    CHECK(!input.terrain_pause);
}

/* ---- Terrain behaviour on the real feed -------------------------------- */

/**
 * Drain the queue once and count both episode events.
 *
 * One pop, because HUMP_START and HUMP_END share the queue: counting them in
 * two passes would lose whichever came first.
 */
typedef struct
{
    int        starts;
    int        ends;
    imu_event_t last_end;
} episode_count_t;

static episode_count_t
pop_episode_events (void)
{
    imu_event_t ev;
    episode_count_t counts;

    memset(&counts, 0, sizeof(counts));

    while (imu_event_pop(&ev))
    {
        if (IMU_EVT_HUMP_START == ev.id)
        {
            counts.starts++;
        }

        if (IMU_EVT_HUMP_END == ev.id)
        {
            counts.ends++;
            counts.last_end = ev;
        }
    }

    return counts;
}

/** One cycle with distance, mirroring the height integral while on the hump. */
static void
mirror_cycle (int16_t d_left, int16_t d_right, double * running, double * peak)
{
    imu_terrain_t const before = terrain_now();
    imu_diag_t diag;

    feed_step(d_left, d_right, false);
    imu_diag_read(&diag);

    if (IMU_TERRAIN_ON_HUMP == before)
    {
        *running += ((double) diag.distance_delta_mm) *
                    sin(((double) diag.pitch_deg) * 3.14159265358979323846 /
                        180.0);

        if (*running > *peak)
        {
            *peak = *running;
        }
    }
}

static void
test_flat_no_start (void)
{
    puts("(a) flat travel stays LEVEL and starts nothing");

    boot_level();
    (void) pop_episode_events();
    CHECK(IMU_TERRAIN_LEVEL == terrain_now());

    run_steps(200, 40, 40);

    imu_diag_t diag;
    imu_diag_read(&diag);

    printf("  flat: tilt %.2f deg, dd=%d mm, h=%.2f mm, terr=%s\n",
           (double) diag.tilt_deg, (int) diag.distance_delta_mm,
           (double) diag.hump_mm, imu_terrain_name(diag.terrain));

    CHECK(IMU_TERRAIN_LEVEL == diag.terrain);
    CHECK(diag.tilt_deg < IMU_CFG_TILT_ENTER_DEG);
    CHECK_NEAR(diag.hump_mm, 0.0, 0.01);
    CHECK(0 == pop_episode_events().starts);
    CHECK(0u == fields_now().highest_hump_mm);
}

static void
test_stationary_zero_height (void)
{
    puts("terrain: a tilted pose with no wheel travel integrates no height");

    boot_level();
    fake_set_pitch(6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));
    CHECK(IMU_TERRAIN_ON_HUMP == terrain_now());

    /* Same attitude, no distance: the pose alone must not create height. */
    run_steps(200, 0, 0);

    imu_diag_t diag;
    imu_diag_read(&diag);

    printf("  stationary: pitch %.1f deg, h=%.2f mm\n",
           (double) diag.pitch_deg, (double) diag.hump_mm);

    CHECK_NEAR(diag.hump_mm, 0.0, 0.01);
    CHECK_NEAR(diag.hump_peak_mm, 0.0, 0.01);
}

static void
test_pitch_ramp (void)
{
    puts("(b) a pitch-only forward ramp starts and ends ON_HUMP");

    boot_level();
    (void) pop_episode_events();

    fake_set_pitch(6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));

    /* Drive up it, over the top and down, then level off. A pitch-led ascent
     * only closes once its descent has been seen. */
    run_steps(120, 40, 40);
    fake_set_pitch(-6.0f);
    run_steps(60, 40, 40);
    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 120));

    episode_count_t const counts = pop_episode_events();

    printf("  pitch ramp: starts=%d ends=%d peak=%d mm, hi=%u mm\n",
           counts.starts, counts.ends, (int) counts.last_end.value,
           (unsigned) fields_now().highest_hump_mm);

    CHECK(1 == counts.starts);
    CHECK(1 == counts.ends);
    CHECK(IMU_EVT_DETAIL_RETAINED == counts.last_end.detail);
    CHECK(counts.last_end.value >= 15);
    CHECK(fields_now().highest_hump_mm >= 15u);
}

static void
test_roll_only_pose (void)
{
    puts("(c) a roll-only one-wheel pose starts ON_HUMP but makes no height");

    boot_level();
    (void) pop_episode_events();

    /* Pure roll, pitch left at zero, so the height integral cannot move even
     * though real distance is fed every cycle. */
    fake_set_attitude(0.0f, 2.5f);

    imu_diag_t diag;

    for (int i = 0; i < 60; i++)
    {
        feed_step(40, 40, false);
        imu_diag_read(&diag);
    }

    printf("  roll-only: pitch %.2f, roll %.2f, tilt %.2f, h=%.2f mm, terr=%s\n",
           (double) diag.pitch_deg, (double) diag.roll_deg,
           (double) diag.tilt_deg, (double) diag.hump_mm,
           imu_terrain_name(diag.terrain));

    CHECK(IMU_TERRAIN_ON_HUMP == diag.terrain);
    CHECK_NEAR(diag.pitch_deg, 0.0, 0.3);
    CHECK(diag.tilt_deg >= IMU_CFG_TILT_ENTER_DEG);
    CHECK_NEAR(diag.hump_mm, 0.0, 0.01);        /* distance, but no slope */

    /* Back to level: the episode ends, and it is rejected rather than retained
     * because a one-wheel pose proves a hump exists but not how tall it is. */
    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 120));

    episode_count_t counts = pop_episode_events();

    printf("  roll-only end: detail=%u value=%d mm, hi=%u mm\n",
           (unsigned) counts.last_end.detail, (int) counts.last_end.value,
           (unsigned) fields_now().highest_hump_mm);

    CHECK(1 == counts.starts);
    CHECK(1 == counts.ends);
    CHECK(IMU_EVT_DETAIL_REJECTED == counts.last_end.detail);
    CHECK(0u == fields_now().highest_hump_mm);

    /* The measured bench pose: one wheel on the hump, from RT-0017. */
    boot_level();
    fake_set_attitude(-2.0f, -2.3f);
    run_steps(60, 0, 0);
    imu_diag_read(&diag);

    printf("  bench pose: pitch %.2f, roll %.2f, tilt %.2f, terr=%s\n",
           (double) diag.pitch_deg, (double) diag.roll_deg,
           (double) diag.tilt_deg, imu_terrain_name(diag.terrain));

    CHECK(IMU_TERRAIN_ON_HUMP == diag.terrain);
    CHECK(diag.tilt_deg > 2.5f);        /* about 3.0 degrees resultant */
}

static void
test_one_episode_full_profile (void)
{
    puts("(d) climb, crest and descent stay one ON_HUMP episode");

    boot_level();
    (void) pop_episode_events();

    fake_set_pitch(6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));

    /* Mirror h += dd * sin(pitch) while the core is on the hump. */
    double running = 0.0;
    double peak    = 0.0;
    imu_diag_t diag;

    for (int i = 0; i < 100; i++)
    {
        imu_terrain_t const before = terrain_now();

        feed_step(40, 40, false);
        imu_diag_read(&diag);

        if (IMU_TERRAIN_ON_HUMP == before)
        {
            running += ((double) diag.distance_delta_mm) *
                       sin(((double) diag.pitch_deg) * 3.14159265358979323846 /
                           180.0);

            if (running > peak)
            {
                peak = running;
            }
        }
    }

    /* Over the crest. The chassis passes through level quickly, so the exit
     * dwell never completes and the same episode continues into the descent. */
    fake_set_pitch(-6.0f);
    run_steps(80, 40, 40);

    imu_diag_read(&diag);
    CHECK(IMU_TERRAIN_ON_HUMP == diag.terrain);

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 120));

    episode_count_t const counts = pop_episode_events();

    printf("  profile: starts=%d ends=%d event peak %d mm, mirrored %.1f mm, "
           "hi=%u mm\n",
           counts.starts, counts.ends, (int) counts.last_end.value, peak,
           (unsigned) fields_now().highest_hump_mm);

    CHECK(1 == counts.starts);          /* one episode, not three */
    CHECK(1 == counts.ends);
    CHECK(IMU_EVT_DETAIL_RETAINED == counts.last_end.detail);
    CHECK_NEAR(counts.last_end.value, peak, 3.0);
    CHECK(fields_now().highest_hump_mm >= 15u);
}

static void
test_tilt_hysteresis (void)
{
    puts("(e) 1.5 / 0.75 tilt hysteresis and the three-sample dwell");

    boot_level();

    /* Below the entry threshold nothing starts, however long we wait. */
    fake_set_pitch(1.2f);
    run_steps(150, 40, 40);
    CHECK(IMU_TERRAIN_LEVEL == terrain_now());
    CHECK(0 == pop_episode_events().starts);

    /* Cross the entry threshold and count the samples above it: the trace must
     * be exactly the three-sample dwell, no sooner and no later. */
    fake_set_pitch(3.0f);

    imu_diag_t diag;
    int above = 0;
    int state_after = -1;

    for (int i = 0; i < 80; i++)
    {
        feed_step(0, 0, false);
        imu_diag_read(&diag);

        if (diag.tilt_deg >= IMU_CFG_TILT_ENTER_DEG)
        {
            above++;
        }

        if (IMU_TERRAIN_ON_HUMP == diag.terrain)
        {
            state_after = above;
            break;
        }
    }

    printf("  dwell: entered after %d samples at or above %.2f deg\n",
           state_after, (double) IMU_CFG_TILT_ENTER_DEG);

    CHECK(3 == state_after);

    /* Between the two thresholds it must stay on the hump: leaving needs 0.75
     * degrees or less, not 1.5. */
    fake_set_pitch(1.0f);
    run_steps(150, 0, 0);
    imu_diag_read(&diag);

    printf("  hysteresis: tilt %.2f deg, terr=%s\n", (double) diag.tilt_deg,
           imu_terrain_name(diag.terrain));

    CHECK(IMU_TERRAIN_ON_HUMP == diag.terrain);
    CHECK(diag.tilt_deg < IMU_CFG_TILT_ENTER_DEG);
    CHECK(diag.tilt_deg > IMU_CFG_TILT_EXIT_DEG);

    /* A pitch-led ascent needs its descent before it can close. */
    fake_set_pitch(-6.0f);
    run_steps(60, 0, 0);

    /* At or below the exit threshold, and only now, the episode ends. */
    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 150));

    episode_count_t const counts = pop_episode_events();

    CHECK(1 == counts.starts);
    CHECK(1 == counts.ends);
}

/**
 * RT-0027: a broad, flat-topped crest must stay inside one episode.
 *
 * The crest holds level for 60 samples, about 780 ms, well past the 39 ms level
 * dwell that used to close the episode and open a second one on the descent.
 */
static void
test_broad_crest_holds (void)
{
    puts("RT-0027: a 780 ms flat crest stays inside one ON_HUMP episode");

    boot_level();
    (void) pop_episode_events();

    double running = 0.0;
    double peak    = 0.0;

    fake_set_pitch(6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));

    /* Ascent. */
    for (int i = 0; i < 60; i++)
    {
        mirror_cycle(40, 40, &running, &peak);
    }

    /* The crest: level, and long. */
    fake_set_pitch(0.0f);

    int left_early = 0;

    for (int i = 0; i < 60; i++)
    {
        mirror_cycle(40, 40, &running, &peak);

        if (IMU_TERRAIN_ON_HUMP != terrain_now())
        {
            left_early++;
        }
    }

    episode_count_t const during = pop_episode_events();

    imu_diag_t diag;
    imu_diag_read(&diag);

    printf("  crest: terr=%s, starts=%d, ends=%d, h held at %.1f mm\n",
           imu_terrain_name(terrain_now()), during.starts, during.ends,
           (double) diag.hump_mm);

    CHECK(0 == left_early);              /* never dropped out of ON_HUMP */
    CHECK(1 == during.starts);           /* one episode, still open */
    CHECK(0 == during.ends);             /* and it did not close at the crest */

    /* Now the descent, on the same episode, then the final level. */
    fake_set_pitch(-6.0f);

    for (int i = 0; i < 60; i++)
    {
        mirror_cycle(40, 40, &running, &peak);
    }

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 150));

    episode_count_t const after = pop_episode_events();

    printf("  closed: ends=%d, event peak %d mm, mirrored %.1f mm, hi=%u mm\n",
           after.ends, (int) after.last_end.value, peak,
           (unsigned) fields_now().highest_hump_mm);

    CHECK(1 == after.ends);              /* exactly one HUMP_END, at the end */
    CHECK(IMU_EVT_DETAIL_RETAINED == after.last_end.detail);
    CHECK_NEAR(after.last_end.value, peak, 3.0);
    CHECK(fields_now().highest_hump_mm >= 15u);
}

/** RT-0027: a nose-down start must still close, or it would stick ON_HUMP. */
static void
test_descending_start_ends (void)
{
    puts("RT-0027: a nose-down start still ends when the chassis levels out");

    boot_level();
    (void) pop_episode_events();

    /* Start the episode going down: there is no ascent, so no descent is owed. */
    fake_set_pitch(-6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));
    (void) pop_episode_events();

    run_steps(60, 40, 40);

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 150));

    episode_count_t const counts = pop_episode_events();

    printf("  nose-down start: terr=%s, ends=%d\n",
           imu_terrain_name(terrain_now()), counts.ends);

    CHECK(1 == counts.ends);
}

static void
test_reverse_isolation (void)
{
    puts("terrain: a reverse recovery neither starts nor modifies a hump");

    boot_level();
    (void) pop_episode_events();

    fake_set_pitch(6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));
    run_steps(60, 40, 40);

    imu_diag_t diag;
    imu_diag_read(&diag);

    float const height_before = diag.hump_mm;
    float const peak_before   = diag.hump_peak_mm;

    printf("  before reverse: h=%.1f mm, peak=%.1f mm\n", (double) height_before,
           (double) peak_before);
    CHECK(height_before > 5.0f);

    /* The Manager drives RECOVERY, so the feed gates distance to zero while the
     * wheels really do turn backwards. */
    for (int i = 0; i < 80; i++)
    {
        feed_step(-40, -40, true);
    }

    imu_diag_read(&diag);
    printf("  during reverse: h=%.1f mm, dd=%d mm\n", (double) diag.hump_mm,
           (int) diag.distance_delta_mm);

    CHECK_NEAR(diag.hump_mm, height_before, 0.01);
    CHECK_NEAR(diag.hump_peak_mm, peak_before, 0.01);
    CHECK(0 == diag.distance_delta_mm);
    CHECK(-40 == diag.encoder_left_delta);   /* raw deltas still visible */
    CHECK(0 == pop_episode_events().ends);

    /* Forward travel resumes from exactly where it stopped. */
    run_steps(40, 40, 40);

    imu_diag_read(&diag);
    CHECK(diag.hump_mm > height_before);
}

static void
test_reverse_blocks_start (void)
{
    puts("terrain: a reverse at hump-level tilt cannot start an episode");

    boot_level();
    (void) pop_episode_events();

    /* Tilt well past the 1.5 degree entry, then reverse for a long time. The
     * pitch-free dwell counters would have started an episode by now. */
    fake_set_pitch(4.0f);

    for (int i = 0; i < 120; i++)
    {
        feed_step(-40, -40, true);
    }

    imu_diag_t diag;
    imu_diag_read(&diag);

    printf("  reverse at +4 deg: terr=%s, starts=%d\n",
           imu_terrain_name(diag.terrain), pop_episode_events().starts);

    CHECK(IMU_TERRAIN_LEVEL == diag.terrain);
    CHECK_NEAR(diag.hump_mm, 0.0, 0.01);

    /* The freeze was the reason, not a missing signal: forward again starts it. */
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 120));
    CHECK(1 == pop_episode_events().starts);
}

static void
test_reverse_freezes_episode (void)
{
    puts("(f) a reverse mid-episode freezes state, height, peak and events");

    boot_level();
    (void) pop_episode_events();

    fake_set_pitch(6.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_ON_HUMP, 80));
    run_steps(60, 40, 40);

    imu_diag_t diag;
    imu_diag_read(&diag);

    float const height_before = diag.hump_mm;
    float const peak_before   = diag.hump_peak_mm;

    CHECK(height_before > 5.0f);

    /* Level the chassis out and reverse: without the pause the exit dwell
     * would end the episode and emit HUMP_END. */
    fake_set_pitch(0.0f);

    for (int i = 0; i < 120; i++)
    {
        feed_step(-40, -40, true);
    }

    imu_diag_read(&diag);

    printf("  reverse at level tilt: terr=%s, h=%.2f (was %.2f), ends=%d\n",
           imu_terrain_name(diag.terrain), (double) diag.hump_mm,
           (double) height_before, pop_episode_events().ends);

    CHECK(IMU_TERRAIN_ON_HUMP == diag.terrain);
    CHECK_NEAR(diag.hump_mm, height_before, 0.01);
    CHECK_NEAR(diag.hump_peak_mm, peak_before, 0.01);

    /* The episode is intact, not merely stalled: on resume the ascent still
     * owes its descent before it can close. */
    fake_set_pitch(-6.0f);
    run_steps(60, 40, 40);
    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 120));

    episode_count_t const counts = pop_episode_events();

    CHECK(1 == counts.ends);
}

int
main (void)
{
    test_conversion();
    test_fractional_carry();
    test_signed_average();
    test_reverse_gate();
    test_flat_no_start();
    test_stationary_zero_height();
    test_pitch_ramp();
    test_roll_only_pose();
    test_one_episode_full_profile();
    test_tilt_hysteresis();
    test_broad_crest_holds();
    test_descending_start_ends();
    test_reverse_isolation();
    test_reverse_blocks_start();
    test_reverse_freezes_episode();

    printf("\n%s (%d failure%s)\n", (0 == g_failures) ? "ALL PASSED" : "FAILED",
           g_failures, (1 == g_failures) ? "" : "s");

    return (0 == g_failures) ? 0 : 1;
}
