/**
 * @file test_imu.c
 * @brief Host tests for the IMU producer, driven through a fake LSM303DLHC.
 *
 * The tests compile the production core, driver, and snapshot codec, and
 * provide the hardware seam from imu_hal.h over a register file. Nothing about
 * the state machine is mocked.
 *
 *   cc -std=c11 -Wall -Wextra -Wshadow -I../src test_imu.c \
 *      ../src/imu_core.c ../src/imu_lsm303.c ../src/imu_snapshot.c \
 *      -lm -o test_imu && ./test_imu
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_config.h"
#include "imu_core.h"
#include "imu_hal.h"
#include "imu_lsm303.h"
#include "imu_snapshot.h"
#include "imu_types.h"

#define TEST_PI 3.14159265358979323846f

/* ---- Fake LSM303DLHC --------------------------------------------------- */

#define ACCEL_IDENTITY_REG 0x0Fu
#define ACCEL_IDENTITY_VAL 0x33u
#define ACCEL_OUT_REG      0x28u
#define MAG_IDENTITY_REG   0x0Au
#define MAG_IDENTITY_VAL   0x48u
#define MAG_OUT_REG        0x03u

static uint8_t g_accel_regs[256];
static uint8_t g_mag_regs[256];
static bool    g_accel_present;
static bool    g_mag_present;
static bool    g_bus_fault;
static bool    g_drdy;

static uint8_t *
regs_for (uint8_t addr7)
{
    if (addr7 == IMU_CFG_ACCEL_ADDR)
    {
        return g_accel_present ? g_accel_regs : NULL;
    }

    if (addr7 == IMU_CFG_MAG_ADDR)
    {
        return g_mag_present ? g_mag_regs : NULL;
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
    uint8_t * regs = regs_for(addr7);

    if (g_bus_fault || (NULL == regs))
    {
        return false;
    }

    regs[reg & 0x7Fu] = value;

    return true;
}

bool
imu_hal_i2c_read_reg (uint8_t addr7, uint8_t reg, uint8_t * dst, size_t len)
{
    uint8_t * regs = regs_for(addr7);

    if (g_bus_fault || (NULL == regs) || (NULL == dst))
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
    return g_drdy;
}

/* ---- Fake device helpers ---------------------------------------------- */

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
    memset(g_accel_regs, 0, sizeof(g_accel_regs));
    memset(g_mag_regs, 0, sizeof(g_mag_regs));

    g_accel_regs[ACCEL_IDENTITY_REG] = ACCEL_IDENTITY_VAL;
    g_mag_regs[MAG_IDENTITY_REG]     = MAG_IDENTITY_VAL;

    g_accel_present = true;
    g_mag_present   = true;
    g_bus_fault     = false;
    g_drdy          = true;
}

/** Accelerometer in milli-g. The full scale comes from imu_config.h, so the
 *  fake device saturates exactly where the real one does. */
static void
fake_set_accel_mg (int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    int16_t const values[3] = { x_mg, y_mg, z_mg };

    for (int a = 0; a < 3; a++)
    {
        int32_t counts = (int32_t) lroundf((float) values[a] /
                                           IMU_CFG_ACCEL_MG_PER_LSB);

        /* A 12-bit signed sensor reading saturates at +/-2047 counts. */
        if (counts > 2047)
        {
            counts = 2047;
        }

        if (counts < -2048)
        {
            counts = -2048;
        }

        int32_t const left_justified = counts * 16;
        uint16_t const bits = (uint16_t) ((int16_t) left_justified);

        g_accel_regs[ACCEL_OUT_REG + (2 * a)] =
            (uint8_t) (bits & 0xFFu);
        g_accel_regs[ACCEL_OUT_REG + (2 * a) + 1] = (uint8_t) (bits >> 8);
    }
}

/** Magnetometer in raw counts. The device orders X, Z, Y in its registers. */
static void
fake_set_mag_lsb (int16_t x, int16_t y, int16_t z)
{
    put_be16(g_mag_regs, MAG_OUT_REG, x);
    put_be16(g_mag_regs, (uint8_t) (MAG_OUT_REG + 2u), z);
    put_be16(g_mag_regs, (uint8_t) (MAG_OUT_REG + 4u), y);
}

/** A level, stationary robot facing `heading_deg`. */
static void
fake_set_level_heading (float heading_deg)
{
    float const rad = heading_deg * (TEST_PI / 180.0f);

    fake_set_accel_mg(0, 0, 1000);
    fake_set_mag_lsb((int16_t) lroundf(-3000.0f * sinf(rad)),
                     (int16_t) lroundf(3000.0f * cosf(rad)), 0);
}

/** Pure pitch, positive nose-up, magnetic north straight ahead. */
static void
fake_set_pitch (float pitch_deg)
{
    float const rad = pitch_deg * (TEST_PI / 180.0f);
    float const fwd = 1000.0f * sinf(rad);
    float const up  = 1000.0f * cosf(rad);

    fake_set_accel_mg(0, (int16_t) lroundf(fwd), (int16_t) lroundf(up));
    fake_set_mag_lsb(0, 3000, 0);
}

/* ---- Harness ----------------------------------------------------------- */

static int  g_failures;
static uint32_t g_now;

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

static void
step_once (void)
{
    g_now += IMU_CFG_PERIOD_MS;
    imu_step(g_now);
}

static void
run_steps (int count)
{
    for (int i = 0; i < count; i++)
    {
        step_once();
    }
}

static imu_terrain_t
terrain_now (void)
{
    imu_diag_t diag;

    imu_diag_read(&diag);

    return diag.terrain;
}

static void
diag_now (imu_diag_t * out)
{
    imu_diag_read(out);
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

/** Count queued events, optionally returning them. */
static int
pop_all (imu_event_t * out, int capacity)
{
    imu_event_t ev;
    int count = 0;

    while (imu_event_pop(&ev))
    {
        if ((NULL != out) && (count < capacity))
        {
            out[count] = ev;
        }

        count++;
    }

    return count;
}

/** Boot from power-on through self-test and stationary calibration. */
static void
boot_running (void)
{
    fake_reset();
    g_now = 1000u;
    imu_init(g_now);
    fake_set_level_heading(0.0f);
    run_steps(200);
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

        step_once();
    }

    return (terrain_now() == want);
}

/** One cycle with a fresh encoder delta published first. */
static void
step_distance (int16_t distance_mm, int16_t left, int16_t right)
{
    imu_encoder_input_t input;

    input.distance_delta_mm  = distance_mm;
    input.encoder_left_delta = left;
    input.encoder_right_delta = right;
    imu_publish_encoder_input(&input);
    step_once();
}

/* ---- Snapshot interleaving hook --------------------------------------- */

/*
 * Called by imu_snapshot_read() between the two word loads when the host build
 * defines IMU_TEST_SNAPSHOT_HOOK. Triggering a full publish here forces exactly
 * the interleaving a Manager task would see.
 */
static int g_hook_publishes_left;

void
imu_test_snapshot_hook (void)
{
    if (g_hook_publishes_left > 0)
    {
        g_hook_publishes_left--;

        /* One complete cycle rewrites both words and the sequence. The fault
         * also clears the heading word, so the two publishes are clearly
         * different and a torn pair is detectable. */
        g_bus_fault = true;
        step_once();
        g_bus_fault = false;
    }
}

/* ---- Tests ------------------------------------------------------------- */

static void
test_packet_packing (void)
{
    puts("snapshot: packing boundaries, sign extension, clamping");

    imu_snapshot_fields_t fields;
    imu_snapshot_t snapshot;
    imu_snapshot_fields_t back;

    memset(&fields, 0, sizeof(fields));
    memset(&back, 0, sizeof(back));

    /* Maximum positive values. */
    fields.heading_ddeg    = 3599u;
    fields.pitch_ddeg      = 511;
    fields.turn_rate_dps   = 511;
    fields.highest_hump_mm = 65535u;
    fields.terrain         = IMU_TERRAIN_SENSOR_FAULT;
    fields.accel_valid     = true;
    fields.mag_valid       = true;
    fields.calibrated      = true;
    fields.selftest_passed = true;
    fields.fault_latched   = true;
    fields.sequence        = 255u;

    imu_snapshot_pack(&fields, &snapshot);

    /* Word 1: heading 0..11, pitch 12..21, turn 22..31. */
    CHECK(0x0E0Fu == (snapshot.word[0] & 0xFFFu));
    CHECK(0x1FFu == ((snapshot.word[0] >> 12) & 0x3FFu));
    CHECK(0x1FFu == ((snapshot.word[0] >> 22) & 0x3FFu));

    /* Word 2: hump, terrain, five flags, sequence. */
    CHECK(0xFFFFu == (snapshot.word[1] & 0xFFFFu));
    CHECK(5u == ((snapshot.word[1] >> 16) & 0x7u));
    CHECK(0x1Fu == ((snapshot.word[1] >> 19) & 0x1Fu));
    CHECK(0xFFu == ((snapshot.word[1] >> 24) & 0xFFu));

    imu_snapshot_unpack(&snapshot, &back);
    CHECK(3599u == back.heading_ddeg);
    CHECK(511 == back.pitch_ddeg);
    CHECK(511 == back.turn_rate_dps);
    CHECK(65535u == back.highest_hump_mm);
    CHECK(IMU_TERRAIN_SENSOR_FAULT == back.terrain);
    CHECK(back.accel_valid && back.mag_valid && back.calibrated);
    CHECK(back.selftest_passed && back.fault_latched);
    CHECK(255u == back.sequence);

    /* Maximum negative values must sign-extend. */
    memset(&fields, 0, sizeof(fields));
    fields.pitch_ddeg    = -512;
    fields.turn_rate_dps = -512;
    fields.terrain       = IMU_TERRAIN_INIT;

    imu_snapshot_pack(&fields, &snapshot);
    CHECK(0x200u == ((snapshot.word[0] >> 12) & 0x3FFu));
    CHECK(0x200u == ((snapshot.word[0] >> 22) & 0x3FFu));

    imu_snapshot_unpack(&snapshot, &back);
    CHECK(-512 == back.pitch_ddeg);
    CHECK(-512 == back.turn_rate_dps);
    CHECK(IMU_TERRAIN_INIT == back.terrain);
    CHECK(0u == back.heading_ddeg);
    CHECK(!back.accel_valid && !back.fault_latched);

    /* Out-of-range input is clamped, not wrapped. */
    memset(&fields, 0, sizeof(fields));
    fields.pitch_ddeg    = 900;
    fields.turn_rate_dps = -900;

    imu_snapshot_pack(&fields, &snapshot);
    imu_snapshot_unpack(&snapshot, &back);
    CHECK(511 == back.pitch_ddeg);
    CHECK(-512 == back.turn_rate_dps);

    /* Every terrain value survives the round trip. */
    for (int t = 0; t <= 5; t++)
    {
        memset(&fields, 0, sizeof(fields));
        fields.terrain = (imu_terrain_t) t;
        imu_snapshot_pack(&fields, &snapshot);
        imu_snapshot_unpack(&snapshot, &back);
        CHECK((imu_terrain_t) t == back.terrain);
    }
}

static void
test_calibration_accept (void)
{
    puts("calibration: a stationary, level window is accepted");

    fake_reset();
    g_now = 1000u;
    imu_init(g_now);
    fake_set_level_heading(0.0f);
    run_steps(200);

    imu_event_t events[16];
    int const count = pop_all(events, 16);
    bool saw_selftest = false;
    bool saw_calibrated = false;

    for (int i = 0; i < count; i++)
    {
        if (events[i].id == IMU_EVT_SELFTEST)
        {
            saw_selftest = (0u == events[i].detail);
        }

        if (events[i].id == IMU_EVT_CALIBRATED)
        {
            saw_calibrated = true;
        }
    }

    CHECK(saw_selftest);
    CHECK(saw_calibrated);

    imu_snapshot_fields_t const fields = fields_now();
    CHECK(fields.calibrated);
    CHECK(fields.selftest_passed);
    CHECK(IMU_TERRAIN_LEVEL == fields.terrain);
    CHECK(fields.accel_valid && fields.mag_valid);
    CHECK(!fields.fault_latched);

    imu_diag_t diag;
    diag_now(&diag);
    CHECK(0u == diag.calibration_rejects);
    CHECK_NEAR(diag.accel_bias_g[2], 0.0, 0.01);
}

static void
test_calibration_reject (void)
{
    puts("calibration: out-of-band and unstable windows are rejected");

    fake_reset();
    g_now = 1000u;
    imu_init(g_now);

    /* Window 1: magnitude far outside 0.90..1.10 g. */
    fake_set_accel_mg(0, 0, 1500);
    fake_set_mag_lsb(0, 3000, 0);
    run_steps(200);

    imu_diag_t diag;
    diag_now(&diag);
    CHECK(diag.calibration_rejects >= 1u);
    CHECK(!fields_now().calibrated);
    CHECK(IMU_TERRAIN_INIT == terrain_now());

    /* Window 2: in band but oscillating, so the deviation is too large. */
    for (int i = 0; i < 200; i++)
    {
        fake_set_accel_mg(0, 0, (0 == (i % 2)) ? 920 : 1080);
        step_once();
    }

    diag_now(&diag);
    CHECK(diag.calibration_rejects >= 2u);
    CHECK(!fields_now().calibrated);

    /* Window 3: clean, so it is accepted. The window that was in progress
     * still holds alternating samples, so allow one more rejection first. */
    uint32_t const rejects_before = diag.calibration_rejects;

    fake_set_level_heading(0.0f);
    run_steps(400);

    CHECK(fields_now().calibrated);
    CHECK(IMU_TERRAIN_LEVEL == terrain_now());

    diag_now(&diag);
    CHECK(diag.calibration_rejects >= rejects_before);
    CHECK(diag.calibration_rejects <= (rejects_before + 1u));
}

static void
test_heading_wrap (void)
{
    puts("heading: 350 deg to 10 deg is a small positive rotation");

    boot_running();
    CHECK(fields_now().calibrated);

    fake_set_level_heading(350.0f);
    run_steps(60);

    imu_diag_t diag;
    diag_now(&diag);
    CHECK_NEAR(diag.heading_deg, 350.0, 2.0);

    /* Cross north. The turn rate must be small and positive, never -26000. */
    fake_set_level_heading(10.0f);

    float worst_magnitude = 0.0f;
    float strongest = 0.0f;

    for (int i = 0; i < 20; i++)
    {
        step_once();
        diag_now(&diag);

        if (fabsf(diag.turn_rate_dps) > worst_magnitude)
        {
            worst_magnitude = fabsf(diag.turn_rate_dps);
            strongest = diag.turn_rate_dps;
        }
    }

    printf("  wrap peak turn rate %.1f dps\n", (double) strongest);
    CHECK(strongest > 0.0f);
    CHECK(worst_magnitude < 2000.0f);

    run_steps(80);
    diag_now(&diag);
    CHECK(diag.heading_deg < 20.0f);
}

static void
test_terrain_transitions (void)
{
    puts("terrain: LEVEL -> CLIMBING -> CREST -> DESCENDING -> LEVEL");

    boot_running();
    CHECK(IMU_TERRAIN_LEVEL == terrain_now());
    (void) pop_all(NULL, 0);

    /* Climb. */
    fake_set_pitch(10.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CLIMBING, 40));

    imu_event_t events[16];
    int count = pop_all(events, 16);
    CHECK(count >= 1);
    CHECK(IMU_EVT_HUMP_START == events[0].id);

    /* Climb for a while so the hump has real height. */
    for (int i = 0; i < 40; i++)
    {
        step_distance(20, 20, 20);
    }

    imu_diag_t diag;
    diag_now(&diag);
    printf("  climb: pitch %.1f, hump %.1f mm\n", (double) diag.pitch_deg,
           (double) diag.hump_mm);
    CHECK(diag.hump_mm > 30.0f);

    /* Crest: level, but no descent yet. */
    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CREST, 40));

    /* Level long enough and the episode ends without a descent. */
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 40));

    count = pop_all(events, 16);
    bool saw_end = false;

    for (int i = 0; i < count; i++)
    {
        if (events[i].id == IMU_EVT_HUMP_END)
        {
            saw_end = true;
            CHECK(IMU_EVT_DETAIL_RETAINED == events[i].detail);
            CHECK(events[i].value > 30);
        }
    }

    CHECK(saw_end);
    CHECK(fields_now().highest_hump_mm > 30u);

    /* Descent straight from level. */
    fake_set_pitch(-10.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_DESCENDING, 40));

    count = pop_all(events, 16);
    CHECK(count >= 1);
    CHECK(IMU_EVT_HUMP_START == events[0].id);

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 40));
}

static void
test_descent_from_crest (void)
{
    puts("terrain: CREST -> DESCENDING");

    boot_running();
    (void) pop_all(NULL, 0);

    fake_set_pitch(10.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CLIMBING, 40));

    for (int i = 0; i < 20; i++)
    {
        step_distance(20, 20, 20);
    }

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CREST, 40));

    fake_set_pitch(-10.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_DESCENDING, 40));

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 40));
}

static void
test_false_start (void)
{
    puts("hump: a peak below 15 mm is a false start and is not retained");

    boot_running();
    (void) pop_all(NULL, 0);

    fake_set_pitch(10.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CLIMBING, 40));

    /* Only enough travel to reach a few millimetres. */
    for (int i = 0; i < 6; i++)
    {
        step_distance(10, 10, 10);
    }

    imu_diag_t diag;
    diag_now(&diag);
    printf("  false start peak %.1f mm\n", (double) diag.hump_peak_mm);
    CHECK(diag.hump_peak_mm < 15.0f);

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CREST, 40));
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 40));

    imu_event_t events[16];
    int const count = pop_all(events, 16);
    bool saw_end = false;

    for (int i = 0; i < count; i++)
    {
        if (events[i].id == IMU_EVT_HUMP_END)
        {
            saw_end = true;
            CHECK(IMU_EVT_DETAIL_REJECTED == events[i].detail);
            CHECK(events[i].value < 15);
        }
    }

    CHECK(saw_end);
    CHECK(0u == fields_now().highest_hump_mm);
}

static void
test_distance_integration (void)
{
    puts("hump: h += distance_delta * sin(pitch) while climbing");

    boot_running();
    (void) pop_all(NULL, 0);

    fake_set_pitch(30.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CLIMBING, 80));

    int16_t const delta_mm = 40;
    double expected = 0.0;
    imu_diag_t diag;

    for (int i = 0; i < 60; i++)
    {
        /* The core integrates only in CLIMBING and DESCENDING, using the pitch
         * it computed during this very step. Read the state before the step,
         * the pitch after it, and mirror exactly that arithmetic. */
        imu_terrain_t const before = terrain_now();

        step_distance(delta_mm, delta_mm, delta_mm);
        diag_now(&diag);

        if ((IMU_TERRAIN_CLIMBING == before) ||
            (IMU_TERRAIN_DESCENDING == before))
        {
            expected += ((double) delta_mm) *
                        sin(((double) diag.pitch_deg) * 3.14159265358979323846 /
                            180.0);
        }
    }

    printf("  integrated %.1f mm, expected %.1f mm\n", (double) diag.hump_peak_mm,
           expected);
    CHECK_NEAR(diag.hump_peak_mm, expected, 3.0);

    /* The reported peak must track the largest integrated height. */
    CHECK(diag.hump_mm <= diag.hump_peak_mm + 0.5f);
}

static void
test_run_reset (void)
{
    puts("run: imu_begin_run clears the retained peak and the odometer");

    boot_running();
    (void) pop_all(NULL, 0);

    fake_set_pitch(10.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_CLIMBING, 40));

    for (int i = 0; i < 40; i++)
    {
        step_distance(20, 20, 20);
    }

    fake_set_pitch(0.0f);
    CHECK(run_until_terrain(IMU_TERRAIN_LEVEL, 80));

    CHECK(fields_now().highest_hump_mm > 15u);

    imu_diag_t diag;
    diag_now(&diag);
    CHECK(diag.run_distance_mm > 0u);

    imu_begin_run();

    /* The packet is rebuilt by the next step, so let it publish. */
    step_once();

    CHECK(0u == fields_now().highest_hump_mm);
    diag_now(&diag);
    CHECK(0u == diag.run_distance_mm);
}

/** Count the events of one kind in a popped batch. */
static int
count_id (const imu_event_t * events, int count, uint8_t id)
{
    int matches = 0;

    for (int i = 0; i < count; i++)
    {
        matches += (events[i].id == id) ? 1 : 0;
    }

    return matches;
}

static void
test_collision_debounce (void)
{
    puts("collision: latch, single event per episode, then re-arm");

    boot_running();
    (void) pop_all(NULL, 0);

    imu_event_t events[32];
    int count;
    int reported;

    /* 3 g gives a 2 g residual, above the shipped 1.5 g threshold, and is
     * measurable at the configured 8 g full scale. The previous 2 g range
     * saturated at 2.047 g and could never reach this. */
    fake_set_accel_mg(0, 0, 3000);
    run_steps(40);

    count = pop_all(events, 32);
    reported = count_id(events, count, IMU_EVT_COLLISION);

    printf("  first impact: %d event(s)\n", reported);
    CHECK(1 == reported);

    /* Staying in the impact must not post again. */
    run_steps(40);

    count = pop_all(events, 32);
    CHECK(0 == count_id(events, count, IMU_EVT_COLLISION));

    /* Settle, so the detector re-arms. */
    fake_set_accel_mg(0, 0, 1000);
    run_steps(40);

    /* A second impact is reported again. */
    fake_set_accel_mg(0, 0, 3000);
    run_steps(40);

    count = pop_all(events, 32);
    CHECK(1 == count_id(events, count, IMU_EVT_COLLISION));
}

static void
test_snapshot_interleaving (void)
{
    puts("snapshot: a publish between the two word loads is retried");

    boot_running();
    (void) pop_all(NULL, 0);

    /* A clearly non-zero heading makes the two publishes differ. */
    fake_set_level_heading(90.0f);
    run_steps(120);

    imu_snapshot_fields_t const before = fields_now();

    printf("  before: seq %u, heading %u ddeg, terr %s\n",
           (unsigned) before.sequence, (unsigned) before.heading_ddeg,
           imu_terrain_name(before.terrain));

    CHECK(before.heading_ddeg > 500u);
    CHECK(!before.fault_latched);
    CHECK(IMU_TERRAIN_LEVEL == before.terrain);

    /* Force exactly one publish between the reader's two word loads. */
    g_hook_publishes_left = 1;

    imu_snapshot_t snapshot;
    bool const ok = imu_snapshot_read(&snapshot);

    imu_snapshot_fields_t after;
    memset(&after, 0, sizeof(after));
    imu_snapshot_unpack(&snapshot, &after);

    CHECK(ok);
    CHECK(0 == g_hook_publishes_left);      /* the hook really fired */

    /* The reader retried, so both words belong to the second publish. A torn
     * pair would keep the old non-zero heading with the new sequence. */
    CHECK(after.sequence == (uint8_t) (before.sequence + 1u));
    CHECK(after.fault_latched);
    CHECK(IMU_TERRAIN_SENSOR_FAULT == after.terrain);
    CHECK(0u == after.heading_ddeg);
    CHECK(!after.accel_valid);

    imu_diag_t diag;
    diag_now(&diag);
    CHECK(0u == diag.snapshot_failures);    /* the retry succeeded */

    /* Without an interleaved publish the pair is stable. */
    run_steps(5);

    imu_snapshot_t first;
    imu_snapshot_t second;

    CHECK(imu_snapshot_read(&first));
    CHECK(imu_snapshot_read(&second));
    CHECK(first.word[0] == second.word[0]);
    CHECK(first.word[1] == second.word[1]);
}

/** Drive one complete hump episode: climb a little, then level off. */
static void
drive_episode (void)
{
    fake_set_pitch(10.0f);
    (void) run_until_terrain(IMU_TERRAIN_CLIMBING, 40);

    for (int i = 0; i < 6; i++)
    {
        step_distance(20, 20, 20);
    }

    fake_set_pitch(0.0f);
    (void) run_until_terrain(IMU_TERRAIN_LEVEL, 80);
}

static void
test_queue_overflow_and_fault (void)
{
    puts("events: overflow drops the newest non-fault event, the fault survives");

    boot_running();
    (void) pop_all(NULL, 0);

    /* Two events per episode and never drained, so the 16-entry queue fills. */
    for (int i = 0; i < 12; i++)
    {
        drive_episode();
    }

    imu_diag_t diag;
    diag_now(&diag);
    printf("  posted %lu, dropped %lu, high water %lu\n",
           (unsigned long) diag.events_posted,
           (unsigned long) diag.events_dropped,
           (unsigned long) diag.queue_high_water);

    CHECK(diag.queue_high_water == IMU_CFG_EVENT_QUEUE_LEN);
    CHECK(diag.events_dropped > 0u);
    CHECK(diag.events_posted > IMU_CFG_EVENT_QUEUE_LEN);

    /* Now fault the bus with a full queue. */
    g_bus_fault = true;
    step_once();

    imu_event_t events[64];
    int const count = pop_all(events, 64);
    int faults = 0;
    int hump_starts = 0;

    for (int i = 0; i < count; i++)
    {
        faults      += (events[i].id == IMU_EVT_SENSOR_FAULT) ? 1 : 0;
        hump_starts += (events[i].id == IMU_EVT_HUMP_START) ? 1 : 0;
    }

    /* The fault event replaced a non-fault entry, so it is present. */
    CHECK(1 == faults);
    CHECK(hump_starts < 12);

    imu_snapshot_fields_t const fields = fields_now();
    CHECK(IMU_TERRAIN_SENSOR_FAULT == fields.terrain);
    CHECK(fields.fault_latched);
    CHECK(!fields.accel_valid);
    CHECK(!fields.mag_valid);
}

static void
test_i2c_fault_and_recovery (void)
{
    puts("fault: bus failure latches, retries once a second, then re-calibrates");

    boot_running();
    (void) pop_all(NULL, 0);
    CHECK(fields_now().calibrated);

    /* Break the bus. */
    g_bus_fault = true;
    step_once();

    imu_event_t events[16];
    int count = pop_all(events, 16);
    CHECK(count >= 1);
    CHECK(IMU_EVT_SENSOR_FAULT == events[0].id);

    imu_diag_t diag;
    diag_now(&diag);
    CHECK(1u == diag.i2c_errors);
    CHECK(IMU_TERRAIN_SENSOR_FAULT == diag.terrain);

    imu_snapshot_fields_t fields = fields_now();
    CHECK(fields.fault_latched);
    CHECK(!fields.accel_valid);
    CHECK(!fields.mag_valid);

    /* Staying broken must not post a new fault every cycle. */
    run_steps(50);

    count = pop_all(events, 16);
    CHECK(0 == count);

    /* Restore the bus: a retry succeeds and the whole start-up sequence runs
     * again before LEVEL is reported. */
    g_bus_fault = false;
    fake_set_level_heading(0.0f);

    run_steps(80);
    diag_now(&diag);
    CHECK(diag.recoveries >= 1u);
    CHECK(IMU_TERRAIN_INIT == diag.terrain);
    CHECK(!fields_now().calibrated);

    run_steps(200);

    fields = fields_now();
    CHECK(fields.calibrated);
    CHECK(fields.selftest_passed);
    CHECK(IMU_TERRAIN_LEVEL == fields.terrain);
    CHECK(!fields.fault_latched);

    count = pop_all(events, 16);
    bool saw_selftest = false;

    for (int i = 0; i < count; i++)
    {
        saw_selftest = saw_selftest || (events[i].id == IMU_EVT_SELFTEST);
    }

    CHECK(saw_selftest);
}

static void
test_missing_device (void)
{
    puts("fault: a missing magnetometer fails the self-test");

    fake_reset();
    g_now = 1000u;
    imu_init(g_now);
    fake_set_level_heading(0.0f);

    g_mag_present = false;
    step_once();

    imu_event_t events[16];
    int const count = pop_all(events, 16);
    CHECK(count >= 2);
    CHECK(IMU_EVT_SELFTEST == events[0].id);
    CHECK(IMU_LSM303_ST_MAG_ID == events[0].detail);
    CHECK(IMU_EVT_SENSOR_FAULT == events[1].id);

    CHECK(IMU_TERRAIN_SENSOR_FAULT == fields_now().terrain);
    CHECK(fields_now().fault_latched);
}

static void
test_sequence_progression (void)
{
    puts("snapshot: the sequence advances once per completed publish");

    boot_running();

    imu_snapshot_fields_t const before = fields_now();
    uint8_t expected = (uint8_t) (before.sequence + 1u);

    for (int i = 0; i < 40; i++)
    {
        step_once();

        imu_snapshot_fields_t const fields = fields_now();

        CHECK(fields.sequence == expected);
        expected = (uint8_t) (expected + 1u);
    }

    /* Wrapping through 255 back to 0 keeps the same rule. */
    run_steps(600);

    uint8_t const wrapped = fields_now().sequence;
    step_once();
    CHECK(fields_now().sequence == (uint8_t) (wrapped + 1u));
}

static void
test_encoder_diagnostics (void)
{
    puts("diagnostics: encoder deltas feed the cross-check but never fault");

    boot_running();
    (void) pop_all(NULL, 0);

    /* Sweep the heading continuously so the turn rate stays above the
     * cross-check floor, with the left wheel running further: a right turn. */
    for (int i = 0; i < 30; i++)
    {
        fake_set_level_heading((float) (5 * (i + 1)));
        step_distance(50, 120, 40);
    }

    imu_diag_t diag;
    diag_now(&diag);
    CHECK(diag.distance_delta_mm == 50);
    CHECK(diag.encoder_left_delta == 120);
    CHECK(diag.encoder_right_delta == 40);
    CHECK(diag.turn_crosscheck == IMU_CROSSCHECK_AGREE);

    /* Sweep the other way while the left wheel still runs further. The two
     * sources disagree, which must never raise an IMU fault. */
    for (int i = 0; i < 30; i++)
    {
        fake_set_level_heading((float) (150 - (5 * (i + 1))));
        step_distance(50, 120, 40);
    }

    diag_now(&diag);
    printf("  disagreeing cross-check: turn %.1f dps, x=%d\n",
           (double) diag.turn_rate_dps, (int) diag.turn_crosscheck);
    CHECK(diag.turn_crosscheck == IMU_CROSSCHECK_DISAGREE);
    CHECK(IMU_TERRAIN_LEVEL == diag.terrain);
    CHECK(!fields_now().fault_latched);

    /* No delta at all is counted, not integrated. */
    uint32_t const missed_before = diag.missed_distance;
    run_steps(10);
    diag_now(&diag);
    CHECK(diag.missed_distance >= missed_before + 10u);
}

int
main (void)
{
    test_packet_packing();
    test_calibration_accept();
    test_calibration_reject();
    test_heading_wrap();
    test_terrain_transitions();
    test_descent_from_crest();
    test_false_start();
    test_distance_integration();
    test_run_reset();
    test_collision_debounce();
    test_snapshot_interleaving();
    test_queue_overflow_and_fault();
    test_i2c_fault_and_recovery();
    test_missing_device();
    test_sequence_progression();
    test_encoder_diagnostics();

    printf("\n%s (%d failure%s)\n", (0 == g_failures) ? "ALL PASSED" : "FAILED",
           g_failures, (1 == g_failures) ? "" : "s");

    return (0 == g_failures) ? 0 : 1;
}
