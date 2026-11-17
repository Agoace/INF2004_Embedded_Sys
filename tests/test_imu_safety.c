/* Synthetic IMU safety fixtures. NOT hardware evidence: these prove the
 * rule structure, the latch and the separation between reasons. The
 * candidate bounds still need recorded lift traces from the robot.
 *
 * Every assertion states an observation that can only happen if the
 * intended path ran. */
#include "../app_program/subsystem4/imu_safety.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned int failures;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);             \
        }                                                                      \
    } while (0)

typedef struct {
    imu_safety_t s;
    imu_safety_input_t in;
    uint32_t now_ms;
} rig_t;

static void rig(rig_t *r)
{
    r->now_ms = 1000u;
    imu_safety_init(&r->s);
    r->in = (imu_safety_input_t){.accel_valid = true,
                                 .calibrated = true,
                                 .wheels_still = true,
                                 .now_ms = r->now_ms};
}

/* One 10 ms observation with the given mapped acceleration. */
static void feed(rig_t *r, float x, float y, float z)
{
    r->in.accel_mg[0] = x;
    r->in.accel_mg[1] = y;
    r->in.accel_mg[2] = z;
    r->in.now_ms = r->now_ms;
    imu_safety_step(&r->s, &r->in);
    r->now_ms += 10u;
}

static void feed_n(rig_t *r, int count, float x, float y, float z)
{
    for (int i = 0; i < count; ++i) {
        feed(r, x, y, z);
    }
}

static uint8_t latched(const rig_t *r)
{
    return r->s.output.latched_trigger;
}

/* --- Upright guard ---------------------------------------------------- */

static void test_upright_guard(void)
{
    const float level[3] = {0.0f, 0.0f, 1000.0f};

    CHECK(imu_safety_upright_ok(level));

    const float top_up[3] = {0.0f, 0.0f, 700.0f};
    CHECK(imu_safety_upright_ok(top_up));

    /* Upside down: gravity reads on -Z. Must never become level bias. */
    const float inverted[3] = {0.0f, 0.0f, -1000.0f};
    CHECK(!imu_safety_upright_ok(inverted));

    const float on_side[3] = {1000.0f, 0.0f, 0.0f};
    CHECK(!imu_safety_upright_ok(on_side));

    /* Upright but too tilted for an acceptable level reference. */
    const float tilted[3] = {0.0f, 0.0f, 500.0f};
    CHECK(!imu_safety_upright_ok(tilted));

    const float skewed[3] = {500.0f, 0.0f, 1000.0f};
    CHECK(!imu_safety_upright_ok(skewed));

    const float nan_pose[3] = {NAN, NAN, NAN};
    CHECK(!imu_safety_upright_ok(nan_pose));
}

/* --- Quiet upright run: no trigger, all-clear accumulates ------------- */

static void test_quiet_upright_is_clear(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 150, 0.0f, 0.0f, 1000.0f);
    CHECK(latched(&r) == IMU_SAFETY_NONE);
    CHECK(r.s.output.stops == 0u);
    CHECK(r.s.output.clear_ms == IMU_SAFETY_CLEAR_MS);
    CHECK((r.s.output.flags & IMU_SAFETY_FLAG_UPRIGHT) != 0u);
    CHECK((r.s.output.flags & IMU_SAFETY_FLAG_VALID) != 0u);
    CHECK(imu_safety_clear(&r.s));
}

/* --- Lift: free fall -------------------------------------------------- */

static void test_freefall_latches_lift(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    CHECK(latched(&r) == IMU_SAFETY_NONE);

    /* One low-g sample is not enough. */
    feed(&r, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_NONE);

    /* The second confirms. */
    feed(&r, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
    CHECK(r.s.output.stops == 1u);
    CHECK(r.s.output.current_trigger == (uint8_t)IMU_SAFETY_LIFT);
}

/* --- Lift: vertical change -------------------------------------------- */

static void test_vertical_lift(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);

    /* 600 mg above the reference: a brisk lift. Two samples are below the
     * three-sample window. */
    feed_n(&r, 2, 0.0f, 0.0f, 1600.0f);
    CHECK(latched(&r) == IMU_SAFETY_NONE);
    CHECK(r.s.output.lift_samples == 2u);

    feed(&r, 0.0f, 0.0f, 1600.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
}

static void test_vertical_lift_downwards(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);

    /* A set-down is also a vertical transient of the same magnitude. */
    feed_n(&r, 3, 0.0f, 0.0f, 400.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
}

/* --- Overturn --------------------------------------------------------- */

static void test_overturn_latches(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);

    /* 90 degrees: |a| stays 1 g, so this is a direction change only. */
    feed_n(&r, 4, 1000.0f, 0.0f, 0.0f);
    CHECK(latched(&r) == IMU_SAFETY_NONE);
    CHECK(r.s.output.overturn_samples == 4u);

    feed(&r, 1000.0f, 0.0f, 0.0f);
    CHECK(latched(&r) == IMU_SAFETY_OVERTURN);
    CHECK(r.s.output.tilt_ddeg > 850);
}

/* --- Reason separation ------------------------------------------------- */

/* A tilt that keeps |a| near 1 g is an overturn, never a lift, even though
 * the gravity vector moved by 1000 mg against the reference. */
static void test_tilt_is_not_a_lift(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 4, 1000.0f, 0.0f, 0.0f);

    CHECK(r.s.output.lift_samples == 0u);
    CHECK(latched(&r) != IMU_SAFETY_LIFT);
}

/* The overturn window is a run of near-1 g observations. A low-g sample
 * breaks the run, so a tilted robot that is then dropped reports LIFT and
 * the overturn streak restarts. This is the reachable form of the
 * precedence rule: lift is reported when the fall happens. */
static void test_tilt_then_fall_reports_lift(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 4, 1000.0f, 0.0f, 0.0f);
    CHECK(r.s.output.overturn_samples == 4u);

    /* The drop: low-g samples break the near-1 g overturn run. */
    feed(&r, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_NONE);
    CHECK(r.s.output.overturn_samples == 0u);

    feed(&r, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
    CHECK(r.s.output.stops == 1u);
}

/* A fifth over-limit near-1 g sample completes the overturn window. */
static void test_full_tilt_window_reports_overturn(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 5, 1000.0f, 0.0f, 0.0f);
    CHECK(latched(&r) == IMU_SAFETY_OVERTURN);
    CHECK(r.s.output.stops == 1u);
}

/* --- Latch ------------------------------------------------------------ */

static void test_latch_holds_and_freezes(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 2, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);

    const uint8_t stops = r.s.output.stops;

    /* Ten seconds of quiet, upright pose does not clear the latch. */
    feed_n(&r, 1000, 0.0f, 0.0f, 1000.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
    CHECK(r.s.output.stops == stops);
    CHECK(r.s.output.current_trigger == (uint8_t)IMU_SAFETY_NONE);

    /* The all-clear window still measured, so a safe restart is possible. */
    CHECK(r.s.output.clear_ms == IMU_SAFETY_CLEAR_MS);
    CHECK(imu_safety_clear(&r.s));
    CHECK(latched(&r) == IMU_SAFETY_NONE);
}

static void test_clear_refused_while_unsafe(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 2, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);

    /* Left on its side: never an all-clear window. */
    feed_n(&r, 200, 1000.0f, 0.0f, 0.0f);
    CHECK(!imu_safety_clear(&r.s));
    CHECK(latched(&r) == IMU_SAFETY_LIFT);

    /* Moving wheels break the window too. */
    feed_n(&r, 50, 0.0f, 0.0f, 1000.0f);
    r.in.wheels_still = false;
    feed_n(&r, 200, 0.0f, 0.0f, 1000.0f);
    CHECK(!imu_safety_clear(&r.s));
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
}

/* A clear needs a full second, not one good sample. */
static void test_clear_needs_one_second(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 2, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);

    feed_n(&r, 99, 0.0f, 0.0f, 1000.0f); /* 990 ms */
    CHECK(!imu_safety_clear(&r.s));

    feed(&r, 0.0f, 0.0f, 1000.0f); /* 1000 ms */
    CHECK(imu_safety_clear(&r.s));
}

/* With the latch clear, detection must work again. */
static void test_detection_resumes_after_clear(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 2, 0.0f, 0.0f, 300.0f);
    feed_n(&r, 120, 0.0f, 0.0f, 1000.0f);
    CHECK(imu_safety_clear(&r.s));
    CHECK(latched(&r) == IMU_SAFETY_NONE);

    feed_n(&r, 2, 0.0f, 0.0f, 300.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
    CHECK(r.s.output.stops == 2u);
}

/* --- Invalid and stale input ------------------------------------------ */

static void test_invalid_input_has_no_trigger(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    r.in.accel_valid = false;
    feed_n(&r, 100, 0.0f, 0.0f, 1000.0f);

    CHECK(latched(&r) == IMU_SAFETY_NONE);
    /* The VALID flag is clear, and the split names the specific reason: the
     * read failed while the vector itself stayed finite (plan Phase 1.3). */
    CHECK((r.s.output.flags & IMU_SAFETY_FLAG_VALID) == 0u);
    CHECK((r.s.output.flags & IMU_SAFETY_FLAG_READ_FAILED) != 0u);
    CHECK(imu_safety_unavailable_cause(&r.s.output, true, r.now_ms,
                                       IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_CAUSE_READ_FAILED);
    CHECK(r.s.output.clear_ms == 0u);
    CHECK(imu_safety_evaluate(&r.s.output, true, r.now_ms,
                              IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE);
}

/* The three invalid sub-causes must be distinguishable. */
static void test_invalid_causes_split(void)
{
    rig_t r;

    /* 1. Degenerate read. imu_core marks a zero or non-finite vector as
     * accel_valid=false, so the detector sees the invalid read and the raw
     * vector is what distinguishes it from an ordinary read failure. */
    rig(&r);
    r.in.accel_valid = false;
    feed_n(&r, 3, 0.0f, 0.0f, 0.0f);
    CHECK((r.s.output.flags & IMU_SAFETY_FLAG_ZERO_VECTOR) != 0u);
    CHECK(imu_safety_unavailable_cause(&r.s.output, true, r.now_ms,
                                       IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_CAUSE_ZERO_VECTOR);

    /* 2. Sample present but the level bias is not trusted. */
    rig(&r);
    r.in.calibrated = false;
    feed_n(&r, 3, 0.0f, 0.0f, 1000.0f);
    CHECK((r.s.output.flags & IMU_SAFETY_FLAG_NOT_CALIBRATED) != 0u);
    CHECK(imu_safety_unavailable_cause(&r.s.output, true, r.now_ms,
                                       IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_CAUSE_NOT_CALIBRATED);

    /* 3. The legacy generic meaning survives a board that sets no specific
     * flag. */
    imu_safety_output_t plain = {0};
    plain.observation_ms = 1000u;
    CHECK(imu_safety_unavailable_cause(&plain, true, 1000u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_INVALID);
}

/* The clear-window blockers must name the condition that stops the window. */
static void test_clear_blockers(void)
{
    rig_t r;

    /* Upright, still, quiet: no blocker. */
    rig(&r);
    feed_n(&r, 5, 0.0f, 0.0f, 1000.0f);
    CHECK(r.s.output.clear_blockers == 0u);

    /* Not upright: a 66 deg tilt is fresh and near 1 g, so only the upright
     * and near-1 g bits may be clear or set independently. */
    rig(&r);
    feed_n(&r, 5, 916.0f, 0.0f, 400.0f);
    CHECK((r.s.output.clear_blockers & IMU_SAFETY_BLOCK_NOT_UPRIGHT) != 0u);

    /* Wheels moving blocks the window even in a perfect pose. */
    rig(&r);
    r.in.wheels_still = false;
    feed_n(&r, 5, 0.0f, 0.0f, 1000.0f);
    CHECK((r.s.output.clear_blockers & IMU_SAFETY_BLOCK_WHEELS_MOVING) != 0u);
    CHECK(r.s.output.clear_ms == 0u);

    /* A gap over the cadence is named as such. */
    rig(&r);
    feed_n(&r, 2, 0.0f, 0.0f, 1000.0f);
    r.now_ms += 40u;
    feed(&r, 0.0f, 0.0f, 1000.0f);
    CHECK((r.s.output.clear_blockers & IMU_SAFETY_BLOCK_GAP) != 0u);
    CHECK(r.s.output.clear_ms == 0u);
}

static void test_uncalibrated_input_has_no_trigger(void)
{
    rig_t r;

    rig(&r);
    r.in.calibrated = false;
    feed_n(&r, 100, 0.0f, 0.0f, 1000.0f);

    CHECK(latched(&r) == IMU_SAFETY_NONE);
    CHECK(imu_safety_evaluate(&r.s.output, true, r.now_ms,
                              IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE);
}

static void test_evaluate_staleness(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);

    const imu_safety_output_t *out = &r.s.output;
    const uint32_t stamp = out->observation_ms;

    CHECK(imu_safety_evaluate(out, true, stamp, IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_NONE);
    CHECK(imu_safety_evaluate(out, true, stamp + IMU_SAFETY_MAX_AGE_MS,
                              IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_NONE);
    CHECK(imu_safety_evaluate(out, true, stamp + IMU_SAFETY_MAX_AGE_MS + 1u,
                              IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE);
    CHECK(imu_safety_evaluate(out, false, stamp, IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE);
    CHECK(imu_safety_evaluate(NULL, true, stamp, IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE);
}

/* A latched trigger must survive a stale copy: the reader still stops. */
static void test_evaluate_reports_latch(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 2, 0.0f, 0.0f, 300.0f);

    CHECK(imu_safety_evaluate(&r.s.output, true, r.now_ms,
                              IMU_SAFETY_MAX_AGE_MS) ==
          (uint8_t)IMU_SAFETY_LIFT);
}

/* --- Gravity reference ------------------------------------------------ */

/* A short transient must not become the new reference: if it did, the
 * third sample would no longer exceed the bound and no lift would latch. */
static void test_gravity_frozen_during_transient(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    CHECK(r.s.gravity_ready);
    CHECK(fabsf(r.s.gravity_mg[2] - 1000.0f) < 1.0f);

    feed_n(&r, 2, 0.0f, 0.0f, 1600.0f);
    /* Frozen: the reference did not follow the transient. */
    CHECK(fabsf(r.s.gravity_mg[2] - 1000.0f) < 5.0f);

    feed(&r, 0.0f, 0.0f, 1600.0f);
    CHECK(latched(&r) == IMU_SAFETY_LIFT);
}

/* A quiet observation does update the reference. */
static void test_gravity_tracks_quiet(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 200, 0.0f, 0.0f, 1010.0f);
    CHECK(r.s.gravity_ready);
    CHECK(fabsf(r.s.gravity_mg[2] - 1010.0f) < 5.0f);
}

/* --- Deci-degree helper ------------------------------------------------ */

static void test_tilt_ddeg(void)
{
    const float level[3] = {0.0f, 0.0f, 1000.0f};
    const float side[3] = {1000.0f, 0.0f, 0.0f};
    const float flip[3] = {0.0f, 0.0f, -1000.0f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    CHECK(imu_safety_tilt_ddeg(level) == 0);
    CHECK(imu_safety_tilt_ddeg(side) == 900);
    CHECK(imu_safety_tilt_ddeg(flip) == 1800);
    CHECK(imu_safety_tilt_ddeg(zero) == 0);
    CHECK(imu_safety_tilt_ddeg(NULL) == 0);
}

/* --- All-clear window needs continuous observation (RT-0094/F1) ------- */

/* A gap longer than the observation cadence restarts the window. Sparse
 * valid samples must not add up to a safe-restart window. */
static void test_clear_window_restarts_after_gap(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 99, 0.0f, 0.0f, 1000.0f);
    CHECK(r.s.output.clear_ms < IMU_SAFETY_CLEAR_MS);

    r.now_ms += 30u; /* 40 ms since the previous observation */
    feed(&r, 0.0f, 0.0f, 1000.0f);
    CHECK(r.s.output.clear_ms == 0u);
    CHECK(!imu_safety_clear(&r.s));
}

/* Sparse observations alone never complete the window, whatever their pose. */
static void test_sparse_observations_never_clear(void)
{
    rig_t r;

    rig(&r);
    for (int i = 0; i < 200; ++i) {
        feed(&r, 0.0f, 0.0f, 1000.0f);
        r.now_ms += 30u; /* 40 ms spacing: every observation is a gap */
    }
    CHECK(r.s.output.clear_ms == 0u);
    CHECK(!imu_safety_clear(&r.s));
}

/* --- Separation gate, deliberate lift coverage limit (RT-0092/F4) ----- */

/* A sudden tilt rotates gravity while |a| stays near 1 g. The vertical-change
 * rule must not report that as a lift, and the tilt is below the overturn
 * limit, so nothing latches. Removing the near-1 g gate makes this a LIFT
 * after three observations. */
static void test_sudden_tilt_near_one_g_is_not_a_lift(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f); /* settle gravity on +Z */
    feed_n(&r, 6, 916.0f, 0.0f, 400.0f); /* 66.4 deg, |a| = 999.5 mg */
    CHECK(latched(&r) == IMU_SAFETY_NONE);
    CHECK(r.s.output.stops == 0u);
    CHECK(r.s.output.tilt_ddeg > 660 && r.s.output.tilt_ddeg < 670);
}

/* The overturn path still catches a tilt past its own limit in the same
 * near-1 g condition. */
static void test_tilt_beyond_overturn_latches_overturn(void)
{
    rig_t r;

    rig(&r);
    feed_n(&r, 20, 0.0f, 0.0f, 1000.0f);
    feed_n(&r, 5, 985.0f, 0.0f, 174.0f); /* 80 deg, |a| = 1000.25 mg */
    CHECK(latched(&r) == IMU_SAFETY_OVERTURN);
    CHECK(r.s.output.stops == 1u);
    CHECK(r.s.output.lift_samples == 0u);
}

/* --- Unavailability cause (RT-0099) ---------------------------------- */

/* The stop uses one trigger value for every kind of IMU unavailability. The
 * cause helper separates them for the field event and the console line. */
static void test_unavailable_causes(void)
{
    imu_safety_output_t out = {0};

    /* No published copy at all: absent, even if a copy pointer exists. */
    CHECK(imu_safety_unavailable_cause(&out, false, 1000u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_ABSENT);
    CHECK(imu_safety_unavailable_cause(NULL, true, 1000u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_ABSENT);

    /* Valid and fresh: not unavailable, so no cause. */
    out.flags = IMU_SAFETY_FLAG_VALID;
    out.observation_ms = 1000u;
    CHECK(imu_safety_unavailable_cause(&out, true, 1020u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_NONE);

    /* Valid but older than the limit: stale. */
    CHECK(imu_safety_unavailable_cause(&out, true, 1080u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_STALE);

    /* Not valid wins over both freshness and staleness, because it is the
     * more specific cause. */
    out.flags = 0u;
    CHECK(imu_safety_unavailable_cause(&out, true, 1000u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_INVALID);
    CHECK(imu_safety_unavailable_cause(&out, true, 2000u, 50u) ==
          (uint8_t)IMU_SAFETY_CAUSE_INVALID);

    /* The cause agrees with evaluate: a cause exists exactly when the reader
     * reports unavailable. */
    out.flags = IMU_SAFETY_FLAG_VALID;
    out.observation_ms = 1000u;
    for (uint32_t age = 0u; age <= 120u; age += 10u) {
        const uint8_t trigger = imu_safety_evaluate(&out, true,
                                                    1000u + age, 50u);
        const uint8_t cause = imu_safety_unavailable_cause(&out, true,
                                                           1000u + age, 50u);
        CHECK((trigger == (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE) ==
              (cause != (uint8_t)IMU_SAFETY_CAUSE_NONE));
    }
}

int main(void)
{
    test_upright_guard();
    test_quiet_upright_is_clear();
    test_freefall_latches_lift();
    test_vertical_lift();
    test_vertical_lift_downwards();
    test_overturn_latches();
    test_tilt_is_not_a_lift();
    test_tilt_then_fall_reports_lift();
    test_full_tilt_window_reports_overturn();
    test_latch_holds_and_freezes();
    test_clear_refused_while_unsafe();
    test_clear_needs_one_second();
    test_detection_resumes_after_clear();
    test_invalid_input_has_no_trigger();
    test_uncalibrated_input_has_no_trigger();
    test_evaluate_staleness();
    test_evaluate_reports_latch();
    test_gravity_frozen_during_transient();
    test_gravity_tracks_quiet();
    test_tilt_ddeg();
    test_clear_window_restarts_after_gap();
    test_sparse_observations_never_clear();
    test_sudden_tilt_near_one_g_is_not_a_lift();
    test_tilt_beyond_overturn_latches_overturn();
    test_unavailable_causes();
    test_invalid_causes_split();
    test_clear_blockers();

    if (failures == 0u) {
        printf("ALL PASSED (0 failures)\n");
        return 0;
    }
    printf("%u FAILURES\n", failures);
    return 1;
}
