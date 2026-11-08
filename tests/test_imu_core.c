#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../app_program/subsystem4/imu_core.h"

static imu_core_input_t level(void)
{
    const imu_core_input_t in = {
        .accel_mg = {-20.0f, 80.0f, 1375.0f},
        .accel_valid = true,
        .motion_valid = true,
        .wheels_still = true,
        .calibration_allowed = true
    };

    return in;
}

static void tick(imu_core_t *core, imu_core_input_t *in)
{
    imu_core_step(core, in);
    in->now_ms += 10u;
    in->travel_ms += 10u;
}

static void ticks(imu_core_t *core, imu_core_input_t *in, unsigned int n)
{
    for (unsigned int i = 0u; i < n; ++i) {
        tick(core, in);
    }
}

static void calibrated(imu_core_t *core, imu_core_input_t *in)
{
    imu_core_init(core);
    *in = level();
    ticks(core, in, 101u);
    assert(core->output.calibrated);
    assert(core->output.calibrations == 1u);
    assert(fabsf(core->output.bias_mg[0] + 20.0f) < 0.01f);
    assert(fabsf(core->output.bias_mg[1] - 80.0f) < 0.01f);
    assert(fabsf(core->output.bias_mg[2] - 375.0f) < 0.01f);
    assert(core->output.pitch_ddeg == 0);
    assert(core->output.roll_ddeg == 0);
}

static void calibration_time_and_refresh(void)
{
    imu_core_t core;
    imu_core_input_t in = level();

    imu_core_init(NULL);
    imu_core_mark(NULL);
    imu_core_step(NULL, &in);
    imu_core_init(&core);
    imu_core_step(&core, NULL);
    ticks(&core, &in, 100u);
    assert(!core.output.calibrated);
    tick(&core, &in);
    assert(core.output.calibrated && core.output.calibrations == 1u);
    assert(core.output.raw_pitch_ddeg > 30);
    assert(core.output.calibration_rejects == 0u);
    assert(core.output.accel_mg[2] == 1000.0f);
    ticks(&core, &in, 101u);
    assert(core.output.calibrations == 2u);
    assert(core.output.samples == 202u);
    assert(core.output.valid_samples == 202u);
    assert(core.output.invalid_samples == 0u);
}

static void calibration_rejects(void)
{
    imu_core_t core;
    imu_core_input_t in = level();

    imu_core_init(&core);
    for (unsigned int i = 0u; i < 101u; ++i) {
        in.accel_mg[0] = -20.0f + (i % 2u == 0u ? 30.0f : -30.0f);
        tick(&core, &in);
    }
    assert(!core.output.calibrated);
    assert(core.output.calibration_rejects == 1u);
    assert(core.output.still_std_mg[0] > IMU_STILL_STD_MG);
    /* Low standard deviation, but a corrected Z envelope above 1.10 g.
     * A corrected-MEAN-only gate would wrongly accept this window. */
    imu_core_init(&core);
    in = level();
    for (unsigned int i = 0u; i < 101u; ++i) {
        in.accel_mg[2] = i == 99u ? 1486.0f : 1375.0f;
        tick(&core, &in);
    }
    assert(!core.output.calibrated);
    assert(core.output.still_std_mg[2] < IMU_STILL_STD_MG);
    assert(core.output.calibration_rejects == 1u);
    in.accel_mg[2] = 1375.0f;
    ticks(&core, &in, 101u);
    assert(core.output.calibrated);
    assert(core.output.calibration_rejects == 1u);
}

static void motion_and_spin_inhibit(void)
{
    imu_core_t core;
    imu_core_input_t in = level();

    imu_core_init(&core);
    /* Mean distance is zero during a spin: BOTH wheel activity matters. */
    in.wheels_still = false;
    ticks(&core, &in, 300u);
    assert(!core.output.calibrated && core.still_count == 0u);
    in.wheels_still = true;
    in.calibration_allowed = false;
    ticks(&core, &in, 150u);
    assert(!core.output.calibrated);
    in.calibration_allowed = true;
    in.calibration_epoch = 1u;
    ticks(&core, &in, 150u);
    assert(!core.output.calibrated);
    in.calibration_epoch = 2u;
    ticks(&core, &in, 90u);
    /* A whole episode between polls still invalidates the old window. */
    in.calibration_epoch = 4u;
    ticks(&core, &in, 100u);
    assert(!core.output.calibrated);
    tick(&core, &in);
    assert(core.output.calibrated);
}

static void missing_and_invalid_data(void)
{
    imu_core_t core;
    imu_core_input_t in = level();

    imu_core_init(&core);
    ticks(&core, &in, 90u);
    in.accel_valid = false;
    tick(&core, &in);
    assert(core.output.invalid_samples == 1u);
    in.accel_valid = true;
    ticks(&core, &in, 100u);
    assert(!core.output.calibrated);
    tick(&core, &in);
    assert(core.output.calibrated);
    in.motion_valid = false;
    ticks(&core, &in, 150u);
    assert(core.output.motion_misses == 150u);
    assert(core.output.calibrations == 1u);
    in.motion_valid = true;
    in.accel_mg[1] = NAN;
    tick(&core, &in);
    assert(!core.output.valid);
    in = level();
    in.now_ms = core.previous_ms + 10u;
    in.travel_ms = in.now_ms;
    in.accel_mg[0] = INFINITY;
    tick(&core, &in);
    assert(!core.output.valid);
    in = level();
    in.now_ms = core.previous_ms + 10u;
    in.travel_ms = in.now_ms;
    in.travel_mm = NAN;
    tick(&core, &in);
    assert(core.output.valid && !core.output.motion_valid);
    assert(core.still_count == 0u);
    in.travel_mm = 0.0f;
    memset(in.accel_mg, 0, sizeof(in.accel_mg));
    tick(&core, &in);
    assert(!core.output.valid);
}

static void angles_and_filter(void)
{
    imu_core_t core;
    imu_core_input_t in;
    const float pitch_rad = 0.261799388f;
    const float roll_rad = 0.523598776f;

    calibrated(&core, &in);
    in.wheels_still = false;
    in.accel_mg[0] = -20.0f + 1000.0f * sinf(roll_rad) * cosf(pitch_rad);
    in.accel_mg[1] = 80.0f + 1000.0f * sinf(pitch_rad);
    in.accel_mg[2] = 375.0f + 1000.0f * cosf(roll_rad) * cosf(pitch_rad);
    tick(&core, &in);
    assert(abs(core.output.pitch_ddeg - 150) <= 1);
    assert(abs(core.output.roll_ddeg - 300) <= 1);
    assert(core.output.filtered_pitch_ddeg > 0);
    assert(core.output.filtered_pitch_ddeg < core.output.pitch_ddeg);
    ticks(&core, &in, 100u);
    assert(abs(core.output.filtered_pitch_ddeg - 150) <= 1);
    assert(abs(core.output.filtered_roll_ddeg - 300) <= 1);
    assert(core.output.calibrations == 1u);
}

static void encoder_acceleration(float acceleration, float initial_speed)
{
    imu_core_t core;
    imu_core_input_t in;

    calibrated(&core, &in);
    in.wheels_still = false;
    in.accel_mg[1] = 80.0f + acceleration / 9.80665f;
    for (unsigned int i = 1u; i <= 200u; ++i) {
        const float t = (float)i * 0.01f;

        in.travel_mm = initial_speed * t + 0.5f * acceleration * t * t;
        tick(&core, &in);
    }
    assert(fabsf(core.output.forward_accel_mm_s2 - acceleration) < 5.0f);
    assert(abs(core.output.corrected_pitch_ddeg) <= 2);
    assert(abs(core.output.pitch_ddeg) >= 25);
    assert(core.output.calibrations == 1u);
}

static void flat_motion(imu_core_t *core, imu_core_input_t *in,
                         unsigned int count, float ds)
{
    in->wheels_still = false;
    for (unsigned int i = 0u; i < count; ++i) {
        in->travel_mm += ds;
        tick(core, in);
    }
}

static void synthetic_hump(float height)
{
    imu_core_t core;
    imu_core_input_t in;
    const unsigned int side_samples = 95u;
    const float p = asinf(height / (float)side_samples);

    calibrated(&core, &in);
    flat_motion(&core, &in, 200u, 1.0f);
    imu_core_mark(&core);
    assert(core.output.height_mm == 0.0f && core.output.peak_mm == 0.0f);
    assert(core.output.distance_mm == 0.0f);
    for (unsigned int i = 0u; i < side_samples; ++i) {
        in.accel_mg[1] = 80.0f + 1000.0f * sinf(p);
        in.accel_mg[2] = 375.0f + 1000.0f * cosf(p);
        flat_motion(&core, &in, 1u, 1.0f);
    }
    in.accel_mg[1] = 80.0f;
    in.accel_mg[2] = 1375.0f;
    flat_motion(&core, &in, 5u, 1.0f);
    for (unsigned int i = 0u; i < side_samples; ++i) {
        in.accel_mg[1] = 80.0f - 1000.0f * sinf(p);
        in.accel_mg[2] = 375.0f + 1000.0f * cosf(p);
        flat_motion(&core, &in, 1u, 1.0f);
    }
    in.accel_mg[1] = 80.0f;
    in.accel_mg[2] = 1375.0f;
    flat_motion(&core, &in, 200u, 1.0f);
    printf("synthetic hump %.0f mm: peak %.3f return %.3f mm\n",
           (double)height, (double)core.output.peak_mm,
           (double)core.output.height_mm);
    assert(fabsf(core.output.peak_mm - height) < 0.12f * height);
    assert(fabsf(core.output.height_mm) < 0.1f);
    assert(fabsf(core.output.distance_mm - 395.0f) < 0.01f);
    assert(core.output.calibrations == 1u);
    const float peak = core.output.peak_mm;

    in.accel_valid = false;
    flat_motion(&core, &in, 50u, 1.0f);
    assert(core.output.peak_mm == peak);
    imu_core_mark(&core);
    assert(core.output.height_mm == 0.0f && core.output.peak_mm == 0.0f);
    assert(core.output.calibrated && core.output.calibrations == 1u);
}

static void flat_start_stop(float top_speed, float direction)
{
    imu_core_t core;
    imu_core_input_t in;
    float actual_travel = 0.0f;
    float speed = 0.0f;
    float max_abs_height = 0.0f;
    const unsigned int ramp = (unsigned int)(top_speed / 5.0f);

    calibrated(&core, &in);
    for (unsigned int i = 0u; i < 2u * ramp + 190u; ++i) {
        const float a = i < ramp ? direction * 500.0f :
            i < ramp + 100u ? 0.0f :
            i < 2u * ramp + 100u ? -direction * 500.0f : 0.0f;

        actual_travel += speed * 0.01f + 0.5f * a * 0.0001f;
        speed += a * 0.01f;
        /* Quantize both encoders to actual configured count resolution. */
        in.travel_mm = roundf(actual_travel * 12.414f) / 12.414f;
        in.accel_mg[1] = 80.0f + a / 9.80665f;
        in.wheels_still = i >= 2u * ramp + 100u;
        tick(&core, &in);
        max_abs_height = fmaxf(max_abs_height, fabsf(core.output.height_mm));
    }
    printf("flat %.0f mm/s dir %.0f: peak %.3f max_abs %.3f "
           "return %.3f mm\n", (double)top_speed, (double)direction,
           (double)core.output.peak_mm, (double)max_abs_height,
           (double)core.output.height_mm);
    /* Numerical regression bounds, NOT a claimed physical accuracy.
     * Record the residual; do not hide it with a ground-height reset. */
    const float expected_peak = top_speed == 100.0f ? 0.405f : 1.590f;
    const float expected_return = top_speed == 100.0f ? -0.431f : -7.994f;

    assert(isfinite(core.output.height_mm));
    assert(fabsf(core.output.peak_mm - expected_peak) < 0.03f);
    assert(fabsf(core.output.height_mm - expected_return) < 0.03f);
    assert(fabsf(max_abs_height + expected_return) < 0.03f);
    assert(core.output.calibrations == 1u);
    assert(core.output.calibrated);
}

static void stationary_slope_refresh(void)
{
    imu_core_t core;
    imu_core_input_t in;
    const float p = 0.104719755f; /* Six degrees, not a detection threshold. */

    calibrated(&core, &in);
    in.accel_mg[1] = 80.0f + sinf(p) * 1000.0f;
    in.accel_mg[2] = 375.0f + cosf(p) * 1000.0f;
    flat_motion(&core, &in, 200u, 1.0f);
    assert(core.output.height_mm > 10.0f);
    in.wheels_still = true;
    ticks(&core, &in, 101u);
    /* A stopped slope must NOT become a new level reference. */
    assert(core.output.calibrations == 1u);
    assert(core.output.refresh_rejects == 1u);
    assert(core.output.height_mm > 10.0f);
    assert(core.output.pitch_ddeg >= 59);
    assert(core.output.bias_mg[1] == 80.0f);
}

static void flat_turn(float v, float omega)
{
    imu_core_t core;
    imu_core_input_t in;

    calibrated(&core, &in);
    in.wheels_still = false;
    for (unsigned int i = 0u; i < 300u; ++i) {
        in.travel_mm += v * 0.01f;
        in.yaw_rad += omega * 0.01f;
        in.accel_mg[0] = -20.0f - v * omega / 9.80665f;
        in.accel_mg[1] = 80.0f - 60.0f * omega * omega / 9.80665f;
        tick(&core, &in);
    }
    assert(fabsf(core.output.yaw_rate_rad_s - omega) < 0.001f);
    assert(fabsf(core.output.lateral_fake_mm_s2 + v * omega) < 0.1f);
    assert(abs(core.output.corrected_pitch_ddeg) <= 1);
    assert(abs(core.output.corrected_roll_ddeg) <= 1);
    if (v > 0.0f) {
        assert(core.output.roll_ddeg * (omega > 0.0f ? 1 : -1) < 0);
    }
}

static void angular_acceleration(void)
{
    imu_core_t core;
    imu_core_input_t in;

    calibrated(&core, &in);
    in.wheels_still = false;
    for (unsigned int i = 1u; i <= 300u; ++i) {
        const float t = (float)i * 0.01f;
        const float omega = 0.5f * t;

        in.yaw_rad = 0.25f * t * t;
        in.accel_mg[0] = -20.0f - 60.0f * 0.5f / 9.80665f;
        in.accel_mg[1] = 80.0f - 60.0f * omega * omega / 9.80665f;
        tick(&core, &in);
    }
    assert(fabsf(core.output.yaw_accel_rad_s2 - 0.5f) < 0.01f);
    assert(abs(core.output.corrected_roll_ddeg) <= 1);
    assert(abs(core.output.corrected_pitch_ddeg) <= 2);
}

static void support_and_refresh_guard(void)
{
    imu_core_t core;
    imu_core_input_t in;
    const float p = asinf(-10.0f / (2.0f * 80.0f));
    const float r = asinf(10.0f / 111.0f);

    calibrated(&core, &in);
    in.accel_mg[0] = -20.0f + 1000.0f * sinf(r) * cosf(p);
    in.accel_mg[1] = 80.0f + 1000.0f * sinf(p);
    in.accel_mg[2] = 375.0f + 1000.0f * cosf(r) * cosf(p);
    ticks(&core, &in, 300u);
    assert(core.output.calibrations == 1u);
    assert(core.output.refresh_rejects >= 2u);
    assert(core.output.support_peak_eligible);
    assert(fabsf(core.output.right_smoothed_mm - 10.0f) < 0.01f);
    assert(fabsf(core.output.left_mm) < 0.01f);
    assert(fabsf(core.output.right_peak_mm - 10.0f) < 0.02f);
    imu_core_mark(&core);
    assert(core.output.right_peak_mm == 0.0f);
    ticks(&core, &in, 9u);
    assert(!core.output.support_peak_eligible);
    tick(&core, &in);
    assert(core.output.support_peak_eligible);
    /* Both wheels lifted is outside the RIGHT-only course hypothesis. */
    in.accel_mg[0] = -20.0f;
    in.accel_mg[1] = 80.0f - 1000.0f * 20.0f / 80.0f;
    in.accel_mg[2] = 375.0f + 1000.0f * sqrtf(1.0f - 0.25f * 0.25f);
    ticks(&core, &in, 200u);
    imu_core_mark(&core);
    ticks(&core, &in, 30u);
    assert(core.output.left_mm > 19.9f && core.output.right_mm > 19.9f);
    assert(!core.output.support_peak_eligible);
    assert(core.output.right_peak_mm == 0.0f);
    assert(core.output.left_peak_mm > 19.9f);
    assert(core.output.support_peak_rejects > 0u);
    /* Faults break the averaging window but preserve the peaks. */
    in.accel_valid = false;
    tick(&core, &in);
    assert(core.support_count == 0u);
    assert(core.output.left_peak_mm > 19.9f);
}

static void support_single_sample(void)
{
    imu_core_t core;
    imu_core_input_t in;
    const float p = asinf(-20.0f / 160.0f);
    const float r = asinf(20.0f / 111.0f);

    calibrated(&core, &in);
    in.wheels_still = false;
    imu_core_mark(&core);
    ticks(&core, &in, 10u);
    /* Isolate the boxcar from the separate angle low-pass. */
    core.have_filter = false;
    in.accel_mg[0] = -20.0f + 1000.0f * sinf(r) * cosf(p);
    in.accel_mg[1] = 80.0f + 1000.0f * sinf(p);
    in.accel_mg[2] = 375.0f + 1000.0f * cosf(r) * cosf(p);
    tick(&core, &in);
    assert(core.output.support_peak_eligible);
    assert(core.output.right_mm > 19.9f);
    assert(fabsf(core.output.right_smoothed_mm - 2.0f) < 0.01f);
    assert(core.output.right_peak_mm < 2.01f);
}

static void support_peaks_survive_refresh(void)
{
    imu_core_t core;
    imu_core_input_t in;

    calibrated(&core, &in);
    /* Exercise persistence independently of the static geometry tests. */
    core.output.left_peak_mm = 20.0f;
    core.output.right_peak_mm = 10.0f;
    core.output.caster_peak_mm = 15.0f;
    core.output.height_mm = 7.0f;
    core.output.peak_mm = 9.0f;
    ticks(&core, &in, 202u);
    assert(core.output.calibrations == 3u);
    assert(core.output.height_mm == 0.0f && core.output.peak_mm == 0.0f);
    assert(core.output.left_peak_mm == 20.0f);
    assert(core.output.right_peak_mm == 10.0f);
    assert(core.output.caster_peak_mm == 15.0f);
    imu_core_mark(&core);
    assert(core.output.left_peak_mm == 0.0f);
    assert(core.output.right_peak_mm == 0.0f);
    assert(core.output.caster_peak_mm == 0.0f);
}

static void duplicates_gaps_and_wrap(void)
{
    imu_core_t core;
    imu_core_input_t in;

    calibrated(&core, &in);
    flat_motion(&core, &in, 200u, 1.0f);
    const uint32_t integrated = core.output.integrated_samples;
    const float distance = core.output.distance_mm;
    const float acceleration = core.output.forward_accel_mm_s2;

    in.travel_ms -= 10u; /* Same encoder observation, later IMU sample. */
    imu_core_step(&core, &in);
    assert(core.output.integrated_samples == integrated);
    assert(core.output.distance_mm == distance);
    assert(core.output.forward_accel_mm_s2 == acceleration);
    in.now_ms += 10u;
    in.travel_ms += 20u;
    in.travel_mm += 2.0f;
    imu_core_step(&core, &in);
    assert(core.output.integrated_samples == integrated + 1u);
    const float height = core.output.height_mm;
    const float peak = core.output.peak_mm;

    in.now_ms += 1000u;
    in.travel_ms += 1000u;
    in.travel_mm += 100.0f;
    imu_core_step(&core, &in);
    assert(core.output.height_mm == height && core.output.peak_mm == peak);
    assert(core.output.gap_resets > 0u);
    assert(core.output.distance_mm > distance + 99.0f);
    /* No reset of height on bad timing or a missing motion source. */
    in.now_ms += 10u;
    in.motion_valid = false;
    imu_core_step(&core, &in);
    assert(core.output.height_mm == height && core.output.peak_mm == peak);
    imu_core_init(&core);
    in = level();
    in.now_ms = UINT32_MAX - 499u;
    in.travel_ms = UINT32_MAX - 499u;
    ticks(&core, &in, 101u);
    assert(core.output.calibrated && core.output.gap_resets == 0u);
    assert(in.now_ms < 1000u);
    /* Zero/rolled-back time must not integrate or complete stillness. */
    in.wheels_still = false;
    in.now_ms = core.previous_ms;
    in.travel_ms = core.previous_travel_ms;
    in.travel_mm = core.previous_travel + 1.0f;
    const uint32_t before = core.output.integrated_samples;

    imu_core_step(&core, &in);
    assert(core.output.integrated_samples == before);
    assert(core.output.gap_resets > 0u);
}

int main(void)
{
    calibration_time_and_refresh();
    calibration_rejects();
    motion_and_spin_inhibit();
    missing_and_invalid_data();
    angles_and_filter();
    encoder_acceleration(500.0f, 0.0f);
    encoder_acceleration(-500.0f, 0.0f);
    encoder_acceleration(-500.0f, 1000.0f);
    synthetic_hump(5.0f);
    synthetic_hump(10.0f);
    synthetic_hump(20.0f);
    flat_start_stop(100.0f, 1.0f);
    flat_start_stop(100.0f, -1.0f);
    flat_start_stop(460.0f, 1.0f);
    flat_start_stop(460.0f, -1.0f);
    stationary_slope_refresh();
    flat_turn(100.0f, 1.0f);
    flat_turn(100.0f, -1.0f);
    flat_turn(-100.0f, 1.0f);
    flat_turn(-100.0f, -1.0f);
    flat_turn(0.0f, 1.0f);
    flat_turn(0.0f, -1.0f);
    angular_acceleration();
    support_and_refresh_guard();
    support_single_sample();
    support_peaks_survive_refresh();
    duplicates_gaps_and_wrap();
    return 0;
}
