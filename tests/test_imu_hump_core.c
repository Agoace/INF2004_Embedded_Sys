/* imu_core feeds the hump detector: straight hump, curve fake roll, MARK
 * and invalid samples. Built for both IMU_HUMP_SOURCE values. Synthetic. */
#include "../app_program/subsystem4/imu_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void tick(imu_core_t *core, imu_core_input_t *in)
{
    imu_core_step(core, in);
    in->now_ms += 10u;
    in->travel_ms += 10u;
}

static void calibrated(imu_core_t *core, imu_core_input_t *in)
{
    imu_core_init(core);
    *in = (imu_core_input_t){.accel_mg = {-20.0f, 80.0f, 1375.0f},
                             .accel_valid = true, .motion_valid = true,
                             .wheels_still = true,
                             .calibration_allowed = true};
    for (unsigned int i = 0u; i < 101u; ++i) {
        tick(core, in);
    }
    assert(core->output.calibrated);
    in->wheels_still = false;
    in->calibration_allowed = false;
}

/* Drive at v (mm/s) and omega (rad/s) for n ticks with right-wheel height
 * h(x): roll from the hump plus the true centripetal X acceleration. */
static void drive(imu_core_t *core, imu_core_input_t *in, unsigned int n,
                  float v, float omega, float peak)
{
    const float start = in->travel_mm;

    for (unsigned int i = 0u; i < n; ++i) {
        const float x = in->travel_mm - start;
        const float h = x < 95.0f ? peak * x / 95.0f :
            fmaxf(0.0f, peak * (195.0f - x) / 100.0f);
        const float r = asinf(h / IMU_TRACK_MM);

        in->travel_mm += v * 0.01f;
        in->yaw_rad += omega * 0.01f;
        in->accel_mg[0] = -20.0f + 1000.0f * sinf(r) - v * omega / 9.80665f;
        in->accel_mg[1] = 80.0f - 60.0f * omega * omega / 9.80665f;
        in->accel_mg[2] = 375.0f + 1000.0f * cosf(r);
        tick(core, in);
    }
}

int main(void)
{
    imu_core_t core;
    imu_core_input_t in;

    /* Straight 10 mm RIGHT hump at 146 mm/s during run 1. */
    calibrated(&core, &in);
    in.run_epoch = 1u;
    drive(&core, &in, 200u, 146.0f, 0.0f, 0.0f);
    drive(&core, &in, 400u, 146.0f, 0.0f, 10.0f);
    assert(core.output.hump.count == 1u && !core.output.hump.max_left);
    assert(core.output.hump.max_peak_mm > 8.8f);
    assert(core.output.hump.max_peak_mm <= 10.0f);
    /* MARK resets the step 2 peaks only. */
    imu_core_mark(&core);
    assert(core.output.peak_mm == 0.0f && core.output.hump.count == 1u);
    /* A 30 mg curve (v*omega, yaw below the gate) with NO hump: the
     * uncorrected fake roll is 3.4 mm, above ON. */
    in.run_epoch = 3u;
    drive(&core, &in, 150u, 300.0f, 1.0f, 0.0f);
    printf("fixture curve30mg source %d: mean %.3f mm count %u\n",
           IMU_HUMP_SOURCE, (double)core.output.hump.smoothed_mm,
           (unsigned int)core.output.hump.count);
    assert(fabsf(core.output.lateral_fake_mm_s2 / 9.80665f + 30.6f) < 0.5f);
#if IMU_HUMP_SOURCE == IMU_HUMP_SOURCE_FILTERED
    drive(&core, &in, 100u, 300.0f, 0.0f, 0.0f);
    assert(core.output.hump.count == 1u); /* False event: why CORRECTED. */
#else
    assert(core.output.hump.count == 0u && core.output.hump.long_rejects == 0u);
    assert(fabsf(core.output.hump.smoothed_mm) < 0.5f);
    /* A hump on the same 30 mg curve still counts with the CORRECTED roll. */
    drive(&core, &in, 200u, 300.0f, 1.0f, 10.0f);
    printf("fixture curve30mg hump10 v300: count %u peak %.3f gated %u\n",
           (unsigned int)core.output.hump.count,
           (double)core.output.hump.max_peak_mm,
           (unsigned int)core.output.hump.gated_starts);
    /* 300 mm/s attenuates the 100 ms mean (pure fixture 7.9 mm). */
    assert(core.output.hump.count == 1u && core.output.hump.max_peak_mm > 7.0f);
#endif
    /* Invalid accel inside a run: fault flag; the run end still applies. */
    in.accel_valid = false;
    tick(&core, &in);
    assert((core.output.hump.flags & IMU_HUMP_FLAG_FAULT) != 0u);
    in.run_epoch = 4u;
    tick(&core, &in);
    assert(!core.output.hump.running);
    puts("imu hump core fixtures passed");
    return 0;
}
