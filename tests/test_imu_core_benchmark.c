/* Host throughput only. This is NOT RP2040 CPU or bus timing. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "../app_program/subsystem4/imu_service.h"

int main(void)
{
    imu_core_t core;
    imu_core_input_t input = {
        .accel_mg = {-20.0f, 80.0f, 1375.0f},
        .accel_valid = true, .motion_valid = true,
        .wheels_still = true, .calibration_allowed = true
    };
    const unsigned int samples = 1000000u;

    imu_core_init(&core);
    for (unsigned int i = 0u; i < 101u; ++i) {
        imu_core_step(&core, &input);
        input.now_ms += 10u;
        input.travel_ms += 10u;
    }
    assert(core.output.calibrated);
    input.wheels_still = false;
    input.accel_mg[1] = 184.5f;
    input.accel_mg[2] = 1369.5f;
    const clock_t before = clock();

    for (unsigned int i = 0u; i < samples; ++i) {
        input.now_ms += 10u;
        input.travel_ms += 10u;
        input.travel_mm += 1.0f;
        imu_core_step(&core, &input);
    }
    const clock_t after = clock();
    const double ns = (double)(after - before) * 1e9 /
                      (double)CLOCKS_PER_SEC / (double)samples;

    assert(before != (clock_t)-1 && after != (clock_t)-1);
    assert(isfinite(core.output.height_mm));
    assert(core.output.samples == samples + 101u);
    printf("Host sizeof: core %zu, output %zu, probe %zu, snapshot %zu B.\n",
           sizeof(imu_core_t), sizeof(imu_core_output_t),
           sizeof(imu_probe_t), sizeof(imu_snapshot_t));
    printf("Host -Os: %.1f ns/update, %u updates; "
           "checksum %.3f mm. Not RP2040 timing.\n", ns, samples,
           (double)core.output.height_mm);
    return 0;
}
