/* Synthetic bench stream must exercise the unmodified detector paths. */
#include "../app_program/imu_safety_bench/replay.h"
#include "../app_program/subsystem4/imu_safety.h"
#include <assert.h>
#include <math.h>
int main(void)
{
    imu_safety_t safety;
    imu_safety_init(&safety);
    bool lift = false, tilt = false;
    for (uint32_t ms = 0u; ms < 70000u; ms += 10u) {
        imu_probe_t probe = {0};
        imu_units_t raw = {0};
        const unsigned int phase = imu_replay_read(&probe, &raw, ms);
        assert(phase < IMU_REPLAY_PHASES && !probe.no_sample);
        assert(probe.fifo_last_samples == 4u && !raw.mag_valid);
        imu_safety_input_t input = {.now_ms=ms,.accel_valid=true,
            .calibrated=true,.wheels_still=true};
        for (unsigned int i=0u;i<3u;++i) {
            input.accel_mg[i]=probe.mean_accel_mg[i];
            assert(isfinite(input.accel_mg[i]));
        }
        imu_safety_step(&safety, &input);
        lift |= safety.output.latched_trigger==IMU_SAFETY_LIFT;
        tilt |= safety.output.latched_trigger==IMU_SAFETY_OVERTURN;
        if (safety.output.clear_ms>=IMU_SAFETY_CLEAR_MS) {
            (void)imu_safety_clear(&safety);
        }
    }
    assert(lift && tilt);
    imu_probe_t a={0},b={0};imu_units_t x={0},y={0};
    assert(imu_replay_read(&a,&x,1u)==imu_replay_read(&b,&y,40001u));
    assert(x.accel_mg[2]==y.accel_mg[2]);
    return 0;
}
