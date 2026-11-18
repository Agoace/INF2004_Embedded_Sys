#include "replay.h"
#if IMU_SAFETY_BENCH
#if !APP_TEST || APP_BENCH
#error "IMU_SAFETY_BENCH requires TEST=1 BENCH=0"
#endif
unsigned int imu_replay_read(imu_probe_t *probe, imu_units_t *raw,
                            uint32_t elapsed_ms)
{
    const uint32_t at = elapsed_ms % 40000u;
    const unsigned int phase = at < 12000u ? 0u : at < 20000u ? 1u :
                               at < 28000u ? 2u : at < 32000u ? 3u : 4u;
    const int32_t jitter = (int32_t)((elapsed_ms / 10u) % 7u) - 3;
    int32_t accel[3] = {jitter, -jitter, 1000};
    /* Near-1 g rotations, not linear interpolation through a low-g vector. */
    static const int16_t pose[9][2] = {
        {0, 1000}, {174, 985}, {342, 940}, {500, 866}, {643, 766},
        {766, 643}, {866, 500}, {940, 342}, {996, 87}};
    if (phase == 1u || (phase == 2u && at < 20300u)) {
        unsigned int index = phase == 1u ? (at - 12000u) / 30u :
                                           8u - (at - 20000u) / 40u;
        if (index > 8u) { index = 8u; }
        accel[0] = pose[index][0] + jitter;
        accel[2] = pose[index][1];
    }
    /* A brief 0.3 g lift/fall transient, not four seconds of free fall. */
    if (phase == 3u && at < 28200u) { accel[2] = 300; }
    probe->no_sample = false;
    probe->fifo_last_samples = 4u;
    raw->mag_valid = false;
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        raw->accel_mg[axis] = accel[axis];
        probe->mean_accel_mg[axis] = (float)accel[axis];
        for (unsigned int sample = 0u; sample < 4u; ++sample) {
            probe->fifo_raw[sample][axis] =
                (int16_t)((accel[axis] / 2) * 16);
        }
    }
    return phase;
}
#endif
