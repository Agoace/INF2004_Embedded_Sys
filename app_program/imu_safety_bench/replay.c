#include "replay.h"
#if IMU_SAFETY_BENCH
#if APP_BENCH || (IMU_SAFETY_BENCH_ROBOT ? APP_TEST : !APP_TEST)
#error "IMU_SAFETY_BENCH: TEST stream needs TEST=1, ROBOT stream TEST=0"
#endif
#include <string.h>

/* Near-1 g rotations, not linear interpolation through a low-g vector. */
static const int16_t pose[9][2] = {
    {0, 1000}, {174, 985}, {342, 940}, {500, 866}, {643, 766},
    {766, 643}, {866, 500}, {940, 342}, {996, 87}};

static void emit(imu_probe_t *probe, imu_units_t *raw, uint32_t ms,
                 unsigned int tilt_index, int32_t z_override)
{
    const int32_t jitter = (int32_t)((ms / 10u) % 7u) - 3;
    int32_t accel[3] = {pose[tilt_index][0] + jitter, -jitter,
                        pose[tilt_index][1]};
    if (z_override != 0) { accel[2] = z_override; }
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
}

unsigned int imu_replay_read(imu_probe_t *probe, imu_units_t *raw,
                            uint32_t elapsed_ms)
{
    const uint32_t at = elapsed_ms % 40000u;
    const unsigned int phase = at < 12000u ? 0u : at < 20000u ? 1u :
                               at < 28000u ? 2u : at < 32000u ? 3u : 4u;
    unsigned int index = 0u;
    if (phase == 1u || (phase == 2u && at < 20300u)) {
        index = phase == 1u ? (at - 12000u) / 30u : 8u - (at - 20000u) / 40u;
        if (index > 8u) { index = 8u; }
    }
    /* A brief 0.3 g lift/fall transient, not four seconds of free fall. */
    const int32_t z = (phase == 3u && at < 28200u) ? 300 : 0;
    emit(probe, raw, elapsed_ms, index, z);
    return phase;
}

void imu_replay_robot_init(imu_replay_robot_t *r)
{
    memset(r, 0, sizeof(*r));
}

uint32_t imu_replay_robot_episodes(const imu_replay_robot_t *r)
{
    return r->episodes;
}

/* Each start adds 1 then 2 to the epoch; count starts without overflow. */
static uint32_t starts_of(uint32_t epoch)
{
    return (epoch >> 1u) + (epoch & 1u);
}

unsigned int imu_replay_robot_read(imu_replay_robot_t *r, imu_probe_t *probe,
                                   imu_units_t *raw, uint32_t now_ms,
                                   uint32_t calibration_epoch)
{
    const uint32_t starts = starts_of(calibration_epoch);
    if (!r->begun) {
        r->begun = true;
        r->starts = starts;
    }
    if (starts != r->starts) {
        r->starts = starts;
        if (!r->scheduled && !r->running) {
            r->scheduled = true;
            r->lift = (r->episodes & 1u) != 0u;
            r->at_ms = now_ms + IMU_REPLAY_ROBOT_DELAY_MS;
            ++r->episodes;
        }
    }
    if (r->scheduled && (int32_t)(now_ms - r->at_ms) >= 0) {
        r->scheduled = false;
        r->running = true;
    }
    unsigned int phase = r->after;
    unsigned int index = 0u;
    int32_t z = 0;
    if (r->running) {
        const uint32_t e = now_ms - r->at_ms;
        if (r->lift) {
            phase = 3u;
            if (e < IMU_REPLAY_LIFT_MS) {
                z = 300;
            } else {
                r->running = false;
                r->after = 4u;
                phase = 4u;
            }
        } else if (e < IMU_REPLAY_TILT_HOLD_MS) {
            phase = 1u;
            index = e / 30u > 8u ? 8u : e / 30u;
        } else if (e < IMU_REPLAY_TILT_HOLD_MS + IMU_REPLAY_TILT_RETURN_MS) {
            phase = 2u;
            const uint32_t back = (e - IMU_REPLAY_TILT_HOLD_MS) / 40u;
            index = back > 8u ? 0u : 8u - back;
        } else {
            r->running = false;
            r->after = 2u;
            phase = 2u;
        }
    }
    emit(probe, raw, now_ms, index, z);
    return phase;
}
#endif
