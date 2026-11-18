/* Pure bounded TEST job. The caller alone uses the motion-owner API. */
#include "imu_drive.h"
#include <math.h>
#include <stddef.h>

void imu_drive_stats_reset(imu_drive_stats_t *s)
{
    if (s != NULL) {
        *s = (imu_drive_stats_t){0};
    }
}

void imu_drive_stats_add(imu_drive_stats_t *s, float pitch, float roll)
{
    const float values[2] = {pitch, roll};

    if (s == NULL || !isfinite(pitch) || !isfinite(roll) ||
        s->samples == UINT32_MAX) {
        return;
    }
    ++s->samples;
    for (unsigned int i = 0u; i < 2u; ++i) {
        const float delta = values[i] - s->mean[i];

        s->mean[i] += delta / (float)s->samples;
        s->m2[i] += delta * (values[i] - s->mean[i]);
    }
}

void imu_drive_stats_std(const imu_drive_stats_t *s, float out[2])
{
    if (s == NULL || out == NULL) {
        return;
    }
    for (unsigned int i = 0u; i < 2u; ++i) {
        out[i] = s->samples > 1u ?
            sqrtf(fmaxf(s->m2[i], 0.0f) / (float)(s->samples - 1u)) : 0.0f;
    }
}

void imu_drive_cancel(imu_drive_t *job)
{
    if (job != NULL) {
        job->active = false;
        job->reason = IMU_DRIVE_FAULT;
    }
}

int16_t imu_drive_step(imu_drive_t *job, uint32_t now_ms, float travel_mm,
                       bool press, bool fault)
{
    if (job == NULL) {
        return 0;
    }
    if (fault || !isfinite(travel_mm)) {
        imu_drive_cancel(job);
        return 0;
    }
    if (job->active) {
        job->distance_mm = travel_mm - job->start_mm;
        if (press) {
            job->reason = IMU_DRIVE_BUTTON;
        } else if ((uint32_t)(now_ms - job->start_ms) >=
                    IMU_DRIVE_TIMEOUT_MS) {
            job->reason = IMU_DRIVE_TIMEOUT;
        } else if (job->distance_mm >= IMU_DRIVE_DISTANCE_MM) {
            job->reason = IMU_DRIVE_DISTANCE;
        } else {
            return IMU_DRIVE_SPEED_MM_S;
        }
        job->active = false;
        return 0;
    }
    if (press) {
        job->active = true;
        job->start_ms = now_ms;
        job->start_mm = travel_mm;
        job->distance_mm = 0.0f;
        job->reason = IMU_DRIVE_RUNNING;
        return IMU_DRIVE_SPEED_MM_S;
    }
    return 0;
}
