/* Pure bounded TEST job. The caller alone uses the motion-owner API. */
#include "imu_drive.h"
#include <math.h>
#include <stddef.h>

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
