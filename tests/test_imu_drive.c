#include <assert.h>
#include <math.h>
#include <stddef.h>
#include "../app_program/subsystem4/imu_drive.h"

int main(void)
{
    imu_drive_t job = {0};

    imu_drive_cancel(NULL);
    assert(imu_drive_step(NULL, 0u, 0.0f, true, false) == 0);
    assert(imu_drive_step(&job, 0u, 200.0f, false, false) == 0);
    assert(imu_drive_step(&job, 1u, 200.0f, true, false) == 100);
    assert(job.active);
    assert(imu_drive_step(&job, 6000u, 799.0f, false, false) == 100);
    assert(imu_drive_step(&job, 6001u, 800.0f, false, false) == 0);
    assert(!job.active && job.reason == IMU_DRIVE_DISTANCE);
    assert(imu_drive_step(&job, 7000u, 800.0f, true, false) == 100);
    assert(imu_drive_step(&job, 7020u, 801.0f, true, false) == 0);
    assert(!job.active && job.reason == IMU_DRIVE_BUTTON);
    assert(imu_drive_step(&job, 8000u, 0.0f, true, true) == 0);
    assert(!job.active && job.reason == IMU_DRIVE_FAULT);
    assert(imu_drive_step(&job, 9000u, 0.0f, true, false) == 100);
    assert(imu_drive_step(&job, 19000u, 0.0f, false, false) == 0);
    assert(!job.active && job.reason == IMU_DRIVE_TIMEOUT);
    assert(imu_drive_step(&job, UINT32_MAX - 999u, 0.0f, true, false) == 100);
    assert(imu_drive_step(&job, 8999u, 20.0f, false, false) == 100);
    assert(imu_drive_step(&job, 9000u, 20.0f, false, false) == 0);
    assert(job.reason == IMU_DRIVE_TIMEOUT);
    assert(imu_drive_step(&job, 10000u, NAN, true, false) == 0);
    assert(!job.active);
    assert(imu_drive_step(&job, 10000u, 20.0f, true, false) == 100);
    assert(imu_drive_step(&job, 10020u, INFINITY, false, false) == 0);
    assert(!job.active);
    imu_drive_stats_t stats;
    float std[2];

    imu_drive_stats_reset(&stats);
    imu_drive_stats_add(&stats, 1, -2);
    imu_drive_stats_add(&stats, 2, 0);
    imu_drive_stats_add(&stats, 3, 2);
    imu_drive_stats_add(&stats, NAN, 0);
    imu_drive_stats_std(&stats, std);
    assert(stats.samples == 3u && stats.mean[0] == 2);
    assert(fabsf(std[0] - 1) < 0.0001f);
    assert(fabsf(std[1] - 2) < 0.0001f);
    imu_drive_stats_reset(&stats);
    imu_drive_stats_std(&stats, std);
    assert(stats.samples == 0u && std[0] == 0 && std[1] == 0);
    return 0;
}
