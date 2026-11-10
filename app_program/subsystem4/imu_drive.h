/* TEST-only bounded straight-drive job, NOT terrain/navigation control. */
#ifndef IMU_DRIVE_H
#define IMU_DRIVE_H
#include <stdbool.h>
#include <stdint.h>
#define IMU_DRIVE_SPEED_MM_S 100
#define IMU_DRIVE_DISTANCE_MM 600.0f
#define IMU_DRIVE_TIMEOUT_MS 10000u

typedef enum {
    IMU_DRIVE_IDLE,
    IMU_DRIVE_RUNNING,
    IMU_DRIVE_DISTANCE,
    IMU_DRIVE_TIMEOUT,
    IMU_DRIVE_BUTTON,
    IMU_DRIVE_FAULT
} imu_drive_reason_t;

typedef struct {
    float start_mm;
    float distance_mm;
    uint32_t start_ms;
    imu_drive_reason_t reason;
    bool active;
} imu_drive_t;

/* One debounced press starts/stops. Every active call requires fresh,
 * valid sensor AND motor observations. Fault always wins over a press. */
int16_t imu_drive_step(imu_drive_t *job, uint32_t now_ms, float travel_mm,
                       bool press, bool fault);
void imu_drive_cancel(imu_drive_t *job);

/* Pure per-drive observer statistics, never a drive input. Sample std. */
typedef struct {
    uint32_t samples;
    float mean[2];           /* Corrected pitch, filtered roll, degrees. */
    float m2[2];
} imu_drive_stats_t;
void imu_drive_stats_reset(imu_drive_stats_t *stats);
void imu_drive_stats_add(imu_drive_stats_t *stats, float pitch, float roll);
void imu_drive_stats_std(const imu_drive_stats_t *stats, float out[2]);
#endif
