/* Pure bounded TEST capture storage. Service owns the atomic handoff. */
#ifndef IMU_CAPTURE_H
#define IMU_CAPTURE_H
#include <stdbool.h>
#include <stdint.h>
#define IMU_CAPTURE_SAMPLES 800u
#define IMU_CAPTURE_BATCHES 256u
#define IMU_CAPTURE_US 2000000u
#ifndef IMU_CAPTURE_ENABLE
#define IMU_CAPTURE_ENABLE 1
#endif

typedef enum {
    IMU_CAPTURE_RUNNING,
    IMU_CAPTURE_FULL,
    IMU_CAPTURE_TIME,
    IMU_CAPTURE_STOP,
    IMU_CAPTURE_GAP,
    IMU_CAPTURE_META_FULL
} imu_capture_reason_t;
typedef struct {
    uint32_t timestamp_us;   /* Batch read wall time, NOT each sample IRQ. */
    uint32_t first;
    uint32_t count;
} imu_capture_batch_t;
typedef struct {
    int16_t raw[IMU_CAPTURE_SAMPLES][3]; /* Exact left-aligned bus words. */
    imu_capture_batch_t batch[IMU_CAPTURE_BATCHES];
    uint32_t samples;
    uint32_t batches;
    uint32_t started_us;
    imu_capture_reason_t reason;
    bool active;
} imu_capture_t;
void imu_capture_begin(imu_capture_t *s, uint32_t now_us);
void imu_capture_finish(imu_capture_t *s, imu_capture_reason_t reason);
/* Empty batches preserve continuity; errors freeze instead of joining gaps.
 * The final batch may be cropped at 800; pipeline averages are NEVER
 * cropped. */
void imu_capture_append(imu_capture_t *s, const int16_t raw[][3],
                         unsigned int count, uint32_t now_us, bool error);
#endif
