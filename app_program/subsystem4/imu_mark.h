/* GP20 press debounce, independent of kernel/SDK. */
#ifndef IMU_MARK_H
#define IMU_MARK_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    bool sample;
    bool stable;
    uint32_t changed_at;
} imu_mark_t;
/* Zero-init state. One event per stable press, after 30 ms; release rearms. */
bool imu_mark_update(imu_mark_t *state, bool down, uint32_t now_ms);
#endif
