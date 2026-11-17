/* Forced include ONLY in an explicit IMU replay TEST service build.
 * The production service, observer, safety math and publish order stay intact. */
#ifndef IMU_SAFETY_BENCH_SERVICE_IMPL_H
#define IMU_SAFETY_BENCH_SERVICE_IMPL_H
#if !IMU_SAFETY_BENCH || !APP_TEST || APP_BENCH
#error "IMU_SAFETY_BENCH service hook is TEST-only"
#endif
#include "stats.h"
#include "../platform/app_board.h"

static inline void imu_bench_publish(uint32_t *sequence, uint32_t *words,
                                     const void *value, unsigned int size)
{
    app_board_publish(sequence, words, value, size);
    if (size == sizeof(imu_snapshot_t)) {
        imu_bench_record(value);
    }
}
#define imu_probe_init imu_bench_probe_init
#define imu_probe_read imu_bench_read
#define app_board_publish imu_bench_publish
#endif
