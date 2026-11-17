/* Bench-only observation. Totals are wall time, not exclusive CPU time. */
#ifndef IMU_SAFETY_BENCH_STATS_H
#define IMU_SAFETY_BENCH_STATS_H
#include "replay.h"
#include "../subsystem4/imu_service.h"

typedef struct {
    uint64_t compute_total_us;
    uint64_t work_total_us;
    uint32_t samples;
    uint32_t work_samples;
    uint32_t compute_max_us;
    uint32_t work_max_us;
    uint32_t overruns;
    uint32_t triggers; /* bit (1 << imu_safety_trigger_t) observed */
} imu_bench_phase_t;
typedef struct {
    imu_bench_phase_t phase[IMU_REPLAY_PHASES];
} imu_bench_stats_t;

void imu_bench_probe_init(imu_probe_t *probe);
bool imu_bench_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *raw);
void imu_bench_record(const imu_snapshot_t *snapshot);
bool imu_bench_stats_try_read(imu_bench_stats_t *out);
void imu_safety_bench_run(void);
#endif
