#define main ordinary_imu_service_main
#define imu_probe_read unused_physical_read
#define imu_probe_init unused_physical_init
#include "test_imu_service.c"
#undef main
#undef imu_probe_read
#undef imu_probe_init
#include <stdlib.h>
#include "../app_program/imu_safety_bench/stats.h"

void imu_probe_init(imu_probe_t *probe)
{
    (void)probe;
    abort(); /* replay must not call physical probe init */
}
bool imu_probe_read(imu_probe_t *probe, uint32_t now, imu_units_t *raw)
{
    (void)probe; (void)now; (void)raw;
    abort(); /* replay must not call physical probe read */
}
int main(void)
{
    assert(imu_service_init() == E_OK);
    assert(imu_service_start() == E_OK);
    for (unsigned int i = 0u; i < 7000u; ++i) {
        run(1u);
        imu_safety_report_t safety;
        assert(imu_safety_try_read(&safety) == E_OK);
        if (safety.value.clear_ms >= IMU_SAFETY_CLEAR_MS &&
            safety.value.latched_trigger != IMU_SAFETY_NONE) {
            assert(imu_safety_clear_request() != 0u);
        }
    }
    imu_bench_stats_t stats;
    assert(imu_bench_stats_try_read(&stats));
    uint32_t samples = 0u;
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        assert(stats.phase[i].samples > 0u);
        samples += stats.phase[i].samples;
    }
    assert(samples == 7000u);
    assert(stats.phase[1].triggers & (1u << IMU_SAFETY_OVERTURN));
    assert(stats.phase[3].triggers & (1u << IMU_SAFETY_LIFT));
    imu_snapshot_t snapshot;
    assert(imu_snapshot_try_read(&snapshot) == E_OK);
    assert(snapshot.probe.reads == 7000u && snapshot.probe.last_us == 0u);
    return 0;
}
