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

#if IMU_SAFETY_BENCH_ROBOT
/* The ROBOT service reads the Vehicle run context; idle here. */
uint32_t vehicle_run_epoch(void) { return 0u; }
uint32_t vehicle_run_start_ms(void) { return 0u; }
uint32_t vehicle_line_run_epoch(void) { return 0u; }
#endif

/* Section times are nested in compute; the remainder makes them exact. */
static void check_split(const imu_bench_stats_t *stats)
{
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        const imu_bench_phase_t *p = &stats->phase[i];
        if (p->samples == 0u) {
            continue;
        }
        uint64_t sum = 0u;
        for (unsigned int s = 0u; s < IMU_BENCH_SECTIONS; ++s) {
            const imu_bench_split_t *x = &p->split[s];
            const uint64_t mean = x->total_us / p->samples;
            assert(x->min_us <= mean && mean <= x->max_us);
            if (s != IMU_BENCH_REST) {
                assert(x->min_us > 0u); /* every tick ran the section */
            }
            sum += x->total_us;
        }
        assert(sum == p->compute_total_us);
        assert(p->compute_min_us <= p->compute_max_us);
    }
}

static void hints(void)
{
    /* Idle window first, then start. Vehicle work always means wait. */
    assert(imu_bench_hint(IMU_SAFETY_NONE, 0u, false, 69999u) ==
           IMU_BENCH_HINT_IDLE_WINDOW);
    assert(imu_bench_hint(IMU_SAFETY_NONE, 0u, false, 70000u) ==
           IMU_BENCH_HINT_PRESS_START);
    assert(imu_bench_hint(IMU_SAFETY_NONE, 0u, true, 90000u) ==
           IMU_BENCH_HINT_WAIT);
    assert(imu_bench_hint(IMU_SAFETY_OVERTURN, IMU_SAFETY_CLEAR_MS - 1u,
                          false, 90000u) == IMU_BENCH_HINT_WAIT_CLEAR);
    assert(imu_bench_hint(IMU_SAFETY_LIFT, IMU_SAFETY_CLEAR_MS, false,
                          90000u) == IMU_BENCH_HINT_PRESS_RECOVER);
    assert(imu_bench_hint(IMU_SAFETY_LIFT, IMU_SAFETY_CLEAR_MS, true,
                          90000u) == IMU_BENCH_HINT_WAIT);
}

static void ticks(unsigned int count)
{
    for (unsigned int i = 0u; i < count; ++i) {
        run(1u);
#if !IMU_SAFETY_BENCH_ROBOT
        /* TEST only: the reporter is the sole requester. In ROBOT the
         * Vehicle owns recovery, so this test never clears there. */
        imu_safety_report_t safety;
        assert(imu_safety_try_read(&safety) == E_OK);
        if (safety.value.clear_ms >= IMU_SAFETY_CLEAR_MS &&
            safety.value.latched_trigger != IMU_SAFETY_NONE) {
            assert(imu_safety_clear_request() != 0u);
        }
#endif
    }
}

int main(void)
{
    hints();
    assert(imu_service_init() == E_OK);
    assert(imu_service_start() == E_OK);
    imu_bench_stats_t stats;
#if IMU_SAFETY_BENCH_ROBOT
    /* Upright with no calibration start: phase 0 only, no trigger. */
    epoch = 0u;
    ticks(3000u);
    assert(imu_bench_stats_try_read(&stats));
    assert(stats.robot && stats.episodes == 0u);
    assert(stats.phase[0].samples == 3000u && stats.phase[1].samples == 0u);
    assert(stats.phase[0].triggers == 1u);
    /* GP21 calibration start: OVERTURN within the 6 s calibration. */
    epoch = 1u;
    ticks(600u);
    assert(imu_bench_stats_try_read(&stats));
    assert(stats.episodes == 1u);
    assert(stats.phase[1].triggers & (1u << IMU_SAFETY_OVERTURN));
    imu_safety_report_t safety;
    assert(imu_safety_try_read(&safety) == E_OK);
    assert(safety.value.latched_trigger == IMU_SAFETY_OVERTURN);
    /* The stream returns upright, but the latch stays until recovery. */
    epoch = 2u;
    ticks(1000u);
    assert(imu_bench_stats_try_read(&stats));
    assert(stats.phase[2].samples > 0u);
    assert(imu_safety_try_read(&safety) == E_OK);
    assert(safety.value.latched_trigger == IMU_SAFETY_OVERTURN);
    check_split(&stats);
#else
    ticks(7000u);
    assert(imu_bench_stats_try_read(&stats));
    assert(!stats.robot);
    uint32_t samples = 0u;
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        assert(stats.phase[i].samples > 0u);
        samples += stats.phase[i].samples;
    }
    assert(samples == 7000u);
    assert(stats.phase[1].triggers & (1u << IMU_SAFETY_OVERTURN));
    assert(stats.phase[3].triggers & (1u << IMU_SAFETY_LIFT));
    check_split(&stats);
#endif
    imu_snapshot_t snapshot;
    assert(imu_snapshot_try_read(&snapshot) == E_OK);
    assert(snapshot.probe.last_us == 0u);
    return 0;
}
