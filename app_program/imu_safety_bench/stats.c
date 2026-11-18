#include "stats.h"
#if IMU_SAFETY_BENCH
#if !APP_TEST || APP_BENCH
#error "IMU_SAFETY_BENCH requires TEST=1 BENCH=0"
#endif
#include "../platform/app_board.h"
#include <string.h>

static imu_bench_stats_t stats;
static APP_BOARD(imu_bench_stats_t) board;
static uint32_t origin_ms;
static unsigned int phase;
static unsigned int previous_phase;
static bool begun;
static bool previous;

void imu_bench_probe_init(imu_probe_t *probe)
{
    memset(probe, 0, sizeof(*probe));
    memset(&stats, 0, sizeof(stats));
    begun = false;
    previous = false;
}

bool imu_bench_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *raw)
{
    if (!begun) { origin_ms = now_ms; begun = true; }
    phase = imu_replay_read(probe, raw, now_ms - origin_ms);
    probe->ready = true;
    ++probe->reads;
    ++probe->fifo_cycles;
    ++probe->fifo_batches;
    probe->fifo_samples += 4u;
    probe->fifo_min = 4u;
    probe->fifo_max = 4u;
    /* No physical bus transaction: do not invent a bus cost. */
    probe->last_us = 0u;
    return true;
}

void imu_bench_record(const imu_snapshot_t *snapshot)
{
    imu_bench_phase_t *current = &stats.phase[phase];
    ++current->samples;
    current->compute_total_us += snapshot->compute_last_us;
    if (snapshot->compute_last_us > current->compute_max_us) {
        current->compute_max_us = snapshot->compute_last_us;
    }
    imu_safety_report_t safety;
    if (imu_safety_try_read(&safety) >= 0) {
        current->triggers |= 1u << safety.value.latched_trigger;
    }
    /* The production board publishes the preceding tick's work value. */
    if (previous) {
        imu_bench_phase_t *prior = &stats.phase[previous_phase];
        ++prior->work_samples;
        prior->work_total_us += snapshot->work_last_us;
        if (snapshot->work_last_us > prior->work_max_us) {
            prior->work_max_us = snapshot->work_last_us;
        }
        if (snapshot->work_last_us >= 10000u) { ++prior->overruns; }
    }
    previous = true;
    previous_phase = phase;
    app_board_publish(&board.sequence, board.words, &stats, sizeof(stats));
}

bool imu_bench_stats_try_read(imu_bench_stats_t *out)
{
    imu_bench_stats_t copy;
    if (!app_board_try_read(&board.sequence, board.words, &copy, sizeof(copy))) {
        return false;
    }
    *out = copy;
    return true;
}
#endif
