#include "stats.h"
#include <stddef.h>

static const char *const hint_text[] = {
    "IDLE_WINDOW_DO_NOT_PRESS", "PRESS_GP21_ONCE_TO_START", "WAIT",
    "WAIT_CLEAR_WINDOW", "PRESS_GP21_ONCE_TO_RECOVER"};

imu_bench_hint_t imu_bench_hint(uint8_t latched, uint32_t clear_ms,
                                bool vehicle_active, uint32_t uptime_ms)
{
    if (vehicle_active) { return IMU_BENCH_HINT_WAIT; }
    if (latched != IMU_SAFETY_NONE) {
        return clear_ms >= IMU_SAFETY_CLEAR_MS ?
               IMU_BENCH_HINT_PRESS_RECOVER : IMU_BENCH_HINT_WAIT_CLEAR;
    }
    return uptime_ms < IMU_BENCH_IDLE_MS ? IMU_BENCH_HINT_IDLE_WINDOW :
                                           IMU_BENCH_HINT_PRESS_START;
}

const char *imu_bench_hint_text(imu_bench_hint_t hint)
{
    return (unsigned int)hint < sizeof(hint_text) / sizeof(hint_text[0]) ?
           hint_text[hint] : "UNKNOWN";
}

#if IMU_SAFETY_BENCH
#if APP_BENCH || (IMU_SAFETY_BENCH_ROBOT ? APP_TEST : !APP_TEST)
#error "IMU_SAFETY_BENCH: TEST stream needs TEST=1, ROBOT stream TEST=0"
#endif
#include "../platform/app_board.h"
#include "../platform/drv_time.h"
#if IMU_SAFETY_BENCH_ROBOT
#include "../vehicle/vehicle_task.h"
#endif
#include <string.h>

static imu_bench_stats_t stats;
static APP_BOARD(imu_bench_stats_t) board;
static uint32_t section_us[IMU_BENCH_REST];
static uint32_t origin_ms;
static unsigned int phase;
static unsigned int previous_phase;
static bool begun;
static bool previous;
#if IMU_SAFETY_BENCH_ROBOT
static imu_replay_robot_t robot;
#endif

void imu_bench_probe_init(imu_probe_t *probe)
{
    memset(probe, 0, sizeof(*probe));
    memset(&stats, 0, sizeof(stats));
    memset(section_us, 0, sizeof(section_us));
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        stats.phase[i].compute_min_us = UINT32_MAX;
        for (unsigned int s = 0u; s < IMU_BENCH_SECTIONS; ++s) {
            stats.phase[i].split[s].min_us = UINT32_MAX;
        }
    }
    stats.robot = IMU_SAFETY_BENCH_ROBOT != 0;
#if IMU_SAFETY_BENCH_ROBOT
    imu_replay_robot_init(&robot);
#endif
    begun = false;
    previous = false;
}

bool imu_bench_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *raw)
{
    if (!begun) { origin_ms = now_ms; begun = true; }
#if IMU_SAFETY_BENCH_ROBOT
    (void)origin_ms;
    phase = imu_replay_robot_read(&robot, probe, raw, now_ms,
                                  vehicle_calibration_epoch());
    stats.episodes = imu_replay_robot_episodes(&robot);
#else
    phase = imu_replay_read(probe, raw, now_ms - origin_ms);
#endif
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

/* Section wrappers. Only the service compiled with service_impl.h
 * calls these; the wrapped functions are unchanged. */
void imu_bench_core_step(imu_core_t *core, const imu_core_input_t *input)
{
    const uint32_t t = drv_time_us();
    imu_core_step(core, input);
    section_us[IMU_BENCH_CORE] += drv_time_us() - t;
}

void imu_bench_hump_diag_step(imu_hump_diag_t *s,
                              const imu_hump_diag_input_t *in,
                              imu_hump_diag_report_t *out)
{
    const uint32_t t = drv_time_us();
    imu_hump_diag_step(s, in, out);
    section_us[IMU_BENCH_DIAG] += drv_time_us() - t;
}

void imu_bench_safety_step(imu_safety_t *s, const imu_safety_input_t *in)
{
    const uint32_t t = drv_time_us();
    imu_safety_step(s, in);
    section_us[IMU_BENCH_SAFETY] += drv_time_us() - t;
}

static void split_add(imu_bench_split_t *x, uint32_t us)
{
    x->total_us += us;
    if (us > x->max_us) { x->max_us = us; }
    if (us < x->min_us) { x->min_us = us; }
}

void imu_bench_record(const imu_snapshot_t *snapshot)
{
    imu_bench_phase_t *current = &stats.phase[phase];
    const uint32_t compute = snapshot->compute_last_us;
    ++current->samples;
    current->compute_total_us += compute;
    if (compute > current->compute_max_us) {
        current->compute_max_us = compute;
    }
    if (compute < current->compute_min_us) {
        current->compute_min_us = compute;
    }
    uint32_t timed = 0u;
    for (unsigned int s = 0u; s < IMU_BENCH_REST; ++s) {
        split_add(&current->split[s], section_us[s]);
        timed += section_us[s];
        section_us[s] = 0u;
    }
    /* The sections are nested in compute, so this never wraps. */
    split_add(&current->split[IMU_BENCH_REST],
              compute >= timed ? compute - timed : 0u);
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
