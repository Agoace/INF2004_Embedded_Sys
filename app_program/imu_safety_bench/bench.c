/* Non-destructive TEST reporter. No actuator command or physical IMU read. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "stats.h"
#if IMU_SAFETY_BENCH && !IMU_SAFETY_BENCH_ROBOT
#if !APP_TEST || APP_BENCH || TK_SUPPORT_SMP
#error "IMU_SAFETY_BENCH reporter requires UP TEST=1 BENCH=0"
#endif
#include "../app.h"
#include "../platform/app_task.h"
#include "../platform/drv_time.h"
#include "../platform/run_timing.h"
#include "../subsystem1/robot_health.h"
#include <string.h>

#define BENCH_DURATION_MS 70000u
#define BENCH_PRINT_MS 5000u
#define BENCH_STACK_BYTES 4096u
#define BENCH_FILL 0xa5u
static uint64_t reporter_stack[BENCH_STACK_BYTES / sizeof(uint64_t)];
static imu_snapshot_t snapshot;
static imu_bench_stats_t totals;

static uint32_t mean(uint64_t total, uint32_t samples)
{
    return samples == 0u ? 0u : (uint32_t)(total / samples);
}

static void report(uint32_t elapsed)
{
    uint32_t samples = 0u, work_samples = 0u, compute_max = 0u;
    uint32_t work_max = 0u, overruns = 0u;
    uint64_t compute_total = 0u, work_total = 0u;
    if (!imu_bench_stats_try_read(&totals) ||
        imu_snapshot_try_read(&snapshot) < E_OK) {
        tm_printf((UB *)"[IMU_BENCH] report unavailable elapsed_ms %lu\n",
                  (unsigned long)elapsed);
        return;
    }
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        const imu_bench_phase_t *p = &totals.phase[i];
        samples += p->samples;
        work_samples += p->work_samples;
        compute_total += p->compute_total_us;
        work_total += p->work_total_us;
        if (p->compute_max_us > compute_max) { compute_max = p->compute_max_us; }
        if (p->work_max_us > work_max) { work_max = p->work_max_us; }
        overruns += p->overruns;
    }
    tm_printf((UB *)"[IMU_BENCH] elapsed_ms %lu samples %lu compute_mean_us %lu "
              "compute_max_us %lu work_mean_us %lu response_wall_max_us %lu "
              "overruns %lu missed_wakes %lu lost_wakes %lu gap_max_us %lu\n",
              (unsigned long)elapsed, (unsigned long)samples,
              (unsigned long)mean(compute_total, samples),
              (unsigned long)compute_max,
              (unsigned long)mean(work_total, work_samples),
              (unsigned long)work_max, (unsigned long)overruns,
              (unsigned long)snapshot.missed_wakes,
              (unsigned long)snapshot.wake_lost,
              (unsigned long)snapshot.gap_max_us);
    rm_heartbeat_t health;
    uint32_t health_us;
    if (robot_health_try_read(&health, &health_us) >= E_OK) {
        tm_printf((UB *)"[IMU_BENCH_CPU] aggregate_permille %u "
                  "exclusive_per_task_cpu NA\n",
                  (unsigned int)health.cpu_load_permille);
    }
    /* TEST images have no Vehicle task; no Vehicle line exists. */
}

static void reporter(INT code, void *context)
{
    (void)code;
    (void)context;
    const uint32_t started = drv_time_ms();
    rt_begin(1u);
    tm_printf((UB *)"[IMU_BENCH_BEGIN] synthetic 100Hz UP duration_ms 70000 "
              "NO_PHYSICAL_IMU NO_ACTUATOR_COMMANDS NO_BUS_COST "
              "compute_and_work_are_wall_time\n");
    while (drv_time_ms() - started < BENCH_DURATION_MS) {
        /* A 1 Hz safe-clear check does not alter the measured IMU cadence. */
        (void)tk_dly_tsk(1000);
        imu_safety_report_t safety;
        if (imu_safety_try_read(&safety) >= E_OK &&
            safety.value.latched_trigger != IMU_SAFETY_NONE &&
            safety.value.clear_ms >= IMU_SAFETY_CLEAR_MS) {
            (void)imu_safety_clear_request();
        }
        const uint32_t elapsed = drv_time_ms() - started;
        if (elapsed % BENCH_PRINT_MS < 1000u) { report(elapsed); }
    }
    report(drv_time_ms() - started);
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        const imu_bench_phase_t *p = &totals.phase[i];
        tm_printf((UB *)"[IMU_BENCH_PHASE] phase %u samples %lu "
                  "compute_mean_us %lu compute_max_us %lu work_mean_us %lu "
                  "work_max_us %lu overruns %lu trigger_bits %lu\n",
                  i, (unsigned long)p->samples,
                  (unsigned long)mean(p->compute_total_us, p->samples),
                  (unsigned long)p->compute_max_us,
                  (unsigned long)mean(p->work_total_us, p->work_samples),
                  (unsigned long)p->work_max_us, (unsigned long)p->overruns,
                  (unsigned long)p->triggers);
        const imu_bench_split_t *x = p->split;
        tm_printf((UB *)"[IMU_BENCH_SPLIT] phase %u core %lu/%lu "
                  "diag %lu/%lu safety %lu/%lu rest %lu/%lu "
                  "compute_min_us %lu mean/max_us\n", i,
                  (unsigned long)mean(x[0].total_us, p->samples),
                  (unsigned long)x[0].max_us,
                  (unsigned long)mean(x[1].total_us, p->samples),
                  (unsigned long)x[1].max_us,
                  (unsigned long)mean(x[2].total_us, p->samples),
                  (unsigned long)x[2].max_us,
                  (unsigned long)mean(x[3].total_us, p->samples),
                  (unsigned long)x[3].max_us,
                  (unsigned long)(p->samples ? p->compute_min_us : 0u));
        (void)tk_dly_tsk(20);
    }
    app_task_report();
    rt_report();
    uint32_t free_bytes = 0u;
    const uint8_t *stack = (const uint8_t *)reporter_stack;
    while (free_bytes < sizeof(reporter_stack) &&
           stack[free_bytes] == BENCH_FILL) {
        ++free_bytes;
    }
    tm_printf((UB *)"[IMU_BENCH_STACK] name imu_bench bytes 4096 min_free %lu\n",
              (unsigned long)free_bytes);
    tm_printf((UB *)"[IMU_BENCH_DONE] completed_70s 1 synthetic_only 1\n");
    (void)tk_slp_tsk(TMO_FEVR);
}

void imu_safety_bench_run(void)
{
    memset(reporter_stack, BENCH_FILL, sizeof(reporter_stack));
    const T_CTSK config = {.task = (FP)reporter, .itskpri = 21,
        .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
        .stksz = (SZ)sizeof(reporter_stack), .bufptr = reporter_stack};
    const ID task = tk_cre_tsk(&config);
    const ER result = task > 0 ? tk_sta_tsk(task, 0) : task;
    tm_printf((UB *)"[IMU_BENCH_START] result %ld reporter_priority 21\n",
              (long)result);
}
#endif
