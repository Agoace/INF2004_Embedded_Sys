/* ROBOT synthetic reporter. Observes only: no actuator command, no clear
 * request, no physical IMU read. The Vehicle owns the gate and recovery. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "stats.h"
#if IMU_SAFETY_BENCH && IMU_SAFETY_BENCH_ROBOT
#if APP_TEST || APP_BENCH || TK_SUPPORT_SMP
#error "IMU_SAFETY_BENCH robot reporter requires UP TEST=0 BENCH=0"
#endif
#include "../app.h"
#include "../platform/drv_time.h"
#include "../subsystem1/robot_health.h"
#include "../vehicle/vehicle_board.h"
#include "../vehicle/vehicle_task.h"
#include <string.h>

#define POLL_MS 250u
#define CONFIG_MS 5000u
#define REPORT_MS 10000u
#define STACK_BYTES 2048u
#define PRIORITY 29 /* Below every control task, above telemetry 30. */
#define FILL 0xa5u
static uint64_t reporter_stack[STACK_BYTES / sizeof(uint64_t)];
static imu_bench_stats_t totals;
static imu_snapshot_t snapshot;

static uint32_t mean(uint64_t total, uint32_t samples)
{
    return samples == 0u ? 0u : (uint32_t)(total / samples);
}

static uint32_t low(uint32_t min, uint32_t samples)
{
    return samples == 0u ? 0u : min;
}

static uint32_t stack_free(void)
{
    uint32_t n = 0u;
    const uint8_t *stack = (const uint8_t *)reporter_stack;
    while (n < sizeof(reporter_stack) && stack[n] == FILL) { ++n; }
    return n;
}

static void config(uint32_t now)
{
    tm_printf((UB *)"[IMU_BENCH_CONFIG] robot_synthetic 100Hz UP "
              "NO_PHYSICAL_IMU uptime_ms %lu episodes %lu delay_ms %u "
              "idle_ms %u wall_time reporter_pri %u free %lu\n",
              (unsigned long)now, (unsigned long)totals.episodes,
              (unsigned int)IMU_REPLAY_ROBOT_DELAY_MS,
              (unsigned int)IMU_BENCH_IDLE_MS, (unsigned int)PRIORITY,
              (unsigned long)stack_free());
}

static void phases(void)
{
    for (unsigned int i = 0u; i < IMU_REPLAY_PHASES; ++i) {
        const imu_bench_phase_t *p = &totals.phase[i];
        if (p->samples == 0u) { continue; }
        tm_printf((UB *)"[IMU_BENCH_PHASE] phase %u samples %lu "
                  "compute_mean_us %lu compute_max_us %lu compute_min_us %lu "
                  "work_mean_us %lu work_max_us %lu overruns %lu "
                  "trigger_bits %lu\n",
                  i, (unsigned long)p->samples,
                  (unsigned long)mean(p->compute_total_us, p->samples),
                  (unsigned long)p->compute_max_us,
                  (unsigned long)low(p->compute_min_us, p->samples),
                  (unsigned long)mean(p->work_total_us, p->work_samples),
                  (unsigned long)p->work_max_us, (unsigned long)p->overruns,
                  (unsigned long)p->triggers);
        const imu_bench_split_t *s = p->split;
        tm_printf((UB *)"[IMU_BENCH_SPLIT] phase %u "
                  "core %lu/%lu/%lu diag %lu/%lu/%lu "
                  "safety %lu/%lu/%lu rest %lu/%lu/%lu mean/max/min_us\n", i,
                  (unsigned long)mean(s[0].total_us, p->samples),
                  (unsigned long)s[0].max_us,
                  (unsigned long)low(s[0].min_us, p->samples),
                  (unsigned long)mean(s[1].total_us, p->samples),
                  (unsigned long)s[1].max_us,
                  (unsigned long)low(s[1].min_us, p->samples),
                  (unsigned long)mean(s[2].total_us, p->samples),
                  (unsigned long)s[2].max_us,
                  (unsigned long)low(s[2].min_us, p->samples),
                  (unsigned long)mean(s[3].total_us, p->samples),
                  (unsigned long)s[3].max_us,
                  (unsigned long)low(s[3].min_us, p->samples));
    }
}

static void report(uint32_t now)
{
    if (imu_snapshot_try_read(&snapshot) >= E_OK) {
        tm_printf((UB *)"[IMU_BENCH] uptime_ms %lu missed_wakes %lu "
                  "lost_wakes %lu gap_max_us %lu\n", (unsigned long)now,
                  (unsigned long)snapshot.missed_wakes,
                  (unsigned long)snapshot.wake_lost,
                  (unsigned long)snapshot.gap_max_us);
    }
    phases();
    rm_heartbeat_t health;
    uint32_t health_us;
    if (robot_health_try_read(&health, &health_us) >= E_OK) {
        tm_printf((UB *)"[IMU_BENCH_CPU] aggregate_permille %u "
                  "exclusive_per_task_cpu NA\n",
                  (unsigned int)health.cpu_load_permille);
    }
}

static void reporter(INT code, void *context)
{
    (void)code;
    (void)context;
    uint32_t config_at = 0u, report_at = 0u, hint_at = 0u;
    int last_hint = -1;
    for (;;) {
        (void)tk_dly_tsk(POLL_MS);
        const uint32_t now = drv_time_ms();
        const bool have = imu_bench_stats_try_read(&totals);
        imu_safety_report_t safety;
        vehicle_snapshot_t vehicle;
        const bool s_ok = imu_safety_try_read(&safety) >= E_OK;
        const bool v_ok = vehicle_snapshot_try_read(&vehicle) >= E_OK;
        if (now - config_at >= CONFIG_MS) {
            config_at = now;
            config(now);
        }
        if (s_ok && v_ok) {
            const imu_bench_hint_t hint = imu_bench_hint(
                safety.value.latched_trigger, safety.value.clear_ms,
                vehicle.active, now);
            if ((int)hint != last_hint || now - hint_at >= CONFIG_MS) {
                last_hint = (int)hint;
                hint_at = now;
                tm_printf((UB *)"[IMU_BENCH_HINT] uptime_ms %lu %s "
                          "latched %u clear_ms %lu state %u active %u "
                          "response_max_us %lu overruns %lu lost %lu\n",
                          (unsigned long)now, imu_bench_hint_text(hint),
                          (unsigned int)safety.value.latched_trigger,
                          (unsigned long)safety.value.clear_ms,
                          (unsigned int)vehicle.state,
                          (unsigned int)vehicle.active,
                          (unsigned long)vehicle.response_max_us,
                          (unsigned long)vehicle.overruns,
                          (unsigned long)vehicle.wake_lost);
            }
        }
        if (have && now - report_at >= REPORT_MS) {
            report_at = now;
            report(now);
        }
    }
}

int imu_bench_robot_start(void)
{
    const int result = vehicle_task_start();
    memset(reporter_stack, FILL, sizeof(reporter_stack));
    const T_CTSK ctsk = {.task = (FP)reporter, .itskpri = PRIORITY,
        .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
        .stksz = (SZ)sizeof(reporter_stack), .bufptr = reporter_stack};
    const ID task = tk_cre_tsk(&ctsk);
    const ER er = task > 0 ? tk_sta_tsk(task, 0) : task;
    tm_printf((UB *)"[IMU_BENCH_START] robot_synthetic reporter %ld\n",
              (long)er);
    return result;
}
#endif
