/*
 * Bench step 0: one [bench] line after the GP20 timeline dump.
 *
 * Built only with BENCH_KERNEL=1, linked with --wrap=rt_timeline_report, so
 * no robot source changes. Runs in the telemetry dump (core 1 in split): it
 * uses only the any-core bench_stats_read() and never reads an open window.
 */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_report.h"
#include <stdint.h>
#include <tk/tkernel.h> /* TK_SUPPORT_SMP before idle_meter.h */
#include <tm/tmonitor.h>
#include "../platform/idle_meter.h"
#include "../platform/run_timing.h"
#include "../subsystem2/motion.h"
#include "bench_top.h"

static void bench_report(void)
{
    bench_stats_t s;
    if (!bench_stats_read(&s)) {
        (void)tm_printf((UB *)"[bench] no snapshot\n");
        return;
    }
    if (s.stopped == 0u) {
        (void)tm_printf((UB *)"[bench] window open\n");
        return;
    }
    const uint32_t window = s.window_end_us - s.window_start_us;
    const char *verdict = "BAD_WINDOW";
    uint32_t cost_bp = 0u;
    if (window != 0u && window <= BENCH_WINDOW_MAX_US) {
        cost_bp = (uint32_t)(((uint64_t)s.gen_us_sum * 10000u) / window);
        verdict = cost_bp < BENCH_COST_LIMIT_BP ? "OK" : "COST_HIGH";
    }
    if (s.overflow != 0u) {
        verdict = "GEN_STOPPED"; /* fail-closed generator: invalid run */
    }
    (void)tm_printf((UB *)"[bench] window_us %lu sched %lu "
                    "delivered %lu coalesced %lu overflow %lu gen_calls %lu "
                    "gen_us_sum %lu gen_us_max %lu cost_bp %lu %s",
                    (unsigned long)window, (unsigned long)s.edges_sched,
                    (unsigned long)s.edges_delivered,
                    (unsigned long)s.coalesced, (unsigned long)s.overflow,
                    (unsigned long)s.gen_calls, (unsigned long)s.gen_us_sum,
                    (unsigned long)s.gen_us_max, (unsigned long)cost_bp,
                    verdict);
    if (s.overflow != 0u) {
        (void)tm_printf((UB *)" stop_late_us %lu stop_at_us %lu",
                        (unsigned long)s.stop_late_us,
                        (unsigned long)s.stop_position_us);
    }
    (void)tm_printf((UB *)"\n");
    /* Public bounded last-tick board; source has stopped before this read. */
    motion_status_t motion;
    uint32_t timestamp;
    if (motion_snapshot_try_read(&motion, &timestamp) != E_OK) {
        (void)tm_printf((UB *)"[bench] counts unavailable COUNT_MISMATCH\n");
        return;
    }
    const int64_t left = (int64_t)motion.counts[0] - s.ack_left;
    const int64_t right = (int64_t)motion.counts[1] - s.ack_right;
    /* Every scheduled edge delivered and counted; no generator stop. */
    const bool match = s.overflow == 0u && left == 0 && right == 0 &&
                       s.sched_left == s.ack_left &&
                       s.sched_right == s.ack_right;
    (void)tm_printf((UB *)"[bench] left sched %lu delivered %lu "
                    "counted %ld diff %ld right sched %lu delivered %lu "
                    "counted %ld diff %ld %s\n",
                    (unsigned long)s.sched_left, (unsigned long)s.ack_left,
                    (long)motion.counts[0],
                    (long)(left < INT32_MIN ? INT32_MIN :
                           left > INT32_MAX ? INT32_MAX : left),
                    (unsigned long)s.sched_right, (unsigned long)s.ack_right,
                    (long)motion.counts[1],
                    (long)(right < INT32_MIN ? INT32_MIN :
                           right > INT32_MAX ? INT32_MAX : right),
                    match ? "COUNT_MATCH" : "COUNT_MISMATCH");
}

#define BK_LOAD_WINDOW_S 601u

/* The ordinary [load]/P1 lines are MQTT-only in app_task_report(); the bench
 * (radio-free) prints them here, once per dump, from public readers. */
static void load_and_p1_report(void (*pause)(void))
{
#if defined(TK_SUPPORT_SMP) && TK_SUPPORT_SMP
    (void)tm_printf((UB *)"[load] core 0 %u permille, core 1 %u permille\n",
                    (unsigned int)idle_meter_load_permille(),
                    (unsigned int)idle_meter_load_permille_core1());
#else
    (void)tm_printf((UB *)"[load] core 0 %u permille\n",
                    (unsigned int)idle_meter_load_permille());
#endif
    /* One sample spans about 601 s: the 32-bit idle count wraps above this
     * calibrated rate, and [load] is then wrong. */
    const uint32_t limit = UINT32_MAX / BK_LOAD_WINDOW_S;
    const uint32_t rate = idle_meter_peak_rate();
    (void)tm_printf((UB *)"[bench_load] peak_rate %lu limit %lu %s\n",
                    (unsigned long)rate, (unsigned long)limit,
                    rate <= limit ? "OK" : "WRAP_RISK");
    rt_report();
    bk_top_report(pause);
}

void __wrap_rt_timeline_report(void (*pause)(void))
{
    __real_rt_timeline_report(pause);
    load_and_p1_report(pause);
    bench_report();
}
#endif
