/*
 * Bench step 0: one [bench] line after the GP20 timeline dump.
 *
 * Built only with BENCH_KERNEL=1, linked with --wrap=rt_timeline_report, so
 * no robot source changes. Runs in the telemetry dump (core 1 in split): it
 * uses only the any-core bench_stats_read() and never reads an open window.
 */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_report.h"
#include "../subsystem2/motion.h"
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

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
                    "gen_us_sum %lu gen_us_max %lu cost_bp %lu %s\n",
                    (unsigned long)window, (unsigned long)s.edges_sched,
                    (unsigned long)s.edges_delivered,
                    (unsigned long)s.coalesced, (unsigned long)s.overflow,
                    (unsigned long)s.gen_calls, (unsigned long)s.gen_us_sum,
                    (unsigned long)s.gen_us_max, (unsigned long)cost_bp,
                    verdict);
    /* Public bounded last-tick board; source has stopped before this read. */
    motion_status_t motion;
    uint32_t timestamp;
    if (motion_snapshot_try_read(&motion, &timestamp) != E_OK) {
        (void)tm_printf((UB *)"[bench] counts unavailable COUNT_MISMATCH\n");
        return;
    }
    const int64_t left = (int64_t)motion.counts[0] - s.ack_left;
    const int64_t right = (int64_t)motion.counts[1] - s.ack_right;
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
                    left == 0 && right == 0 ?
                        "COUNT_MATCH" : "COUNT_MISMATCH");
}

void __wrap_rt_timeline_report(void (*pause)(void))
{
    __real_rt_timeline_report(pause);
    bench_report();
}
#endif
