/* Bench step 0 report: one [bench] line after the GP20 timeline dump. */
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app_program/bench_kernel/bench_report.h"
#include "../app_program/subsystem2/motion.h"
static bench_stats_t snap;
static int32_t native[2] = {500, 500};
int motion_snapshot_try_read(motion_status_t *s, uint32_t *us)
{
    assert(snap.stopped); /* Never read counts for an open window. */
    s->counts[0] = native[0]; s->counts[1] = native[1];
    *us = 0u;
    return E_OK;
}

static char out[2048];
static size_t used;
static bool have_snap;
static unsigned int real_calls;
static unsigned int read_calls;

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
int tm_printf(const UB *format, ...)
{
    va_list ap;
    va_start(ap, format);
    const int n = vsnprintf(out + used, sizeof(out) - used,
                            (const char *)format, ap);
    va_end(ap);
    assert(n >= 0 && (size_t)n < sizeof(out) - used);
    used += (size_t)n;
    return n;
}

/* [load] and the P1 report print after [timeline], before [bench]. */
/* Over 601 s a 32-bit idle count wraps above 7146368 per second. */
static uint32_t peak_rate = 7146368u;
uint32_t idle_meter_peak_rate(void)
{
    return peak_rate;
}
#define RATE "[bench_load] peak_rate 7146368 limit 7146368 OK\n"
#if TK_SUPPORT_SMP
#define HEAD "[timeline]\n[load] core 0 640 permille, core 1 600 permille\n" \
             RATE "[P1]\n[TOP]\n"
uint16_t idle_meter_load_permille_core1(void)
{
    return 600u;
}
#else
#define HEAD "[timeline]\n[load] core 0 640 permille\n" RATE \
             "[P1]\n[TOP]\n"
#endif
static unsigned int p1_calls;
uint16_t idle_meter_load_permille(void)
{
    return 640u;
}
void rt_report(void)
{
    ++p1_calls;
    used += (size_t)snprintf(out + used, sizeof(out) - used, "[P1]\n");
}

static void pause_stub(void);
static unsigned int top_calls;
void bk_top_report(void (*pause)(void))
{
    assert(pause == pause_stub);
    ++top_calls;
    used += (size_t)snprintf(out + used, sizeof(out) - used, "[TOP]\n");
}

bool bench_stats_read(bench_stats_t *dst)
{
    ++read_calls;
    if (have_snap) {
        *dst = snap;
    }
    return have_snap;
}

static void pause_stub(void)
{
}

void __real_rt_timeline_report(void (*pause)(void))
{
    assert(pause == pause_stub);
    ++real_calls;
    used += (size_t)snprintf(out + used, sizeof(out) - used, "[timeline]\n");
}

static void run(void)
{
    used = 0u;
    out[0] = '\0';
    real_calls = 0u;
    read_calls = 0u;
    p1_calls = 0u;
    top_calls = 0u;
    __wrap_rt_timeline_report(pause_stub);
    assert(real_calls == 1u && p1_calls == 1u && top_calls == 1u);
    assert(read_calls == 1u);
}

static bench_stats_t stopped(uint32_t start, uint32_t end, uint32_t sum)
{
    return (bench_stats_t){
        .edges_sched = 1000u, .edges_delivered = 1000u, .coalesced = 0u,
        .sched_left = 500u, .sched_right = 500u,
        .ack_left = 500u, .ack_right = 500u,
        .overflow = 0u, .gen_us_sum = sum, .gen_us_max = 9u,
        .gen_calls = 500u, .window_start_us = start, .window_end_us = end,
        .stopped = 1u};
}

static void check_no_snapshot(void)
{
    have_snap = false;
    run();
    assert(strcmp(out, HEAD "[bench] no snapshot\n") == 0);
}

static void check_wrap_risk(void)
{
    have_snap = false;
    peak_rate = 7146369u;
    run();
    assert(strstr(out, "[bench_load] peak_rate 7146369 limit 7146368 "
                       "WRAP_RISK\n") != NULL);
    peak_rate = 7146368u;
}

static void check_open_window(void)
{
    have_snap = true;
    snap = stopped(0u, 0u, 0u);
    snap.stopped = 0u;
    run();
    assert(strcmp(out, HEAD "[bench] window open\n") == 0);
}

static void check_stopped_ok(void)
{
    have_snap = true;
    /* 10 min window, 2.99 % generator cost: 299 basis points, OK. */
    snap = stopped(1000u, 1000u + 600000000u, 17940000u);
    run();
    assert(strstr(out, "left sched 500 delivered 500 counted 500 diff 0")
           != NULL);
    assert(strstr(out, "right sched 500 delivered 500 counted 500 diff 0")
           != NULL);
    assert(strstr(out, "COUNT_MATCH") != NULL);
    native[1] = 499;
    run();
    assert(strstr(out, "right sched 500 delivered 500 counted 499 diff -1")
           != NULL);
    assert(strstr(out, "COUNT_MISMATCH") != NULL);
    native[1] = 500;
    run();
    const char *prefix = HEAD "[bench] window_us 600000000 "
        "sched 1000 delivered 1000 coalesced 0 overflow 0 "
        "gen_calls 500 gen_us_sum 17940000 gen_us_max 9 cost_bp 299 OK\n";
    assert(strncmp(out, prefix, strlen(prefix)) == 0);
}

static void check_count_policy(void)
{
    have_snap = true;
    /* A scheduled edge that was never delivered is a loss, even when the
     * native count agrees with the delivered count. */
    snap = stopped(0u, 600000000u, 1000u);
    snap.ack_left = 499u;
    native[0] = 499;
    run();
    assert(strstr(out, "left sched 500 delivered 499 counted 499 diff 0")
           != NULL);
    assert(strstr(out, "COUNT_MISMATCH") != NULL);
    native[0] = 500;
    /* A generator stop is never a count match. */
    snap = stopped(0u, 600000000u, 1000u);
    snap.overflow = 1u;
    run();
    assert(strstr(out, "diff 0 COUNT_MISMATCH") != NULL);
}

static void check_cost_high_and_wrap(void)
{
    have_snap = true;
    /* The window crosses the 32-bit us wrap; exactly 3 % is HIGH. */
    snap = stopped(0xFFFFFF00u, 0x000F41FFu, 30000u);
    run();
    assert(strstr(out, "window_us 1000191 ") != NULL);
    assert(strstr(out, "cost_bp 299 OK\n") != NULL);
    snap = stopped(0u, 1000000u, 30000u);
    run();
    assert(strstr(out, "cost_bp 300 COST_HIGH\n") != NULL);
}

static void check_bad_windows(void)
{
    have_snap = true;
    snap = stopped(5u, 5u, 0u);
    run();
    assert(strstr(out, "window_us 0 ") != NULL);
    assert(strstr(out, "cost_bp 0 BAD_WINDOW\n") != NULL);
    /* Over 60 min cannot be told apart from a wrapped short window. */
    snap = stopped(0u, 3600000001u, 1u);
    run();
    assert(strstr(out, "BAD_WINDOW\n") != NULL);
    snap = stopped(0u, 3600000000u, 1u);
    run();
    assert(strstr(out, "cost_bp 0 OK\n") != NULL);
}

static void check_generator_stopped(void)
{
    have_snap = true;
    /* A fail-closed generator stop overrides the cost verdict. */
    snap = stopped(0u, 1000000u, 1000u);
    snap.overflow = 1u;
    run();
    assert(strstr(out, "overflow 1 ") != NULL);
    assert(strstr(out, "cost_bp 10 GEN_STOPPED stop_late_us 0 "
                       "stop_at_us 0\n") != NULL);
    snap = stopped(0u, 0u, 0u);
    snap.overflow = 2u;
    snap.stop_late_us = 512u;
    snap.stop_position_us = 4000000000u;
    run();
    assert(strstr(out, "cost_bp 0 GEN_STOPPED stop_late_us 512 "
                       "stop_at_us 4000000000\n") != NULL);
}

int main(void)
{
    check_no_snapshot();
    check_wrap_risk();
    check_open_window();
    check_stopped_ok();
    check_cost_high_and_wrap();
    check_bad_windows();
    check_generator_stopped();
    check_count_policy();
    puts("test_bench_report: PASS");
    return 0;
}
