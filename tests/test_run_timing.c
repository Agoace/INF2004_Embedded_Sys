/* P1 host checks: per-run latches, single-writer reset and report lines. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/platform/run_timing.h"

static char out[4096];
static size_t used;

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
int tm_printf(const unsigned char *format, ...)
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

static void check_max_and_reset(void)
{
    rt_snapshot_t s;
    rt_begin(1u);
    /* A latch written before its owner sees the new run is cleared first. */
    rt_max(RT_ISR_LATE, 40u);
    rt_max(RT_ISR_LATE, 25u);
    rt_max(RT_ISR_ADC, 70u);
    rt_snapshot(&s);
    assert(s.epoch == 1u);
    assert(s.max[RT_ISR_LATE] == 40u && s.max[RT_ISR_ADC] == 70u);
    assert(s.max[RT_LINE_SAMPLE] == 0u);
    /* New run: every owner clears only its own word on its next write. */
    rt_begin(3u);
    rt_max(RT_ISR_LATE, 5u);
    rt_snapshot(&s);
    assert(s.epoch == 3u && s.max[RT_ISR_LATE] == 5u);
    /* RT_ISR_ADC has not been written in run 3: it reads as 0, not 70. */
    assert(s.max[RT_ISR_ADC] == 0u);
    /* Even epoch (STOP) freezes: writes are ignored. */
    rt_begin(4u);
    rt_max(RT_ISR_LATE, 999u);
    rt_snapshot(&s);
    assert(s.epoch == 3u && s.max[RT_ISR_LATE] == 5u);
}

static void check_calls(void)
{
    rt_snapshot_t s;
    rt_begin(5u);
    rt_call(RT_K_ISR_WUP, 10u);
    rt_call(RT_K_ISR_WUP, 30u);
    rt_call(RT_K_LINE_LOC, 7u);
    rt_snapshot(&s);
    assert(s.call[RT_K_ISR_WUP].count == 2u);
    assert(s.call[RT_K_ISR_WUP].sum_us == 40u);
    assert(s.call[RT_K_ISR_WUP].max_us == 30u);
    assert(s.call[RT_K_LINE_LOC].count == 1u);
    assert(s.call[RT_K_VEH_SEND].count == 0u);
    rt_begin(7u);
    rt_call(RT_K_LINE_LOC, 3u);
    rt_snapshot(&s);
    assert(s.call[RT_K_LINE_LOC].count == 1u);
    assert(s.call[RT_K_LINE_LOC].sum_us == 3u);
    assert(s.call[RT_K_ISR_WUP].count == 0u); /* not yet written in run 7 */
    rt_begin(8u);
    rt_call(RT_K_LINE_LOC, 100u);
    rt_snapshot(&s);
    assert(s.call[RT_K_LINE_LOC].count == 1u);
}

static void check_first_overrun(void)
{
    rt_snapshot_t s;
    rt_begin(9u);
    rt_snapshot(&s);
    assert(s.first.step == 0u);
    rt_overrun_t a = {.step = 12u, .elapsed_us = 8000u, .tick_delta = 1u,
                      .slow_part = RT_STEP_SONAR, .slow_us = 900u,
                      .wake_us = 1500u, .uptime_ms = 4321u};
    rt_overrun(&a);
    rt_overrun_t b = a;
    b.step = 13u;
    rt_overrun(&b); /* only the first one in a run is kept */
    rt_snapshot(&s);
    assert(s.first.step == 12u && s.first.slow_part == RT_STEP_SONAR);
    assert(s.first.wake_us == 1500u && s.first.uptime_ms == 4321u);
    rt_begin(11u);
    rt_overrun(&b);
    rt_snapshot(&s);
    assert(s.first.step == 13u);
}

static void check_report(void)
{
    rt_begin(13u);
    rt_max(RT_ISR_LATE, 11u);
    rt_max(RT_ISR_ADC, 12u);
    rt_max(RT_ISR_WUP, 13u);
    rt_max(RT_LINE_SAMPLE, 21u);
    rt_max(RT_LINE_LPU, 22u);
    rt_max(RT_LINE_BARCODE, 23u);
    rt_max(RT_STEP_WAKE, 31u);
    rt_max(RT_STEP_READ_LINE, 32u);
    rt_max(RT_STEP_COMMAND, 33u);
    rt_max(RT_STEP_SONAR, 34u);
    rt_max(RT_STEP_VSM, 35u);
    rt_max(RT_STEP_LOG, 36u);
    rt_max(RT_STEP_PUBLISH, 37u);
    rt_max(RT_STEP_STATUS, 38u);
    rt_call(RT_K_ISR_WUP, 9u);
    rt_call(RT_K_ISR_WUP, 11u);
    rt_overrun_t a = {.step = 5u, .elapsed_us = 9100u, .tick_delta = 1u,
                      .slow_part = RT_STEP_LOG, .slow_us = 36u,
                      .wake_us = 31u, .uptime_ms = 777u};
    rt_overrun(&a);
    used = 0u;
    out[0] = '\0';
    rt_report();
    assert(strstr(out, "[isr_max] epoch 13 late 11 adc 12 wup 13 us\n"));
    assert(strstr(out, "[line_max] epoch 13 sample 21 lp_update 22 "
                       "barcode 23 us\n"));
    assert(strstr(out, "[step_max] epoch 13 wake 31 read_line 32 command 33 "
                       "sonar 34 vsm 35 log 36 publish 37 status 38 us\n"));
    assert(strstr(out, "[kcall] isr_wup n 2 sum 20 max 11 us\n"));
    assert(strstr(out, "[kcall] veh_send n 0 sum 0 max 0 us\n"));
    assert(strstr(out, "[first_overrun] epoch 13 step 5 elapsed 9100 us "
                       "ticks +1 slowest log 36 us wake 31 us uptime 777 ms\n"));
    rt_begin(15u);
    used = 0u;
    out[0] = '\0';
    rt_report();
    assert(strstr(out, "[first_overrun] epoch 15 none\n"));
}

static void check_load_and_bkl(void)
{
    rt_snapshot_t s;
    rt_begin(17u);
    assert(rt_epoch() == 17u);
    rt_load(17u, 4u, 640u, 65535u);
    const rt_bkl_t k = {.valid = 1u, .acquisitions = 8000u,
                        .contentions = 450u, .max_wait_us = 111u,
                        .max_hold_us = 293u, .max_depth = 3u,
                        .detached = 12u, .run_ms = 1000u};
    rt_bkl(&k);
    rt_begin(18u);
    assert(rt_epoch() == 18u);
    rt_bkl(&k); /* frozen: ignored, the stored copy stays */
    rt_snapshot(&s);
    assert(s.load_epoch == 17u && s.load_samples == 4u);
    assert(s.load_peak[0] == 640u && s.load_peak[1] == 65535u);
    assert(s.bkl.valid == 1u && s.bkl.contentions == 450u);
    used = 0u;
    out[0] = '\0';
    rt_report();
    assert(strstr(out, "[load_run] epoch 17 samples 4 peak c0 640 "
                       "c1 65535 permille\n"));
    assert(strstr(out, "[bkl] epoch 17 run 1000 ms acq 8000 cont 450 "
                       "max_wait 111 max_hold 293 us depth 3 "
                       "detached 12\n"));
    rt_begin(19u);
    rt_snapshot(&s);
    assert(s.bkl.valid == 0u); /* not yet written in run 19 */
    const rt_bkl_t none = {0};
    rt_bkl(&none);
    used = 0u;
    out[0] = '\0';
    rt_report();
    assert(strstr(out, "[bkl] epoch 19 unavailable\n"));
}

int main(void)
{
    rt_snapshot_t s;
    rt_snapshot(&s);
    assert(s.epoch == 0u); /* never started: frozen at zero */
    rt_max(RT_ISR_LATE, 50u);
    rt_snapshot(&s);
    assert(s.max[RT_ISR_LATE] == 0u);
    check_max_and_reset();
    check_calls();
    check_first_overrun();
    check_report();
    check_load_and_bkl();
    /* Motion wake stamp: written always, also while frozen. */
    assert(rt_motion_stamp_us() == 0u);
    rt_motion_stamp(1234u);
    assert(rt_motion_stamp_us() == 1234u);
    return 0;
}
