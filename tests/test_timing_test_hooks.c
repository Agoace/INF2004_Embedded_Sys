/* Exercise S1/S2 TEST reporting with a bad lifetime and an independent window. */
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem1/telemetry.h"
#include "../app_program/platform/drv_watchdog.h"
static unsigned int captured,probe_begin,probe_end,probe_rows,passes,failures;
static unsigned int sends,s0_prints,s1_prints,lifetime_infos;
static uint32_t now_us;
static bool window_open,window_ended,bad_window;
static motion_timing_probe_t probes[MOTION_PROBE_COUNT];
int drv_time_check_window(uint32_t ms,unsigned int tolerance,uint32_t *us,uint32_t *kernel) {
    assert(ms==1000u && tolerance==1u);*us=1010000u;*kernel=1010u;now_us=*us;return E_OK;
}
uint32_t drv_time_ms(void) { return now_us/1000u; }
ER tk_dly_tsk(RELTIM timeout) { now_us+=(uint32_t)timeout*1000u;return E_OK; }
void telemetry_stats(telemetry_stats_t *s) {
    *s=(telemetry_stats_t){0};if (sends) { s->lost=66u;s->rows=34u; }
}
int telemetry_send(const telemetry_message_t *m) {
    assert(captured==3u && (m->kind==TELEMETRY_BEGIN || m->kind==TELEMETRY_ROW));
    ++sends;return sends<=35u ? E_OK : E_TMOUT;
}
int motion_timing_probe_capture(motion_probe_stage_t stage) {
    assert((unsigned int)stage==captured);
    if (stage==MOTION_PROBE_AFTER_S0_PRINT) assert(s0_prints==1u);
    if (stage==MOTION_PROBE_S1_BURST_END) assert(sends==101u);
    if (stage==MOTION_PROBE_AFTER_S1_PRINT) assert(s1_prints==1u);
    if (stage==MOTION_PROBE_S2_EXIT) assert(window_ended && !window_open);
    probes[stage]=(motion_timing_probe_t){.us=now_us,.ticks=100u+captured,
        .gap_min_us=8535u,.gap_max_us=11475u,.tick_max_us=198u,
        .late_wakes=stage>=MOTION_PROBE_S1_BURST_END ? 1u : 0u};
    ++captured;return E_OK;
}
int motion_timing_probe_read(motion_probe_stage_t stage,motion_timing_probe_t *p) {
    assert(captured==MOTION_PROBE_COUNT && window_ended);*p=probes[stage];return E_OK;
}
int motion_timing_window_begin(void) { assert(captured==7u && !window_open);window_open=true;return E_OK; }
int motion_timing_window_end(motion_timing_window_t *w) {
    assert(window_open);window_open=false;window_ended=true;
    *w=(motion_timing_window_t){.ticks=501u,.gaps=500u,.gap_min_us=9990u,
        .gap_max_us=bad_window ? 11001u : 10010u,.tick_max_us=100u,.late_wakes=bad_window ? 1u : 0u};
    return E_OK;
}
int tm_printf(const UB *format,...) {
    const char *f=(const char *)format;
    assert(!window_open);
    if (strstr(f,"S0-U01")) { assert(captured==1u);++s0_prints; }
    if (strstr(f,"S1-U02")) { assert(captured==5u);++s1_prints; }
    if (strstr(f,"INFO lifetime ticks")) { assert(captured==8u);++lifetime_infos; }
    if (strstr(f,"[TEST] PROBE")) {
        assert(captured==8u && window_ended);
        if (strstr(f,"PROBE begin")) ++probe_begin;
        else if (strstr(f,"PROBE end")) ++probe_end;
        else ++probe_rows;
    }
    if (strstr(f,"[TEST] %s PASS") || strstr(f,"[TEST] %s FAIL")) {
        va_list ap;va_start(ap,format);const char *id=va_arg(ap,const char *);va_end(ap);
        if (!strcmp(id,"S2-U01")) {
            if (strstr(f,"PASS")) ++passes;else ++failures;
        }
    }
    return 0;
}
int motion_status(motion_status_t *s) { *s=(motion_status_t){.faults=MC_FAULT_STALE};return E_OK; }
int motion_clear_faults(void) { return E_OK; }
int motion_set_speed(int16_t l,int16_t r) { assert(l==40 && r==40);return E_OK; }
int motion_stop(void) { return E_OK; }
int motion_estop(void) { return E_OK; }
bool drv_watchdog_test_previous(uint32_t *ms) { (void)ms;assert(0);return false; }
uint32_t drv_watchdog_test_sample(void) { assert(0);return 500u; }
int motion_watchdog_test_suppress_feed(void) { assert(0);return E_OBJ; }
bool motion_watchdog_test_feed_stopped(void) { assert(0);return false; }
extern void subsystem1_test(void);
extern void subsystem2_test(void);
int main(int argc,char **argv) {
    (void)argv;bad_window=argc>1;
    subsystem1_test();subsystem2_test();
    assert(probe_begin==1u && probe_end==1u && probe_rows==8u && lifetime_infos==1u);
    assert(sends==101u && s0_prints==1u && s1_prints==1u);
    assert(passes==(bad_window ? 0u : 1u) && failures==(bad_window ? 1u : 0u));
    return 0;
}
