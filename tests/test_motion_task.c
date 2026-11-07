/* Host scheduler/hardware doubles: checks the glue, not real-time behaviour. */
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem2/subsystem2.h"
#include "../app_program/subsystem2/drv_motor_hw.h"
#include "../app_program/platform/drv_gpio_irq.h"
static FP task_entry, cyclic_entry;
static drv_gpio_irq_fn encoder;
static void *encoder_ctx;
static jmp_buf sleep_again;
static unsigned int sleeps;
static int held, irq_off, queued;
static unsigned int wake_calls, notify_calls, bench_calls, error_lines, mutex_calls;
static int init_error, check_error;
static unsigned int check_calls, wd_starts, wd_feeds, hw_writes;
static int cyclic_started, cyclic_error, task_error, watchdog_error;
#if APP_TEST
static unsigned int test_begins;
#endif
int drv_watchdog_start(uint32_t ms) {
    assert(ms==100u && task_entry!=0 && cyclic_started && !held);
    ++wd_starts; return watchdog_error;
}
void drv_watchdog_feed(void) {
    assert(cyclic_started && !held && !irq_off && hw_writes>wd_feeds); ++wd_feeds;
}
#if APP_TEST
int drv_watchdog_test_begin(void) { ++test_begins; return E_OK; }
#endif
static uint32_t now_us;
static uint8_t ab[2];
static int16_t output[2];
ID tk_cre_mtx(const T_CMTX *m) { assert(m->mtxatr == TA_INHERIT); ++mutex_calls; return 1; }
int drv_time_init(void) { return init_error; }
int drv_time_check(void) { ++check_calls; return check_error; }
int motion_bench_start(void) { ++bench_calls; return E_OK; }
int tm_printf(const UB *format, ...) { (void)format; ++error_lines; return 0; }
ER tk_loc_mtx(ID id, TMO timeout) { assert(id == 1 && timeout == TMO_FEVR && !held); held = 1; return E_OK; }
ER tk_unl_mtx(ID id) { assert(id == 1 && held); held = 0; return E_OK; }
ER tk_del_mtx(ID id) { (void)id; return E_OK; }
ID tk_cre_cyc(const T_CCYC *c) {
    assert(c->cyctim == 10u && c->cycphs == 10u && c->assprc == TP_PRC1);
    assert((c->cycatr & TA_ASSPRC) != 0u);
    cyclic_entry = c->cychdr; return 2;
}
ER tk_sta_cyc(ID id) { assert(id == 2); if (cyclic_error) return E_SYS; cyclic_started=1; return E_OK; }
ID app_start_task(FP entry, PRI pri, UW prc, const char *name) {
    (void)name; assert(pri == 10 && prc == TP_PRC1);
    if (task_error) return E_SYS;
    task_entry = entry; return 3;
}
ER tk_slp_tsk(TMO timeout) {
    assert(timeout == TMO_FEVR);
    if (sleeps++ != 0u) longjmp(sleep_again, 1);
    return E_OK;
}
INT tk_can_wup(ID id) { assert(id == TSK_SELF); return queued; }
ER tk_wup_tsk(ID id) {
    assert(!held && !irq_off);
    if (id == 3) ++wake_calls;
    else { assert(id == 4); ++notify_calls; }
    return E_OK;
}
INT tk_get_prc(void) { return 1; }
void mock_di(UINT *irq) { assert(!irq_off); *irq = 0u; irq_off = 1; }
void mock_ei(UINT irq) { assert(irq == 0u && irq_off); irq_off = 0; }
int drv_gpio_irq_init(void) { return E_OK; }
int drv_gpio_irq_add(uint32_t mask, drv_gpio_irq_fn fn, void *ctx) {
    assert(mask == DRV_MOTOR_ENCODER_MASK); encoder = fn; encoder_ctx = ctx; return E_OK;
}
uint32_t drv_time_us(void) { return now_us; }
uint32_t drv_time_ms(void) { return now_us / 1000u; }
void drv_motor_hw_init(uint16_t servo) { assert(servo == 1500u); output[0] = output[1] = 0; }
uint8_t drv_motor_hw_ab(unsigned int wheel) { return ab[wheel]; }
void drv_motor_hw_write(const int16_t duty[2], uint16_t servo) {
    (void)servo; assert(held && !irq_off); ++hw_writes;
    output[0] = duty[0]; output[1] = duty[1]; now_us += 7u;
}
void drv_motor_hw_stop(void) { assert(held); output[0] = output[1] = 0; }
static void tick(uint32_t at) {
    now_us = at; sleeps = 0u;
    ((void (*)(void *))cyclic_entry)(0);
    if (setjmp(sleep_again) == 0) ((void (*)(INT, void *))task_entry)(0, 0);
    assert(!held && !irq_off);
}
int main(int argc, char **argv) {
#if APP_TEST
    motion_timing_window_t unopened;
    assert(motion_timing_window_begin()==E_OBJ);
    assert(motion_timing_window_end(&unopened)==E_OBJ);
    assert(motion_timing_probe_capture(MOTION_PROBE_BEFORE_S0_PRINT)==E_OBJ);
#endif
    motion_status_t board_status;uint32_t board_us=123u;
    assert(motion_snapshot_try_read(0,&board_us)==E_PAR);
    assert(motion_snapshot_try_read(&board_status,0)==E_PAR);
    assert(motion_snapshot_try_read(&board_status,&board_us)==E_OBJ && board_us==123u);
    init_error = E_SYS;
    output[0] = output[1] = 100;
    assert(motion_init() == E_SYS && check_calls == 0u);
    assert(output[0] == 0 && output[1] == 0 && mutex_calls == 0u);
    assert(subsystem2_start() == E_OBJ && bench_calls == 0u);
    init_error = E_OK; check_error = E_SYS;
    assert(motion_init() == E_SYS && check_calls == 1u);
    assert(mutex_calls == 0u && task_entry == 0 && cyclic_entry == 0);
    assert(subsystem2_start() == E_OBJ && bench_calls == 0u && error_lines == 2u && wd_starts == 0u);
    check_error = E_OK;
    assert(motion_init() == E_OK && mutex_calls == 1u);
    assert(motion_set_speed(40,40)==E_OBJ); /* No driving before watchdog startup. */
    if (argc>1) {
        cyclic_error=strcmp(argv[1],"cyclic")==0;
        task_error=strcmp(argv[1],"task")==0;
        watchdog_error=strcmp(argv[1],"watchdog")==0 ? E_SYS : E_OK;
        assert(cyclic_error || task_error || watchdog_error);
        assert(subsystem2_start()==E_SYS && bench_calls==0u);
        assert(wd_starts==(watchdog_error ? 1u : 0u) && wd_feeds==0u);
        motion_status_t failed;
        assert(motion_status(&failed)==E_OK && !failed.watchdog_armed);
        assert(motion_set_speed(40,40)==E_OBJ && output[0]==0 && output[1]==0);
        if (watchdog_error) { tick(10000u); assert(wd_feeds==0u); }
        return 0;
    }
    assert(subsystem2_start() == E_OK && wd_starts==1u);
#if APP_BENCH
    assert(bench_calls == 1u);
#else
    assert(bench_calls == 0u);
#endif
    assert(motion_snapshot_try_read(&board_status,&board_us)==E_TMOUT && board_us==123u);
    tick(10000u);
    assert(notify_calls == 0u);
    assert(motion_notify_after_tick(-1) == E_PAR);
    assert(motion_notify_after_tick(4) == E_OK);
    const uint8_t left[] = {2, 3, 1, 0}, right[] = {1, 3, 2, 0};
    for (unsigned int i = 0u; i < 4u; ++i) {
        ab[0] = left[i]; encoder(16u, 8u, encoder_ctx);
        ab[1] = right[i]; encoder(4u, 8u, encoder_ctx);
    }
    motion_status_t s;
    assert(motion_status(&s) == E_OK && s.counts[0] == 0 && s.counts[1] == 0);
    tick(20000u);
    assert(motion_status(&s) == E_OK && s.counts[0] == 4 && s.counts[1] == 4);
    assert(motion_snapshot_try_read(&board_status,&board_us)==E_OK && !held);
    assert(board_us==20000u && board_status.ticks==2u && board_status.counts[0]==4 && board_status.counts[1]==4);
    assert(board_status.watchdog_armed && board_status.faults==s.faults);
    assert(s.ticks == 2u && s.gap_min_us == 10000u && s.tick_max_us == 7u);
    assert(motion_set_speed(150, 150) == E_OK);
    tick(30000u);
    assert(output[0] > 0 && output[1] > 0);
    assert(motion_estop() == E_OK && output[0] == 0 && output[1] == 0);
    tick(40000u);
    assert(output[0] == 0 && output[1] == 0);
    assert(motion_status(&s) == E_OK && (s.faults & MC_FAULT_ESTOP) != 0u);
    assert(motion_clear_faults() == E_OK);
    assert(motion_set_speed(40, 40) == E_OK);
    tick(150000u);
    assert(motion_status(&s) == E_OK && (s.faults & MC_FAULT_STALE) != 0u);
    assert(output[0] == 0 && output[1] == 0);
    queued = 2;
    tick(170000u);
    assert(motion_status(&s) == E_OK && s.missed_wakes == 2u && s.late_wakes == 2u);
    assert(s.gap_max_us == 110000u && wake_calls == 6u);
    assert(notify_calls == 5u); /* Exactly one per tick, even with queued wakes. */
    assert(motion_notify_after_tick(0) == E_OK);
    tick(180000u);
    assert(wake_calls == 7u && notify_calls == 5u);
    int accepted;
    assert(motion_set_servo(999, &accepted) == E_OK && accepted == 36);
    assert(motion_stop() == E_OK && output[0] == 0 && output[1] == 0);
    assert(motion_status(0) == E_PAR);
    assert(s.watchdog_armed && wd_starts==1u && wd_feeds==7u);
    ((void (*)(void *))cyclic_entry)(0); /* Stuck task: cyclic wake alone never feeds. */
    assert(wd_feeds==7u);
#if APP_TEST
    motion_timing_probe_t probe;
    assert(motion_timing_probe_read(MOTION_PROBE_BEFORE_S0_PRINT,&probe)==E_OBJ);
    assert(motion_timing_probe_capture((motion_probe_stage_t)-1)==E_PAR);
    assert(motion_timing_probe_capture(MOTION_PROBE_COUNT)==E_PAR);
    assert(motion_timing_probe_read(MOTION_PROBE_COUNT,&probe)==E_PAR);
    assert(motion_timing_probe_read(MOTION_PROBE_BEFORE_S0_PRINT,0)==E_PAR);
    assert(motion_timing_probe_capture(MOTION_PROBE_BEFORE_S0_PRINT)==E_OK);
    assert(motion_timing_probe_read(MOTION_PROBE_BEFORE_S0_PRINT,&probe)==E_OK);
    const motion_timing_probe_t saved=probe;
    motion_timing_window_t w;
    assert(motion_timing_window_begin()==E_OK && motion_timing_window_begin()==E_OBJ);
    assert(motion_timing_window_end(0)==E_PAR);
    queued=0;tick(250000u); /* Cross-start historical gap is outside the window. */
    tick(260000u);tick(271500u);
    queued=3;tick(280000u);
    assert(motion_timing_window_end(&w)==E_OK && motion_timing_window_end(&w)==E_OBJ);
    assert(w.ticks==4u && w.gaps==3u && w.gap_min_us==8500u && w.gap_max_us==11500u);
    assert(w.late_wakes==1u && w.missed_wakes==3u && w.tick_max_us==7u);
    assert(motion_status(&s)==E_OK && s.gap_max_us==110000u);
    assert(s.late_wakes==saved.late_wakes+2u && s.missed_wakes==saved.missed_wakes+3u);
    assert(motion_timing_window_begin()==E_OK);
    queued=0;tick(350000u);tick(360000u);tick(370000u);
    assert(motion_timing_window_end(&w)==E_OK && w.ticks==3u && w.gaps==2u);
    assert(w.gap_min_us==10000u && w.gap_max_us==10000u && w.late_wakes==0u && w.missed_wakes==0u);
    assert(motion_timing_probe_read(MOTION_PROBE_BEFORE_S0_PRINT,&probe)==E_OK);
    assert(!memcmp(&probe,&saved,sizeof(probe))); /* Deferred report is immutable. */
    assert(motion_status(&s)==E_OK && s.gap_max_us==110000u && s.late_wakes>0u && s.missed_wakes>0u);
    const unsigned int feeds_before=wd_feeds;
    assert(motion_watchdog_test_suppress_feed()==E_OK && !motion_watchdog_test_feed_stopped());
    tick(380000u);
    assert(wd_feeds==feeds_before+1u && test_begins==1u && motion_watchdog_test_feed_stopped());
    tick(390000u);
    assert(wd_feeds==feeds_before+1u && test_begins==1u); /* Live task, intentional TEST suppression. */
#endif
    return 0;
}
