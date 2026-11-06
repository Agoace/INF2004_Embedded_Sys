/* Host scheduler/hardware doubles: checks the glue, not real-time behaviour. */
#include <assert.h>
#include <setjmp.h>
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
static unsigned int check_calls;
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
ER tk_sta_cyc(ID id) { assert(id == 2); return E_OK; }
ID app_start_task(FP entry, PRI pri, UW prc, const char *name) {
    (void)name; assert(pri == 10 && prc == TP_PRC1); task_entry = entry; return 3;
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
    (void)servo; assert(held && !irq_off); output[0] = duty[0]; output[1] = duty[1]; now_us += 7u;
}
void drv_motor_hw_stop(void) { assert(held); output[0] = output[1] = 0; }
static void tick(uint32_t at) {
    now_us = at; sleeps = 0u;
    ((void (*)(void *))cyclic_entry)(0);
    if (setjmp(sleep_again) == 0) ((void (*)(INT, void *))task_entry)(0, 0);
    assert(!held && !irq_off);
}
int main(void) {
    init_error = E_SYS;
    output[0] = output[1] = 100;
    assert(motion_init() == E_SYS && check_calls == 0u);
    assert(output[0] == 0 && output[1] == 0 && mutex_calls == 0u);
    assert(subsystem2_start() == E_OBJ && bench_calls == 0u);
    init_error = E_OK; check_error = E_SYS;
    assert(motion_init() == E_SYS && check_calls == 1u);
    assert(mutex_calls == 0u && task_entry == 0 && cyclic_entry == 0);
    assert(subsystem2_start() == E_OBJ && bench_calls == 0u && error_lines == 2u);
    check_error = E_OK;
    assert(motion_init() == E_OK && mutex_calls == 1u);
    assert(subsystem2_start() == E_OK && bench_calls == 1u);
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
    return 0;
}
