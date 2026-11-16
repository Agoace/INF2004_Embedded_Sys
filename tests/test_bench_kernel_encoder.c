/* Real dispatcher + real motion callback/controller; host IRQ/timer doubles.
 * Does not emulate NVIC exception entry or prove target timing. */
#include <assert.h>
#include <setjmp.h>
#include <tk/tkernel.h>
#include "../app_program/bench_kernel/bench_gpio.h"
#include "../app_program/platform/drv_gpio_irq.h"
#include "../app_program/subsystem2/drv_motor_hw.h"
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem2/subsystem2.h"
static FP gpio_handler, task_entry, cyclic_entry;
static jmp_buf done;
static uint32_t now_us;
static unsigned int sleeps, feeds, dispatches, irq_off;
static int held;
int drv_time_init(void) { return E_OK; }
int drv_time_check(void) { return E_OK; }
uint32_t drv_time_us(void) { return now_us; }
uint32_t drv_time_ms(void) { return now_us / 1000u; }
int drv_watchdog_start(uint32_t ms) { assert(ms == 100u); return E_OK; }
void drv_watchdog_feed(void) { ++feeds; }
int tm_printf(const UB *fmt, ...) { (void)fmt; return 0; }
int motion_bench_start(void) { assert(0); return E_SYS; }
INT tk_get_prc(void) { return 1; }
void mock_di(UINT *irq) { assert(!irq_off); *irq = 0u; irq_off = 1u; }
void mock_ei(UINT irq) { assert(irq == 0u && irq_off); irq_off = 0u; }
ER tk_def_int(UINT irq, const T_DINT *d)
{
    assert(irq == 13u && d->intatr == TA_HLNG);
    gpio_handler = d->inthdr;
    return E_OK;
}
void DisableInt(UINT irq) { assert(irq == 13u); }
void EnableInt(UINT irq, INT pri) { assert(irq == 13u && pri == 1); }
void ClearInt(UINT irq) { assert(irq == 13u); }
ID tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
ER tk_del_mtx(ID id) { assert(id == 1); return E_OK; }
ER tk_loc_mtx(ID id, TMO t)
{
    assert(id == 1 && t == TMO_FEVR && !held); held = 1; return E_OK;
}
ER tk_unl_mtx(ID id) { assert(id == 1 && held); held = 0; return E_OK; }
ID tk_cre_cyc(const T_CCYC *c)
{
    assert(c->cyctim == 10u); cyclic_entry = c->cychdr; return 2;
}
ER tk_sta_cyc(ID id) { assert(id == 2); return E_OK; }
ID app_start_task(FP f, PRI pri, UW prc, const char *name)
{
    (void)name;
    assert(pri == 10 && prc == TP_PRC1); task_entry = f; return 3;
}
ER tk_slp_tsk(TMO t)
{
    assert(t == TMO_FEVR);
    if (sleeps++ != 0u) { longjmp(done, 1); }
    return E_OK;
}
INT tk_can_wup(ID id) { assert(id == TSK_SELF); return 0; }
ER tk_wup_tsk(ID id) { assert(id == 3); return E_OK; }
bool bk_runtime_button(unsigned int pin) { (void)pin; return false; }
static void tick(uint32_t at)
{
    now_us = at; sleeps = 0u;
    ((void (*)(void *))cyclic_entry)(0);
    if (setjmp(done) == 0) { ((void (*)(INT, void *))task_entry)(0, 0); }
}
static void dispatch(void)
{
    assert(gpio_handler && !irq_off);
    ((void (*)(UINT))gpio_handler)(13u);
    ++dispatches;
}
int main(void)
{
    bk_gpio_reset();
    assert(motion_init() == E_OK);
    assert(subsystem2_start() == E_OK);
    tick(10000u);
    const uint8_t left[4] = {2u, 3u, 1u, 0u};
    const uint8_t right[4] = {1u, 3u, 2u, 0u};
    for (unsigned int i = 0u; i < 4u; ++i) {
        assert(bk_gpio_edge(0u, left[i]));
        assert(bk_gpio_edge(1u, right[i]));
        /* Pending-wheel state cannot advance before the real ACK/callback. */
        assert(!bk_gpio_edge(0u, left[(i + 1u) % 4u]));
        dispatch();
    }
    assert(dispatches == 4u && bk_gpio_acked() == 8u);
    dispatch(); /* Spurious pend cannot create an encoder count. */
    assert(bk_gpio_acked() == 8u);
    tick(20000u);
    motion_status_t s;
    assert(motion_status(&s) == E_OK);
    assert(s.counts[0] == 4 && s.counts[1] == 4 && feeds == 2u);
    for (unsigned int i = 0u; i < 4u; ++i) {
        assert(bk_gpio_edge(0u, right[i]));
        assert(bk_gpio_edge(1u, left[i]));
        dispatch();
    }
    tick(30000u);
    assert(motion_status(&s) == E_OK);
    assert(s.counts[0] == 0 && s.counts[1] == 0);
    assert(bk_gpio_acked() == 16u && feeds == 3u);
    return 0;
}
