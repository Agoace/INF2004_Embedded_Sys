/* Kernel owns the alarm1 HLL handler; no task/kernel call in the ISR. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "../platform/drv_time.h"
#include "../platform/run_timing.h"
#include "bench_generator.h"
#include "bench_runtime.h"
#include "bench_timer_hw.h"
#define WINDOW_US 600000000u
static uint32_t stopped_at;
static uint32_t stopped_valid;
static void generator_irq(UINT intno)
{
    (void)intno;
    const uint32_t entry = drv_time_us();
    bk_timer_ack();
    const unsigned int edges = bk_gen_tick(entry);
    if (edges != 0u) { bk_gpio_pend(); }
    if (bk_gen_active() && !bk_timer_arm(bk_gen_next())) {
        bk_gen_abort(drv_time_us());
    }
    if (!bk_gen_active()) {
        bk_timer_enable(false);
        if (bk_gen_failed()) { rt_tl((rt_tl_site_t)11, drv_time_us(), 0u); }
        __atomic_store_n(&stopped_at, drv_time_us(), __ATOMIC_RELAXED);
        __atomic_store_n(&stopped_valid, 1u, __ATOMIC_RELEASE);
    }
    rt_tl((rt_tl_site_t)10, drv_time_us(), edges);
    bk_gen_account(drv_time_us() - entry);
}
extern void __real_app_startup_complete_publish(void);
void __wrap_app_startup_complete_publish(void)
{
    __real_app_startup_complete_publish();
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) { return; }
#endif
    DisableInt(1u);
    bk_timer_enable(false);
    const T_DINT irq = {.intatr = TA_HLNG, .inthdr = (FP)generator_irq};
    if (tk_def_int(1u, &irq) < E_OK) { return; }
    ClearInt(1u);
    if (!bk_gen_start(drv_time_us() + 100000u, WINDOW_US)) { return; }
    if (!bk_timer_arm(bk_gen_next())) {
        bk_gen_abort(drv_time_us());
        bk_gen_publish();
        return;
    }
    bk_timer_enable(true);
    EnableInt(1u, 1); /* Same existing F-NVIC priority behavior; no repair. */
}
bool bk_runtime_button(unsigned int pin)
{
    if (pin != 20u ||
        __atomic_load_n(&stopped_valid, __ATOMIC_ACQUIRE) == 0u) {
        return false;
    }
    const uint32_t elapsed = drv_time_us() -
        __atomic_load_n(&stopped_at, __ATOMIC_RELAXED);
    /* Native core0 button polling publishes only after pending IRQs and
     * a final10ms motion tick can drain, including a fail-closed stop. */
    if (elapsed >= 20000u &&
        __atomic_load_n(&stopped_valid, __ATOMIC_RELAXED) == 1u) {
        bk_gen_publish();
        __atomic_store_n(&stopped_valid, 2u, __ATOMIC_RELEASE);
    }
    return elapsed >= 500000u && elapsed < 700000u;
}
#endif
