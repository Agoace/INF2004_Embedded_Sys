/* Kernel side only; SDK tick setup and register reads stay in drv_time_hw.c. */
#include <tk/tkernel.h>
#include "drv_time.h"
#include "drv_time_hw.h"
int drv_time_init(void) {
    /* Kernel startup selects the 12 MHz XOSC for clk_ref. */
    return drv_time_hw_init((uint32_t)XOSC_MHz) == 0 ? (int)E_OK : (int)E_SYS;
}
uint32_t drv_time_us(void) { return drv_time_hw_us(); }
uint32_t drv_time_ms(void) { return drv_time_hw_ms(); }
int drv_time_check_window(uint32_t window_ms, unsigned int tolerance_pct,
                          uint32_t *timer_us, uint32_t *kernel_ms) {
    if (window_ms == 0u || window_ms > 1000u || tolerance_pct > 100u) return (int)E_PAR;
    /* Start just after a kernel tick, not at an arbitrary phase within 10 ms.
     * Otherwise kernel-time quantization alone can exceed the 2% limit. */
    ER er = tk_dly_tsk(1);
    if (er < E_OK) return (int)er;
    SYSTIM before, after;
    er = tk_get_otm(&before);
    if (er < E_OK) return (int)er;
    const uint32_t start = drv_time_us();
    er = tk_dly_tsk((TMO)window_ms);
    if (er < E_OK) return (int)er;
    er = tk_get_otm(&after);
    if (er < E_OK) return (int)er;
    const uint32_t elapsed_us = drv_time_us() - start;
    /* Both windows are at most 1 s; unsigned subtraction also handles wrap. */
    const uint32_t elapsed_ms = after.lo - before.lo;
    if (timer_us) *timer_us = elapsed_us;
    if (kernel_ms) *kernel_ms = elapsed_ms;
    const uint64_t expected = (uint64_t)elapsed_ms * 1000u;
    const uint64_t diff = elapsed_us > expected ? elapsed_us - expected : expected - elapsed_us;
    return expected != 0u && diff * 100u <= expected * tolerance_pct ? (int)E_OK : (int)E_SYS;
}
int drv_time_check(void) { return drv_time_check_window(100u, 2u, 0, 0); }
