/* Kernel side only. No other periodic task may call drv_watchdog_feed(). */
#include <tk/tkernel.h>
#include "drv_watchdog.h"
#include "drv_watchdog_hw.h"
#include "drv_time.h"
static bool captured, started;
static drv_watchdog_reset_t last_reset;
#if APP_TEST
static uint32_t previous_ms = UINT32_MAX, test_start_us;
#endif
static void capture(void) {
    if (captured) return;
#if APP_TEST
    last_reset = drv_watchdog_hw_reset(&previous_ms);
#else
    last_reset = drv_watchdog_hw_reset(0);
#endif
    captured = true;
}
drv_watchdog_reset_t drv_watchdog_last_reset(void) { capture(); return last_reset; }
const char *drv_watchdog_reset_name(void) {
    switch (drv_watchdog_last_reset()) {
        case DRV_RESET_POWER_RUN: return "power/RUN";
        case DRV_RESET_WATCHDOG: return "watchdog";
        default: return "other";
    }
}
int drv_watchdog_start(uint32_t ms) {
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) return (int)E_CTX;
#endif
    if (started) return (int)E_OBJ;
    if (!drv_time_ready()) return (int)E_OBJ;
    if (ms == 0u || ms > 8388u) return (int)E_PAR;
    capture(); /* Before watchdog_enable replaces scratch4 with our own magic. */
    if (drv_watchdog_hw_start(ms, (uint32_t)XOSC_MHz) != 0) return (int)E_SYS;
    started = true;
    return (int)E_OK;
}
void drv_watchdog_feed(void) { if (started) drv_watchdog_hw_feed(); }
#if APP_TEST
bool drv_watchdog_test_previous(uint32_t *ms) {
    capture();
    if (previous_ms == UINT32_MAX || last_reset != DRV_RESET_WATCHDOG) return false;
    if (ms) *ms = previous_ms;
    return true;
}
int drv_watchdog_test_begin(void) {
    if (!started) return (int)E_OBJ;
    test_start_us = drv_time_us();
    drv_watchdog_hw_test_begin();
    return (int)E_OK;
}
uint32_t drv_watchdog_test_sample(void) {
    const uint32_t elapsed = (drv_time_us() - test_start_us) / 1000u;
    drv_watchdog_hw_test_sample(elapsed);
    return elapsed;
}
#endif
