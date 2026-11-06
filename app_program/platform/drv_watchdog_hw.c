/* SDK-only. RP2040 SDK watchdog paths used here need no SDK runtime. */
#include "drv_watchdog_hw.h"
#include "hardware/watchdog.h"
#define TEST_MAGIC 0x53325535u /* S2U5: scratch0/1 reserved by this app. */
int drv_watchdog_hw_start(uint32_t ms, uint32_t ref_mhz) {
    const uint32_t mask = WATCHDOG_TICK_ENABLE_BITS | WATCHDOG_TICK_RUNNING_BITS | WATCHDOG_TICK_CYCLES_BITS;
    if (ms == 0u || ms > 8388u || ref_mhz == 0u || ref_mhz > WATCHDOG_TICK_CYCLES_BITS ||
        (watchdog_hw->tick & mask) != (WATCHDOG_TICK_ENABLE_BITS | WATCHDOG_TICK_RUNNING_BITS | ref_mhz)) return -1;
    /* SDK applies RP2040-E1 x2 load, PSM reset selection, scratch4 magic
     * and pause on DBG0/DBG1/JTAG. No tick reconfiguration here. */
    watchdog_enable(ms, true);
    return (watchdog_hw->ctrl & WATCHDOG_CTRL_ENABLE_BITS) != 0u ? 0 : -1;
}
void drv_watchdog_hw_feed(void) { watchdog_update(); }
drv_watchdog_reset_t drv_watchdog_hw_reset(uint32_t *previous_test_ms) {
    drv_watchdog_reset_t reason = DRV_RESET_POWER_RUN;
    if (watchdog_caused_reboot()) {
        reason = watchdog_enable_caused_reboot() && watchdog_hw->reason == WATCHDOG_REASON_TIMER_BITS
                 ? DRV_RESET_WATCHDOG : DRV_RESET_OTHER;
    }
    if (previous_test_ms) *previous_test_ms = UINT32_MAX;
    if (watchdog_hw->scratch[0] == TEST_MAGIC) {
        if (previous_test_ms && reason == DRV_RESET_WATCHDOG && watchdog_hw->scratch[1] <= 500u)
            *previous_test_ms = watchdog_hw->scratch[1];
        watchdog_hw->scratch[0] = 0u; /* One result, not an endless TEST reset loop. */
    }
    return reason;
}
#if APP_TEST
void drv_watchdog_hw_test_begin(void) {
    watchdog_hw->scratch[1] = 0u;
    watchdog_hw->scratch[0] = TEST_MAGIC;
}
void drv_watchdog_hw_test_sample(uint32_t ms) { watchdog_hw->scratch[1] = ms; }
#endif
