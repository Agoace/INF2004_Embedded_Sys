#ifndef DRV_WATCHDOG_HW_H
#define DRV_WATCHDOG_HW_H
#include "drv_watchdog.h"
int drv_watchdog_hw_start(uint32_t ms, uint32_t ref_mhz);
void drv_watchdog_hw_feed(void);
/* Snapshot before enable overwrites SDK scratch[4]; consume our TEST marker. */
drv_watchdog_reset_t drv_watchdog_hw_reset(uint32_t *previous_test_ms);
#if APP_TEST
void drv_watchdog_hw_test_begin(void);
void drv_watchdog_hw_test_sample(uint32_t ms);
#endif
#endif
