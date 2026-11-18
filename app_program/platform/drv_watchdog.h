#ifndef DRV_WATCHDOG_H
#define DRV_WATCHDOG_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { DRV_RESET_POWER_RUN, DRV_RESET_WATCHDOG, DRV_RESET_OTHER } drv_watchdog_reset_t;
/* Core-0 task context. Start once, only after drv_time_init(); 1..8388 ms.
 * Only motion_task may feed. Debug halt pauses the watchdog. */
int drv_watchdog_start(uint32_t ms);
void drv_watchdog_feed(void);
drv_watchdog_reset_t drv_watchdog_last_reset(void);
const char *drv_watchdog_reset_name(void);
#if APP_TEST
/* TEST-only persistent result and elapsed sampling; these never feed. */
bool drv_watchdog_test_previous(uint32_t *ms);
int drv_watchdog_test_begin(void);
uint32_t drv_watchdog_test_sample(void);
#endif
#endif
