#include "subsystem1.h"
#include "../platform/drv_time.h"

#if APP_TEST
/* Platform test runs in the initial task; no separate S1 task is created. */
EXPORT void subsystem1_test(void) {
    uint32_t timer_us = 0u, kernel_ms = 0u;
    const int er = drv_time_check_window(1000u, 1u, &timer_us, &kernel_ms);
    tm_printf((UB*)"[TEST] S0-U01 %s TIMER %lu us, kernel %lu ms (limit 1%%), error %d\n",
              er >= E_OK ? "PASS" : "FAIL", (unsigned long)timer_us,
              (unsigned long)kernel_ms, er);
}
#endif
