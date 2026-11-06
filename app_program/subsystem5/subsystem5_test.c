#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "../platform/drv_time.h"
#if APP_TEST
EXPORT void subsystem5_test(void) {
    uint16_t mm[20]; unsigned int valid = 0u, no_echo = 0u;
    tm_printf((UB*)"[TEST] S5-U01: fixed object 60..511 mm, compare with a ruler.\n");
    for (unsigned int i = 0u; i < 20u; ++i) {
        (void)tk_dly_tsk(100u);
        if (!drv_ultrasonic_trigger()) { ++no_echo; continue; }
        const uint32_t start = drv_time_ms();
        while (drv_ultrasonic_busy() && drv_time_ms() - start < 50u) {
            (void)drv_ultrasonic_service();
            if (drv_ultrasonic_busy()) (void)tk_dly_tsk(1u);
        }
        const ultrasonic_result_t r = drv_ultrasonic_latest();
        if (ultrasonic_result_status(r) != ULTRASONIC_STATUS_VALID || ultrasonic_result_too_near(r)) ++no_echo;
        else mm[valid++] = ultrasonic_result_distance(r);
    }
    for (unsigned int i = 1u; i < valid; ++i) {
        const uint16_t x = mm[i]; unsigned int j = i;
        while (j > 0u && mm[j - 1u] > x) { mm[j] = mm[j - 1u]; --j; }
        mm[j] = x;
    }
    tm_printf((UB*)"[TEST] S5-U01 20 pings: valid %u unknown %u median %u min %u max %u mm; manual check required\n",
              valid, no_echo, valid ? (unsigned int)mm[valid / 2u] : 0u,
              valid ? (unsigned int)mm[0] : 0u, valid ? (unsigned int)mm[valid - 1u] : 0u);
}
#endif
