#include "subsystem2.h"
#include "motion.h"
#include "motor_ctl_robot.h"
#include "../test_report.h"
#include "../platform/drv_watchdog.h"
#include "../platform/drv_time.h"
#if APP_TEST
EXPORT void subsystem2_test(void) {
    TEST_INIT();
    motion_status_t before = {0}, after = {0};
    const int begin_ok = motion_status(&before);
    (void)tk_dly_tsk(5000);
    const int end_ok = motion_status(&after);
    const uint32_t count = after.ticks - before.ticks;
    TEST_CHECK("S2-U01", begin_ok >= E_OK && end_ok >= E_OK && count >= 499u && count <= 501u &&
               after.gap_max_us <= 11000u && after.missed_wakes == 0u,
               "5 s: 500 +/- 1 ticks, gap <= 11 ms, no queued wakes lost");
    tm_printf((UB*)"[TEST] S2-U01 ticks %lu gap %lu..%lu us tick_max_us %lu late %lu missed %lu\n",
              (unsigned long)count, (unsigned long)after.gap_min_us,
              (unsigned long)after.gap_max_us, (unsigned long)after.tick_max_us,
              (unsigned long)after.late_wakes, (unsigned long)after.missed_wakes);
    tm_printf((UB*)"[TEST] WARNING S2-U02: LIFT BOTH WHEELS before powering this TEST image.\n"
              "[TEST] In 2 s: brief 40 mm/s command, then no refresh; stale must stop PWM.\n");
    (void)tk_dly_tsk(2000);
    static const mc_config_t cfg = MC_ROBOT_CONFIG;
    (void)motion_clear_faults();
    const int set_ok = motion_set_speed(40, 40);
    (void)tk_dly_tsk((RELTIM)(cfg.stale_ms + 30u));
    const int status_ok = motion_status(&after);
    TEST_CHECK("S2-U02", set_ok >= E_OK && status_ok >= E_OK &&
               (after.faults & MC_FAULT_STALE) != 0u &&
               after.duty_permille[0] == 0 && after.duty_permille[1] == 0,
               "stale command sets MC_FAULT_STALE and both duties zero");
    (void)motion_stop();
    (void)motion_clear_faults();
    TEST_SUMMARY("subsystem2");
}
/* Called by usermain LAST, after S1..S5. Never periodically feeds. */
EXPORT void subsystem2_watchdog_test(void) {
    uint32_t previous;
    if (drv_watchdog_test_previous(&previous)) {
        tm_printf((UB*)"[TEST] S2-U05 previous boot: watchdog reset after %lu ms (last observed before reset; target 100 ms)\n",
                  (unsigned long)previous);
        tm_printf((UB*)"[TEST] S2-U05 %s: own timeout, previous elapsed sample 90..110 ms; no repeat reset this boot\n",
                  previous >= 90u && previous <= 110u ? "PASS" : "FAIL");
        return;
    }
    motion_status_t status;
    if (motion_status(&status) < E_OK || !status.watchdog_armed) {
        tm_printf((UB*)"[TEST] S2-U05 FAIL: watchdog not armed; no reset test.\n");
        return;
    }
    (void)motion_stop();
    tm_printf((UB*)"[TEST] WARNING S2-U05 LAST: LIFT BOTH WHEELS; intentional watchdog RESET.\n"
              "[TEST] In 1 s stop feeding after a completed tick; expect reset about 100 ms later.\n"
              "[TEST] Do not power-cycle: reconnect USB and wait for next boot's S2-U05 previous-boot line.\n");
    (void)tk_dly_tsk(1000u); /* Let USB deliver the warning while motion still feeds. */
    if (motion_watchdog_test_suppress_feed() < E_OK) {
        tm_printf((UB*)"[TEST] S2-U05 FAIL: feed suppression request rejected.\n");
        return;
    }
    const uint32_t requested = drv_time_ms();
    while (!motion_watchdog_test_feed_stopped()) {
        (void)tk_dly_tsk(1u); /* Allow the lower-priority motion task to acknowledge. */
        if (drv_time_ms() - requested > 200u) {
            tm_printf((UB*)"[TEST] S2-U05 FAIL: no final-tick acknowledgement.\n");
            return;
        }
    }
    /* A live CPU saves the last observation, without updating watchdog LOAD.
     * Busy sampling is intentional: no other task is needed for reset. */
    while (drv_watchdog_test_sample() < 500u) { }
    (void)motion_estop();
    tm_printf((UB*)"[TEST] S2-U05 FAIL: no watchdog reset within 500 ms.\n");
}
#endif
