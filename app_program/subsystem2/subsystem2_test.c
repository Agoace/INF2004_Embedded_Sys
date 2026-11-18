#include "subsystem2.h"
#include "motion.h"
#include "motor_ctl_robot.h"
#include "../test_report.h"
#include "../platform/drv_watchdog.h"
#include "../platform/drv_time.h"
#if TM_WIFI_CYW43
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#endif
#if APP_TEST
static void timing_probe_report(void) {
    static const char *const names[MOTION_PROBE_COUNT] = {
        "before_S0_print", "after_S0_print", "S1_burst_begin", "S1_burst_end",
        "after_S1_yield", "after_S1_print", "S2_entry", "S2_exit"
    };
    tm_printf((UB*)"[TEST] PROBE begin: buffered lifetime snapshots (not window values)\n");
    for (unsigned int i=0u;i<MOTION_PROBE_COUNT;++i) {
        motion_timing_probe_t p;
        const int er=motion_timing_probe_read((motion_probe_stage_t)i,&p);
        if (er<E_OK) { tm_printf((UB*)"[TEST] PROBE %s unavailable error %d\n",names[i],er);continue; }
        tm_printf((UB*)"[TEST] PROBE %s us %lu ticks %lu gap %lu..%lu us tick_max_us %lu late %lu missed %lu\n",
            names[i],(unsigned long)p.us,(unsigned long)p.ticks,(unsigned long)p.gap_min_us,
            (unsigned long)p.gap_max_us,(unsigned long)p.tick_max_us,
            (unsigned long)p.late_wakes,(unsigned long)p.missed_wakes);
    }
    tm_printf((UB*)"[TEST] PROBE end\n");
}
EXPORT void subsystem2_test(void) {
    TEST_INIT();
    motion_status_t after = {0};
    motion_timing_window_t window = {0};
    (void)motion_timing_probe_capture(MOTION_PROBE_S2_ENTRY);
    const int begin_ok = motion_timing_window_begin();
#if TM_WIFI_CYW43
    T_CYW43_TIMING wifi={0},wifi_lifetime={0};
    const int wifi_begin=cyw43_utk_timing_window_begin();
#endif
    (void)tk_dly_tsk(5000);
    const int end_ok = motion_timing_window_end(&window);
#if TM_WIFI_CYW43
    const int wifi_end=cyw43_utk_timing_window_end(&wifi);
    (void)cyw43_utk_timing_read(&wifi_lifetime);
#endif
    (void)motion_timing_probe_capture(MOTION_PROBE_S2_EXIT);
    TEST_CHECK("S2-U01", begin_ok >= E_OK && end_ok >= E_OK && window.ticks >= 499u && window.ticks <= 501u &&
               window.gap_max_us <= 11000u && window.missed_wakes == 0u,
               "5 s: 500 +/- 1 ticks, gap <= 11 ms, no queued wakes lost");
    tm_printf((UB*)"[TEST] S2-U01 window ticks %lu gaps %lu gap %lu..%lu us tick_max_us %lu late %lu missed %lu\n",
              (unsigned long)window.ticks,(unsigned long)window.gaps,(unsigned long)window.gap_min_us,
              (unsigned long)window.gap_max_us,(unsigned long)window.tick_max_us,
              (unsigned long)window.late_wakes,(unsigned long)window.missed_wakes);
    motion_timing_probe_t lifetime;
    const int info_ok=motion_timing_probe_read(MOTION_PROBE_S2_EXIT,&lifetime);
    if (info_ok>=E_OK)
        tm_printf((UB*)"[TEST] S2-U01 INFO lifetime ticks %lu gap %lu..%lu us tick_max_us %lu late %lu missed %lu\n",
            (unsigned long)lifetime.ticks,(unsigned long)lifetime.gap_min_us,(unsigned long)lifetime.gap_max_us,
            (unsigned long)lifetime.tick_max_us,(unsigned long)lifetime.late_wakes,(unsigned long)lifetime.missed_wakes);
    else tm_printf((UB*)"[TEST] S2-U01 INFO lifetime unavailable error %d\n",info_ok);
    timing_probe_report();
#if TM_WIFI_CYW43
    tm_printf((UB*)"[TEST] S2-U01 INFO WiFi wall window valid %u polls %lu gaps %lu max_gap_us %lu gaps_over_10000 %lu work_max_us %lu work_total_us %lu probe_max_us %lu; lifetime polls %lu max_gap_us %lu work_max_us %lu\n",
        (unsigned int)(wifi_begin>=E_OK && wifi_end>=E_OK),(unsigned long)wifi.polls,(unsigned long)wifi.gaps,
        (unsigned long)wifi.gap_max_us,(unsigned long)wifi.gaps_over_10000,(unsigned long)wifi.work_max_us,
        (unsigned long)wifi.work_total_us,(unsigned long)wifi.probe_max_us,(unsigned long)wifi_lifetime.polls,
        (unsigned long)wifi_lifetime.gap_max_us,(unsigned long)wifi_lifetime.work_max_us);
#endif
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
