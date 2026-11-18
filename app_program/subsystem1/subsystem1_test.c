#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "subsystem1.h"
#include "../platform/drv_time.h"
#include "telemetry.h"
#include "../subsystem2/motion.h"
#if TM_MQTT
#include "robot_health.h"
#include "../platform/idle_meter.h"
#endif

#if APP_TEST
/* Tests run in the initial task while the low-priority telemetry task waits. */
EXPORT void subsystem1_test(void) {
    uint32_t timer_us = 0u, kernel_ms = 0u;
    const int er = drv_time_check_window(1000u, 1u, &timer_us, &kernel_ms);
    (void)motion_timing_probe_capture(MOTION_PROBE_BEFORE_S0_PRINT);
    tm_printf((UB*)"[TEST] S0-U01 %s TIMER %lu us, kernel %lu ms (limit 1%%), error %d\n",
              er >= E_OK ? "PASS" : "FAIL", (unsigned long)timer_us,
              (unsigned long)kernel_ms, er);
    (void)motion_timing_probe_capture(MOTION_PROBE_AFTER_S0_PRINT);
    telemetry_stats_t before,after;telemetry_stats(&before);
    telemetry_message_t message={.kind=TELEMETRY_BEGIN};
    (void)motion_timing_probe_capture(MOTION_PROBE_S1_BURST_BEGIN);
    (void)telemetry_send(&message);message.kind=TELEMETRY_ROW;
    unsigned int accepted=0u;
    for (unsigned int i=0u;i<100u;++i) if (telemetry_send(&message)>=E_OK) ++accepted;
    (void)motion_timing_probe_capture(MOTION_PROBE_S1_BURST_END);
    (void)tk_dly_tsk(20u); /* Let the consumer run after the bounded saturation burst. */
    (void)motion_timing_probe_capture(MOTION_PROBE_AFTER_S1_YIELD);
    telemetry_stats(&after);
    tm_printf((UB*)"[TEST] S1-U02 %s: nonblocking telemetry accepted %u lost %lu rows %lu\n",
        accepted>0u && after.lost>before.lost && after.rows>0u ? "PASS" : "FAIL",accepted,
        (unsigned long)(after.lost-before.lost),(unsigned long)after.rows);
    (void)motion_timing_probe_capture(MOTION_PROBE_AFTER_S1_PRINT);
#if TM_MQTT
    /* Observation only: establish an idle-board range from target evidence. */
    (void)tk_dly_tsk(2200u);
    rm_heartbeat_t health;
    uint32_t timestamp_us;
    const int load_result = robot_health_try_read(&health, &timestamp_us);
    tm_printf((UB *)"[TEST] S1-U07 %s: CPU load %u permille, peak %lu iter/s "
                     "(idle board, wheels lifted; range not established)\n",
              load_result >= E_OK && health.cpu_load_permille <= 1000u
                  ? "PASS" : "FAIL",
              load_result >= E_OK ? health.cpu_load_permille : 65535u,
              (unsigned long)idle_meter_peak_rate());
#endif
}
#endif
