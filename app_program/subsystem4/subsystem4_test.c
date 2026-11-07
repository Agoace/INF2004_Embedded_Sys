/* TEST-only tilt check. Register/axis ideas: Jaison, b2a6bb6.
 * No vehicle, encoder, MQTT, calibration or hump path is connected. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "subsystem4.h"
#if APP_TEST
#include "imu_probe.h"
#include "imu_hw.h"
#include "imu_mark.h"
#include "../platform/drv_time.h"

static ID runner;

static void release(void *context)
{
    (void)context;
    (void)tk_wup_tsk(runner);
}

EXPORT void subsystem4_test(void)
{
    imu_probe_t probe;
    imu_units_t units = {0};
    uint32_t previous;
    uint32_t gap_max = 0u;
    uint32_t missed = 0u;
    ID cyclic;
#if APP_IMU_STREAM
    imu_mark_t marker = {0};
    uint32_t marks = 0u;
#endif
    T_CCYC config = {.cycatr = TA_HLNG, .cychdr = (FP)release,
                     .cyctim = 10u, .cycphs = 10u};

#if TK_SUPPORT_SMP
    config.cycatr |= TA_ASSPRC;
    config.assprc = TP_PRC1;
#endif
#if APP_IMU_STREAM
    imu_hw_mark_init();
    tm_printf((UB *)"[TEST] IMU_STREAM until power-off; GP20 prints MARK. "
              "Other tests and destructive watchdog reset are skipped.\n");
#endif
    tm_printf((UB *)"[TEST] S4-U01 operator tilt check: GY-511. "
              "I2C0 GP0/1, 400 kHz, accel 100 Hz HR 2g, mag 75 Hz.\n");
    tm_printf((UB *)"[TEST] S4 no gyro; uncalibrated +X right +Y forward "
              "+Z up. Lift wheels; GP0/1 must not have a UART adapter.\n");
    imu_probe_init(&probe);
    runner = tk_get_tid();
    cyclic = tk_cre_cyc(&config);
    if ((cyclic < E_OK) || (tk_sta_cyc(cyclic) < E_OK)) {
        tm_printf((UB *)"[TEST] S4-U01 FAIL: cannot start 10 ms cyclic\n");
        if (cyclic >= E_OK) {
            (void)tk_del_cyc(cyclic);
        }
        return;
    }
    previous = drv_time_us();
    for (unsigned int cycle = 0u;
#if APP_IMU_STREAM
         ;
#else
         cycle < 3000u;
#endif
         ++cycle) {
        uint32_t now;
        uint32_t gap;
        bool valid;
        INT extra;

        if (tk_slp_tsk(100u) < E_OK) {
            tm_printf((UB *)"[TEST] S4-U01 FAIL: cyclic wake timeout\n");
            break;
        }
        extra = tk_can_wup(TSK_SELF);
        if (extra > 0) {
            missed += (uint32_t)extra;
        }
        now = drv_time_us();
        gap = now - previous;
        previous = now;
        if (gap > gap_max) {
            gap_max = gap;
        }
#if APP_IMU_STREAM
        if (imu_mark_update(&marker, imu_hw_mark_down(), drv_time_ms())) {
            ++marks;
            tm_printf((UB *)"[S4] MARK %lu\n", (unsigned long)marks);
        }
#endif
        valid = imu_probe_read(&probe, drv_time_ms(), &units);
        if ((cycle % 10u) == 9u) {
            float gravity;
            float pitch;
            float roll;
            const bool tilt = imu_tilt(units.accel_mg, &gravity, &pitch, &roll);

            tm_printf((UB *)"[S4] valid %u mag_valid %u a_mg %ld %ld %ld "
                      "m_mgauss %ld %ld %ld g_mg %ld pitch_ddeg %ld "
                      "roll_ddeg %ld read_us %lu max_us %lu "
                      "reads %lu faults %lu nack %lu timeout %lu "
                      "id %lu cfg %lu gap_max_us %lu missed %lu "
                      "ctrl1 %02x ctrl4 %02x\n",
                      (unsigned int)(valid && tilt),
                      (unsigned int)units.mag_valid,
                      (long)units.accel_mg[0], (long)units.accel_mg[1],
                      (long)units.accel_mg[2], (long)units.mag_mgauss[0],
                      (long)units.mag_mgauss[1], (long)units.mag_mgauss[2],
                      (long)gravity, (long)(pitch * 10.0f),
                      (long)(roll * 10.0f), (unsigned long)probe.last_us,
                      (unsigned long)probe.max_us, (unsigned long)probe.reads,
                      (unsigned long)probe.faults, (unsigned long)probe.nacks,
                      (unsigned long)probe.timeouts,
                      (unsigned long)probe.identity_errors,
                      (unsigned long)probe.config_errors,
                      (unsigned long)gap_max, (unsigned long)missed,
                      (unsigned int)probe.ctrl1, (unsigned int)probe.ctrl4);
        }
    }
    (void)tk_stp_cyc(cyclic);
    (void)tk_del_cyc(cyclic);
    (void)tk_can_wup(TSK_SELF);
    tm_printf((UB *)"[TEST] S4-U01 operator judgement required; "
              "read_total_us %lu reads %lu max_us %lu\n",
              (unsigned long)probe.total_us, (unsigned long)probe.reads,
              (unsigned long)probe.max_us);
}
#endif
