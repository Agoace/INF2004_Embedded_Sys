/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	Subsystem init/start order stays unchanged. Empty subsystems create no
 *	tasks. Subsystem 2 adds the motion task in the motion branch.
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "platform/drv_time.h"
#include "vehicle/vehicle_task.h"
#include "platform/app_task.h"
#include "subsystem4/imu_settings.h"
#if TM_MQTT
#include "platform/idle_meter.h"
#include "platform/boot_id.h"
#endif

#include "vehicle/junction_source.h"

/* TEST work runs below motion/sensors and above telemetry (30). */
#define TEST_RUNNER_PRIORITY 20

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc comes from app_prc_core0() or app_prc_core1(); every subsystem still
 *   passes core 0, and platform/app_task.c rejects any task placed on a core its
 *   slot does not allow.
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
    const ID tid = app_task_start(entry, pri, prc, name);
    CHECK(tid, name);
    return tid;
}

/*
 * Initialize platform time, create objects, then start application tasks.
 */
EXPORT INT usermain(void)
{
    T_RVER ver;

    /* Kernel hardware startup sets 1 us TIMER ticks; verify idempotently. */
    CHECK(drv_time_init(), "drv_time_init");
#if TM_MQTT
    /* Before application startup: no watchdog or acquisition deadline yet. */
    CHECK(idle_meter_calibrate(), "CPU meter calibration (noncritical)");
    CHECK(boot_id_init(), "boot observation ID (noncritical)");
#endif

#if !APP_BENCH && !APP_TEST
    tm_printf((UB *)"[JUNCTION] source FAKE cmd %s forced_corner "
                    "bar_clear_mm %u N_mm %u black_min %u "
                    "D skip_first_left\n",
              JUNCTION_TEST_CMD_NAME, JA_BAR_CLEAR_MM, JA_CONFIRM_MM,
              JA_BLACK_MIN);
#endif
#if !APP_BENCH
    tm_printf((UB *)"[S4] hump_source %u %s\n",
              (unsigned int)IMU_HUMP_SOURCE, IMU_HUMP_SOURCE_NAME);
#endif
    /* tk_ref_ver: ask the kernel who it is */
    tk_ref_ver(&ver);
    tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                    " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
    tm_printf((UB *)"single-core build\n");
#endif

    /* Phase 1: every subsystem creates its kernel objects */
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem5_init(), "subsystem5_init");

    /* Phase 2: every subsystem starts its tasks */
    CHECK(subsystem1_start(), "subsystem1_start");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem5_start(), "subsystem5_start");
    CHECK(vehicle_task_start(), "vehicle_task_start");

    tm_printf((UB *)"all subsystems started\n\n");

#if APP_TEST
    /* Startup stays at priority 1. TEST work must be below motion/sensors,
	   but above telemetry30 so its nonblocking saturation test still fills. */
    const ER test_priority = tk_chg_pri(TSK_SELF, TEST_RUNNER_PRIORITY);
    CHECK(test_priority, "TEST runner priority 20");
    if (test_priority >= E_OK) {
        tm_printf((UB *)"[TEST] === test run start (runner priority 20) ===\n");
#if APP_IMU_STREAM
        /* Operator-only S4 stream: no S2 command or destructive reset test. */
        subsystem4_test();
#else
        subsystem1_test();
        subsystem2_test();
        subsystem3_test();
        subsystem4_test();
        subsystem5_test();
        app_lock_stress_run();
        log_stream_admission_selftest();
        app_task_report();
        /* Destructive reset check follows every other subsystem test. */
        subsystem2_watchdog_test();
        tm_printf((UB *)"[TEST] === test run end ===\n");
#endif /* APP_IMU_STREAM */
    } else {
        tm_printf((UB *)"[TEST] ABORT: cannot set runner priority 20; tests "
                        "not run\n");
    }
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
