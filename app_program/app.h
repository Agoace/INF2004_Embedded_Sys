/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest. Motion 10 is the highest periodic
 *  task; line 11, sonar 12 (or bench 11), vehicle 13, IMU 14, Wi-Fi/TEST 20,
 *  kernel USB console 21, telemetry 30, health 31, CPU meter 32.
 *  USB is below every driving task. See TEAM.md for ownership.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE		2048	/* legacy default; app_task.c registers actual sizes */

/* Physical core a registered task or cyclic handler may run on.
   Movement and hardware owners stay on core 0.  Observers may move to core 1
   only in a two-core build (TK_MAX_CORE > 1), and the core-migration plan moves
   them one phase at a time. */
typedef enum {
	APP_CORE_0 = 0,
	APP_CORE_1 = 1
} app_core_t;

/* PRC value for a core.  TP_PRC1 is core 0, so core 1 is TP_PRC1 + 1.  In a
   single-core build both answers are TP_PRC1, because there is nowhere else to
   run.  A cyclic handler must use the same core as the task it feeds. */
IMPORT UW app_prc_core0(void);
IMPORT UW app_prc_core1(void);

/* Create/start a registered task.  prc must be app_prc_core0() or
   app_prc_core1(), and it must agree with the placement recorded in
   platform/app_task.c, which rejects a mismatch with E_CTX.
   app_main.c delegates to the static-stack helper in platform/app_task.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem2_watchdog_test(void); /* destructive S2-U05; run last */
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
/* Phase 1b: handler-versus-task lock stress. See docs/kernel/CORE_PLACEMENT.md */
IMPORT void app_lock_stress_run(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

#endif /* APP_H */
