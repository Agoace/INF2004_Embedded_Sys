/**
 * @file motion_task_utkernel.c
 * @brief μT-Kernel 3.0 wrapper for the Motion Control Task (priority 10).
 *
 * TEMPLATE: the repository does not contain the μT-Kernel port yet, so this
 * file is compiled only when MOTION_USE_UTKERNEL is defined. When the port
 * lands, add this file and -DMOTION_USE_UTKERNEL to the RTOS target and
 * call motion_task_create() from usermain().
 *
 * Mapping to the Week 6 design:
 *  - 20 ms cyclic handler (NFR17) wakes the task; the handler sets a flag
 *    so the task knows that this wake-up is a PID deadline.
 *  - A queued command also wakes the task (motion_set_notify), which runs
 *    motion_poll_commands() at once without moving the 20 ms deadline.
 *  - The VC and Line & Barcode do not call tk_snd_mbf for motion: they call
 *    motion_command() / motion_set_steer() directly. motion_command() is the
 *    mbf_motion_cmd: a bounded queue with STOP/ESTOP on a separate fast path,
 *    so STOP can never wait behind STEER or a full buffer (Tribunal 1, R2).
 *  - Motion events are forwarded to mbf_ctrl_evt here, in task context.
 *  - Snapshot words are copied to robot_snapshot_t under mtx_snapshot.
 */
#ifdef MOTION_USE_UTKERNEL

#include <tk/tkernel.h>

#include "motion_control.h"
#include "motion_hal.h"
#include "robot_shared.h"

#define MOTION_TASK_PRI     10
#define MOTION_TASK_STACK   1024

/* Created by the integration code (Buddy 5) and shared by all tasks. */
extern ID mbf_ctrl_evt;
extern ID mtx_snapshot;
extern volatile uint32_t g_robot_snapshot_motion_w1;    /* robot_snapshot_t */
extern volatile uint32_t g_robot_snapshot_motion_w2;

static ID              g_task_id;
static ID              g_cyc_id;
static volatile BOOL   g_tick_due;

static void
motion_cyclic_handler (void * exinf)
{
    (void) exinf;
    g_tick_due = TRUE;
    (void) tk_wup_tsk(g_task_id);   /* Legal from a handler. */
}

static void
motion_wake (void)
{
    (void) tk_wup_tsk(g_task_id);
}

static void
motion_task (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;

    motion_hal_pico_init();
    motion_init();
    motion_set_notify(motion_wake);
    (void) tk_sta_cyc(g_cyc_id);

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);

        if (g_tick_due)
        {
            g_tick_due = FALSE;
            motion_tick();
        }
        else
        {
            motion_poll_commands();
        }

        motion_evt_t evt;

        while (motion_get_event(&evt))
        {
            /* Typed event: the VC decodes motion_evt_t (robot_shared.h). */
            (void) tk_snd_mbf(mbf_ctrl_evt, &evt, sizeof(evt), TMO_POL);
        }

        uint32_t w1;
        uint32_t w2;

        motion_get_snapshot(&w1, &w2);

        if (E_OK == tk_loc_mtx(mtx_snapshot, TMO_POL))
        {
            g_robot_snapshot_motion_w1 = w1;
            g_robot_snapshot_motion_w2 = w2;
            (void) tk_unl_mtx(mtx_snapshot);
        }
    }
}

ER
motion_task_create (void)
{
    T_CTSK ctsk =
    {
        .tskatr  = TA_HLNG | TA_RNG0,
        .task    = motion_task,
        .itskpri = MOTION_TASK_PRI,
        .stksz   = MOTION_TASK_STACK,
    };
    T_CCYC ccyc =
    {
        .cycatr  = TA_HLNG,
        .cychdr  = motion_cyclic_handler,
        .cyctim  = 20,      /* ms, with a 1 ms system tick (A11) */
        .cycphs  = 0,
    };

    g_task_id = tk_cre_tsk(&ctsk);
    if (g_task_id < E_OK)
    {
        return g_task_id;
    }

    g_cyc_id = tk_cre_cyc(&ccyc);
    if (g_cyc_id < E_OK)
    {
        return g_cyc_id;
    }

    return tk_sta_tsk(g_task_id, 0);
}

#endif /* MOTION_USE_UTKERNEL */
