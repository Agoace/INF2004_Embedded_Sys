/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "cyw43_utk.h"

extern uint64_t time_us_64(void);
static T_CYW43_TIMING timing;
static T_CYW43_TIMING window;
static UW started;
static UW previous;
static UW window_previous;
static UW window_id;
static UW poll_window_id;
static UW probe_begin_us;
static BOOL have_previous;
static BOOL window_previous_valid;
static BOOL window_active;
static ER start_status = E_OBJ;
#define CYW43_LONG_POLL_GAP_US 10000u

/* Accumulate work and start-to-start gap observations. */
static void timing_add(T_CYW43_TIMING *t, UW work, UW gap, BOOL has_gap)
{
    ++t->polls;
    t->work_total_us += work;
    if (work > t->work_max_us) {
        t->work_max_us = work;
    }
    if (has_gap) {
        ++t->gaps;
        if (gap > t->gap_max_us) {
            t->gap_max_us = gap;
        }
        if (gap > CYW43_LONG_POLL_GAP_US) {
            ++t->gaps_over_10000;
        }
    }
}

/* Mark a full Wi-Fi loop start, including preemption in its wall time. */
void cyw43_utk_poll_begin(void)
{
    const UW entered = (UW)time_us_64();
    UINT irq;
    DI(irq);
    started = (UW)time_us_64();
    poll_window_id = window_active ? window_id : 0u;
    EI(irq);
    probe_begin_us = (UW)time_us_64() - entered;
}

/* Finish a loop and update lifetime and optional diagnostic-window data. */
void cyw43_utk_poll_end(void)
{
    const UW end = (UW)time_us_64();
    const UW work = end - started;
    UINT irq;
    DI(irq);
    timing_add(&timing, work, started - previous, have_previous);
    previous = started;
    have_previous = TRUE;
    const BOOL in_window = window_active && poll_window_id == window_id;
    if (in_window) {
        timing_add(&window, work, started - window_previous,
                   window_previous_valid);
        window_previous = started;
        window_previous_valid = TRUE;
    }
    EI(irq);
    /* Probe wall cost excludes its own final two max stores. */
    const UW cost = probe_begin_us + ((UW)time_us_64() - end);
    DI(irq);
    if (cost > timing.probe_max_us) {
        timing.probe_max_us = cost;
    }
    if (in_window && window_active && poll_window_id == window_id &&
        cost > window.probe_max_us) {
        window.probe_max_us = cost;
    }
    EI(irq);
}

/* Copy lifetime timing counters under a short interrupt mask. */
int32_t cyw43_utk_timing_read(T_CYW43_TIMING *out)
{
    if (!out) {
        return E_PAR;
    }
    UINT irq;
    DI(irq);
    *out = timing;
    EI(irq);
    return E_OK;
}

/* Start an independent timing window without resetting lifetime counters. */
int32_t cyw43_utk_timing_window_begin(void)
{
    UINT irq;
    DI(irq);
    if (window_active) {
        EI(irq);
        return E_OBJ;
    }
    window = (T_CYW43_TIMING){0};
    window_previous_valid = FALSE;
    if (++window_id == 0u) {
        ++window_id;
    }
    window_active = TRUE;
    EI(irq);
    return E_OK;
}

/* Close the current diagnostic window and return its counters. */
int32_t cyw43_utk_timing_window_end(T_CYW43_TIMING *out)
{
    if (!out) {
        return E_PAR;
    }
    UINT irq;
    DI(irq);
    if (!window_active) {
        EI(irq);
        return E_OBJ;
    }
    window_active = FALSE;
    *out = window;
    EI(irq);
    return E_OK;
}

/* Read the latest task-start result without changing it. */
int32_t cyw43_utk_start_status(void)
{
    UINT irq;
    DI(irq);
    const ER result = start_status;
    EI(irq);
    return result;
}

/* The template used priority 4: below USB 3, above its qualification task.
 * Driving tasks 10-13 must not wait for network processing. Create the radio
 * at priority 20 directly; USB stays at 3 and telemetry stays at 30.
 * See docs/TEMPLATE_CHANGES.md for the local deviation and update procedure. */
#define CYW43_TASK_PRIORITY 20
#if TM_NET_LWIP
/* lwIP's own call chains (tcp_input/dhcp/etharp) run on this same task and
 * go deeper than the bare radio poll loop did. */
#define CYW43_TASK_STACK     (8 * 1024)
#else
#define CYW43_TASK_STACK     (6 * 1024)
#endif

EXPORT int32_t cyw43_utk_start(void)
{
    T_CTSK ctsk = {0};
    ID task;

    /* Pinned to processor 1 (core 0) for the same reason usb_console.c pins
     * its service task: the CYW43439's PIO/DMA and pin ownership, and the
     * lwIP state serviced alongside it, must not migrate between cores. On a
     * single-core build there is only one processor, so the affinity request
     * is dropped rather than being passed an ID the kernel does not define. */
#if TK_SUPPORT_SMP
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
    ctsk.assprc = TP_PRC1;
#else
    ctsk.tskatr = TA_HLNG | TA_RNG1;
    ctsk.assprc = 0;
#endif
	ctsk.task = (FP)cyw43_utk_task;
    ctsk.itskpri = CYW43_TASK_PRIORITY;
    ctsk.stksz = CYW43_TASK_STACK;
#if USE_OBJECT_NAME
    ctsk.dsname = (UB*)"cyw43";
#endif

    task = tk_cre_tsk(&ctsk);
    if (task < E_OK) {
        start_status = task;
        return task;
    }
    start_status = tk_sta_tsk(task, 0);
    if (start_status < E_OK) {
        (void)tk_del_tsk(task);
    }
    return start_status;
}
