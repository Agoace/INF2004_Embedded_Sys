/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tk/smp.h>   /* T_SPLOCK, SpinLock: cross-core timing state */
#include "cyw43_utk.h"

extern uint64_t time_us_64(void);
static T_CYW43_TIMING timing;
static T_CYW43_TIMING window;

/*
 * Guards every shared field in this file, instead of a core-local mask.
 * Phase 4: the poll path runs on the Wi-Fi task and the window, read and status
 * calls run on the Vehicle task.  Once those are different cores, a DI()/EI() on
 * either core protects neither, and the begin/end window is a shared state
 * machine rather than a snapshot, so a seqlock would be the wrong primitive.
 * These are task-context only, so plain SpinLock is the correct variant under
 * the lock rule.  A zero-filled static already holds the unlocked state.
 */
static T_SPLOCK timing_lock;
static UW started;
static UW previous;
static UW window_previous;
static UW window_id;
static UW poll_window_id;
static UW probe_begin_us;
static BOOL have_previous;
static BOOL window_previous_valid;
static BOOL window_active;
/* RT-0071/F2: a close request from a caller that could not take the lock.  It is
 * applied under the lock by the poll path or by the next window begin, so no
 * caller ever writes window_active from outside the lock. */
static UW window_close_req;
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
    UINT intsts;
    (void)ISpinLock(&timing_lock, &intsts);
    started = (UW)time_us_64();
    poll_window_id = window_active ? window_id : 0u;
    (void)ISpinUnlock(&timing_lock, intsts);
    probe_begin_us = (UW)time_us_64() - entered;
}

/* Finish a loop and update lifetime and optional diagnostic-window data. */
void cyw43_utk_poll_end(void)
{
    const UW end = (UW)time_us_64();
    const UW work = end - started;
    UINT intsts;
    (void)ISpinLock(&timing_lock, &intsts);
    if (0u != atomic_xchg(&window_close_req, 0u)) {
        window_active = FALSE;   /* apply a deferred close from a missed try */
    }
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
    (void)ISpinUnlock(&timing_lock, intsts);
    /* Probe wall cost excludes its own final two max stores. */
    const UW cost = probe_begin_us + ((UW)time_us_64() - end);
    (void)ISpinLock(&timing_lock, &intsts);
    if (cost > timing.probe_max_us) {
        timing.probe_max_us = cost;
    }
    if (window_active && poll_window_id == window_id &&
        cost > window.probe_max_us) {
        window.probe_max_us = cost;
    }
    (void)ISpinUnlock(&timing_lock, intsts);
}

/* Copy lifetime timing counters under the cross-core timing lock. */
int32_t cyw43_utk_timing_read(T_CYW43_TIMING *out)
{
    if (!out) {
        return E_PAR;
    }
    /* Interrupt-preserving: a core-0 holder must not be preemptible by Vehicle. */
    UINT intsts;
    (void)ISpinLock(&timing_lock, &intsts);
    *out = timing;
    (void)ISpinUnlock(&timing_lock, intsts);
    return E_OK;
}

/* Start an independent timing window without resetting lifetime counters. */
int32_t cyw43_utk_timing_window_begin(void)
{
    /*
     * RT-0069: Vehicle must not wait on a lock held by a task it has preempted, so
     * this fails fast.  On failure no window is opened at all: the caller defers
     * its measurement and no duplicate or half-open window can result.
     */
    UINT intsts;
    if (E_OK != ISpinTryLock(&timing_lock, &intsts)) {
        return E_TMOUT;
    }
    if (0u != atomic_xchg(&window_close_req, 0u)) {
        window_active = FALSE;   /* a deferred close beats the stale flag */
    }
    if (window_active) {
        (void)ISpinUnlock(&timing_lock, intsts);
        return E_OBJ;
    }
    window = (T_CYW43_TIMING){0};
    window_previous_valid = FALSE;
    if (++window_id == 0u) {
        ++window_id;
    }
    window_active = TRUE;
    (void)ISpinUnlock(&timing_lock, intsts);
    return E_OK;
}

/* Close the current diagnostic window and return its counters. */
int32_t cyw43_utk_timing_window_end(T_CYW43_TIMING *out)
{
    if (!out) {
        return E_PAR;
    }
    /*
     * RT-0069: fail fast, but an active window must never be left behind.  When
     * the lock cannot be taken, the window is closed with a single aligned store
     * and no counters are returned: `window_active` is a word, so the store is
     * atomic, and the poll path only uses it to decide attribution, which is
     * diagnostics.  Losing one window's counters is the accepted cost.
     */
    UINT intsts;
    if (E_OK != ISpinTryLock(&timing_lock, &intsts)) {
        /* RT-0071/F2: never touch window_active outside the lock, because the
         * poll path owns it under the same lock.  Leave a request the poll path or
         * the next begin applies, and return no counters for this window. */
        (void)atomic_inc(&window_close_req);
        return E_TMOUT;
    }
    if (!window_active) {
        (void)ISpinUnlock(&timing_lock, intsts);
        return E_OBJ;
    }
    window_active = FALSE;
    *out = window;
    (void)ISpinUnlock(&timing_lock, intsts);
    return E_OK;
}

/* Read the latest task-start result without changing it. */
int32_t cyw43_utk_start_status(void)
{
    /* A 32-bit status word; on contention return the current value without
     * waiting, since the reader only reports it. */
    UINT intsts;
    if (E_OK != ISpinTryLock(&timing_lock, &intsts)) {
        /* RT-0071/F2: a defined single-word read through the atomic builtin,
         * rather than an unsynchronised read of state the writer stores under the
         * lock.  A status word cannot tear, and a momentarily stale value is all
         * this reader reports. */
        return (int32_t)__atomic_load_n(&start_status, __ATOMIC_SEQ_CST);
    }
    const ER result = start_status;
    (void)ISpinUnlock(&timing_lock, intsts);
    return result;
}

/* The template used priority 4: below USB 3, above its qualification task.
 * Driving tasks 10-13 must not wait for network processing. Create the radio
 * at priority 20 directly. USB is now 21, below the radio (see usb_console.c),
 * and telemetry stays at 30.
 * See docs/architecture/TEMPLATE_CHANGES.md for the local deviation and update
 * procedure.
 */
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

    /* Phase 4: the radio service runs on core 1. The CYW43439 needs ONE core to
     * own its PIO/DMA and pin state permanently, and this task is that owner;
     * what matters is that it never migrates, not that it is core 0. Nothing here
     * requires core 0: the port does not use the pico-sdk async context or the
     * SDK IRQ path (the hooks in cyw43_utk_compat.c are weak stubs), the driver
     * polls, and PIO and DMA are core-independent hardware on RP2040. Its shared
     * timing state is guarded by timing_lock in this file, not by a core-local
     * mask. On a single-core build there is only one processor, so the affinity
     * request is dropped rather than passing an ID the kernel does not define.
     * See docs/kernel/CORE_PLACEMENT.md. */
#if TK_SUPPORT_SMP
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
    ctsk.assprc = TP_PRC1 + 1u;   /* core 1 */
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
        UINT intsts;
        (void)ISpinLock(&timing_lock, &intsts);
        start_status = task;
        (void)ISpinUnlock(&timing_lock, intsts);
        return task;
    }
    /* start_status is read under timing_lock by cyw43_utk_start_status(), so every
     * write to it must take the same lock (RT-0058 finding 4).  The kernel call
     * stays OUTSIDE the section (RT-0069): a task start may dispatch, and a
     * critical section must hold no call that can dispatch. */
    const ER sta = tk_sta_tsk(task, 0);
    UINT intsts;
    (void)ISpinLock(&timing_lock, &intsts);
    start_status = sta;
    (void)ISpinUnlock(&timing_lock, intsts);
    if (sta < E_OK) {
        (void)tk_del_tsk(task);
    }
    return sta;
}
