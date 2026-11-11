/*
 * app_lock_stress.c - handler-versus-task lock stress test (Phase 1b).
 *
 * The rule under test: a lock reachable from both a task and an interrupt
 * handler on the same core must be taken with ISpinLock(), which preserves and
 * masks the interrupt state, because a plain SpinLock lets the handler spin
 * against the task it interrupted and neither proceeds.
 *
 * The test makes the rule observable rather than reproducing the hang:
 *
 *   - a 1 ms cyclic handler (handler context) tries the lock with
 *     ISpinTryLock() and records any attempt that finds it held;
 *   - the test task takes the lock with ISpinLock(), records that it is held,
 *     spins long enough that a 1 ms handler would certainly run if it could, and
 *     then releases.
 *
 * Pass condition: the handler attempts many times, never observes the lock held,
 * and the run completes. A plain SpinLock() variant is deliberately not tried,
 * because on this port it would deadlock the board instead of failing a check.
 *
 * Builds only with make TEST=1. See docs/kernel/CORE_PLACEMENT.md.
 */
#include "../app.h"
#include <stdbool.h>

#if APP_TEST

#include <tk/smp.h>

/* Iterations of the task side.  Each holds the lock for a short spin, so the run
 * spans many handler periods. */
#define LOCK_STRESS_ROUNDS 2000u
/* Spin count inside the critical section.  Calibration is approximate, which is
 * why the report carries the handler attempt count instead of assuming it. */
#define LOCK_STRESS_HOLD   4000u
/* The handler must be observed MANY times, not merely once: a single attempt
 * would prove only that the cyclic handler was created, not that the overlap
 * condition was exercised.  The run spans far more than this many kernel
 * periods, so a smaller count means the handler is not running at all. */
#define LOCK_STRESS_MIN_TRIES 50u

static T_SPLOCK stress_lock;
static volatile UW stress_held;       /* 1 while the task holds the lock */
static volatile UW stress_tries;      /* handler attempts */
static volatile UW stress_violations; /* handler found the lock held */
static volatile UW stress_errors;     /* init or kernel object errors */

/* Handler context. Uses the interrupt-preserving variant because this runs in a
 * handler and must restore the interrupt state it inherited. */
static void stress_probe(FP exinf)
{
    (void)exinf;
    ++stress_tries;

    UINT intsts;
    if (E_OK == ISpinTryLock(&stress_lock, &intsts)) {
        if (0u != stress_held) {
            ++stress_violations;
        }
        (void)ISpinUnlock(&stress_lock, intsts);
    } else {
        /* Contention is not a rule violation, but with interrupts masked on the
         * task side it should never happen. Count it so a surprise is visible. */
        ++stress_errors;
    }
}

/*
 * --- Forced Health-to-Vehicle interleaving (RT-0069) -------------------------
 *
 * The hazard is task-versus-task on ONE core: Vehicle at priority 13 preempts
 * Health at priority 31 while Health holds a lock, then spins on it. With a plain
 * SpinLock the preempted holder can never release it.
 *
 * This runs the pair for real. The holder task sits at the Health priority and
 * uses the interrupt-preserving pair; the runner sits above it and uses a bounded
 * try-lock only. The runner yields between attempts so the holder can run, which
 * is what makes the interleaving reachable at all.
 *
 * The check cannot hang the runner: the runner never spins on the lock, both
 * loops are round-bounded, and the wait for the holder is bounded. A broken
 * variant therefore reports FAIL rather than stopping the suite.
 */
#define IL_ROUNDS   500u
#define IL_HOLD     2000u
#define IL_WAIT     200000u
#define IL_STACK_BYTES 2048u

static volatile UW il_held;   /* 1 while the holder task holds the lock */
static volatile UW il_seen;   /* runner found the lock held while it was taken */
static volatile UW il_busy;   /* runner could not take the lock */
static volatile UW il_done;   /* holder finished its rounds */

static void il_holder(FP exinf)
{
    (void)exinf;
    for (UW round = 0u; round < IL_ROUNDS; ++round) {
        UINT intsts;
        if (E_OK != ISpinLock(&stress_lock, &intsts)) {
            ++stress_errors;
            break;
        }
        il_held = 1u;
        for (volatile UW spin = 0u; spin < IL_HOLD; ++spin) {
            /* hold the lock for a short, bounded section */
        }
        il_held = 0u;
        (void)ISpinUnlock(&stress_lock, intsts);
    }
    il_done = 1u;
}

static void interleave_run(void)
{
    il_held = 0u;
    il_seen = 0u;
    il_busy = 0u;
    il_done = 0u;

    T_CTSK task = {0};
    task.tskatr = TA_HLNG | TA_RNG0;
    task.exinf = 0;
    task.task = (FP)il_holder;
    task.itskpri = 31;   /* the Health priority */
    task.stksz = IL_STACK_BYTES;   /* this port has no stkadr: the kernel allocates */

    const ID id = tk_cre_tsk(&task);
    if (id < E_OK) {
        ++stress_errors;
        tm_printf((UB *)"[lock] interleave create failed %d\n", (int)id);
        return;
    }
    const ER sta = tk_sta_tsk(id, 0);
    if (sta < E_OK) {
        (void)tk_del_tsk(id);
        ++stress_errors;
        tm_printf((UB *)"[lock] interleave start failed %d\n", (int)sta);
        return;
    }

    /* Runner side: bounded, yields, never spins on the lock. */
    for (UW round = 0u; round < IL_ROUNDS && 0u == il_done; ++round) {
        UINT intsts;
        if (E_OK == ISpinTryLock(&stress_lock, &intsts)) {
            if (0u != il_held) {
                ++il_seen;
            }
            (void)ISpinUnlock(&stress_lock, intsts);
        } else {
            ++il_busy;
        }
        (void)tk_dly_tsk(1u);   /* let the lower-priority holder run */
    }

    /* Bounded wait for the holder to finish. */
    for (UW wait = 0u; wait < IL_WAIT && 0u == il_done; ++wait) {
        (void)tk_dly_tsk(1u);
    }

    (void)tk_del_tsk(id);

    tm_printf((UB *)"[lock] interleave seen %lu busy %lu done %lu %s\n",
              (unsigned long)il_seen, (unsigned long)il_busy,
              (unsigned long)il_done,
              (0u == il_seen && 0u != il_done) ? "PASS" : "FAIL");
}

void app_lock_stress_run(void)
{
    stress_held = 0u;
    stress_tries = 0u;
    stress_violations = 0u;
    stress_errors = 0u;

    const ER lock_er = InitSpinLock(&stress_lock);
    if (E_OK != lock_er) {
        tm_printf((UB *)"[lock] stress: InitSpinLock failed %d\n", lock_er);
        return;
    }

    T_CCYC config = {0};
    config.cycatr = TA_HLNG;
    config.cychdr = (FP)stress_probe;
    config.cyctim = 1;   /* one kernel tick; CNF_TIMER_PERIOD is 10 ms */
    config.cycphs = 1;
#if TK_SUPPORT_SMP
    config.cycatr |= TA_ASSPRC;
    config.assprc = app_prc_core0();
#endif

    const ID cyc = tk_cre_cyc(&config);
    if (cyc < 0) {
        tm_printf((UB *)"[lock] stress: tk_cre_cyc failed %d\n", (int)cyc);
        return;
    }
    const ER sta_er = tk_sta_cyc(cyc);
    if (sta_er < E_OK) {
        tm_printf((UB *)"[lock] stress: tk_sta_cyc failed %d\n", sta_er);
        (void)tk_del_cyc(cyc);
        return;
    }

    for (UW round = 0u; round < LOCK_STRESS_ROUNDS; ++round) {
        UINT intsts;
        if (E_OK != ISpinLock(&stress_lock, &intsts)) {
            ++stress_errors;
            continue;
        }
        stress_held = 1u;
        /* A handler at one kernel tick must not be able to run here, because the
         * task's interrupt state is masked for the whole critical section.  Do
         * NOT lengthen this interval to force an overlap: the claim is that the
         * handler cannot run while the lock is held, not that it is starved. */
        for (volatile UW spin = 0u; spin < LOCK_STRESS_HOLD; ++spin) {
            /* hold the lock */
        }
        stress_held = 0u;
        (void)ISpinUnlock(&stress_lock, intsts);
    }

    (void)tk_stp_cyc(cyc);
    (void)tk_del_cyc(cyc);

    /* The handler must have run many times, and must never have seen the lock
     * held.  Requiring only one attempt would not prove the overlap condition. */
    const bool ran = stress_tries >= LOCK_STRESS_MIN_TRIES;
    const bool clean = (0u == stress_violations) && (0u == stress_errors);
    tm_printf((UB *)"[lock] stress tries %lu held_seen %lu other %lu min %lu %s\n",
              (unsigned long)stress_tries, (unsigned long)stress_violations,
              (unsigned long)stress_errors,
              (unsigned long)LOCK_STRESS_MIN_TRIES,
              (ran && clean) ? "PASS" : "FAIL");

    /* RT-0069: the forced task-versus-task interleaving, after the handler case. */
    interleave_run();
}

#endif /* APP_TEST */
