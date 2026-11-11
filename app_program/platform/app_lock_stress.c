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

/* Iterations of the task side. Each holds the lock for roughly a millisecond of
 * spin, so the run covers many handler periods. */
#define LOCK_STRESS_ROUNDS 2000u
/* Spin count inside the critical section: long enough that a 1 ms handler would
 * preempt this task if interrupts were not masked. Calibration is approximate
 * and is why the report includes the handler attempt count. */
#define LOCK_STRESS_HOLD   4000u

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
    config.cyctim = 1;
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
        /* A 1 ms handler must not be able to run here, because the task's
         * interrupt state is masked for the whole critical section. */
        for (volatile UW spin = 0u; spin < LOCK_STRESS_HOLD; ++spin) {
            /* hold the lock */
        }
        stress_held = 0u;
        (void)ISpinUnlock(&stress_lock, intsts);
    }

    (void)tk_stp_cyc(cyc);
    (void)tk_del_cyc(cyc);

    /* The handler must have run, and must never have seen the lock held. */
    const bool ran = stress_tries > 0u;
    const bool clean = (0u == stress_violations) && (0u == stress_errors);
    tm_printf((UB *)"[lock] stress tries %lu held_seen %lu other %lu %s\n",
              (unsigned long)stress_tries, (unsigned long)stress_violations,
              (unsigned long)stress_errors,
              (ran && clean) ? "PASS" : "FAIL");
}

#endif /* APP_TEST */
