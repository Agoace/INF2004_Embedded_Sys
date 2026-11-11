/*
 * telemetry_contention_test.c - on-target contention and admission tests.
 *
 * RT-0073/F3 asked for focused tests that observe the specified collisions and
 * outcomes, rather than a smoke check that can pass without ever colliding. These
 * are TEST builds only (APP_TEST=1) and need hardware: the telemetry task, the
 * message buffer and the CYW43 timing lock are all kernel objects.
 *
 * Limits, stated so the output is not over-read:
 *   - the outcome of a race is asserted, not the interleaving; a PASS here means
 *     the specified outcome was observed on this run, not that a collision always
 *     occurs;
 *   - assertions that need the consumer NOT to have run yet are restricted to
 *     single-core builds, where the task cannot run until this test yields;
 *   - no timing bound is claimed beyond the loose one noted in the progress test.
 */
#include "../app.h"

#if APP_TEST

#include <stdbool.h>
#include "telemetry.h"

#include <tk/smp.h>

/* Microsecond uptime, used to bound the producer-progress check. Declared here
 * rather than pulled in through a platform header this file does not otherwise
 * need. */
IMPORT UW drv_time_us(void);

#if TM_MQTT
/* libwifi hook, TEST builds only: hold the static timing lock to force the
 * "could not take the lock" path (see cyw43_utk_kernel.c). */
IMPORT void cyw43_utk_timing_test_hold(void);
IMPORT void cyw43_utk_timing_test_release(void);
IMPORT int32_t cyw43_utk_timing_window_begin(void);
IMPORT int32_t cyw43_utk_timing_test_window_end(void);
#endif

/* Payload-lock hook, TEST builds only (telemetry.c). */
IMPORT void telemetry_test_hold_payload_lock(void);
IMPORT void telemetry_test_release_payload_lock(void);

static unsigned int fails;

static void expect(const char *what, bool ok)
{
    tm_printf((UB *)"[tel] %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) {
        ++fails;
    }
}

/* Wait until the telemetry task has drained the buffer, bounded. */
static void settle(void)
{
    for (unsigned int i = 0u; i < 50u; ++i) {
        (void)tk_dly_tsk(2u);
    }
}

/*
 * T1. DUMP admission, duplicate rejection and rollback.
 *
 * RT-0075/F3: the previous version discarded the second result and accepted either
 * outcome, so it proved nothing about duplicate rejection. On ONE core the consumer
 * cannot run between the two calls, so refusal is deterministic and is asserted. On
 * two cores the consumer may already have completed the dump, so that assertion is
 * scoped out rather than weakened into a tautology.
 *
 * Rollback is exercised by saturating the message buffer and then submitting a DUMP:
 * it takes the reservation, the send fails, and the reservation must be released
 * rather than left stuck.
 */
static void test_dump_admission(void)
{
    telemetry_message_t dump = {0};
    dump.kind = TELEMETRY_DUMP;
    telemetry_message_t row = {0};
    row.kind = TELEMETRY_ROW;

    settle();
    telemetry_stats_t before;
    telemetry_stats(&before);

    const int first = telemetry_send(&dump);
    expect("first DUMP admitted", 0 == first);
#if !TK_SUPPORT_SMP
    telemetry_stats_t held;
    telemetry_stats(&held);
    expect("reservation visible before consumption", held.busy);
    const int second = telemetry_send(&dump);
    expect("duplicate DUMP refused while the reservation is held", E_OBJ == second);
#else
    (void)telemetry_send(&dump);   /* consumer may have finished; both are legal */
#endif

    settle();
    telemetry_stats_t after;
    telemetry_stats(&after);
    expect("reservation released after the dump", !after.busy);
    expect("loss accounting did not go backwards", after.lost >= before.lost);

#if !TK_SUPPORT_SMP
    /*
     * Rollback. RT-0077/F1: the previous version only PRINTED the result and then
     * settled, so it could PASS on a DUMP that was accepted, consumed and released
     * normally without any send ever failing. Now the buffer is drained first and
     * then saturated without yielding, so the state is deterministic, and the send
     * failure itself is asserted before the reservation is examined.
     */
    settle();   /* drain, so saturation starts from a known state */
    for (unsigned int i = 0u; i < 2000u; ++i) {
        (void)telemetry_send(&row);
    }
    const int saturated = telemetry_send(&dump);
    expect("DUMP send fails against a saturated buffer", E_TMOUT == saturated);
    if (E_TMOUT == saturated) {
        /* No settle() here: the rollback happens inside the send, so the
         * reservation must already be free. */
        telemetry_stats_t rb;
        telemetry_stats(&rb);
        expect("reservation released by the failed send", !rb.busy);
    }
#endif
}

/*
 * T2. Storage-full stop. Writing past the row capacity must set storage_full, and
 * the flag must be readable from here without taking any lock.
 */
static void test_storage_full(void)
{
    telemetry_message_t m = {0};
    m.kind = TELEMETRY_ROW;

    settle();
    unsigned int sent = 0u;
    bool full = false;
    for (unsigned int i = 0u; i < (TELEMETRY_ROWS + 32u); ++i) {
        if (0 == telemetry_send(&m)) {
            ++sent;
        }
        telemetry_stats_t st;
        telemetry_stats(&st);
        if (st.storage_full) {
            full = true;
            break;
        }
    }
    expect("rows accepted before the log filled", sent > 0u);
    expect("storage_full set when the log is full", full);
    if (full) {
        /* This is the input Vehicle's storage-full stop decides on. The stop itself
         * - that Vehicle actually halts - is not asserted here: it lives in
         * vehicle_task.c and needs Phase 6 device evidence (RT-0075/F3). */
        telemetry_stats_t st;
        telemetry_stats(&st);
        expect("storage_full remains readable", st.storage_full);
    }
}

/*
 * T3. Loss counting. Overflowing the message buffer must increase the loss
 * counter, and it must not decrease it.
 */
static void test_loss_counting(void)
{
    telemetry_message_t m = {0};
    m.kind = TELEMETRY_ROW;

    settle();
    telemetry_stats_t before;
    telemetry_stats(&before);

    /* Far more than the buffer holds, with no yield: some must be dropped. */
    for (unsigned int i = 0u; i < 2000u; ++i) {
        (void)telemetry_send(&m);
    }
    telemetry_stats_t after;
    telemetry_stats(&after);
    expect("loss counter increased under overflow", after.lost > before.lost);
}

/*
 * T4. Producer progress while the telemetry task is busy writing rows. The point of
 * RT-0071/F1 was that telemetry_send() takes no lock, so a core-1 payload mutation
 * must not delay a core-0 producer. The bound is deliberately loose: this detects a
 * producer that blocks on the payload lock, not a precise latency figure.
 */
static void test_producer_progress(void)
{
    telemetry_message_t row = {0};
    row.kind = TELEMETRY_ROW;

    /*
     * RT-0075/F3, corrected by RT-0077/F2: hold the payload lock for the whole
     * measurement, so the producer demonstrably runs while the RAM-log lock is
     * unavailable. The buffer is drained first, so "admitted > 0" means the producer
     * progressed rather than that the queue happened to have room left over from the
     * previous test.
     *
     * Scope of the claim, stated because the previous wording overstated it: the
     * holder here is THIS task on core 0, not the Telemetry task on core 1. What the
     * test therefore establishes is that telemetry_send() takes no part in the
     * payload lock at all - which is the mechanism the fix relies on - not a
     * reproduced core-1-holder interleaving. That interleaving needs the consumer to
     * be inside its critical section, which this test cannot force.
     */
    settle();   /* drain, so admitted > 0 is meaningful */
    telemetry_test_hold_payload_lock();
    const UW start = drv_time_us();
    unsigned int admitted = 0u;
    for (unsigned int i = 0u; i < 200u; ++i) {
        if (0 == telemetry_send(&row)) {
            ++admitted;
        }
    }
    const UW elapsed = drv_time_us() - start;
    telemetry_test_release_payload_lock();

    tm_printf((UB *)"[tel] producer 200 sends in %lu us under a held payload lock, "
              "%u admitted\n", (unsigned long)elapsed, admitted);
    expect("producer admitted work while the payload lock was held", admitted > 0u);
    /* Loose: 200 sends must not take longer than a 10 ms control tick, which a
     * payload-lock wait would exceed. */
    expect("producer not blocked by the payload lock", elapsed < 10000u);
}

#if TM_MQTT && TK_SUPPORT_SMP
/*
 * T5. Timing window collision at begin and end (RT-0071/F2).
 *
 * RT-0075/F3: scoped to two-core builds. The single-core shim's ISpinTryLock always
 * succeeds, so a self-held collision cannot produce E_TMOUT there and the check
 * asserted a result that profile cannot return.
 *
 * Begin while the lock is held: the call must fail fast and open NO window.
 * End while the lock is held: the call must fail fast AND leave a close request
 * that the next begin applies, so a stale window cannot block a new one.
 */
static void test_timing_collision(void)
{
    const int32_t begin_ok = cyw43_utk_timing_window_begin();
    /* Normal path first, so the collision is compared against a working window. */
    expect("window begins when uncontended", E_OK == begin_ok);
    expect("window ends when uncontended", E_OK == cyw43_utk_timing_test_window_end());

    cyw43_utk_timing_test_hold();
    const int32_t begin_rc = cyw43_utk_timing_window_begin();
    cyw43_utk_timing_test_release();
    expect("begin fails fast while the lock is held", E_TMOUT == begin_rc);
    /* No window was opened by the failed begin, so an uncontended begin must now
     * succeed. If the failed call had opened one, this would return E_OBJ. */
    expect("no window left open by the failed begin",
           E_OK == cyw43_utk_timing_window_begin());

    cyw43_utk_timing_test_hold();
    const int32_t end_rc = cyw43_utk_timing_test_window_end();
    cyw43_utk_timing_test_release();
    expect("end fails fast while the lock is held", E_TMOUT == end_rc);
    /* The missed close is applied by the next begin: if it were lost, the window
     * would still look active and this begin would return E_OBJ instead. */
    const int32_t reopen = cyw43_utk_timing_window_begin();
    expect("missed close applied by the next begin", E_OK == reopen);
    (void)cyw43_utk_timing_test_window_end();
}
#endif

void telemetry_contention_test(void)
{
    if (!telemetry_running()) {
        tm_printf((UB *)"[tel] skipped: telemetry not running\n");
        return;
    }
    fails = 0u;

    test_dump_admission();
    test_storage_full();
    test_loss_counting();
    test_producer_progress();
#if TM_MQTT && TK_SUPPORT_SMP
    test_timing_collision();
#endif

    tm_printf((UB *)"[tel] contention suite %s (%u failures)\n",
              (0u == fails) ? "PASS" : "FAIL", fails);
}

#endif /* APP_TEST */
