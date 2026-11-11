/*
 * telemetry.c - application telemetry and service interface.
 */
#include "subsystem1.h"

/* Task timing and resource limits. */
#define DUMP_YIELD_MS 20u

#include <tk/syslib.h>
#include "telemetry.h"
#include "../platform/drv_watchdog.h"
#include "../platform/app_task.h"
#include "../platform/drv_time.h"
#include "../subsystem2/motion_diag.h"
#include "../vehicle/vehicle_task.h"
#include "../subsystem4/imu_settings.h"
#include <tk/smp.h>   /* T_SPLOCK, SpinLock: multi-writer stats */
static ID mbf_telemetry;
static ID task;
static telemetry_stats_t stats;

/*
 * RT-0071/F1: the CONTROL fields - DUMP reservation, storage full, loss count -
 * are published as atomics, independent of stats_lock.  Vehicle decides on them
 * (vehicle_task.c:393, 659, 1311), so reading them unsynchronised or behind the
 * payload lock is a control defect rather than a diagnostic nicety.  stats_lock
 * keeps guarding the payload and the diagnostic counters.
 */
static UW admit_busy;   /* DUMP reservation: 0 free, 1 held */
static UW admit_full;   /* log storage full */
static UW admit_lost;   /* losses, incremented from both cores */

/*
 * Diagnostic counters published for lock-free readers.  Written only by the
 * Telemetry task while it holds stats_lock; a reader may mix two instants, which
 * is exactly why the control fields above are kept as separate atomics.
 */
static telemetry_stats_t stats_pub;

/* Publish the diagnostic counters. Caller holds stats_lock.
 *
 * RT-0073/F1: per-field atomic stores, so each counter's publication is a defined
 * operation rather than a plain write that a reader could race. */
static void publish_stats(void)
{
    __atomic_store_n(&stats_pub.rows, stats.rows, __ATOMIC_SEQ_CST);
    __atomic_store_n(&stats_pub.events, stats.events, __ATOMIC_SEQ_CST);
}
static uint8_t rows[TELEMETRY_ROWS][TELEMETRY_BYTES];
static telemetry_event_t events[TELEMETRY_EVENTS];
static telemetry_info_t info;
static ja_result_t last_junction;
static uint32_t loss_base;
/*
 * Guards the multi-writer part of `stats`.  `lost` is incremented by submitters
 * on core 0 and by the telemetry task on core 1, and `busy` is set by a
 * submitter and cleared by the dump task.  A seqlock cannot serve these fields:
 * a sequence assumes a SINGLE writer, so two concurrent begins would leave it
 * even while a mutation was still in flight.  A spinlock can.  These are touched
 * only from task context, so plain SpinLock is enough under the lock rule; a
 * zero-filled static already holds the unlocked state.
 */
static T_SPLOCK stats_lock;

/*
 * The log has ONE contract: stats_lock.  An earlier revision used a separate
 * odd/even write sequence here, with readers detecting a changed copy and
 * retrying.  It was removed after review (RT-0056): a sequence can reject a copy
 * that changed underneath it, but it cannot make a byte-wise copy of rows[] or a
 * struct copy of events[] atomic, and it left the index publication outside the
 * mutation interval.  Mutual exclusion covers all three cases, and a second
 * mechanism that no longer carries the contract would only invite confusion about
 * which one is authoritative.
 */


#if TM_MQTT
#include "../platform/app_board.h"
static uint32_t log_generation;
static uint32_t newest_row_ms;
static APP_BOARD(telemetry_log_index_t) log_index;

/*
 * Publish the current RAM-log extent to the bounded-reader board.
 */
static void publish_index(void)
{
    const telemetry_log_index_t v = {log_generation, stats.rows, stats.events,
                                     __atomic_load_n(&admit_lost, __ATOMIC_SEQ_CST),
                                     newest_row_ms};
    app_board_publish(&log_index.sequence, log_index.words, &v, sizeof(v));
}

/*
 * Read a bounded snapshot of the current RAM-log index.
 */
int telemetry_log_open(telemetry_log_index_t *out)
{
    if (!out) {
        return E_PAR;
    }
    telemetry_log_index_t v;
    if (!app_board_try_read(&log_index.sequence, log_index.words, &v,
                            sizeof(v))) {
        return E_TMOUT;
    }
    *out = v;
    return E_OK;
}

/*
 * Copy one RAM row only when its generation and index remain valid.
 */
int telemetry_log_row(uint32_t generation, uint32_t index,
                      uint8_t out[TELEMETRY_BYTES])
{
    if (!out) {
        return E_PAR;
    }
    /*
     * RT-0056 defect 3: the payload copy happens with the writer EXCLUDED, not
     * merely checked afterwards.  A sequence can reject a copy that changed
     * underneath it, but it cannot make a byte-wise copy of rows[] atomic, so
     * mutual exclusion is the contract here.  The lock is held only for the
     * bounded copy.
     */
    /*
     * RT-0063: ONE immediate try, no retry loop. The Wi-Fi task runs above
     * Telemetry on this core, so spinning here would block the very holder that
     * must release the lock. ISpinTryLock also keeps this reader non-preemptible
     * for the short copy.
     */
    UINT intsts;
    if (E_OK != ISpinTryLock(&stats_lock, &intsts)) {
        return E_TMOUT;
    }
    telemetry_log_index_t v;
    const int er = telemetry_log_open(&v);
    if (er < E_OK) {
        (void)ISpinUnlock(&stats_lock, intsts);
        return er;
    }
    if (v.generation != generation) {
        (void)ISpinUnlock(&stats_lock, intsts);
        return E_OBJ;
    }
    if (index >= v.rows) {
        (void)ISpinUnlock(&stats_lock, intsts);
        return E_PAR;
    }
    for (unsigned int i = 0u; i < TELEMETRY_BYTES; ++i) {
        out[i] = rows[index][i];
    }
    (void)ISpinUnlock(&stats_lock, intsts);
    return E_OK;
}

/*
 * Copy one RAM event only when its generation and index remain valid.
 */
int telemetry_log_event(uint32_t generation, uint32_t index,
                        telemetry_event_t *out)
{
    if (!out) {
        return E_PAR;
    }
    /* Same contract as the row read: one immediate try, never a spin. */
    UINT intsts;
    if (E_OK != ISpinTryLock(&stats_lock, &intsts)) {
        return E_TMOUT;
    }
    telemetry_log_index_t v;
    const int er = telemetry_log_open(&v);
    if (er < E_OK) {
        (void)ISpinUnlock(&stats_lock, intsts);
        return er;
    }
    if (v.generation != generation) {
        (void)ISpinUnlock(&stats_lock, intsts);
        return E_OBJ;
    }
    if (index >= v.events) {
        (void)ISpinUnlock(&stats_lock, intsts);
        return E_PAR;
    }
    *out = events[index];
    (void)ISpinUnlock(&stats_lock, intsts);
    return E_OK;
}
#endif
#if TM_CONSOLE_USB_CDC
IMPORT volatile UW tm_usb_dropped;

/*
 * Read the USB drop counter, or return zero without USB support.
 */
static UW dropped(void)
{
    return tm_usb_dropped;
}
#else
/*
 * Read the USB drop counter, or return zero without USB support.
 */
static UW dropped(void)
{
    return 0u;
}
#endif
/*
 * Copy the telemetry counters under short interrupt protection.
 */
void telemetry_stats(telemetry_stats_t *out)
{
    if (!out) {
        return;
    }
    /*
     * RT-0071/F1: two different contracts, because these are not the same kind of
     * field.
     *
     * The CONTROL fields - busy, storage_full, lost - drive Vehicle's DUMP
     * admission and storage-full stop decisions, so they come from the atomics
     * the writers update. They are exact, and no lock is taken to get them.
     *
     * The diagnostic counters - rows and events - come from the snapshot the
     * Telemetry task publishes under its own lock. Locked state is never read
     * without synchronisation here, which is what the previous version did when
     * its try-lock failed.
     */
    /* RT-0073/F1: every read here is a defined atomic load.  An aligned word read
     * keeps the hardware value whole, but it is still a plain C access racing an
     * atomic RMW on the other core, which is not the contract claimed. */
    out->busy = (0u != __atomic_load_n(&admit_busy, __ATOMIC_SEQ_CST));
    out->storage_full = (0u != __atomic_load_n(&admit_full, __ATOMIC_SEQ_CST));
    out->lost = __atomic_load_n(&admit_lost, __ATOMIC_SEQ_CST);
    out->rows = __atomic_load_n(&stats_pub.rows, __ATOMIC_SEQ_CST);
    out->events = __atomic_load_n(&stats_pub.events, __ATOMIC_SEQ_CST);
}

/*
 * Increment the loss counter with the log lock ALREADY HELD.  The task uses this
 * inside its own critical section; lose() below takes the lock itself and is for
 * callers outside one.  Using lose() inside the section would self-deadlock,
 * because a spinlock is not recursive.
 */
static void lose_locked(void)
{
    (void)atomic_inc(&admit_lost);
}

/*
 * Account for a telemetry message that could not be stored.
 */
/* Lock-free: reached from telemetry_send on core 0, which must never wait. */
static void lose(void)
{
    (void)atomic_inc(&admit_lost);
}

/*
 * Write a bounded RAM-log snapshot to the local USB console.
 */
static void dump(void)
{
    const UW before = dropped();
    telemetry_stats_t st;
    telemetry_stats(&st);
    tm_printf((UB *)"[robot] KR1 reset %s; %s rows %lu events %lu "
                    "lost_telemetry %lu lost_wakes %lu overruns %lu "
                    "response_max_us %lu faults %u gap %lu..%lu us\n",
              drv_watchdog_reset_name(), info.reason, (unsigned long)st.rows,
              (unsigned long)st.events, (unsigned long)(st.lost - loss_base),
              (unsigned long)info.wake_lost, (unsigned long)info.overruns,
              (unsigned long)info.response_max_us, (unsigned int)info.faults,
              (unsigned long)info.gap_min_us, (unsigned long)info.gap_max_us);
    tm_printf((UB *)"[robot] counts L/R %ld/%ld; line L floor/line %u/%u C "
                    "%u/%u; sampled %lu missed %lu dropped %lu; barcode raw "
                    "%u; ISR max %lu us line sample max %lu us\n",
              (long)info.counts[0], (long)info.counts[1],
              (unsigned int)info.floor[0], (unsigned int)info.line[0],
              (unsigned int)info.floor[1], (unsigned int)info.line[1],
              (unsigned long)info.sampled, (unsigned long)info.missed,
              (unsigned long)info.dropped, (unsigned int)info.raw_bc,
              (unsigned long)info.isr_max_us, (unsigned long)info.line_max_us);
    imu_hump_summary_t hump;
    if (vehicle_hump_try_read(&hump) == E_OK) {
        tm_printf((UB *)"[robot] hump peak_0.1mm %u count %u flags %02x "
                        "gated %u source %u %s\n",
                  (unsigned int)hump.peak_01mm, (unsigned int)hump.count,
                  (unsigned int)hump.flags, (unsigned int)hump.gated,
                  (unsigned int)IMU_HUMP_SOURCE, IMU_HUMP_SOURCE_NAME);
    } else {
        tm_printf((UB *)"[robot] hump no finished run source %u %s\n",
                  (unsigned int)IMU_HUMP_SOURCE, IMU_HUMP_SOURCE_NAME);
    }
    imu_hump_diag_report_t diag;
    if (vehicle_hump_diag_try_read(&diag) == E_OK) {
        imu_hump_diag_print(&diag);
    }
    app_task_report();
    for (uint32_t i = 0u; i < st.rows; ++i) {
        tm_printf((UB *)"KR %lu ", (unsigned long)i);
        for (unsigned int b = 0u; b < TELEMETRY_BYTES; ++b) {
            tm_printf((UB *)"%02x", (unsigned int)rows[i][b]);
        }
        tm_printf((UB *)"\n");
        if ((i + 1u) % 8u == 0u) {
            (void)tk_dly_tsk(DUMP_YIELD_MS);
        }
    }
    for (uint32_t i = 0u; i < st.events; ++i) {
        const telemetry_event_t *e = &events[i];
        tm_printf((UB *)"VE %lu %u %u %u %u\n", (unsigned long)e->ms,
                  (unsigned int)e->kind, (unsigned int)e->from,
                  (unsigned int)e->to, (unsigned int)e->word);
        if ((i + 1u) % 8u == 0u) {
            (void)tk_dly_tsk(DUMP_YIELD_MS);
        }
    }
    app_task_report(); /* Include the earlier report/formatting path in high water. */
    (void)tk_dly_tsk(DUMP_YIELD_MS);
    const UW after = dropped();
    tm_printf((UB *)"[robot] end tm_usb_dropped %lu (dump delta %lu)\n",
              (unsigned long)after, (unsigned long)(after - before));
    /* The dump is over: release the reservation atomically, so the release can
     * never strand or clear somebody else's reservation. */
    (void)atomic_xchg(&admit_busy, 0u);
}

/*
 * Own RAM-log storage and service the existing local USB dump.
 */
static void telemetry_task(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    for (;;) {
        telemetry_message_t m;
        if (tk_rcv_mbf(mbf_telemetry, &m, TMO_FEVR) != (INT)sizeof(m)) {
            continue;
        }
        switch (m.kind) {
        case TELEMETRY_BEGIN:
            /* One critical section for the whole mutation AND the index
             * publication, so a reader can never pair a new row with an old
             * index (RT-0056 defect 2). */
    UINT intsts;
            (void)ISpinLock(&stats_lock, &intsts);
            last_junction = (ja_result_t){0};
            stats.rows = 0u;
            stats.events = 0u;
            (void)atomic_xchg(&admit_full, 0u);
            loss_base = m.loss_base;
#if TM_MQTT
            ++log_generation;
            newest_row_ms = 0u;
            publish_index();
#endif
            publish_stats();
            (void)ISpinUnlock(&stats_lock, intsts);
            break;
        case TELEMETRY_ROW:
            (void)ISpinLock(&stats_lock, &intsts);
            if (stats.rows == TELEMETRY_ROWS) {
                (void)atomic_xchg(&admit_full, 1u);
                lose_locked();
            } else {
                for (unsigned int i = 0u; i < TELEMETRY_BYTES; ++i) {
                    rows[stats.rows][i] = m.data.row[i];
                }
                ++stats.rows;
#if TM_MQTT
                newest_row_ms = drv_time_ms();
#endif
            }
#if TM_MQTT
            publish_index();
#endif
            publish_stats();
            (void)ISpinUnlock(&stats_lock, intsts);
            break;
        case TELEMETRY_EVENT:
            (void)ISpinLock(&stats_lock, &intsts);
            if (stats.events == TELEMETRY_EVENTS) {
                (void)atomic_xchg(&admit_full, 1u);
                lose_locked();
            } else {
                events[stats.events++] = m.data.event;
            }
#if TM_MQTT
            publish_index();
#endif
            publish_stats();
            (void)ISpinUnlock(&stats_lock, intsts);
            break;
        case TELEMETRY_INFO:
            info = m.data.info;
            info.reason[sizeof(info.reason) - 1u] = '\0';
            break;
        case TELEMETRY_JUNCTION:
            last_junction = m.data.junction;
            ja_print(&last_junction);
            break;
        case TELEMETRY_DUMP:
            info = m.data.info;
            info.reason[sizeof(info.reason) - 1u] = '\0';
            motion_diag_dump(1);
            motion_diag_print();
            dump();
            ja_print(&last_junction);
            motion_diag_dump(0);
            break;
        default:
            (void)ISpinLock(&stats_lock, &intsts);
            lose_locked();
#if TM_MQTT
            publish_index();
#endif
            (void)ISpinUnlock(&stats_lock, intsts);
            break;
        }
    }
}

/*
 * Create the bounded telemetry message buffer.
 */
int telemetry_init(void)
{
    if (mbf_telemetry > 0) {
        return E_OK;
    }
    const T_CMBF c = {.mbfatr = TA_TFIFO,
                      .bufsz = 4096,
                      .maxmsz = (INT)sizeof(telemetry_message_t)};
    mbf_telemetry = tk_cre_mbf(&c);
#if TM_MQTT
    if (mbf_telemetry > 0) {
        publish_index();
    }
#endif
    return mbf_telemetry < 0 ? (int)mbf_telemetry : E_OK;
}

/*
 * Start the RAM-log owner on its registered static stack.
 */
int telemetry_start(void)
{
    if (mbf_telemetry <= 0 || task > 0) {
        return E_OBJ;
    }
    task = app_start_task((FP)telemetry_task, 30, app_prc_core1(),
                          "telemetry_task");
    return task < 0 ? (int)task : E_OK;
}

/*
 * Report whether the telemetry task has started.
 */
bool telemetry_running(void)
{
    return mbf_telemetry > 0 && task > 0;
}

/*
 * Submit a value message without waiting for the consumer.
 */
int telemetry_send(const telemetry_message_t *message)
{
    if (mbf_telemetry <= 0 || task <= 0) {
        return E_OBJ;
    }
    if (!message || message->kind > TELEMETRY_JUNCTION) {
        return E_PAR;
    }
    telemetry_message_t m = *message;
    /*
     * RT-0071/F1: reservation and rollback are ONE coherent operation on the
     * admission atomic, with no payload lock involved.
     *
     * Reserve only if the reservation is free - a single compare-exchange, so a
     * second DUMP is refused rather than admitted beside the first.  Release only
     * what this call took, so a failed send can never clear another DUMP's
     * reservation.  Vehicle never waits, because there is no lock here at all, and
     * it never observes busy=false while its own DUMP is pending.
     */
    bool reserved = false;
    if (m.kind == TELEMETRY_DUMP) {
        if (0u != atomic_cmpxchg(&admit_busy, 1u, 0u)) {
            lose();
            return E_OBJ;
        }
        reserved = true;
    }
    m.loss_base = __atomic_load_n(&admit_lost, __ATOMIC_SEQ_CST);
    const ER er = tk_snd_mbf(mbf_telemetry, &m, (INT)sizeof(m), TMO_POL);
    if (er < E_OK) {
        /* A real message-buffer failure: the caller may stop the run. */
        lose();
        if (reserved) {
            (void)atomic_xchg(&admit_busy, 0u);
        }
    }
    return (int)er;
}

_Static_assert(sizeof(telemetry_event_t) == 12u, "VE event layout");
