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
 * Separate from log_generation, which stays the capture-run identity that MQTT
 * live and DUMP compare against.  This sequence marks a row, event or reset
 * mutation in progress: odd means a mutation is underway, even means the log is
 * consistent.  It exists because the run identity alone cannot tell a reader
 * that the row it is copying was overwritten underneath it.  Unlike the board
 * sequence, zero is a valid consistent state here, because it is the state
 * before the first mutation.  It is deliberately OUTSIDE the TM_MQTT guard: the
 * writer and its readers exist in every profile, and only the run identity is
 * MQTT-side.
 */
static uint32_t log_write_seq;

/* Writer side: make the sequence odd for the duration of one mutation. */
static void log_write_begin(void)
{
    const uint32_t seq = __atomic_load_n(&log_write_seq, __ATOMIC_SEQ_CST);
    __atomic_store_n(&log_write_seq, seq + 1u, __ATOMIC_SEQ_CST);
}

/* Writer side: make the sequence even again once the mutation is published. */
static void log_write_end(void)
{
    const uint32_t seq = __atomic_load_n(&log_write_seq, __ATOMIC_SEQ_CST);
    __atomic_store_n(&log_write_seq, seq + 1u, __ATOMIC_SEQ_CST);
}
/* Bounded reader retries. A writer that keeps mutating must never block a
 * caller, so after this many attempts the read fails instead of returning data
 * that may not be stable. */
#define TELEMETRY_READ_TRIES 3u

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
                                     stats.lost, newest_row_ms};
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
    for (unsigned int attempt = 0u; attempt < TELEMETRY_READ_TRIES; ++attempt) {
        telemetry_log_index_t v;
        const int er = telemetry_log_open(&v);
        if (er < E_OK) {
            return er;
        }
        if (v.generation != generation) {
            return E_OBJ;
        }
        if (index >= v.rows) {
            return E_PAR;
        }
        /* A mutation in progress means the payload is not consistent yet. */
        const uint32_t before = __atomic_load_n(&log_write_seq, __ATOMIC_SEQ_CST);
        if (0u != (before & 1u)) {
            continue;
        }
        uint8_t scratch[TELEMETRY_BYTES];
        for (unsigned int i = 0u; i < TELEMETRY_BYTES; ++i) {
            scratch[i] = rows[index][i];
        }
        /* Copied, but the writer may have overwritten the row underneath us. */
        if (before != __atomic_load_n(&log_write_seq, __ATOMIC_SEQ_CST)) {
            continue;
        }
        /* The run identity must also still hold now that the copy is done. */
        telemetry_log_index_t again;
        if (telemetry_log_open(&again) < E_OK) {
            continue;
        }
        if (again.generation != generation) {
            return E_OBJ;
        }
        for (unsigned int i = 0u; i < TELEMETRY_BYTES; ++i) {
            out[i] = scratch[i];
        }
        return E_OK;
    }
    /* Bounded failure: never hand back a row that may not be stable. */
    return E_TMOUT;
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
    for (unsigned int attempt = 0u; attempt < TELEMETRY_READ_TRIES; ++attempt) {
        telemetry_log_index_t v;
        const int er = telemetry_log_open(&v);
        if (er < E_OK) {
            return er;
        }
        if (v.generation != generation) {
            return E_OBJ;
        }
        if (index >= v.events) {
            return E_PAR;
        }
        const uint32_t before = __atomic_load_n(&log_write_seq, __ATOMIC_SEQ_CST);
        if (0u != (before & 1u)) {
            continue;
        }
        const telemetry_event_t scratch = events[index];
        if (before != __atomic_load_n(&log_write_seq, __ATOMIC_SEQ_CST)) {
            continue;
        }
        telemetry_log_index_t again;
        if (telemetry_log_open(&again) < E_OK) {
            continue;
        }
        if (again.generation != generation) {
            return E_OBJ;
        }
        *out = scratch;
        return E_OK;
    }
    /* Bounded failure: an event read gets the same contract as a row read. */
    return E_TMOUT;
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
     * Phase 3: the writer runs on core 1 while this reader can run on core 0, so
     * a local interrupt mask no longer covers it.  This takes the spinlock rather
     * than the write sequence, because `lost` and `busy` have two writers - a
     * submitter on core 0 and the task on core 1 - and a sequence cannot serve
     * more than one writer.  The struct is a set of 32-bit diagnostic counters
     * that never gate control, so the cost is limited to a short spin.
     */
    (void)SpinLock(&stats_lock);
    *out = stats;
    (void)SpinUnlock(&stats_lock);
}

/*
 * Account for a telemetry message that could not be stored.
 */
static void lose(void)
{
    (void)SpinLock(&stats_lock);
    ++stats.lost;
    (void)SpinUnlock(&stats_lock);
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
    (void)SpinLock(&stats_lock);
    stats.busy = false;
    (void)SpinUnlock(&stats_lock);
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
            log_write_begin();
            last_junction = (ja_result_t){0};
            stats.rows = 0u;
            stats.events = 0u;
            stats.storage_full = false;
            loss_base = m.loss_base;
#if TM_MQTT
            ++log_generation;
            newest_row_ms = 0u;
#endif
            log_write_end();
            break;
        case TELEMETRY_ROW:
            log_write_begin();
            if (stats.rows == TELEMETRY_ROWS) {
                stats.storage_full = true;
                lose();
            } else {
                for (unsigned int i = 0u; i < TELEMETRY_BYTES; ++i) {
                    rows[stats.rows][i] = m.data.row[i];
                }
                ++stats.rows;
#if TM_MQTT
                newest_row_ms = drv_time_ms();
#endif
            }
            log_write_end();
            break;
        case TELEMETRY_EVENT:
            log_write_begin();
            if (stats.events == TELEMETRY_EVENTS) {
                stats.storage_full = true;
                lose();
            } else {
                events[stats.events++] = m.data.event;
            }
            log_write_end();
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
            lose();
            break;
        }
#if TM_MQTT
        publish_index();
#endif
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
    (void)SpinLock(&stats_lock);
    const bool conflict = stats.busy;
    m.loss_base = stats.lost;
    if (!conflict && m.kind == TELEMETRY_DUMP) {
        stats.busy = true;
    }
    (void)SpinUnlock(&stats_lock);
    if (conflict) {
        lose();
        return E_OBJ;
    }
    const ER er = tk_snd_mbf(mbf_telemetry, &m, (INT)sizeof(m), TMO_POL);
    if (er < E_OK) {
        lose();
        if (m.kind == TELEMETRY_DUMP) {
            (void)SpinLock(&stats_lock);
            stats.busy = false;
            (void)SpinUnlock(&stats_lock);
        }
    }
    return (int)er;
}

_Static_assert(sizeof(telemetry_event_t) == 12u, "VE event layout");
