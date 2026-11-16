/**
 * @file log_stream.c
 * @brief Live RAM-log stream and DUMP refill. See log_stream.h.
 *
 * All functions except log_stream_dump_request() run on the Wi-Fi task only.
 * Static frame buffers are therefore safe and keep the task stack small.
 */
#if TM_MQTT

#include "log_stream.h"

#include <string.h>
#include <tk/syslib.h>
#include <tk/tkernel.h>

#include "mqtt_wire.h"
#include "telemetry.h"
#include "../platform/drv_time.h"

/** Frames in flight tracked for one DUMP. Equals LOG_DUMP_WINDOW. */
#define SLOT_COUNT LOG_DUMP_WINDOW
/** Word offsets in the 16-byte body of a control frame. */
#define BODY_WORD0 0u
#define BODY_WORD1 4u
#define BODY_WORD2 8u
/** Bytes in one 32-bit word. */
#define WORD_BYTES 4u
/** Bit shift per byte. */
#define BYTE_BITS 8u
/** Event flag: time is run-relative (matches the live event stream). */
#define EVENT_FLAG_RUN_TIME 1u
/** Serial that no frame uses. */
#define SERIAL_NONE 0u

typedef enum {
    FRAME_ROWS,
    FRAME_EVENTS,
    FRAME_BEGIN,
    FRAME_END,
    FRAME_ABORT
} frame_type_t;

typedef enum {
    DUMP_IDLE,
    DUMP_OPEN,
    DUMP_BEGIN,
    DUMP_BEGIN_WAIT,
    DUMP_DATA,
    DUMP_END,
    DUMP_END_WAIT,
    DUMP_ABORT,
    DUMP_ABORT_WAIT
} dump_stage_t;

typedef struct {
    bool acked;
    frame_type_t type;
    uint32_t serial;
    uint32_t first;
    uint32_t count;
} slot_t;

typedef struct {
    bool have_generation;
    bool jump_pending;
    bool retry_armed;
    uint8_t flags_pending;
    uint32_t generation;
    uint32_t next;
    uint32_t rows_seen;
    uint32_t skipped_pending;
    uint32_t last_publish_ms;
    uint32_t retry_at_ms;
} live_t;

typedef struct {
    dump_stage_t stage;
    uint32_t id;    bool retry_armed;
    uint32_t generation;
    uint32_t rows_total;
    uint32_t events_total;
    uint32_t lost;
    uint32_t rows_sent;
    uint32_t rows_acked;
    uint32_t events_sent;
    uint32_t events_acked;
    uint32_t next_serial;
    uint32_t retry_at_ms;
    uint32_t head;
    uint32_t count;
    slot_t slots[SLOT_COUNT];
} dump_t;

static live_t s_live;
#include <tk/smp.h>   /* T_SPLOCK, SpinLock: cross-core DUMP request state */

/* Phase 4: `log_stream_dump_request()` is called from Vehicle on core 0
 * (vehicle_task.c:404) while this module's stepping runs in the Wi-Fi task on
 * core 1, so a core-local mask protects neither.  Task context only, so plain
 * SpinLock is the right variant under the lock rule.  A zero-filled static
 * already holds the unlocked state. */
static T_SPLOCK dump_lock;

static dump_t s_dump;

/*
 * Phase 4 split (RT-0060).  The admission state is the ONLY DUMP state both cores
 * touch: Vehicle submits a request from core 0, the Wi-Fi task consumes it and
 * later returns it to IDLE.  It is a small state machine under dump_lock.
 * Everything else in dump_t - the transfer stage, cursors, slots and
 * acknowledgement counters - is owned by the Wi-Fi task and is never read from
 * core 0.  A plain single-writer board would not be enough here, because Vehicle
 * writes the request while the Wi-Fi task consumes it, and the request ID must
 * match the accepted state.
 */
typedef enum {
    DUMP_ADMIT_IDLE = 0,
    DUMP_ADMIT_PENDING = 1,
    DUMP_ADMIT_ACTIVE = 2
} dump_admit_t;

static struct {
    dump_admit_t state;
    uint32_t request_id;
} s_admit;
static log_stream_stats_t s_stats;
static uint8_t s_records[RM_LOG_BATCH_MAX_COUNT * RM_LOG_RECORD_SIZE];
static uint8_t s_frame[RM_LOG_BATCH_MAX_SIZE];
static uint32_t s_boot_id;

/**
 * Wrap-safe "now is at or after deadline" test for 32-bit millisecond times.
 */
static bool time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline)) >= 0;
}

/**
 * Store a 32-bit value into a frame body in little-endian order.
 */
static void store_word(uint8_t *out, uint32_t value)
{
    uint32_t i;

    for (i = 0u; i < WORD_BYTES; ++i) {
        out[i] = (uint8_t)(value >> (BYTE_BITS * i));
    }
}

/**
 * Copy rows [first, first + count) of one generation into s_records.
 * @return Number of rows copied (0 if none could be read).
 */
static uint32_t copy_rows(uint32_t generation, uint32_t first, uint32_t count)
{
    uint32_t copied = 0u;
    bool ok = true;

    while ((copied < count) && ok) {
        const int er =
            telemetry_log_row(generation, first + copied,
                              &s_records[copied * RM_LOG_RECORD_SIZE]);

        if (er < E_OK) {
            ok = false;
        } else {
            ++copied;
        }
    }
    return copied;
}

/**
 * Copy events [first, first + count) of one generation into s_records, in
 * wire form.
 * @return Number of events copied (0 if none could be read).
 */
static uint32_t copy_events(uint32_t generation, uint32_t first, uint32_t count)
{
    uint32_t copied = 0u;
    bool ok = true;

    while ((copied < count) && ok) {
        telemetry_event_t raw;
        const int er = telemetry_log_event(generation, first + copied, &raw);

        if (er < E_OK) {
            ok = false;
        } else {
            rm_event_t event;

            (void)memset(&event, 0, sizeof(event));
            event.kind = raw.kind;
            event.from = raw.from;
            event.to = raw.to;
            event.sequence = first + copied;
            event.uptime_ms = raw.ms;
            event.word = raw.word;
            event.detail = raw.detail;
            event.flags = EVENT_FLAG_RUN_TIME;
            rm_pack_log_event(&s_records[copied * RM_LOG_RECORD_SIZE], &event);
            ++copied;
        }
    }
    return copied;
}

/**
 * Live stream step. Publishes at most LOG_LIVE_SENDS_PER_POLL batches.
 *
 * The cursor follows the newest rows. It jumps (skipping older rows) when it
 * lags more than LOG_LIVE_LAG_MAX_ROWS or after a link loss. A new run
 * generation restarts the cursor at index 0.
 */
static void live_service(log_stream_publish_fn publish)
{
    telemetry_log_index_t index;
    telemetry_log_index_t after;
    uint32_t pending;
    uint32_t count;
    uint32_t copied;
    uint32_t now = drv_time_ms();
    rm_log_batch_t header;
    size_t length;

    if (telemetry_log_open(&index) < E_OK) {
        return;
    }
    if (!s_live.have_generation) {
        s_live.have_generation = true;
        s_live.generation = index.generation;
    } else if (index.generation != s_live.generation) {
        s_live.skipped_pending = (s_live.rows_seen > s_live.next)
                                     ? (s_live.rows_seen - s_live.next)
                                     : 0u;
        s_live.flags_pending = RM_LOG_FLAG_NEW_GENERATION;
        s_live.generation = index.generation;
        s_live.next = 0u;
        s_live.jump_pending = false;
    } else {
        /* Same generation: nothing to reset. */
    }
    s_live.rows_seen = index.rows;
    if (s_live.next > index.rows) {
        s_live.next = index.rows;
    }
    pending = index.rows - s_live.next;
    if ((pending > LOG_LIVE_LAG_MAX_ROWS) ||
        (s_live.jump_pending && (pending > LOG_LIVE_RESUME_ROWS))) {
        const uint32_t skip = pending - LOG_LIVE_RESUME_ROWS;

        s_live.next += skip;
        s_live.skipped_pending += skip;
        s_live.flags_pending |= RM_LOG_FLAG_JUMPED;
        pending = LOG_LIVE_RESUME_ROWS;
        ++s_stats.live_jumps;
        s_stats.live_skipped += skip;
    }
    s_live.jump_pending = false;
    if ((pending == 0u) ||
        (s_live.retry_armed && !time_reached(now, s_live.retry_at_ms))) {
        return;
    }
    if ((pending < LOG_LIVE_BATCH_ROWS) &&
        !time_reached(now, s_live.last_publish_ms + LOG_LIVE_PERIOD_MS)) {
        return;
    }
    count = (pending < LOG_LIVE_BATCH_ROWS) ? pending : LOG_LIVE_BATCH_ROWS;
    copied = copy_rows(s_live.generation, s_live.next, count);
    if ((copied == 0u) || (telemetry_log_open(&after) < E_OK) ||
        (after.generation != s_live.generation)) {
        return;
    }
    (void)memset(&header, 0, sizeof(header));
    header.kind = RM_LOG_ROWS;
    header.count = (uint8_t)copied;
    header.flags = s_live.flags_pending;
    header.generation = s_live.generation;
    header.first_index = s_live.next;
    header.publish_ms = drv_time_ms();
    header.newest_ms = after.newest_ms;
    header.log_rows = after.rows;
    header.skipped_before = s_live.skipped_pending;
    header.boot_id = s_boot_id;
    length = rm_pack_log_batch(s_frame, sizeof(s_frame), &header, s_records);
    if ((length == 0u) || (publish(s_frame, length, LOG_QOS_LIVE, 0u) != 0)) {
        s_live.retry_armed = true;
        s_live.retry_at_ms = now + LOG_RETRY_MS;
        ++s_stats.live_failed;
        return;
    }
    s_live.retry_armed = false;
    s_live.next += copied;
    s_live.skipped_pending = 0u;
    s_live.flags_pending = 0u;
    s_live.last_publish_ms = now;
    ++s_stats.live_batches;
    s_stats.live_rows += copied;
}

/**
 * Forget all unacknowledged DUMP frames and resend from the acknowledged
 * cursors. Used after a failed publish, a failed acknowledgement or a link
 * loss. Late callbacks for forgotten frames are ignored (serial not found).
 */
static void dump_rewind(void)
{
    const bool had_frames = (s_dump.count != 0u);

    s_dump.head = 0u;
    s_dump.count = 0u;
    s_dump.rows_sent = s_dump.rows_acked;
    s_dump.events_sent = s_dump.events_acked;
    if (s_dump.stage == DUMP_BEGIN_WAIT) {
        s_dump.stage = DUMP_BEGIN;
    } else if (s_dump.stage == DUMP_END_WAIT) {
        s_dump.stage = DUMP_END;
    } else if (s_dump.stage == DUMP_ABORT_WAIT) {
        s_dump.stage = DUMP_ABORT;
    } else {
        /* DATA stays DATA; other stages hold no frames. */
    }
    if (had_frames) {
        ++s_stats.dump_rewinds;
    }
}

/**
 * Publish one DUMP frame with QoS 1 and record it in the window.
 * @return True if the frame was accepted.
 */
static bool dump_submit(log_stream_publish_fn publish, frame_type_t type,
                        uint32_t first, uint32_t count, size_t length)
{
    const uint32_t tail = (s_dump.head + s_dump.count) % SLOT_COUNT;
    uint32_t serial = s_dump.next_serial + 1u;
    bool accepted;

    if (serial == SERIAL_NONE) {
        serial = 1u;
    }
    accepted = (publish(s_frame, length, LOG_QOS_DUMP, serial) == 0);
    if (accepted) {
        s_dump.next_serial = serial;
        s_dump.slots[tail].acked = false;
        s_dump.slots[tail].type = type;
        s_dump.slots[tail].serial = serial;
        s_dump.slots[tail].first = first;
        s_dump.slots[tail].count = count;
        ++s_dump.count;
        ++s_stats.dump_frames;
    } else {
        s_dump.retry_armed = true;
        s_dump.retry_at_ms = drv_time_ms() + LOG_RETRY_MS;
        ++s_stats.dump_failed;
    }
    return accepted;
}

/**
 * Build and publish a BEGIN, END or ABORT control frame.
 * @return True if the frame was accepted.
 */
static bool dump_control(log_stream_publish_fn publish, frame_type_t type)
{
    rm_log_t frame;
    uint32_t word0 = s_dump.rows_acked;
    uint32_t word1 = s_dump.events_acked;
    uint32_t word2 = s_dump.lost;
    uint8_t kind = RM_LOG_END;

    (void)memset(&frame, 0, sizeof(frame));
    if (type == FRAME_BEGIN) {
        kind = RM_LOG_BEGIN;
        word0 = s_dump.rows_total;
        word1 = s_dump.events_total;
    } else if (type == FRAME_ABORT) {
        kind = RM_LOG_ABORT;
        word0 = LOG_DUMP_REASON_GENERATION;
        word1 = s_dump.rows_acked;
        word2 = s_dump.events_acked;
    } else {
        /* END keeps the defaults. */
    }
    frame.kind = kind;
    frame.boot_id = s_boot_id;
    frame.dump_id = s_dump.id;
    frame.generation = s_dump.generation;
    store_word(&frame.body[BODY_WORD0], word0);
    store_word(&frame.body[BODY_WORD1], word1);
    store_word(&frame.body[BODY_WORD2], word2);
    rm_pack_log(s_frame, &frame);
    return dump_submit(publish, type, 0u, 0u, RM_LOG_SIZE);
}

/**
 * Build and publish one DUMP data batch (rows first, then events).
 * @return True if a frame was accepted.
 */
static bool dump_data(log_stream_publish_fn publish)
{
    const bool rows = (s_dump.rows_sent < s_dump.rows_total);
    const uint32_t left = rows ? (s_dump.rows_total - s_dump.rows_sent)
                               : (s_dump.events_total - s_dump.events_sent);
    const uint32_t first = rows ? s_dump.rows_sent : s_dump.events_sent;
    const uint32_t want =
        (left < LOG_DUMP_BATCH_RECORDS) ? left : LOG_DUMP_BATCH_RECORDS;
    const uint32_t copied = rows ? copy_rows(s_dump.generation, first, want)
                                 : copy_events(s_dump.generation, first, want);
    rm_log_batch_t header;
    size_t length;
    bool accepted = false;

    if (copied != 0u) {
        (void)memset(&header, 0, sizeof(header));
        header.kind = rows ? RM_LOG_ROWS : RM_LOG_EVENTS;
        header.count = (uint8_t)copied;
        header.flags = RM_LOG_FLAG_REFILL;
        header.stream_id = s_dump.id;
        header.generation = s_dump.generation;
        header.first_index = first;
        header.publish_ms = drv_time_ms();
        header.log_rows = s_dump.rows_total;
        header.boot_id = s_boot_id;
        length =
            rm_pack_log_batch(s_frame, sizeof(s_frame), &header, s_records);
        accepted = (length != 0u) &&
                   dump_submit(publish, rows ? FRAME_ROWS : FRAME_EVENTS, first,
                               copied, length);
    }
    if (accepted) {
        if (rows) {
            s_dump.rows_sent += copied;
        } else {
            s_dump.events_sent += copied;
        }
    }
    return accepted;
}

/**
 * Freeze the snapshot that a new dump will send.
 * @return True if the log index could be read.
 */
static bool dump_open(void)
{
    telemetry_log_index_t index;
    const bool ok = (telemetry_log_open(&index) >= E_OK);

    if (ok) {
        s_dump.generation = index.generation;
        s_dump.rows_total = index.rows;
        s_dump.events_total = index.events;
        s_dump.lost = index.lost;
        s_dump.rows_sent = 0u;
        s_dump.rows_acked = 0u;
        s_dump.events_sent = 0u;
        s_dump.events_acked = 0u;
        s_dump.head = 0u;
        s_dump.count = 0u;
        s_dump.stage = DUMP_BEGIN;
    }
    return ok;
}

/**
 * Move to ABORT if the run generation changed since the dump opened.
 */
static void dump_check_generation(void)
{
    telemetry_log_index_t index;
    const bool active =
        (s_dump.stage != DUMP_IDLE) && (s_dump.stage != DUMP_OPEN) &&
        (s_dump.stage != DUMP_ABORT) && (s_dump.stage != DUMP_ABORT_WAIT);

    if (active && (telemetry_log_open(&index) >= E_OK) &&
        (index.generation != s_dump.generation)) {
        s_dump.head = 0u;
        s_dump.count = 0u;
        s_dump.stage = DUMP_ABORT;
    }
}

/**
 * DUMP step. Publishes at most LOG_DUMP_SENDS_PER_POLL frames.
 */
static void dump_service(log_stream_publish_fn publish)
{
    uint32_t sends = 0u;
    const uint32_t now = drv_time_ms();

    if (s_dump.stage == DUMP_IDLE) {
        /* Pending to active handoff: take the request ID under the lock, then
         * change the Wi-Fi-owned stage outside it. */
        bool accepted = false;
        UINT intsts;
        (void)ISpinLock(&dump_lock, &intsts);
        if (DUMP_ADMIT_PENDING == s_admit.state) {
            s_admit.state = DUMP_ADMIT_ACTIVE;
            s_dump.id = s_admit.request_id;
            accepted = true;
        }
        (void)ISpinUnlock(&dump_lock, intsts);
        if (accepted) {
            s_dump.stage = DUMP_OPEN;
        }
    }
    if ((s_dump.stage == DUMP_OPEN) && !dump_open()) {
        return;
    }
    dump_check_generation();
    if (s_dump.retry_armed && !time_reached(now, s_dump.retry_at_ms)) {
        return;
    }
    s_dump.retry_armed = false;
    if ((s_dump.stage == DUMP_BEGIN) && (s_dump.count == 0u) &&
        dump_control(publish, FRAME_BEGIN)) {
        s_dump.stage = DUMP_BEGIN_WAIT;
    }
    while ((s_dump.stage == DUMP_DATA) && (sends < LOG_DUMP_SENDS_PER_POLL) &&
           (s_dump.count < LOG_DUMP_WINDOW) &&
           ((s_dump.rows_sent < s_dump.rows_total) ||
            (s_dump.events_sent < s_dump.events_total)) &&
           dump_data(publish)) {
        ++sends;
    }
    if ((s_dump.stage == DUMP_DATA) && (s_dump.count == 0u) &&
        (s_dump.rows_acked == s_dump.rows_total) &&
        (s_dump.events_acked == s_dump.events_total)) {
        s_dump.stage = DUMP_END;
    }
    if ((s_dump.stage == DUMP_END) && (s_dump.count == 0u) &&
        dump_control(publish, FRAME_END)) {
        s_dump.stage = DUMP_END_WAIT;
    }
    if ((s_dump.stage == DUMP_ABORT) && (s_dump.count == 0u) &&
        dump_control(publish, FRAME_ABORT)) {
        s_dump.stage = DUMP_ABORT_WAIT;
    }
}

/*
 * Reset live and DUMP stream state before network service starts.
 */
void log_stream_init(uint32_t boot_id)
{
    s_boot_id = boot_id;
    (void)memset(&s_live, 0, sizeof(s_live));
    (void)memset(&s_dump, 0, sizeof(s_dump));
    (void)memset(&s_stats, 0, sizeof(s_stats));
    UINT intsts;
    (void)ISpinLock(&dump_lock, &intsts);
    s_admit.state = DUMP_ADMIT_IDLE;
    s_admit.request_id = 0u;
    (void)ISpinUnlock(&dump_lock, intsts);
}

/*
 * Rewind DUMP state and require a freshness jump after link recovery.
 */
void log_stream_link_down(void)
{
    s_live.jump_pending = true;
    dump_rewind();
}

/*
 * Accept one bounded DUMP request without doing any network work.
 */
int log_stream_dump_request(uint32_t request_id)
{
    int result = E_OBJ;

    /*
     * RT-0069: Vehicle runs at priority 13 on core 0 and Health at 31, so Vehicle
     * must not wait on a lock a lower-priority holder took and cannot release.
     * One immediate try.  When it fails the request is refused rather than
     * delayed, and no duplicate can slip in because the state changes only on
     * success.
     */
    UINT intsts;
    if (E_OK != ISpinTryLock(&dump_lock, &intsts)) {
        return E_TMOUT;
    }
    if (DUMP_ADMIT_IDLE == s_admit.state) {
        s_admit.request_id = request_id;
        s_admit.state = DUMP_ADMIT_PENDING;
        result = E_OK;
    }
    (void)ISpinUnlock(&dump_lock, intsts);
    return result;
}

/*
 * Report whether a DUMP request or transfer is active.
 */
bool log_stream_dump_active(void)
{
    /* True for PENDING and ACTIVE, and never reads the Wi-Fi-owned stage.
     * Interrupt-preserving: Health holds this on core 0 and Vehicle preempts it. */
    UINT intsts;
    (void)ISpinLock(&dump_lock, &intsts);
    const bool active = (DUMP_ADMIT_PENDING == s_admit.state) ||
                        (DUMP_ADMIT_ACTIVE == s_admit.state);
    (void)ISpinUnlock(&dump_lock, intsts);
    return active;
}

/*
 * Service bounded live and refill batches through the supplied sink. A
 * paused live stream (IDLE mode) does no work; DUMP refill always runs.
 */
void log_stream_service(log_stream_publish_fn publish, bool live)
{
    if (live) {
        live_service(publish);
    } else {
        /* Paused live stream: resume with a jump to the newest rows, so the
         * first RUN batches are fresh and the skip is reported. */
        s_live.jump_pending = true;
    }
    dump_service(publish);
}

/**
 * Apply an acknowledged frame to the DUMP cursors, in send order.
 */
static void dump_apply(const slot_t *slot)
{
    if (slot->type == FRAME_ROWS) {
        s_dump.rows_acked = slot->first + slot->count;
    } else if (slot->type == FRAME_EVENTS) {
        s_dump.events_acked = slot->first + slot->count;
    } else if (slot->type == FRAME_BEGIN) {
        s_dump.stage = DUMP_DATA;
    } else {
        /* END or ABORT acknowledged: the transfer is over, so admission returns
         * to IDLE and a new request can be accepted. */
        s_dump.stage = DUMP_IDLE;
        UINT intsts;
        (void)ISpinLock(&dump_lock, &intsts);
        s_admit.state = DUMP_ADMIT_IDLE;
        (void)ISpinUnlock(&dump_lock, intsts);
    }
}

/*
 * Apply one matching DUMP acknowledgement or rewind on failure.
 */
void log_stream_complete(uint32_t token, bool success)
{
    uint32_t i;
    bool found = false;

    for (i = 0u; i < s_dump.count; ++i) {
        slot_t *slot = &s_dump.slots[(s_dump.head + i) % SLOT_COUNT];

        if (slot->serial == token) {
            found = true;
            slot->acked = success;
        }
    }
    if (found && !success) {
        dump_rewind();
        s_dump.retry_armed = true;
        s_dump.retry_at_ms = drv_time_ms() + LOG_RETRY_MS;
        ++s_stats.dump_failed;
    }
    while (found && success && (s_dump.count != 0u) &&
           s_dump.slots[s_dump.head].acked) {
        dump_apply(&s_dump.slots[s_dump.head]);
        s_dump.head = (s_dump.head + 1u) % SLOT_COUNT;
        --s_dump.count;
    }
}

/*
 * Copy stream counters without exposing mutable stream state.
 */
void log_stream_stats(log_stream_stats_t *out)
{
    if (out != NULL) {
        *out = s_stats;
    }
}

#endif /* TM_MQTT */
