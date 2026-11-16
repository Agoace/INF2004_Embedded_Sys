/*
 * Host tests for the live RAM-log stream and the DUMP refill (log_stream.c).
 * Time, the RAM log, and the MQTT publish sink are test doubles.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tk/smp.h>

#include "../app_program/subsystem1/log_stream.h"
#include "../app_program/subsystem1/mqtt_wire.h"
#include "../app_program/subsystem1/telemetry.h"

#define MAX_FRAMES 64u
#define PENDING_MAX 8u

typedef struct {
    uint8_t data[RM_LOG_BATCH_MAX_SIZE];
    size_t length;
    uint8_t qos;
    uint32_t token;
} frame_t;

static uint32_t now_ms;
static telemetry_log_index_t ram;
static bool ram_busy;
static unsigned int row_fail_at; /* 1-based row read that fails; 0 = none */
static unsigned int row_reads;
static bool row_gen_error;
static bool bump_generation_on_open;
static unsigned int opens;
#define irq_off test_smp_irq_mask
static frame_t frames[MAX_FRAMES];
static unsigned int frame_count;
static unsigned int publish_fail_count;
static unsigned int unacked_peak;
static unsigned int unacked_now;
static bool track_unacked;
static bool keep_live;
static uint8_t fail_qos;

void mock_di(UINT *irq)
{
    *irq = test_smp_irq_save();
}

void mock_ei(UINT irq)
{
    test_smp_irq_restore(irq);
}

uint32_t drv_time_ms(void)
{
    return now_ms;
}

int telemetry_log_open(telemetry_log_index_t *out)
{
    ++opens;
    if (ram_busy) {
        return E_TMOUT;
    }
    if (bump_generation_on_open && (opens > 1u)) {
        ++ram.generation;
        ram.rows = 0u;
        bump_generation_on_open = false;
    }
    *out = ram;
    return E_OK;
}

int telemetry_log_row(uint32_t generation, uint32_t index,
    uint8_t out[TELEMETRY_BYTES])
{
    ++row_reads;
    if ((row_fail_at != 0u) && (row_reads == row_fail_at)) {
        return row_gen_error ? E_OBJ : E_TMOUT;
    }
    assert(index < ram.rows);
    if (generation != ram.generation) {
        return E_OBJ;
    }
    (void)memset(out, 0, TELEMETRY_BYTES);
    out[0] = (uint8_t)index;
    out[1] = (uint8_t)(index >> 8);
    out[2] = (uint8_t)generation;
    return E_OK;
}

int telemetry_log_event(uint32_t generation, uint32_t index,
    telemetry_event_t *out)
{
    assert(index < ram.events);
    if (generation != ram.generation) {
        return E_OBJ;
    }
    *out = (telemetry_event_t){ .ms = 1000u + index, .word = (uint16_t)index,
        .kind = 2u, .from = 1u, .to = 3u, .detail = 9u };
    return E_OK;
}

static int publish_sink(const uint8_t *data, size_t length, uint8_t qos,
    uint32_t token)
{
    assert(length <= RM_LOG_BATCH_MAX_SIZE && !irq_off);
    assert((qos == 0u) || (qos == 1u));
    if ((publish_fail_count != 0u) && (qos == fail_qos)) {
        --publish_fail_count;
        return 1;
    }
    if ((qos == 0u) && !keep_live) {
        return 0;
    }
    assert(frame_count < MAX_FRAMES);
    memcpy(frames[frame_count].data, data, length);
    frames[frame_count].length = length;
    frames[frame_count].qos = qos;
    frames[frame_count].token = token;
    ++frame_count;
    if (track_unacked && (qos == 1u)) {
        ++unacked_now;
        if (unacked_now > unacked_peak) {
            unacked_peak = unacked_now;
        }
    }
    return 0;
}

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void reset(void)
{
    log_stream_init(0x11223344u);
    memset(&ram, 0, sizeof(ram));
    ram.generation = 5u;
    now_ms = 1000u;
    ram_busy = false;
    row_fail_at = 0u;
    row_reads = 0u;
    row_gen_error = false;
    bump_generation_on_open = false;
    opens = 0u;
    frame_count = 0u;
    publish_fail_count = 0u;
    unacked_peak = 0u;
    unacked_now = 0u;
    track_unacked = false;
    keep_live = true;
    fail_qos = 0u;
}

/* Dump tests ignore the live frames that the same polls publish. */
static void dump_mode(void)
{
    keep_live = false;
    fail_qos = 1u;
}

static void poll(void)
{
    log_stream_service(publish_sink, true);
    assert(irq_off == 0u);
}

/* Acknowledge frames[index]. */
static void ack(unsigned int index)
{
    assert(index < frame_count && frames[index].qos == 1u);
    log_stream_complete(frames[index].token, true);
}

static void assert_batch(const frame_t *f, uint8_t kind, uint8_t count,
    uint8_t flags, uint32_t stream, uint32_t generation, uint32_t first,
    uint32_t skipped)
{
    assert(f->length == RM_LOG_BATCH_HEADER_SIZE +
        ((size_t)count * RM_LOG_RECORD_SIZE));
    assert(f->data[0] == RM_LOG_VERSION && f->data[1] == kind);
    assert(f->data[2] == count && f->data[3] == flags);
    assert(u32(f->data + 4) == stream && u32(f->data + 8) == generation);
    assert(u32(f->data + 12) == first && u32(f->data + 28) == skipped);
}

static void test_two_boot_ids(void)
{
    reset();
    ram.generation = 2u;
    ram.rows = 10u;
    poll();
    assert(frame_count == 1u);
    log_stream_init(0xaabbccddu);
    poll();
    assert(frame_count == 2u);
    assert(u32(frames[0].data + 8u) == 2u && u32(frames[1].data + 8u) == 2u);
    assert(u32(frames[0].data + 12u) == 0u && u32(frames[1].data + 12u) == 0u);
    assert(u32(frames[0].data + 32u) == 0x11223344u);
    assert(u32(frames[1].data + 32u) == 0xaabbccddu);
}

static void test_live_batches(void)
{
    reset();
    ram.rows = 25u;
    ram.newest_ms = 990u;
    poll();
    poll();
    poll();
    assert(frame_count == 2u); /* One live batch per poll; 5 rows must wait. */
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, 0u, 0u, 5u, 0u, 0u);
    assert_batch(&frames[1], RM_LOG_ROWS, 10u, 0u, 0u, 5u, 10u, 0u);
    assert(frames[0].qos == 0u && frames[0].token == 0u);
    assert(u32(frames[0].data + 16) == 1000u); /* publish_ms */
    assert(u32(frames[0].data + 20) == 990u);  /* newest_ms */
    assert(u32(frames[0].data + 24) == 25u);   /* log_rows */
    assert(frames[0].data[RM_LOG_BATCH_HEADER_SIZE] == 0u && frames[0].data[RM_LOG_BATCH_HEADER_SIZE + 16 * 9] == 9u);
    assert(frames[1].data[RM_LOG_BATCH_HEADER_SIZE] == 10u);
    now_ms = 1099u;
    poll();
    assert(frame_count == 2u); /* partial batch waits for the period */
    now_ms = 1100u;
    poll();
    assert(frame_count == 3u);
    assert_batch(&frames[2], RM_LOG_ROWS, 5u, 0u, 0u, 5u, 20u, 0u);
    now_ms = 1500u;
    poll();
    assert(frame_count == 3u); /* nothing new */
    log_stream_stats_t stats;
    log_stream_stats(&stats);
    assert(stats.live_batches == 3u && stats.live_rows == 25u);
    assert(stats.live_skipped == 0u && stats.live_jumps == 0u);
}

static void test_live_steady(void)
{
    uint32_t next_index = 0u;
    uint32_t expected = 0u;
    uint32_t worst_age = 0u;
    uint32_t batches = 0u;

    reset();
    for (uint32_t t = 0u; t < 10000u; t += 10u) {
        now_ms = 1000u + t;
        ram.rows = ++next_index; /* 100 rows/s, logged just before the poll */
        ram.newest_ms = now_ms;
        frame_count = 0u;
        poll();
        assert(frame_count <= 1u);
        if (frame_count == 1u) {
            const uint8_t *d = frames[0].data;
            const uint32_t first = u32(d + 12);
            const uint32_t age = u32(d + 16) - u32(d + 20);

            assert(first == expected && d[3] == 0u && u32(d + 28) == 0u);
            expected += d[2];
            ++batches;
            if (age > worst_age) {
                worst_age = age;
            }
            assert(u32(d + 24) >= expected);
        }
    }
    assert(expected + 10u >= next_index && expected <= next_index);
    assert(batches >= 99u && batches <= 101u); /* one batch about every 100 ms */
    assert(worst_age == 0u); /* newest row is the last row of a batch */
}

static void test_live_jump_lag(void)
{
    log_stream_stats_t stats;

    reset();
    ram.rows = LOG_LIVE_LAG_MAX_ROWS; /* exactly the limit: no jump */
    poll();
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, 0u, 0u, 5u, 0u, 0u);
    reset();
    ram.rows = LOG_LIVE_LAG_MAX_ROWS + 1u; /* one more: jump to newest 10 */
    poll();
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, RM_LOG_FLAG_JUMPED, 0u, 5u,
        91u, 91u);
    assert(frames[0].data[RM_LOG_BATCH_HEADER_SIZE] == 91u);
    poll(); /* skipped is reported once */
    assert(frame_count == 1u);
    ram.rows = 120u; /* 19 behind: no jump */
    poll();
    assert_batch(&frames[1], RM_LOG_ROWS, 10u, 0u, 0u, 5u, 101u, 0u);
    reset();
    ram.rows = 500u;
    poll();
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, RM_LOG_FLAG_JUMPED, 0u, 5u,
        490u, 490u);
    log_stream_stats(&stats);
    assert(stats.live_jumps == 1u && stats.live_skipped == 490u);
}

static void test_live_jump_reconnect(void)
{
    reset();
    ram.rows = 50u;
    for (unsigned int i = 0u; i < 5u; ++i) {
        poll();
    }
    assert(frame_count == 5u);
    log_stream_link_down();
    ram.rows = 80u; /* 30 pending, below the lag limit */
    frame_count = 0u;
    poll();
    assert(frame_count == 1u);
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, RM_LOG_FLAG_JUMPED, 0u, 5u,
        70u, 20u);
    log_stream_link_down();
    ram.rows = 85u; /* 5 pending: at most one batch, no skip */
    frame_count = 0u;
    now_ms += 1000u;
    poll();
    assert(frame_count == 1u);
    assert_batch(&frames[0], RM_LOG_ROWS, 5u, 0u, 0u, 5u, 80u, 0u);
}

static void test_live_generation(void)
{
    reset();
    ram.rows = 25u;
    poll();
    poll();
    assert(frame_count == 2u); /* next = 20; rows_seen = 25 */
    ++ram.generation;
    ram.rows = 0u;
    now_ms += 500u;
    poll();
    assert(frame_count == 2u); /* nothing to send in the new run yet */
    ram.rows = 3u;
    ram.newest_ms = now_ms;
    poll();
    assert(frame_count == 3u);
    assert_batch(&frames[2], RM_LOG_ROWS, 3u, RM_LOG_FLAG_NEW_GENERATION,
        0u, 6u, 0u, 5u); /* 5 old rows were never published */
    ram.rows = 4u;
    now_ms += 100u;
    poll();
    assert(frame_count == 4u);
    assert_batch(&frames[3], RM_LOG_ROWS, 1u, 0u, 0u, 6u, 3u, 0u);
    /* A new run with a backlog both restarts and jumps. */
    ++ram.generation;
    ram.rows = 300u;
    now_ms += 100u;
    poll();
    assert_batch(&frames[4], RM_LOG_ROWS, 10u,
        RM_LOG_FLAG_NEW_GENERATION | RM_LOG_FLAG_JUMPED, 0u, 7u, 290u,
        290u);
}

static void test_live_failures(void)
{
    reset();
    ram.rows = 10u;
    publish_fail_count = 1u;
    poll();
    assert(frame_count == 0u);
    poll();
    assert(frame_count == 0u); /* retry delay */
    now_ms += 49u;
    poll();
    assert(frame_count == 0u);
    now_ms += 1u;
    poll();
    assert(frame_count == 1u);
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, 0u, 0u, 5u, 0u, 0u);
    log_stream_stats_t stats;
    log_stream_stats(&stats);
    assert(stats.live_failed == 1u && stats.live_rows == 10u);
    /* Busy board: no frame, state unchanged. */
    ram.rows = 20u;
    ram_busy = true;
    poll();
    assert(frame_count == 1u);
    ram_busy = false;
    row_fail_at = row_reads + 4u; /* 4th row of the next batch fails */
    poll();
    assert(frame_count == 2u);
    assert_batch(&frames[1], RM_LOG_ROWS, 3u, 0u, 0u, 5u, 10u, 0u);
    row_fail_at = 0u;
    now_ms += 100u;
    poll();
    assert_batch(&frames[2], RM_LOG_ROWS, 7u, 0u, 0u, 5u, 13u, 0u);
    /* A generation change on the first row read: nothing is published. */
    ram.rows = 40u;
    row_gen_error = true;
    row_fail_at = row_reads + 1u;
    now_ms += 100u;
    poll();
    assert(frame_count == 3u);
    row_fail_at = 0u;
    /* The generation changes between the row copy and the final check. */
    bump_generation_on_open = false;
    opens = 0u;
    bump_generation_on_open = true;
    now_ms += 100u;
    poll();
    assert(frame_count == 3u);
}

static unsigned int find_kind(unsigned int from, uint8_t kind)
{
    for (unsigned int i = from; i < frame_count; ++i) {
        if (frames[i].qos == 1u && frames[i].data[1] == kind) {
            return i;
        }
    }
    return MAX_FRAMES;
}

static void test_dump_flow(void)
{
    reset();
    dump_mode();
    ram.rows = 30u;
    ram.events = 14u;
    ram.lost = 9u;
    assert(!log_stream_dump_active());
    assert(log_stream_dump_request(77u) == E_OK);
    assert(log_stream_dump_active());
    assert(log_stream_dump_request(78u) == E_OBJ);
    poll();
    unsigned int begin = find_kind(0u, RM_LOG_BEGIN);
    assert(begin < MAX_FRAMES && frames[begin].length == RM_LOG_SIZE);
    assert(u32(frames[begin].data + 4) == 77u);
    assert(u32(frames[begin].data + 16) == 30u);
    assert(u32(frames[begin].data + 20) == 14u);
    assert(u32(frames[begin].data + 24) == 9u);
    ram.rows += 100u; /* appended after open: not in this dump */
    assert(log_stream_dump_request(79u) == E_OBJ);
    frame_count = 0u;
    poll();
    assert(find_kind(0u, RM_LOG_ROWS) == MAX_FRAMES); /* waits for BEGIN ACK */
    log_stream_complete(frames[0].token, true);
    unsigned int data_start = frame_count;
    (void)data_start;
    frame_count = 0u;
    track_unacked = true;
    poll();
    assert(frame_count == 2u); /* two DUMP frames per poll */
    poll();
    assert(frame_count == 3u); /* window of three */
    poll();
    assert(frame_count == 3u);
    assert_batch(&frames[0], RM_LOG_ROWS, 12u, RM_LOG_FLAG_REFILL, 77u, 5u,
        0u, 0u);
    assert_batch(&frames[1], RM_LOG_ROWS, 12u, RM_LOG_FLAG_REFILL, 77u, 5u,
        12u, 0u);
    assert_batch(&frames[2], RM_LOG_ROWS, 6u, RM_LOG_FLAG_REFILL, 77u, 5u,
        24u, 0u);
    assert(frames[0].qos == 1u && frames[0].token != 0u);
    assert(u32(frames[0].data + 24) == 30u && u32(frames[0].data + 28) == 0u);
    assert(u32(frames[0].data + 20) == 0u);
    /* Acknowledge out of order: nothing completes until frame 0 does. */
    log_stream_complete(frames[1].token, true);
    log_stream_complete(frames[2].token, true);
    frame_count = 0u;
    poll();
    assert(frame_count == 0u); /* two still held in the window */
    log_stream_complete(frames[0].token, true); /* stale: frame_count reset */
    (void)0;
}

static void test_dump_complete(void)
{
    reset();
    dump_mode();
    ram.rows = 30u;
    ram.events = 14u;
    track_unacked = true;
    assert(log_stream_dump_request(5u) == E_OK);
    poll();
    ack(0u);
    unsigned int acked = 1u;
    unsigned int rows = 0u;
    unsigned int events = 0u;
    bool done = false;

    for (unsigned int step = 0u; step < 100u && !done; ++step) {
        poll();
        assert(unacked_now <= LOG_DUMP_WINDOW);
        while (acked < frame_count) {
            const frame_t *f = &frames[acked];

            if (f->data[1] == RM_LOG_ROWS) {
                assert(u32(f->data + 12) == rows);
                rows += f->data[2];
            } else if (f->data[1] == RM_LOG_EVENTS) {
                assert(rows == 30u && u32(f->data + 12) == events);
                assert(f->data[RM_LOG_BATCH_HEADER_SIZE] == RM_VERSION && f->data[RM_LOG_BATCH_HEADER_SIZE+1u] == 2u && f->data[RM_LOG_BATCH_HEADER_SIZE+3u] == 3u);
                assert(u32(f->data + RM_LOG_BATCH_HEADER_SIZE + 8) == 1000u + events);
                events += f->data[2];
            } else if (f->data[1] == RM_LOG_END) {
                assert(rows == 30u && events == 14u);
                assert(u32(f->data + 16) == 30u && u32(f->data + 20) == 14u);
                done = true;
            }
            ack(acked);
            --unacked_now;
            ++acked;
        }
    }
    assert(done && rows == 30u && events == 14u);
    assert(unacked_peak == LOG_DUMP_WINDOW);
    assert(!log_stream_dump_active());
    assert(log_stream_dump_request(6u) == E_OK); /* next dump allowed */
}

static void test_dump_failures(void)
{
    reset();
    dump_mode();
    ram.rows = 36u;
    track_unacked = true;
    assert(log_stream_dump_request(8u) == E_OK);
    publish_fail_count = 1u;
    poll(); /* BEGIN publish fails */
    assert(frame_count == 0u);
    poll();
    assert(frame_count == 0u);
    now_ms += 50u;
    poll();
    assert(frame_count == 1u);
    log_stream_complete(frames[0].token, false); /* BEGIN ACK error */
    now_ms += 49u;
    poll();
    assert(frame_count == 1u);
    now_ms += 1u;
    poll();
    assert(frame_count == 2u && frames[1].data[1] == RM_LOG_BEGIN);
    log_stream_complete(frames[1].token, true);
    frame_count = 0u;
    poll();
    poll();
    assert(frame_count == 3u);
    assert(u32(frames[0].data + 12) == 0u && u32(frames[2].data + 12) == 24u);
    ack(0u); /* rows 0..11 acknowledged */
    log_stream_complete(frames[1].token, false); /* rows 12.. failed */
    /* The third frame was forgotten with the failed one; late ACK ignored. */
    log_stream_complete(frames[2].token, true);
    frame_count = 0u;
    now_ms += 50u;
    poll();
    assert(frame_count == 2u);
    assert(u32(frames[0].data + 12) == 12u); /* resent from the ACK cursor */
    assert(u32(frames[1].data + 12) == 24u);
    /* A link loss drops everything unacknowledged without callbacks. */
    log_stream_link_down();
    frame_count = 0u;
    poll();
    assert(frame_count == 2u); /* again from the acknowledged cursor */
    assert(u32(frames[0].data + 12) == 12u && u32(frames[1].data + 12) == 24u);
    log_stream_stats_t stats;
    log_stream_stats(&stats);
    assert(stats.dump_rewinds >= 2u && stats.dump_failed >= 2u);
}

static void test_dump_link_down(void)
{
    reset();
    dump_mode();
    ram.rows = 24u;
    assert(log_stream_dump_request(11u) == E_OK);
    poll();
    ack(0u);
    frame_count = 0u;
    poll();
    assert(frame_count == 2u);
    const uint32_t token0 = frames[0].token;
    log_stream_link_down();
    log_stream_complete(token0, true); /* cannot happen in SDK; must be safe */
    frame_count = 0u;
    poll(); /* BEGIN was acknowledged; the batches restart from the cursor */
    assert(frame_count == 2u);
    assert(u32(frames[0].data + 12) == 0u && u32(frames[1].data + 12) == 12u);
    /* An unacknowledged BEGIN is sent again after a reconnect. */
    reset();
    dump_mode();
    ram.rows = 24u;
    assert(log_stream_dump_request(12u) == E_OK);
    poll();
    assert(frame_count == 1u && frames[0].data[1] == RM_LOG_BEGIN);
    log_stream_link_down();
    frame_count = 0u;
    poll();
    assert(frame_count == 1u && frames[0].data[1] == RM_LOG_BEGIN);
}

static void test_dump_abort(void)
{
    reset();
    dump_mode();
    ram.rows = 24u;
    ram.events = 2u;
    assert(log_stream_dump_request(21u) == E_OK);
    poll();
    ack(0u);
    frame_count = 0u;
    poll();
    assert(frame_count == 2u);
    ack(0u);
    ++ram.generation;
    ram.rows = 1u;
    ram.events = 0u;
    const uint32_t stale = frames[1].token;
    frame_count = 0u;
    poll();
    assert(frame_count == 1u && frames[0].data[1] == RM_LOG_ABORT);
    assert(u32(frames[0].data + 4) == 21u && u32(frames[0].data + 8) == 5u);
    assert(u32(frames[0].data + 16) == 1u);  /* reason: new generation */
    assert(u32(frames[0].data + 20) == 12u); /* rows acknowledged */
    assert(u32(frames[0].data + 24) == 0u);
    log_stream_complete(stale, true); /* stale frame cleared with the abort */
    assert(log_stream_dump_active());
    ack(0u);
    assert(!log_stream_dump_active());
    assert(log_stream_dump_request(22u) == E_OK);
}

typedef struct { uint32_t due_ms; uint32_t token; bool used; } pending_ack_t;

static uint32_t simulate(uint32_t rtt_ms, bool with_live, uint32_t *max_per_poll)
{
    (void)with_live;
    pending_ack_t pending[PENDING_MAX];
    uint32_t end_ms = 0u;
    uint32_t t;
    uint32_t next_row = 0u;
    uint32_t most = 0u;

    reset();
    memset(pending, 0, sizeof(pending));
    ram.rows = TELEMETRY_ROWS;
    ram.events = TELEMETRY_EVENTS;
    assert(log_stream_dump_request(1u) == E_OK);
    track_unacked = true;
    for (t = 0u; (t < 120000u) && (end_ms == 0u); t += 10u) {
        now_ms = 1000u + t;
        if (with_live) {
            /* The live run continues in a new generation: 100 rows/s. */
            next_row = (t / 10u) % 1000u + 1u;
            (void)next_row;
        }
        for (unsigned int i = 0u; i < PENDING_MAX; ++i) {
            if (pending[i].used && ((int32_t)(now_ms - pending[i].due_ms) >= 0)) {
                pending[i].used = false;
                log_stream_complete(pending[i].token, true);
                --unacked_now;
            }
        }
        frame_count = 0u;
        log_stream_service(publish_sink, true);
        if (frame_count > most) {
            most = frame_count;
        }
        for (unsigned int i = 0u; i < frame_count; ++i) {
            if (frames[i].qos == 1u) {
                unsigned int slot = 0u;

                while (pending[slot].used) {
                    ++slot;
                    assert(slot < PENDING_MAX);
                }
                pending[slot] = (pending_ack_t){ now_ms + rtt_ms,
                    frames[i].token, true };
                if (frames[i].data[1] == RM_LOG_END) {
                    end_ms = t + rtt_ms;
                }
            }
        }
    }
    assert(end_ms != 0u);
    *max_per_poll = most;
    return end_ms;
}

static void test_dump_throughput(void)
{
    uint32_t most;
    const uint32_t at_100 = simulate(100u, false, &most);
    const uint32_t at_20 = simulate(20u, false, &most);
    const uint32_t at_200 = simulate(200u, false, &most);

    printf("full log (%u rows, %u events) DUMP: %.1f s at 20 ms RTT, "
        "%.1f s at 100 ms, %.1f s at 200 ms (poll every 10 ms)\n",
        (unsigned)TELEMETRY_ROWS, (unsigned)TELEMETRY_EVENTS,
        at_20 / 1000.0, at_100 / 1000.0, at_200 / 1000.0);
    assert(at_100 <= 15000u && at_20 <= 4000u && at_200 <= 30000u);
    assert(most <= LOG_DUMP_SENDS_PER_POLL);
}

static void test_shared_budget(void)
{
    reset();
    ram.rows = 400u;
    ram.events = 50u;
    assert(log_stream_dump_request(3u) == E_OK);
    poll();
    ack(find_kind(0u, RM_LOG_BEGIN));
    for (unsigned int i = 0u; i < 4u; ++i) {
        frame_count = 0u;
        now_ms += 100u;
        ram.rows += 10u;
        poll();
        assert(frame_count <= LOG_LIVE_SENDS_PER_POLL + LOG_DUMP_SENDS_PER_POLL);
    }
    unsigned int live = 0u;
    for (unsigned int i = 0u; i < frame_count; ++i) {
        live += (frames[i].qos == 0u) ? 1u : 0u;
    }
    assert(live <= LOG_LIVE_SENDS_PER_POLL);
}

/* IDLE: the live stream pauses, a DUMP still completes, and RUN resumes
 * with a jump to the newest rows (the skip is reported). */
static void test_live_paused(void)
{
    unsigned int acked = 0u;
    bool end_seen = false;

    reset();
    ram.rows = 15u;
    poll();
    assert(frame_count == 1u && frames[0].qos == 0u);
    frame_count = 0u;
    ram.rows = 300u;
    for (unsigned int i = 0u; i < 50u; ++i) {
        now_ms += 100u;
        log_stream_service(publish_sink, false);
    }
    assert(frame_count == 0u); /* no live frames while paused */
    assert(log_stream_dump_request(9u) == E_OK);
    for (unsigned int i = 0u; (i < 400u) && !end_seen; ++i) {
        now_ms += 10u;
        log_stream_service(publish_sink, false);
        for (; acked < frame_count; ++acked) {
            assert(frames[acked].qos == 1u); /* only DUMP frames */
            end_seen = end_seen || (frames[acked].data[1] == RM_LOG_END);
            log_stream_complete(frames[acked].token, true);
        }
    }
    assert(end_seen && !log_stream_dump_active());
    frame_count = 0u;
    poll();
    assert(frame_count == 1u);
    assert_batch(&frames[0], RM_LOG_ROWS, 10u, RM_LOG_FLAG_JUMPED, 0u, 5u,
                 290u, 280u);
}

/* M4: isolated owner lifecycle; never reset a running network instance. */
static void test_admission_lifecycle(void)
{
    const uint32_t boot_ids[] = {0x11223344u, 0xa1b2c3d4u};
    for (unsigned int boot = 0u; boot < 2u; ++boot) {
        reset();
        log_stream_init(boot_ids[boot]);
        ram.rows = 25u;
        ram.events = 3u;
        for (uint32_t request = 11u; request <= 13u; request += 2u) {
            assert(!log_stream_dump_active());
            assert(log_stream_dump_request(request) == E_OK);
            assert(log_stream_dump_active());
            assert(log_stream_dump_request(request + 1u) < E_OK);
            unsigned int acked = frame_count;
            bool saw_begin = false, saw_end = false;
            for (unsigned int i = 0u; i < 200u &&
                 log_stream_dump_active(); ++i) {
                now_ms += 10u;
                log_stream_service(publish_sink, false);
                for (; acked < frame_count; ++acked) {
                    const frame_t *frame = &frames[acked];
                    assert(frame->qos == 1u);
                    assert(u32(frame->data + 4u) == request);
                    assert(u32(frame->data + 32u) == boot_ids[boot]);
                    saw_begin = saw_begin || frame->data[1] == RM_LOG_BEGIN;
                    saw_end = saw_end || frame->data[1] == RM_LOG_END;
                    ack(acked);
                }
            }
            assert(saw_begin && saw_end && !log_stream_dump_active());
        }
        /* Both requests completed; no unsolicited request remains. */
        const unsigned int complete_frames = frame_count;
        for (unsigned int i = 0u; i < 5u; ++i) {
            now_ms += 100u;
            log_stream_service(publish_sink, false);
        }
        assert(frame_count == complete_frames && !log_stream_dump_active());
    }
}

int main(int argc, char **argv)
{
    const char *name = (argc == 2) ? argv[1] : "all";
    bool all = (strcmp(name, "all") == 0);

    if (all || !strcmp(name, "admission_lifecycle")) test_admission_lifecycle();
    if (all || !strcmp(name, "two_boot_ids")) test_two_boot_ids();
    if (all || !strcmp(name, "live_batches")) test_live_batches();
    if (all || !strcmp(name, "live_steady")) test_live_steady();
    if (all || !strcmp(name, "live_jump_lag")) test_live_jump_lag();
    if (all || !strcmp(name, "live_jump_reconnect")) test_live_jump_reconnect();
    if (all || !strcmp(name, "live_generation")) test_live_generation();
    if (all || !strcmp(name, "live_failures")) test_live_failures();
    if (all || !strcmp(name, "dump_flow")) test_dump_flow();
    if (all || !strcmp(name, "dump_complete")) test_dump_complete();
    if (all || !strcmp(name, "dump_failures")) test_dump_failures();
    if (all || !strcmp(name, "dump_link_down")) test_dump_link_down();
    if (all || !strcmp(name, "dump_abort")) test_dump_abort();
    if (all || !strcmp(name, "dump_throughput")) test_dump_throughput();
    if (all || !strcmp(name, "shared_budget")) test_shared_budget();
    if (all || !strcmp(name, "live_paused")) test_live_paused();
    return 0;
}
