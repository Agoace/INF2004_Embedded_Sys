#include "buddy1_service.h"
#include <string.h>
static void clear_pending(b1_service_t *s)
{
    b1_assembler_abort(&s->assembler);
    s->receiving = false;
    s->command_head = s->command_count = 0;
    s->reply_head = s->reply_count = 0;
}

void b1_service_init(b1_service_t *s, uint32_t now)
{
    memset(s, 0, sizeof(*s));
    s->now_ms = now;
    s->since_ms = now;
    s->phase = B1_BACKOFF;
}

void b1_service_phase(b1_service_t *s, b1_phase_t phase, uint32_t now)
{
    s->phase = phase;
    s->now_ms = now;
    s->since_ms = now;
    if (phase == B1_READY) {
        s->failures = 0;
        s->telemetry_ms = now - 110u;
        s->heartbeat_ms = now - 1000u;
    }
}

void b1_service_session(b1_service_t *s, uint32_t session, uint32_t now)
{
    clear_pending(s);
    b1_sequence_init(&s->sequence, session);
    b1_service_phase(s, B1_MQTT, now);
}

void b1_service_disconnect(b1_service_t *s, uint32_t now)
{
    clear_pending(s);
    s->retry_ms = s->failures < 3 ? (1000u << s->failures) : 5000u;
    if (s->failures < 3) { ++s->failures; }
    b1_service_phase(s, B1_BACKOFF, now);
}

bool b1_service_timeout(const b1_service_t *s, uint32_t now)
{
    return s->phase != B1_BACKOFF && s->phase != B1_READY &&
           (uint32_t)(now - s->since_ms) >= B1_CONNECT_TIMEOUT_MS;
}

bool b1_service_retry_due(const b1_service_t *s, uint32_t now)
{
    return s->phase == B1_BACKOFF && (uint32_t)(now - s->since_ms) >= s->retry_ms;
}

static bool due(b1_service_t *s, uint32_t now, uint32_t *last, uint32_t period)
{
    const uint32_t elapsed = now - *last;
    if (s->phase != B1_READY || elapsed < period) { return false; }
    *last += (elapsed / period) * period;
    return true;
}

bool b1_service_telemetry_due(b1_service_t *s, uint32_t now)
{ return due(s, now, &s->telemetry_ms, 110); }
bool b1_service_heartbeat_due(b1_service_t *s, uint32_t now)
{ return due(s, now, &s->heartbeat_ms, 1000); }

static void result(b1_service_t *s, b1_outcome_t outcome, const b1_command_t *command)
{
    if (s->reply_count == B1_PENDING_RESULTS) { ++s->dropped_results; return; }
    b1_reply_t *reply = &s->replies[((unsigned int)s->reply_head + s->reply_count) % B1_PENDING_RESULTS];
    reply->data[0] = (uint8_t)(0x40u | ((unsigned int)outcome << 3));
    reply->length = 1;
    if (command != NULL && b1_command_encode(command, reply->data + 1, 7) == B1_OK) {
        reply->length = 8;
    }
    ++s->reply_count;
}

void b1_service_update_time(b1_service_t *s, uint32_t now)
{
    s->now_ms = now;
    if (s->receiving && (uint32_t)(now - s->rx_started_ms) >= B1_COMMAND_MAX_AGE_MS) {
        s->receiving = false;
        b1_assembler_abort(&s->assembler);
        ++s->malformed;
        result(s, B1_MALFORMED, NULL);
    }
}

void b1_service_rx_begin(b1_service_t *s, size_t total)
{
    s->rx_started_ms = s->now_ms;
    s->receiving = b1_assembler_begin(&s->assembler, total) == B1_OK;
    if (!s->receiving) { ++s->malformed; result(s, B1_MALFORMED, NULL); }
}

void b1_service_rx_data(b1_service_t *s, const uint8_t *data, size_t length, bool last)
{
    if (!s->receiving) { return; }
    b1_command_t command;
    b1_result_t decoded = b1_assembler_feed(&s->assembler, data, length, last, &command);
    if (decoded == B1_MORE) { return; }
    s->receiving = false;
    if (decoded != B1_OK) { ++s->malformed; result(s, B1_MALFORMED, NULL); return; }
    if (s->phase != B1_READY) { result(s, B1_NOT_READY, &command); return; }
    if (s->command_count == B1_PENDING_COMMANDS) {
        ++s->busy;
        result(s, B1_BUSY, &command);
        return;
    }
    const unsigned int tail = ((unsigned int)s->command_head + s->command_count) % B1_PENDING_COMMANDS;
    s->commands[tail] = command;
    s->command_received_ms[tail] = s->now_ms;
    ++s->command_count;
}

void b1_service_process(b1_service_t *s)
{
    if (s->phase != B1_READY || s->command_count == 0 || s->reply_count == B1_PENDING_RESULTS) { return; }
    const b1_command_t *command = &s->commands[s->command_head];
    const b1_sequence_result_t checked = b1_sequence_check(&s->sequence, command);
    b1_outcome_t outcome;
    if ((uint32_t)(s->now_ms - s->command_received_ms[s->command_head]) >= B1_COMMAND_MAX_AGE_MS) {
        outcome = B1_BUSY; /* Retryable, never consumes the sequence or performs an action. */
    } else switch (checked) {
        case B1_SEQUENCE_NEW:
            (void)b1_sequence_record(&s->sequence, command);
            ++s->recorded;
            outcome = B1_BENCH_RECORDED;
            break;
        case B1_SEQUENCE_DUPLICATE: outcome = B1_DUPLICATE; break;
        case B1_SEQUENCE_CONFLICT: outcome = B1_CONFLICT; break;
        case B1_SEQUENCE_OLD: outcome = B1_OLD; break;
        case B1_SEQUENCE_WRONG_SESSION: outcome = B1_WRONG_SESSION; break;
        default: outcome = B1_MALFORMED; break;
    }
    result(s, outcome, command);
    s->command_head = (uint8_t)((s->command_head + 1u) % B1_PENDING_COMMANDS);
    --s->command_count;
}

const b1_reply_t *b1_service_reply(const b1_service_t *s)
{ return s->reply_count != 0 ? &s->replies[s->reply_head] : NULL; }

void b1_service_reply_sent(b1_service_t *s)
{
    if (s->reply_count != 0) {
        s->reply_head = (uint8_t)((s->reply_head + 1u) % B1_PENDING_RESULTS);
        --s->reply_count;
    }
}

void b1_service_status(const b1_service_t *s, uint8_t output[5])
{
    const bool online = s->phase == B1_READY || s->phase == B1_SUBSCRIBING;
    output[0] = (uint8_t)(0x40u | (online ? 0x20u : 0u) |
                          (s->phase == B1_READY ? 0x10u : 0u));
    output[1] = (uint8_t)(s->sequence.session >> 24);
    output[2] = (uint8_t)(s->sequence.session >> 16);
    output[3] = (uint8_t)(s->sequence.session >> 8);
    output[4] = (uint8_t)s->sequence.session;
}
