#include "mqtt_wire.h"
#include <string.h>

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 255u);
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 255u);
    p[1] = (uint8_t)((v >> 8) & 255u);
    p[2] = (uint8_t)((v >> 16) & 255u);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

void rm_pack_state(uint8_t out[RM_STATE_SIZE], const rm_state_t *v)
{
    memset(out, 0, RM_STATE_SIZE);
    out[0] = RM_VERSION;
    out[1] = v->flags;
    out[2] = (uint8_t)((v->state & 15u) | ((v->module & 15u) << 4));
    out[3] = v->motion_faults;
    put32(out + 4, v->uptime_ms);
    put32(out + 8, v->sequence);
    put16(out + 12, v->line_age_ms);
    put16(out + 14, v->motion_age_ms);
    put16(out + 16, v->vehicle_age_ms);
    put16(out + 18, (uint16_t)(v->raw_bc & 4095u));
    memcpy(out + 20, v->kr1, 16u);
    put32(out + 36, (uint32_t)v->count_l);
    put32(out + 40, (uint32_t)v->count_r);
    put32(out + 44, ((uint32_t)v->norm_l & 1023u) |
                        (((uint32_t)v->norm_c & 1023u) << 10));
}

void rm_pack_sonar(uint8_t out[RM_SONAR_SIZE], const rm_sonar_t *v)
{
    memset(out, 0, RM_SONAR_SIZE);
    out[0] = RM_VERSION;
    out[1] = (uint8_t)((v->status & 3u) | (v->valid ? 4u : 0u) |
                       (v->pending ? 8u : 0u));
    out[2] = (uint8_t)v->angle;
    out[3] = (uint8_t)v->ping_angle;
    put32(out + 4, v->history_sequence);
    put32(out + 8, v->generation);
    put32(out + 12, v->request_id);
    put32(out + 16, v->trigger_ms);
    put32(out + 20, v->timestamp_ms);
    put16(out + 24, v->echo);
    put32(out + 28, v->ping_sequence);
}

void rm_pack_event(uint8_t out[RM_EVENT_SIZE], const rm_event_t *v)
{
    memset(out, 0, RM_EVENT_SIZE);
    out[0] = RM_VERSION;
    out[1] = v->kind;
    out[2] = v->from;
    out[3] = v->to;
    put32(out + 4, v->sequence);
    put32(out + 8, v->uptime_ms);
    put16(out + 12, v->word);
    out[14] = v->detail;
    out[15] = (uint8_t)(v->flags & 1u);
}

void rm_pack_heartbeat(uint8_t out[RM_HEARTBEAT_SIZE], const rm_heartbeat_t *v)
{
    memset(out, 0, RM_HEARTBEAT_SIZE);
    out[0] = RM_VERSION;
    out[1] = (uint8_t)(v->flags & 63u);
    out[2] = v->reset_reason;
    put32(out + 4, v->uptime_ms);
    put32(out + 8, v->connect_attempts);
    put32(out + 12, v->telemetry_lost);
    put32(out + 16, v->event_lost);
    put32(out + 20, v->command_lost);
    put32(out + 24, v->sonar_history_missed);
    put32(out + 28, v->vehicle_overruns);
    put32(out + 32, v->motion_late);
    put32(out + 36, v->motion_missed);
    put16(out + 40, v->stack_min_free);
    put32(out + 44, v->mqtt_hook_max_us);
    put32(out + 48, v->publish_ok);
    put32(out + 52, v->publish_failed);
    put32(out + 56, v->line_busy);
    put32(out + 60, v->sonar_busy);
    put32(out + 64, v->motion_busy);
    put32(out + 68, v->vehicle_busy);
    put32(out + 72, v->health_busy);
    put32(out + 76, v->latest_sonar_history_sequence);
    put16(out + 80, v->health_age_ms);
    out[82] = (uint8_t)v->wifi_link_status;
    out[83] = v->mqtt_state;
    put32(out + 84, v->wifi_poll_gap_max_us);
    put32(out + 88, v->wifi_poll_gap_samples);
    put32(out + 92, (uint32_t)v->wifi_start_status);
    out[96] = v->wifi_requested_priority;
    out[97] = v->wifi_observed_priority;
}

void rm_pack_command(uint8_t out[RM_COMMAND_SIZE], const rm_command_t *v)
{
    out[0] = RM_VERSION;
    out[1] = v->opcode;
    put32(out + 2, v->request_id);
    memcpy(out + 6, v->token, RM_TOKEN_SIZE);
}

void rm_pack_log(uint8_t out[RM_LOG_SIZE], const rm_log_t *v)
{
    memset(out, 0, RM_LOG_SIZE);
    out[0] = RM_VERSION;
    out[1] = v->kind;
    put32(out + 4, v->dump_id);
    put32(out + 8, v->generation);
    put32(out + 12, v->index);
    memcpy(out + 16, v->body, 16u);
}

/*
 * Pack a batched log frame. Layout (little-endian): 0 version, 1 kind,
 * 2 count, 3 flags, 4 stream id, 8 generation, 12 first index,
 * 16 publish ms, 20 newest-row ms, 24 log rows, 28 skipped rows,
 * then count 16-byte records.
 */
size_t rm_pack_log_batch(uint8_t *out, size_t capacity,
                         const rm_log_batch_t *header, const uint8_t *records)
{
    size_t length = 0u;
    bool valid = (out != NULL) && (header != NULL) && (records != NULL);

    if (valid) {
        valid = ((header->kind == RM_LOG_ROWS) ||
                 (header->kind == RM_LOG_EVENTS)) &&
                (header->count != 0u) &&
                (header->count <= RM_LOG_BATCH_MAX_COUNT);
    }
    if (valid) {
        length = RM_LOG_BATCH_HEADER_SIZE +
                 ((size_t)header->count * RM_LOG_RECORD_SIZE);
        valid = (capacity >= length);
    }
    if (!valid) {
        return 0u;
    }
    (void)memset(out, 0, RM_LOG_BATCH_HEADER_SIZE);
    out[0] = RM_VERSION;
    out[1] = header->kind;
    out[2] = header->count;
    out[3] = header->flags;
    put32(out + 4, header->stream_id);
    put32(out + 8, header->generation);
    put32(out + 12, header->first_index);
    put32(out + 16, header->publish_ms);
    put32(out + 20, header->newest_ms);
    put32(out + 24, header->log_rows);
    put32(out + 28, header->skipped_before);
    (void)memcpy(out + RM_LOG_BATCH_HEADER_SIZE, records,
                 (size_t)header->count * RM_LOG_RECORD_SIZE);
    return length;
}

bool rm_command_allowed(uint8_t opcode, bool idle)
{
    return opcode == RM_CMD_ESTOP || opcode == RM_CMD_DUMP ||
           (idle && (opcode == RM_CMD_CALIBRATE || opcode == RM_CMD_START));
}

bool rm_command_decode(const uint8_t *data, size_t len,
                       const uint8_t expected_token[RM_TOKEN_SIZE],
                       bool enabled, rm_command_t *out)
{
    if (!data || !expected_token || !out || len != RM_COMMAND_SIZE ||
        data[0] != RM_VERSION || !rm_command_allowed(data[1], true)) {
        return false;
    }
    if (!enabled && data[1] != RM_CMD_ESTOP) {
        return false;
    }
    if (enabled) {
        uint32_t difference = 0u;
        for (size_t i = 0u; i < RM_TOKEN_SIZE; ++i) {
            difference |= (uint32_t)(data[6u + i] ^ expected_token[i]);
        }
        if (difference != 0u) {
            return false;
        }
    }
    rm_command_t value;
    value.opcode = data[1];
    value.request_id = get32(data + 2);
    memcpy(value.token, data + 6, RM_TOKEN_SIZE);
    *out = value;
    return true;
}
