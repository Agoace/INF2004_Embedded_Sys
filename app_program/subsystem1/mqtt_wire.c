/*
 * mqtt_wire.c - application telemetry and service interface.
 */
#include "mqtt_wire.h"
#include <string.h>

#define WIRE_BYTE_MASK            255u
#define WIRE_BYTE_BITS            8
#define WIRE_HALF_BITS            16
#define WIRE_HIGH_BYTE_BITS       24
#define WIRE_NIBBLE_MASK          15u
#define WIRE_NIBBLE_BITS          4
#define WIRE_ADC_MASK             4095u
#define WIRE_NORM_MASK            1023u
#define WIRE_NORM_BITS            10
#define WIRE_KR1_BYTES            16u
#define WIRE_SUMMARY_FLAGS_MASK   7u
#define WIRE_HEARTBEAT_FLAGS_MASK 63u
#define WIRE_SONAR_STATUS_MASK    3u
#define WIRE_SONAR_VALID_FLAG     4u
#define WIRE_SONAR_PENDING_FLAG   8u

enum {
    COMMAND_VERSION_OFFSET = 0,
    COMMAND_OPCODE_OFFSET = 1,
    COMMAND_REQUEST_OFFSET = 2,
    COMMAND_TOKEN_OFFSET = 6
};

/*
 * Store a 16-bit value in little-endian byte order.
 */
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & WIRE_BYTE_MASK);
    p[1] = (uint8_t)(v >> WIRE_BYTE_BITS);
}

/*
 * Store a 32-bit value in little-endian byte order.
 */
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & WIRE_BYTE_MASK);
    p[1] = (uint8_t)((v >> WIRE_BYTE_BITS) & WIRE_BYTE_MASK);
    p[2] = (uint8_t)((v >> WIRE_HALF_BITS) & WIRE_BYTE_MASK);
    p[3] = (uint8_t)(v >> WIRE_HIGH_BYTE_BITS);
}

/*
 * Read a 32-bit value in little-endian byte order.
 */
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << WIRE_BYTE_BITS) |
           ((uint32_t)p[2] << WIRE_HALF_BITS) |
           ((uint32_t)p[3] << WIRE_HIGH_BYTE_BITS);
}

/*
 * Compute the signed mean without overflowing the 32-bit inputs.
 */
int32_t rm_average_mm(int32_t left_mm, int32_t right_mm)
{
    /* Floor of the mean without overflow (GCC shifts negative values
     * arithmetically). Then move a negative, inexact result up by one so the
     * mean truncates toward zero. */
    int32_t mean = (left_mm >> 1) + (right_mm >> 1) + (left_mm & right_mm & 1);

    if ((mean < 0) && (((left_mm ^ right_mm) & 1) != 0)) {
        ++mean;
    }
    return mean;
}

/*
 * Pack state, ages and encoder measurements into the state wire frame.
 */
void rm_pack_state(uint8_t out[RM_STATE_SIZE], const rm_state_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        FLAGS_OFFSET = 1,
        STATE_OFFSET = 2,
        MOTION_FAULTS_OFFSET = 3,
        UPTIME_MS_OFFSET = 4,
        SEQUENCE_OFFSET = 8,
        LINE_AGE_MS_OFFSET = 12,
        MOTION_AGE_MS_OFFSET = 14,
        VEHICLE_AGE_MS_OFFSET = 16,
        RAW_BC_OFFSET = 18,
        KR1_OFFSET = 20,
        COUNT_L_OFFSET = 36,
        COUNT_R_OFFSET = 40,
        NORM_L_OFFSET = 44,
        DIST_LEFT_MM_OFFSET = 48,
        DIST_RIGHT_MM_OFFSET = 52,
        DIST_AVG_MM_OFFSET = 56,
        DUTY_LEFT_PERMILLE_OFFSET = 60,
        DUTY_RIGHT_PERMILLE_OFFSET = 62,
        HEADING_CDEG_OFFSET = 64,
        TURN_RATE_DDPS_OFFSET = 68,
    };

    memset(out, 0, RM_STATE_SIZE);
    out[VERSION_OFFSET] = RM_STATE_VERSION;
    out[FLAGS_OFFSET] = v->flags;
    out[STATE_OFFSET] =
        (uint8_t)((v->state & WIRE_NIBBLE_MASK) |
                  ((v->module & WIRE_NIBBLE_MASK) << WIRE_NIBBLE_BITS));
    out[MOTION_FAULTS_OFFSET] = v->motion_faults;
    put32(out + UPTIME_MS_OFFSET, v->uptime_ms);
    put32(out + SEQUENCE_OFFSET, v->sequence);
    put16(out + LINE_AGE_MS_OFFSET, v->line_age_ms);
    put16(out + MOTION_AGE_MS_OFFSET, v->motion_age_ms);
    put16(out + VEHICLE_AGE_MS_OFFSET, v->vehicle_age_ms);
    put16(out + RAW_BC_OFFSET, (uint16_t)(v->raw_bc & WIRE_ADC_MASK));
    memcpy(out + KR1_OFFSET, v->kr1, WIRE_KR1_BYTES);
    put32(out + COUNT_L_OFFSET, (uint32_t)v->count_l);
    put32(out + COUNT_R_OFFSET, (uint32_t)v->count_r);
    put32(out + NORM_L_OFFSET,
          ((uint32_t)v->norm_l & WIRE_NORM_MASK) |
              (((uint32_t)v->norm_c & WIRE_NORM_MASK) << WIRE_NORM_BITS));
    put32(out + DIST_LEFT_MM_OFFSET, (uint32_t)v->dist_left_mm);
    put32(out + DIST_RIGHT_MM_OFFSET, (uint32_t)v->dist_right_mm);
    put32(out + DIST_AVG_MM_OFFSET, (uint32_t)v->dist_avg_mm);
    put16(out + DUTY_LEFT_PERMILLE_OFFSET, (uint16_t)v->duty_left_permille);
    put16(out + DUTY_RIGHT_PERMILLE_OFFSET, (uint16_t)v->duty_right_permille);
    put32(out + HEADING_CDEG_OFFSET, (uint32_t)v->heading_cdeg);
    put16(out + TURN_RATE_DDPS_OFFSET, (uint16_t)v->turn_rate_ddps);
}

/*
 * Pack a completed scan without relying on C structure layout.
 */
void rm_pack_obstacle(uint8_t out[RM_OBSTACLE_SIZE], const rm_obstacle_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        FLAGS_OFFSET = 1,
        DECISION_OFFSET = 2,
        CLOSEST_ANGLE_DEG_OFFSET = 3,
        SCAN_ID_OFFSET = 4,
        UPTIME_MS_OFFSET = 8,
        CLOSEST_MM_OFFSET = 12,
        PROFILE_WORD_OFFSET = 14,
        RIGHT_FREE_STEP_OFFSET = 16,
        LEFT_FREE_STEP_OFFSET = 17,
        RIGHT_CLEAR_DEG_OFFSET = 18,
        LEFT_CLEAR_DEG_OFFSET = 19,
    };

    memset(out, 0, RM_OBSTACLE_SIZE);
    out[VERSION_OFFSET] = RM_VERSION;
    out[FLAGS_OFFSET] = v->flags;
    out[DECISION_OFFSET] = v->decision;
    out[CLOSEST_ANGLE_DEG_OFFSET] = (uint8_t)v->closest_angle_deg;
    put32(out + SCAN_ID_OFFSET, v->scan_id);
    put32(out + UPTIME_MS_OFFSET, v->uptime_ms);
    put16(out + CLOSEST_MM_OFFSET, v->closest_mm);
    put16(out + PROFILE_WORD_OFFSET, v->profile_word);
    out[RIGHT_FREE_STEP_OFFSET] = v->right_free_step;
    out[LEFT_FREE_STEP_OFFSET] = v->left_free_step;
    out[RIGHT_CLEAR_DEG_OFFSET] = v->right_clear_deg;
    out[LEFT_CLEAR_DEG_OFFSET] = v->left_clear_deg;
}

/*
 * Pack one finished detour and its encoder-derived measurements.
 */
void rm_pack_detour(uint8_t out[RM_DETOUR_SIZE], const rm_detour_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        OUTCOME_OFFSET = 1,
        SIDE_OFFSET = 2,
        FLAGS_OFFSET = 3,
        SEQUENCE_OFFSET = 4,
        START_MS_OFFSET = 8,
        END_MS_OFFSET = 12,
        DURATION_MS_OFFSET = 16,
        DISTANCE_MM_OFFSET = 20,
        MAX_LATERAL_MM_OFFSET = 24,
        PAYLOAD_OFFSET = 26,
        SCAN_ID_OFFSET = 28,
    };

    memset(out, 0, RM_DETOUR_SIZE);
    out[VERSION_OFFSET] = RM_VERSION;
    out[OUTCOME_OFFSET] = v->outcome;
    out[SIDE_OFFSET] = v->side;
    out[FLAGS_OFFSET] = (uint8_t)(v->flags & RM_DETOUR_LATERAL_VALID);
    put32(out + SEQUENCE_OFFSET, v->sequence);
    put32(out + START_MS_OFFSET, v->start_ms);
    put32(out + END_MS_OFFSET, v->end_ms);
    put32(out + DURATION_MS_OFFSET, v->duration_ms);
    put32(out + DISTANCE_MM_OFFSET, v->distance_mm);
    put16(out + MAX_LATERAL_MM_OFFSET, v->max_lateral_mm);
    put16(out + PAYLOAD_OFFSET, v->payload);
    put32(out + SCAN_ID_OFFSET, v->scan_id);
}

/*
 * Pack end-of-run totals and preserve the reserved-field contract.
 */
void rm_pack_summary(uint8_t out[RM_SUMMARY_SIZE], const rm_summary_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        FLAGS_OFFSET = 1,
        STOP_SOURCE_OFFSET = 2,
        STOP_REASON_OFFSET = 3,
        RUN_ID_OFFSET = 4,
        START_MS_OFFSET = 8,
        DURATION_MS_OFFSET = 12,
        DISTANCE_MM_OFFSET = 16,
        LINE_LOSSES_OFFSET = 20,
        OBSTACLES_OFFSET = 22,
        DETOURS_DONE_OFFSET = 24,
        DETOURS_SUCCEEDED_OFFSET = 26,
        DETOUR_TOTAL_MS_OFFSET = 28,
        DETOURS_STARTED_OFFSET = 32,
        MAX_SPEED_ERROR_MM_S_OFFSET = 34,
        MOTION_FAULT_BITS_OFFSET = 36,
        SHELL_FAULT_OFFSET = 37,
        SHELL_FAULT_DETAIL_OFFSET = 38,
        BARCODE_OK_OFFSET = 40,
        BARCODE_FAILED_OFFSET = 41,
        HUMP_PEAK_OFFSET = 42,
    };

    memset(out, 0, RM_SUMMARY_SIZE);
    out[VERSION_OFFSET] = RM_VERSION;
    out[FLAGS_OFFSET] = (uint8_t)(v->flags & WIRE_SUMMARY_FLAGS_MASK);
    out[STOP_SOURCE_OFFSET] = v->stop_source;
    out[STOP_REASON_OFFSET] = v->stop_reason;
    put32(out + RUN_ID_OFFSET, v->run_id);
    put32(out + START_MS_OFFSET, v->start_ms);
    put32(out + DURATION_MS_OFFSET, v->duration_ms);
    put32(out + DISTANCE_MM_OFFSET, v->distance_mm);
    put16(out + LINE_LOSSES_OFFSET, v->line_losses);
    put16(out + OBSTACLES_OFFSET, v->obstacles);
    put16(out + DETOURS_DONE_OFFSET, v->detours_done);
    put16(out + DETOURS_SUCCEEDED_OFFSET, v->detours_succeeded);
    put32(out + DETOUR_TOTAL_MS_OFFSET, v->detour_total_ms);
    put16(out + DETOURS_STARTED_OFFSET, v->detours_started);
    put16(out + MAX_SPEED_ERROR_MM_S_OFFSET, v->max_speed_error_mm_s);
    out[MOTION_FAULT_BITS_OFFSET] = v->motion_fault_bits;
    out[SHELL_FAULT_OFFSET] = v->shell_fault;
    put16(out + SHELL_FAULT_DETAIL_OFFSET, v->shell_fault_detail);
    out[BARCODE_OK_OFFSET] = v->barcode_ok;
    out[BARCODE_FAILED_OFFSET] = v->barcode_failed;
    put16(out + HUMP_PEAK_OFFSET, v->hump_peak);
}

/*
 * Pack the online or offline state for the retained status topic.
 */
void rm_pack_status(uint8_t out[RM_STATUS_SIZE], uint8_t state)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        BODY_OFFSET = 1,
    };

    out[VERSION_OFFSET] = RM_VERSION;
    out[BODY_OFFSET] = state;
}

/*
 * Pack a sonar history result and its trigger/result timestamps.
 */
void rm_pack_sonar(uint8_t out[RM_SONAR_SIZE], const rm_sonar_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        STATUS_OFFSET = 1,
        ANGLE_OFFSET = 2,
        PING_ANGLE_OFFSET = 3,
        HISTORY_SEQUENCE_OFFSET = 4,
        GENERATION_OFFSET = 8,
        REQUEST_ID_OFFSET = 12,
        TRIGGER_MS_OFFSET = 16,
        TIMESTAMP_MS_OFFSET = 20,
        ECHO_OFFSET = 24,
        PING_SEQUENCE_OFFSET = 28,
    };

    memset(out, 0, RM_SONAR_SIZE);
    out[VERSION_OFFSET] = RM_VERSION;
    out[STATUS_OFFSET] = (uint8_t)((v->status & WIRE_SONAR_STATUS_MASK) |
                                   (v->valid ? WIRE_SONAR_VALID_FLAG : 0u) |
                                   (v->pending ? WIRE_SONAR_PENDING_FLAG : 0u));
    out[ANGLE_OFFSET] = (uint8_t)v->angle;
    out[PING_ANGLE_OFFSET] = (uint8_t)v->ping_angle;
    put32(out + HISTORY_SEQUENCE_OFFSET, v->history_sequence);
    put32(out + GENERATION_OFFSET, v->generation);
    put32(out + REQUEST_ID_OFFSET, v->request_id);
    put32(out + TRIGGER_MS_OFFSET, v->trigger_ms);
    put32(out + TIMESTAMP_MS_OFFSET, v->timestamp_ms);
    put16(out + ECHO_OFFSET, v->echo);
    put32(out + PING_SEQUENCE_OFFSET, v->ping_sequence);
}

/*
 * Pack a vehicle event, keeping only the defined flag bits.
 */
void rm_pack_event(uint8_t out[RM_EVENT_SIZE], const rm_event_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        KIND_OFFSET = 1,
        FROM_OFFSET = 2,
        TO_OFFSET = 3,
        SEQUENCE_OFFSET = 4,
        UPTIME_MS_OFFSET = 8,
        WORD_OFFSET = 12,
        DETAIL_OFFSET = 14,
        FLAGS_OFFSET = 15,
    };

    memset(out, 0, RM_EVENT_SIZE);
    out[VERSION_OFFSET] = RM_VERSION;
    out[KIND_OFFSET] = v->kind;
    out[FROM_OFFSET] = v->from;
    out[TO_OFFSET] = v->to;
    put32(out + SEQUENCE_OFFSET, v->sequence);
    put32(out + UPTIME_MS_OFFSET, v->uptime_ms);
    put16(out + WORD_OFFSET, v->word);
    out[DETAIL_OFFSET] = v->detail;
    out[FLAGS_OFFSET] = (uint8_t)(v->flags & 1u);
}

/*
 * Pack health counters, task timing and the CPU-load observation.
 */
void rm_pack_heartbeat(uint8_t out[RM_HEARTBEAT_SIZE], const rm_heartbeat_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        FLAGS_OFFSET = 1,
        RESET_REASON_OFFSET = 2,
        UPTIME_MS_OFFSET = 4,
        CONNECT_ATTEMPTS_OFFSET = 8,
        TELEMETRY_LOST_OFFSET = 12,
        EVENT_LOST_OFFSET = 16,
        COMMAND_LOST_OFFSET = 20,
        SONAR_HISTORY_MISSED_OFFSET = 24,
        VEHICLE_OVERRUNS_OFFSET = 28,
        MOTION_LATE_OFFSET = 32,
        MOTION_MISSED_OFFSET = 36,
        STACK_MIN_FREE_OFFSET = 40,
        MQTT_HOOK_MAX_US_OFFSET = 44,
        PUBLISH_OK_OFFSET = 48,
        PUBLISH_FAILED_OFFSET = 52,
        LINE_BUSY_OFFSET = 56,
        SONAR_BUSY_OFFSET = 60,
        MOTION_BUSY_OFFSET = 64,
        VEHICLE_BUSY_OFFSET = 68,
        HEALTH_BUSY_OFFSET = 72,
        LATEST_SONAR_HISTORY_SEQUENCE_OFFSET = 76,
        HEALTH_AGE_MS_OFFSET = 80,
        WIFI_LINK_STATUS_OFFSET = 82,
        MQTT_STATE_OFFSET = 83,
        WIFI_POLL_GAP_MAX_US_OFFSET = 84,
        WIFI_POLL_GAP_SAMPLES_OFFSET = 88,
        WIFI_START_STATUS_OFFSET = 92,
        WIFI_REQUESTED_PRIORITY_OFFSET = 96,
        WIFI_OBSERVED_PRIORITY_OFFSET = 97,
        CPU_LOAD_PERMILLE_OFFSET = 98,
    };

    memset(out, 0, RM_HEARTBEAT_SIZE);
    out[VERSION_OFFSET] = RM_HEARTBEAT_VERSION;
    out[FLAGS_OFFSET] = (uint8_t)(v->flags & WIRE_HEARTBEAT_FLAGS_MASK);
    out[RESET_REASON_OFFSET] = v->reset_reason;
    put32(out + UPTIME_MS_OFFSET, v->uptime_ms);
    put32(out + CONNECT_ATTEMPTS_OFFSET, v->connect_attempts);
    put32(out + TELEMETRY_LOST_OFFSET, v->telemetry_lost);
    put32(out + EVENT_LOST_OFFSET, v->event_lost);
    put32(out + COMMAND_LOST_OFFSET, v->command_lost);
    put32(out + SONAR_HISTORY_MISSED_OFFSET, v->sonar_history_missed);
    put32(out + VEHICLE_OVERRUNS_OFFSET, v->vehicle_overruns);
    put32(out + MOTION_LATE_OFFSET, v->motion_late);
    put32(out + MOTION_MISSED_OFFSET, v->motion_missed);
    put16(out + STACK_MIN_FREE_OFFSET, v->stack_min_free);
    put32(out + MQTT_HOOK_MAX_US_OFFSET, v->mqtt_hook_max_us);
    put32(out + PUBLISH_OK_OFFSET, v->publish_ok);
    put32(out + PUBLISH_FAILED_OFFSET, v->publish_failed);
    put32(out + LINE_BUSY_OFFSET, v->line_busy);
    put32(out + SONAR_BUSY_OFFSET, v->sonar_busy);
    put32(out + MOTION_BUSY_OFFSET, v->motion_busy);
    put32(out + VEHICLE_BUSY_OFFSET, v->vehicle_busy);
    put32(out + HEALTH_BUSY_OFFSET, v->health_busy);
    put32(out + LATEST_SONAR_HISTORY_SEQUENCE_OFFSET,
          v->latest_sonar_history_sequence);
    put16(out + HEALTH_AGE_MS_OFFSET, v->health_age_ms);
    out[WIFI_LINK_STATUS_OFFSET] = (uint8_t)v->wifi_link_status;
    out[MQTT_STATE_OFFSET] = v->mqtt_state;
    put32(out + WIFI_POLL_GAP_MAX_US_OFFSET, v->wifi_poll_gap_max_us);
    put32(out + WIFI_POLL_GAP_SAMPLES_OFFSET, v->wifi_poll_gap_samples);
    put32(out + WIFI_START_STATUS_OFFSET, (uint32_t)v->wifi_start_status);
    out[WIFI_REQUESTED_PRIORITY_OFFSET] = v->wifi_requested_priority;
    out[WIFI_OBSERVED_PRIORITY_OFFSET] = v->wifi_observed_priority;
    put16(out + CPU_LOAD_PERMILLE_OFFSET, v->cpu_load_permille);
}

/*
 * Pack a command with its request ID and fixed-length token.
 */
void rm_pack_command(uint8_t out[RM_COMMAND_SIZE], const rm_command_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        OPCODE_OFFSET = 1,
        REQUEST_ID_OFFSET = 2,
        TOKEN_OFFSET = 6,
    };

    out[VERSION_OFFSET] = RM_VERSION;
    out[OPCODE_OFFSET] = v->opcode;
    put32(out + REQUEST_ID_OFFSET, v->request_id);
    memcpy(out + TOKEN_OFFSET, v->token, RM_TOKEN_SIZE);
}

/*
 * Pack one legacy DUMP control or single-record frame.
 */
void rm_pack_log(uint8_t out[RM_LOG_SIZE], const rm_log_t *v)
{
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        KIND_OFFSET = 1,
        DUMP_ID_OFFSET = 4,
        GENERATION_OFFSET = 8,
        INDEX_OFFSET = 12,
        BODY_OFFSET = 16,
    };

    memset(out, 0, RM_LOG_SIZE);
    out[VERSION_OFFSET] = RM_VERSION;
    out[KIND_OFFSET] = v->kind;
    put32(out + DUMP_ID_OFFSET, v->dump_id);
    put32(out + GENERATION_OFFSET, v->generation);
    put32(out + INDEX_OFFSET, v->index);
    memcpy(out + BODY_OFFSET, v->body, RM_LOG_RECORD_SIZE);
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
    /* Byte offsets in this topic's wire frame. */
    enum {
        VERSION_OFFSET = 0,
        KIND_OFFSET = 1,
        COUNT_OFFSET = 2,
        FLAGS_OFFSET = 3,
        STREAM_ID_OFFSET = 4,
        GENERATION_OFFSET = 8,
        FIRST_INDEX_OFFSET = 12,
        PUBLISH_MS_OFFSET = 16,
        NEWEST_MS_OFFSET = 20,
        LOG_ROWS_OFFSET = 24,
        SKIPPED_BEFORE_OFFSET = 28,
    };

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
    out[VERSION_OFFSET] = RM_VERSION;
    out[KIND_OFFSET] = header->kind;
    out[COUNT_OFFSET] = header->count;
    out[FLAGS_OFFSET] = header->flags;
    put32(out + STREAM_ID_OFFSET, header->stream_id);
    put32(out + GENERATION_OFFSET, header->generation);
    put32(out + FIRST_INDEX_OFFSET, header->first_index);
    put32(out + PUBLISH_MS_OFFSET, header->publish_ms);
    put32(out + NEWEST_MS_OFFSET, header->newest_ms);
    put32(out + LOG_ROWS_OFFSET, header->log_rows);
    put32(out + SKIPPED_BEFORE_OFFSET, header->skipped_before);
    (void)memcpy(out + RM_LOG_BATCH_HEADER_SIZE, records,
                 (size_t)header->count * RM_LOG_RECORD_SIZE);
    return length;
}

/*
 * Apply the restricted-command policy without touching hardware.
 */
bool rm_command_allowed(uint8_t opcode, bool idle)
{
    return opcode == RM_CMD_ESTOP || opcode == RM_CMD_DUMP ||
           (idle && (opcode == RM_CMD_CALIBRATE || opcode == RM_CMD_START));
}

/*
 * Validate length, opcode and token before changing the output.
 */
bool rm_command_decode(const uint8_t *data, size_t len,
                       const uint8_t expected_token[RM_TOKEN_SIZE],
                       bool enabled, rm_command_t *out)
{
    if (!data || !expected_token || !out || len != RM_COMMAND_SIZE ||
        data[COMMAND_VERSION_OFFSET] != RM_VERSION ||
        !rm_command_allowed(data[COMMAND_OPCODE_OFFSET], true)) {
        return false;
    }
    if (!enabled && data[COMMAND_OPCODE_OFFSET] != RM_CMD_ESTOP) {
        return false;
    }
    if (enabled) {
        uint32_t difference = 0u;
        for (size_t i = 0u; i < RM_TOKEN_SIZE; ++i) {
            difference |=
                (uint32_t)(data[COMMAND_TOKEN_OFFSET + i] ^ expected_token[i]);
        }
        if (difference != 0u) {
            return false;
        }
    }
    rm_command_t value;
    value.opcode = data[COMMAND_OPCODE_OFFSET];
    value.request_id = get32(data + COMMAND_REQUEST_OFFSET);
    memcpy(value.token, data + COMMAND_TOKEN_OFFSET, RM_TOKEN_SIZE);
    *out = value;
    return true;
}
