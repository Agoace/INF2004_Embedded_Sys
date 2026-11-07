/*
 * mqtt_wire.h - Binary MQTT frame sizes, value types and pure codec API.
 */
#ifndef MQTT_WIRE_H
#define MQTT_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define RM_VERSION    1u
#define RM_STATE_SIZE 72u
/* robot/state is version 3: v1 (48 B), v2 distances, v3 duty and odometry. */
#define RM_STATE_VERSION  3u
#define RM_STATE_V1_SIZE  48u
#define RM_STATE_V2_SIZE  60u
#define RM_SONAR_SIZE     32u
#define RM_OBSTACLE_SIZE  20u
#define RM_STATUS_SIZE    2u
#define RM_EVENT_SIZE     16u
#define RM_HEARTBEAT_SIZE 100u
/* robot/heartbeat is version 2: bytes 98-99 are the CPU load. */
#define RM_HEARTBEAT_VERSION 2u
#define RM_CPU_LOAD_UNKNOWN  65535u
#define RM_DETOUR_SIZE       32u
#define RM_SUMMARY_SIZE      44u
/* Value of an unavailable 16-bit measurement in robot/summary. */
#define RM_SUMMARY_NONE_U16 65535u
#define RM_COMMAND_SIZE     22u
#define RM_LOG_SIZE         32u
#define RM_TOKEN_SIZE       16u
/* Saturated age, or "not available" for 16-bit stack/age fields. */
#define RM_NO_DATA_U16 65535u

enum {
    RM_CMD_ESTOP = 1,
    RM_CMD_DUMP = 2,
    RM_CMD_CALIBRATE = 3,
    RM_CMD_START = 4
};

enum {
    RM_LOG_BEGIN = 0,
    RM_LOG_ROW = 1,
    RM_LOG_EVENT = 2,
    RM_LOG_END = 3,
    RM_LOG_ABORT = 4,
    RM_LOG_ROWS = 5,
    RM_LOG_EVENTS = 6
};

typedef struct {
    uint8_t flags;
    uint8_t state;
    uint8_t module;
    uint8_t motion_faults;
    uint32_t uptime_ms;
    uint32_t sequence;
    uint16_t line_age_ms;
    uint16_t motion_age_ms;
    uint16_t vehicle_age_ms;
    uint16_t raw_bc;
    uint8_t kr1[16];
    int32_t count_l;
    int32_t count_r;
    uint16_t norm_l;
    uint16_t norm_c;
    int32_t dist_left_mm;
    int32_t dist_right_mm;
    int32_t dist_avg_mm;
    int16_t duty_left_permille;
    int16_t duty_right_permille;
    int32_t heading_cdeg;
    int16_t turn_rate_ddps;
} rm_state_t;

/* robot/status payload byte 1. Both values are non-zero on purpose. */
enum {
    RM_STATUS_ONLINE = 1,
    RM_STATUS_OFFLINE = 2
};

/* robot/obstacle flags (byte 1). */
enum {
    RM_OBSTACLE_CLOSEST_VALID = 0x01,
    RM_OBSTACLE_TOO_NEAR = 0x02,
    RM_OBSTACLE_LEFT_BLOCKED = 0x04,
    RM_OBSTACLE_RIGHT_BLOCKED = 0x08,
    RM_OBSTACLE_STAGE_SHIFT = 4,
    RM_OBSTACLE_STAGE_MASK = 0x30
};

/* One completed obstacle scan. Decision is obstacle_side_t: 0 none, 1 right,
 * 2 left. Angles: negative right, positive left. */
typedef struct {
    uint8_t flags;
    uint8_t decision;
    int8_t closest_angle_deg;
    uint32_t scan_id;
    uint32_t uptime_ms;
    uint16_t closest_mm;
    uint16_t profile_word;
    uint8_t right_free_step;
    uint8_t left_free_step;
    uint8_t right_clear_deg;
    uint8_t left_clear_deg;
} rm_obstacle_t;

/* robot/detour outcome (byte 1): the shell outcome numbers, plus ABORTED. */
enum {
    RM_DETOUR_DONE_LINE = 3,
    RM_DETOUR_DONE_NO_LINE = 4,
    RM_DETOUR_FAILED = 7,
    RM_DETOUR_ABORTED = 8
};

/* robot/detour flags (byte 3). */
enum {
    RM_DETOUR_LATERAL_VALID = 0x01
};

/* One finished detour. Distances are millimetres from encoder odometry. */
typedef struct {
    uint8_t outcome;
    uint8_t side; /* 0 none, 1 right, 2 left */
    uint8_t flags;
    uint32_t sequence;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t duration_ms;
    uint32_t distance_mm;
    uint16_t max_lateral_mm;
    uint16_t payload;
    uint32_t scan_id;
} rm_detour_t;

/* robot/summary stop source (byte 2). */
enum {
    RM_STOP_SOURCE_NONE = 0,
    RM_STOP_SOURCE_GP21 = 1,
    RM_STOP_SOURCE_MQTT = 2,
    RM_STOP_SOURCE_FAULT = 3,
    RM_STOP_SOURCE_LIMIT = 4
};

/* robot/summary stop reason (byte 3). */
enum {
    RM_STOP_UNKNOWN = 0,
    RM_STOP_GP21 = 1,
    RM_STOP_MQTT_ESTOP = 2,
    RM_STOP_TIME_LIMIT = 3,
    RM_STOP_SHELL = 4,
    RM_STOP_DEADLINE = 5,
    RM_STOP_MOTION_WAKE = 6,
    RM_STOP_MOTION_STATUS = 7,
    RM_STOP_LINE_STALE = 8,
    RM_STOP_LINE_LOST_SAMPLES = 9,
    RM_STOP_SONAR = 10,
    RM_STOP_TELEMETRY = 11,
    RM_STOP_OTHER = 12
};

/* robot/summary flags (byte 1). */
enum {
    RM_SUMMARY_BARCODE_IMPLEMENTED = 0x01,
    RM_SUMMARY_SPEED_ERROR_VALID = 0x02,
    RM_SUMMARY_DETOUR_ABORTED = 0x04
};

/* End-of-run totals. Retained on the broker. */
typedef struct {
    uint8_t flags;
    uint8_t stop_source;
    uint8_t stop_reason;
    uint32_t run_id;
    uint32_t start_ms;
    uint32_t duration_ms;
    uint32_t distance_mm;
    uint16_t line_losses;
    uint16_t obstacles;
    uint16_t detours_done;
    uint16_t detours_succeeded;
    uint32_t detour_total_ms;
    uint16_t detours_started;
    uint16_t max_speed_error_mm_s;
    uint8_t motion_fault_bits;
    uint8_t shell_fault;
    uint16_t shell_fault_detail;
    uint8_t barcode_ok;
    uint8_t barcode_failed;
    uint16_t hump_peak;
} rm_summary_t;

typedef struct {
    uint8_t status;
    bool valid;
    bool pending;
    int8_t angle;
    int8_t ping_angle;
    uint32_t history_sequence;
    uint32_t generation;
    uint32_t request_id;
    uint32_t trigger_ms;
    uint32_t timestamp_ms;
    uint16_t echo;
    uint32_t ping_sequence;
} rm_sonar_t;

typedef struct {
    uint8_t kind;
    uint8_t from;
    uint8_t to;
    uint32_t sequence;
    uint32_t uptime_ms;
    uint16_t word;
    uint8_t detail;
    uint8_t flags;
} rm_event_t;

typedef struct {
    uint8_t flags;
    uint8_t reset_reason;
    uint32_t uptime_ms;
    uint32_t connect_attempts;
    uint32_t telemetry_lost;
    uint32_t event_lost;
    uint32_t command_lost;
    uint32_t sonar_history_missed;
    uint32_t vehicle_overruns;
    uint32_t motion_late;
    uint32_t motion_missed;
    uint16_t stack_min_free;
    uint32_t mqtt_hook_max_us;
    uint32_t publish_ok;
    uint32_t publish_failed;
    uint32_t line_busy;
    uint32_t sonar_busy;
    uint32_t motion_busy;
    uint32_t vehicle_busy;
    uint32_t health_busy;
    uint32_t latest_sonar_history_sequence;
    uint16_t health_age_ms;
    int8_t wifi_link_status;
    uint8_t mqtt_state;
    uint32_t wifi_poll_gap_max_us;
    uint32_t wifi_poll_gap_samples;
    int32_t wifi_start_status;
    uint8_t wifi_requested_priority;
    uint8_t wifi_observed_priority;
    uint16_t cpu_load_permille; /* RM_CPU_LOAD_UNKNOWN if not measured */
} rm_heartbeat_t;

typedef struct {
    uint8_t opcode;
    uint32_t request_id;
    uint8_t token[RM_TOKEN_SIZE];
} rm_command_t;

/* Body is already a 16-byte wire record, or four LE u32 values. */
typedef struct {
    uint8_t kind;
    uint32_t dump_id;
    uint32_t generation;
    uint32_t index;
    uint8_t body[16];
} rm_log_t;

/* Batched log frames (kinds RM_LOG_ROWS and RM_LOG_EVENTS): a 32-byte header
 * followed by count records of 16 bytes. Record i has index first_index+i.
 * Kinds RM_LOG_ROW and RM_LOG_EVENT (one record per frame) are retired. */
#define RM_LOG_BATCH_HEADER_SIZE 32u
#define RM_LOG_RECORD_SIZE       16u
#define RM_LOG_BATCH_MAX_COUNT   12u
#define RM_LOG_BATCH_MAX_SIZE                                                  \
    (RM_LOG_BATCH_HEADER_SIZE + (RM_LOG_BATCH_MAX_COUNT * RM_LOG_RECORD_SIZE))
/* Batch flags. */
#define RM_LOG_FLAG_REFILL         0x01u /* DUMP refill, not live stream */
#define RM_LOG_FLAG_NEW_GENERATION 0x02u /* first live batch of a generation */
#define RM_LOG_FLAG_JUMPED         0x04u /* live cursor jumped to newest rows */

typedef struct {
    uint8_t kind;
    uint8_t count;
    uint8_t flags;
    uint32_t stream_id; /* 0 for the live stream; DUMP request id */
    uint32_t generation;
    uint32_t first_index;
    uint32_t publish_ms;     /* robot uptime when the batch was built */
    uint32_t newest_ms;      /* robot uptime when the newest row was logged */
    uint32_t log_rows;       /* rows in the RAM log when built */
    uint32_t skipped_before; /* rows skipped just before first_index */
} rm_log_batch_t;

/* Returns the frame length, or 0 for a null pointer, bad kind, count 0 or
 * above RM_LOG_BATCH_MAX_COUNT, or capacity too small. out is then unchanged. */
size_t rm_pack_log_batch(uint8_t *out, size_t capacity,
                         const rm_log_batch_t *header, const uint8_t *records);
/* Mean of two signed distances, truncated toward zero, no overflow. */
int32_t rm_average_mm(int32_t left_mm, int32_t right_mm);
void rm_pack_obstacle(uint8_t out[RM_OBSTACLE_SIZE], const rm_obstacle_t *v);
void rm_pack_detour(uint8_t out[RM_DETOUR_SIZE], const rm_detour_t *v);
void rm_pack_summary(uint8_t out[RM_SUMMARY_SIZE], const rm_summary_t *v);
void rm_pack_status(uint8_t out[RM_STATUS_SIZE], uint8_t state);
void rm_pack_state(uint8_t out[RM_STATE_SIZE], const rm_state_t *value);
void rm_pack_sonar(uint8_t out[RM_SONAR_SIZE], const rm_sonar_t *value);
void rm_pack_event(uint8_t out[RM_EVENT_SIZE], const rm_event_t *value);
void rm_pack_heartbeat(uint8_t out[RM_HEARTBEAT_SIZE],
                       const rm_heartbeat_t *value);
void rm_pack_command(uint8_t out[RM_COMMAND_SIZE], const rm_command_t *value);
void rm_pack_log(uint8_t out[RM_LOG_SIZE], const rm_log_t *value);
/* Invalid input leaves out unchanged. With commands_enabled (a valid,
 * non-example token configured), all commands require a matching token;
 * comparison visits all 16 bytes. Otherwise only a shape-valid ESTOP is
 * accepted, without checking its token. expected_token must still be nonnull. */
bool rm_command_decode(const uint8_t *data, size_t len,
                       const uint8_t expected_token[RM_TOKEN_SIZE],
                       bool commands_enabled, rm_command_t *out);
bool rm_command_allowed(uint8_t opcode, bool idle);
#endif
