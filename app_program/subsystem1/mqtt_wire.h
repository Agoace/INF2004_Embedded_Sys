#ifndef MQTT_WIRE_H
#define MQTT_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define RM_VERSION 1u
#define RM_STATE_SIZE 48u
#define RM_SONAR_SIZE 32u
#define RM_EVENT_SIZE 16u
#define RM_HEARTBEAT_SIZE 100u
#define RM_COMMAND_SIZE 22u
#define RM_LOG_SIZE 32u
#define RM_TOKEN_SIZE 16u
enum { RM_CMD_ESTOP=1,RM_CMD_DUMP=2,RM_CMD_CALIBRATE=3,RM_CMD_START=4 };
enum { RM_LOG_BEGIN=0,RM_LOG_ROW=1,RM_LOG_EVENT=2,RM_LOG_END=3,RM_LOG_ABORT=4,
    RM_LOG_ROWS=5,RM_LOG_EVENTS=6 };
typedef struct {
    uint8_t flags,state,module,motion_faults;
    uint32_t uptime_ms,sequence;
    uint16_t line_age_ms,motion_age_ms,vehicle_age_ms,raw_bc;
    uint8_t kr1[16];
    int32_t count_l,count_r;
    uint16_t norm_l,norm_c;
} rm_state_t;
typedef struct {
    uint8_t status;bool valid,pending;
    int8_t angle,ping_angle;
    uint32_t history_sequence,generation,request_id,trigger_ms,timestamp_ms;
    uint16_t echo;
    uint32_t ping_sequence;
} rm_sonar_t;
typedef struct {
    uint8_t kind,from,to;
    uint32_t sequence,uptime_ms;
    uint16_t word;
    uint8_t detail,flags;
} rm_event_t;
typedef struct {
    uint8_t flags,reset_reason;
    uint32_t uptime_ms,connect_attempts,telemetry_lost,event_lost,command_lost;
    uint32_t sonar_history_missed,vehicle_overruns,motion_late,motion_missed;
    uint16_t stack_min_free;
    uint32_t mqtt_hook_max_us,publish_ok,publish_failed;
    uint32_t line_busy,sonar_busy,motion_busy,vehicle_busy,health_busy;
    uint32_t latest_sonar_history_sequence;
    uint16_t health_age_ms;
    int8_t wifi_link_status;
    uint8_t mqtt_state;
    uint32_t wifi_poll_gap_max_us,wifi_poll_gap_samples;
    int32_t wifi_start_status;
    uint8_t wifi_requested_priority,wifi_observed_priority;
} rm_heartbeat_t;
typedef struct { uint8_t opcode;uint32_t request_id;uint8_t token[RM_TOKEN_SIZE]; } rm_command_t;
/* Body is already a 16-byte wire record, or four LE u32 values. */
typedef struct { uint8_t kind;uint32_t dump_id,generation,index;uint8_t body[16]; } rm_log_t;
/* Batched log frames (kinds RM_LOG_ROWS and RM_LOG_EVENTS): a 32-byte header
 * followed by count records of 16 bytes. Record i has index first_index+i.
 * Kinds RM_LOG_ROW and RM_LOG_EVENT (one record per frame) are retired. */
#define RM_LOG_BATCH_HEADER_SIZE 32u
#define RM_LOG_RECORD_SIZE 16u
#define RM_LOG_BATCH_MAX_COUNT 12u
#define RM_LOG_BATCH_MAX_SIZE \
    (RM_LOG_BATCH_HEADER_SIZE + (RM_LOG_BATCH_MAX_COUNT * RM_LOG_RECORD_SIZE))
/* Batch flags. */
#define RM_LOG_FLAG_REFILL 0x01u         /* DUMP refill, not live stream */
#define RM_LOG_FLAG_NEW_GENERATION 0x02u /* first live batch of a generation */
#define RM_LOG_FLAG_JUMPED 0x04u         /* live cursor jumped to newest rows */
typedef struct {
    uint8_t kind;
    uint8_t count;
    uint8_t flags;
    uint32_t stream_id;      /* 0 for the live stream; DUMP request id */
    uint32_t generation;
    uint32_t first_index;
    uint32_t publish_ms;     /* robot uptime when the batch was built */
    uint32_t newest_ms;      /* robot uptime when the newest row was logged */
    uint32_t log_rows;       /* rows in the RAM log when built */
    uint32_t skipped_before; /* rows skipped just before first_index */
} rm_log_batch_t;
/* Returns the frame length, or 0 for a null pointer, bad kind, count 0 or
 * above RM_LOG_BATCH_MAX_COUNT, or capacity too small. out is then unchanged. */
size_t rm_pack_log_batch(uint8_t *out,size_t capacity,
    const rm_log_batch_t *header,const uint8_t *records);
void rm_pack_state(uint8_t out[RM_STATE_SIZE],const rm_state_t *value);
void rm_pack_sonar(uint8_t out[RM_SONAR_SIZE],const rm_sonar_t *value);
void rm_pack_event(uint8_t out[RM_EVENT_SIZE],const rm_event_t *value);
void rm_pack_heartbeat(uint8_t out[RM_HEARTBEAT_SIZE],const rm_heartbeat_t *value);
void rm_pack_command(uint8_t out[RM_COMMAND_SIZE],const rm_command_t *value);
void rm_pack_log(uint8_t out[RM_LOG_SIZE],const rm_log_t *value);
/* Invalid input leaves out unchanged. With commands_enabled (a valid,
 * non-example token configured), all commands require a matching token;
 * comparison visits all 16 bytes. Otherwise only a shape-valid ESTOP is
 * accepted, without checking its token. expected_token must still be nonnull. */
bool rm_command_decode(const uint8_t *data,size_t len,const uint8_t expected_token[RM_TOKEN_SIZE],bool commands_enabled,rm_command_t *out);
bool rm_command_allowed(uint8_t opcode,bool idle);
#endif
