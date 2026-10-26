#ifndef BUDDY1_SERVICE_H
#define BUDDY1_SERVICE_H

/* Standalone diagnostic bench only: no robot or RTOS interfaces. Single owner. */
#include "buddy1_command.h"

#define B1_PENDING_COMMANDS 4u
#define B1_PENDING_RESULTS 8u
#define B1_CONNECT_TIMEOUT_MS 10000u
/* Local bench residence limit, not an MQTT delivery-age/expiry guarantee. */
#define B1_COMMAND_MAX_AGE_MS 1000u

typedef enum { B1_BACKOFF, B1_WIFI, B1_MQTT, B1_SUBSCRIBING, B1_READY } b1_phase_t;
typedef enum {
    B1_BENCH_RECORDED, B1_DUPLICATE, B1_CONFLICT, B1_OLD,
    B1_WRONG_SESSION, B1_MALFORMED, B1_BUSY, B1_NOT_READY
} b1_outcome_t;

typedef struct { uint8_t data[8]; uint8_t length; } b1_reply_t;
typedef struct {
    b1_phase_t phase;
    uint32_t since_ms, retry_ms, telemetry_ms, heartbeat_ms;
    uint32_t now_ms, rx_started_ms, command_received_ms[B1_PENDING_COMMANDS];
    uint8_t failures;
    b1_sequence_t sequence;
    b1_assembler_t assembler;
    bool receiving;
    b1_command_t commands[B1_PENDING_COMMANDS];
    uint8_t command_head, command_count;
    b1_reply_t replies[B1_PENDING_RESULTS];
    uint8_t reply_head, reply_count;
    uint32_t recorded, malformed, busy, dropped_results;
} b1_service_t;

void b1_service_init(b1_service_t *s, uint32_t now);
/* Single owner calls before receiving/processing and after servicing networking. */
void b1_service_update_time(b1_service_t *s, uint32_t now);
void b1_service_phase(b1_service_t *s, b1_phase_t phase, uint32_t now);
void b1_service_session(b1_service_t *s, uint32_t session, uint32_t now);
void b1_service_disconnect(b1_service_t *s, uint32_t now);
bool b1_service_timeout(const b1_service_t *s, uint32_t now);
bool b1_service_retry_due(const b1_service_t *s, uint32_t now);
bool b1_service_telemetry_due(b1_service_t *s, uint32_t now);
bool b1_service_heartbeat_due(b1_service_t *s, uint32_t now);
void b1_service_rx_begin(b1_service_t *s, size_t total);
void b1_service_rx_data(b1_service_t *s, const uint8_t *data, size_t length, bool last);
/* Process at most one diagnostic command. Never controls the robot. */
void b1_service_process(b1_service_t *s);
const b1_reply_t *b1_service_reply(const b1_service_t *s);
void b1_service_reply_sent(b1_service_t *s);
/* Status/LWT: version[7:6], online[5], command-ready[4], zero[3:0], session BE. */
void b1_service_status(const b1_service_t *s, uint8_t output[5]);

#endif
