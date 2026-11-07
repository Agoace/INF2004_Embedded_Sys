/*
 * sonar_service.h - Sonar request service, snapshots and sequence-numbered history.
 */
#ifndef SONAR_SERVICE_H
#define SONAR_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
#include "ultrasonic.h"

typedef enum {
    SONAR_STOP,
    SONAR_WATCH,
    SONAR_ONCE,
    SONAR_ACK
} sonar_mode_t;

typedef struct {
    uint32_t generation;
    uint32_t id;
    uint32_t deadline_ms;
    uint32_t not_before_ms;
    uint16_t interval_ms;
    int8_t angle;
    uint8_t mode;
} sonar_request_t;

typedef enum {
    SONAR_EMPTY,
    SONAR_ECHO,
    SONAR_DEADLINE,
    SONAR_CANCELLED
} sonar_status_t;

/* ECHO timestamp is actual trigger time, not the later service publication. */
typedef struct {
    uint32_t generation;
    uint32_t id;
    uint32_t ping_sequence;
    uint32_t result_sequence;
    uint32_t trigger_ms;
    uint32_t timestamp_ms;
    uint32_t deadline_ms;
    uint32_t ping_generation;
    uint32_t ping_id;
    int8_t ping_angle;
    ultrasonic_result_t echo;
    int8_t angle;
    uint8_t status;
    bool pending;
    bool valid;
} sonar_snapshot_t;

int sonar_service_init(void);
int sonar_service_start(void);
bool sonar_service_running(void);
/* Copies by value through MBF, TMO_POL; callers must retry or stop on failure. */
int sonar_request(const sonar_request_t *request);
int sonar_result_read(sonar_snapshot_t *out, bool *new_result);
/* Bounded, non-consuming read: E_TMOUT if a writer is active or changed. */
int sonar_snapshot_try_read(sonar_snapshot_t *out);
#define SONAR_HISTORY_CAPACITY 16u

typedef struct {
    uint32_t sequence;
    sonar_snapshot_t result;
} sonar_history_entry_t;

/* One next publication. E_NOEXS: no new entry; E_TMOUT: writer busy;
 * E_OBJ: service not ready. These results are distinct for health accounting.
 * Set cursor to zero initially. Overflow returns the oldest retained entry
 * and reports the number of evicted entries. Busy leaves cursor/out intact.
 * Readers must run at least once per 2^31 publications. No retry loop. */
int sonar_history_read(uint32_t *cursor, sonar_history_entry_t *out,
                       uint32_t *missed);
#endif
