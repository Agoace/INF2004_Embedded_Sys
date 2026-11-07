#ifndef KERNEL_TELEMETRY_H
#define KERNEL_TELEMETRY_H
#include <stdbool.h>
#include <stdint.h>
#define TELEMETRY_ROWS   4501u
#define TELEMETRY_BYTES  16u
#define TELEMETRY_EVENTS 512u

typedef struct {
    uint32_t ms;
    uint16_t word;
    uint8_t kind;
    uint8_t from;
    uint8_t to;
    uint8_t detail;
} telemetry_event_t;

typedef struct {
    char reason[48];
    uint32_t wake_lost;
    uint32_t overruns;
    uint32_t response_max_us;
    uint32_t gap_min_us;
    uint32_t gap_max_us;
    uint32_t sampled;
    uint32_t missed;
    uint32_t dropped;
    uint32_t isr_max_us;
    uint32_t line_max_us;
    int32_t counts[2];
    uint16_t floor[2];
    uint16_t line[2];
    uint16_t raw_bc;
    uint8_t faults;
} telemetry_info_t;

typedef enum {
    TELEMETRY_BEGIN,
    TELEMETRY_ROW,
    TELEMETRY_EVENT,
    TELEMETRY_INFO,
    TELEMETRY_DUMP
} telemetry_kind_t;

typedef struct {
    uint32_t kind;
    uint32_t loss_base;

    union {
        uint8_t row[TELEMETRY_BYTES];
        telemetry_event_t event;
        telemetry_info_t info;
    } data;
} telemetry_message_t;

typedef struct {
    uint32_t lost;
    uint32_t rows;
    uint32_t events;
    bool busy;
    bool storage_full;
} telemetry_stats_t;

int telemetry_init(void);
int telemetry_start(void);
bool telemetry_running(void);
/* Core-0 task producers, value copy, MBF TMO_POL; never wait for the consumer. */
int telemetry_send(const telemetry_message_t *message);
void telemetry_stats(telemetry_stats_t *out);
#if TM_MQTT
typedef struct {
    uint32_t generation;
    uint32_t rows;
    uint32_t events;
    uint32_t lost;
    uint32_t newest_ms; /* robot uptime, ms, when the newest row was stored */
} telemetry_log_index_t;

/* Wi-Fi/core0 bounded readers: committed highwater only; never stall logging.
 * E_OBJ generation changed, E_TMOUT publication busy, E_PAR bad index. */
int telemetry_log_open(telemetry_log_index_t *out);
int telemetry_log_row(uint32_t generation, uint32_t index,
                      uint8_t out[TELEMETRY_BYTES]);
int telemetry_log_event(uint32_t generation, uint32_t index,
                        telemetry_event_t *out);
#endif
#endif
