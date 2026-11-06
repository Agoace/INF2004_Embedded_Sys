#ifndef KERNEL_TELEMETRY_H
#define KERNEL_TELEMETRY_H
#include <stdbool.h>
#include <stdint.h>
#define TELEMETRY_ROWS 4501u
#define TELEMETRY_BYTES 16u
#define TELEMETRY_EVENTS 512u
typedef struct { uint32_t ms;uint16_t word;uint8_t kind,from,to,detail; } telemetry_event_t;
typedef struct {
    char reason[48];
    uint32_t wake_lost,overruns,response_max_us,gap_min_us,gap_max_us;
    uint32_t sampled,missed,dropped,isr_max_us,line_max_us;
    int32_t counts[2];
    uint16_t floor[2],line[2],raw_bc;
    uint8_t faults;
} telemetry_info_t;
typedef enum { TELEMETRY_BEGIN,TELEMETRY_ROW,TELEMETRY_EVENT,TELEMETRY_INFO,TELEMETRY_DUMP } telemetry_kind_t;
typedef struct {
    uint32_t kind,loss_base;
    union { uint8_t row[TELEMETRY_BYTES];telemetry_event_t event;telemetry_info_t info; } data;
} telemetry_message_t;
typedef struct { uint32_t lost,rows,events;bool busy,storage_full; } telemetry_stats_t;
int telemetry_init(void);
int telemetry_start(void);
/* Core-0 task producers, value copy, MBF TMO_POL; never wait for the consumer. */
int telemetry_send(const telemetry_message_t *message);
void telemetry_stats(telemetry_stats_t *out);
#endif
