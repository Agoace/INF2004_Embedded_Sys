#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BARCODE_MAX_RUNS 512u
#define BARCODE_MAX_TEXT 48u

typedef enum {
    BARCODE_IDLE = 0,
    BARCODE_OK,
    BARCODE_NO_BARCODE,
    BARCODE_PARTIAL,
    BARCODE_DECODE_FAIL,
    BARCODE_TRUNCATED,
} barcode_status_t;

typedef struct {
    bool black_level;         /* Digital level produced by a black bar. */
    uint32_t min_run_us;      /* Shorter pulses are rejected as noise. */
    uint32_t quiet_zone_us;   /* Minimum clear space before and after a scan. */
} barcode_reader_config_t;

typedef struct {
    barcode_reader_config_t config;
    uint32_t runs[BARCODE_MAX_RUNS];
    size_t run_count;
    uint32_t last_edge_us;
    uint32_t shortest_run_us;
    bool level;
    bool collecting;
} barcode_reader_t;

typedef struct {
    barcode_status_t status;
    uint16_t edge_count;
    char text[BARCODE_MAX_TEXT];
} barcode_result_t;

void barcode_reader_init(barcode_reader_t *reader, const barcode_reader_config_t *config,
                         bool initial_level, uint32_t now_us);
/* Returns true only when a complete scan or a rejected capture is available. */
bool barcode_reader_process_edge(barcode_reader_t *reader, bool level,
                                 uint32_t timestamp_us, barcode_result_t *result);
bool barcode_reader_poll(barcode_reader_t *reader, uint32_t now_us,
                         barcode_result_t *result);
