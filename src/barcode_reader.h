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
    uint16_t threshold;
    uint16_t hysteresis;
    uint16_t min_run_samples;
} barcode_reader_config_t;

typedef struct {
    barcode_reader_config_t config;
    uint16_t runs[BARCODE_MAX_RUNS];
    size_t run_count;
    uint16_t current_run;
    bool active;
    bool level;
    bool overflowed;
} barcode_reader_t;

typedef struct {
    barcode_status_t status;
    uint16_t edge_count;
    char text[BARCODE_MAX_TEXT];
} barcode_result_t;

void barcode_reader_init(barcode_reader_t *reader, const barcode_reader_config_t *config);
void barcode_reader_start(barcode_reader_t *reader, uint16_t first_sample);
void barcode_reader_process_sample(barcode_reader_t *reader, uint16_t sample);
void barcode_reader_finish(barcode_reader_t *reader, barcode_result_t *result);
