#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Public contract between the line task and the Code 39 decoder. */

#define BARCODE_TEXT_MAX 48u

typedef enum {
    BARCODE_OK = 0,
    BARCODE_NO_BARCODE,
    BARCODE_PARTIAL,
    BARCODE_DECODE_FAIL,
    BARCODE_TRUNCATED,
} barcode_status_t;

typedef struct {
    bool black_level;        /* Digital level from a black bar. */
    uint32_t min_run_us;     /* Shorter pulses are treated as noise. */
    uint32_t quiet_zone_us;  /* Clear space required before and after a code. */
} barcode_config_t;

typedef struct {
    barcode_status_t status;
    char text[BARCODE_TEXT_MAX]; /* Empty unless status is BARCODE_OK. */
} barcode_result_t;

/*
 * Initialise the reader before its first sample. The caller owns the GPIO and
 * supplies the initial digital level and every later sample timestamp.
 */
void barcode_init(const barcode_config_t *config, bool initial_level,
                  uint32_t now_us);

/*
 * Submit one sample from the barcode sensor. Call this at a fixed period that
 * is short enough to observe the narrowest expected bar at the chosen duty
 * limit. This function does not access hardware or block.
 */
void barcode_sample(bool level, uint32_t now_us);

/*
 * Copy and remove the next completed barcode result. A result describes a
 * valid Code 39 value or a completed rejected capture. Returns false when no
 * result is waiting.
 */
bool barcode_take_result(barcode_result_t *result);
