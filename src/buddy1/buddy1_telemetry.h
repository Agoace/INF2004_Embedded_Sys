#ifndef BUDDY1_TELEMETRY_H
#define BUDDY1_TELEMETRY_H

/* Temporary Stage 2 bench format; not the final robot telemetry contract. */
#include <stddef.h>
#include <stdint.h>

#define B1_TLM_BYTES 16u
#define B1_TLM_VERSION 1u
#define B1_TLM_UNAVAILABLE 0u
#define B1_TLM_VALID 1u
#define B1_TLM_STALE 2u
#define B1_TLM_FAULT 3u

/* Working values, not a packed RAM/shared snapshot representation.
 * timestamp_ms is caller-supplied frame-capture time modulo 2^32 ms.
 * Quality, freshness, session lifecycle and previous values belong to caller.
 */
typedef struct {
    uint32_t session;
    uint32_t timestamp_ms;
    uint16_t sequence;
    uint16_t adc[3];
    uint8_t pattern;
    uint8_t barcode;
    uint8_t line_quality;
    uint8_t barcode_quality;
} b1_tlm_frame_t;

typedef enum {
    B1_TLM_OK = 0,
    B1_TLM_BAD_ARGUMENT,
    B1_TLM_BAD_LENGTH,
    B1_TLM_BAD_VERSION,
    B1_TLM_BAD_PADDING,
    B1_TLM_BAD_VALUE
} b1_tlm_result_t;

/* Exact length required. Explicit MSB-first packing, big-endian metadata.
 * Unavailable/fault groups require zero values; stale preserves supplied data.
 * Encode leaves output unchanged on error. Decode clears output on error
 * (both groups unavailable). Always check the result before using output.
 * Input/output must not overlap. No allocation, I/O, locks or shared state.
 */
b1_tlm_result_t b1_tlm_encode(const b1_tlm_frame_t *frame,
                             uint8_t *output, size_t length);
b1_tlm_result_t b1_tlm_decode(const uint8_t *data, size_t length,
                             b1_tlm_frame_t *output);

#endif
