/*
 * Line search module for the Vehicle Controller shell (module SEARCH,
 * src/vehicle_sm.h). See docs/s3-line-barcode/VC_LINE_RUN.md.
 *
 * The search of detour_test (docs/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md)
 * in wheel
 * speeds: turn on the spot toward the side where the line was last seen
 * (payload: vc_line_side_t; RIGHT turns right, else left) until the turn
 * (right - left counts) / 2 reaches turn_counts, then the other way to
 * turn_counts past the start.
 *
 * Outcomes:
 *   FOUND   the line detector's LINE_FOUND event
 *   FAILED  both turns done without the line; payload LS_FAIL_NOT_FOUND
 *
 * Pure logic: host-tested by tests/test_line_trace.c.
 */
#ifndef LINE_SEARCH_H
#define LINE_SEARCH_H

#include <stdbool.h>
#include <stdint.h>

#include "../vehicle/vehicle_inputs.h"
#include "../vehicle/vehicle_sm.h"

#define LS_FAIL_NOT_FOUND 1u

typedef struct {
    int16_t speed_mm_s; /* each wheel, on the spot */
    int32_t turn_counts;
} ls_config_t;

typedef struct {
    ls_config_t cfg;
    int direction; /* +1 left first, -1 right first */
    uint8_t step;  /* 0 first side, 1 other side */
    bool have_start;
    int32_t start[2];
    uint16_t searches; /* since ls_init() */
} ls_t;

void ls_init(ls_t *ls, const ls_config_t *cfg);
void ls_start(void *ctx, uint16_t payload, uint32_t now_ms);
vsm_outcome_t ls_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive);

static inline vsm_module_t ls_module(ls_t *ls) {
    return (vsm_module_t){ls_start, ls_step, ls};
}

#endif
