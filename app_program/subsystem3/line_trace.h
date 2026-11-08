/*
 * Line tracing module for the Vehicle Controller shell (module LINE,
 * src/vehicle_sm.h). See docs/s3-line-barcode/LINE.md.
 *
 * The behaviour of the locked FOLLOW phase of detour_test
 * (docs/s3-line-barcode/LINE.md) in wheel speeds:
 *   - steering: steer_mm_update() on the line position
 *     (docs/s3-line-barcode/LINE.md); straight across a junction for
 *     straight_counts of travel;
 *   - side hint to the line detector from the wheel counts: turned more
 *     than hint_counts to the right means the line is probably left;
 *   - forward watch: obstacle_update() on each ultrasonic reading, servo
 *     straight.
 *
 * Outcomes:
 *   LINE_LOST  the line detector's LINE_LOST event; payload: its side
 *   OBSTACLE   forward watch STOP; payload: distance mm
 *   FAILED     forward watch SENSOR_FAULT; payload LT_FAIL_ECHO
 *
 * Pure logic: host-tested by tests/test_line_trace.c.
 */
#ifndef LINE_TRACE_H
#define LINE_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#include "line_steer.h"
#include "../subsystem5/obstacle.h"
#include "../vehicle/vehicle_inputs.h"
#include "../vehicle/vehicle_sm.h"

#define LT_FAIL_ECHO 1u /* FAILED payload: the ultrasonic sensor is faulty */

typedef struct {
    steer_mm_config_t steer;
    int32_t straight_counts; /* junction: straight for this travel (mean of both wheels) */
    int32_t hint_counts;     /* side hint: turn (left - right counts) above this */
} lt_config_t;

typedef struct {
    lt_config_t cfg;
    steer_mm_t steer;
    obstacle_monitor_t watch;
    bool have_counts; /* last[] is valid (false until the first step) */
    int32_t last[2];
    int32_t heading; /* left - right counts since the line was last centred */
    bool was_hold;
    bool straight; /* this step drove straight (junction) */
    int32_t straight_from;
} lt_t;

void lt_init(lt_t *lt, const lt_config_t *cfg);

/* vsm_module_t functions; ctx is an lt_t. */
void lt_start(void *ctx, uint16_t payload, uint32_t now_ms);
vsm_outcome_t lt_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive);

static inline vsm_module_t lt_module(lt_t *lt) {
    return (vsm_module_t){lt_start, lt_step, lt};
}

#endif
