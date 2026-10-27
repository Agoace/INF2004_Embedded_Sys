/**
 * @file line_motion_bridge.h
 * @brief Interface between the Line & Barcode Task (Buddy 3, branch
 *        ziven-line_tracing_algo) and Motion Control (Buddy 2).
 *
 * What Line & Barcode gives Motion:
 *   - The 3-bit black pattern (L = 1, C = 2, R = 4), unknown mask and
 *     fault mask that robocar_ir_sensors.c already computes. Call
 *     line_bridge_update() once per line cycle (1 ms; at least every 100 ms).
 *     The bridge turns them into STEER with motion_set_steer().
 *
 * What Motion gives Line & Barcode (the "encoder input" asked for at
 * Tribunal 1):
 *   - encoder_count(MOTOR_LEFT / MOTOR_RIGHT): raw wrapping 16-bit counts.
 *   - line_bridge_mark() / line_bridge_travel_mm(): distance since a mark,
 *     using a reader-owned odometer_t (never resets the shared counter).
 *     Use it to bound SEARCH_LINE sweeps and to measure barcode length.
 *   - line_bridge_speed_mm_s(): current robot speed from the motion
 *     snapshot, so bar widths timed in ms can be checked against A6
 *     (constant speed +/-10 %).
 *
 * No Pico SDK or RTOS includes: the bridge is host-testable.
 */
#ifndef LINE_MOTION_BRIDGE_H
#define LINE_MOTION_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_shared.h"

#define LINE_BIT_LEFT    (1u << 0)
#define LINE_BIT_CENTRE  (1u << 1)
#define LINE_BIT_RIGHT   (1u << 2)

typedef enum
{
    LINE_STATUS_TRACKING = 0,   /* Contiguous pattern: steer is valid.   */
    LINE_STATUS_LOST,           /* 000: no sensor on black.               */
    LINE_STATUS_JUNCTION,       /* 111: all sensors on black.             */
    LINE_STATUS_SPLIT,          /* 101: ambiguous.                        */
    LINE_STATUS_UNKNOWN,        /* A sensor between thresholds.           */
    LINE_STATUS_FAULT           /* A sensor stuck at a rail.              */
} line_status_t;

/**
 * Same truth table as report_line() in robocar_ir_sensors.c:
 * 1 -> -2, 3 -> -1, 2 -> 0, 6 -> +1, 4 -> +2 (negative = line on the left).
 */
line_status_t line_bridge_classify(uint8_t black_pattern, uint8_t unknown_mask,
                                   uint8_t fault_mask, int8_t * p_error);

/** Map the -2..+2 line error to the -127..+127 motion STEER scale. */
int8_t        line_bridge_steer_from_error(int8_t error);

/** Set the STEER gain table (index = error + 2). Default -110,-50,0,50,110. */
void          line_bridge_set_steer_table(const int8_t table[5]);

/**
 * Per line cycle: classify, then refresh STEER.
 *  TRACKING      -> new steer from the error.
 *  LOST          -> keep the last steer (search bias toward the side the
 *                   line left); the VC decides LINE_SEARCH after 50 ms.
 *  JUNCTION/SPLIT/UNKNOWN -> steer 0 (cross straight until the VC acts).
 *  FAULT         -> STEER is NOT refreshed, so Motion holds the wheels
 *                   after 100 ms (stale-STEER timeout).
 * Returns the status so the Line task can post LINE_LOST/JUNCTION events.
 */
line_status_t line_bridge_update(uint8_t black_pattern, uint8_t unknown_mask,
                                 uint8_t fault_mask);

/** Start a distance measurement (e.g. at the start of SEARCH_LINE). */
void          line_bridge_mark(void);

/** Signed average travel of both wheels since line_bridge_mark(), in mm. */
int32_t       line_bridge_travel_mm(void);

/** Average wheel speed from the motion snapshot, mm/s (signed). */
int32_t       line_bridge_speed_mm_s(void);

#endif /* LINE_MOTION_BRIDGE_H */
