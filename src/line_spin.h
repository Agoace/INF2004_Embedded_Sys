/*
 * Spin calibration for the line position (src/line_position.c).
 *
 * Start with the centre sensor over the line. The robot turns on the spot
 * a quarter wheel turn left, then through the start to a quarter wheel turn
 * right, sampling the sensors every 1 ms. Then the calibration is made and
 * the robot turns back until the line is under the centre sensor.
 *
 * Limits for USB power (USB_POWER_SAFETY.md, spin-calibration exception):
 * duty 20 %, the right wheel starts 50 ms after the left, and a stall (no
 * encoder count for 500 ms after a 1 s start) or 6 s per step stops both.
 *
 * Needs drive_init() and line_sensors_init(). Prints its progress.
 */
#ifndef LINE_SPIN_H
#define LINE_SPIN_H

#include <stdbool.h>

#include "line_position.h"

typedef enum {
    LINE_SPIN_OK = 0,          /* calibrated and back on the line */
    LINE_SPIN_STOPPED = 1,     /* stall or time limit; calibration not changed */
    LINE_SPIN_CALIB_FAILED = 2 /* sweep done but calibration failed */
} line_spin_result_t;

line_spin_result_t line_spin_calibrate(lp_t *lp);

/* Ends a calibration (lp_calib_end) and prints the result. True if OK. */
bool line_calib_end_and_report(lp_t *lp);

#endif
