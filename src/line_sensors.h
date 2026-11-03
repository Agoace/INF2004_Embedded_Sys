/*
 * The line-tracing IR sensors (ADC), robot's own sides.
 *
 * LINE_SENSORS_PAIR 1 (course rule 2026-11-03: two line sensors):
 *   left GP27 (the old left sensor, moved from GP28), centre GP26 (the old
 *   centre sensor, moved from GP27). The right sensor is disabled: raw[2]
 *   is always 0. Use lp_set_pair() with this wiring.
 * LINE_SENSORS_PAIR 0: the old three-sensor wiring, left GP28, centre
 *   GP27, right GP26. src/ir_module_pins.h has left and right the other way
 *   round (checked on the robot, 2026-10-27).
 * Each read is the mean of 8 ADC samples per sensor.
 */
#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <stdint.h>

#include "line_position.h"

#ifndef LINE_SENSORS_PAIR
#define LINE_SENSORS_PAIR 1
#endif

void line_sensors_init(void);
void line_sensors_read(uint16_t raw[LP_SENSORS]);

#endif
