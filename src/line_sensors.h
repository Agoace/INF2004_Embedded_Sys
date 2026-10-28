/*
 * The three line-tracing IR sensors (ADC), robot's own sides:
 * left GP28, centre GP27, right GP26. src/ir_module_pins.h has left and
 * right the other way round (checked on the robot, 2026-10-27).
 * Each read is the mean of 8 ADC samples per sensor (about 50 us in total).
 */
#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <stdint.h>

#include "line_position.h"

void line_sensors_init(void);
void line_sensors_read(uint16_t raw[LP_SENSORS]);

#endif
