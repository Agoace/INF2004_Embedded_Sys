/*
 * The three line-tracing IR sensors (ADC): left GP26, centre GP27, right GP28.
 * Each read is the mean of 8 ADC samples per sensor (about 50 us in total).
 */
#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <stdint.h>

#include "line_position.h"

void line_sensors_init(void);
void line_sensors_read(uint16_t raw[LP_SENSORS]);

#endif
