#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "line_detector.h"

#define IR_LINE_SENSOR_COUNT 3u

/* Three 12-bit ADC means and three 2-bit classifications occupy 42 bits. */
typedef struct {
    uint8_t bytes[6];
} ir_line_data_t;

bool ir_line_data_encode(const uint16_t adc[IR_LINE_SENSOR_COUNT],
                         const line_class_t classes[IR_LINE_SENSOR_COUNT],
                         ir_line_data_t *data);
bool ir_line_data_decode(const ir_line_data_t *data,
                         uint16_t adc[IR_LINE_SENSOR_COUNT],
                         line_class_t classes[IR_LINE_SENSOR_COUNT]);
uint8_t ir_line_black_pattern(const ir_line_data_t *data);
bool ir_line_steering_error(const ir_line_data_t *data, int8_t *error);
