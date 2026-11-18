#include "ir_line_data.h"

bool ir_line_data_encode(const uint16_t adc[IR_LINE_SENSOR_COUNT],
                         const line_class_t classes[IR_LINE_SENSOR_COUNT],
                         ir_line_data_t *data) {
    for (unsigned i = 0; i < IR_LINE_SENSOR_COUNT; ++i)
        if (adc[i] > 4095u || classes[i] < LINE_UNKNOWN ||
            classes[i] > LINE_SENSOR_FAULT) return false;

    data->bytes[0] = (uint8_t)adc[0];
    data->bytes[1] = (uint8_t)((adc[0] >> 8) | (adc[1] << 4));
    data->bytes[2] = (uint8_t)(adc[1] >> 4);
    data->bytes[3] = (uint8_t)adc[2];
    data->bytes[4] = (uint8_t)((adc[2] >> 8) |
                               ((uint8_t)classes[0] << 4) |
                               ((uint8_t)classes[1] << 6));
    data->bytes[5] = (uint8_t)classes[2];
    return true;
}

bool ir_line_data_decode(const ir_line_data_t *data,
                         uint16_t adc[IR_LINE_SENSOR_COUNT],
                         line_class_t classes[IR_LINE_SENSOR_COUNT]) {
    if (data->bytes[5] & 0xfcu) return false; /* Reserved bits must be zero. */
    adc[0] = (uint16_t)(data->bytes[0] | ((uint16_t)(data->bytes[1] & 0x0fu) << 8));
    adc[1] = (uint16_t)((data->bytes[1] >> 4) | ((uint16_t)data->bytes[2] << 4));
    adc[2] = (uint16_t)(data->bytes[3] | ((uint16_t)(data->bytes[4] & 0x0fu) << 8));
    classes[0] = (line_class_t)((data->bytes[4] >> 4) & 0x03u);
    classes[1] = (line_class_t)((data->bytes[4] >> 6) & 0x03u);
    classes[2] = (line_class_t)(data->bytes[5] & 0x03u);
    return true;
}

uint8_t ir_line_black_pattern(const ir_line_data_t *data) {
    const uint8_t classes[] = {
        (uint8_t)((data->bytes[4] >> 4) & 0x03u),
        (uint8_t)((data->bytes[4] >> 6) & 0x03u),
        (uint8_t)(data->bytes[5] & 0x03u),
    };
    uint8_t pattern = 0;
    for (unsigned i = 0; i < IR_LINE_SENSOR_COUNT; ++i)
        if (classes[i] == LINE_BLACK) pattern |= (uint8_t)(1u << i);
    return pattern;
}

bool ir_line_steering_error(const ir_line_data_t *data, int8_t *error) {
    const uint8_t classes[] = {
        (uint8_t)((data->bytes[4] >> 4) & 0x03u),
        (uint8_t)((data->bytes[4] >> 6) & 0x03u),
        (uint8_t)(data->bytes[5] & 0x03u),
    };
    for (unsigned i = 0; i < IR_LINE_SENSOR_COUNT; ++i)
        if (classes[i] == LINE_UNKNOWN || classes[i] == LINE_SENSOR_FAULT)
            return false;
    switch (ir_line_black_pattern(data)) {
        case 1u: *error = -2; return true;
        case 3u: *error = -1; return true;
        case 2u: *error = 0; return true;
        case 6u: *error = 1; return true;
        case 4u: *error = 2; return true;
        default: return false; /* Lost, split or junction. */
    }
}
