#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "ir_line_data.h"

int main(void)
{
    const uint16_t adc[IR_LINE_SENSOR_COUNT] = {0u, 4095u, 0xabcu};
    const line_class_t classes[IR_LINE_SENSOR_COUNT] = {
        LINE_UNKNOWN, LINE_BLACK, LINE_SENSOR_FAULT,
    };
    ir_line_data_t packet;
    uint16_t decoded_adc[IR_LINE_SENSOR_COUNT];
    line_class_t decoded_classes[IR_LINE_SENSOR_COUNT];
    assert(sizeof(packet) == 6u);
    assert(ir_line_data_encode(adc, classes, &packet));
    assert(ir_line_black_pattern(&packet) == 2u);
    int8_t steering = 99;
    assert(!ir_line_steering_error(&packet, &steering));
    const line_class_t centered[IR_LINE_SENSOR_COUNT] = {
        LINE_WHITE, LINE_BLACK, LINE_WHITE,
    };
    assert(ir_line_data_encode(adc, centered, &packet));
    assert(ir_line_steering_error(&packet, &steering) && steering == 0);
    assert(ir_line_data_encode(adc, classes, &packet));
    assert(ir_line_data_decode(&packet, decoded_adc, decoded_classes));
    assert(memcmp(adc, decoded_adc, sizeof(adc)) == 0);
    for (unsigned int i = 0; i < IR_LINE_SENSOR_COUNT; ++i) {
        assert(classes[i] == decoded_classes[i]);
    }
    packet.bytes[5] |= 0x80u;
    assert(!ir_line_data_decode(&packet, decoded_adc, decoded_classes));
    const uint16_t out_of_range[IR_LINE_SENSOR_COUNT] = {4096u, 0u, 0u};
    assert(!ir_line_data_encode(out_of_range, classes, &packet));
    return 0;
}
