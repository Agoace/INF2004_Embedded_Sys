#include "line_sensors.h"

#include "hardware/adc.h"
#include "ir_module_pins.h"

#define SAMPLES_PER_SENSOR 8u

static const unsigned int sensor_gpio[LP_SENSORS] = {
    LINE_IR_LEFT_ANALOG_GPIO,
    LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
};

void line_sensors_init(void) {
    adc_init();
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_gpio_init(sensor_gpio[s]);
    }
}

void line_sensors_read(uint16_t raw[LP_SENSORS]) {
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_select_input(sensor_gpio[s] - PICO_ADC_GPIO_MIN);
        uint32_t sum = 0u;
        for (unsigned int i = 0u; i < SAMPLES_PER_SENSOR; ++i) {
            sum += adc_read();
        }
        raw[s] = (uint16_t)(sum / SAMPLES_PER_SENSOR);
    }
}
