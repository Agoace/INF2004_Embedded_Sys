#include "line_sensors.h"

#include "hardware/adc.h"

#define SAMPLES_PER_SENSOR 8u

/* Physical sides, seen from behind the robot (operator, 2026-10-27).
 * src/ir_module_pins.h names GP26 "left" and GP28 "right"; on this robot
 * it is the other way round. */
#define LINE_SENSOR_LEFT_GPIO 28u
#define LINE_SENSOR_CENTRE_GPIO 27u
#define LINE_SENSOR_RIGHT_GPIO 26u

static const unsigned int sensor_gpio[LP_SENSORS] = {
    LINE_SENSOR_LEFT_GPIO,
    LINE_SENSOR_CENTRE_GPIO,
    LINE_SENSOR_RIGHT_GPIO,
};

void line_sensors_init(void) {
    adc_init();
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_gpio_init(sensor_gpio[s]);
    }
}

void line_sensors_read(uint16_t raw[LP_SENSORS]) {
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_select_input(sensor_gpio[s] - 26u); /* ADC0 is GP26 */
        uint32_t sum = 0u;
        for (unsigned int i = 0u; i < SAMPLES_PER_SENSOR; ++i) {
            sum += adc_read();
        }
        raw[s] = (uint16_t)(sum / SAMPLES_PER_SENSOR);
    }
}
