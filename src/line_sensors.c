#include "line_sensors.h"

#include "hardware/adc.h"

#define SAMPLES_PER_SENSOR 8u

/* Physical sides, seen from behind the robot (see line_sensors.h). */
#if LINE_SENSORS_PAIR
#define SENSORS_READ 2u
static const unsigned int sensor_gpio[2] = {27u, 26u}; /* left, centre */
#else
#define SENSORS_READ 3u
static const unsigned int sensor_gpio[3] = {28u, 27u, 26u}; /* left, centre, right */
#endif

void line_sensors_init(void) {
    adc_init();
    for (unsigned int s = 0u; s < SENSORS_READ; ++s) {
        adc_gpio_init(sensor_gpio[s]);
    }
}

void line_sensors_read(uint16_t raw[LP_SENSORS]) {
    raw[LP_RIGHT] = 0u; /* overwritten when three sensors are read */
    for (unsigned int s = 0u; s < SENSORS_READ; ++s) {
        adc_select_input(sensor_gpio[s] - 26u); /* ADC0 is GP26 */
        uint32_t sum = 0u;
        for (unsigned int i = 0u; i < SAMPLES_PER_SENSOR; ++i) {
            sum += adc_read();
        }
        raw[s] = (uint16_t)(sum / SAMPLES_PER_SENSOR);
    }
}
