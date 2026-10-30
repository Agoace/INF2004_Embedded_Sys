/**
 * @file line_adc.h
 * @brief Hardware seam for the three analogue line sensors.
 *
 * The firmware implements this in line_adc_pico.c. The host tests replace it
 * with scripted readings, so the producer, the health check and the manager all
 * run unchanged on the host.
 *
 * Readings are raw 12-bit ADC values in **physical** left, centre, right order.
 * The pin mapping is deliberately kept out of this header and out of
 * `ir_module_pins.h`, because that shared header is also used by the standalone
 * IR diagnostic targets and must not change under them.
 */
#ifndef LINE_ADC_H
#define LINE_ADC_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_ADC_SENSORS 3u

/** Bring up the ADC and the three sensor pins. */
bool line_adc_init(void);

/**
 * @brief Read one averaged sample per sensor, in physical L/C/R order.
 *
 * @param raw Receives LINE_ADC_SENSORS raw 12-bit readings.
 */
void line_adc_read(uint16_t raw[LINE_ADC_SENSORS]);

#endif /* LINE_ADC_H */
