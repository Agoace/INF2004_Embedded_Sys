#pragma once

/* Left, centre and right analogue line sensors: ADC0, ADC1 and ADC2. */
#define LINE_IR_LEFT_ANALOG_GPIO    26u
#define LINE_IR_CENTRE_ANALOG_GPIO  27u
#define LINE_IR_RIGHT_ANALOG_GPIO   28u

#define BARCODE_IR_DIGITAL_GPIO     7u

/*
 * Connect the barcode module's DO signal to GP7 and leave its AO signal
 * disconnected. GP7 is a digital input, not an RP2040 ADC input. Keep GP6
 * available for the IMU data-ready signal. Tune the barcode module's
 * comparator for a clean one-bit waveform.
 */

#define PICO_ADC_GPIO_MIN 26u
#define PICO_ADC_GPIO_MAX 28u
#define GPIO_IS_PICO_ADC(gpio) ((gpio) >= PICO_ADC_GPIO_MIN && (gpio) <= PICO_ADC_GPIO_MAX)
