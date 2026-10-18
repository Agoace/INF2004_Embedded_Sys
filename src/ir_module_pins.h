#pragma once

/* Physical wiring supplied for the two TCRT5000 modules. */
#define LINE_IR_DIGITAL_GPIO       26u
#define LINE_IR_ANALOG_GPIO        27u /* ADC1: valid */

#define BARCODE_IR_DIGITAL_GPIO     6u
#define BARCODE_IR_ANALOG_GPIO      7u /* NOT ADC-capable on RP2040 */

/*
 * The fixed 4-pin Grove connection leaves barcode AO on GP7, which cannot be
 * sampled by the RP2040 ADC. Barcode decoding therefore uses DO/GP6 as a
 * one-bit waveform; tune the TCRT5000 comparator for a clean transition.
 * AO/GP7 is deliberately unused by this firmware.
 */

#define PICO_ADC_GPIO_MIN 26u
#define PICO_ADC_GPIO_MAX 28u
#define GPIO_IS_PICO_ADC(gpio) ((gpio) >= PICO_ADC_GPIO_MIN && (gpio) <= PICO_ADC_GPIO_MAX)
