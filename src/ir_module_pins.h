#pragma once

/* Reserved for the line-tracing branch: ADC0, ADC1 and ADC2. */
#define LINE_IR_LEFT_ANALOG_GPIO    26u
#define LINE_IR_CENTRE_ANALOG_GPIO  27u
#define LINE_IR_RIGHT_ANALOG_GPIO   28u

/* Barcode module DO connects to GP7. Its AO signal is not used. */
#define BARCODE_IR_DIGITAL_GPIO     7u
