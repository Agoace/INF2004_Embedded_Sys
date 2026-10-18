#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "barcode_reader.h"
#include "ir_module_pins.h"
#include "line_detector.h"

#define LINE_BUFFER_SAMPLES 256u
#define LINE_DEBUG_INTERVAL_US 500000u
#define BARCODE_CAPTURE_SAMPLES 4096u
#define BARCODE_DIGITAL_SAMPLE_PERIOD_US 50u
#define BARCODE_TRIGGER_ACTIVE_LEVEL 0u /* Change if the module's DO is active high. */

static uint16_t line_samples[LINE_BUFFER_SAMPLES];

static const line_detector_config_t LINE_CONFIG = {
    .black_threshold = 1200,
    .white_threshold = 2600,
    .debounce_buffers = 3,
    .fault_low = 10,
    .fault_high = 4085,
    .fault_buffers = 100,
};

static const barcode_reader_config_t BARCODE_CONFIG = {
    .threshold = 2048,
    .hysteresis = 80,
    .min_run_samples = 2,
};

static void select_line_adc(void) {
    adc_select_input(LINE_IR_ANALOG_GPIO - PICO_ADC_GPIO_MIN);
}

static uint16_t line_mean_adc(void) {
    uint32_t sum = 0;
    for (size_t i = 0; i < LINE_BUFFER_SAMPLES; ++i) sum += line_samples[i];
    return (uint16_t)(sum / LINE_BUFFER_SAMPLES);
}

static uint16_t barcode_digital_sample(void) {
    return gpio_get(BARCODE_IR_DIGITAL_GPIO) ? 4095u : 0u;
}

static void capture_barcode(barcode_reader_t *barcode) {
    barcode_reader_start(barcode, barcode_digital_sample());
    for (size_t i = 1; i < BARCODE_CAPTURE_SAMPLES; ++i) {
        busy_wait_us_32(BARCODE_DIGITAL_SAMPLE_PERIOD_US);
        barcode_reader_process_sample(barcode, barcode_digital_sample());
    }

    barcode_result_t result;
    barcode_reader_finish(barcode, &result);
    if (result.status == BARCODE_OK)
        printf("barcode OK: %s (%u edges)\n", result.text, result.edge_count);
    else
        printf("barcode status=%d (%u edges)\n", result.status, result.edge_count);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("RoboCar IR sensor firmware starting\n");

    adc_init();
    adc_gpio_init(LINE_IR_ANALOG_GPIO);             /* First IR AO: GP27 / ADC1. */
    gpio_init(LINE_IR_DIGITAL_GPIO);
    gpio_set_dir(LINE_IR_DIGITAL_GPIO, GPIO_IN);
    gpio_init(BARCODE_IR_DIGITAL_GPIO);
    gpio_set_dir(BARCODE_IR_DIGITAL_GPIO, GPIO_IN);

    line_detector_t line;
    barcode_reader_t barcode;
    line_detector_init(&line, &LINE_CONFIG);
    barcode_reader_init(&barcode, &BARCODE_CONFIG);

    bool previous_trigger = gpio_get(BARCODE_IR_DIGITAL_GPIO) == BARCODE_TRIGGER_ACTIVE_LEVEL;
    uint32_t last_line_debug_us = 0;
    while (true) {
        const bool trigger = gpio_get(BARCODE_IR_DIGITAL_GPIO) == BARCODE_TRIGGER_ACTIVE_LEVEL;
        if (trigger && !previous_trigger) capture_barcode(&barcode);
        previous_trigger = trigger;

        select_line_adc();
        for (size_t i = 0; i < LINE_BUFFER_SAMPLES; ++i)
            line_samples[i] = adc_read();

        const uint32_t now_us = time_us_32();
        line_event_t event;
        if (line_detector_process(&line, line_samples, LINE_BUFFER_SAMPLES,
                                  now_us, &event)) {
            printf("line class=%d mean=%u\n", event.classification, event.mean_adc);
        }
        if ((uint32_t)(now_us - last_line_debug_us) >= LINE_DEBUG_INTERVAL_US) {
            last_line_debug_us = now_us;
            printf("line raw mean=%u DO=%u\n", line_mean_adc(),
                   gpio_get(LINE_IR_DIGITAL_GPIO));
        }
        sleep_ms(5);
    }
}
