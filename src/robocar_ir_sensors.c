#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "barcode_reader.h"
#include "ir_module_pins.h"
#include "line_detector.h"

#define LINE_SENSOR_COUNT 3u
#define LINE_SAMPLES_PER_SENSOR 8u
#define LINE_DEBUG_INTERVAL_US 500000u
#define BARCODE_CAPTURE_SAMPLES 4096u
#define BARCODE_DIGITAL_SAMPLE_PERIOD_US 50u
#define BARCODE_TRIGGER_ACTIVE_LEVEL 0u /* Change if the module's DO is active high. */

static const uint8_t LINE_ANALOG_GPIOS[LINE_SENSOR_COUNT] = {
    LINE_IR_LEFT_ANALOG_GPIO,
    LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
};

static uint16_t line_samples[LINE_SENSOR_COUNT][LINE_SAMPLES_PER_SENSOR];
static uint16_t line_means[LINE_SENSOR_COUNT];

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

static void sample_line(line_detector_t lines[LINE_SENSOR_COUNT]) {
    const uint32_t now_us = time_us_32();
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor) {
        adc_select_input(LINE_ANALOG_GPIOS[sensor] - PICO_ADC_GPIO_MIN);
        uint32_t sum = 0;
        for (size_t sample = 0; sample < LINE_SAMPLES_PER_SENSOR; ++sample) {
            const uint16_t value = adc_read();
            line_samples[sensor][sample] = value;
            sum += value;
        }
        line_means[sensor] = (uint16_t)(sum / LINE_SAMPLES_PER_SENSOR);
        line_event_t event;
        line_detector_process(&lines[sensor], line_samples[sensor],
                              LINE_SAMPLES_PER_SENSOR, now_us, &event);
    }
}

static void report_line(const line_detector_t lines[LINE_SENSOR_COUNT]) {
    uint8_t black_pattern = 0;
    uint8_t unknown_mask = 0;
    uint8_t fault_mask = 0;
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor) {
        const uint8_t bit = (uint8_t)(1u << sensor); /* L=1, C=2, R=4. */
        if (lines[sensor].current == LINE_BLACK) black_pattern |= bit;
        else if (lines[sensor].current == LINE_SENSOR_FAULT) fault_mask |= bit;
        else if (lines[sensor].current == LINE_UNKNOWN) unknown_mask |= bit;
    }

    /* Only contiguous black patterns give an unambiguous steering error. */
    int8_t steer_error = 0;
    bool steer_valid = unknown_mask == 0 && fault_mask == 0;
    switch (black_pattern) {
        case 1u: steer_error = -2; break; /* left only */
        case 3u: steer_error = -1; break; /* left + centre */
        case 2u: steer_error = 0; break;  /* centre only */
        case 6u: steer_error = 1; break;  /* centre + right */
        case 4u: steer_error = 2; break;  /* right only */
        default: steer_valid = false; break; /* lost, split or junction */
    }

    const char *status = fault_mask ? "sensor_fault" :
        unknown_mask ? "unknown" :
        black_pattern == 0u ? "lost" :
        black_pattern == 7u ? "junction" :
        black_pattern == 5u ? "split" : "tracking";
    printf("line ADC L/C/R=%u/%u/%u pattern=%u unknown=%u fault=%u status=%s",
           (unsigned)line_means[0], (unsigned)line_means[1],
           (unsigned)line_means[2], (unsigned)black_pattern,
           (unsigned)unknown_mask, (unsigned)fault_mask, status);
    if (steer_valid) printf(" steer=%d\n", steer_error);
    else printf(" steer=invalid\n");
}

static uint16_t barcode_digital_sample(void) {
    return gpio_get(BARCODE_IR_DIGITAL_GPIO) ? 4095u : 0u;
}

static void capture_barcode(barcode_reader_t *barcode,
                            line_detector_t lines[LINE_SENSOR_COUNT]) {
    barcode_reader_start(barcode, barcode_digital_sample());
    for (size_t i = 1; i < BARCODE_CAPTURE_SAMPLES; ++i) {
        busy_wait_us_32(BARCODE_DIGITAL_SAMPLE_PERIOD_US);
        barcode_reader_process_sample(barcode, barcode_digital_sample());
        if (i % 20u == 0u) sample_line(lines); /* Keep line sensing active. */
    }

    barcode_result_t result;
    barcode_reader_finish(barcode, &result);
    if (result.status == BARCODE_OK)
        printf("barcode OK: %s (%u edges)\n", result.text,
               (unsigned)result.edge_count);
    else
        printf("barcode status=%d (%u edges)\n", result.status,
               (unsigned)result.edge_count);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("RoboCar IR sensor firmware starting\n");

    adc_init();
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        adc_gpio_init(LINE_ANALOG_GPIOS[sensor]);
    gpio_init(BARCODE_IR_DIGITAL_GPIO);
    gpio_set_dir(BARCODE_IR_DIGITAL_GPIO, GPIO_IN);

    line_detector_t lines[LINE_SENSOR_COUNT];
    barcode_reader_t barcode;
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        line_detector_init(&lines[sensor], &LINE_CONFIG);
    barcode_reader_init(&barcode, &BARCODE_CONFIG);

    bool previous_trigger = gpio_get(BARCODE_IR_DIGITAL_GPIO) == BARCODE_TRIGGER_ACTIVE_LEVEL;
    uint32_t last_line_debug_us = 0;
    while (true) {
        const bool trigger = gpio_get(BARCODE_IR_DIGITAL_GPIO) == BARCODE_TRIGGER_ACTIVE_LEVEL;
        if (trigger && !previous_trigger) capture_barcode(&barcode, lines);
        previous_trigger = trigger;

        sample_line(lines);
        const uint32_t now_us = time_us_32();
        if ((uint32_t)(now_us - last_line_debug_us) >= LINE_DEBUG_INTERVAL_US) {
            last_line_debug_us = now_us;
            report_line(lines);
        }
        sleep_ms(1);
    }
}
