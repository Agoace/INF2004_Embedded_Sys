#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "barcode_reader.h"
#include "ir_module_pins.h"
#include "ir_line_data.h"
#include "line_detector.h"

#define LINE_SENSOR_COUNT IR_LINE_SENSOR_COUNT
#define LINE_SAMPLES_PER_SENSOR 8u
#define LINE_SAMPLE_PERIOD_US 1000u
#define LINE_DEBUG_INTERVAL_US 500000u

static const uint8_t LINE_ANALOG_GPIOS[LINE_SENSOR_COUNT] = {
    LINE_IR_LEFT_ANALOG_GPIO,
    LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
};

static uint16_t line_samples[LINE_SENSOR_COUNT][LINE_SAMPLES_PER_SENSOR];
static uint16_t line_means[LINE_SENSOR_COUNT];
static ir_line_data_t latest_line_data;
static uint32_t line_pack_errors;
static int8_t latest_steering_error;
static bool latest_steering_valid;
static line_detector_t lines[LINE_SENSOR_COUNT];
static barcode_reader_t barcode_black_high;

static const line_detector_config_t LINE_CONFIG = {
    .black_threshold = 1200,
    .white_threshold = 2600,
    .debounce_buffers = 3,
    .fault_low = 10,
    .fault_high = 4085,
    .fault_buffers = 100,
};

static const barcode_reader_config_t BARCODE_BLACK_HIGH_CONFIG = {
    .black_level = true,
    /* A 1 ms sample can be slightly early; retain a one-sample bar or space. */
    .min_run_us = 250,
    .quiet_zone_us = 5000,
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
    line_class_t classes[LINE_SENSOR_COUNT];
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        classes[sensor] = lines[sensor].current;
    if (!ir_line_data_encode(line_means, classes, &latest_line_data)) {
        ++line_pack_errors;
        latest_steering_valid = false;
    } else {
        latest_steering_valid = ir_line_steering_error(&latest_line_data,
                                                       &latest_steering_error);
    }
}

static void report_line(const line_detector_t lines[LINE_SENSOR_COUNT]) {
    const uint8_t black_pattern = ir_line_black_pattern(&latest_line_data);
    uint8_t unknown_mask = 0;
    uint8_t fault_mask = 0;
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor) {
        const uint8_t bit = (uint8_t)(1u << sensor); /* L=1, C=2, R=4. */
        if (lines[sensor].current == LINE_SENSOR_FAULT) fault_mask |= bit;
        else if (lines[sensor].current == LINE_UNKNOWN) unknown_mask |= bit;
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
    if (latest_steering_valid) printf(" steer=%d\n", latest_steering_error);
    else printf(" steer=invalid\n");
}

static const char *barcode_status_name(barcode_status_t status) {
    switch (status) {
        case BARCODE_PARTIAL: return "partial";
        case BARCODE_DECODE_FAIL: return "decode_fail";
        case BARCODE_TRUNCATED: return "truncated";
        default: return "none";
    }
}

static void report_barcode(const barcode_result_t *result, uint32_t *rejected,
                           const char *black_level) {
    if (result->status == BARCODE_OK) {
        if (result->text[1] == '\0')
            printf("barcode char=%c black=%s\n", result->text[0], black_level);
        else
            printf("barcode text=%s black=%s\n", result->text, black_level);
    } else if (result->status == BARCODE_NO_BARCODE) {
        if (result->edge_count)
            printf("barcode short runs=%u black=%s\n",
                   (unsigned)result->edge_count, black_level);
    } else if (result->status != BARCODE_NO_BARCODE) {
        ++*rejected;
        printf("barcode reject reason=%s runs=%u black=%s\n",
               barcode_status_name(result->status), (unsigned)result->edge_count,
               black_level);
        if (result->status == BARCODE_DECODE_FAIL && result->failure_runs[0]) {
            printf("barcode symbol=%u durations_us=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\n",
                   (unsigned)(result->failure_offset / 10u),
                   (unsigned long)result->failure_runs[0],
                   (unsigned long)result->failure_runs[1],
                   (unsigned long)result->failure_runs[2],
                   (unsigned long)result->failure_runs[3],
                   (unsigned long)result->failure_runs[4],
                   (unsigned long)result->failure_runs[5],
                   (unsigned long)result->failure_runs[6],
                   (unsigned long)result->failure_runs[7],
                   (unsigned long)result->failure_runs[8]);
        }
    }
}

static void process_barcode(barcode_reader_t *black_high, bool level,
                            uint32_t now_us, uint32_t *rejected) {
    barcode_result_t result;
    if (barcode_reader_process_edge(black_high, level, now_us, &result))
        report_barcode(&result, rejected, "high");
    if (barcode_reader_poll(black_high, now_us, &result))
        report_barcode(&result, rejected, "high");
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("RoboCar IR sensor firmware starting\n");
    printf("line AO GP26/GP27/GP28; barcode DO GP7, black level=1\n");

    adc_init();
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        adc_gpio_init(LINE_ANALOG_GPIOS[sensor]);
    gpio_init(BARCODE_IR_DIGITAL_GPIO);
    gpio_set_dir(BARCODE_IR_DIGITAL_GPIO, GPIO_IN);

    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        line_detector_init(&lines[sensor], &LINE_CONFIG);
    const bool barcode_initial_level = gpio_get(BARCODE_IR_DIGITAL_GPIO);
    const uint32_t barcode_initial_time = time_us_32();
    barcode_reader_init(&barcode_black_high, &BARCODE_BLACK_HIGH_CONFIG,
                        barcode_initial_level, barcode_initial_time);

    uint32_t next_line_us = time_us_32();
    uint32_t last_line_debug_us = next_line_us;
    uint32_t missed_line_cycles = 0;
    uint32_t max_line_late_us = 0;
    uint32_t max_line_sample_us = 0;
    uint32_t rejected_barcodes = 0;
    while (true) {
        const uint32_t now_us = time_us_32();
        if ((int32_t)(now_us - next_line_us) >= 0) {
            const uint32_t late = now_us - next_line_us;
            missed_line_cycles += late / LINE_SAMPLE_PERIOD_US;
            if (late > max_line_late_us) max_line_late_us = late;
            next_line_us += (late / LINE_SAMPLE_PERIOD_US + 1u) * LINE_SAMPLE_PERIOD_US;
            const uint32_t start_us = time_us_32();
            sample_line(lines);
            const uint32_t duration_us = time_us_32() - start_us;
            if (duration_us > max_line_sample_us) max_line_sample_us = duration_us;
            /* GP7 is sampled with the three line sensors once per 1 ms cycle. */
            process_barcode(&barcode_black_high, gpio_get(BARCODE_IR_DIGITAL_GPIO), time_us_32(),
                            &rejected_barcodes);
        }
        if ((uint32_t)(now_us - last_line_debug_us) >= LINE_DEBUG_INTERVAL_US) {
            last_line_debug_us = now_us;
            report_line(lines);
            printf("timing missed_line=%lu max_late_us=%lu max_line_us=%lu line_pack_err=%lu barcode_do=%u barcode_reject=%lu\n",
                   (unsigned long)missed_line_cycles, (unsigned long)max_line_late_us,
                   (unsigned long)max_line_sample_us,
                   (unsigned long)line_pack_errors,
                   (unsigned)gpio_get(BARCODE_IR_DIGITAL_GPIO),
                   (unsigned long)rejected_barcodes);
        }
        tight_loop_contents();
    }
}
