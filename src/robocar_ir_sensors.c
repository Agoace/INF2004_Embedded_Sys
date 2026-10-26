#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#include "barcode_reader.h"
#include "ir_module_pins.h"
#include "ir_line_data.h"
#include "line_detector.h"

#define LINE_SENSOR_COUNT IR_LINE_SENSOR_COUNT
#define LINE_SAMPLES_PER_SENSOR 8u
#define LINE_SAMPLE_PERIOD_US 1000u
#define LINE_DEBUG_INTERVAL_US 500000u
#define BARCODE_EDGE_QUEUE_SIZE 128u
#define BARCODE_BLACK_LEVEL 0u /* Change if the module's DO is active high. */

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
static barcode_reader_t barcode;

typedef struct {
    uint32_t timestamp_us;
    bool level;
} barcode_edge_t;

static barcode_edge_t barcode_edges[BARCODE_EDGE_QUEUE_SIZE];
static volatile uint16_t barcode_edge_head;
static volatile uint16_t barcode_edge_tail;
static volatile uint32_t barcode_edges_dropped;

static const line_detector_config_t LINE_CONFIG = {
    .black_threshold = 1200,
    .white_threshold = 2600,
    .debounce_buffers = 3,
    .fault_low = 10,
    .fault_high = 4085,
    .fault_buffers = 100,
};

static const barcode_reader_config_t BARCODE_CONFIG = {
    .black_level = BARCODE_BLACK_LEVEL,
    .min_run_us = 80,
    .quiet_zone_us = 5000,
};

static void barcode_gpio_irq(uint gpio, uint32_t events) {
    if (gpio != BARCODE_IR_DIGITAL_GPIO) return;
    const uint32_t edges = events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    if (!edges) return;
    if (edges == (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        ++barcode_edges_dropped; /* The order of two pending edges is unknown. */
        return;
    }
    const uint16_t next = (uint16_t)((barcode_edge_head + 1u) % BARCODE_EDGE_QUEUE_SIZE);
    if (next == barcode_edge_tail) {
        ++barcode_edges_dropped;
        return;
    }
    barcode_edges[barcode_edge_head].timestamp_us = time_us_32();
    barcode_edges[barcode_edge_head].level = edges == GPIO_IRQ_EDGE_RISE;
    barcode_edge_head = next;
}

static bool take_barcode_edge(barcode_edge_t *edge) {
    const uint32_t flags = save_and_disable_interrupts();
    const bool available = barcode_edge_tail != barcode_edge_head;
    if (available) {
        *edge = barcode_edges[barcode_edge_tail];
        barcode_edge_tail = (uint16_t)((barcode_edge_tail + 1u) % BARCODE_EDGE_QUEUE_SIZE);
    }
    restore_interrupts(flags);
    return available;
}

static uint32_t take_dropped_edges(void) {
    const uint32_t flags = save_and_disable_interrupts();
    const uint32_t dropped = barcode_edges_dropped;
    barcode_edges_dropped = 0;
    if (dropped) barcode_edge_tail = barcode_edge_head;
    restore_interrupts(flags);
    return dropped;
}

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

static void report_barcode(const barcode_result_t *result, uint32_t *rejected) {
    if (result->status == BARCODE_OK) {
        if (result->text[1] == '\0') printf("barcode char=%c\n", result->text[0]);
        else printf("barcode text=%s\n", result->text);
    } else if (result->status != BARCODE_NO_BARCODE) {
        ++*rejected;
    }
}

static void process_barcode(barcode_reader_t *barcode, uint32_t *rejected,
                            uint32_t *dropped_total) {
    const uint32_t dropped = take_dropped_edges();
    if (dropped) {
        *dropped_total += dropped;
        barcode_reader_init(barcode, &BARCODE_CONFIG,
                            gpio_get(BARCODE_IR_DIGITAL_GPIO), time_us_32());
        return; /* A scan with missing transitions cannot be decoded. */
    }
    barcode_edge_t edge;
    barcode_result_t result;
    while (take_barcode_edge(&edge)) {
        if (barcode_reader_process_edge(barcode, edge.level, edge.timestamp_us, &result))
            report_barcode(&result, rejected);
    }
    if (barcode_reader_poll(barcode, time_us_32(), &result))
        report_barcode(&result, rejected);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("RoboCar IR sensor firmware starting\n");
    printf("line AO GP26/GP27/GP28; barcode DO GP7, black level=%u\n",
           (unsigned)BARCODE_BLACK_LEVEL);

    adc_init();
    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        adc_gpio_init(LINE_ANALOG_GPIOS[sensor]);
    gpio_init(BARCODE_IR_DIGITAL_GPIO);
    gpio_set_dir(BARCODE_IR_DIGITAL_GPIO, GPIO_IN);

    for (size_t sensor = 0; sensor < LINE_SENSOR_COUNT; ++sensor)
        line_detector_init(&lines[sensor], &LINE_CONFIG);
    barcode_reader_init(&barcode, &BARCODE_CONFIG,
                        gpio_get(BARCODE_IR_DIGITAL_GPIO), time_us_32());
    gpio_set_irq_enabled_with_callback(BARCODE_IR_DIGITAL_GPIO,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, barcode_gpio_irq);

    uint32_t next_line_us = time_us_32();
    uint32_t last_line_debug_us = next_line_us;
    uint32_t missed_line_cycles = 0;
    uint32_t max_line_late_us = 0;
    uint32_t max_line_sample_us = 0;
    uint32_t rejected_barcodes = 0;
    uint32_t dropped_barcode_edges = 0;
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
        }
        process_barcode(&barcode, &rejected_barcodes, &dropped_barcode_edges);
        if ((uint32_t)(now_us - last_line_debug_us) >= LINE_DEBUG_INTERVAL_US) {
            last_line_debug_us = now_us;
            report_line(lines);
            printf("timing missed_line=%lu max_late_us=%lu max_line_us=%lu line_pack_err=%lu barcode_reject=%lu barcode_dropped=%lu\n",
                   (unsigned long)missed_line_cycles, (unsigned long)max_line_late_us,
                   (unsigned long)max_line_sample_us,
                   (unsigned long)line_pack_errors,
                   (unsigned long)rejected_barcodes, (unsigned long)dropped_barcode_edges);
        }
        tight_loop_contents();
    }
}
