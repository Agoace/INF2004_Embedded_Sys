#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    LINE_UNKNOWN = 0,
    LINE_BLACK,
    LINE_WHITE,
    LINE_SENSOR_FAULT,
} line_class_t;

typedef struct {
    uint16_t black_threshold; /* Mean ADC value at/below this is black. */
    uint16_t white_threshold; /* Mean ADC value at/above this is white. */
    uint8_t debounce_buffers;
    uint16_t fault_low;       /* Sustained values below this indicate a fault. */
    uint16_t fault_high;      /* Sustained values above this indicate a fault. */
    uint16_t fault_buffers;
} line_detector_config_t;

typedef struct {
    line_detector_config_t config;
    line_class_t current;
    line_class_t candidate;
    uint16_t candidate_count;
    uint16_t fault_count;
} line_detector_t;

typedef struct {
    uint32_t timestamp_us;
    line_class_t classification;
    uint16_t mean_adc;
} line_event_t;

void line_detector_init(line_detector_t *detector, const line_detector_config_t *config);
bool line_detector_process(line_detector_t *detector, const uint16_t *samples,
                           size_t sample_count, uint32_t timestamp_us,
                           line_event_t *event);
