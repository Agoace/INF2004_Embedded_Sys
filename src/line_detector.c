#include "line_detector.h"

static line_class_t classify(const line_detector_t *detector, uint16_t mean) {
    if (mean <= detector->config.black_threshold) return LINE_BLACK;
    if (mean >= detector->config.white_threshold) return LINE_WHITE;
    return LINE_UNKNOWN;
}

void line_detector_init(line_detector_t *detector, const line_detector_config_t *config) {
    detector->config = *config;
    detector->current = LINE_UNKNOWN;
    detector->candidate = LINE_UNKNOWN;
    detector->candidate_count = 0;
    detector->fault_count = 0;
}

bool line_detector_process(line_detector_t *detector, const uint16_t *samples,
                           size_t sample_count, uint32_t timestamp_us,
                           line_event_t *event) {
    if (sample_count == 0) return false;

    uint32_t sum = 0;
    for (size_t i = 0; i < sample_count; ++i) sum += samples[i];
    const uint16_t mean = (uint16_t)(sum / sample_count);
    const bool fault_level = mean <= detector->config.fault_low ||
                             mean >= detector->config.fault_high;

    if (!fault_level) detector->fault_count = 0;
    else if (detector->fault_count < detector->config.fault_buffers)
        ++detector->fault_count; /* Saturate: a persistent fault must not wrap. */
    const line_class_t next = detector->fault_count >= detector->config.fault_buffers
        ? LINE_SENSOR_FAULT : classify(detector, mean);

    if (next == detector->current) {
        detector->candidate_count = 0;
        return false;
    }
    if (next != detector->candidate) {
        detector->candidate = next;
        detector->candidate_count = 1;
        return false;
    }
    if (++detector->candidate_count < detector->config.debounce_buffers) return false;

    detector->current = next;
    detector->candidate_count = 0;
    event->timestamp_us = timestamp_us;
    event->classification = next;
    event->mean_adc = mean;
    return true;
}
