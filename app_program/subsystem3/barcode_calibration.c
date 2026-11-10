/*
 * barcode_calibration.c - bounded, integer-only GP28 spin evidence.
 */
#include "barcode_calibration.h"

#define CAL_ADC_MAX         4095u
#define CAL_SAMPLE_LIMIT    65535u
#define CAL_WHITE_MIN       120u
#define CAL_WHITE_MAX       220u
#define CAL_WHITE_SAMPLES   16u
#define CAL_BLACK_MIN       800u
#define CAL_BLACK_HOLD_US   10000u
#define CAL_MAX_GAP_US      5000u
#define CAL_FATAL_MASK      (BC_CAL_FAULT_RAW | BC_CAL_FAULT_TIME | \
                             BC_CAL_FAULT_FULL)

/* Start a new current-boot result; never retain old readiness on failure. */
void barcode_calibration_begin(barcode_calibration_t *state)
{
    uint32_t generation = state->report.generation + 1u;
    if (generation == 0u) {
        generation = 1u;
    }
    *state = (barcode_calibration_t){0};
    state->report.generation = generation;
    state->report.minimum = CAL_ADC_MAX;
    state->report.mode = BC_CAL_COLLECTING;
    state->report.rise = BC_CAL_RISE_FALLBACK;
    state->report.fall = BC_CAL_FALL_FALLBACK;
}

/* Observe only in task context during the existing calibration interval. */
void barcode_calibration_add(barcode_calibration_t *state, uint32_t us,
                             uint16_t raw)
{
    barcode_cal_report_t *report = &state->report;
    if (report->mode != BC_CAL_COLLECTING) {
        return;
    }
    if (report->samples == CAL_SAMPLE_LIMIT) {
        report->faults |= BC_CAL_FAULT_FULL;
        return;
    }
    ++report->samples;
    if (raw < report->minimum) {
        report->minimum = raw;
    }
    if (raw > report->maximum) {
        report->maximum = raw;
    }
    const uint32_t gap = us - state->previous_us;
    const bool continuous = state->have_previous && gap > 0u &&
                            gap <= CAL_MAX_GAP_US;
    if (state->have_previous) {
        if (gap > report->max_gap_us) {
            report->max_gap_us = gap;
        }
        if (!continuous) {
            report->faults |= BC_CAL_FAULT_TIME;
        }
    }
    state->have_previous = true;
    state->previous_us = us;
    if (raw == 0u || raw >= CAL_ADC_MAX) {
        report->faults |= BC_CAL_FAULT_RAW;
        state->previous_black = false;
        state->black_run_us = 0u;
        return;
    }
    if (raw >= CAL_WHITE_MIN && raw <= CAL_WHITE_MAX) {
        ++report->white_samples;
        state->white_sum += raw;
    }
    const bool black = raw >= CAL_BLACK_MIN;
    if (black) {
        ++report->black_samples;
        state->black_sum += raw;
    }
    if (black && state->previous_black && continuous) {
        state->black_run_us += gap;
    } else {
        state->black_run_us = 0u;
    }
    if (state->black_run_us > report->black_hold_us) {
        report->black_hold_us = state->black_run_us;
    }
    state->previous_black = black;
}

/* Finish does not authorize commands and does not change LINE calibration. */
void barcode_calibration_end(barcode_calibration_t *state, bool completed)
{
    barcode_cal_report_t *report = &state->report;
    if (report->mode != BC_CAL_COLLECTING) {
        return;
    }
    if (report->samples == 0u) {
        report->minimum = 0u;
    }
    if (report->white_samples > 0u) {
        report->white_mean = state->white_sum / report->white_samples;
    }
    if (report->black_samples > 0u) {
        report->black_mean = state->black_sum / report->black_samples;
    }
    if (report->white_samples < CAL_WHITE_SAMPLES) {
        report->faults |= BC_CAL_NO_WHITE;
    }
    if (report->black_hold_us < CAL_BLACK_HOLD_US) {
        report->faults |= BC_CAL_NO_BLACK;
    }
    if (!completed) {
        report->mode = BC_CAL_ABORTED;
    } else if ((report->faults & (CAL_FATAL_MASK | BC_CAL_NO_WHITE)) != 0u) {
        report->mode = BC_CAL_INVALID;
    } else if ((report->faults & BC_CAL_NO_BLACK) != 0u) {
        report->mode = BC_CAL_FALLBACK;
    } else {
        const uint32_t span = report->black_mean - report->white_mean;
        const uint32_t rise = span / 10u;
        const uint32_t fall = span * 6u / 100u;
        report->rise = report->white_mean + (rise > 32u ? rise : 32u);
        report->fall = report->white_mean + (fall > 16u ? fall : 16u);
        report->mode = BC_CAL_LEARNED;
    }
}
