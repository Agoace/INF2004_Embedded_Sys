/* Single LINE-owner policy. No hardware, waits, allocation or action calls. */
#ifndef BARCODE_LIVE_PRODUCER_H
#define BARCODE_LIVE_PRODUCER_H
#include "barcode_window.h"
#include "barcode_live_source.h"

enum {
    BP_CONTROL = 100, BP_DISABLED, BP_CONTEXT, BP_CUT, BP_CALIBRATION,
    BP_RECORD_AGE, BP_MOTOR_AGE, BP_CLOCK
};
typedef struct {
    const barcode_cal_report_t *calibration;
    barcode_live_control_t control;
    uint32_t tag, now_us, now_ms, adc_us;
    uint32_t motor_us, left, right, motor_faults;
    uint32_t missed, dropped;
    uint16_t raw;
    bool control_ok, motor_ok;
} barcode_live_input_t;

typedef struct {
    barcode_live_record_t record;
    uint32_t index, reason, start_us, edge_us, frame_us, eligible_us;
    uint32_t control_age_ms, record_age_ms, spin_white, leading_white;
    float narrow_mm, wide_mm, leading_mm;
    uint16_t rise, fall;
} barcode_live_frame_t;

typedef struct {
    barcode_window_t window;
    barcode_live_control_t context;
    barcode_live_record_t slot;
    barcode_live_frame_t event;
    uint32_t tag, success_ms, progress_ms, sequence, cal_generation;
    uint32_t missed, dropped, run_epoch;
    uint32_t control_age_ms, max_control_age_ms, max_record_age_ms;
    uint32_t samples, publications, failures;
    bool seen, qualified, window_live, bound, occupied;
    bool counters_seen, event_pending, record_pending;
} barcode_live_producer_t;

void barcode_live_producer_observe(barcode_live_producer_t *state,
                                  const barcode_live_input_t *input);
/* Exact modulo-clock conversion, including independent us/ms timer wraps.
 * now_ms/now_us must name the same observation (phase0..999us). */
uint32_t barcode_live_ms_at(uint32_t now_ms, uint32_t now_us, uint32_t us);
#endif
