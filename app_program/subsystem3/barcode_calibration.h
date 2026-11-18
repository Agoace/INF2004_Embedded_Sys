/*
 * barcode_calibration.h - GP28 evidence from the existing LINE spin.
 */
#ifndef BARCODE_CALIBRATION_H
#define BARCODE_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define BC_CAL_RISE_FALLBACK 320u
#define BC_CAL_FALL_FALLBACK 240u

typedef enum {
    BC_CAL_IDLE = 0,
    BC_CAL_COLLECTING,
    BC_CAL_LEARNED,
    BC_CAL_FALLBACK,
    BC_CAL_INVALID,
    BC_CAL_ABORTED
} barcode_cal_mode_t;

#define BC_CAL_FAULT_RAW   0x01u
#define BC_CAL_FAULT_TIME  0x02u
#define BC_CAL_FAULT_FULL  0x04u
#define BC_CAL_NO_WHITE   0x08u
#define BC_CAL_NO_BLACK   0x10u

/* All fields are fixed-width board values. No actuation permission is implied
 * by LEARNED/FALLBACK: live reading still needs qualified leading white,
 * a complete window and a verified sensor-offset bound. */
typedef struct {
    uint32_t generation;
    uint32_t samples;
    uint32_t minimum;
    uint32_t maximum;
    uint32_t white_samples;
    uint32_t black_samples;
    uint32_t white_mean;
    uint32_t black_mean;
    uint32_t black_hold_us;
    uint32_t max_gap_us;
    uint32_t rise;
    uint32_t fall;
    uint32_t mode;
    uint32_t faults;
} barcode_cal_report_t;

typedef struct {
    barcode_cal_report_t report;
    uint32_t previous_us;
    uint32_t white_sum;
    uint32_t black_sum;
    uint32_t black_run_us;
    bool have_previous;
    bool previous_black;
} barcode_calibration_t;

/* Zero-initialize the state before its first begin. No allocation or I/O. */
void barcode_calibration_begin(barcode_calibration_t *state);
void barcode_calibration_add(barcode_calibration_t *state, uint32_t us,
                             uint16_t raw);
void barcode_calibration_end(barcode_calibration_t *state, bool completed);

#endif
