/* Pure large-print course window reader. Single owner; never actuates. */
#ifndef BARCODE_WINDOW_H
#define BARCODE_WINDOW_H
#include "barcode_calibration.h"

#define BW_ADC_CAPACITY 64u
#define BW_MOTOR_CAPACITY 8u
#define BW_ELEMENTS 29u
#define BW_STEP_MAX 32u

typedef enum {
    BW_WAIT = 0, BW_OK, BW_CALIBRATION, BW_LOSS, BW_MOTION,
    BW_START, BW_WIDTH, BW_GAP, BW_EXTENT, BW_TAIL, BW_NOT_LETTER
} barcode_window_reason_t;

typedef struct {
    uint32_t counts_per_m;
    uint32_t sample_gap_us;
    uint32_t motor_gap_us;
} barcode_window_config_t;
extern const barcode_window_config_t barcode_window_default_config;

typedef struct {
    uint32_t us;
    uint16_t raw;
    uint16_t flags;
} barcode_window_adc_t;

typedef struct {
    uint32_t us;
    uint32_t left;
    uint32_t right;
    float mm;
} barcode_window_motor_t;

typedef struct {
    uint32_t serial;
    uint32_t reason;
    uint32_t start_us; /* FIRST BAR edge, for coherent context admission. */
    uint32_t edge_us; /* ADC-observed final black->white transition. */
    /* Original upper motor brackets, not fabricated/interpolated counts.
     * frame_us/counts are a conservative point AT OR AFTER the final edge. */
    uint32_t frame_us;
    uint32_t frame_left;
    uint32_t frame_right;
    uint32_t eligible_us; /* ADC time after the short white confirmation. */
    uint32_t read_us;
    uint32_t read_left;
    uint32_t read_right;
    float narrow_mm;
    float wide_mm;
    float leading_mm;
    uint16_t rise;
    uint16_t fall;
    char letter;
    uint8_t direction; /* 1 print order, 2 reverse print; motion is forward. */
    uint8_t mapped_action; /* Observation only: 0/1/2/3/4 = NONE/L/R/S/U. */
    bool accepted;
} barcode_window_result_t;

typedef struct {
    barcode_cal_report_t calibration;
    barcode_window_config_t config;
    barcode_window_result_t result;
    barcode_window_adc_t adc[BW_ADC_CAPACITY];
    barcode_window_motor_t motors[BW_MOTOR_CAPACITY];
    float runs[BW_ELEMENTS];
    uint32_t serial;
    uint32_t previous_adc_us;
    uint32_t start_us;
    uint32_t edge_us;
    uint32_t frame_us;
    uint32_t frame_left;
    uint32_t frame_right;
    float white_start;
    float white_end;
    float transition_start;
    float white_mean;
    float edge_mm;
    float first_mm;
    float leading_mm;
    float tail_start;
    float narrow;
    float wide;
    float bias;
    float space;
    float tail_narrow;
    uint16_t rise;
    uint16_t fall;
    uint16_t white_count;
    uint8_t head;
    uint8_t queued;
    uint8_t motor_count;
    uint8_t elements;
    uint8_t phase;
    uint8_t direction;
    char letter;
    bool have_adc;
    bool white_started;
    bool transitioning;
    bool black;
} barcode_window_t;

/* Valid calibration/configuration is required. No old samples survive begin.
 * All methods must be called by ONE task or externally serialized owner.
 * Future cross-task delivery needs its own queue; this is not an SPSC API. */
bool barcode_window_begin(barcode_window_t *state,
                          const barcode_cal_report_t *calibration,
                          const barcode_window_config_t *config);
void barcode_window_adc(barcode_window_t *state, uint32_t us, uint16_t raw,
                        uint16_t flags);
void barcode_window_motor(barcode_window_t *state, uint32_t us,
                          uint32_t left, uint32_t right, uint32_t faults);
/* Processes at most min(budget,32) samples. Never extrapolates position. */
unsigned int barcode_window_step(barcode_window_t *state, unsigned int budget);
/* Accepted report is immutable until explicit rearm. Repeated reads do not
 * create new events. Rejections require fresh leading white, no run repair. */
bool barcode_window_rearm(barcode_window_t *state);
/* Includes a latched complete window. False leaves optional time unchanged. */
bool barcode_window_candidate(const barcode_window_t *state,
                              uint32_t *first_bar_us);
uint8_t barcode_window_table_entry(unsigned int index);
bool barcode_window_table_confirmed(void);
uint32_t barcode_window_mapping_id(void);
#endif
