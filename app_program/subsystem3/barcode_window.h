/* Pure large-print course window reader. Single owner; never actuates. */
#ifndef BARCODE_WINDOW_H
#define BARCODE_WINDOW_H
#include "barcode_calibration.h"

#define BW_ADC_CAPACITY 64u
#define BW_MOTOR_CAPACITY 8u
#define BW_ELEMENTS 29u
#define BW_LEVELS 17u /* 0..4096 counts in 256-count steps */
#define BW_PEAKS 3u
#define BW_STEP_MAX 32u

/* Leading white before the first bar. Operator 2026-11-19: 40 -> 30 mm.
 * 30 mm accepts the known FC2 false start (see test_barcode_quiet_real). */
#define QUIET_MIN_MM 30.0f

typedef enum {
    BW_WAIT = 0, BW_OK, BW_CALIBRATION, BW_LOSS, BW_MOTION,
    BW_START, BW_WIDTH, BW_GAP, BW_EXTENT, BW_TAIL, BW_NOT_LETTER
} barcode_window_reason_t;

typedef struct {
    uint32_t counts_per_m;
    uint32_t sample_gap_us;
    uint32_t motor_gap_us;
    int16_t sensor_forward_mm;
    int16_t sensor_lateral_mm; /* Positive is left of the drive-centre line. */
    uint16_t track_mm;
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
    bool held; /* Accepted from the held letter; end star/tail not proven. */
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
    float centre_x_mm;
    float centre_y_mm;
    float heading_rad;
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
    /* WINDOW local-midpoint edges (see barcode_window.c). */
    bool started;        /* The first edge position is set. */
    bool closing;        /* Last bar: wait for its falling midpoint. */
    bool mode_set;       /* The first bar chose the edge mode. */
    bool mid;            /* Local-midpoint edges (else fixed levels). */
    uint8_t bars;        /* Bars detected in this window. */
    uint8_t shifts;      /* Faint first marks dropped. */
    uint16_t prev_raw;   /* Previous processed sample. */
    uint16_t run_peak;   /* Highest raw in the current black run. */
    uint16_t run_valley; /* Lowest raw in the current white run. */
    uint16_t ref_peak;   /* Peak of the last black run. */
    uint16_t ref_valley; /* Valley of the last white run (floor white). */
    uint16_t bar_peak[BW_PEAKS];
    float prev_mm;
    float det_mm;        /* Position of the last detected edge. */
    uint32_t fall_set, rise_set;
    float fall_mm[BW_LEVELS]; /* First crossing of level b*256 counts. */
    float rise_mm[BW_LEVELS];
    /* Blip grace: SEEK ignores the samples from freeze_us (to thaw_us);
     * the white run, mean and count wait. Frozen travel is not white. */
    uint32_t freeze_us;
    uint32_t thaw_us;
    float seek_mm; /* Last SEEK sample position; frozen travel offset. */
    bool frozen;   /* Open interval: no thaw yet. */
    bool thaw;     /* Closed interval waiting for its first later sample. */
    uint8_t white_lost; /* BW_WHITE_LOST_*; the owner reads and clears. */
    /* Held letter (operator 2026-11-20): parts 1..19 passed the letter
     * checks; a later failure accepts this letter instead of rejecting. */
    bool held;
    bool held_tried; /* Midpoint mode: provisional part 19 was tested. */
    char held_letter;
    uint32_t held_edge_us;
    uint32_t held_frame_us;
    uint32_t held_frame_left;
    uint32_t held_frame_right;
} barcode_window_t;

#define BW_WHITE_LOST_BLACK 0x01u /* Unadmitted ink ended a white run. */
#define BW_WHITE_LOST_OTHER 0x02u /* Loss, motion or grey ended it. */
#define BW_WHITE_LOST_BLIP 0x04u  /* Not white just after a blip grace. */
/* Blip grace owner calls. freeze: ADC samples at or after us wait.
 * thaw: samples from us on count again. Only SEEK uses them. */
void barcode_window_freeze(barcode_window_t *state, uint32_t us);
void barcode_window_thaw(barcode_window_t *state, uint32_t us);
/* True in SEEK (before the first bar; 1u is SEEK in the .c). */
static inline bool barcode_window_seeking(const barcode_window_t *s)
{
    return s->phase == 1u;
}
/* True when a SEEK white run is in progress. */
static inline bool barcode_window_white_run(const barcode_window_t *s)
{
    return barcode_window_seeking(s) && s->white_started;
}

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
