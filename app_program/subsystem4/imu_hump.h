/* Pure hump events from the roll height. Report only: no SDK, kernel,
 * packed protocol or driving decision. The course humps hit the RIGHT
 * drive wheel (operator fact), so RIGHT is the primary side. A LEFT event
 * is an unproven bonus, reported only through a side flag. */
#ifndef IMU_HUMP_H
#define IMU_HUMP_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_HUMP_MEAN_SAMPLES 10u
#define IMU_HUMP_PEAK_NONE 0xFFFFu   /* Unavailable 0.1 mm value. */
#define IMU_HUMP_PEAK_MAX 0xFFFEu
#define IMU_HUMP_NUMBER_MAX 127u
#define IMU_HUMP_DETAIL_LEFT 0x80u
#define IMU_HUMP_GATED_MAX 255u

/* Run flags. The bit values are the summary v3 hump_flags byte. */
#define IMU_HUMP_FLAG_IMPLEMENTED 0x01u
#define IMU_HUMP_FLAG_FAULT 0x02u    /* Invalid or gap sample in the run. */
#define IMU_HUMP_FLAG_OPEN 0x04u     /* An event was still open at STOP. */
#define IMU_HUMP_FLAG_CURVE 0x08u    /* A curve gate froze an event peak. */
#define IMU_HUMP_FLAG_LONG 0x10u     /* An event exceeded maximum travel. */
#define IMU_HUMP_FLAG_LEFT 0x20u     /* The highest event is LEFT. */
#define IMU_HUMP_FLAGS_MASK 0x3Fu

typedef enum {
    IMU_HUMP_IDLE = 0,
    IMU_HUMP_CANDIDATE = 1,
    IMU_HUMP_ACTIVE = 2,
    IMU_HUMP_LONG = 3
} imu_hump_state_t;

typedef struct {
    float height_mm;      /* Signed source height; RIGHT wheel up is +. */
    float travel_mm;      /* Encoder travel of this observation, any sign. */
    float speed_mm_s;
    float yaw_rate_rad_s;
    float lateral_mg;     /* Encoder fake lateral acceleration, any sign. */
    uint32_t run_epoch;   /* Odd while a run is active. */
    bool valid;           /* Accel valid, calibrated, no sample gap. */
    bool motion_valid;
} imu_hump_input_t;

typedef struct {
    uint32_t run_epoch;     /* Run of these totals; 0 before any run. */
    uint32_t valid_samples; /* Valid observations in this run. */
    float smoothed_mm;      /* Signed 10-sample (100 ms) mean. */
    float max_peak_mm;      /* Highest closed event, magnitude. */
    float last_peak_mm;     /* Last closed event, magnitude. */
    float event_peak_mm;    /* Open event so far, magnitude. */
    float event_travel_mm;  /* Open event travel since its ON crossing. */
    uint16_t count;         /* Closed events, saturating. */
    uint16_t gated_starts;  /* ON crossings blocked by a gate. */
    uint16_t long_rejects;
    uint8_t flags;          /* IMU_HUMP_FLAG_*. */
    uint8_t state;          /* imu_hump_state_t. */
    bool running;
    bool event_left;
    bool last_left;
    bool max_left;
    bool smooth_ready;
} imu_hump_output_t;

typedef struct {
    imu_hump_output_t output;
    float window[IMU_HUMP_MEAN_SAMPLES];
    float sum;
    float below_mm;       /* Travel below OFF in the open event. */
    uint32_t samples;
    uint32_t index;
    bool gate_latched;    /* One gated count per ON crossing. */
} imu_hump_t;

/* Board copy for other tasks. timestamp_ms is the IMU observation time. */
typedef struct {
    imu_hump_output_t hump;
    uint32_t timestamp_ms;
} imu_hump_report_t;

/* Final run values for the summary, the USB line and the DUMP. */
typedef struct {
    uint16_t peak_01mm;   /* 0.1 mm; IMU_HUMP_PEAK_NONE if unavailable. */
    uint16_t count;
    uint8_t flags;        /* IMU_HUMP_FLAG_*. */
    uint8_t gated;        /* Saturates at IMU_HUMP_GATED_MAX. */
} imu_hump_summary_t;

void imu_hump_init(imu_hump_t *s);
/* An odd new run_epoch starts a run: totals clear, smoothing continues.
 * An even epoch ends it: a confirmed open event closes with FLAG_OPEN.
 * Outside a run nothing is counted. MARK never resets these values.
 *
 * Start: |mean| >= ON for ON_TRAVEL of continuous travel, no gate.
 * End: the same-side mean < OFF for OFF_TRAVEL. Longer than MAX_TRAVEL is
 * rejected (FLAG_LONG) and waits for the same OFF condition.
 * Gates: motion invalid, |lateral| > GATE_LATERAL or |yaw| > GATE_YAW
 * block a start and freeze an open peak (FLAG_CURVE); |speed| < MIN_SPEED
 * blocks a start only. Event peak = highest 100 ms mean, same side. */
void imu_hump_step(imu_hump_t *s, const imu_hump_input_t *in);
/* Report the run as if STOP happened now: a confirmed open event counts
 * with FLAG_OPEN. Peak IMU_HUMP_PEAK_NONE (flags IMPLEMENTED|FAULT, count
 * 0) if have is false, run_epoch is even or different, the copy is older
 * than max_age_ms, or the run had no valid sample. Never uses stale data. */
imu_hump_summary_t imu_hump_summarize(const imu_hump_report_t *r, bool have,
                                      uint32_t run_epoch, uint32_t now_ms,
                                      uint32_t max_age_ms);
/* 0.1 mm, rounded, saturating at IMU_HUMP_PEAK_MAX; nonfinite or <= 0
 * gives 0. */
uint16_t imu_hump_tenths(float mm);
/* Event detail byte: number in bits 0..6 saturating at 127, bit 7 LEFT. */
uint8_t imu_hump_detail(uint16_t number, bool left);

#endif
