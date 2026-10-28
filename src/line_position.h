#ifndef LINE_POSITION_H
#define LINE_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_controller.h"

/*
 * Normalised line position from the three analogue IR line sensors.
 *
 * No fixed ADC thresholds. Each sensor learns its own floor value and line
 * value:
 *   A. Calibration sweep: the sensors move across the line for a few
 *      seconds (by hand for now; later by the CALIBRATE state). Each sensor
 *      records its minimum and maximum. The polarity (line reads high or
 *      low) is found from the data: the floor is seen most of the time, so
 *      the average reading is nearer the floor end.
 *   B. While running, each end follows slow changes (light, surface). A
 *      sensor learns its floor value only when it clearly sees the floor,
 *      and its line value only when it clearly sees the line.
 *
 * Each reading is normalised to 0-100 (0 = floor, 100 = line). The line
 * position is the weighted average of the three sensors, -100 (under the
 * left sensor) to +100 (under the right sensor).
 *
 * Pure logic: no hardware or RTOS calls. See docs/LINE_POSITION.md.
 */

#define LP_SENSORS 3u
#define LP_LEFT 0u
#define LP_CENTRE 1u
#define LP_RIGHT 2u

/* Thresholds as a percentage of each sensor's own floor-to-line range. */
#define LP_LOST_PCT 30u     /* All sensors below this: line not seen. */
#define LP_FOUND_PCT 50u    /* Any sensor at or above this: line seen. */
#define LP_JUNCTION_PCT 50u /* All sensors at or above this: junction. */
#define LP_SIDE_PCT 20      /* |position| above this gives a side. */
#define LP_ADAPT_FLOOR_PCT 20u /* Learn the floor value below this. */
#define LP_ADAPT_LINE_PCT 80u  /* Learn the line value above this. */
/* Learning speed: each update moves an end 1/256 of the way to the new
 * reading. At 1 kHz this follows a change in about 0.25 s. */
#define LP_ADAPT_SHIFT 8u
/* Design review: LINE_LOST after more than 50 ms without the line. */
#define LP_LOST_MS 50u
/* Sanity limit, not a classification threshold: a sensor whose floor and
 * line values differ by less than this is broken or never saw the line.
 * About 0.16 V. */
#define LP_MIN_CONTRAST_ADC 200u

typedef struct {
    /* Learned ends, 12-bit ADC values with 8 fraction bits for the slow
     * update (Q8). */
    int32_t floor_q8[LP_SENSORS];
    int32_t line_q8[LP_SENSORS];

    /* Calibration sweep data. */
    uint16_t cal_min[LP_SENSORS];
    uint16_t cal_max[LP_SENSORS];
    uint32_t cal_sum[LP_SENSORS];
    uint32_t cal_count;

    uint8_t norm[LP_SENSORS]; /* 0-100 */
    int8_t position;          /* -100..+100 */
    uint16_t flags;           /* LP_FLAG_* */
    uint32_t lost_since_ms;
} lp_t;

#define LP_FLAG_CALIBRATED 0x0001u
#define LP_FLAG_LINE_SEEN 0x0002u
#define LP_FLAG_JUNCTION 0x0004u
#define LP_FLAG_LOST_TIMING 0x0008u
#define LP_FLAG_SIDE_SHIFT 4u
#define LP_FLAG_SIDE_MASK 0x0030u /* vc_line_side_t, 2 bits */
#define LP_FLAG_LINE_HIGH 0x0040u /* Polarity: line reads higher than floor */

void lp_init(lp_t *lp);

/* Calibration sweep. Feed every reading while the sensors cross the line. */
void lp_calib_begin(lp_t *lp);
void lp_calib_add(lp_t *lp, const uint16_t raw[LP_SENSORS]);

typedef enum {
    LP_CALIB_OK = 0,
    LP_CALIB_NO_DATA = 1,      /* No readings. */
    LP_CALIB_LOW_CONTRAST = 2, /* A sensor range is below the sanity limit. */
} lp_calib_result_t;

/* Finish the sweep. On failure the previous calibration stays in use.
 * bad_mask gets one bit per sensor with low contrast (L=1, C=2, R=4). */
lp_calib_result_t lp_calib_end(lp_t *lp, unsigned int *bad_mask);

/*
 * Process one reading (one averaged ADC value per sensor).
 * Returns VC_EVENT_LINE_LOST (payload: last side) after LP_LOST_MS without
 * the line, VC_EVENT_LINE_FOUND when the line returns, else VC_EVENT_NONE.
 * Returns VC_EVENT_NONE until a calibration succeeds.
 */
vc_event_t lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now_ms);

static inline bool lp_calibrated(const lp_t *lp) {
    return (lp->flags & LP_FLAG_CALIBRATED) != 0u;
}
static inline bool lp_line_seen(const lp_t *lp) {
    return (lp->flags & LP_FLAG_LINE_SEEN) != 0u;
}
static inline bool lp_junction(const lp_t *lp) {
    return (lp->flags & LP_FLAG_JUNCTION) != 0u;
}
static inline bool lp_line_reads_high(const lp_t *lp) {
    return (lp->flags & LP_FLAG_LINE_HIGH) != 0u;
}
static inline vc_line_side_t lp_last_side(const lp_t *lp) {
    return (vc_line_side_t)((lp->flags & LP_FLAG_SIDE_MASK) >> LP_FLAG_SIDE_SHIFT);
}
static inline uint16_t lp_floor(const lp_t *lp, unsigned int s) {
    return (uint16_t)(lp->floor_q8[s] >> 8);
}
static inline uint16_t lp_line(const lp_t *lp, unsigned int s) {
    return (uint16_t)(lp->line_q8[s] >> 8);
}

/*
 * Line word for other tasks and telemetry: 16 bits.
 *
 *   bits 0-7    position, two's complement, -100..+100 (8 bits)
 *   bit  8      line seen
 *   bit  9      junction
 *   bits 10-11  last side (vc_line_side_t)
 *   bits 12-15  spare, always 0
 */
typedef uint16_t lp_word_t;

static inline lp_word_t lp_word(const lp_t *lp) {
    return (lp_word_t)((uint8_t)lp->position |
                       (lp_line_seen(lp) ? 0x0100u : 0u) |
                       (lp_junction(lp) ? 0x0200u : 0u) |
                       ((unsigned int)lp_last_side(lp) << 10));
}

#endif
