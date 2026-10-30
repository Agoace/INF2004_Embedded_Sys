#ifndef LINE_POSITION_H
#define LINE_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#include "line_types.h"

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
 * Each reading is normalised to 0-1000 (0 = floor, 1000 = line). The line
 * position is -100 (under the left sensor) to +100 (under the right sensor).
 *
 * The sensors cannot see the line in the gaps between them (measured
 * 2026-10-27): there only a weak rise of about 0.7 % of the range reaches
 * the neighbours. So the side the line moves to is tracked:
 *   - while the line is centred (C >= 80 %, outers dark), the outer
 *     sensors' readings are kept as a baseline;
 *   - when the centre reading fades, the side is taken from, newest
 *     evidence first: an outer sensor that sees the line, an outer sensor
 *     that rises above its baseline (gap rise), or the caller's hint
 *     (wheel encoders: which way the robot is turning);
 *   - the side is kept until the line is centred again.
 * With a known side, position = side * 50 * ((1000 - C) + outer) / 1000:
 * 0 under the centre, +/-50 in the gap, +/-100 under an outer sensor.
 *
 * Pure logic: no hardware or RTOS calls. See docs/LINE_POSITION.md.
 */

#define LP_SENSORS 3u
#define LP_LEFT 0u
#define LP_CENTRE 1u
#define LP_RIGHT 2u

/* Readings are per mille (0-1000) of each sensor's own floor-to-line
 * range. Thresholds are given in percent and compared as LP_PERMILLE(x). */
#define LP_PERMILLE(pct) ((pct) * 10u)

#define LP_LOST_PCT 30u     /* All sensors below this: line not seen. */
#define LP_FOUND_PCT 50u    /* Any sensor at or above this: line seen. */
#define LP_JUNCTION_PCT 50u /* All sensors at or above this: junction. */
#define LP_SIDE_PCT 20      /* |position| above this gives a side. */
#define LP_ADAPT_FLOOR_PCT 20u /* Learn the floor value below this. */
#define LP_ADAPT_LINE_PCT 80u  /* Learn the line value above this. */
/* Learning speed: an end moves dt/256 of the way to the new reading, where dt
 * is the elapsed time since the previous sample. At the nominal 1 ms period
 * that is exactly one 1/256 step, so the upstream behaviour is preserved, and a
 * longer sample interval cannot change the effective response (RT-0031 B5). */
#define LP_ADAPT_SHIFT 8u
/* Nominal and maximum sample interval used for the dt scaling. */
#define LP_NOMINAL_DT_MS 1u
#define LP_MAX_DT_MS 16u
/* Design review: LINE_LOST after more than 50 ms without the line. */
#define LP_LOST_MS 50u
/* Dark from the middle (|position| < LP_OUTWARD_POSITION): the line is
 * probably in a gap and the side may come late. Wait longer, steer back. */
#define LP_GAP_LOST_MS 300u

/* Side tracking (see above). */
#define LP_CENTRED_PCT 80u     /* C at or above this, outers dark: centred */
#define LP_OUTER_SEEN_PCT 5u   /* an outer sensor at or above this sees the line */
#define LP_GAP_RISE_PERMILLE 3 /* gap rise above the baseline (about 11 ADC) */
#define LP_BASELINE_SHIFT 4u   /* baseline moves dt/16 per sample */
#define LP_BASELINE_MIN_MS 16u /* centred time before the baseline is used */
/* |position| at or above this when the line goes dark: it left outward,
 * past an outer sensor (not into a gap). */
#define LP_OUTWARD_POSITION 75
/* After a junction (all sensors on the line), a side arm of the junction
 * can stay near an outer sensor and look like a gap rise. For this time
 * after the last junction reading, the gap rise is not used and the
 * baseline does not learn. Bench run 3, 2026-10-27: the wrong side came
 * about 150 ms after the junction. */
#define LP_JUNCTION_HOLD_MS 300u
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

    uint16_t norm[LP_SENSORS]; /* 0-1000 */
    int8_t position;           /* -100..+100 */
    uint16_t flags;            /* LP_FLAG_* */
    uint32_t lost_since_ms;

    /* Side tracking. */
    int32_t baseline_q8[LP_SENSORS]; /* outer sensors only, Q8 raw */
    uint16_t baseline_ms; /* centred time the baseline has learned for */
    uint8_t fade_side;   /* line_side_t: where the line went */
    uint8_t fade_source; /* lp_side_source_t */
    uint8_t hint;        /* line_side_t from the caller */
    uint32_t junction_ms; /* time of the last junction reading */
    uint32_t last_ms;     /* previous lp_update() time, for the dt scaling */
    bool has_last_ms;
} lp_t;

/* Where the side came from (for logs and tests). */
typedef enum {
    LP_SIDE_SOURCE_NONE = 0,
    LP_SIDE_SOURCE_SENSOR = 1, /* an outer sensor sees the line */
    LP_SIDE_SOURCE_GAP = 2,    /* an outer sensor rose above its baseline */
    LP_SIDE_SOURCE_HINT = 3,   /* the caller's hint (wheel encoders) */
} lp_side_source_t;

#define LP_FLAG_CALIBRATED 0x0001u
#define LP_FLAG_LINE_SEEN 0x0002u
#define LP_FLAG_JUNCTION 0x0004u
#define LP_FLAG_LOST_TIMING 0x0008u
#define LP_FLAG_SIDE_SHIFT 4u
#define LP_FLAG_SIDE_MASK 0x0030u /* line_side_t, 2 bits */
#define LP_FLAG_LINE_HIGH 0x0040u /* Polarity: line reads higher than floor */
#define LP_FLAG_CENTRED 0x0080u   /* Line under the centre sensor only */
#define LP_FLAG_JUNCTION_HOLD 0x0100u /* Less than LP_JUNCTION_HOLD_MS after a junction */

void lp_init(lp_t *lp);

/* Calibration sweep. Feed every reading while the sensors cross the line. */
void lp_calib_begin(lp_t *lp);
void lp_calib_add(lp_t *lp, const uint16_t raw[LP_SENSORS]);

typedef enum {
    LP_CALIB_OK = 0,
    LP_CALIB_NO_DATA = 1,      /* No readings. */
    LP_CALIB_LOW_CONTRAST = 2, /* A sensor range is below the sanity limit. */
} lp_calib_result_t;

/* Finish the sweep. A failed sweep leaves the calibration invalid, because
 * lp_calib_begin() already discarded the previous one (RT-0031 B4).
 * bad_mask gets one bit per sensor with low contrast (L=1, C=2, R=4). */
lp_calib_result_t lp_calib_end(lp_t *lp, unsigned int *bad_mask);

/*
 * Discard the usable calibration and all transient history.
 *
 * lp_calib_begin() calls this, and the Manager calls it on cancellation and on
 * a sensor fault, so an old endpoint pair can never start a run. After it,
 * lp_calibrated() is false and lp_update() returns LP_EVENT_NONE until a new
 * sweep succeeds.
 */
void lp_invalidate(lp_t *lp);

/* Qualified line edges. The 50 / 300 ms dwell already happened inside this
 * module, so the Manager must not add another one. */
typedef enum {
    LP_EVENT_NONE = 0,
    LP_EVENT_LOST = 1,  /* read lp_last_side() for the retained side */
    LP_EVENT_FOUND = 2,
} lp_event_t;

/*
 * Process one reading (one averaged ADC value per sensor).
 * Returns LP_EVENT_LOST after the qualified loss dwell, LP_EVENT_FOUND when
 * the line returns, else LP_EVENT_NONE. Returns LP_EVENT_NONE until a
 * calibration succeeds.
 */
lp_event_t lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now_ms);

/* Optional: the side the line is probably moving to, from the caller's
 * other data. Example: the left wheel turned more than the right, so the
 * robot turns right and the line moves to the LEFT. Used only when no
 * sensor shows the side. Set LINE_SIDE_UNKNOWN when there is no hint. */
static inline void lp_set_hint(lp_t *lp, line_side_t side) {
    lp->hint = (uint8_t)side;
}

static inline bool lp_calibrated(const lp_t *lp) {
    return (lp->flags & LP_FLAG_CALIBRATED) != 0u;
}
static inline bool lp_line_seen(const lp_t *lp) {
    return (lp->flags & LP_FLAG_LINE_SEEN) != 0u;
}
static inline bool lp_junction(const lp_t *lp) {
    return (lp->flags & LP_FLAG_JUNCTION) != 0u;
}
/* True up to LP_JUNCTION_HOLD_MS after the last junction reading. */
static inline bool lp_junction_hold(const lp_t *lp) {
    return (lp->flags & LP_FLAG_JUNCTION_HOLD) != 0u;
}
static inline bool lp_line_reads_high(const lp_t *lp) {
    return (lp->flags & LP_FLAG_LINE_HIGH) != 0u;
}
static inline bool lp_centred(const lp_t *lp) {
    return (lp->flags & LP_FLAG_CENTRED) != 0u;
}
static inline line_side_t lp_last_side(const lp_t *lp) {
    return (line_side_t)((lp->flags & LP_FLAG_SIDE_MASK) >> LP_FLAG_SIDE_SHIFT);
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
 *   bits 10-11  last side (line_side_t)
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
