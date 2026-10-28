#include "line_position.h"

static void set_flag(lp_t *lp, uint16_t bit, bool on) {
    lp->flags = (uint16_t)(on ? (lp->flags | bit) : (lp->flags & ~bit));
}

static void set_side(lp_t *lp, vc_line_side_t side) {
    lp->flags = (uint16_t)((lp->flags & ~LP_FLAG_SIDE_MASK) |
                           (((unsigned int)side << LP_FLAG_SIDE_SHIFT) &
                            LP_FLAG_SIDE_MASK));
}

static uint32_t abs_diff(int32_t a, int32_t b) {
    return a > b ? (uint32_t)(a - b) : (uint32_t)(b - a);
}

void lp_init(lp_t *lp) {
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        lp->floor_q8[s] = 0;
        lp->line_q8[s] = 0;
        lp->norm[s] = 0u;
    }
    lp->position = 0;
    lp->flags = 0u;
    lp->lost_since_ms = 0u;
    lp_calib_begin(lp);
}

void lp_calib_begin(lp_t *lp) {
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        lp->cal_min[s] = UINT16_MAX;
        lp->cal_max[s] = 0u;
        lp->cal_sum[s] = 0u;
    }
    lp->cal_count = 0u;
}

void lp_calib_add(lp_t *lp, const uint16_t raw[LP_SENSORS]) {
    if (lp->cal_count == UINT32_MAX / 4096u) {
        return; /* The sums are full; more data adds nothing. */
    }
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        if (raw[s] < lp->cal_min[s]) lp->cal_min[s] = raw[s];
        if (raw[s] > lp->cal_max[s]) lp->cal_max[s] = raw[s];
        lp->cal_sum[s] += raw[s];
    }
    ++lp->cal_count;
}

lp_calib_result_t lp_calib_end(lp_t *lp, unsigned int *bad_mask) {
    *bad_mask = 0u;
    if (lp->cal_count == 0u) {
        return LP_CALIB_NO_DATA;
    }

    /* Polarity: the floor is seen most of the time, so the average is
     * nearer the floor end. One decision for all three sensors (same
     * module type). */
    uint32_t mean_to_min = 0u;
    uint32_t mean_to_max = 0u;
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        if ((uint32_t)(lp->cal_max[s] - lp->cal_min[s]) < LP_MIN_CONTRAST_ADC) {
            *bad_mask |= 1u << s;
        }
        const uint32_t mean = lp->cal_sum[s] / lp->cal_count;
        mean_to_min += mean - lp->cal_min[s];
        mean_to_max += lp->cal_max[s] - mean;
    }
    if (*bad_mask != 0u) {
        return LP_CALIB_LOW_CONTRAST;
    }

    const bool line_high = mean_to_min < mean_to_max;
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        const int32_t lo = (int32_t)lp->cal_min[s] << 8;
        const int32_t hi = (int32_t)lp->cal_max[s] << 8;
        lp->floor_q8[s] = line_high ? lo : hi;
        lp->line_q8[s] = line_high ? hi : lo;
    }
    set_flag(lp, LP_FLAG_LINE_HIGH, line_high);
    set_flag(lp, LP_FLAG_CALIBRATED, true);
    /* Assume the line is under the robot at the start, so a missing line
     * gives LINE_LOST after LP_LOST_MS. */
    set_flag(lp, LP_FLAG_LINE_SEEN, true);
    set_flag(lp, LP_FLAG_LOST_TIMING, false);
    set_side(lp, VC_LINE_SIDE_UNKNOWN);
    return LP_CALIB_OK;
}

/* 0-100: where the reading sits between this sensor's floor and line. */
static uint8_t normalise(const lp_t *lp, unsigned int s, uint16_t raw) {
    const int32_t span = lp->line_q8[s] - lp->floor_q8[s]; /* sign = polarity */
    const int32_t from_floor = ((int32_t)raw << 8) - lp->floor_q8[s];
    if (span == 0) {
        return 0u;
    }
    int32_t n = (from_floor * 100) / span; /* |from_floor| < 2^20: no overflow */
    if (n < 0) n = 0;
    if (n > 100) n = 100;
    return (uint8_t)n;
}

/* Move one end slowly toward the reading, but never so far that the
 * floor-to-line range drops below the sanity limit. */
static void adapt(int32_t *end, int32_t other, uint16_t raw) {
    const int32_t target = (int32_t)raw << 8;
    const int32_t next = *end + (target - *end) / (1 << LP_ADAPT_SHIFT);
    if (abs_diff(next, other) >= (LP_MIN_CONTRAST_ADC << 8)) {
        *end = next;
    }
}

vc_event_t lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now_ms) {
    if (!lp_calibrated(lp)) {
        return vc_event(VC_EVENT_NONE, 0u);
    }

    unsigned int max_n = 0u;
    unsigned int min_n = 100u;
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        const uint8_t n = normalise(lp, s, raw[s]);
        lp->norm[s] = n;
        if (n > max_n) max_n = n;
        if (n < min_n) min_n = n;

        if (n < LP_ADAPT_FLOOR_PCT) {
            adapt(&lp->floor_q8[s], lp->line_q8[s], raw[s]);
        } else if (n > LP_ADAPT_LINE_PCT) {
            adapt(&lp->line_q8[s], lp->floor_q8[s], raw[s]);
        }
    }
    set_flag(lp, LP_FLAG_JUNCTION, min_n >= LP_JUNCTION_PCT);

    if (lp_line_seen(lp)) {
        if (max_n < LP_LOST_PCT) {
            if (!(lp->flags & LP_FLAG_LOST_TIMING)) {
                set_flag(lp, LP_FLAG_LOST_TIMING, true);
                lp->lost_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - lp->lost_since_ms) >= LP_LOST_MS) {
                set_flag(lp, LP_FLAG_LINE_SEEN, false);
                set_flag(lp, LP_FLAG_LOST_TIMING, false);
                return vc_event(VC_EVENT_LINE_LOST, (uint16_t)lp_last_side(lp));
            }
            return vc_event(VC_EVENT_NONE, 0u);
        }
        set_flag(lp, LP_FLAG_LOST_TIMING, false);
    } else if (max_n < LP_FOUND_PCT) {
        return vc_event(VC_EVENT_NONE, 0u);
    }

    /* Line seen: weighted average, left = -100, centre = 0, right = +100. */
    const int32_t sum = (int32_t)lp->norm[LP_LEFT] + lp->norm[LP_CENTRE] +
                        lp->norm[LP_RIGHT];
    if (sum > 0) {
        lp->position = (int8_t)((100 * ((int32_t)lp->norm[LP_RIGHT] -
                                        (int32_t)lp->norm[LP_LEFT])) / sum);
    }
    set_side(lp, lp->position < -LP_SIDE_PCT  ? VC_LINE_SIDE_LEFT
                 : lp->position > LP_SIDE_PCT ? VC_LINE_SIDE_RIGHT
                                              : VC_LINE_SIDE_UNKNOWN);

    if (!lp_line_seen(lp)) {
        set_flag(lp, LP_FLAG_LINE_SEEN, true);
        return vc_event(VC_EVENT_LINE_FOUND, 0u);
    }
    return vc_event(VC_EVENT_NONE, 0u);
}
