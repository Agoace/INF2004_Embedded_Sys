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
        lp->baseline_q8[s] = 0;
    }
    lp->baseline_count = 0u;
    lp->fade_side = (uint8_t)VC_LINE_SIDE_UNKNOWN;
    lp->fade_source = (uint8_t)LP_SIDE_SOURCE_NONE;
    lp->hint = (uint8_t)VC_LINE_SIDE_UNKNOWN;
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
    set_flag(lp, LP_FLAG_CENTRED, false);
    lp->baseline_count = 0u;
    lp->fade_side = (uint8_t)VC_LINE_SIDE_UNKNOWN;
    lp->fade_source = (uint8_t)LP_SIDE_SOURCE_NONE;
    return LP_CALIB_OK;
}

/* 0-1000: where the reading sits between this sensor's floor and line. */
static uint16_t normalise(const lp_t *lp, unsigned int s, uint16_t raw) {
    const int32_t span = lp->line_q8[s] - lp->floor_q8[s]; /* sign = polarity */
    const int32_t from_floor = ((int32_t)raw << 8) - lp->floor_q8[s];
    if (span == 0) {
        return 0u;
    }
    /* |from_floor| < 2^20, so from_floor * 1000 < 2^30: no overflow. */
    int32_t n = (from_floor * 1000) / span;
    if (n < 0) n = 0;
    if (n > 1000) n = 1000;
    return (uint16_t)n;
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

/* Rise of an outer sensor above its baseline, per mille of its range,
 * positive toward the line value. Not limited to 0-1000. */
static int32_t gap_rise(const lp_t *lp, unsigned int s, uint16_t raw) {
    const int32_t span = lp->line_q8[s] - lp->floor_q8[s];
    if (span == 0) {
        return 0;
    }
    return ((((int32_t)raw << 8) - lp->baseline_q8[s]) * 1000) / span;
}

static void set_fade_side(lp_t *lp, vc_line_side_t side, lp_side_source_t source) {
    lp->fade_side = (uint8_t)side;
    lp->fade_source = (uint8_t)source;
}

/* Keeps the baseline of the outer sensors and the side the line went to. */
static void track_side(lp_t *lp, const uint16_t raw[LP_SENSORS]) {
    const uint16_t *n = lp->norm;
    const bool left_sees = n[LP_LEFT] >= LP_PERMILLE(LP_OUTER_SEEN_PCT);
    const bool right_sees = n[LP_RIGHT] >= LP_PERMILLE(LP_OUTER_SEEN_PCT);
    const bool outers_dark = !left_sees && !right_sees;
    const bool centred = outers_dark && n[LP_CENTRE] >= LP_PERMILLE(LP_CENTRED_PCT);
    set_flag(lp, LP_FLAG_CENTRED, centred);

    /* Learn the baseline only while centred: as C fades, the outer sensor
     * that the line moves toward rises, and the baseline must not follow. */
    if (centred) {
        const unsigned int outer[2] = {LP_LEFT, LP_RIGHT};
        for (unsigned int i = 0u; i < 2u; ++i) {
            const unsigned int s = outer[i];
            const int32_t target = (int32_t)raw[s] << 8;
            if (lp->baseline_count == 0u) {
                lp->baseline_q8[s] = target;
            } else {
                lp->baseline_q8[s] += (target - lp->baseline_q8[s]) / (1 << LP_BASELINE_SHIFT);
            }
        }
        if (lp->baseline_count < LP_BASELINE_MIN) {
            ++lp->baseline_count;
        }
    }

    if (centred) {
        set_fade_side(lp, VC_LINE_SIDE_UNKNOWN, LP_SIDE_SOURCE_NONE);
        return;
    }
    if (left_sees != right_sees) {
        set_fade_side(lp, left_sees ? VC_LINE_SIDE_LEFT : VC_LINE_SIDE_RIGHT,
                      LP_SIDE_SOURCE_SENSOR);
        return;
    }
    if (outers_dark && lp->baseline_count >= LP_BASELINE_MIN) {
        const int32_t rise_l = gap_rise(lp, LP_LEFT, raw[LP_LEFT]);
        const int32_t rise_r = gap_rise(lp, LP_RIGHT, raw[LP_RIGHT]);
        if (rise_l >= LP_GAP_RISE_PERMILLE && rise_l - rise_r >= LP_GAP_RISE_PERMILLE) {
            set_fade_side(lp, VC_LINE_SIDE_LEFT, LP_SIDE_SOURCE_GAP);
            return;
        }
        if (rise_r >= LP_GAP_RISE_PERMILLE && rise_r - rise_l >= LP_GAP_RISE_PERMILLE) {
            set_fade_side(lp, VC_LINE_SIDE_RIGHT, LP_SIDE_SOURCE_GAP);
            return;
        }
    }
    if (lp->fade_side == (uint8_t)VC_LINE_SIDE_UNKNOWN &&
        lp->hint != (uint8_t)VC_LINE_SIDE_UNKNOWN) {
        set_fade_side(lp, (vc_line_side_t)lp->hint, LP_SIDE_SOURCE_HINT);
    }
}

/* Weighted average: left = -100, centre = 0, right = +100. */
static bool weighted_position(const lp_t *lp, int8_t *position) {
    const int32_t sum = (int32_t)lp->norm[LP_LEFT] + lp->norm[LP_CENTRE] +
                        lp->norm[LP_RIGHT];
    if (sum == 0) {
        return false;
    }
    *position = (int8_t)((100 * ((int32_t)lp->norm[LP_RIGHT] -
                                 (int32_t)lp->norm[LP_LEFT])) / sum);
    return true;
}

/* Position while the line is seen. dark: no sensor reaches LP_LOST_PCT. */
static void update_position(lp_t *lp, bool dark) {
    const uint16_t *n = lp->norm;
    const unsigned int seen = LP_PERMILLE(LP_OUTER_SEEN_PCT);
    const vc_line_side_t side = (vc_line_side_t)lp->fade_side;

    if ((n[LP_LEFT] >= seen && n[LP_RIGHT] >= seen) || side == VC_LINE_SIDE_UNKNOWN) {
        if (!dark) {
            (void)weighted_position(lp, &lp->position);
        }
        return; /* dark with no side: keep the last position */
    }

    const int32_t sign = side == VC_LINE_SIDE_LEFT ? -1 : 1;
    const int32_t outer = side == VC_LINE_SIDE_LEFT ? n[LP_LEFT] : n[LP_RIGHT];
    const int32_t previous = lp->position < 0 ? -lp->position : lp->position;
    int32_t magnitude;
    if (dark && previous >= LP_OUTWARD_POSITION) {
        magnitude = previous; /* left outward past the outer sensor */
    } else if (n[LP_CENTRE] < seen && outer >= (int32_t)seen) {
        magnitude = 100; /* under the outer sensor only */
    } else {
        magnitude = (50 * ((1000 - (int32_t)n[LP_CENTRE]) + outer)) / 1000;
        if (magnitude > 100) magnitude = 100;
    }
    lp->position = (int8_t)(sign * magnitude);
}

static vc_line_side_t position_side(const lp_t *lp) {
    if (lp->fade_side != (uint8_t)VC_LINE_SIDE_UNKNOWN) {
        return (vc_line_side_t)lp->fade_side;
    }
    return lp->position < -LP_SIDE_PCT  ? VC_LINE_SIDE_LEFT
           : lp->position > LP_SIDE_PCT ? VC_LINE_SIDE_RIGHT
                                        : VC_LINE_SIDE_UNKNOWN;
}

vc_event_t lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now_ms) {
    if (!lp_calibrated(lp)) {
        return vc_event(VC_EVENT_NONE, 0u);
    }

    unsigned int max_n = 0u;
    unsigned int min_n = 1000u;
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        const uint16_t n = normalise(lp, s, raw[s]);
        lp->norm[s] = n;
        if (n > max_n) max_n = n;
        if (n < min_n) min_n = n;

        if (n < LP_PERMILLE(LP_ADAPT_FLOOR_PCT)) {
            adapt(&lp->floor_q8[s], lp->line_q8[s], raw[s]);
        } else if (n > LP_PERMILLE(LP_ADAPT_LINE_PCT)) {
            adapt(&lp->line_q8[s], lp->floor_q8[s], raw[s]);
        }
    }
    set_flag(lp, LP_FLAG_JUNCTION, min_n >= LP_PERMILLE(LP_JUNCTION_PCT));
    track_side(lp, raw);

    if (lp_line_seen(lp)) {
        const bool dark = max_n < LP_PERMILLE(LP_LOST_PCT);
        const int32_t before = lp->position < 0 ? -lp->position : lp->position;
        update_position(lp, dark);
        set_side(lp, position_side(lp));
        if (dark) {
            if (!(lp->flags & LP_FLAG_LOST_TIMING)) {
                set_flag(lp, LP_FLAG_LOST_TIMING, true);
                lp->lost_since_ms = now_ms;
            }
            /* Dark from the middle: the line is probably in a gap, and the
             * gap rise or the hint may give the side late. Wait longer.
             * Dark after an outer sensor: it left outward, LP_LOST_MS. */
            const bool in_gap = before < LP_OUTWARD_POSITION;
            const uint32_t limit = in_gap ? LP_GAP_LOST_MS : LP_LOST_MS;
            if ((uint32_t)(now_ms - lp->lost_since_ms) >= limit) {
                set_flag(lp, LP_FLAG_LINE_SEEN, false);
                set_flag(lp, LP_FLAG_LOST_TIMING, false);
                return vc_event(VC_EVENT_LINE_LOST, (uint16_t)lp_last_side(lp));
            }
            return vc_event(VC_EVENT_NONE, 0u);
        }
        set_flag(lp, LP_FLAG_LOST_TIMING, false);
        return vc_event(VC_EVENT_NONE, 0u);
    }

    if (max_n < LP_PERMILLE(LP_FOUND_PCT)) {
        return vc_event(VC_EVENT_NONE, 0u);
    }
    update_position(lp, false);
    set_side(lp, position_side(lp));
    set_flag(lp, LP_FLAG_LINE_SEEN, true);
    return vc_event(VC_EVENT_LINE_FOUND, 0u);
}
