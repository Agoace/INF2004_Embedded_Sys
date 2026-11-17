/*
 * LINE_GAP=1 only: force-included (-include) at the top of
 * line_position.c, so that line_position.c itself does not change.
 *
 * Pair mode defect (school runs 2026-11-17): when the line crosses the
 * gap between L and C, the sensor that saw it fades and the other one
 * starts to rise. lp_update() took the fade as "the line left outward"
 * (fall added to the position, about -77) or, with both below
 * LP_LOST_PCT, held that outward value and used the 50 ms LOST limit.
 * The far term then steered hard while the robot crossed the centre.
 *
 * Gap rule, checked before each lp_update() (every 1 kHz sample): pair
 * mode, calibrated, line seen, and either
 *   (a) both sensors at or above LP_OUTER_SEEN_PCT and both below
 *       LP_LOST_PCT (the line is in the gap), or
 *   (b) one sensor alone sees the line and falls below its peak, while
 *       the other is at or above LP_PAIR_GAP_PERMILLE (it starts to see
 *       the line coming).
 * Then the position is the plain interpolation (C - L - target) / 20,
 * the same formula as with both sensors on the line, and the "only one
 * sensor" memory is cleared, so the gap LOST limit (LP_GAP_LOST_MS)
 * applies. Otherwise lp_update() is unchanged (curve exit: the other
 * sensor stays at its floor).
 *
 * LP_PAIR_GAP_PERMILLE: school logs 2026-11-16/17, rows where the lone
 * sensor fell: gap crossings had the other sensor mostly at 15 per mille
 * or more; real exits mostly below 10. See docs/s3-line-barcode/LINE.md.
 */
#ifndef LINE_GAP_H
#define LINE_GAP_H

#include "line_position.h"

#define LP_PAIR_GAP_PERMILLE 15

static uint16_t normalise(const lp_t *lp, unsigned int s, uint16_t raw);
vc_event_t lp_update_base(lp_t *lp, const uint16_t raw[LP_SENSORS],
                          uint32_t now_ms);

/* True if the gap rule holds for this reading (see above). */
static bool lp_gap_rule(const lp_t *lp, int32_t l, int32_t c)
{
    const int32_t seen = (int32_t)LP_PERMILLE(LP_OUTER_SEEN_PCT);
    const int32_t lost = (int32_t)LP_PERMILLE(LP_LOST_PCT);
    if (!lp->pair || !lp_calibrated(lp) || !lp_line_seen(lp)) {
        return false;
    }
    if (l >= seen && c >= seen) {
        return l < lost && c < lost; /* (a) */
    }
    if (l < seen && c < seen) {
        return false; /* dark: the normal hold and LOST timing */
    }
    const unsigned int alone = c >= seen ? LP_CENTRE : LP_LEFT;
    const int32_t value = c >= seen ? c : l;
    const int32_t other = c >= seen ? l : c;
    return lp->pair_alone == (uint8_t)alone && value < (int32_t)lp->pair_peak &&
           other >= LP_PAIR_GAP_PERMILLE; /* (b) */
}

vc_event_t lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now_ms)
{
    if (lp->pair && lp_calibrated(lp)) {
        const int32_t l = normalise(lp, LP_LEFT, raw[LP_LEFT]);
        const int32_t c = normalise(lp, LP_CENTRE, raw[LP_CENTRE]);
        if (lp_gap_rule(lp, l, c)) {
            int32_t p = (c - l - lp->pair_target) / LP_PAIR_DIVISOR;
            p = p > 100 ? 100 : p < -100 ? -100 : p;
            lp->position = (int8_t)p;
            lp->pair_alone = (uint8_t)LP_SENSORS;
            lp->pair_peak = 0u;
        }
    }
    return lp_update_base(lp, raw, now_ms);
}

#define lp_update lp_update_base

#endif
