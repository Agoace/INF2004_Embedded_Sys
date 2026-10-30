#include "obstacle_bypass.h"

/* sin(0..90 degrees) x 1000, rounded. */
static const uint16_t SIN_PERMILLE[91] = {
    0,   17,  35,  52,  70,  87,  105, 122, 139, 156, 174, 191, 208, 225, 242, 259,
    276, 292, 309, 326, 342, 358, 375, 391, 407, 423, 438, 454, 469, 485, 500, 515,
    530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682, 695, 707, 719, 731,
    743, 755, 766, 777, 788, 799, 809, 819, 829, 839, 848, 857, 866, 875, 883, 891,
    899, 906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
    985, 988, 990, 993, 995, 996, 998, 999, 999, 1000, 1000,
};

/* tan(0..30 degrees) x 1000, rounded. */
static const uint16_t TAN_PERMILLE[31] = {
    0,   17,  35,  52,  70,  87,  105, 123, 141, 158, 176, 194, 213, 231, 249, 268,
    287, 306, 325, 344, 364, 384, 404, 424, 445, 466, 488, 510, 532, 554, 577,
};

static int clamp_deg(int deg, int limit) {
    return deg > limit ? limit : deg < -limit ? -limit : deg;
}

int bp_clear_angle(uint16_t clearance_mm, uint16_t distance_mm) {
    /* sin(a) >= c / d  <=>  sin1000(a) x d >= 1000 x c (no division). */
    const uint32_t need = 1000u * clearance_mm;
    for (int a = 0; a <= 90; ++a) {
        if ((uint32_t)SIN_PERMILLE[a] * distance_mm >= need) {
            return a;
        }
    }
    return 90;
}

uint32_t bp_leg_mm(uint16_t clearance_mm, uint16_t distance_mm) {
    if (distance_mm < clearance_mm) {
        return 0u; /* (d = c gives the root of 0) */
    }
    const uint32_t n = (uint32_t)distance_mm * distance_mm - (uint32_t)clearance_mm * clearance_mm;
    uint32_t r = 0u;
    for (uint32_t bit = 1u << 15; bit != 0u; bit >>= 1) { /* n < 2^32: root < 2^16 */
        const uint32_t t = r | bit;
        if (t * t <= n) {
            r = t;
        }
    }
    return r;
}

uint32_t bp_mm_to_counts(uint32_t mm) {
    const uint64_t num = (uint64_t)mm * BP_COUNTS_PER_WHEEL_TURN * 1000u;
    return (uint32_t)((num + BP_WHEEL_CIRCUMFERENCE_UM / 2u) / BP_WHEEL_CIRCUMFERENCE_UM);
}

int bp_steer_position(int steer_deg) {
    const int a = clamp_deg(steer_deg < 0 ? -steer_deg : steer_deg, BP_ANGLE_BIAS);
    const uint32_t num = (uint32_t)TAN_PERMILLE[a] * BP_HALF_TRACK_TENTHS_MM * 100u;
    const uint32_t den = BP_WHEELBASE_TENTHS_MM * 1000u;
    const int position = (int)((num + den / 2u) / den);
    return steer_deg < 0 ? position : -position;
}

static void set_servo(bp_t *bp, int deg, uint32_t now_ms) {
    deg = clamp_deg(deg, bp->cfg.limit_deg);
    const int move = deg > bp->servo_deg ? deg - bp->servo_deg : bp->servo_deg - deg;
    bp->servo_deg = (int8_t)deg;
    bp->settle_ms = now_ms + BP_SETTLE_BASE_MS + BP_SETTLE_PER_DEG_MS * (uint32_t)move;
}

static void fail(bp_t *bp, bp_fail_t reason) {
    bp->state = BP_FAILED;
    bp->fail = (uint8_t)reason;
}

void bp_init(bp_t *bp, const bp_config_t *cfg, obstacle_side_t side, uint16_t obstacle_mm,
             uint32_t now_ms) {
    *bp = (bp_t){0};
    bp->cfg = *cfg;
    if (bp->cfg.limit_deg > BP_ANGLE_BIAS) {
        bp->cfg.limit_deg = BP_ANGLE_BIAS;
    }
    bp->free_mm = (uint16_t)(obstacle_mm + OBSTACLE_PROFILE_GAP_MM);
    bp->last_mm = obstacle_mm;
    bp->state = BP_PIVOT;
    bp->state_ms = now_ms;
    if (side == OBSTACLE_SIDE_LEFT) {
        bp->side = +1;
    } else if (side == OBSTACLE_SIDE_RIGHT) {
        bp->side = -1;
    } else {
        fail(bp, BP_FAIL_NO_SIDE);
        return;
    }
    set_servo(bp, -bp->side * bp->cfg.limit_deg, now_ms);
}

static bool active(const bp_t *bp) {
    return bp->state == BP_PIVOT || bp->state == BP_AROUND;
}

bool bp_wants_reading(const bp_t *bp, uint32_t now_ms) {
    return active(bp) && !bp->pending && (int32_t)(now_ms - bp->settle_ms) >= 0;
}

void bp_triggered(bp_t *bp) {
    bp->pending = true;
    bp->reading_deg = bp->servo_deg;
}

/* Steer so that the sensor end passes the point (deg, mm) with the
 * clearance to spare, on the S side. */
static void steer_past(bp_t *bp, int deg, uint16_t mm) {
    const int steer = deg + bp->side * bp_clear_angle(bp->cfg.clearance_mm, mm);
    bp->steer_deg = (int8_t)clamp_deg(steer, bp->cfg.limit_deg);
    /* New leg: to the closest point to (deg, mm), from the next update. */
    bp->leg_counts = bp_mm_to_counts(bp_leg_mm(bp->cfg.clearance_mm, mm));
    bp->leg_started = false;
}

static bool leg_done(const bp_t *bp) {
    return bp->leg_started && bp->travel - bp->leg_from >= (int32_t)bp->leg_counts;
}

static void pivot_reading(bp_t *bp, ultrasonic_status_t status, uint16_t mm,
                          uint32_t now_ms) {
    const bool free = status == ULTRASONIC_STATUS_NO_RETURN || mm >= bp->free_mm;
    if (!free) {
        bp->seen = true;
        bp->last_mm = mm;
        bp->clear_count = 0u;
    } else if (bp->seen) {
        ++bp->clear_count;
    }
    if (bp->clear_count >= BP_STRAIGHT_CLEAR) {
        /* The last readings were free at the limit: that is the edge. */
        bp->state = BP_AROUND;
        bp->phase = BP_PHASE_DRIVE;
        bp->state_ms = now_ms;
        steer_past(bp, -bp->side * bp->cfg.limit_deg, bp->last_mm);
    }
    set_servo(bp, bp->servo_deg, now_ms);
}

void bp_reading(bp_t *bp, ultrasonic_result_t result, uint32_t now_ms) {
    if (!bp->pending) {
        return;
    }
    bp->pending = false;
    if (!active(bp)) {
        return;
    }
    if (ultrasonic_result_fault(result)) {
        fail(bp, BP_FAIL_SENSOR);
        return;
    }
    const ultrasonic_status_t status = ultrasonic_result_status(result);
    if (status != ULTRASONIC_STATUS_VALID && status != ULTRASONIC_STATUS_NO_RETURN) {
        set_servo(bp, bp->servo_deg, now_ms); /* no information: read again */
        return;
    }
    const uint16_t mm = ultrasonic_result_distance(result);
    if (bp->state == BP_PIVOT) {
        pivot_reading(bp, status, mm, now_ms);
        return;
    }

    const bool seen = status == ULTRASONIC_STATUS_VALID && mm < bp->free_mm;
    const int at = bp->reading_deg;
    const int limit = bp->cfg.limit_deg;
    const int toward_obstacle = -bp->side;
    if (seen && mm < BP_TOO_NEAR_MM && at <= BP_TOO_NEAR_DEG && at >= -BP_TOO_NEAR_DEG) {
        fail(bp, BP_FAIL_TOO_NEAR);
        return;
    }
    if (seen) {
        bp->last_mm = mm;
    }

    switch (bp_phase(bp)) {
    case BP_PHASE_DRIVE:
        if (seen) {
            steer_past(bp, at, mm);
            set_servo(bp, at + bp->side * bp->cfg.bounce_deg, now_ms);
        } else if (at == toward_obstacle * limit) {
            if (leg_done(bp)) {
                bp->phase = BP_PHASE_TURN; /* lost: turn toward it until seen */
                bp->turn_ms = now_ms;
            }
            set_servo(bp, at, now_ms);
        } else {
            set_servo(bp, at + toward_obstacle * bp->cfg.step_deg, now_ms);
        }
        break;
    case BP_PHASE_TURN:
        if (seen) {
            bp->phase = BP_PHASE_EDGE;
            bp->clear_count = 0u;
            set_servo(bp, at + bp->side * bp->cfg.step_deg, now_ms);
        } else {
            set_servo(bp, at, now_ms);
        }
        break;
    default: /* BP_PHASE_EDGE */
        if (seen) {
            bp->clear_count = 0u;
            if (at == bp->side * limit) {
                steer_past(bp, at, mm); /* seen across the whole view */
                bp->phase = BP_PHASE_DRIVE;
                set_servo(bp, at, now_ms);
            } else {
                set_servo(bp, at + bp->side * bp->cfg.step_deg, now_ms);
            }
        } else if (++bp->clear_count >= BP_EDGE_FREE) {
            steer_past(bp, at, bp->last_mm);
            bp->phase = BP_PHASE_DRIVE;
            set_servo(bp, at, now_ms);
        } else {
            set_servo(bp, at, now_ms); /* read this angle again */
        }
        break;
    }
}

static bp_word_t word(bp_move_t move, int steer_deg, int servo_deg) {
    return (bp_word_t)((unsigned int)move |
                       ((unsigned int)(steer_deg + BP_ANGLE_BIAS) & BP_ANGLE_MASK)
                           << BP_STEER_SHIFT |
                       ((unsigned int)(servo_deg + BP_ANGLE_BIAS) & BP_ANGLE_MASK)
                           << BP_SERVO_SHIFT);
}

bp_word_t bp_update(bp_t *bp, bool line_seen, bool junction, int32_t travel_counts,
                    uint32_t now_ms) {
    bp->travel = travel_counts;
    /* DRIVE is always entered through steer_past(), which starts a new leg. */
    if (!bp->leg_started) {
        bp->leg_from = travel_counts;
        bp->leg_started = true;
    }
    if (!line_seen) {
        if (!bp->dark) {
            bp->dark = true;
            bp->dark_ms = now_ms;
        }
        if ((uint32_t)(now_ms - bp->dark_ms) >= bp->cfg.line_arm_ms) {
            bp->armed = true;
        }
    } else {
        bp->dark = false;
    }

    const uint32_t in_state = now_ms - bp->state_ms;
    if (bp->state == BP_PIVOT && in_state >= bp->cfg.pivot_max_ms) {
        fail(bp, BP_FAIL_PIVOT_TIME);
    } else if (bp->state == BP_AROUND) {
        if (bp->armed && line_seen && !junction) {
            bp->state = BP_DONE;
        } else if (in_state >= bp->cfg.around_max_ms) {
            fail(bp, BP_FAIL_AROUND_TIME);
        } else if (bp->phase == BP_PHASE_TURN &&
                   (uint32_t)(now_ms - bp->turn_ms) >= bp->cfg.pivot_max_ms) {
            fail(bp, BP_FAIL_PIVOT_TIME);
        }
    }

    switch (bp_state(bp)) {
    case BP_PIVOT:
        return word(BP_MOVE_PIVOT, bp->side * BP_ANGLE_BIAS, bp->servo_deg);
    case BP_AROUND:
        if (bp->phase == BP_PHASE_TURN) {
            return word(BP_MOVE_PIVOT, -bp->side * BP_ANGLE_BIAS, bp->servo_deg);
        }
        if (bp->phase == BP_PHASE_EDGE) {
            return word(BP_MOVE_STOP, 0, bp->servo_deg);
        }
        return word(BP_MOVE_STEER, bp->steer_deg, bp->servo_deg);
    default:
        return word(BP_MOVE_STOP, 0, 0);
    }
}
