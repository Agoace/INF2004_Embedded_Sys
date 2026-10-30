#include "obstacle_bypass.h"

static int clamp_deg(int deg, int limit) {
    return deg > limit ? limit : deg < -limit ? -limit : deg;
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
    bp->free_mm = (uint16_t)(obstacle_mm + OBSTACLE_PROFILE_GAP_MM);
    bp->cfg = *cfg;
    if (bp->cfg.limit_deg > BP_ANGLE_BIAS) {
        bp->cfg.limit_deg = BP_ANGLE_BIAS;
    }
    bp->state = BP_PIVOT;
    bp->state_ms = now_ms;
    set_servo(bp, 0, now_ms);
    if (side == OBSTACLE_SIDE_LEFT) {
        bp->side = +1;
    } else if (side == OBSTACLE_SIDE_RIGHT) {
        bp->side = -1;
    } else {
        fail(bp, BP_FAIL_NO_SIDE);
    }
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
    const bool hit = status == ULTRASONIC_STATUS_VALID && mm < bp->cfg.hit_mm;
    const int at = bp->reading_deg;
    const int limit = bp->cfg.limit_deg;
    const int toward_obstacle = -bp->side;

    if (bp->state == BP_PIVOT) {
        const bool free = status == ULTRASONIC_STATUS_NO_RETURN || mm >= bp->free_mm;
        bp->clear_count = free ? (uint8_t)(bp->clear_count + 1u) : 0u;
        if (bp->clear_count >= BP_STRAIGHT_CLEAR) {
            bp->state = BP_AROUND;
            bp->state_ms = now_ms;
            bp->steer_deg = 0;
            set_servo(bp, toward_obstacle * bp->cfg.step_deg, now_ms);
        } else {
            set_servo(bp, 0, now_ms);
        }
        return;
    }

    /* AROUND */
    if (hit) {
        if (mm < BP_TOO_NEAR_MM && at <= BP_TOO_NEAR_DEG && at >= -BP_TOO_NEAR_DEG) {
            fail(bp, BP_FAIL_TOO_NEAR);
            return;
        }
        bp->steer_deg = (int8_t)clamp_deg(at + bp->side * bp->cfg.offset_deg, limit);
        set_servo(bp, at + bp->side * bp->cfg.bounce_deg, now_ms);
    } else if (at == toward_obstacle * limit) {
        /* Clear at the limit: the obstacle is beside the robot. */
        bp->steer_deg =
            (int8_t)clamp_deg(toward_obstacle * limit + bp->side * bp->cfg.offset_deg, limit);
        set_servo(bp, at, now_ms);
    } else {
        set_servo(bp, at + toward_obstacle * bp->cfg.step_deg, now_ms);
    }
}

static bp_word_t word(bp_move_t move, int steer_deg, int servo_deg) {
    return (bp_word_t)((unsigned int)move |
                       ((unsigned int)(steer_deg + BP_ANGLE_BIAS) & BP_ANGLE_MASK)
                           << BP_STEER_SHIFT |
                       ((unsigned int)(servo_deg + BP_ANGLE_BIAS) & BP_ANGLE_MASK)
                           << BP_SERVO_SHIFT);
}

bp_word_t bp_update(bp_t *bp, bool line_seen, bool junction, uint32_t now_ms) {
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
        }
    }

    switch (bp_state(bp)) {
    case BP_PIVOT:
        return word(BP_MOVE_PIVOT, bp->side * BP_ANGLE_BIAS, bp->servo_deg);
    case BP_AROUND:
        return word(BP_MOVE_STEER, bp->steer_deg, bp->servo_deg);
    default:
        return word(BP_MOVE_STOP, 0, 0);
    }
}

/* tan(0..30 degrees) x 1000, rounded. */
static const uint16_t TAN_PERMILLE[31] = {
    0,   17,  35,  52,  70,  87,  105, 123, 141, 158, 176, 194, 213, 231, 249, 268,
    287, 306, 325, 344, 364, 384, 404, 424, 445, 466, 488, 510, 532, 554, 577,
};

int bp_steer_position(int steer_deg) {
    const int a = clamp_deg(steer_deg < 0 ? -steer_deg : steer_deg, BP_ANGLE_BIAS);
    const uint32_t num = (uint32_t)TAN_PERMILLE[a] * BP_HALF_TRACK_TENTHS_MM * 100u;
    const uint32_t den = BP_WHEELBASE_TENTHS_MM * 1000u;
    const int position = (int)((num + den / 2u) / den);
    return steer_deg < 0 ? position : -position;
}
