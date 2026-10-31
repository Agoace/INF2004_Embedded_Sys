#include "obstacle_detour.h"

/* sin(0..90 degrees) x 1000, rounded. */
static const uint16_t SIN_PERMILLE[91] = {
    0,   17,  35,  52,  70,  87,  105, 122, 139, 156, 174, 191, 208, 225, 242, 259,
    276, 292, 309, 326, 342, 358, 375, 391, 407, 423, 438, 454, 469, 485, 500, 515,
    530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682, 695, 707, 719, 731,
    743, 755, 766, 777, 788, 799, 809, 819, 829, 839, 848, 857, 866, 875, 883, 891,
    899, 906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
    985, 988, 990, 993, 995, 996, 998, 999, 999, 1000, 1000,
};

#define NO_ECHO 0xFFFFu

int dt_sin1000(int deg) {
    deg %= 360;
    if (deg > 180) {
        deg -= 360;
    } else if (deg < -180) {
        deg += 360;
    }
    const int sign = deg < 0 ? -1 : 1;
    int a = deg < 0 ? -deg : deg; /* 0..180 */
    if (a > 90) {
        a = 180 - a;
    }
    return sign * (int)SIN_PERMILLE[a];
}

int dt_cos1000(int deg) {
    return dt_sin1000(deg + 90);
}

uint32_t dt_mm_to_counts(uint32_t mm) {
    const uint64_t num = (uint64_t)mm * DT_COUNTS_PER_WHEEL_TURN * 1000u;
    return (uint32_t)((num + DT_WHEEL_CIRCUMFERENCE_UM / 2u) / DT_WHEEL_CIRCUMFERENCE_UM);
}

int32_t dt_counts_to_mm(int32_t counts) {
    return (int32_t)((int64_t)counts * DT_WHEEL_CIRCUMFERENCE_UM /
                     ((int64_t)DT_COUNTS_PER_WHEEL_TURN * 1000));
}

static void fail(dt_t *dt, dt_fail_t reason) {
    dt->step = DT_FAILED;
    dt->fail = (uint8_t)reason;
}

static bool is_turn(const dt_t *dt) {
    return dt->step == DT_TURN_AWAY || dt->step == DT_TURN_TOWARD;
}

/* Turn direction of the current step, S frame: +1 toward S. */
static int turn_dir(const dt_t *dt) {
    if (dt->step == DT_TURN_AWAY) {
        return +1;
    }
    if (dt->step == DT_TURN_TOWARD) {
        return dt->backing ? +1 : -1;
    }
    return dt->heading < 0 ? +1 : -1; /* ALIGN: back to 0 */
}

/* Turn of this step toward S, count difference x 100 (S frame). */
static int64_t turn_x100(const dt_t *dt) {
    const int32_t diff = (dt->r - dt->r0) - (dt->l - dt->l0); /* > 0: left */
    return (int64_t)dt->side * diff * 100;
}

/* Turn of this step in its own direction, compared with deg. */
static bool turned(const dt_t *dt, int deg) {
    return turn_x100(dt) * turn_dir(dt) >= (int64_t)deg * dt->cfg.counts_per_deg_x100;
}

/* Turn of this step toward S in whole degrees, rounded. */
static int turn_deg(const dt_t *dt) {
    const int64_t t = turn_x100(dt);
    const int64_t half = dt->cfg.counts_per_deg_x100 / 2;
    return (int)((t + (t < 0 ? -half : half)) / dt->cfg.counts_per_deg_x100);
}

/* Straight distance of this step so far, mm (back: negative). */
static int32_t travel_mm(const dt_t *dt) {
    return dt_counts_to_mm(((dt->l - dt->l0) + (dt->r - dt->r0)) / 2);
}

static void begin(dt_t *dt, dt_step_t step) {
    dt->step = (uint8_t)step;
    dt->pending = false; /* a reading of the last step is not for this one */
    dt->l0 = dt->l;
    dt->r0 = dt->r;
    dt->from_mm = 0;
    dt->best_mm = NO_ECHO;
    dt->looking = false;
    dt->backing = false;
    dt->target_deg = 0u; /* turns look first */
    if (step == DT_ON_LINE) {
        dt->leg_mm = dt->cfg.line_ahead_mm;
    }
}

/* End a straight: move the pose by the distance driven. */
static void end_straight(dt_t *dt) {
    const int32_t s = travel_mm(dt);
    dt->x += s * dt_cos1000(dt->heading) / 1000;
    dt->y += s * dt_sin1000(dt->heading) / 1000;
}

/* End a turn: add it to the heading. */
static void end_turn(dt_t *dt, dt_step_t next) {
    dt->heading = (int16_t)(dt->heading + turn_deg(dt));
    begin(dt, next);
}

void dt_init(dt_t *dt, const dt_config_t *cfg, obstacle_side_t side, uint16_t obstacle_mm,
             int32_t left, int32_t right, uint32_t now_ms) {
    *dt = (dt_t){0};
    dt->cfg = *cfg;
    if (dt->cfg.turn_step_deg == 0u) {
        dt->cfg.turn_step_deg = 1u;
    }
    if (dt->cfg.counts_per_deg_x100 == 0u) {
        dt->cfg.counts_per_deg_x100 = 1u;
    }
    dt->l = dt->l0 = left;
    dt->r = dt->r0 = right;
    dt->start_ms = now_ms;
    if (side == OBSTACLE_SIDE_LEFT) {
        dt->side = +1;
    } else if (side == OBSTACLE_SIDE_RIGHT) {
        dt->side = -1;
    } else {
        fail(dt, DT_FAIL_NO_SIDE);
        return;
    }
    /* The servo goes from the centre to 30 degrees toward the obstacle. */
    dt->servo_deg = -DT_SERVO_BIAS;
    dt->settle_ms = now_ms + DT_SETTLE_BASE_MS + DT_SETTLE_PER_DEG_MS * DT_SERVO_BIAS;
    /* The obstacle must be outside the turn circle round the rear axle. */
    const int32_t front = (int32_t)cfg->sensor_ahead_mm + obstacle_mm;
    const int32_t need = (int32_t)cfg->sweep_mm + cfg->margin_mm;
    if (front < need) {
        dt->leg_mm = (uint32_t)(need - front);
        begin(dt, DT_BACK);
    } else {
        begin(dt, DT_TURN_AWAY);
    }
}

bool dt_wants_reading(const dt_t *dt, uint32_t now_ms) {
    const bool step_wants = dt->step == DT_STRAIGHT || (is_turn(dt) && dt->looking);
    return step_wants && !dt->pending && (int32_t)(now_ms - dt->settle_ms) >= 0;
}

void dt_triggered(dt_t *dt) {
    dt->pending = true;
    dt->trig_mm = travel_mm(dt);
}

/* A look in a turn found nothing to stop for: turn one more step. */
static void next_look(dt_t *dt, uint8_t max_deg) {
    dt->looking = false;
    dt->target_deg = (uint8_t)(dt->target_deg + dt->cfg.turn_step_deg);
    if (dt->target_deg > max_deg) {
        fail(dt, DT_FAIL_TURN_LIMIT);
    }
}

void dt_reading(dt_t *dt, ultrasonic_result_t result, uint32_t now_ms) {
    if (!dt->pending) {
        return;
    }
    dt->pending = false;
    if (dt->step != DT_STRAIGHT && !(is_turn(dt) && dt->looking)) {
        return;
    }
    if (ultrasonic_result_fault(result)) {
        fail(dt, DT_FAIL_SENSOR);
        return;
    }
    dt->settle_ms = now_ms + DT_SETTLE_BASE_MS;
    const ultrasonic_status_t status = ultrasonic_result_status(result);
    if (status != ULTRASONIC_STATUS_VALID && status != ULTRASONIC_STATUS_NO_RETURN) {
        return; /* no information: read again */
    }
    const uint16_t mm = ultrasonic_result_distance(result);
    const bool seen = status == ULTRASONIC_STATUS_VALID && mm < dt->cfg.seen_mm;
    if (dt->step == DT_STRAIGHT) {
        if (status == ULTRASONIC_STATUS_VALID && mm < DT_TOO_NEAR_MM) {
            fail(dt, DT_FAIL_TOO_NEAR);
        } else if (seen && mm < dt->best_mm) {
            dt->best_mm = mm; /* nearest so far: count again from here */
            dt->from_mm = dt->trig_mm;
        }
    } else if (dt->step == DT_TURN_AWAY) {
        if (seen) {
            dt->seen_once = true;
            next_look(dt, dt->cfg.away_max_deg);
        } else if (dt->seen_once) {
            end_turn(dt, DT_STRAIGHT); /* past the edge */
        } else {
            next_look(dt, dt->cfg.away_max_deg);
        }
    } else if (seen) { /* DT_TURN_TOWARD */
        const int toward = -turn_deg(dt);
        dt->last_turned_deg = toward > 0 ? toward : 0;
        int back = dt->cfg.back_off_deg;
        if (dt->cfg.back_off_fn) {
            back = dt->cfg.back_off_fn(dt->last_turned_deg);
            back = back < 0 ? 0 : back > 180 ? 180 : back;
        }
        dt->last_back_deg = back;
        if (back == 0) {
            end_turn(dt, DT_STRAIGHT);
        } else {
            dt->looking = false;
            dt->backing = true; /* turn back, away from the obstacle */
            dt->back_x100 = (int32_t)turn_x100(dt) +
                            (int32_t)back * dt->cfg.counts_per_deg_x100;
        }
    } else {
        next_look(dt, dt->cfg.toward_max_deg);
    }
}

static dt_word_t word(dt_move_t move, int dir, int servo_robot) {
    return (dt_word_t)((unsigned int)move | (dir < 0 ? DT_DIR_BIT : 0u) |
                       ((unsigned int)(servo_robot + DT_SERVO_BIAS) & DT_SERVO_MASK)
                           << DT_SERVO_SHIFT);
}

dt_word_t dt_update(dt_t *dt, bool line_seen, int32_t left, int32_t right, uint32_t now_ms) {
    dt->l = left;
    dt->r = right;
    if (!line_seen) {
        dt->armed = true;
    }
    if (dt->step < DT_DONE && (uint32_t)(now_ms - dt->start_ms) >= dt->cfg.total_max_ms) {
        fail(dt, DT_FAIL_TIME);
    }
    const int32_t s = travel_mm(dt);
    const int32_t dist = s < 0 ? -s : s;

    switch (dt_step(dt)) {
    case DT_BACK:
        if ((uint32_t)dist >= dt->leg_mm) {
            begin(dt, DT_TURN_AWAY);
        }
        break;
    case DT_TURN_TOWARD:
        if (dt->backing && turn_x100(dt) >= dt->back_x100) {
            end_turn(dt, DT_STRAIGHT);
        }
        break;
    case DT_STRAIGHT:
        if (dt->armed && line_seen) {
            end_straight(dt);
            begin(dt, DT_ON_LINE);
        } else if (s - dt->from_mm >= (int32_t)dt->cfg.vehicle_mm) {
            end_straight(dt);
            ++dt->straights;
            if (dt->straights >= dt->cfg.max_straights) {
                fail(dt, DT_FAIL_NO_LINE);
            } else {
                begin(dt, DT_TURN_TOWARD);
            }
        }
        break;
    case DT_ON_LINE:
        if ((uint32_t)dist >= dt->leg_mm) {
            end_straight(dt);
            begin(dt, DT_ALIGN);
        }
        break;
    case DT_ALIGN: {
        const int need = dt->heading < 0 ? -dt->heading : dt->heading;
        if (turned(dt, need)) {
            dt->heading = 0;
            dt->step = DT_DONE;
        }
        break;
    }
    default:
        break;
    }

    /* After the step switch, so that a new turn looks before it moves. */
    if (is_turn(dt) && !dt->looking && !dt->backing && turned(dt, dt->target_deg)) {
        dt->looking = true; /* stop, let the robot settle, then read */
        const uint32_t look = now_ms + dt->cfg.look_ms;
        if ((int32_t)(look - dt->settle_ms) > 0) {
            dt->settle_ms = look;
        }
    }

    const int servo = dt->side * dt->servo_deg; /* S frame to robot (+ left) */
    switch (dt_step(dt)) {
    case DT_BACK:
        return word(DT_MOVE_STRAIGHT, -1, servo);
    case DT_TURN_AWAY:
    case DT_TURN_TOWARD:
        if (dt->looking) {
            return word(DT_MOVE_STOP, +1, servo);
        }
        return word(DT_MOVE_TURN, dt->side * turn_dir(dt), servo);
    case DT_ALIGN:
        return word(DT_MOVE_TURN, dt->side * turn_dir(dt), servo);
    case DT_STRAIGHT:
    case DT_ON_LINE:
        return word(DT_MOVE_STRAIGHT, +1, servo);
    default:
        return word(DT_MOVE_STOP, +1, 0);
    }
}
