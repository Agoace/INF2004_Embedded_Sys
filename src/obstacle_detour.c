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

static int clamp(int v, int lo, int hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

int dt_sin1000(int deg) {
    deg = clamp(deg, -90, 90);
    return deg < 0 ? -(int)SIN_PERMILLE[-deg] : (int)SIN_PERMILLE[deg];
}

int dt_cos1000(int deg) {
    deg = clamp(deg, -90, 90);
    return (int)SIN_PERMILLE[90 - (deg < 0 ? -deg : deg)];
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

static void set_servo(dt_t *dt, int deg, uint32_t now_ms) {
    deg = clamp(deg, -DT_SERVO_BIAS, DT_SERVO_BIAS);
    const int move = deg > dt->servo_deg ? deg - dt->servo_deg : dt->servo_deg - deg;
    dt->servo_deg = (int8_t)deg;
    dt->settle_ms = now_ms + DT_SETTLE_BASE_MS + DT_SETTLE_PER_DEG_MS * (uint32_t)move;
}

static void begin(dt_t *dt, dt_step_t step, uint32_t now_ms) {
    dt->step = (uint8_t)step;
    dt->l0 = dt->l;
    dt->r0 = dt->r;
    dt->step_ms = now_ms;
    dt->from_mm = 0;
    dt->best_mm = NO_ECHO;
    const int t = dt->cfg.turn_deg;
    int servo = dt->servo_deg;
    switch (step) {
    case DT_OUT:
        dt->heading = (int16_t)t;
        servo = -DT_SERVO_BIAS;
        break;
    case DT_PASS:
        dt->heading = 0; /* servo still 30 degrees toward the obstacle from OUT */
        break;
    case DT_IN:
        dt->heading = (int16_t)-t;
        dt->leg_mm = 2u * dt->out_mm; /* guard */
        servo = 0;
        break;
    case DT_ON_LINE:
        dt->leg_mm = dt->cfg.line_ahead_mm;
        break;
    default:
        break;
    }
    set_servo(dt, servo, now_ms);
}

void dt_init(dt_t *dt, const dt_config_t *cfg, obstacle_side_t side, uint16_t obstacle_mm,
             int32_t left, int32_t right, uint32_t now_ms) {
    *dt = (dt_t){0};
    dt->cfg = *cfg;
    dt->cfg.turn_deg = (uint8_t)clamp(dt->cfg.turn_deg, 10, 80);
    dt->l = dt->l0 = left;
    dt->r = dt->r0 = right;
    dt->start_ms = dt->step_ms = now_ms;
    if (side == OBSTACLE_SIDE_LEFT) {
        dt->side = +1;
    } else if (side == OBSTACLE_SIDE_RIGHT) {
        dt->side = -1;
    } else {
        fail(dt, DT_FAIL_NO_SIDE);
        return;
    }
    /* The obstacle must be outside the turn circle round the rear axle. */
    const int32_t front = (int32_t)cfg->sensor_ahead_mm + obstacle_mm;
    const int32_t need = (int32_t)cfg->sweep_mm + cfg->margin_mm;
    if (front < need) {
        dt->leg_mm = (uint32_t)(need - front);
        begin(dt, DT_BACK, now_ms);
    } else {
        begin(dt, DT_TURN_OUT, now_ms);
    }
}

static bool wants(const dt_t *dt) {
    return dt->step == DT_OUT || dt->step == DT_PASS || dt->step == DT_IN;
}

bool dt_wants_reading(const dt_t *dt, uint32_t now_ms) {
    return wants(dt) && !dt->pending && (int32_t)(now_ms - dt->settle_ms) >= 0;
}

/* Straight distance of this step so far, mm (back: negative). */
static int32_t travel_mm(const dt_t *dt) {
    return dt_counts_to_mm(((dt->l - dt->l0) + (dt->r - dt->r0)) / 2);
}

void dt_triggered(dt_t *dt) {
    dt->pending = true;
    dt->trig_mm = travel_mm(dt);
}

void dt_reading(dt_t *dt, ultrasonic_result_t result, uint32_t now_ms) {
    if (!dt->pending) {
        return;
    }
    dt->pending = false;
    if (!wants(dt)) {
        return;
    }
    if (ultrasonic_result_fault(result)) {
        fail(dt, DT_FAIL_SENSOR);
        return;
    }
    dt->settle_ms = now_ms + DT_SETTLE_BASE_MS;
    if (ultrasonic_result_status(result) != ULTRASONIC_STATUS_VALID) {
        return; /* no echo or no information */
    }
    const uint16_t mm = ultrasonic_result_distance(result);
    if (dt->step == DT_IN) {
        if (mm < DT_TOO_NEAR_MM) {
            fail(dt, DT_FAIL_TOO_NEAR);
        }
        return;
    }
    if (mm < dt->cfg.seen_mm && mm < dt->best_mm) {
        dt->best_mm = mm; /* nearest so far: count again from here */
        dt->from_mm = dt->trig_mm;
    }
}

/* Turn progress toward S (dir_s +1) or away (-1): compare the count
 * difference x 100 with turn_deg x counts_per_deg_x100. */
static bool turned(const dt_t *dt, int dir_s) {
    const int32_t diff = (dt->r - dt->r0) - (dt->l - dt->l0); /* > 0: left */
    const int64_t progress = (int64_t)dt->side * dir_s * diff * 100;
    return progress >= (int64_t)dt->cfg.turn_deg * dt->cfg.counts_per_deg_x100;
}

/* End a straight step: move the pose by the distance driven. */
static void end_straight(dt_t *dt) {
    const int32_t s = travel_mm(dt);
    dt->x += s * dt_cos1000(dt->heading) / 1000;
    dt->y += s * dt_sin1000(dt->heading) / 1000;
}

static dt_word_t word(dt_move_t move, int dir, int servo_robot) {
    return (dt_word_t)((unsigned int)move | (dir < 0 ? DT_DIR_BIT : 0u) |
                       ((unsigned int)(servo_robot + DT_SERVO_BIAS) & DT_SERVO_MASK)
                           << DT_SERVO_SHIFT);
}

dt_word_t dt_update(dt_t *dt, bool line_seen, int32_t left, int32_t right, uint32_t now_ms) {
    dt->l = left;
    dt->r = right;
    const bool active = dt->step < DT_DONE;
    if (active && (uint32_t)(now_ms - dt->start_ms) >= dt->cfg.total_max_ms) {
        fail(dt, DT_FAIL_TIME);
    }
    const bool turning = dt->step == DT_TURN_OUT || dt->step == DT_TURN_BACK ||
                         dt->step == DT_TURN_IN || dt->step == DT_ALIGN;
    if (turning && (uint32_t)(now_ms - dt->step_ms) >= dt->cfg.turn_max_ms) {
        fail(dt, DT_FAIL_TURN_TIME);
    }
    const int32_t s = travel_mm(dt);
    const int32_t dist = s < 0 ? -s : s;
    const bool leg_done = (uint32_t)dist >= dt->leg_mm;
    const bool past = s - dt->from_mm >= (int32_t)dt->cfg.vehicle_mm;

    switch (dt_step(dt)) {
    case DT_BACK:
        if (leg_done) {
            begin(dt, DT_TURN_OUT, now_ms);
        }
        break;
    case DT_TURN_OUT:
        if (turned(dt, +1)) {
            begin(dt, DT_OUT, now_ms);
        }
        break;
    case DT_OUT:
        if (past) {
            dt->out_mm = (uint32_t)s;
            end_straight(dt);
            begin(dt, DT_TURN_BACK, now_ms);
        }
        break;
    case DT_TURN_BACK:
        if (turned(dt, -1)) {
            begin(dt, DT_PASS, now_ms);
        }
        break;
    case DT_PASS:
        if (past) {
            end_straight(dt);
            begin(dt, DT_TURN_IN, now_ms);
        }
        break;
    case DT_TURN_IN:
        if (turned(dt, -1)) {
            begin(dt, DT_IN, now_ms);
        }
        break;
    case DT_IN:
        if (line_seen) {
            end_straight(dt);
            begin(dt, DT_ON_LINE, now_ms);
        } else if (leg_done) {
            fail(dt, DT_FAIL_NO_LINE);
        }
        break;
    case DT_ON_LINE:
        if (leg_done) {
            end_straight(dt);
            begin(dt, DT_ALIGN, now_ms);
        }
        break;
    case DT_ALIGN:
        if (turned(dt, +1)) {
            dt->heading = 0;
            dt->step = DT_DONE;
        }
        break;
    default:
        break;
    }

    const int servo = dt->side * dt->servo_deg; /* S frame to robot (+ left) */
    switch (dt_step(dt)) {
    case DT_BACK:
        return word(DT_MOVE_STRAIGHT, -1, servo);
    case DT_TURN_OUT:
    case DT_ALIGN:
        return word(DT_MOVE_TURN, dt->side, servo);
    case DT_TURN_BACK:
    case DT_TURN_IN:
        return word(DT_MOVE_TURN, -dt->side, servo);
    case DT_OUT:
    case DT_PASS:
    case DT_IN:
    case DT_ON_LINE:
        return word(DT_MOVE_STRAIGHT, +1, servo);
    default:
        return word(DT_MOVE_STOP, +1, 0);
    }
}
