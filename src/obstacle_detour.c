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

/* a x 1000 / b1000, rounded up (b1000 > 0). */
static int32_t div_up(int32_t a, int b1000) {
    return (int32_t)(((int64_t)a * 1000 + b1000 - 1) / b1000);
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

/* OUT: the lane and the OUT length from the edge. */
static void plan_out(dt_t *dt) {
    const int t = dt->cfg.turn_deg;
    dt->lane_y = dt->edge_y + dt->cfg.sweep_mm + dt->cfg.margin_mm;
    dt->leg_mm = (uint32_t)div_up(dt->lane_y, dt_sin1000(t));
}

/* PASS length: from the robot (x at the start of PASS) to the turn point C,
 * which is sweep + margin past the farthest obstacle point. */
static void plan_pass(dt_t *dt) {
    int32_t far = dt->front_x + dt->cfg.depth_mm;
    if (dt->seen_x > far) {
        far = dt->seen_x;
    }
    const int32_t c = far + dt->cfg.sweep_mm + dt->cfg.margin_mm;
    dt->leg_mm = c > dt->x ? (uint32_t)(c - dt->x) : 0u;
}

static void begin(dt_t *dt, dt_step_t step, uint32_t now_ms) {
    dt->step = (uint8_t)step;
    dt->l0 = dt->l;
    dt->r0 = dt->r;
    dt->step_ms = now_ms;
    const int t = dt->cfg.turn_deg;
    int servo = dt->servo_deg;
    switch (step) {
    case DT_OUT:
        dt->heading = (int16_t)t;
        servo = -dt->cfg.look_deg;
        break;
    case DT_PASS:
        dt->heading = 0;
        plan_pass(dt);
        servo = -dt->cfg.look_deg;
        break;
    case DT_IN:
        dt->heading = (int16_t)-t;
        /* Guard: twice the planned IN side. */
        dt->leg_mm = 2u * (uint32_t)div_up(dt->lane_y, dt_sin1000(t));
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
             unsigned int free_step, int32_t left, int32_t right, uint32_t now_ms) {
    *dt = (dt_t){0};
    dt->cfg = *cfg;
    dt->cfg.turn_deg = (uint8_t)clamp(dt->cfg.turn_deg, 10, 80);
    dt->cfg.look_deg = (uint8_t)clamp(dt->cfg.look_deg, 0, DT_SERVO_BIAS);
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
    /* Obstacle: front face at the sensor + distance; extent toward S from
     * the first free scan angle (5 degrees a step; the beam is wide, so
     * this is on the safe side). */
    const int edge_deg = clamp((int)free_step * 5, 0, 30);
    dt->front_x = (int32_t)cfg->sensor_ahead_mm + obstacle_mm;
    dt->edge_y = (int32_t)obstacle_mm * dt_sin1000(edge_deg) / dt_cos1000(edge_deg);
    plan_out(dt);

    /* A must keep the obstacle outside the turn circle, and the A-B side
     * must pass the front corner (front_x, edge_y) with half width +
     * margin: front_x sin t - edge_y cos t >= half width + margin. */
    const int t = dt->cfg.turn_deg;
    const int32_t keep = (int32_t)dt->cfg.half_width_mm + dt->cfg.margin_mm;
    int32_t need = (int32_t)dt->cfg.sweep_mm + dt->cfg.margin_mm;
    const int32_t side_need =
        div_up(dt->edge_y * dt_cos1000(t) / 1000 + keep, dt_sin1000(t));
    if (side_need > need) {
        need = side_need;
    }
    dt->heading = 0;
    if (dt->front_x < need) {
        dt->leg_mm = (uint32_t)(need - dt->front_x);
        begin(dt, DT_BACK, now_ms);
        set_servo(dt, 0, now_ms);
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
    const int32_t s = travel_mm(dt);
    const int h = dt->heading;
    const int32_t ax = dt->x + s * dt_cos1000(h) / 1000;
    const int32_t ay = dt->y + s * dt_sin1000(h) / 1000;
    dt->trig_x = ax + (int32_t)dt->cfg.sensor_ahead_mm * dt_cos1000(h) / 1000;
    dt->trig_y = ay + (int32_t)dt->cfg.sensor_ahead_mm * dt_sin1000(h) / 1000;
    dt->trig_ray = (int16_t)(h + dt->servo_deg);
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
    set_servo(dt, dt->servo_deg, now_ms);
    if (ultrasonic_result_status(result) != ULTRASONIC_STATUS_VALID) {
        return; /* no echo or no information */
    }
    const int32_t mm = ultrasonic_result_distance(result);
    if (mm >= dt->cfg.seen_mm) {
        return;
    }
    const int32_t px = dt->trig_x + mm * dt_cos1000(dt->trig_ray) / 1000;
    const int32_t py = dt->trig_y + mm * dt_sin1000(dt->trig_ray) / 1000;
    if (dt->step == DT_OUT) {
        if (py > dt->edge_y) {
            dt->edge_y = py; /* wider than the scan showed: move the lane out */
            plan_out(dt);
        }
    } else if (dt->step == DT_PASS) {
        if (py > dt->lane_y - (int32_t)dt->cfg.half_width_mm - dt->cfg.margin_mm) {
            fail(dt, DT_FAIL_SIDE);
            return;
        }
        if (px > dt->seen_x) {
            dt->seen_x = px; /* deeper than assumed: drive farther */
            plan_pass(dt);   /* dt->x is the start of PASS, like the travel */
        }
    } else if (mm < (int32_t)DT_TOO_NEAR_MM) { /* DT_IN, servo ahead */
        fail(dt, DT_FAIL_TOO_NEAR);
    }
}

/* Turn progress toward S (dir_s +1) or away (-1), in degrees x 100 of
 * counts: compare with turn_deg x counts_per_deg_x100. */
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

    switch (dt_step(dt)) {
    case DT_BACK:
        if (leg_done) {
            dt->front_x += dist; /* the obstacle is farther ahead now */
            begin(dt, DT_TURN_OUT, now_ms);
        }
        break;
    case DT_TURN_OUT:
        if (turned(dt, +1)) {
            begin(dt, DT_OUT, now_ms);
        }
        break;
    case DT_OUT:
        if (leg_done) {
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
        if (leg_done) {
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
