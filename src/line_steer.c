#include "line_steer.h"

void steer_init(steer_t *s, const steer_config_t *cfg) {
    s->cfg = *cfg;
    if (s->cfg.max_pct > 100u) {
        s->cfg.max_pct = 100u;
    }
    steer_reset(s);
}

void steer_reset(steer_t *s) {
    s->previous = 0;
    s->has_previous = false;
}

/* x / 10000, rounded to the nearest whole number (half away from zero). */
static int32_t div_10000_rounded(int32_t x) {
    return x >= 0 ? (x + 5000) / 10000 : -((-x + 5000) / 10000);
}

static int32_t apply_floor(int32_t duty, int32_t min_pct) {
    if (duty < 0) {
        return 0;
    }
    return duty < min_pct ? 0 : duty;
}

/* The steering rule, in any unit (duty % or mm/s). Updates the previous
 * position. */
typedef struct {
    int32_t base, min, max, trim;
    int32_t gain, kd, far_from, far_gain;
} rule_t;

static void steer_rule(const rule_t *r, int8_t *previous, bool *has_previous, int8_t position,
                       int32_t *left_out, int32_t *right_out) {
    const int32_t min = r->min;
    const int32_t max = r->max;
    const int32_t speed = r->base > min ? r->base - min : 0;
    int32_t change = *has_previous ? (int32_t)position - *previous : 0;
    if (change > STEER_MAX_CHANGE) change = STEER_MAX_CHANGE;
    if (change < -STEER_MAX_CHANGE) change = -STEER_MAX_CHANGE;
    int32_t turn = r->gain * position + r->kd * change;
    const int32_t far = r->far_from;
    if (position > far) {
        turn += r->far_gain * (position - far);
    } else if (position < -far) {
        turn -= r->far_gain * (-position - far);
    }
    const int32_t delta = div_10000_rounded(speed * turn);

    /* Wheel speed: min is speed zero. */
    const int32_t left_speed = speed + delta;
    const int32_t right_speed = speed - delta;
    int32_t left = left_speed > 0 ? min + left_speed : 0;
    int32_t right = right_speed > 0 ? min + right_speed : 0;

    const int32_t trim = r->trim;
    if (trim > 0 && right > 0) right += trim;
    if (trim < 0 && left > 0) left -= trim;
    if (left > max) {
        right -= left - max;
        left = max;
    }
    if (right > max) {
        left -= right - max;
        right = max;
    }
    *left_out = apply_floor(left, min);
    *right_out = apply_floor(right, min);
    *previous = position;
    *has_previous = true;
}

steer_word_t steer_update(steer_t *s, int8_t position) {
    const rule_t r = {s->cfg.base_pct, s->cfg.min_pct, s->cfg.max_pct, s->cfg.right_trim_pct,
                      s->cfg.gain_pct, s->cfg.kd_pct, s->cfg.far_from, s->cfg.far_gain_pct};
    int32_t left, right;
    steer_rule(&r, &s->previous, &s->has_previous, position, &left, &right);
    return (steer_word_t)(((uint32_t)left & STEER_DUTY_MASK) |
                          (((uint32_t)right & STEER_DUTY_MASK) << STEER_RIGHT_SHIFT));
}

void steer_mm_init(steer_mm_t *s, const steer_mm_config_t *cfg) {
    s->cfg = *cfg;
    if (s->cfg.max_mm_s > STEER_MM_MAX) {
        s->cfg.max_mm_s = STEER_MM_MAX;
    }
    steer_mm_reset(s);
}

void steer_mm_reset(steer_mm_t *s) {
    s->previous = 0;
    s->has_previous = false;
}

steer_mm_word_t steer_mm_update(steer_mm_t *s, int8_t position) {
    const rule_t r = {s->cfg.base_mm_s, s->cfg.min_mm_s, s->cfg.max_mm_s, 0,
                      s->cfg.gain_pct, s->cfg.kd_pct, s->cfg.far_from, s->cfg.far_gain_pct};
    int32_t left, right;
    steer_rule(&r, &s->previous, &s->has_previous, position, &left, &right);
    return (steer_mm_word_t)(((uint32_t)left & STEER_MM_MASK) |
                             (((uint32_t)right & STEER_MM_MASK) << STEER_MM_RIGHT_SHIFT));
}

steer_mm_word_t steer_mm_straight(steer_mm_t *s) {
    steer_mm_reset(s);
    const steer_mm_word_t w = steer_mm_update(s, 0);
    steer_mm_reset(s);
    return w;
}

steer_word_t steer_straight(steer_t *s) {
    steer_reset(s);
    const steer_word_t w = steer_update(s, 0);
    steer_reset(s);
    return w;
}

void steer_set_config(steer_t *s, const steer_config_t *cfg) {
    const int8_t previous = s->previous;
    const bool has_previous = s->has_previous;
    steer_init(s, cfg);
    s->previous = previous;
    s->has_previous = has_previous;
}

void steer_calm_init(steer_calm_t *c) {
    c->calm = true;
    c->near = false;
    c->near_ms = 0u;
}

bool steer_calm_update(steer_calm_t *c, int8_t position, bool trigger, uint32_t now_ms) {
    const int a = position < 0 ? -(int)position : (int)position;
    if (trigger || a >= STEER_CALM_ENTER) {
        c->calm = true;
        c->near = false;
        return true;
    }
    if (!c->calm) {
        return false;
    }
    if (a > STEER_CALM_NEAR) {
        c->near = false;
        return true;
    }
    if (!c->near) {
        c->near = true;
        c->near_ms = now_ms;
    }
    if ((uint32_t)(now_ms - c->near_ms) >= STEER_CALM_HOLD_MS) {
        c->calm = false;
        c->near = false;
    }
    return c->calm;
}
