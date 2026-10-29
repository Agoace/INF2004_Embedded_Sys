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

steer_word_t steer_update(steer_t *s, int8_t position) {
    const int32_t min = s->cfg.min_pct;
    const int32_t max = s->cfg.max_pct;
    const int32_t speed = s->cfg.base_pct > min ? s->cfg.base_pct - min : 0;
    int32_t change = s->has_previous ? (int32_t)position - s->previous : 0;
    if (change > STEER_MAX_CHANGE) change = STEER_MAX_CHANGE;
    if (change < -STEER_MAX_CHANGE) change = -STEER_MAX_CHANGE;
    const int32_t turn =
        (int32_t)s->cfg.gain_pct * position + (int32_t)s->cfg.kd_pct * change;
    const int32_t delta = div_10000_rounded(speed * turn);

    /* Wheel speed to duty: min_pct is speed zero. */
    const int32_t left_speed = speed + delta;
    const int32_t right_speed = speed - delta;
    int32_t left = left_speed > 0 ? min + left_speed : 0;
    int32_t right = right_speed > 0 ? min + right_speed : 0;

    const int32_t trim = s->cfg.right_trim_pct;
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
    left = apply_floor(left, s->cfg.min_pct);
    right = apply_floor(right, s->cfg.min_pct);

    s->previous = position;
    s->has_previous = true;
    return (steer_word_t)(((uint32_t)left & STEER_DUTY_MASK) |
                          (((uint32_t)right & STEER_DUTY_MASK) << STEER_RIGHT_SHIFT));
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
