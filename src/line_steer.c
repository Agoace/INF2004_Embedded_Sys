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
    const int32_t base = s->cfg.base_pct;
    const int32_t max = s->cfg.max_pct;
    const int32_t change = s->has_previous ? (int32_t)position - s->previous : 0;
    const int32_t turn =
        (int32_t)s->cfg.gain_pct * position + (int32_t)s->cfg.kd_pct * change;
    const int32_t delta = div_10000_rounded(base * turn);

    const int32_t trim = s->cfg.right_trim_pct;
    int32_t left = base + delta + (trim < 0 ? -trim : 0);
    int32_t right = base - delta + (trim > 0 ? trim : 0);
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
