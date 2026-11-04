#include "motor_ctl.h"

static int32_t abs32(int32_t v) {
    return v < 0 ? -v : v;
}

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

static void reset_wheel(mc_wheel_t *w) {
    w->target_q8 = 0;
    w->setpoint_q8 = 0;
    w->integral_q8 = 0;
    w->duty = 0;
    w->kicking = false;
    w->kick_ms = 0u;
    w->kick_travel = 0;
    w->still_ms = 0u;
}

void mc_init(mc_t *mc, const mc_config_t *cfg, const int32_t counts[MC_WHEELS], uint32_t now_ms) {
    mc->cfg = *cfg;
    if (mc->cfg.max_permille > MC_PERMILLE_MAX) {
        mc->cfg.max_permille = MC_PERMILLE_MAX;
    }
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        reset_wheel(&mc->w[i]);
        mc->w[i].origin = counts[i];
        mc->w[i].last_count = counts[i];
        mc->w[i].speed_cpt = 0;
    }
    mc->command_ms = now_ms;
    mc->last_tick_ms = now_ms;
    mc->has_command = false;
    mc->faults = 0u;
    mc->servo_deg = 0;
    mc->servo_us = cfg->servo_centre_us;
    mc->servo_settled_ms = now_ms + cfg->servo_settle_base_ms;
}

/* mm/s to counts per tick, Q8. */
static int32_t speed_q8(const mc_config_t *cfg, int16_t mm_s) {
    const int64_t q8 = (int64_t)mm_s * cfg->counts_per_m * cfg->tick_ms * 256 / 1000000;
    return (int32_t)q8;
}

void mc_set_speed(mc_t *mc, int16_t left_mm_s, int16_t right_mm_s, uint32_t now_ms) {
    /* While a fault is latched the output is 0, and mc_clear_faults()
     * stops the wheels, so a speed sent now has no effect. */
    mc->w[MC_LEFT].target_q8 = speed_q8(&mc->cfg, left_mm_s);
    mc->w[MC_RIGHT].target_q8 = speed_q8(&mc->cfg, right_mm_s);
    mc->command_ms = now_ms;
    mc->has_command = true;
    mc->faults &= (uint8_t)~MC_FAULT_STALE;
}

void mc_stop(mc_t *mc) {
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        reset_wheel(&mc->w[i]);
    }
    mc->has_command = false;
}

void mc_estop(mc_t *mc) {
    mc->faults |= MC_FAULT_ESTOP;
    mc_stop(mc);
}

void mc_clear_faults(mc_t *mc) {
    mc->faults &= (uint8_t)~MC_FAULT_LATCHED;
    mc_stop(mc);
}

int mc_set_servo(mc_t *mc, int deg, uint32_t now_ms) {
    const mc_config_t *c = &mc->cfg;
    const int32_t per_x10 = (int32_t)c->servo_us_per_deg_x10;
    /* Clamp the angle so that the pulse stays in the safe range. */
    const int32_t max_left = per_x10 ? ((int32_t)(c->servo_max_us - c->servo_centre_us) * 10) / per_x10 : 0;
    const int32_t max_right = per_x10 ? ((int32_t)(c->servo_centre_us - c->servo_min_us) * 10) / per_x10 : 0;
    const int32_t d = clamp32(deg, -max_right, max_left);
    const int32_t change = abs32(d - mc->servo_deg);
    const int32_t us = (int32_t)c->servo_centre_us +
                       (d >= 0 ? (d * per_x10 + 5) / 10 : -((-d * per_x10 + 5) / 10));
    mc->servo_us = (uint16_t)us;
    if (change != 0) {
        mc->servo_settled_ms = now_ms + c->servo_settle_base_ms +
                               (uint32_t)((change * c->servo_settle_per_deg_x10_ms + 5) / 10);
    }
    mc->servo_deg = (int8_t)d;
    return (int)d;
}

/* One wheel. Returns the duty in permille; sets *stalled. */
static int16_t wheel_control(const mc_config_t *c, unsigned int i, mc_wheel_t *w, bool *stalled) {
    const int32_t prev = w->setpoint_q8;
    const int32_t want = w->target_q8;
    const int32_t accel = (int32_t)c->accel_cpt_q8;
    const bool slowing = ((want >= 0) == (prev >= 0)) && abs32(want) < abs32(prev);
    if (accel == 0 || slowing || abs32(want - prev) <= accel) {
        w->setpoint_q8 = want;
    } else {
        w->setpoint_q8 = prev + (want > prev ? accel : -accel);
    }
    const int32_t sp = w->setpoint_q8;
    if (sp == 0) {
        /* The next start arms the kick again; that clears I and the
         * stall time. */
        return 0;
    }
    const int32_t sign = sp > 0 ? 1 : -1;
    const unsigned int dir = sp > 0 ? MC_FORWARD : MC_REVERSE;
    if (prev == 0 || (prev > 0) != (sp > 0)) {
        w->kicking = true;
        w->kick_ms = 0u;
        w->kick_travel = 0;
        w->integral_q8 = 0;
    }
    const int32_t max = (int32_t)c->max_permille;
    const int32_t err_q8 = sp - (int32_t)w->speed_cpt * 256;
    const int32_t ff = sign * ((int32_t)c->ff_offset_permille[i][dir] +
                               (int32_t)(((int64_t)c->ff_slope_q8[i][dir] * abs32(sp)) / 65536));
    const int32_t p = (int32_t)(((int64_t)c->kp_q8 * err_q8) / 65536);
    int32_t out = ff + p + w->integral_q8 / 256;
    /* Anti-windup: integrate only when not saturated in the error sign,
     * and not while the set point still rises or the start kick runs: the
     * wheel lags then, and I would wind up and overshoot (bench run B1). */
    const bool sat_hi = out >= max && err_q8 > 0;
    const bool sat_lo = out <= -max && err_q8 < 0;
    const bool hold = w->kicking || sp != want;
    if (!sat_hi && !sat_lo && !hold) {
        const int32_t lim = (int32_t)c->integral_max_permille * 256;
        w->integral_q8 = clamp32(w->integral_q8 + (int32_t)(((int64_t)c->ki_q8 * err_q8) / 256),
                                 -lim, lim);
    }
    out = clamp32(out, -max, max);
    if (w->kicking) {
        w->kick_travel += abs32(w->speed_cpt);
        w->kick_ms = (uint16_t)(w->kick_ms + c->tick_ms);
        if (w->kick_travel >= (int32_t)c->kick_counts) {
            w->kicking = false;
        } else if (w->kick_ms > c->kick_timeout_ms) {
            *stalled = true;
            return 0;
        } else if (out * sign < (int32_t)c->kick_permille) {
            out = sign * (int32_t)c->kick_permille;
        }
    }
    if (!w->kicking && abs32(out) >= (int32_t)c->stall_min_permille && w->speed_cpt == 0) {
        w->still_ms = (uint16_t)(w->still_ms + c->tick_ms);
        if (w->still_ms >= c->stall_timeout_ms) {
            *stalled = true;
            return 0;
        }
    } else {
        w->still_ms = 0u;
    }
    return (int16_t)out;
}

void mc_tick(mc_t *mc, const int32_t counts[MC_WHEELS], uint32_t now_ms, mc_output_t *out) {
    const uint32_t elapsed = now_ms - mc->last_tick_ms;
    mc->last_tick_ms = now_ms;
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        int32_t delta = counts[i] - mc->w[i].last_count;
        mc->w[i].last_count = counts[i];
        /* Speed in counts per nominal tick, if the call came late or early. */
        if (elapsed != 0u && elapsed != mc->cfg.tick_ms) {
            delta = (int32_t)(((int64_t)delta * mc->cfg.tick_ms) / (int64_t)elapsed);
        }
        mc->w[i].speed_cpt = (int16_t)clamp32(delta, INT16_MIN, INT16_MAX);
    }
    if (mc->has_command && now_ms - mc->command_ms >= mc->cfg.stale_ms) {
        mc->faults |= MC_FAULT_STALE;
        mc_stop(mc);
    }
    bool stalled[MC_WHEELS] = {false, false};
    if (!(mc->faults & MC_FAULT_LATCHED)) {
        for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
            mc->w[i].duty = wheel_control(&mc->cfg, i, &mc->w[i], &stalled[i]);
        }
        if (stalled[MC_LEFT] || stalled[MC_RIGHT]) {
            /* Latched: the output below is 0 until mc_clear_faults(). */
            mc->faults |= (uint8_t)((stalled[MC_LEFT] ? MC_FAULT_STALL_LEFT : 0u) |
                                    (stalled[MC_RIGHT] ? MC_FAULT_STALL_RIGHT : 0u));
        }
    }
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        out->duty_permille[i] = (mc->faults & MC_FAULT_LATCHED) ? 0 : mc->w[i].duty;
    }
    out->servo_us = mc->servo_us;
}

int32_t mc_travel_mm(const mc_t *mc, unsigned int wheel) {
    const int64_t c = (int64_t)mc->w[wheel].last_count - mc->w[wheel].origin;
    return (int32_t)(c * 1000 / (int64_t)mc->cfg.counts_per_m);
}

int16_t mc_speed_mm_s(const mc_t *mc, unsigned int wheel) {
    const int64_t v = (int64_t)mc->w[wheel].speed_cpt * 1000000 /
                      ((int64_t)mc->cfg.counts_per_m * mc->cfg.tick_ms);
    return (int16_t)clamp32((int32_t)v, INT16_MIN, INT16_MAX);
}

int32_t mc_heading_cdeg(const mc_t *mc) {
    const int64_t dl = (int64_t)mc->w[MC_LEFT].last_count - mc->w[MC_LEFT].origin;
    const int64_t dr = (int64_t)mc->w[MC_RIGHT].last_count - mc->w[MC_RIGHT].origin;
    return (int32_t)((dr - dl) * 10000 / (int64_t)mc->cfg.counts_per_deg_x100);
}

bool mc_servo_settled(const mc_t *mc, uint32_t now_ms) {
    return (int32_t)(now_ms - mc->servo_settled_ms) >= 0;
}

int mc_servo_deg(const mc_t *mc) {
    return mc->servo_deg;
}
