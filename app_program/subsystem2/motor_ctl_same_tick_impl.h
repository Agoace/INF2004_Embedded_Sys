/*
 * MOTION_SAME_TICK=1 only: force-included (-include) at the top of
 * motor_ctl.c, so that motor_ctl.c itself does not change. The code below
 * uses wheel_control(), the static control step defined in motor_ctl.c.
 */
#ifndef MOTOR_CTL_SAME_TICK_IMPL_H
#define MOTOR_CTL_SAME_TICK_IMPL_H

#include "motor_ctl_same_tick.h"

static int16_t wheel_control(const mc_config_t *c, unsigned int i,
                             mc_wheel_t *w, bool *stalled);

void mc_same_tick_tick(mc_same_tick_t *st, mc_t *mc,
                       const int32_t counts[MC_WHEELS], uint32_t now_ms,
                       mc_output_t *out)
{
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        st->pre[i] = mc->w[i];
    }
    mc_tick(mc, counts, now_ms, out);
    st->open = (mc->faults & (MC_FAULT_LATCHED | MC_FAULT_STALE)) == 0u;
}

void mc_same_tick_close(mc_same_tick_t *st)
{
    st->open = false;
}

bool mc_retarget(mc_same_tick_t *st, mc_t *mc, mc_output_t *out)
{
    if (!st->open || (mc->faults & MC_FAULT_LATCHED) != 0u) {
        return false;
    }
    bool stalled[MC_WHEELS] = {false, false};
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        mc_wheel_t w = st->pre[i];
        /* Keep this tick's measurement and the new target. */
        w.last_count = mc->w[i].last_count;
        w.speed_cpt = mc->w[i].speed_cpt;
        w.target_q8 = mc->w[i].target_q8;
        w.duty = wheel_control(&mc->cfg, i, &w, &stalled[i]);
        mc->w[i] = w;
    }
    if (stalled[MC_LEFT] || stalled[MC_RIGHT]) {
        const unsigned int left = stalled[MC_LEFT] ? MC_FAULT_STALL_LEFT : 0u;
        const unsigned int right =
            stalled[MC_RIGHT] ? MC_FAULT_STALL_RIGHT : 0u;
        mc->faults |= (uint8_t)(left | right);
        st->open = false;
    }
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        out->duty_permille[i] =
            (mc->faults & MC_FAULT_LATCHED) != 0u ? 0 : mc->w[i].duty;
    }
    out->servo_us = mc->servo_us;
    return true;
}

#endif
