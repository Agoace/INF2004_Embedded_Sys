/*
 * Same-tick retarget for motor_ctl (opt-in, MOTION_SAME_TICK=1).
 *
 * Problem: each period mc_tick() runs with the old target, then the vehicle
 * task sets a new one about 2 ms later; the new target waits about 8 ms for
 * the next tick. mc_retarget() redoes this period's control step with the
 * new target and the same measured speed, so the result equals a tick run
 * with the new target: one rise step and one integral step per period.
 *
 * Contract:
 *   - Call mc_same_tick_tick() in place of mc_tick().
 *   - Call mc_same_tick_close() before mc_init(), mc_stop(), mc_estop() and
 *     mc_clear_faults(). A closed window stays closed until the next tick.
 *   - After mc_set_speed(), call mc_retarget(); write *out if it returns
 *     true. It returns false (out unchanged) when the window is closed,
 *     after a tick that set STALE, or while a fault is latched. A stall seen
 *     by the retarget latches and gives duty 0.
 *
 * The default build does not use this file, so default images do not
 * change. See docs/s2-motion/MOTION.md#same-tick-retarget-opt-in.
 */
#ifndef MOTOR_CTL_SAME_TICK_H
#define MOTOR_CTL_SAME_TICK_H

#include "motor_ctl.h"

typedef struct {
    mc_wheel_t pre[MC_WHEELS]; /* wheel state before this tick's control */
    bool open;                 /* a retarget may redo this tick */
} mc_same_tick_t;

void mc_same_tick_tick(mc_same_tick_t *st, mc_t *mc,
                       const int32_t counts[MC_WHEELS], uint32_t now_ms,
                       mc_output_t *out);
void mc_same_tick_close(mc_same_tick_t *st);
bool mc_retarget(mc_same_tick_t *st, mc_t *mc, mc_output_t *out);

#endif
