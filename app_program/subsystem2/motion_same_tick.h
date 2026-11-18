/*
 * MOTION_SAME_TICK=1 only: force-included (-include) at the top of
 * motion.c, so that motion.c itself does not change. Each macro below
 * routes one motor_ctl call in motion.c through the same-tick window:
 *   - mc_tick: tick and open the window (motion task, under mtx_motion).
 *   - mc_set_speed: set the target, then retarget and write the motor at
 *     once (motion_set_speed, under mtx_motion; it never feeds the watchdog).
 *   - mc_init/mc_stop/mc_estop/mc_clear_faults: close the window first.
 * Stop and ESTOP keep the same mutex order as the default build.
 */
#ifndef MOTION_SAME_TICK_H
#define MOTION_SAME_TICK_H

#include "motor_ctl_same_tick.h"
#include "drv_motor_hw.h"

static inline mc_same_tick_t *motion_same_tick(void)
{
    static mc_same_tick_t window;
    return &window;
}

static inline void motion_same_tick_set(mc_t *mc, int16_t left, int16_t right,
                                        uint32_t now_ms)
{
    mc_set_speed(mc, left, right, now_ms);
    mc_output_t out;
    if (mc_retarget(motion_same_tick(), mc, &out)) {
        drv_motor_hw_write(out.duty_permille, out.servo_us);
    }
}

#define mc_tick(mc, counts, now, out)                                          \
    mc_same_tick_tick(motion_same_tick(), (mc), (counts), (now), (out))
#define mc_set_speed(mc, left, right, now)                                     \
    motion_same_tick_set((mc), (left), (right), (now))
#define mc_init(mc, cfg, counts, now)                                          \
    (mc_same_tick_close(motion_same_tick()),                                   \
     mc_init((mc), (cfg), (counts), (now)))
#define mc_stop(mc)  (mc_same_tick_close(motion_same_tick()), mc_stop(mc))
#define mc_estop(mc) (mc_same_tick_close(motion_same_tick()), mc_estop(mc))
#define mc_clear_faults(mc)                                                    \
    (mc_same_tick_close(motion_same_tick()), mc_clear_faults(mc))

#endif
