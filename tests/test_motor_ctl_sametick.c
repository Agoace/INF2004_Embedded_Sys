/* Host test for mc_retarget() (MOTION_SAME_TICK=1): a speed set after a tick
 * is applied in the same period. Property: tick(old) + set(new) +
 * retarget == set(new) + tick, for the wheel state, faults and output.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../app_program/subsystem2/motor_ctl_same_tick.h"
#include "motor_ctl_robot.h"

static mc_same_tick_t win;

/* The motion.c hooks: close the window before stop, estop, clear. */
static void stop(mc_t *mc, mc_same_tick_t *w)
{
    mc_same_tick_close(w);
    mc_stop(mc);
}

static void estop(mc_t *mc, mc_same_tick_t *w)
{
    mc_same_tick_close(w);
    mc_estop(mc);
}

static void clear(mc_t *mc, mc_same_tick_t *w)
{
    mc_same_tick_close(w);
    mc_clear_faults(mc);
}

static int failures;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            printf("FAIL line %d: ", __LINE__);                                \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
        }                                                                      \
    } while (0)

static const mc_config_t CFG = MC_ROBOT_CONFIG;

static bool same_wheel(const mc_wheel_t *a, const mc_wheel_t *b)
{
    return a->target_q8 == b->target_q8 && a->setpoint_q8 == b->setpoint_q8 &&
           a->integral_q8 == b->integral_q8 && a->last_count == b->last_count &&
           a->origin == b->origin && a->kick_travel == b->kick_travel &&
           a->speed_cpt == b->speed_cpt && a->duty == b->duty &&
           a->kick_ms == b->kick_ms && a->still_ms == b->still_ms &&
           a->kicking == b->kicking;
}

static bool same_out(const mc_output_t *a, const mc_output_t *b)
{
    return a->duty_permille[0] == b->duty_permille[0] &&
           a->duty_permille[1] == b->duty_permille[1] &&
           a->servo_us == b->servo_us;
}

static uint32_t seed = 12345u;

static uint32_t rnd(uint32_t n)
{
    seed = seed * 1103515245u + 12345u;
    return (seed >> 8) % n;
}

/* Reference: set then tick. Same tick: tick then set and retarget. */
static void property(void)
{
    mc_t ref, st;
    const int32_t c0[2] = {1000, 2000};
    int32_t c[2] = {1000, 2000};
    uint32_t now = 1000u;
    mc_init(&ref, &CFG, c0, now);
    mc_init(&st, &CFG, c0, now);
    mc_same_tick_close(&win);
    int16_t old_l = 0, old_r = 0;
    unsigned int applied = 0u, latched = 0u, kept = 0u;
    for (unsigned int k = 0u; k < 20000u; ++k) {
        now += CFG.tick_ms;
        /* Plant: random travel, often 0 to reach stall paths. */
        for (int i = 0; i < 2; ++i) {
            const bool blocked = (k % 300u) < 80u && (int)(k / 300u % 2u) == i;
            c[i] += blocked || rnd(4u) == 0u ? 0 : (int32_t)rnd(41u) - 10;
        }
        int16_t l = (int16_t)((int)rnd(801u) - 400);
        int16_t r = (int16_t)((int)rnd(801u) - 400);
        if (rnd(5u) == 0u) {
            l = old_l;
            r = old_r;
        } else if (rnd(6u) == 0u) {
            l = 0;
            r = 0;
        }
        mc_output_t ro, so, so2;
        mc_set_speed(&ref, l, r, now);
        mc_tick(&ref, c, now, &ro);
        mc_same_tick_tick(&win, &st, c, now, &so);
        mc_set_speed(&st, l, r, now + 2u);
        const bool ok = mc_retarget(&win, &st, &so2);
        if (ok) {
            ++applied;
            so = so2;
        } else {
            /* Only a latch from the old target may block the retarget:
             * the stall stays latched (safe side); resync the copies. */
            CHECK((st.faults & MC_FAULT_LATCHED) != 0u &&
                      so.duty_permille[0] == 0 && so.duty_permille[1] == 0,
                  "property: not applied without a latch at %u", k);
            ++kept;
            mc_clear_faults(&ref);
            clear(&st, &win);
            st = ref;
            old_l = l;
            old_r = r;
            if (failures != 0) {
                return;
            }
            continue;
        }
        CHECK(same_wheel(&ref.w[0], &st.w[0]) &&
                  same_wheel(&ref.w[1], &st.w[1]),
              "property: wheel state differs at step %u", k);
        CHECK(ref.faults == st.faults, "property: faults %02x vs %02x at %u",
              ref.faults, st.faults, k);
        CHECK(!ok || same_out(&ro, &so), "property: output differs at %u", k);
        if ((ref.faults & MC_FAULT_LATCHED) != 0u) {
            ++latched;
            CHECK(!ok || (so.duty_permille[0] == 0 && so.duty_permille[1] == 0),
                  "property: latched output not 0 at %u", k);
            mc_clear_faults(&ref);
            clear(&st, &win);
        }
        if (failures != 0) {
            return;
        }
        old_l = l;
        old_r = r;
    }
    CHECK(applied > 10000u && kept > 0u,
          "property: coverage applied %u latched %u kept %u", applied, latched,
          kept);
    printf("property: applied %u, latched in retarget %u, old-target latch "
           "kept %u\n",
           applied, latched, kept);
}

static void start(mc_t *mc, int32_t c[2], uint32_t *now)
{
    c[0] = 0;
    c[1] = 0;
    *now = 1000u;
    mc_same_tick_close(&win);
    mc_init(mc, &CFG, c, *now);
}

int main(void)
{
    mc_t mc;
    mc_output_t out;
    int32_t c[2];
    uint32_t now;

    /* Window closed before the first tick. */
    start(&mc, c, &now);
    mc_set_speed(&mc, 200, 200, now);
    CHECK(!mc_retarget(&win, &mc, &out), "no tick yet: not applied");

    /* Rise once per period, also with two retargets in one window. */
    start(&mc, c, &now);
    const int32_t a = (int32_t)CFG.accel_cpt_q8;
    now += CFG.tick_ms;
    mc_same_tick_tick(&win, &mc, c, now, &out);
    mc_set_speed(&mc, 400, 400, now + 1u);
    CHECK(mc_retarget(&win, &mc, &out), "rise: applied");
    CHECK(mc.w[0].setpoint_q8 == a, "rise: first period %ld",
          (long)mc.w[0].setpoint_q8);
    CHECK(out.duty_permille[0] > 0, "rise: duty %d", out.duty_permille[0]);
    mc_set_speed(&mc, 400, 400, now + 2u);
    CHECK(mc_retarget(&win, &mc, &out), "rise: second retarget applied");
    CHECK(mc.w[0].setpoint_q8 == a, "rise: still one step %ld",
          (long)mc.w[0].setpoint_q8);
    CHECK(mc.w[0].kick_ms == CFG.tick_ms, "kick: one period %u",
          mc.w[0].kick_ms);

    /* Next tick re-opens the window from its own start state. */
    now += CFG.tick_ms;
    mc_same_tick_tick(&win, &mc, c, now, &out);
    CHECK(mc.w[0].setpoint_q8 == 2 * a, "next tick: %ld",
          (long)mc.w[0].setpoint_q8);
    mc_set_speed(&mc, 400, 400, now + 1u);
    CHECK(mc_retarget(&win, &mc, &out) && mc.w[0].setpoint_q8 == 2 * a,
          "next window: %ld", (long)mc.w[0].setpoint_q8);

    /* No double integral: steady speed, then retarget twice. */
    start(&mc, c, &now);
    for (int k = 0; k < 60; ++k) {
        now += CFG.tick_ms;
        c[0] += 10;
        c[1] += 10;
        mc_set_speed(&mc, 150, 150, now);
        mc_same_tick_tick(&win, &mc, c, now, &out);
    }
    now += CFG.tick_ms;
    c[0] += 10;
    c[1] += 10;
    mc_same_tick_tick(&win, &mc, c, now, &out);
    mc_set_speed(&mc, 160, 160, now + 1u);
    CHECK(mc_retarget(&win, &mc, &out), "integral: applied");
    const int32_t i1 = mc.w[0].integral_q8;
    const int16_t d1 = out.duty_permille[0];
    mc_set_speed(&mc, 160, 160, now + 2u);
    CHECK(mc_retarget(&win, &mc, &out), "integral: applied again");
    CHECK(mc.w[0].integral_q8 == i1 && out.duty_permille[0] == d1,
          "integral: %ld vs %ld", (long)mc.w[0].integral_q8, (long)i1);

    /* ESTOP closes the window and stays latched. */
    estop(&mc, &win);
    mc_set_speed(&mc, 160, 160, now + 3u);
    CHECK(!mc_retarget(&win, &mc, &out), "estop: not applied");
    now += CFG.tick_ms;
    mc_same_tick_tick(&win, &mc, c, now, &out);
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0,
          "estop: tick 0");
    mc_set_speed(&mc, 160, 160, now + 1u);
    CHECK(!mc_retarget(&win, &mc, &out), "estop latched: not applied");
    CHECK(mc_faults(&mc) & MC_FAULT_ESTOP, "estop: still latched");

    /* Stop closes the window until the next tick. */
    clear(&mc, &win);
    now += CFG.tick_ms;
    mc_set_speed(&mc, 160, 160, now);
    mc_same_tick_tick(&win, &mc, c, now, &out);
    stop(&mc, &win);
    mc_set_speed(&mc, 160, 160, now + 1u);
    CHECK(!mc_retarget(&win, &mc, &out), "stop: not applied");

    /* Stale: the tick that sets STALE leaves the window closed. */
    start(&mc, c, &now);
    mc_set_speed(&mc, 160, 160, now);
    for (int k = 0; k < 10; ++k) {
        now += CFG.tick_ms;
        mc_same_tick_tick(&win, &mc, c, now, &out);
    }
    CHECK(mc_faults(&mc) & MC_FAULT_STALE, "stale: set");
    mc_set_speed(&mc, 160, 160, now + 1u);
    CHECK(!mc_retarget(&win, &mc, &out), "stale: not applied");
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0,
          "stale: out 0");

    /* A stall found in the retarget latches and gives 0 at once. */
    start(&mc, c, &now);
    for (int k = 0; k < 20; ++k) { /* past the start kick */
        now += CFG.tick_ms;
        c[0] += 10;
        mc_set_speed(&mc, 150, 0, now);
        mc_same_tick_tick(&win, &mc, c, now, &out);
    }
    while (mc.w[0].still_ms + CFG.tick_ms <
           CFG.stall_timeout_ms) { /* blocked */
        now += CFG.tick_ms;
        mc_set_speed(&mc, 150, 0, now);
        mc_same_tick_tick(&win, &mc, c, now, &out);
    }
    CHECK(mc_faults(&mc) == 0u && mc.w[0].still_ms > 0u, "stall: armed %u",
          mc.w[0].still_ms);
    now += CFG.tick_ms;
    mc_set_speed(&mc, 0, 0, now); /* old target 0: the tick does not stall */
    mc_same_tick_tick(&win, &mc, c, now, &out);
    CHECK(mc_faults(&mc) == 0u, "stall: tick with 0 does not latch");
    mc_set_speed(&mc, 150, 0, now + 1u);
    CHECK(mc_retarget(&win, &mc, &out), "stall: retarget ran");
    CHECK((mc_faults(&mc) & MC_FAULT_STALL_LEFT) && out.duty_permille[0] == 0 &&
              out.duty_permille[1] == 0,
          "stall: latched in retarget, out 0");
    mc_set_speed(&mc, 150, 0, now + 2u);
    CHECK(!mc_retarget(&win, &mc, &out), "stall: window closed");

    property();

    if (failures != 0) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("test_motor_ctl_sametick: ok\n");
    return 0;
}
