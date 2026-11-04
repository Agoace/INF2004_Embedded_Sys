/* Host test for src/motor_ctl.c with a simulated pair of wheels.
 * Build: cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
 *        tests/test_motor_ctl.c src/motor_ctl.c -o build/test_motor_ctl
 */
#include <stdio.h>
#include <stdlib.h>

#include "motor_ctl.h"
#include "motor_ctl_robot.h"

static int failures;
#define CHECK(cond, ...)                                                                \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            ++failures;                                                                 \
            printf("FAIL line %d: ", __LINE__);                                         \
            printf(__VA_ARGS__);                                                        \
            printf("\n");                                                               \
        }                                                                               \
    } while (0)

/* Simulated wheel: no movement below the dead zone, then gain counts/tick
 * per permille above it; first-order lag. Like the loaded robot wheel:
 * dead about 90 permille, 140 -> 5, 200 -> 12, 320 -> 24 counts/tick. */
typedef struct {
    double dead[2]; /* forward, reverse */
    double gain[2];
    double scale;   /* battery: 1.0 full */
    double speed;   /* counts per tick */
    double pos;
    bool blocked;
} sim_t;

static sim_t sim[2];
static mc_t mc;
static mc_output_t out;
static uint32_t now;

static const mc_config_t CFG = MC_ROBOT_CONFIG;

static void sim_reset(void) {
    for (int i = 0; i < 2; ++i) {
        sim[i] = (sim_t){{90.0, 90.0}, {0.105, 0.105}, 1.0, 0.0, 1000.0 * (i + 1), false};
    }
}

static void counts(int32_t c[2]) {
    c[0] = (int32_t)sim[0].pos;
    c[1] = (int32_t)sim[1].pos;
}

static void sim_step(void) {
    for (int i = 0; i < 2; ++i) {
        const double d = out.duty_permille[i];
        const int dir = d < 0 ? 1 : 0;
        const double mag = d < 0 ? -d : d;
        double target = mag > sim[i].dead[dir] ? (mag - sim[i].dead[dir]) * sim[i].gain[dir] * sim[i].scale : 0.0;
        if (d < 0) target = -target;
        if (sim[i].blocked) target = 0.0;
        sim[i].speed += (target - sim[i].speed) * 0.5;
        if (sim[i].blocked) sim[i].speed = 0.0;
        sim[i].pos += sim[i].speed;
    }
}

static void start(void) {
    sim_reset();
    now = 1000u;
    int32_t c[2];
    counts(c);
    mc_init(&mc, &CFG, c, now);
    out = (mc_output_t){{0, 0}, 0u};
}

/* Run for ms, asking for the speeds every tick. */
static void run(int left, int right, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += CFG.tick_ms) {
        now += CFG.tick_ms;
        sim_step();
        mc_set_speed(&mc, (int16_t)left, (int16_t)right, now);
        int32_t c[2];
        counts(c);
        mc_tick(&mc, c, now, &out);
    }
}

/* Run without new commands. */
static void idle(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += CFG.tick_ms) {
        now += CFG.tick_ms;
        sim_step();
        int32_t c[2];
        counts(c);
        mc_tick(&mc, c, now, &out);
    }
}

/* Mean measured speed over ms. */
static int mean_speed(int left, int right, uint32_t ms, unsigned int wheel) {
    long sum = 0;
    int n = 0;
    for (uint32_t t = 0; t < ms; t += CFG.tick_ms) {
        run(left, right, CFG.tick_ms);
        sum += mc_speed_mm_s(&mc, wheel);
        ++n;
    }
    return (int)(sum / n);
}

static bool near(int v, int want, int tol) {
    return abs(v - want) <= tol;
}

int main(void) {
    /* 1. Nothing asked: duty 0, servo at the centre. */
    start();
    idle(200u);
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0, "idle: duty 0");
    CHECK(out.servo_us == 1500u, "idle: servo centre (%u)", out.servo_us);
    CHECK(mc_faults(&mc) == 0u, "idle: no command is not stale (%02x)", mc_faults(&mc));

    /* 2. 150 mm/s both: reached in 1 s, within 5 %. */
    start();
    run(150, 150, 1000u);
    {
        const int l = mean_speed(150, 150, 500u, MC_LEFT);
        CHECK(near(l, 150, 8), "150 mm/s: left %d", l);
        CHECK(near(mc_speed_mm_s(&mc, MC_RIGHT), 150, 15), "150 mm/s: right %d",
              mc_speed_mm_s(&mc, MC_RIGHT));
    }
    CHECK(near(mc_travel_mm(&mc, MC_LEFT), mc_travel_mm(&mc, MC_RIGHT), 15),
          "straight: travel L %ld R %ld", (long)mc_travel_mm(&mc, MC_LEFT),
          (long)mc_travel_mm(&mc, MC_RIGHT));

    /* 3. Battery voltage drop (30 % less speed per duty): still reached. */
    start();
    sim[0].scale = sim[1].scale = 0.7;
    run(150, 150, 1500u);
    {
        const int l = mean_speed(150, 150, 500u, MC_LEFT);
        CHECK(near(l, 150, 8), "battery low: left %d", l);
    }

    /* 4. Motors differ (left 20 % weaker): both reach the speed. */
    start();
    sim[0].gain[0] = 0.084;
    run(150, 150, 1500u);
    {
        const int l = mean_speed(150, 150, 500u, MC_LEFT);
        const int r = mean_speed(150, 150, 500u, MC_RIGHT);
        CHECK(near(l, 150, 8) && near(r, 150, 8), "motors differ: L %d R %d", l, r);
    }

    /* 5. Weak reverse (left dead zone 140 in reverse): -80 mm/s reached. */
    start();
    sim[0].dead[1] = 140.0;
    run(-80, -80, 1500u);
    {
        const int l = mean_speed(-80, -80, 500u, MC_LEFT);
        CHECK(near(l, -80, 6), "weak reverse: left %d", l);
    }

    /* 6. Slow turn on the spot (the detour sweep): 40 / -40 mm/s. */
    start();
    sim[0].scale = sim[1].scale = 0.75; /* later in the run */
    run(-40, 40, 1500u);
    {
        const int l = mean_speed(-40, 40, 500u, MC_LEFT);
        const int r = mean_speed(-40, 40, 500u, MC_RIGHT);
        CHECK(near(l, -40, 5) && near(r, 40, 5), "slow spin: L %d R %d", l, r);
        CHECK(mc_faults(&mc) == 0u, "slow spin: no stall (%02x)", mc_faults(&mc));
        CHECK(mc_heading_cdeg(&mc) > 1000, "spin left: heading %ld", (long)mc_heading_cdeg(&mc));
    }

    /* 7. Set point rise limit: accel_cpt_q8 per tick (robot: 8 counts/tick
     * per tick). 300 mm/s is about 37 counts/tick, above two steps. */
    start();
    run(300, 300, CFG.tick_ms);
    CHECK(CFG.accel_cpt_q8 > 0u && mc.w[0].target_q8 > 2 * (int32_t)CFG.accel_cpt_q8,
          "rise: test needs a target above two steps");
    CHECK(mc.w[0].setpoint_q8 == (int32_t)CFG.accel_cpt_q8, "rise: first tick %ld",
          (long)mc.w[0].setpoint_q8);
    run(300, 300, CFG.tick_ms);
    CHECK(mc.w[0].setpoint_q8 == 2 * (int32_t)CFG.accel_cpt_q8, "rise: second tick %ld",
          (long)mc.w[0].setpoint_q8);
    run(300, 300, 1000u);
    run(100, 100, CFG.tick_ms);
    CHECK(mc.w[0].setpoint_q8 == mc.w[0].target_q8, "slowing: at once");

    /* 8. Start kick: duty at least 12 % from rest. */
    start();
    run(20, 20, CFG.tick_ms);
    CHECK(out.duty_permille[0] >= 120, "kick: %d", out.duty_permille[0]);

    /* 9. Stale: no command for 100 ms stops both wheels. */
    start();
    run(150, 150, 500u);
    idle(90u);
    CHECK(out.duty_permille[0] != 0 && !(mc_faults(&mc) & MC_FAULT_STALE), "stale: not at 90 ms");
    idle(10u);
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0 && (mc_faults(&mc) & MC_FAULT_STALE),
          "stale: stopped at 100 ms (%d)", out.duty_permille[0]);
    run(150, 150, CFG.tick_ms);
    CHECK(!(mc_faults(&mc) & MC_FAULT_STALE) && out.duty_permille[0] > 0, "stale: new command clears");

    /* 10. Stop: duty 0 at once, not latched. */
    start();
    run(150, 150, 500u);
    mc_stop(&mc);
    idle(CFG.tick_ms);
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0, "stop: duty 0");
    run(150, 150, CFG.tick_ms);
    CHECK(out.duty_permille[0] > 0, "stop: not latched");

    /* 11. Emergency stop: latched until cleared. */
    start();
    run(150, 150, 500u);
    mc_estop(&mc);
    run(150, 150, 200u);
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0 && (mc_faults(&mc) & MC_FAULT_ESTOP),
          "estop: latched");
    mc_clear_faults(&mc);
    idle(CFG.tick_ms);
    CHECK(out.duty_permille[0] == 0 && mc_faults(&mc) == 0u, "estop clear: still stopped");
    run(150, 150, CFG.tick_ms);
    CHECK(out.duty_permille[0] > 0, "estop clear: new speed runs");

    /* 12. Stall: a blocked wheel, latched, both duties 0. */
    start();
    run(150, 150, 500u);
    sim[1].blocked = true;
    run(150, 150, 490u);
    CHECK(!(mc_faults(&mc) & MC_FAULT_STALL_RIGHT), "stall: not at 490 ms");
    run(150, 150, 30u);
    CHECK((mc_faults(&mc) & MC_FAULT_STALL_RIGHT) && !(mc_faults(&mc) & MC_FAULT_STALL_LEFT) &&
              out.duty_permille[0] == 0 && out.duty_permille[1] == 0,
          "stall: right latched, all 0 (%02x)", mc_faults(&mc));
    /* Blocked from rest: the kick times out. */
    start();
    sim[0].blocked = true;
    run(100, 100, 250u);
    CHECK(!(mc_faults(&mc) & MC_FAULT_STALL_LEFT), "kick stall: not at 250 ms");
    run(100, 100, 20u);
    CHECK(mc_faults(&mc) & MC_FAULT_STALL_LEFT, "kick stall: left after 250 ms");

    /* 13. Measurements: travel and heading from counts. */
    start();
    {
        int32_t c[2];
        sim[0].pos += 12414.0; /* 1 m left */
        sim[1].pos += 12414.0 + 2405.0; /* plus 100 degrees left (24.05 counts each) */
        counts(c);
        now += CFG.tick_ms;
        mc_tick(&mc, c, now, &out);
        CHECK(mc_travel_mm(&mc, MC_LEFT) == 1000, "travel 1000 (%ld)", (long)mc_travel_mm(&mc, MC_LEFT));
        CHECK(mc_heading_cdeg(&mc) == 10000, "heading +100.00 (%ld)", (long)mc_heading_cdeg(&mc));
    }

    /* 14. Servo: pulse, clamp, settle estimate. */
    start();
    idle(100u);
    CHECK(mc_set_servo(&mc, 30, now) == 30 && mc.servo_us == 1833u, "servo +30: %u", mc.servo_us);
    CHECK(!mc_servo_settled(&mc, now + 149u) && mc_servo_settled(&mc, now + 150u), "servo 30 deg: 150 ms");
    CHECK(mc_set_servo(&mc, 25, now) == 25 && mc_servo_settled(&mc, now + 60u) &&
              !mc_servo_settled(&mc, now + 59u),
          "servo 5 deg: 60 ms");
    CHECK(mc_set_servo(&mc, -60, now) == -36 && mc.servo_us == 1100u, "servo clamp right: %u", mc.servo_us);
    CHECK(mc_set_servo(&mc, 60, now) == 36 && mc.servo_us == 1900u, "servo clamp left: %u", mc.servo_us);
    idle(CFG.tick_ms);
    CHECK(out.servo_us == 1900u, "servo pulse out");

    /* 15. Exact loop terms, with the other terms off. The wheel does not
     * move (speed 0), so the error is the set point. */
    {
        mc_config_t pure = CFG;
        pure.kp_q8 = 0u;
        pure.ki_q8 = 0u;
        pure.kick_permille = 0u;
        pure.accel_cpt_q8 = 0u;
        pure.stall_min_permille = 1000u;
        pure.ff_offset_permille[0][0] = 50u;
        pure.ff_offset_permille[0][1] = 70u;
        pure.ff_offset_permille[1][0] = 60u;
        pure.ff_offset_permille[1][1] = 80u;
        pure.ff_slope_q8[0][0] = 256u;  /* 1.0 */
        pure.ff_slope_q8[0][1] = 512u;  /* 2.0 */
        pure.ff_slope_q8[1][0] = 768u;  /* 3.0 */
        pure.ff_slope_q8[1][1] = 1024u; /* 4.0 */
        const int32_t c0[2] = {0, 0};
        /* 161 mm/s = 19.98 counts/tick: Q8 5116 -> 19 (floor of 5116/256). */
        mc_init(&mc, &pure, c0, 0u);
        mc_set_speed(&mc, 161, -161, 0u);
        mc_tick(&mc, c0, 10u, &out);
        CHECK(mc.w[0].setpoint_q8 == 5116, "pure: set point Q8 %ld", (long)mc.w[0].setpoint_q8);
        CHECK(out.duty_permille[0] == 50 + 19 && out.duty_permille[1] == -(80 + 79),
              "pure ff: L fwd %d, R rev %d", out.duty_permille[0], out.duty_permille[1]);
        mc_init(&mc, &pure, c0, 0u);
        mc_set_speed(&mc, -161, 161, 0u);
        mc_tick(&mc, c0, 10u, &out);
        CHECK(out.duty_permille[0] == -(70 + 39) && out.duty_permille[1] == 60 + 59,
              "pure ff: L rev %d, R fwd %d", out.duty_permille[0], out.duty_permille[1]);
        /* P only: kp 2.0, error 19.98 counts/tick -> 39. */
        mc_config_t ponly = pure;
        ponly.ff_offset_permille[0][0] = 0u;
        ponly.ff_slope_q8[0][0] = 0u;
        ponly.kp_q8 = 512u;
        mc_init(&mc, &ponly, c0, 0u);
        mc_set_speed(&mc, 161, 0, 0u);
        mc_tick(&mc, c0, 10u, &out);
        CHECK(out.duty_permille[0] == 39, "P only: %d", out.duty_permille[0]);
        /* Clamp and anti-windup: max 300, P alone is above it, I stays 0. */
        ponly.kp_q8 = 25600u; /* 100 */
        ponly.ki_q8 = 256u;
        ponly.max_permille = 300u;
        ponly.kick_counts = 0u; /* kick over after the first tick, so I is not held by it */
        mc_init(&mc, &ponly, c0, 0u);
        for (uint32_t k = 1u; k <= 10u; ++k) {
            mc_set_speed(&mc, 161, -161, k * 10u);
            mc_tick(&mc, c0, k * 10u, &out);
        }
        CHECK(out.duty_permille[0] == 300, "clamp: %d", out.duty_permille[0]);
        CHECK(mc.w[0].integral_q8 == 0, "anti-windup high: I %ld", (long)mc.w[0].integral_q8);
        CHECK(mc.w[1].integral_q8 == 0, "anti-windup low: I %ld", (long)mc.w[1].integral_q8);
        /* A limit above 1000 is set to 1000. */
        ponly.max_permille = 2000u;
        mc_init(&mc, &ponly, c0, 0u);
        CHECK(mc.cfg.max_permille == 1000u, "max limit 1000 (%u)", mc.cfg.max_permille);
    }

    /* 16. Direction change arms the kick again. */
    start();
    run(150, 150, 500u);
    CHECK(!mc.w[0].kicking, "reverse: no kick while running");
    {
        bool kicked = false;
        for (int k = 0; k < 40 && !kicked; ++k) {
            run(-150, -150, CFG.tick_ms);
            kicked = mc.w[0].setpoint_q8 < 0 && mc.w[0].kicking;
        }
        CHECK(kicked, "reverse: kick on the direction change");
    }

    /* 17. Stall exactly at 500 ms of no count. */
    start();
    run(150, 150, 500u);
    sim[1].blocked = true;
    run(150, 150, 490u);
    CHECK(!(mc_faults(&mc) & MC_FAULT_STALL_RIGHT), "stall 490 ms: no");
    run(150, 150, 10u);
    CHECK(mc_faults(&mc) & MC_FAULT_STALL_RIGHT, "stall 500 ms: yes");
    CHECK(out.duty_permille[0] == 0, "stall: the other wheel 0 too (%d)", out.duty_permille[0]);
    /* Clear: stopped until a new speed. */
    sim[1].blocked = false;
    mc_clear_faults(&mc);
    idle(CFG.tick_ms);
    CHECK(out.duty_permille[0] == 0 && out.duty_permille[1] == 0, "stall clear: stopped");

    /* 18. Speed sent while latched has no effect after the clear. */
    start();
    run(150, 150, 300u);
    mc_estop(&mc);
    mc_set_speed(&mc, 150, 150, now);
    mc_clear_faults(&mc);
    idle(CFG.tick_ms);
    CHECK(out.duty_permille[0] == 0, "latched speed dropped (%d)", out.duty_permille[0]);

    /* 19. A late tick (20 ms): speed in counts per nominal tick. */
    start();
    {
        int32_t c[2];
        counts(c);
        c[0] += 40;
        c[1] += 40;
        now += 20u;
        mc_tick(&mc, c, now, &out);
        CHECK(mc.w[0].speed_cpt == 20, "late tick: %d counts/tick", mc.w[0].speed_cpt);
    }

    /* 21. I holds while the set point rises and while the kick runs. */
    {
        mc_config_t ic = CFG;
        ic.kp_q8 = 0u;
        ic.ki_q8 = 256u;          /* 1.0 per tick */
        ic.accel_cpt_q8 = 256u;   /* 1 count/tick per tick */
        ic.kick_permille = 100u;
        ic.kick_counts = 10u;
        ic.kick_timeout_ms = 1000u;
        ic.stall_min_permille = 1000u;
        const int32_t c0[2] = {0, 0};
        mc_init(&mc, &ic, c0, 0u);
        /* 81 mm/s = 10.06 counts/tick: about 10 ticks of rise. The wheel
         * does not move, so the error is large the whole time. */
        uint32_t t = 0u;
        for (int k = 0; k < 5; ++k) {
            t += 10u;
            mc_set_speed(&mc, 81, 0, t);
            mc_tick(&mc, c0, t, &out);
        }
        CHECK(mc.w[0].setpoint_q8 < mc.w[0].target_q8 && mc.w[0].integral_q8 == 0,
              "rising: I held (%ld)", (long)mc.w[0].integral_q8);
        /* Kick done (wheel turned 10 counts), set point still rising. */
        int32_t moved[2] = {10, 0};
        t += 10u;
        mc_set_speed(&mc, 81, 0, t);
        mc_tick(&mc, moved, t, &out);
        CHECK(!mc.w[0].kicking && mc.w[0].integral_q8 == 0, "kick over, still rising: I held");
        /* One more tick: no kick, set point still below the target. */
        t += 10u;
        mc_set_speed(&mc, 81, 0, t);
        mc_tick(&mc, moved, t, &out);
        CHECK(!mc.w[0].kicking && mc.w[0].setpoint_q8 < mc.w[0].target_q8 && mc.w[0].integral_q8 == 0,
              "no kick, still rising: I held (%ld)", (long)mc.w[0].integral_q8);
        /* Set point at the target, wheel stopped again: I grows. */
        for (int k = 0; k < 10; ++k) {
            t += 10u;
            mc_set_speed(&mc, 81, 0, t);
            mc_tick(&mc, moved, t, &out);
        }
        CHECK(mc.w[0].setpoint_q8 == mc.w[0].target_q8 && mc.w[0].integral_q8 > 0,
              "at the target: I grows (%ld)", (long)mc.w[0].integral_q8);
        /* At the target but kicking (from rest, no rise limit): I held. */
        ic.accel_cpt_q8 = 0u;
        mc_init(&mc, &ic, c0, 0u);
        t = 0u;
        for (int k = 0; k < 5; ++k) {
            t += 10u;
            mc_set_speed(&mc, 81, 0, t);
            mc_tick(&mc, c0, t, &out);
        }
        CHECK(mc.w[0].kicking && mc.w[0].setpoint_q8 == mc.w[0].target_q8 && mc.w[0].integral_q8 == 0,
              "kicking at the target: I held (%ld)", (long)mc.w[0].integral_q8);
    }

    /* 22. Speed limit: +/-460 mm/s each wheel; 0 = none. */
    start();
    {
        mc_set_speed(&mc, 460, -460, now);
        const int32_t q460 = mc.w[0].target_q8;
        CHECK(mc.w[1].target_q8 == -q460, "limit: -460 = -(460)");
        mc_set_speed(&mc, 461, -2000, now);
        CHECK(mc.w[0].target_q8 == q460 && mc.w[1].target_q8 == -q460, "limit: 461 and -2000 -> +/-460");
        mc_set_speed(&mc, 459, 0, now);
        CHECK(mc.w[0].target_q8 < q460 && mc.w[0].target_q8 > 0, "limit: 459 kept");
        mc_config_t open = CFG;
        open.max_speed_mm_s = 0u;
        const int32_t c0[2] = {0, 0};
        mc_init(&mc, &open, c0, now);
        mc_set_speed(&mc, 1000, -1000, now);
        CHECK(mc.w[0].target_q8 > 2 * q460 && mc.w[1].target_q8 < -2 * q460, "limit 0: none");
    }

    /* 20. Servo settle rounding: 1 degree = 42 + 3.6 -> 46 ms. */
    start();
    (void)mc_set_servo(&mc, 1, now);
    CHECK(!mc_servo_settled(&mc, now + 45u) && mc_servo_settled(&mc, now + 46u), "servo 1 deg: 46 ms");

    if (failures) {
        printf("test_motor_ctl: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_motor_ctl: all tests passed\n");
    return 0;
}
