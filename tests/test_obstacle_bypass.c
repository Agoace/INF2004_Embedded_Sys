/*
 * Host unit test for src/obstacle_bypass.c. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_obstacle_bypass.c src/obstacle_bypass.c \
 *      -o build/test_obstacle_bypass
 *   ./build/test_obstacle_bypass
 */
#include <stdio.h>

#include "obstacle_bypass.h"

static int failures;

static void check(int ok, const char *what) {
    if (!ok) {
        printf("FAIL %s\n", what);
        ++failures;
    }
}

static const bp_config_t CFG = {
    .hit_mm = 250u,
    .limit_deg = 30u,
    .step_deg = 5u,
    .bounce_deg = 10u,
    .offset_deg = 20u,
    .line_arm_ms = 150u,
    .pivot_max_ms = 4000u,
    .around_max_ms = 10000u,
};

/* Reading markers. */
#define NR -1    /* NO_RETURN */
#define RISE -2  /* NO_RISE, streak 1 */
#define FAULT -3 /* NO_RISE, streak 5 */

static ultrasonic_result_t result(int mm) {
    if (mm == NR) {
        return (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN << ULTRASONIC_STATUS_SHIFT);
    }
    if (mm == RISE || mm == FAULT) {
        const unsigned int streak = mm == RISE ? 1u : ULTRASONIC_FAULT_STREAK;
        return (ultrasonic_result_t)((ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT) |
                                     (streak << ULTRASONIC_STREAK_SHIFT));
    }
    return (ultrasonic_result_t)mm; /* VALID, distance code */
}

static uint32_t now;

/* Obstacle distance from the scan: the pivot needs 260 + 150 = 410 mm. */
#define OBST_MM 260u

/* Wait for the servo to settle, take one reading, and update. */
static bp_word_t read(bp_t *bp, int mm) {
    while (!bp_wants_reading(bp, now)) {
        now += 1u;
    }
    bp_triggered(bp);
    now += 10u;
    bp_reading(bp, result(mm), now);
    return bp_update(bp, false, false, now);
}

static bp_t start(obstacle_side_t side) {
    bp_t bp;
    now = 1000u;
    bp_init(&bp, &CFG, side, OBST_MM, now);
    return bp;
}

/* Pivot done: the obstacle seen at 300 mm, then two free readings. */
static bp_t around(obstacle_side_t side) {
    bp_t bp = start(side);
    (void)read(&bp, 300);
    (void)read(&bp, NR);
    (void)read(&bp, NR);
    return bp;
}

int main(void) {
    bp_t bp;
    bp_word_t w;

    /* No free side. */
    bp = start(OBSTACLE_SIDE_NONE);
    w = bp_update(&bp, true, false, now);
    check(bp_state(&bp) == BP_FAILED && bp_fail(&bp) == BP_FAIL_NO_SIDE, "side NONE fails");
    check(bp_move(w) == BP_MOVE_STOP, "side NONE: stop");
    check(!bp_wants_reading(&bp, now + 1000u), "failed: no readings");

    /* Pivot toward the bypass side, servo at the limit toward the obstacle. */
    bp = start(OBSTACLE_SIDE_LEFT);
    w = bp_update(&bp, true, false, now);
    check(bp_move(w) == BP_MOVE_PIVOT && bp_steer_deg(w) == 30 && bp_servo_deg(w) == -30,
          "pivot left: steer field +30, servo -30 (obstacle right)");
    bp = start(OBSTACLE_SIDE_RIGHT);
    w = bp_update(&bp, true, false, now);
    check(bp_move(w) == BP_MOVE_PIVOT && bp_steer_deg(w) == -30 && bp_servo_deg(w) == 30,
          "pivot right: steer field -30, servo +30");

    /* Settle and pending. */
    bp = start(OBSTACLE_SIDE_LEFT);
    check(!bp_wants_reading(&bp, now + 159u), "settle 0 -> -30: not before 160 ms");
    check(bp_wants_reading(&bp, now + 160u), "settle 0 -> -30: at 160 ms");
    bp_triggered(&bp);
    check(!bp_wants_reading(&bp, now + 300u), "pending: no second trigger");
    bp_reading(&bp, result(NR), now + 300u);
    check(!bp_wants_reading(&bp, now + 339u) && bp_wants_reading(&bp, now + 340u),
          "after a reading: settle again 40 ms");

    /* A reading with no trigger is ignored. */
    bp = start(OBSTACLE_SIDE_LEFT);
    bp_reading(&bp, result(300), now);
    bp_reading(&bp, result(NR), now);
    bp_reading(&bp, result(NR), now);
    check(bp_state(&bp) == BP_PIVOT && !bp.seen, "readings without a trigger are ignored");

    /* Pivot: the obstacle must go through the beam and out. */
    bp = start(OBSTACLE_SIDE_LEFT);
    (void)read(&bp, NR);
    (void)read(&bp, NR);
    (void)read(&bp, NR);
    check(bp_state(&bp) == BP_PIVOT, "pivot: free before the obstacle is seen does not count");
    (void)read(&bp, 300); /* in the beam */
    (void)read(&bp, NR);
    (void)read(&bp, 409); /* < 410: still the obstacle */
    (void)read(&bp, 410); /* free */
    check(bp_state(&bp) == BP_PIVOT, "pivot: 409 mm between frees restarts the count");
    w = read(&bp, NR);
    check(bp_state(&bp) == BP_AROUND, "pivot: seen, then two free -> AROUND");
    check(bp_move(w) == BP_MOVE_STEER && bp_steer_deg(w) == -10 && bp_servo_deg(w) == -30,
          "AROUND start: safe -30 (the limit) + 20 = -10, servo stays at the limit");

    /* No information, and the fault streak. */
    bp = start(OBSTACLE_SIDE_LEFT);
    (void)read(&bp, 300);
    (void)read(&bp, NR);
    (void)read(&bp, RISE);
    check(!bp_wants_reading(&bp, now + 39u) && bp_wants_reading(&bp, now + 40u),
          "no information: wait 40 ms, then read again");
    (void)read(&bp, NR);
    check(bp_state(&bp) == BP_AROUND, "NO_RISE without a fault streak: no information");
    (void)read(&bp, FAULT);
    check(bp_state(&bp) == BP_FAILED && bp_fail(&bp) == BP_FAIL_SENSOR, "fault streak fails");

    /* Rubber band, bypass left (obstacle on the right). */
    bp = around(OBSTACLE_SIDE_LEFT);
    w = read(&bp, NR); /* clear at the limit -30 */
    check(bp_servo_deg(w) == -30 && bp_steer_deg(w) == -10,
          "clear at the limit: servo stays, steer -30 + 20 = -10 (back toward it)");
    w = read(&bp, 200); /* hit at -30 */
    check(bp_servo_deg(w) == -20 && bp_steer_deg(w) == 0,
          "hit at -30: bounce to -20 (safe), steer -20 + 20 = 0");
    check(!bp_wants_reading(&bp, now + 79u) && bp_wants_reading(&bp, now + 80u),
          "10 degree bounce: 80 ms settle");
    w = read(&bp, NR); /* clear at -20 */
    check(bp_servo_deg(w) == -25 && bp_steer_deg(w) == 0, "clear at -20: steer 0, servo -25");
    check(!bp_wants_reading(&bp, now + 59u) && bp_wants_reading(&bp, now + 60u),
          "5 degree step: 60 ms settle");
    w = read(&bp, 220); /* hit at -25 */
    check(bp_servo_deg(w) == -15 && bp_steer_deg(w) == 5, "hit at -25: safe -15, steer +5");
    w = read(&bp, NR); /* clear at -15 */
    check(bp_servo_deg(w) == -20 && bp_steer_deg(w) == 5, "clear at -15: steer +5");
    w = read(&bp, 249); /* hit at -20 */
    check(bp_steer_deg(w) == 10 && bp_servo_deg(w) == -10, "249 mm is a hit: safe -10, +10");
    w = read(&bp, 250); /* clear at -10 */
    check(bp_steer_deg(w) == 10 && bp_servo_deg(w) == -15, "250 mm (hit_mm) is clear");

    /* Rubber band, bypass right (obstacle on the left). */
    bp = around(OBSTACLE_SIDE_RIGHT);
    w = bp_update(&bp, false, false, now);
    check(bp_steer_deg(w) == 10 && bp_servo_deg(w) == 30, "bypass right: start steer +10");
    w = read(&bp, 200); /* hit at +30 */
    check(bp_servo_deg(w) == 20 && bp_steer_deg(w) == 0, "bypass right: hit +30 -> 0");
    w = read(&bp, NR); /* clear at +20 */
    check(bp_servo_deg(w) == 25 && bp_steer_deg(w) == 0, "bypass right: servo steps left");

    /* Steering clamp: hits walk the servo to the bypass side. */
    bp = around(OBSTACLE_SIDE_LEFT);
    for (int i = 0; i < 4; ++i) {
        (void)read(&bp, 200); /* -30, -20, -10, 0 */
    }
    w = read(&bp, 200); /* hit at +10: safe +20 */
    check(bp_steer_deg(w) == 30 && bp_servo_deg(w) == 20, "steer clamps at +30");
    w = read(&bp, 200); /* hit at +20: safe +30 */
    w = read(&bp, 200); /* hit at +30: safe +40 */
    check(bp_steer_deg(w) == 30 && bp_servo_deg(w) == 30, "servo clamps at +30");

    /* Too near ahead fails. */
    bp = around(OBSTACLE_SIDE_LEFT);
    (void)read(&bp, 200); /* -30 -> -20 */
    (void)read(&bp, 200); /* -20 -> -10 */
    (void)read(&bp, 0);   /* TOO_NEAR at -10 */
    check(bp_state(&bp) == BP_FAILED && bp_fail(&bp) == BP_FAIL_TOO_NEAR,
          "TOO_NEAR within 10 degrees fails");
    bp = around(OBSTACLE_SIDE_LEFT);
    (void)read(&bp, 200); /* -30 -> -20 */
    w = read(&bp, 0);     /* TOO_NEAR at -20 */
    check(bp_state(&bp) == BP_AROUND && bp_steer_deg(w) == 10,
          "TOO_NEAR at 20 degrees: a hit, no failure");

    /* Line: seen at the start (on the line); armed after 150 ms dark. */
    bp = around(OBSTACLE_SIDE_LEFT);
    uint32_t t = now;
    (void)bp_update(&bp, true, false, t);
    (void)bp_update(&bp, false, false, t + 10u);
    (void)bp_update(&bp, false, false, t + 150u); /* 140 ms dark */
    (void)bp_update(&bp, true, false, t + 160u);
    check(bp_state(&bp) == BP_AROUND, "line back after 140 ms dark: not armed");
    (void)bp_update(&bp, false, false, t + 200u);
    (void)bp_update(&bp, false, false, t + 349u);
    (void)bp_update(&bp, true, false, t + 350u);
    check(bp_state(&bp) == BP_AROUND, "dark 149 ms (a break restarts): not armed");
    (void)bp_update(&bp, false, false, t + 400u);
    (void)bp_update(&bp, false, false, t + 550u);
    w = bp_update(&bp, true, true, t + 560u);
    check(bp_state(&bp) == BP_AROUND, "junction reading does not end the bypass");
    w = bp_update(&bp, true, false, t + 570u);
    check(bp_state(&bp) == BP_DONE && bp_move(w) == BP_MOVE_STOP, "line found: DONE, stop");
    check(!bp_wants_reading(&bp, t + 1000u), "done: no readings");

    /* The line does not end the pivot. */
    bp = start(OBSTACLE_SIDE_LEFT);
    (void)bp_update(&bp, false, false, now);
    (void)bp_update(&bp, false, false, now + 200u);
    (void)bp_update(&bp, true, false, now + 210u);
    check(bp_state(&bp) == BP_PIVOT, "line during the pivot: still PIVOT");

    /* Time limits. */
    bp = start(OBSTACLE_SIDE_LEFT);
    (void)bp_update(&bp, true, false, now + 3999u);
    check(bp_state(&bp) == BP_PIVOT, "pivot 3999 ms: running");
    (void)bp_update(&bp, true, false, now + 4000u);
    check(bp_fail(&bp) == BP_FAIL_PIVOT_TIME, "pivot 4000 ms: fails");
    bp = around(OBSTACLE_SIDE_LEFT);
    t = now;
    (void)bp_update(&bp, true, false, t + 9999u);
    check(bp_state(&bp) == BP_AROUND, "around 9999 ms: running");
    (void)bp_update(&bp, true, false, t + 10000u);
    check(bp_fail(&bp) == BP_FAIL_AROUND_TIME, "around 10000 ms: fails");

    /* Time wrap. */
    bp_init(&bp, &CFG, OBSTACLE_SIDE_LEFT, OBST_MM, 0xFFFFFFF0u);
    check(!bp_wants_reading(&bp, 0xFFFFFFFFu), "settle: not before the wrap");
    check(!bp_wants_reading(&bp, 0x0000008Fu) && bp_wants_reading(&bp, 0x00000090u),
          "settle over the time wrap (160 ms)");
    (void)bp_update(&bp, true, false, 0xFFFFFFF5u);
    check(bp_state(&bp) == BP_PIVOT, "pivot limit: running just after the start, before the wrap");
    (void)bp_update(&bp, true, false, 0x00000F8Fu);
    check(bp_state(&bp) == BP_PIVOT, "pivot limit over the wrap: running");
    (void)bp_update(&bp, true, false, 0x00000F90u);
    check(bp_fail(&bp) == BP_FAIL_PIVOT_TIME, "pivot limit over the wrap: fails at 4000 ms");

    /* Steering angle to line_steer position (W/2 55.5 mm, L 65 mm). */
    check(bp_steer_position(0) == 0, "steer 0 -> position 0");
    check(bp_steer_position(30) == -49, "steer +30 (left) -> -49: 0.854 x 0.577");
    check(bp_steer_position(-30) == 49, "steer -30 (right) -> +49");
    check(bp_steer_position(15) == -23, "steer +15 -> -23 (22.9 rounded)");
    check(bp_steer_position(-1) == 1, "steer -1 -> +1 (1.45 rounded)");
    check(bp_steer_position(45) == -49, "steer clamped to 30");
    check(bp_steer_position(-45) == 49, "steer clamped to -30");

    /* Word layout. */
    bp = around(OBSTACLE_SIDE_LEFT);
    w = bp_update(&bp, false, false, now);
    check(w == (bp_word_t)(2u | (20u << 2) | (0u << 8)), "word: STEER, steer -10, servo -30");

    if (failures == 0) {
        printf("test_obstacle_bypass: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_bypass: %d failure(s)\n", failures);
    return 1;
}
