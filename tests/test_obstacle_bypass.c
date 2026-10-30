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
    .clearance_mm = 110u,
    .limit_deg = 30u,
    .step_deg = 5u,
    .bounce_deg = 10u,
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
static uint32_t t0_turn;
static int32_t travel; /* mean wheel count passed to bp_update */

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
    return bp_update(bp, false, false, travel, now);
}

/* Drive the rest of the DRIVE leg. */
static void drive_leg(bp_t *bp) {
    travel += (int32_t)bp->leg_counts;
    (void)bp_update(bp, false, false, travel, now);
}

static bp_t start(obstacle_side_t side) {
    bp_t bp;
    now = 1000u;
    travel = 0;
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
    w = bp_update(&bp, true, false, travel, now);
    check(bp_state(&bp) == BP_FAILED && bp_fail(&bp) == BP_FAIL_NO_SIDE, "side NONE fails");
    check(bp_move(w) == BP_MOVE_STOP, "side NONE: stop");
    check(!bp_wants_reading(&bp, now + 1000u), "failed: no readings");

    /* Pivot toward the bypass side, servo at the limit toward the obstacle. */
    bp = start(OBSTACLE_SIDE_LEFT);
    w = bp_update(&bp, true, false, travel, now);
    check(bp_move(w) == BP_MOVE_PIVOT && bp_steer_deg(w) == 30 && bp_servo_deg(w) == -30,
          "pivot left: steer field +30, servo -30 (obstacle right)");
    bp = start(OBSTACLE_SIDE_RIGHT);
    w = bp_update(&bp, true, false, travel, now);
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
    check(bp_state(&bp) == BP_AROUND && bp_phase(&bp) == BP_PHASE_DRIVE,
          "pivot: seen, then two free -> AROUND, DRIVE");
    check(bp_move(w) == BP_MOVE_STEER && bp_steer_deg(w) == -14 && bp_servo_deg(w) == -30,
          "AROUND start: edge -30 (the limit) + clear angle(409 mm) 16 = -14");

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

    /* Clear angle: smallest a with sin(a) >= 110 / d. */
    check(bp_clear_angle(110u, 220u) == 30, "clear angle 110/220: 30 (sin 30 = 0.500)");
    check(bp_clear_angle(110u, 219u) == 31, "clear angle 110/219: 31");
    check(bp_clear_angle(110u, 300u) == 22, "clear angle 110/300: 22");
    check(bp_clear_angle(110u, 110u) == 89, "clear angle c = d: 89 (sin 89 rounds to 1000)");
    check(bp_clear_angle(110u, 109u) == 90, "clear angle c > d: 90");
    check(bp_clear_angle(110u, 0u) == 90, "clear angle d 0: 90");
    check(bp_clear_angle(0u, 300u) == 0, "clear angle c 0: 0");

    /* Leg length and wheel counts. */
    check(bp_leg_mm(110u, 300u) == 279u, "leg 110/300: sqrt(77900) = 279.1 -> 279");
    check(bp_leg_mm(110u, 111u) == 14u, "leg 110/111: sqrt(221) = 14.9 -> 14");
    check(bp_leg_mm(110u, 110u) == 0u && bp_leg_mm(110u, 50u) == 0u, "leg d <= c: 0");
    check(bp_leg_mm(0u, 65535u) == 65535u, "leg c 0, d 65535: 65535 (no overflow)");
    check(bp_leg_mm(3u, 5u) == 4u, "leg 3/5: 4 exactly");
    check(bp_mm_to_counts(1000u) == 12414u, "1000 mm: 12414 counts (2535 / 204.204 mm)");
    check(bp_mm_to_counts(279u) == 3464u, "279 mm: 3463.5 -> 3464 counts");
    check(bp_mm_to_counts(0u) == 0u, "0 mm: 0 counts");

    /* Bypass left (obstacle on the right): DRIVE, lost, TURN, EDGE, DRIVE. */
    bp = around(OBSTACLE_SIDE_LEFT); /* last seen 300 mm: 22 */
    w = bp_update(&bp, false, false, travel, now);
    check(bp_move(w) == BP_MOVE_STEER && bp_steer_deg(w) == -8 && bp_servo_deg(w) == -30,
          "DRIVE start: -30 + 22 = -8");
    check(bp.leg_counts == 3464u && bp.leg_started && bp.leg_from == 0,
          "DRIVE leg: 279 mm from 300 mm, started at the first update");
    w = read(&bp, NR); /* free at the limit, leg not done */
    check(bp_phase(&bp) == BP_PHASE_DRIVE && bp_servo_deg(w) == -30,
          "free at the limit before the leg ends: keep driving, read the limit again");
    check(!bp_wants_reading(&bp, now + 39u) && bp_wants_reading(&bp, now + 40u),
          "read the limit again: 40 ms settle");
    travel = 3463;
    w = read(&bp, NR);
    check(bp_phase(&bp) == BP_PHASE_DRIVE, "leg 3463 of 3464 counts: still DRIVE");
    travel = 3464;
    (void)bp_update(&bp, false, false, travel, now);
    w = read(&bp, NR); /* free at the limit -30, leg done: lost */
    check(bp_phase(&bp) == BP_PHASE_TURN && bp_move(w) == BP_MOVE_PIVOT &&
              bp_steer_deg(w) == -30 && bp_servo_deg(w) == -30,
          "lost at the limit: TURN right on the spot, servo stays");
    const uint32_t turn_start = now;
    w = read(&bp, NR);
    check(bp_phase(&bp) == BP_PHASE_TURN && bp_servo_deg(w) == -30, "TURN: free, keep turning");
    w = read(&bp, 200); /* seen */
    check(bp_phase(&bp) == BP_PHASE_EDGE && bp_move(w) == BP_MOVE_STOP &&
              bp_servo_deg(w) == -25 && bp.last_mm == 200u,
          "TURN: seen -> EDGE, stop, servo steps to -25");
    check(!bp_wants_reading(&bp, now + 59u) && bp_wants_reading(&bp, now + 60u),
          "5 degree step: 60 ms settle");
    check(now - turn_start < 4000u, "test timing");
    w = read(&bp, 190); /* seen at -25 */
    check(bp_servo_deg(w) == -20 && bp.last_mm == 190u, "EDGE: seen, step on to -20");
    w = read(&bp, NR); /* free at -20, once */
    check(bp_phase(&bp) == BP_PHASE_EDGE && bp_servo_deg(w) == -20,
          "EDGE: one free reading: read the same angle again");
    check(!bp_wants_reading(&bp, now + 39u) && bp_wants_reading(&bp, now + 40u),
          "read again: 40 ms settle");
    w = read(&bp, 195); /* seen again: a slanted face missed one echo */
    check(bp_phase(&bp) == BP_PHASE_EDGE && bp_servo_deg(w) == -15,
          "EDGE: seen after one free: the count restarts, step on");
    (void)read(&bp, NR); /* -15, once */
    w = read(&bp, NR);   /* -15, twice: the edge */
    /* 110/195 = 0.564: sin 35 = 0.574 -> -15 + 35 = +20 */
    check(bp_phase(&bp) == BP_PHASE_DRIVE && bp_move(w) == BP_MOVE_STEER &&
              bp_steer_deg(w) == 20 && bp_servo_deg(w) == -15,
          "EDGE: two free at -15 -> DRIVE, steer -15 + 35 = +20");
    w = read(&bp, NR); /* DRIVE: free at -15 */
    check(bp_servo_deg(w) == -20 && bp_steer_deg(w) == 20,
          "DRIVE: free steps the servo toward the obstacle; steering stays");
    w = read(&bp, 250); /* seen at -20 */
    check(bp_steer_deg(w) == 7 && bp_servo_deg(w) == -10,
          "DRIVE: seen 250 mm at -20: steer -20 + 27 = +7, bounce to -10");
    check(!bp_wants_reading(&bp, now + 79u) && bp_wants_reading(&bp, now + 80u),
          "10 degree bounce: 80 ms settle");
    (void)read(&bp, NR); /* -10 -> -15 */
    w = read(&bp, 410);  /* free (410 = free_mm) at -15 */
    check(bp_steer_deg(w) == 7 && bp_servo_deg(w) == -20, "410 mm is free");
    travel += 100000; /* the leg so far is long done */
    w = read(&bp, 409); /* seen at -20 */
    check(bp_steer_deg(w) == -4 && bp_servo_deg(w) == -10, "409 mm is seen: -20 + 16 = -4");
    check(bp.leg_started && bp.leg_from == travel && bp.leg_counts == bp_mm_to_counts(393u),
          "a seen reading starts a new leg (sqrt(409^2 - 110^2) = 393.9 -> 393 mm)");
    w = read(&bp, NR); /* -10 */
    w = read(&bp, NR); /* -15 */
    w = read(&bp, NR); /* -20 */
    w = read(&bp, NR); /* -25 */
    check(bp_phase(&bp) == BP_PHASE_DRIVE && bp_servo_deg(w) == -30, "DRIVE until the limit");
    w = read(&bp, NR); /* free at -30, new leg not done */
    check(bp_phase(&bp) == BP_PHASE_DRIVE, "the new leg must be driven too");
    drive_leg(&bp);
    w = read(&bp, NR); /* free at -30 */
    check(bp_phase(&bp) == BP_PHASE_TURN, "free at the limit: TURN again");

    /* A new leg counts from the next update, not from the old leg's start. */
    bp = around(OBSTACLE_SIDE_LEFT);
    drive_leg(&bp); /* old leg done */
    bp_triggered(&bp);
    bp_reading(&bp, result(300), now); /* seen at -30: new leg, no update yet */
    bp.servo_deg = -30;                /* look at the limit at once */
    bp.settle_ms = now;
    bp_triggered(&bp);
    bp_reading(&bp, result(NR), now);
    check(bp_phase(&bp) == BP_PHASE_DRIVE, "new leg not started yet: free at the limit is not lost");

    /* Bypass right (obstacle on the left). */
    bp = around(OBSTACLE_SIDE_RIGHT);
    w = bp_update(&bp, false, false, travel, now);
    check(bp_steer_deg(w) == 8 && bp_servo_deg(w) == 30, "bypass right: DRIVE start +30 - 22");
    drive_leg(&bp);
    w = read(&bp, NR);
    check(bp_move(w) == BP_MOVE_PIVOT && bp_steer_deg(w) == 30, "bypass right: TURN left");
    w = read(&bp, 200);
    check(bp_servo_deg(w) == 25 && bp_move(w) == BP_MOVE_STOP, "bypass right: EDGE steps right");
    (void)read(&bp, NR);
    w = read(&bp, NR);
    check(bp_steer_deg(w) == -9 && bp_phase(&bp) == BP_PHASE_DRIVE,
          "bypass right: edge +25 - 34 = -9");
    w = read(&bp, NR);
    check(bp_servo_deg(w) == 30, "bypass right: DRIVE steps the servo left");
    w = read(&bp, 300);
    check(bp_steer_deg(w) == 8 && bp_servo_deg(w) == 20, "bypass right: seen bounces right");

    /* EDGE seen across the whole view; steering clamp. */
    bp = around(OBSTACLE_SIDE_LEFT);
    drive_leg(&bp);
    (void)read(&bp, NR);  /* TURN */
    (void)read(&bp, 300); /* EDGE, servo -25 */
    for (int i = 0; i < 11; ++i) {
        (void)read(&bp, 300); /* seen -25 .. +25 */
    }
    w = bp_update(&bp, false, false, travel, now);
    check(bp_phase(&bp) == BP_PHASE_EDGE && bp_servo_deg(w) == 30,
          "EDGE: seen from -25 to +25, servo at the limit +30");
    w = read(&bp, 300); /* seen at +30, the limit toward S */
    check(bp_phase(&bp) == BP_PHASE_DRIVE && bp_steer_deg(w) == 30 && bp_servo_deg(w) == 30,
          "EDGE: seen at the limit: edge is the limit, steer clamps at +30");
    bp = around(OBSTACLE_SIDE_LEFT);
    w = read(&bp, 100); /* DRIVE: seen 100 mm at -30: -30 + 90 */
    check(bp_steer_deg(w) == 30 && bp_servo_deg(w) == -20, "very near at the side: steer +30");

    /* TURN time limit. */
    bp = around(OBSTACLE_SIDE_LEFT);
    drive_leg(&bp);
    (void)read(&bp, NR); /* TURN at now */
    t0_turn = now;
    (void)bp_update(&bp, false, false, travel, t0_turn + 3999u);
    check(bp_state(&bp) == BP_AROUND, "TURN 3999 ms: running");
    (void)bp_update(&bp, false, false, travel, t0_turn + 4000u);
    check(bp_state(&bp) == BP_FAILED && bp_fail(&bp) == BP_FAIL_PIVOT_TIME, "TURN 4000 ms: fails");
    bp = around(OBSTACLE_SIDE_LEFT);
    (void)bp_update(&bp, false, false, travel, now + 5000u);
    check(bp_state(&bp) == BP_AROUND, "DRIVE: no TURN time limit");

    /* Too near ahead fails. */
    bp = around(OBSTACLE_SIDE_LEFT);
    (void)read(&bp, 200); /* DRIVE: seen at -30 -> -20 */
    (void)read(&bp, 200); /* -20 -> -10 */
    (void)read(&bp, 0);   /* TOO_NEAR at -10 */
    check(bp_state(&bp) == BP_FAILED && bp_fail(&bp) == BP_FAIL_TOO_NEAR,
          "TOO_NEAR within 10 degrees fails");
    bp = around(OBSTACLE_SIDE_LEFT);
    (void)read(&bp, 200); /* -30 -> -20 */
    w = read(&bp, 0);     /* TOO_NEAR at -20 */
    check(bp_state(&bp) == BP_AROUND && bp_steer_deg(w) == 30,
          "TOO_NEAR at 20 degrees: seen, no failure; steer +30");

    /* Line: seen at the start (on the line); armed after 150 ms dark. */
    bp = around(OBSTACLE_SIDE_LEFT);
    uint32_t t = now;
    (void)bp_update(&bp, true, false, travel, t);
    (void)bp_update(&bp, false, false, travel, t + 10u);
    (void)bp_update(&bp, false, false, travel, t + 150u); /* 140 ms dark */
    (void)bp_update(&bp, true, false, travel, t + 160u);
    check(bp_state(&bp) == BP_AROUND, "line back after 140 ms dark: not armed");
    (void)bp_update(&bp, false, false, travel, t + 200u);
    (void)bp_update(&bp, false, false, travel, t + 349u);
    (void)bp_update(&bp, true, false, travel, t + 350u);
    check(bp_state(&bp) == BP_AROUND, "dark 149 ms (a break restarts): not armed");
    (void)bp_update(&bp, false, false, travel, t + 400u);
    (void)bp_update(&bp, false, false, travel, t + 550u);
    w = bp_update(&bp, true, true, travel, t + 560u);
    check(bp_state(&bp) == BP_AROUND, "junction reading does not end the bypass");
    w = bp_update(&bp, true, false, travel, t + 570u);
    check(bp_state(&bp) == BP_DONE && bp_move(w) == BP_MOVE_STOP, "line found: DONE, stop");
    check(!bp_wants_reading(&bp, t + 1000u), "done: no readings");

    /* The line does not end the pivot. */
    bp = start(OBSTACLE_SIDE_LEFT);
    (void)bp_update(&bp, false, false, travel, now);
    (void)bp_update(&bp, false, false, travel, now + 200u);
    (void)bp_update(&bp, true, false, travel, now + 210u);
    check(bp_state(&bp) == BP_PIVOT, "line during the pivot: still PIVOT");

    /* Time limits. */
    bp = start(OBSTACLE_SIDE_LEFT);
    (void)bp_update(&bp, true, false, travel, now + 3999u);
    check(bp_state(&bp) == BP_PIVOT, "pivot 3999 ms: running");
    (void)bp_update(&bp, true, false, travel, now + 4000u);
    check(bp_fail(&bp) == BP_FAIL_PIVOT_TIME, "pivot 4000 ms: fails");
    bp = around(OBSTACLE_SIDE_LEFT);
    t = now;
    (void)bp_update(&bp, true, false, travel, t + 9999u);
    check(bp_state(&bp) == BP_AROUND, "around 9999 ms: running");
    (void)bp_update(&bp, true, false, travel, t + 10000u);
    check(bp_fail(&bp) == BP_FAIL_AROUND_TIME, "around 10000 ms: fails");

    /* Time wrap. */
    bp_init(&bp, &CFG, OBSTACLE_SIDE_LEFT, OBST_MM, 0xFFFFFFF0u);
    check(!bp_wants_reading(&bp, 0xFFFFFFFFu), "settle: not before the wrap");
    check(!bp_wants_reading(&bp, 0x0000008Fu) && bp_wants_reading(&bp, 0x00000090u),
          "settle over the time wrap (160 ms)");
    (void)bp_update(&bp, true, false, travel, 0xFFFFFFF5u);
    check(bp_state(&bp) == BP_PIVOT, "pivot limit: running just after the start, before the wrap");
    (void)bp_update(&bp, true, false, travel, 0x00000F8Fu);
    check(bp_state(&bp) == BP_PIVOT, "pivot limit over the wrap: running");
    (void)bp_update(&bp, true, false, travel, 0x00000F90u);
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
    w = bp_update(&bp, false, false, travel, now);
    check(w == (bp_word_t)(2u | (22u << 2) | (0u << 8)), "word: STEER, steer -8, servo -30");

    if (failures == 0) {
        printf("test_obstacle_bypass: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_bypass: %d failure(s)\n", failures);
    return 1;
}
