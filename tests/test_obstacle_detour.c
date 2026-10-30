/*
 * Host unit test for src/obstacle_detour.c. It needs no Pico hardware.
 *   cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
 *      tests/test_obstacle_detour.c src/obstacle_detour.c \
 *      -o build/test_obstacle_detour
 *   ./build/test_obstacle_detour
 */
#include <stdio.h>

#include "obstacle_detour.h"

static int failures;

static void check(int ok, const char *what) {
    if (!ok) {
        printf("FAIL %s\n", what);
        ++failures;
    }
}

static const dt_config_t CFG = {
    .turn_deg = 45u,
    .vehicle_mm = 190u,
    .sweep_mm = 150u,
    .margin_mm = 30u,
    .sensor_ahead_mm = 65u,
    .line_ahead_mm = 100u,
    .seen_mm = 400u,
    .counts_per_deg_x100 = 2405u, /* 45 degrees: 1082.25 counts */
    .turn_max_ms = 4000u,
    .total_max_ms = 30000u,
};

#define NR -1    /* NO_RETURN */
#define RISE -2  /* NO_RISE, streak 1 */
#define FAULT -3 /* NO_RISE, streak 5 */

static ultrasonic_result_t result(int mm) {
    if (mm == NR) {
        return (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN << ULTRASONIC_STATUS_SHIFT);
    }
    if (mm == RISE) {
        return (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT |
                                     1u << ULTRASONIC_STREAK_SHIFT);
    }
    if (mm == FAULT) {
        return (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT |
                                     5u << ULTRASONIC_STREAK_SHIFT);
    }
    return (ultrasonic_result_t)mm;
}

static uint32_t now;
static int32_t cl, cr; /* wheel counts */

static dt_word_t update(dt_t *dt, bool line) {
    now += 10u;
    return dt_update(dt, line, cl, cr, now);
}

/* Turn on the spot: diff = right - left change. */
static dt_word_t turn(dt_t *dt, int32_t diff) {
    cr += diff / 2;
    cl -= diff - diff / 2;
    return update(dt, false);
}

static dt_word_t drive(dt_t *dt, int32_t counts) {
    cl += counts;
    cr += counts;
    return update(dt, false);
}

/* Drive to mm from the start of the step (mm >= the distance now). */
static dt_word_t drive_to(dt_t *dt, uint32_t mm) {
    const int32_t at = ((cl - dt->l0) + (cr - dt->r0)) / 2;
    int32_t c = (int32_t)dt_mm_to_counts(mm) - 2;
    while (dt_counts_to_mm(c) < (int32_t)mm) {
        ++c; /* the fewest counts that read as mm */
    }
    return drive(dt, c - at);
}

/* One reading after the servo has settled, triggered where the robot is. */
static void read(dt_t *dt, int mm) {
    if ((int32_t)(now - dt->settle_ms) < 0) {
        now = dt->settle_ms;
    }
    check(dt_wants_reading(dt, now), "read: a reading is wanted");
    dt_triggered(dt);
    now += 10u;
    dt_reading(dt, result(mm), now);
}

static dt_t start(obstacle_side_t side, uint16_t obstacle_mm) {
    dt_t dt;
    now = 1000u;
    cl = 5000;
    cr = -3000;
    dt_init(&dt, &CFG, side, obstacle_mm, cl, cr, now);
    return dt;
}

/* From the start to PASS, with no readings: OUT 190 mm. */
static dt_t to_pass(obstacle_side_t side) {
    dt_t dt = start(side, 220u);
    const int s = side == OBSTACLE_SIDE_LEFT ? +1 : -1;
    (void)turn(&dt, s * 1083);
    (void)drive_to(&dt, 190u);
    (void)turn(&dt, -s * 1083);
    return dt;
}

int main(void) {
    dt_t dt;
    dt_word_t w;

    /* Helpers. */
    check(dt_sin1000(30) == 500 && dt_sin1000(-30) == -500 && dt_sin1000(90) == 1000,
          "sin: 30, -30, 90");
    check(dt_cos1000(0) == 1000 && dt_cos1000(-60) == 500 && dt_cos1000(90) == 0,
          "cos: 0, -60, 90");
    check(dt_sin1000(120) == 1000 && dt_cos1000(-100) == 0, "sin/cos clamp to +-90");
    check(dt_mm_to_counts(1000u) == 12414u && dt_mm_to_counts(0u) == 0u, "mm to counts");
    check(dt_mm_to_counts(4u) == 50u, "4 mm = 49.66 counts: rounded to 50");
    check(dt_counts_to_mm(2359) == 190 && dt_counts_to_mm(2358) == 189,
          "counts to mm, rounded toward zero");
    check(dt_counts_to_mm(-2359) == -190, "counts to mm: negative");

    /* No free side. */
    dt = start(OBSTACLE_SIDE_NONE, 220u);
    w = update(&dt, false);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_NO_SIDE &&
              dt_move(w) == DT_MOVE_STOP,
          "side NONE fails and stops");

    /* TURN_OUT: left (toward S), 45 degrees = 1083 counts of difference. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    check(dt_step(&dt) == DT_TURN_OUT, "220 mm: front 285 >= 180: no BACK");
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1 && dt_servo_deg(w) == 0,
          "TURN_OUT: turn left, servo ahead");
    check(!dt_wants_reading(&dt, now + 1000u), "TURN_OUT: no readings");
    (void)turn(&dt, 1082);
    check(dt_step(&dt) == DT_TURN_OUT, "1082 counts: still turning");
    w = turn(&dt, 1);
    check(dt_step(&dt) == DT_OUT, "1083 counts: OUT");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == +1 && dt_servo_deg(w) == -30,
          "OUT: straight, servo 30 toward the obstacle (right)");
    check(!dt_wants_reading(&dt, now + 159u) && dt_wants_reading(&dt, now + 160u),
          "servo 0 -> -30: 160 ms settle");

    /* OUT: count from the nearest reading. */
    read(&dt, 300); /* at 0 mm */
    check(dt.best_mm == 300u && dt.from_mm == 0, "OUT: 300 mm at 0: from 0");
    check(!dt_wants_reading(&dt, now + 39u) && dt_wants_reading(&dt, now + 40u),
          "after a reading: 40 ms settle");
    (void)drive_to(&dt, 100u);
    read(&dt, 250);
    check(dt.best_mm == 250u && dt.from_mm == 100, "OUT: 250 mm at 100: from 100");
    (void)drive_to(&dt, 150u);
    read(&dt, 250);
    check(dt.from_mm == 100, "OUT: the same distance again is not nearer");
    read(&dt, 260);
    read(&dt, NR);
    read(&dt, RISE);
    check(dt.from_mm == 100 && dt_step(&dt) == DT_OUT, "OUT: farther, no echo, no info");
    now = dt.settle_ms;
    dt_triggered(&dt);
    check(!dt_wants_reading(&dt, now + 1000u), "no new reading while one is pending");
    dt_reading(&dt, result(NR), now);
    (void)drive_to(&dt, 289u);
    check(dt_step(&dt) == DT_OUT, "OUT: 189 mm after the nearest: driving");
    w = drive_to(&dt, 290u);
    check(dt_step(&dt) == DT_TURN_BACK, "OUT: 190 mm after the nearest: TURN_BACK");
    check(dt.out_mm == 290u && dt.x == 205 && dt.y == 205, "OUT 290 mm at 45: B (205, 205)");
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1 && dt_servo_deg(w) == -30,
          "TURN_BACK: turn right, servo stays");
    check(!dt_wants_reading(&dt, now + 1000u), "TURN_BACK: no readings");

    /* seen_mm: 400 is not the obstacle, 399 is. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083);
    (void)drive_to(&dt, 50u);
    read(&dt, 400);
    check(dt.from_mm == 0 && dt.best_mm == 0xFFFFu, "OUT: 400 mm (seen_mm): not the obstacle");
    read(&dt, 399);
    check(dt.from_mm == 50, "OUT: 399 mm: nearest");
    /* The count starts where the reading was triggered, not where it came in. */
    read(&dt, 380);
    check(dt.from_mm == 50, "OUT: nearer reading at the same place");
    now = dt.settle_ms;
    dt_triggered(&dt);
    (void)drive_to(&dt, 80u);
    dt_reading(&dt, result(370), now);
    check(dt.from_mm == 50, "OUT: from the trigger place (50), not 80");

    /* No readings: 190 mm from the start of the straight. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083);
    (void)drive_to(&dt, 189u);
    check(dt_step(&dt) == DT_OUT, "OUT, no echo: 189 mm: driving");
    (void)drive_to(&dt, 190u);
    check(dt_step(&dt) == DT_TURN_BACK && dt.out_mm == 190u, "OUT, no echo: 190 mm");

    /* PASS: the nearest is per straight; OUT readings do not carry over. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083);
    (void)drive_to(&dt, 50u);
    read(&dt, 100);
    (void)drive_to(&dt, 240u);
    check(dt_step(&dt) == DT_TURN_BACK, "OUT done 190 mm after 50");
    (void)turn(&dt, -1082);
    check(dt_step(&dt) == DT_TURN_BACK, "TURN_BACK 1082: turning");
    w = turn(&dt, -1);
    check(dt_step(&dt) == DT_PASS && dt.best_mm == 0xFFFFu && dt.from_mm == 0,
          "PASS: starts with no nearest reading");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_servo_deg(w) == -30, "PASS: straight, look right");
    (void)drive_to(&dt, 189u);
    check(dt_step(&dt) == DT_PASS, "PASS: OUT's nearest (50) is not used: 189 driving");
    read(&dt, 350); /* at 189 */
    (void)drive_to(&dt, 260u);
    read(&dt, 340); /* at 260: nearer */
    (void)drive_to(&dt, 300u);
    read(&dt, 345); /* at 300: farther */
    (void)drive_to(&dt, 449u);
    check(dt_step(&dt) == DT_PASS, "PASS: 189 mm after the nearest (260): driving");
    w = drive_to(&dt, 450u);
    check(dt_step(&dt) == DT_TURN_IN && dt.x == 169 + 450 && dt.y == 169,
          "PASS: 190 mm after: TURN_IN; C (619, 169)");
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "TURN_IN: turn right");
    w = turn(&dt, -1083);
    check(dt_step(&dt) == DT_IN && dt.leg_mm == 480u, "IN: guard 2 x OUT 240 = 480 mm");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_servo_deg(w) == 0, "IN: straight, servo ahead");
    read(&dt, 80);
    check(dt_step(&dt) == DT_IN, "IN: 80 mm ahead is not too near");
    read(&dt, 79);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_TOO_NEAR, "IN: 79 mm: TOO_NEAR");

    /* IN, ON_LINE, ALIGN, DONE. */
    dt = to_pass(OBSTACLE_SIDE_LEFT);
    (void)drive_to(&dt, 190u);
    check(dt_step(&dt) == DT_TURN_IN, "PASS, no echo: 190 mm");
    (void)turn(&dt, -1083);
    read(&dt, 300); /* far ahead: no effect in IN */
    check(dt_step(&dt) == DT_IN && dt.from_mm == 0, "IN: a far echo changes nothing");
    (void)drive_to(&dt, 200u);
    check(dt_step(&dt) == DT_IN, "IN: no line yet");
    cl += 10;
    cr += 10;
    w = update(&dt, true);
    check(dt_step(&dt) == DT_ON_LINE && dt.leg_mm == 100u, "line seen: ON_LINE 100 mm");
    check(dt.x == 134 + 190 + 141 && dt.y == 134 - 141, "IN 200 mm at -45: x +141, y -141");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == +1, "ON_LINE: straight on");
    (void)drive_to(&dt, 99u);
    check(dt_step(&dt) == DT_ON_LINE, "ON_LINE 99 mm: driving");
    (void)drive_to(&dt, 100u);
    check(dt_step(&dt) == DT_ALIGN, "ON_LINE 100 mm: ALIGN");
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1, "ALIGN: turn left");
    (void)turn(&dt, 1082);
    check(dt_step(&dt) == DT_ALIGN, "ALIGN 1082: turning");
    w = turn(&dt, 1);
    check(dt_step(&dt) == DT_DONE && dt_move(w) == DT_MOVE_STOP, "ALIGN done: DONE, stop");
    check(!dt_wants_reading(&dt, now + 1000u), "DONE: no readings");

    /* No line in IN. */
    dt = to_pass(OBSTACLE_SIDE_LEFT);
    (void)drive_to(&dt, 190u);
    (void)turn(&dt, -1083);
    (void)drive_to(&dt, 379u);
    check(dt_step(&dt) == DT_IN, "IN 379 of 380 mm: still looking");
    (void)drive_to(&dt, 380u);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_NO_LINE, "IN 380 mm: NO_LINE");

    /* Detour right: the mirror. */
    dt = start(OBSTACLE_SIDE_RIGHT, 220u);
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "right: TURN_OUT turns right");
    (void)turn(&dt, -1082);
    check(dt_step(&dt) == DT_TURN_OUT, "right: -1082 counts: still turning");
    w = turn(&dt, -1);
    check(dt_step(&dt) == DT_OUT && dt_servo_deg(w) == 30, "right: OUT looks left (+30)");
    (void)turn(&dt, 2000); /* turning is not straight travel */
    check(dt_step(&dt) == DT_OUT, "right: turning is ignored in OUT");
    (void)drive_to(&dt, 190u);
    w = update(&dt, false);
    check(dt_step(&dt) == DT_TURN_BACK && dt_dir(w) == +1, "right: TURN_BACK turns left");
    check(dt.y == 134, "right: y is toward S");
    (void)turn(&dt, 1083);
    check(dt_step(&dt) == DT_PASS, "right: PASS");
    (void)drive_to(&dt, 190u);
    w = update(&dt, false);
    check(dt_step(&dt) == DT_TURN_IN && dt_dir(w) == +1, "right: TURN_IN turns left");
    (void)turn(&dt, 1083);
    (void)drive_to(&dt, 50u);
    (void)update(&dt, true);
    (void)drive_to(&dt, 100u);
    w = update(&dt, false);
    check(dt_step(&dt) == DT_ALIGN && dt_dir(w) == -1, "right: ALIGN turns right");

    /* BACK when the obstacle is inside the turn circle: 180 - (65 + 100). */
    dt = start(OBSTACLE_SIDE_LEFT, 100u);
    check(dt_step(&dt) == DT_BACK && dt.leg_mm == 15u, "100 mm: BACK 15 mm");
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == -1 && dt_servo_deg(w) == 0,
          "BACK: straight back, servo ahead");
    check(!dt_wants_reading(&dt, now + 1000u), "BACK: no readings");
    (void)drive(&dt, -(int32_t)dt_mm_to_counts(14u));
    check(dt_step(&dt) == DT_BACK, "BACK 14 mm: driving");
    (void)drive(&dt, -13);
    check(dt_step(&dt) == DT_TURN_OUT, "BACK 15 mm: TURN_OUT");
    dt = start(OBSTACLE_SIDE_LEFT, 115u);
    check(dt_step(&dt) == DT_TURN_OUT, "115 mm: front 180 = need: no BACK");

    /* Turn end exactly at the target: 45 x 2400 = 108000 = 1080 x 100. */
    dt_config_t exact = CFG;
    exact.counts_per_deg_x100 = 2400u;
    now = 1000u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, cl, cr, now);
    (void)turn(&dt, 1079);
    check(dt_step(&dt) == DT_TURN_OUT, "exact: 1079 counts: turning");
    (void)turn(&dt, 1);
    check(dt_step(&dt) == DT_OUT, "exact: 1080 counts = the target: OUT");

    /* turn_deg is clamped to 10-80: 90 turns 80 = 1924 counts. */
    exact = CFG;
    exact.turn_deg = 90u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, cl, cr, now);
    (void)turn(&dt, 1923);
    check(dt_step(&dt) == DT_TURN_OUT, "90 -> 80: 1923 counts: turning");
    (void)turn(&dt, 1);
    check(dt_step(&dt) == DT_OUT, "90 -> 80: 1924 counts: OUT");

    /* Time limits. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    now = 1000u + 3989u;
    (void)update(&dt, false);
    check(dt_step(&dt) == DT_TURN_OUT, "turn 3999 ms: running");
    (void)update(&dt, false);
    check(dt_fail(&dt) == DT_FAIL_TURN_TIME, "turn 4009 ms: fails");
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083); /* OUT: no turn limit */
    now = 1000u + 29989u;
    (void)update(&dt, false);
    check(dt_step(&dt) == DT_OUT, "OUT: no turn limit; 29999 ms: running");
    (void)update(&dt, false);
    check(dt_fail(&dt) == DT_FAIL_TIME, "detour 30009 ms: fails");

    /* Sensor fault; readings without a trigger. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083);
    dt_reading(&dt, result(300), now);
    check(dt.best_mm == 0xFFFFu, "a reading with no trigger is ignored");
    read(&dt, FAULT);
    check(dt_fail(&dt) == DT_FAIL_SENSOR, "fault streak fails");

    /* A reading that comes in after the step changed to a turn is ignored. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083);
    now = dt.settle_ms;
    dt_triggered(&dt);
    (void)drive_to(&dt, 190u);
    check(dt_step(&dt) == DT_TURN_BACK, "turn before the reading came in");
    dt_reading(&dt, result(FAULT), now);
    check(dt_step(&dt) == DT_TURN_BACK && !dt.pending, "reading in a turn: ignored, not pending");

    /* Word layout. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)turn(&dt, 1083);
    w = update(&dt, false);
    check(w == (dt_word_t)(2u | (0u << 3)), "word: STRAIGHT forward, servo -30");
    dt = start(OBSTACLE_SIDE_RIGHT, 220u);
    w = update(&dt, false);
    check(w == (dt_word_t)(1u | 4u | (30u << 3)), "word: TURN right, servo 0");

    if (failures == 0) {
        printf("test_obstacle_detour: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_detour: %d failure(s)\n", failures);
    return 1;
}
