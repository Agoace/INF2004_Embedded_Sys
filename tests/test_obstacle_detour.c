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
    .look_deg = 30u,
    .half_width_mm = 80u,
    .sweep_mm = 150u,
    .margin_mm = 30u,
    .sensor_ahead_mm = 65u,
    .line_ahead_mm = 100u,
    .depth_mm = 150u,
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

/* One reading after the servo has settled. */
static void read(dt_t *dt, int mm) {
    now = dt->settle_ms;
    dt_triggered(dt);
    now += 10u;
    dt_reading(dt, result(mm), now);
}

static dt_t start(obstacle_side_t side, uint16_t obstacle_mm, unsigned int free_step) {
    dt_t dt;
    now = 1000u;
    cl = 5000;
    cr = -3000;
    dt_init(&dt, &CFG, side, obstacle_mm, free_step, cl, cr, now);
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
    check(dt_counts_to_mm(4569) == 368 && dt_counts_to_mm(4568) == 367,
          "counts to mm, rounded toward zero");
    check(dt_counts_to_mm(-4569) == -368, "counts to mm: negative");

    /* No free side. */
    dt = start(OBSTACLE_SIDE_NONE, 220u, 4u);
    w = update(&dt, false);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_NO_SIDE &&
              dt_move(w) == DT_MOVE_STOP,
          "side NONE fails and stops");

    /* Plan for 220 mm, free step 4 (20 degrees). */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    check(dt.front_x == 285 && dt.edge_y == 80, "obstacle: front 65 + 220, edge 220 tan 20 = 80");
    check(dt.lane_y == 260 && dt.leg_mm == 368u, "lane 80 + 150 + 30 = 260; OUT 260 / sin 45 = 368");
    check(dt_step(&dt) == DT_TURN_OUT, "front 285 >= 235 needed: no BACK");

    /* TURN_OUT: left (toward S), 45 degrees = 1083 counts of difference. */
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1 && dt_servo_deg(w) == 0,
          "TURN_OUT: turn left, servo ahead");
    check(!dt_wants_reading(&dt, now + 1000u), "TURN_OUT: no readings");
    w = turn(&dt, 1082);
    check(dt_step(&dt) == DT_TURN_OUT, "1082 counts: still turning");
    w = turn(&dt, 1);
    check(dt_step(&dt) == DT_OUT, "1083 counts: OUT");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == +1 && dt_servo_deg(w) == -30,
          "OUT: straight, servo 30 toward the obstacle (right)");
    check(!dt_wants_reading(&dt, now + 159u) && dt_wants_reading(&dt, now + 160u),
          "servo 0 -> -30: 160 ms settle");

    /* OUT: a point farther toward S moves the lane out. Sensor at
     * (45, 45), ray 45 - 30 = 15: 300 mm -> (334, 122). */
    read(&dt, 300);
    check(dt.edge_y == 122 && dt.lane_y == 302 && dt.leg_mm == 428u,
          "OUT: point y 122 > 80: lane 302, OUT 428 mm");
    check(!dt_wants_reading(&dt, now + 39u) && dt_wants_reading(&dt, now + 40u),
          "after a reading: 40 ms settle");
    read(&dt, 100); /* (45 + 96, 45 + 25): y 70, not farther */
    check(dt.edge_y == 122, "OUT: a point nearer the line does not move the lane in");
    read(&dt, 400); /* 400 = seen_mm: not the obstacle */
    check(dt.edge_y == 122, "OUT: 400 mm (seen_mm) is not the obstacle");
    read(&dt, NR);
    read(&dt, RISE);
    check(dt_step(&dt) == DT_OUT, "OUT: no echo, no information: no change");

    /* Back to the first plan for exact numbers. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083);
    w = drive(&dt, 4568); /* 367 mm */
    check(dt_step(&dt) == DT_OUT, "OUT 367 of 368 mm: driving");
    w = drive(&dt, 1); /* 368 mm */
    check(dt_step(&dt) == DT_TURN_BACK && dt.x == 260 && dt.y == 260,
          "OUT done: B = (260, 260)");
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "TURN_BACK: turn right");
    w = turn(&dt, -1082);
    check(dt_step(&dt) == DT_TURN_BACK, "TURN_BACK 1082: turning");
    w = turn(&dt, -1);
    check(dt_step(&dt) == DT_PASS && dt.leg_mm == 355u,
          "PASS: C at 285 + 150 depth + 180 = 615; 615 - 260 = 355 mm");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_servo_deg(w) == -30, "PASS: straight, look right");

    /* PASS: a point farther ahead makes PASS longer. Sensor (325, 260),
     * ray -30: 350 mm -> (628, 85). */
    now = dt.settle_ms;
    read(&dt, 350);
    check(dt.seen_x == 628 && dt.leg_mm == 548u, "PASS: point x 628: C 808, PASS 548 mm");
    (void)drive(&dt, (int32_t)dt_mm_to_counts(100u)); /* 1241 counts: 99 mm */
    read(&dt, 300); /* sensor (424, 260): 300 x 0.866 = 259 -> (683, 110) */
    check(dt.seen_x == 683 && dt.leg_mm == 603u, "PASS: point seen after 99 mm: C 863");
    read(&dt, 200); /* (598, 160): y 160 > 302 - 110? lane is 260: 260 - 110 = 150 */
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_SIDE,
          "PASS: point y 160 inside the lane (> 150): SIDE");

    /* Full detour, left. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083);
    (void)drive(&dt, 4569);
    (void)turn(&dt, -1083);
    now = dt.settle_ms;
    read(&dt, 300); /* (325 + 259, 260 - 150) = (584, 110): y 110 <= 150, x > 435 */
    check(dt.seen_x == 584 && dt.leg_mm == 504u, "PASS: x 584: C 764, PASS 504 mm");
    w = drive(&dt, (int32_t)dt_mm_to_counts(503u));
    check(dt_step(&dt) == DT_PASS, "PASS 503 mm: driving");
    w = drive(&dt, 13);
    check(dt_step(&dt) == DT_TURN_IN && dt.x == 764 && dt.y == 260, "PASS done: C = (764, 260)");
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "TURN_IN: turn right");
    w = turn(&dt, -1083);
    check(dt_step(&dt) == DT_IN && dt.leg_mm == 736u, "IN: guard 2 x 368 = 736 mm");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_servo_deg(w) == 0, "IN: straight, servo ahead");
    now = dt.settle_ms;
    read(&dt, 80);
    check(dt_step(&dt) == DT_IN, "IN: 80 mm ahead is not too near");
    read(&dt, 79);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_TOO_NEAR, "IN: 79 mm: TOO_NEAR");

    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083);
    (void)drive(&dt, 4569);
    (void)turn(&dt, -1083);
    (void)drive(&dt, 4407); /* 355 mm */
    check(dt_step(&dt) == DT_TURN_IN, "PASS 355 mm with no points: TURN_IN");
    (void)turn(&dt, -1083);
    (void)drive(&dt, 4000);
    check(dt_step(&dt) == DT_IN, "IN: no line yet");
    cl += 10;
    cr += 10;
    w = update(&dt, true);
    check(dt_step(&dt) == DT_ON_LINE && dt.leg_mm == 100u, "line seen: ON_LINE 100 mm");
    check(dt.x == 615 + 228 && dt.y == 260 - 228, "IN 4010 counts = 323 mm at -45: x +228, y -228");
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == +1, "ON_LINE: straight on");
    (void)drive(&dt, 1242); /* 100 mm */
    check(dt_step(&dt) == DT_ALIGN, "ON_LINE 100 mm: ALIGN");
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1, "ALIGN: turn left");
    w = turn(&dt, 1083);
    check(dt_step(&dt) == DT_DONE && dt_move(w) == DT_MOVE_STOP, "ALIGN done: DONE, stop");
    check(!dt_wants_reading(&dt, now + 1000u), "DONE: no readings");

    /* No line in IN. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083);
    (void)drive(&dt, 4569);
    (void)turn(&dt, -1083);
    (void)drive(&dt, 4407);
    (void)turn(&dt, -1083);
    (void)drive(&dt, (int32_t)dt_mm_to_counts(735u));
    check(dt_step(&dt) == DT_IN, "IN 735 of 736 mm: still looking");
    (void)drive(&dt, 13);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_NO_LINE, "IN 736 mm: NO_LINE");

    /* Detour right: the mirror. */
    dt = start(OBSTACLE_SIDE_RIGHT, 220u, 4u);
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "right: TURN_OUT turns right");
    w = turn(&dt, -1082);
    check(dt_step(&dt) == DT_TURN_OUT, "right: -1082 counts: still turning");
    w = turn(&dt, -1);
    check(dt_step(&dt) == DT_OUT && dt_servo_deg(w) == 30, "right: OUT looks left (+30)");
    w = turn(&dt, 2000); /* a wrong-way turn is not progress */
    check(dt_step(&dt) == DT_OUT, "right: turning is ignored in OUT");
    (void)drive(&dt, 4569);
    w = update(&dt, false);
    check(dt_step(&dt) == DT_TURN_BACK && dt_dir(w) == +1, "right: TURN_BACK turns left");
    w = turn(&dt, 1083);
    check(dt_step(&dt) == DT_PASS, "right: PASS");
    now = dt.settle_ms;
    read(&dt, 300);
    check(dt.seen_x == 584, "right: PASS point in the S frame");

    /* BACK when too near. 112 mm, free step 4: edge 112 tan 20 = 40,
     * front 177; need max(180, (28 + 110) / sin 45 = 196) = 196. */
    dt = start(OBSTACLE_SIDE_LEFT, 112u, 4u);
    check(dt_step(&dt) == DT_BACK && dt.leg_mm == 19u, "112 mm: BACK 196 - 177 = 19 mm");
    w = update(&dt, false);
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == -1 && dt_servo_deg(w) == 0,
          "BACK: straight back, servo ahead");
    check(!dt_wants_reading(&dt, now + 1000u), "BACK: no readings");
    (void)drive(&dt, -(int32_t)dt_mm_to_counts(18u));
    check(dt_step(&dt) == DT_BACK, "BACK 18 mm: driving");
    (void)drive(&dt, -13);
    check(dt_step(&dt) == DT_TURN_OUT && dt.front_x == 196, "BACK 19 mm: front now 196, TURN_OUT");
    /* 150 mm, step 2: edge 150 tan 10 = 26, front 215; need max(180,
     * (18 + 110) / sin 45 = 182) = 182: no BACK. */
    dt = start(OBSTACLE_SIDE_LEFT, 150u, 2u);
    check(dt.edge_y == 26 && dt_step(&dt) == DT_TURN_OUT, "150 mm, step 2: front 215 >= 182");
    /* 140 mm: front 205; edge 24 (16 + 110 -> 179); need 180: BACK 0? 205 >= 180. 110 mm,
     * step 1: edge 110 tan 5 = 9, front 175; need max(180, (6 + 110) / 0.707 = 165) = 180. */
    dt = start(OBSTACLE_SIDE_LEFT, 110u, 1u);
    check(dt.edge_y == 9 && dt_step(&dt) == DT_BACK && dt.leg_mm == 5u,
          "110 mm, step 1: turn circle needs 180: BACK 5 mm");

    /* Turn end exactly at the target: 45 x 2400 = 108000 = 1080 x 100. */
    dt_config_t exact = CFG;
    exact.counts_per_deg_x100 = 2400u;
    now = 1000u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, 4u, cl, cr, now);
    (void)turn(&dt, 1079);
    check(dt_step(&dt) == DT_TURN_OUT, "exact: 1079 counts: turning");
    (void)turn(&dt, 1);
    check(dt_step(&dt) == DT_OUT, "exact: 1080 counts = the target: OUT");

    /* Time limits. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    now = 1000u + 3989u;
    (void)update(&dt, false);
    check(dt_step(&dt) == DT_TURN_OUT, "turn 3999 ms: running");
    (void)update(&dt, false);
    check(dt_fail(&dt) == DT_FAIL_TURN_TIME, "turn 4009 ms: fails");
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083); /* OUT: no turn limit */
    now = 1000u + 29989u;
    (void)update(&dt, false);
    check(dt_step(&dt) == DT_OUT, "OUT: no turn limit; 29999 ms: running");
    (void)update(&dt, false);
    check(dt_fail(&dt) == DT_FAIL_TIME, "detour 30009 ms: fails");

    /* Sensor fault; readings without a trigger. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083);
    dt_reading(&dt, result(300), now);
    check(dt.edge_y == 80, "a reading with no trigger is ignored");
    read(&dt, FAULT);
    check(dt_fail(&dt) == DT_FAIL_SENSOR, "fault streak fails");

    /* Word layout. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u, 4u);
    (void)turn(&dt, 1083);
    w = update(&dt, false);
    check(w == (dt_word_t)(2u | (0u << 3)), "word: STRAIGHT forward, servo -30");
    dt = start(OBSTACLE_SIDE_RIGHT, 220u, 4u);
    w = update(&dt, false);
    check(w == (dt_word_t)(1u | 4u | (30u << 3)), "word: TURN right, servo 0");

    if (failures == 0) {
        printf("test_obstacle_detour: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_detour: %d failure(s)\n", failures);
    return 1;
}
