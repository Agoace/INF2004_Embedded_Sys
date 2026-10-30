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
    .turn_step_deg = 5u,
    .away_max_deg = 90u,
    .toward_max_deg = 150u,
    .max_straights = 5u,
    .back_off_deg = 10u,
    .look_ms = 150u,
    .vehicle_mm = 190u,
    .sweep_mm = 150u,
    .margin_mm = 30u,
    .sensor_ahead_mm = 65u,
    .line_ahead_mm = 100u,
    .seen_mm = 400u,
    .counts_per_deg_x100 = 2405u, /* 5 degrees: 120.25 counts */
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
static bool line;      /* line sensors */

static dt_word_t update(dt_t *dt) {
    now += 10u;
    return dt_update(dt, line, cl, cr, now);
}

/* Set the turn of this step to diff_s counts of (right - left), S frame. */
static dt_word_t turn_to(dt_t *dt, int32_t diff_s) {
    const int32_t want = dt->side * diff_s;
    const int32_t have = (cr - dt->r0) - (cl - dt->l0);
    const int32_t delta = want - have;
    cr += delta / 2;
    cl -= delta - delta / 2;
    return update(dt);
}

/* Drive to mm from the start of the step: the fewest counts that read as mm. */
static dt_word_t drive_to(dt_t *dt, int32_t mm) {
    const int32_t at = ((cl - dt->l0) + (cr - dt->r0)) / 2;
    int32_t c = (int32_t)dt_mm_to_counts((uint32_t)(mm < 0 ? -mm : mm)) - 2;
    while (dt_counts_to_mm(c) < (mm < 0 ? -mm : mm)) {
        ++c;
    }
    const int32_t delta = (mm < 0 ? -c : c) - at;
    cl += delta;
    cr += delta;
    return update(dt);
}

/* One reading when it is wanted. */
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
    line = false;
    dt_init(&dt, &CFG, side, obstacle_mm, cl, cr, now);
    return dt;
}

/* Counts for deg degrees: ceil(deg x 2405 / 100). */
static int32_t counts(int deg) {
    return (deg * 2405 + 99) / 100;
}

/* Bottle, side S: looks NR at 0, seen at 5 and 10, NR at 15: heading 15. */
static dt_t to_straight(obstacle_side_t side) {
    dt_t dt = start(side, 220u);
    (void)update(&dt);
    read(&dt, NR);
    (void)turn_to(&dt, counts(5));
    read(&dt, 250);
    (void)turn_to(&dt, counts(10));
    read(&dt, 260);
    (void)turn_to(&dt, counts(15));
    read(&dt, NR);
    return dt;
}

int main(void) {
    dt_t dt;
    dt_word_t w;

    /* Helpers. */
    check(dt_sin1000(30) == 500 && dt_sin1000(-30) == -500 && dt_sin1000(90) == 1000,
          "sin: 30, -30, 90");
    check(dt_sin1000(150) == 500 && dt_sin1000(180) == 0 && dt_sin1000(270) == -1000 &&
              dt_sin1000(-200) == 342 && dt_sin1000(450) == 1000 && dt_sin1000(-450) == -1000,
          "sin: any angle");
    check(dt_sin1000(185) == -87 && dt_sin1000(-185) == 87 && dt_sin1000(750) == 500 &&
              dt_sin1000(91) == 1000 && dt_sin1000(-91) == -1000,
          "sin: 185, -185, 750, 91, -91");
    check(dt_cos1000(0) == 1000 && dt_cos1000(-60) == 500 && dt_cos1000(180) == -1000 &&
              dt_cos1000(-135) == -707,
          "cos: any angle");
    check(dt_mm_to_counts(1000u) == 12414u && dt_mm_to_counts(4u) == 50u, "mm to counts");
    check(dt_counts_to_mm(2359) == 190 && dt_counts_to_mm(2358) == 189 &&
              dt_counts_to_mm(-2359) == -190,
          "counts to mm, toward zero");

    /* No free side. */
    dt = start(OBSTACLE_SIDE_NONE, 220u);
    w = update(&dt);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_NO_SIDE &&
              dt_move(w) == DT_MOVE_STOP,
          "side NONE fails and stops");

    /* TURN_AWAY, bottle, left. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    check(dt_step(&dt) == DT_TURN_AWAY, "220 mm: front 285 >= 180: no BACK");
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_STOP && dt_servo_deg(w) == -30, "first look before turning");
    check(!dt_wants_reading(&dt, 1159u) && dt_wants_reading(&dt, 1160u),
          "servo 0 -> -30: wait 160 ms (look 150 ms)");
    read(&dt, NR);
    check(!dt.looking && dt.target_deg == 5u && !dt.seen_once, "NR at 0: turn to 5");
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1, "TURN_AWAY: turn left (toward S)");
    check(!dt_wants_reading(&dt, now + 1000u), "no reading while turning");
    w = turn_to(&dt, 120);
    check(!dt.looking && dt_move(w) == DT_MOVE_TURN, "120 counts: still turning");
    w = turn_to(&dt, 121);
    check(dt.looking && dt_move(w) == DT_MOVE_STOP, "121 counts (5 deg): stop and look");
    check(!dt_wants_reading(&dt, now + 149u) && dt_wants_reading(&dt, now + 150u),
          "look: 150 ms stopped");
    read(&dt, 250);
    check(dt.seen_once && dt.target_deg == 10u, "seen at 5: turn to 10");
    (void)turn_to(&dt, counts(10));
    read(&dt, RISE);
    check(dt.looking && dt.target_deg == 10u, "NO_RISE: look again, no turn");
    check(!dt_wants_reading(&dt, now + 39u) && dt_wants_reading(&dt, now + 40u),
          "look again after 40 ms");
    read(&dt, 399);
    check(dt.target_deg == 15u, "399 mm is seen: turn to 15");
    (void)turn_to(&dt, counts(15) + 30); /* coasting: 16.2 degrees */
    read(&dt, 400);
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 16,
          "400 mm is not seen: past the edge; heading 16 (coasting counted)");
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == +1 && dt_servo_deg(w) == -30,
          "STRAIGHT: forward, servo 30 toward the obstacle");

    /* TURN_AWAY, box: seen at once, then not seen. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)update(&dt);
    read(&dt, 230);
    (void)turn_to(&dt, counts(5));
    read(&dt, NR);
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 5, "box: seen at 0, NR at 5: heading 5");

    /* TURN_AWAY limit: never seen. Looks at 0..90, then 95 > 90 fails. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)update(&dt);
    for (int deg = 0; deg <= 85; deg += 5) {
        (void)turn_to(&dt, counts(deg));
        read(&dt, NR);
    }
    check(dt_step(&dt) == DT_TURN_AWAY && dt.target_deg == 90u, "never seen: look at 90");
    (void)turn_to(&dt, counts(90));
    read(&dt, NR);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_TURN_LIMIT,
          "never seen: 95 > 90 fails");

    /* STRAIGHT: the nearest reading. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 15, "to_straight: heading 15");
    read(&dt, 300); /* at 0 */
    check(dt.best_mm == 300u && dt.from_mm == 0, "300 at 0: from 0");
    check(!dt_wants_reading(&dt, now + 39u) && dt_wants_reading(&dt, now + 40u),
          "straight: 40 ms between readings");
    (void)drive_to(&dt, 100);
    read(&dt, 250);
    check(dt.from_mm == 100, "250 at 100: from 100");
    (void)drive_to(&dt, 110);
    read(&dt, 250);
    read(&dt, 260);
    read(&dt, NR);
    read(&dt, RISE);
    check(dt.from_mm == 100 && dt.best_mm == 250u, "same, farther, NR, NO_RISE: no change");
    read(&dt, 80);
    check(dt_step(&dt) == DT_STRAIGHT && dt.from_mm == 110, "80 mm at 110: not too near, nearest");
    dt.best_mm = 250u; /* back to 250 at 100: keep the numbers below simple */
    dt.from_mm = 100;
    now = dt.settle_ms;
    dt_triggered(&dt);
    check(!dt_wants_reading(&dt, now + 1000u), "no new reading while one is pending");
    (void)drive_to(&dt, 150);
    dt_reading(&dt, result(240), now);
    check(dt.from_mm == 110, "nearest counts from the trigger place (110), not 150");
    (void)drive_to(&dt, 299);
    check(dt_step(&dt) == DT_STRAIGHT, "189 mm after the nearest: driving");
    w = drive_to(&dt, 300);
    check(dt_step(&dt) == DT_TURN_TOWARD && dt.straights == 1u, "190 mm after: TURN_TOWARD");
    check(dt.x == 289 && dt.y == 77, "pose: 300 mm at 15 deg = (289, 77)");
    check(dt.from_mm == 0 && dt.best_mm == 0xFFFFu, "TURN_TOWARD: nearest from 110 cleared");
    check(dt_move(w) == DT_MOVE_STOP, "TURN_TOWARD: look first");

    /* STRAIGHT: too near; no echo: 190 from the start. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    read(&dt, 79);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_TOO_NEAR, "79 mm: TOO_NEAR");
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    (void)drive_to(&dt, 189);
    check(dt_step(&dt) == DT_STRAIGHT, "no echo: 189 mm driving");
    (void)drive_to(&dt, 190);
    check(dt_step(&dt) == DT_TURN_TOWARD, "no echo: 190 mm");
    check(dt.best_mm == 0xFFFFu && dt.from_mm == 0, "TURN_TOWARD: nearest cleared");

    /* TURN_TOWARD: turn right (away from S) until seen. */
    (void)update(&dt);
    read(&dt, NR);
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "TURN_TOWARD: turn right");
    (void)turn_to(&dt, -120);
    check(!dt.looking, "-120 counts: turning");
    (void)turn_to(&dt, -121);
    check(dt.looking, "-121 counts: look");
    read(&dt, 410);
    (void)turn_to(&dt, -counts(10));
    read(&dt, 350);
    check(dt_step(&dt) == DT_TURN_TOWARD && dt.backing && !dt.looking,
          "seen at -10: back off");
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1, "back off: turn left (away)");
    check(!dt_wants_reading(&dt, now + 1000u), "back off: no readings");
    line = true;
    (void)turn_to(&dt, -counts(10) + 240);
    check(dt_step(&dt) == DT_TURN_TOWARD, "back off 240 of 240.5 counts: turning; line ignored");
    line = false;
    (void)turn_to(&dt, -counts(10) + 241);
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 15,
          "back off 10: heading 15 - 10 + 10 = 15");
    check(dt.straights == 1u && dt.best_mm == 0xFFFFu, "second straight starts clean");

    /* TURN_TOWARD: seen at the first look: no turn. */
    (void)drive_to(&dt, 190);
    (void)update(&dt);
    read(&dt, 300);
    check(dt.backing, "seen at once: back off");
    w = update(&dt);
    check(!dt.looking && dt_move(w) == DT_MOVE_TURN, "back off from 0: turning, no look");
    (void)turn_to(&dt, counts(10));
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 25, "seen at once: 15 + 10 = 25");

    /* TURN_TOWARD limit: 150. */
    (void)drive_to(&dt, 190);
    (void)update(&dt);
    for (int deg = 0; deg <= 145; deg += 5) {
        (void)turn_to(&dt, -counts(deg));
        read(&dt, NR);
    }
    check(dt_step(&dt) == DT_TURN_TOWARD && dt.target_deg == 150u, "never seen: look at 150");
    (void)turn_to(&dt, -counts(150));
    read(&dt, NR);
    check(dt_fail(&dt) == DT_FAIL_TURN_LIMIT, "toward: 155 > 150 fails");

    /* max_straights: the fifth straight ends with NO_LINE. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    for (int i = 0; i < 4; ++i) {
        (void)drive_to(&dt, 190);
        (void)update(&dt);
        read(&dt, 300);
        (void)turn_to(&dt, counts(10));
    }
    check(dt_step(&dt) == DT_STRAIGHT && dt.straights == 4u, "4 straights done: fifth");
    (void)drive_to(&dt, 190);
    check(dt_step(&dt) == DT_FAILED && dt_fail(&dt) == DT_FAIL_NO_LINE, "5 straights: NO_LINE");

    /* The line: armed only after it was lost. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    line = true;
    (void)update(&dt);
    read(&dt, 250);
    (void)turn_to(&dt, counts(5));
    read(&dt, NR);
    (void)drive_to(&dt, 50);
    check(dt_step(&dt) == DT_STRAIGHT && !dt.armed, "on the line from the start: not armed");
    line = false;
    (void)drive_to(&dt, 60);
    check(dt.armed && dt_step(&dt) == DT_STRAIGHT, "line lost: armed");
    line = true;
    w = drive_to(&dt, 100);
    check(dt_step(&dt) == DT_ON_LINE && dt.leg_mm == 100u, "line seen: ON_LINE 100 mm");
    check(dt.x == 99 && dt.y == 8, "pose: 100 mm at 5 deg");
    check(dt_move(w) == DT_MOVE_STRAIGHT, "ON_LINE: straight");

    /* Line in a turn: ignored. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    (void)drive_to(&dt, 190);
    line = true;
    (void)update(&dt);
    check(dt_step(&dt) == DT_TURN_TOWARD, "line in a turn: ignored");
    line = false;

    /* ON_LINE, ALIGN, DONE: heading -15 turns left 15. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    (void)drive_to(&dt, 190);
    (void)update(&dt);
    read(&dt, NR);
    (void)turn_to(&dt, -counts(30));
    read(&dt, 300);
    (void)turn_to(&dt, -counts(30) + counts(10));
    check(dt.heading == -5, "heading 15 - 30 + 10 = -5");
    (void)drive_to(&dt, 50);
    line = true;
    (void)update(&dt);
    check(dt_step(&dt) == DT_ON_LINE, "ON_LINE");
    (void)drive_to(&dt, 99);
    check(dt_step(&dt) == DT_ON_LINE, "ON_LINE 99 mm");
    (void)drive_to(&dt, 100);
    check(dt_step(&dt) == DT_ALIGN, "ON_LINE 100 mm: ALIGN");
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == +1, "ALIGN from -5: turn left");
    (void)turn_to(&dt, counts(5) - 1);
    check(dt_step(&dt) == DT_ALIGN, "ALIGN: 4.99 deg");
    w = turn_to(&dt, counts(5));
    check(dt_step(&dt) == DT_DONE && dt.heading == 0 && dt_move(w) == DT_MOVE_STOP,
          "ALIGN 5 deg: DONE");
    check(!dt_wants_reading(&dt, now + 1000u), "DONE: no readings");

    /* ALIGN from +15 turns right; from 0 is done at once. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    (void)drive_to(&dt, 10);
    line = true;
    (void)drive_to(&dt, 20);
    (void)drive_to(&dt, 120);
    w = update(&dt);
    check(dt_step(&dt) == DT_ALIGN && dt_dir(w) == -1, "ALIGN from +15: turn right");
    (void)turn_to(&dt, -counts(15));
    check(dt_step(&dt) == DT_DONE, "ALIGN -15 deg: DONE");
    line = false;
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)update(&dt);
    read(&dt, 250);
    (void)turn_to(&dt, counts(5));
    read(&dt, NR);
    dt.heading = 0;
    line = true;
    (void)drive_to(&dt, 10);
    (void)drive_to(&dt, 110);
    (void)update(&dt);
    check(dt_step(&dt) == DT_DONE, "ALIGN from 0: DONE at once");
    line = false;

    /* Detour right: the mirror. */
    dt = start(OBSTACLE_SIDE_RIGHT, 220u);
    w = update(&dt);
    check(dt_servo_deg(w) == 30, "right: servo 30 left (toward the obstacle)");
    read(&dt, NR);
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_TURN && dt_dir(w) == -1, "right: TURN_AWAY turns right");
    cr -= 60;
    cl += 61;
    (void)update(&dt);
    check(dt.looking, "right: 121 counts right = 5 deg toward S");
    read(&dt, 250);
    (void)turn_to(&dt, counts(10));
    read(&dt, NR);
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 10, "right: heading 10 toward S");
    (void)drive_to(&dt, 190);
    check(dt.y == 33, "right: y toward S (190 x sin 10)");
    (void)update(&dt);
    read(&dt, NR);
    w = update(&dt);
    check(dt_dir(w) == +1, "right: TURN_TOWARD turns left");

    /* BACK when the obstacle is inside the turn circle: 180 - (65 + 100). */
    dt = start(OBSTACLE_SIDE_LEFT, 100u);
    check(dt_step(&dt) == DT_BACK && dt.leg_mm == 15u, "100 mm: BACK 15 mm");
    w = update(&dt);
    check(dt_move(w) == DT_MOVE_STRAIGHT && dt_dir(w) == -1, "BACK: straight back");
    check(!dt_wants_reading(&dt, now + 1000u), "BACK: no readings");
    (void)drive_to(&dt, -14);
    check(dt_step(&dt) == DT_BACK, "BACK 14 mm: driving");
    (void)drive_to(&dt, -15);
    check(dt_step(&dt) == DT_TURN_AWAY && dt.heading == 0, "BACK 15 mm: TURN_AWAY");
    dt = start(OBSTACLE_SIDE_LEFT, 115u);
    check(dt_step(&dt) == DT_TURN_AWAY, "115 mm: front 180 = need: no BACK");

    /* Turn exactly at the target: 5 x 2400 = 12000 = 120 x 100. */
    dt_config_t exact = CFG;
    exact.counts_per_deg_x100 = 2400u;
    exact.back_off_deg = 0u; /* back off 0: STRAIGHT when seen */
    now = 1000u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, cl, cr, now);
    (void)update(&dt);
    read(&dt, NR);
    (void)turn_to(&dt, 119);
    check(!dt.looking, "exact: 119 counts: turning");
    (void)turn_to(&dt, 120);
    check(dt.looking, "exact: 120 counts = 5 deg: look");
    /* heading rounding: 1.5 degrees = 36 counts rounds to 2; -36 to -2. */
    read(&dt, 250);
    (void)turn_to(&dt, 240 + 36 - 240);
    dt.target_deg = 0u;
    (void)update(&dt);
    read(&dt, NR);
    check(dt.heading == 2, "exact: 36 counts = 1.5 deg rounds to 2");
    (void)drive_to(&dt, 190);
    (void)update(&dt);
    read(&dt, NR);
    (void)turn_to(&dt, -36);
    dt.target_deg = 0u;
    (void)update(&dt);
    read(&dt, 300);
    check(dt.heading == 0, "exact: -36 counts = -1.5 deg rounds to -2: 2 - 2 = 0");

    /* Back off exactly at the target: seen at -5 (-120 counts), back 10
     * x 2400 = 240 counts: ends at +120. */
    exact = CFG;
    exact.counts_per_deg_x100 = 2400u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, cl, cr, now);
    (void)update(&dt);
    read(&dt, 250);
    (void)turn_to(&dt, 120);
    read(&dt, NR);
    (void)drive_to(&dt, 190);
    (void)update(&dt);
    read(&dt, NR);
    (void)turn_to(&dt, -120);
    read(&dt, 300);
    (void)turn_to(&dt, 119);
    check(dt.backing, "exact back off: 119: turning");
    (void)turn_to(&dt, 120);
    check(dt_step(&dt) == DT_STRAIGHT && dt.heading == 10, "exact back off: 120: heading 10");

    /* Look 0 ms: the servo still needs 160 ms at the start. */
    exact = CFG;
    exact.look_ms = 0u;
    now = 1000u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, cl, cr, now);
    (void)update(&dt);
    check(dt.looking && !dt_wants_reading(&dt, 1159u) && dt_wants_reading(&dt, 1160u),
          "look 0: servo settle 160 ms");

    /* A reading triggered in a step with no readings is ignored. */
    dt = start(OBSTACLE_SIDE_LEFT, 100u);
    dt_triggered(&dt);
    dt_reading(&dt, result(FAULT), now);
    check(dt_step(&dt) == DT_BACK, "BACK: a reading is ignored");

    /* Zero config values do not divide by zero. */
    exact = CFG;
    exact.turn_step_deg = 0u;
    exact.counts_per_deg_x100 = 0u;
    dt_init(&dt, &exact, OBSTACLE_SIDE_LEFT, 220u, cl, cr, now);
    check(dt.cfg.turn_step_deg == 1u && dt.cfg.counts_per_deg_x100 == 1u, "zero config: 1");

    /* Time limit. */
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    now = 1000u + 29989u;
    (void)update(&dt);
    check(dt_step(&dt) == DT_TURN_AWAY, "29999 ms: running");
    (void)update(&dt);
    check(dt_fail(&dt) == DT_FAIL_TIME, "30009 ms: fails");

    /* Sensor fault; readings without a trigger; reading after a step change. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    dt_reading(&dt, result(300), now);
    check(dt.best_mm == 0xFFFFu, "a reading with no trigger is ignored");
    read(&dt, FAULT);
    check(dt_fail(&dt) == DT_FAIL_SENSOR, "fault streak fails");
    dt = start(OBSTACLE_SIDE_LEFT, 220u);
    (void)update(&dt);
    read(&dt, FAULT);
    check(dt_fail(&dt) == DT_FAIL_SENSOR, "fault in a look fails");
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    now = dt.settle_ms;
    dt_triggered(&dt);
    (void)drive_to(&dt, 190);
    check(dt_step(&dt) == DT_TURN_TOWARD && dt.looking && !dt.pending,
          "new step before the reading came in: looking, not pending");
    dt_reading(&dt, result(300), now);
    check(dt_step(&dt) == DT_TURN_TOWARD && dt.looking,
          "the straight's reading is not used as the look");

    /* Word layout. */
    dt = to_straight(OBSTACLE_SIDE_LEFT);
    w = update(&dt);
    check(w == (dt_word_t)(2u | (0u << 3)), "word: STRAIGHT forward, servo -30");
    dt = start(OBSTACLE_SIDE_RIGHT, 220u);
    (void)update(&dt);
    read(&dt, NR);
    w = update(&dt);
    check(w == (dt_word_t)(1u | 4u | (60u << 3)), "word: TURN right, servo +30");

    if (failures == 0) {
        printf("test_obstacle_detour: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_detour: %d failure(s)\n", failures);
    return 1;
}
