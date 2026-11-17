/* Pure sweep/controller tests. Link with app_program/subsystem5/obstacle_detour.c only. */
#include <stdbool.h>
#include <stdio.h>
#include "../app_program/subsystem5/obstacle_detour.h"

static int failures;
#define CHECK(ok, text) do { if (!(ok)) { printf("FAIL: %s\n", text); ++failures; } } while (0)

static const dt_config_t CFG = {
    .turn_step_deg = 5u, .away_max_deg = 90u, .toward_max_deg = 150u,
    .max_straights = 5u, .back_off_deg = 10u, .look_ms = 150u,
    .vehicle_mm = 350u, .sensor_ahead_mm = 65u, .line_ahead_mm = 123u,
    .seen_mm = 400u, .counts_per_deg_x100 = 2000u, .total_max_ms = 25000u,
    .return_heading_deg = 90u, .return_max_mm = 1000u,
    .sweep_deg_s = 20u, .sweep_ping_ms = 60u, .sweep_seen_count = 2u,
    .sweep_min_duty = 12u, .sweep_max_duty = 30u,
};

static int half(int angle) { return angle / 2; }
static dt_word_t at(dt_t *dt, int degrees, uint32_t now) {
    return dt_update(dt, false, dt->l0 - dt->side * degrees * 10,
                     dt->r0 + dt->side * degrees * 10, now);
}
static void reading(dt_t *dt, ultrasonic_result_t result, uint32_t now) {
    CHECK(dt_wants_reading(dt, now), "reading wanted");
    dt_triggered(dt);
    dt_reading(dt, result, now);
}
static const ultrasonic_result_t NR =
    (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN << ULTRASONIC_STATUS_SHIFT);

static void test_controller(void) {
    dt_sweep_control_t ctl;
    dt_sweep_control_init(&ctl, &CFG, 0, 0, 0u);
    dt_sweep_control_update(&ctl, &CFG, +1, -20, 20, 99u);
    CHECK(ctl.sample_ms == 0u, "controller waits for its count window");
    dt_sweep_control_update(&ctl, &CFG, +1, -200, 200, 1000u);
    CHECK(ctl.rate_deg_s == 20 && ctl.duty[0] == 12 && ctl.duty[1] == 12,
          "target rate gives equal duties");
    for (uint32_t t = 1100u; t < 10000u; t += 100u) {
        dt_sweep_control_update(&ctl, &CFG, +1, -200, 200, t);
    }
    CHECK(ctl.duty[0] == 30 && ctl.duty[1] == 30, "stalled feedback clamps at maximum");
    dt_sweep_control_update(&ctl, &CFG, +1, -10200, 10200, 10000u);
    CHECK(ctl.duty[0] == 12 && ctl.duty[1] == 12, "fast feedback clamps at minimum");
    dt_sweep_control_init(&ctl, &CFG, 0, 0, UINT32_MAX - 49u);
    dt_sweep_control_update(&ctl, &CFG, -1, -10, -40, 50u);
    dt_sweep_control_update(&ctl, &CFG, -1, -10, -40, 60u);
    CHECK(ctl.duty[0] > ctl.duty[1], "wrong-way wheel gets more duty; right turn and wrap");
    CHECK(ctl.duty[0] <= 30 && ctl.duty[1] >= 12, "independent duties stay clamped");
}

/* Deliberately simple asymmetric plant, not a fitted robot/simulator model.
 * Each percent above 12 gives wheel-equivalent rates 25/33 deg/s. A
 * 50 ms Euler response and integer encoders exercise fractional PI output. */
static void test_convergence(void) {
    dt_sweep_control_t ctl;
    double rate[2] = {0.0, 0.0}, counts[2] = {0.0, 0.0};
    double sum[2] = {0.0, 0.0};
    int samples = 0;
    dt_sweep_control_init(&ctl, &CFG, 0, 0, 0u);
    for (uint32_t t = 10u; t <= 6000u; t += 10u) {
        for (unsigned int i = 0u; i < 2u; ++i) {
            const double target = (ctl.duty[i] - 12) * (i == 0u ? 25.0 : 33.0);
            rate[i] += (target - rate[i]) * 0.2;
            counts[i] += rate[i] * 0.1; /* 10 wheel counts/degree, 10 ms */
            if (t > 4000u) sum[i] += rate[i];
        }
        dt_sweep_control_update(&ctl, &CFG, +1, -(int32_t)counts[0], (int32_t)counts[1], t);
        CHECK(ctl.duty[0] >= 12 && ctl.duty[0] <= 30 &&
              ctl.duty[1] >= 12 && ctl.duty[1] <= 30, "plant: clamps");
        if (t > 4000u) {
            ++samples;
            CHECK(ctl.rate_deg_s >= 12 && ctl.rate_deg_s <= 28,
                  "plant: settled 100 ms rate within target +/-8");
        }
    }
    CHECK(sum[0] / samples > 19.0 && sum[0] / samples < 21.0 &&
          sum[1] / samples > 19.0 && sum[1] / samples < 21.0,
          "plant: asymmetric wheels converge to equal/opposite 20 deg/s targets");
}

static void test_seen_and_coast(obstacle_side_t side) {
    dt_t dt;
    dt_config_t cfg = CFG;
    cfg.back_off_fn = half;
    dt_init(&dt, &cfg, side, 250u, 0, 0, 0u);
    dt.step = DT_TURN_TOWARD;
    CHECK(dt_move(at(&dt, -20, 200u)) == DT_MOVE_TURN, "toward sweeps without stops");
    reading(&dt, 200u, 200u);
    CHECK(dt.sweep_end == DT_SWEEP_NONE, "one seen reading does not end sweep");
    CHECK(!dt_wants_reading(&dt, 259u), "sweep ping spacing");
    (void)at(&dt, -22, 260u);
    reading(&dt, 200u, 260u);
    CHECK(dt.sweep_end == DT_SWEEP_SEEN && !dt.backing, "two seen readings stop before backoff");
    CHECK(!dt_wants_reading(&dt, 400u), "no readings while settling");
    CHECK(dt_move(at(&dt, -23, 270u)) == DT_MOVE_STOP, "coasting counts do not start backoff");
    (void)at(&dt, -23, 280u);
    CHECK(!dt.backing, "wait for encoder quiet");
    (void)at(&dt, -23, 290u);
    CHECK(dt.backing && dt.last_turned_deg == 23 && dt.last_back_deg == 11,
          "callback angle includes coast before choosing backoff");
    CHECK(dt.sweep_serial == 1u && dt.sweep_heading == -23 && dt.sweep_coast_x10 == 10,
          "one sweep diagnostic includes heading and coast");
    (void)at(&dt, -12, 300u);
    CHECK(dt.step == DT_STRAIGHT && dt.heading == -12, "blind backoff ends at count target");
}

static void test_away_and_return(void) {
    dt_t dt;
    dt_init(&dt, &CFG, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    (void)at(&dt, 10, 200u);
    reading(&dt, 200u, 200u);
    CHECK(!dt.seen_once, "away needs two seen readings to arm edge");
    reading(&dt, 200u, 260u);
    CHECK(dt.seen_once, "away saw obstacle twice");
    reading(&dt, NR, 320u);
    CHECK(dt.sweep_end == DT_SWEEP_NONE, "one unseen reading does not lose edge");
    reading(&dt, 200u, 380u);
    reading(&dt, NR, 440u);
    CHECK(dt.sweep_end == DT_SWEEP_NONE, "seen resets unseen streak");
    reading(&dt, NR, 500u);
    CHECK(dt.sweep_end == DT_SWEEP_LOST, "second consecutive unseen stops away");
    (void)at(&dt, 11, 510u);
    (void)at(&dt, 12, 650u); /* deadline reached even if still moving */
    CHECK(dt.step == DT_STRAIGHT && dt.heading == 12 && dt.sweep_coast_x10 == 20,
          "settle deadline includes away coast");

    dt_init(&dt, &CFG, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    dt.step = DT_TURN_TOWARD;
    dt.heading = 30;
    (void)at(&dt, -100, 200u);
    reading(&dt, NR, 200u);
    reading(&dt, NR, 260u);
    CHECK(dt.sweep_end == DT_SWEEP_NONE, "unseen but not at return heading");
    CHECK(dt_move(at(&dt, -120, 270u)) == DT_MOVE_STOP, "continuous return threshold, no new ping");
    (void)at(&dt, -121, 280u);
    (void)at(&dt, -121, 300u);
    CHECK(dt.returning && dt.heading == -91 && dt.step == DT_STRAIGHT,
          "return straight includes coast");

    dt_init(&dt, &CFG, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    dt.step = DT_TURN_TOWARD;
    CHECK(dt_move(at(&dt, -150, 200u)) == DT_MOVE_STOP, "continuous turn limit, no ping needed");
    (void)at(&dt, -151, 350u);
    CHECK(dt.fail == DT_FAIL_TURN_LIMIT && dt.heading == -151, "limit stops and accounts coast");
}

/* ---- Return-turn line rule (line_turn_deg). ---- */

/* Robot detour settings (vehicle_robot_config.h DETOUR_CFG.dt, 024d01a). */
static const dt_config_t ROBOT = {
    .return_heading_deg = 90u, .return_max_mm = 1000u, .sweep_deg_s = 40u,
    .sweep_ping_ms = 60u, .sweep_seen_count = 2u, .line_return_only = true,
    .turn_step_deg = 5u, .away_max_deg = 90u, .toward_max_deg = 180u,
    .max_straights = 5u, .back_off_deg = 25u, .look_ms = 150u,
    .vehicle_mm = 350u, .sweep_mm = 150u, .margin_mm = 30u,
    .sensor_ahead_mm = 65u, .line_ahead_mm = 123u, .seen_mm = 400u,
    .counts_per_deg_x100 = 2405u, .total_max_ms = 40000u,
    .first_extra_mm = 50u, .first_max_mm = 700u, .first_clear_mm = 110u,
};

/* Kinematic scene: a straight line (18 mm) through (lx, ly) at la deg, an
 * axis-aligned box, the robot rear axle at the origin, heading 0. Not a
 * fitted robot model: it only drives dt_t through a plausible sequence. */
typedef struct {
    double lx, ly, la;           /* line point and direction, mm, deg */
    double bx0, bx1, by0, by1;   /* box, mm */
    double x, y, th;             /* rear axle pose, mm, deg */
    double fl, fr;               /* wheel counts, fractional */
    uint32_t t, echo_at;
    bool echo_due;
    uint16_t echo;
    uint32_t hash;               /* FNV-1a of the step trace */
    int line_steps;              /* updates with line_seen in TURN_TOWARD */
    int line_heading;            /* heading when ON_LINE began */
    int on_line_from;            /* step before ON_LINE */
} scene_t;

/* sin and cos of degrees, no libm: reduce, then a Taylor series. */
static double sin_d(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    const double x = deg * 3.14159265358979 / 180.0;
    double term = x, sum = x;
    for (int n = 1; n < 12; ++n) {
        term *= -x * x / ((2.0 * n) * (2.0 * n + 1.0));
        sum += term;
    }
    return sum;
}
static double cos_d(double deg) { return sin_d(deg + 90.0); }

static bool scene_line(const scene_t *s) {
    const double px = s->x + 119.0 * cos_d(s->th) - 7.0 * sin_d(s->th);
    const double py = s->y + 119.0 * sin_d(s->th) + 7.0 * cos_d(s->th);
    const double d = -(px - s->lx) * sin_d(s->la) +
                     (py - s->ly) * cos_d(s->la);
    return d > -12.0 && d < 12.0;
}

/* Echo along the servo ray (one ray, no cone), mm. */
static uint16_t scene_echo(const scene_t *s, int servo_deg) {
    const double a = s->th + servo_deg;
    const double ox = s->x + 65.0 * cos_d(s->th);
    const double oy = s->y + 65.0 * sin_d(s->th);
    for (double r = 20.0; r < 1000.0; r += 2.0) {
        const double px = ox + r * cos_d(a), py = oy + r * sin_d(a);
        if (px >= s->bx0 && px <= s->bx1 && py >= s->by0 && py <= s->by1) {
            return (uint16_t)r;
        }
    }
    return 0xFFFFu;
}

static void hash_add(scene_t *s, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        s->hash ^= (v >> (8 * i)) & 0xFFu;
        s->hash *= 16777619u;
    }
}

static dt_step_t scene_run(scene_t *s, const dt_config_t *cfg) {
    dt_t dt;
    const double cpm = 12.414, cpd = 24.05, v_turn = 1626.0 / 100.0;
    const double v_sweep = cfg->sweep_deg_s * cpd / 2.0 / 100.0;
    s->hash = 2166136261u;
    s->t = 1000u;
    s->line_steps = 0;
    s->on_line_from = -1;
    dt_init(&dt, cfg, OBSTACLE_SIDE_LEFT, 220u, 0, 0, s->t);
    while (dt_step(&dt) < DT_DONE && s->t < 60000u) {
        s->t += 10u;
        if (s->echo_due && (int32_t)(s->t - s->echo_at) >= 0) {
            s->echo_due = false;
            dt_reading(&dt, s->echo == 0xFFFFu ? NR
                                               : (ultrasonic_result_t)s->echo,
                       s->t);
        }
        const bool line_now = scene_line(s);
        const uint8_t before = dt.step;
        const dt_word_t w = dt_update(&dt, line_now, (int32_t)s->fl,
                                      (int32_t)s->fr, s->t);
        if (before == DT_TURN_TOWARD && line_now) {
            ++s->line_steps;
        }
        if (dt.step == DT_ON_LINE && before != DT_ON_LINE) {
            s->on_line_from = before;
            s->line_heading = (int)s->th;
        }
        hash_add(s, (uint32_t)dt.step | (uint32_t)w << 8 |
                        (uint32_t)(uint16_t)dt.heading << 24);
        hash_add(s, (uint32_t)dt.returning);
        if (!s->echo_due && dt_wants_reading(&dt, s->t)) {
            dt_triggered(&dt);
            s->echo = scene_echo(s, dt_servo_deg(w));
            s->echo_at = s->t + 30u;
            s->echo_due = true;
        }
        double dl = 0.0, dr = 0.0;
        const bool sweep = cfg->sweep_deg_s != 0u && !dt.backing &&
                           (dt.step == DT_TURN_AWAY ||
                            dt.step == DT_TURN_TOWARD);
        if (dt_move(w) == DT_MOVE_TURN) {
            const double v = sweep ? v_sweep : v_turn;
            dl = -dt_dir(w) * v;
            dr = dt_dir(w) * v;
        } else if (dt_move(w) == DT_MOVE_STRAIGHT) {
            dl = dr = dt_dir(w) * v_turn;
        }
        s->fl += dl;
        s->fr += dr;
        s->th += (dr - dl) / cpd;
        s->x += (dl + dr) / 2.0 / cpm * cos_d(s->th);
        s->y += (dl + dr) / 2.0 / cpm * sin_d(s->th);
    }
    return dt_step(&dt);
}

static scene_t fc2_like(void) {
    /* FC_2 (2026-11-17): the line leaves the stop at about -7 deg; the box
     * is on the line: its left (detour side) wall at the left line edge, its
     * body right of the line (operator 2026-11-17). */
    return (scene_t){.lx = 119.0, .ly = 7.0, .la = -7.0,
                     .bx0 = 290.0, .bx1 = 435.0, .by0 = -110.0, .by1 = -10.0};
}

static scene_t fc1_like(void) {
    /* FC_1: box 100 x 145 mm on the line. */
    return (scene_t){.lx = 119.0, .ly = 7.0, .la = -7.0,
                     .bx0 = 290.0, .bx1 = 435.0, .by0 = -50.0, .by1 = 50.0};
}

/* Return turn from heading h0 to h1 (whole degrees, S frame), line seen
 * from line_from (degrees, absolute) on. Returns the step at the end. */
static dt_step_t sweep_line(dt_t *dt, int h0, int h1, int line_from,
                            uint32_t *now) {
    dt->step = DT_TURN_TOWARD;
    dt->heading = (int16_t)h0;
    for (int h = h0; h >= h1 && dt->step == DT_TURN_TOWARD; --h) {
        const bool seen = h <= line_from;
        *now += 10u;
        (void)dt_update(dt, seen, dt->l0 - dt->side * (h - h0) * 10,
                        dt->r0 + dt->side * (h - h0) * 10, *now);
    }
    return dt_step(dt);
}

static void test_line_in_return_turn(void) {
    dt_t dt;
    dt_config_t cfg = CFG;
    uint32_t now = 100u;
    cfg.line_turn_deg = 45u;
    /* (1) Line at -60 in the return turn: ON_LINE, then ALIGN, DONE. */
    dt_init(&dt, &cfg, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    (void)at(&dt, 0, 50u); /* line not seen: armed */
    CHECK(sweep_line(&dt, -12, -90, -60, &now) == DT_ON_LINE &&
          dt.heading == -60 && dt.leg_mm == cfg.line_ahead_mm,
          "line at -60 in the return turn: ON_LINE, heading -60");
    CHECK(dt_move(dt_update(&dt, true, dt.l, dt.r, now + 10u)) ==
              DT_MOVE_STRAIGHT,
          "ON_LINE drives straight");
    {
        const int32_t c = (int32_t)dt_mm_to_counts(cfg.line_ahead_mm);
        (void)dt_update(&dt, false, dt.l0 + c, dt.r0 + c, now + 20u);
    }
    CHECK(dt_step(&dt) == DT_ALIGN, "line_ahead_mm: ALIGN");
    (void)dt_update(&dt, false, dt.l0 - 600, dt.r0 + 600, now + 30u);
    CHECK(dt_step(&dt) == DT_DONE && dt.heading == 0, "ALIGN +60: DONE");

    /* (3) Line at a shallow heading (above -45) is ignored. */
    dt_init(&dt, &cfg, OBSTACLE_SIDE_RIGHT, 250u, 0, 0, 0u);
    (void)at(&dt, 0, 50u);
    now = 100u;
    dt.step = DT_TURN_TOWARD;
    for (int h = -1; h >= -44; --h) {
        now += 10u;
        (void)dt_update(&dt, true, dt.l0 - dt.side * h * 10,
                        dt.r0 + dt.side * h * 10, now);
    }
    CHECK(dt_step(&dt) == DT_TURN_TOWARD, "line at 0..-44: ignored");
    now += 10u;
    (void)dt_update(&dt, true, dt.l0 + dt.side * 450,
                    dt.r0 - dt.side * 450, now);
    CHECK(dt_step(&dt) == DT_ON_LINE && dt.heading == -45,
          "line at -45 (right detour): ON_LINE");

    /* (4) TURN_AWAY, trim and back-off ignore the line, at any heading. */
    dt_init(&dt, &cfg, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    (void)at(&dt, 0, 50u);
    dt.heading = -80;
    (void)dt_update(&dt, true, dt.l0 - 100, dt.r0 + 100, 60u);
    CHECK(dt_step(&dt) == DT_TURN_AWAY, "TURN_AWAY: line ignored");
    dt.step = DT_TURN_TOWARD;
    dt.trimming = true;
    dt.trim_deg = 50;
    (void)dt_update(&dt, true, dt.l0 + 100, dt.r0 - 100, 70u);
    CHECK(dt_step(&dt) == DT_TURN_TOWARD && dt.trimming, "trim: line ignored");
    dt.trimming = false;
    dt.backing = true;
    dt.back_x100 = 1000000;
    (void)dt_update(&dt, true, dt.l0 + 200, dt.r0 - 200, 80u);
    CHECK(dt_step(&dt) == DT_TURN_TOWARD && dt.backing,
          "back-off: line ignored");
    dt.backing = false;
    dt.sweep_end = DT_SWEEP_SEEN;
    dt.sweep_quiet_ms = dt.sweep_stop_ms = 90u;
    (void)dt_update(&dt, true, dt.l0 + 300, dt.r0 - 300, 90u);
    CHECK(dt_step(&dt) == DT_TURN_TOWARD, "box seen, stopping: line ignored");

    /* Not armed: the line was never lost in the detour. */
    dt_init(&dt, &cfg, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    now = 100u;
    dt.step = DT_TURN_TOWARD;
    for (int h = -1; h >= -60; --h) {
        now += 10u;
        (void)dt_update(&dt, true, dt.l0 - h * 10, dt.r0 + h * 10, now);
    }
    CHECK(dt_step(&dt) == DT_TURN_TOWARD, "not armed: line ignored");

    /* (5) line_turn_deg 0: the old rule, the turn goes on. */
    cfg.line_turn_deg = 0u;
    dt_init(&dt, &cfg, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    (void)at(&dt, 0, 50u);
    now = 100u;
    CHECK(sweep_line(&dt, -12, -88, -60, &now) == DT_TURN_TOWARD,
          "line_turn_deg 0: line in the turn ignored");
}

static void test_scenes(void) {
    dt_config_t cfg = ROBOT;
    scene_t s;
    /* Old-rule hashes: obstacle_detour.c at 024d01a, same scene code. */
    const uint32_t old_fc2 = 0x21e3016eu, old_fc1 = 0x391dc490u;

    /* (5) line_turn_deg 0: FC_2-like run is byte-identical to the old one. */
    s = fc2_like();
    CHECK(scene_run(&s, &cfg) == DT_FAILED && s.hash == old_fc2 &&
              s.line_steps > 0,
          "off: FC_2-like crosses the line in the turn and fails, as before");

    cfg.line_turn_deg = 45u;
    /* (2) No line: the step trace is byte-identical to the old one. */
    s = fc2_like();
    s.ly = -5000.0;
    CHECK(scene_run(&s, &cfg) == DT_FAILED && s.hash == old_fc2,
          "no line: trace identical to the old rule");
    /* (6) FC_2-like: the return turn crosses the line; it rejoins. */
    s = fc2_like();
    CHECK(scene_run(&s, &cfg) == DT_DONE && s.on_line_from == DT_TURN_TOWARD &&
          s.line_heading <= -45 && s.line_heading >= -90,
          "FC_2-like: line in the return turn, ON_LINE, DONE");
    {
        const double d = -(s.x - s.lx) * sin_d(s.la) +
                         (s.y - s.ly) * cos_d(s.la);
        CHECK(d > -30.0 && d < 30.0 && s.th > -10.0 && s.th < 10.0,
              "FC_2-like: axle on the line, heading near 0");
    }
    /* (7) FC_1-like: still DONE, trace identical (no line in the turn).
     * FC_1 kr rows: return turn 36.6-37.7 s all white; the line came in
     * the returning STRAIGHT (38.28-38.40 s): ON_LINE from STRAIGHT. */
    s = fc1_like();
    CHECK(scene_run(&s, &cfg) == DT_DONE && s.hash == old_fc1 &&
              s.line_steps == 0 && s.on_line_from == DT_STRAIGHT,
          "FC_1-like: ON_LINE from the return STRAIGHT, DONE, identical");
}

int main(void) {
    test_controller();
    test_convergence();
    test_seen_and_coast(OBSTACLE_SIDE_LEFT);
    test_seen_and_coast(OBSTACLE_SIDE_RIGHT);
    test_away_and_return();
    test_line_in_return_turn();
    test_scenes();
    if (failures) return 1;
    puts("test_detour_sweep: all tests passed");
    return 0;
}
