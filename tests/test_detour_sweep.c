/* Pure sweep/controller tests. Link with app_program/subsystem5/obstacle_detour.c only. */
#include <stdbool.h>
#include <stdio.h>
#include "../app_program/subsystem5/obstacle_detour.h"
#include "../app_program/vehicle/vehicle_robot_config.h"

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

/* Run LAST 2026-11-18: the result of a ping at the step start was lost (the
 * vehicle cancelled it). The detour waited for it, asked for no more pings,
 * missed the -90 return heading and failed at the turn limit. */
static void test_lost_result(void) {
    dt_t dt;
    uint32_t now = 200u;
    dt_init(&dt, &CFG, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    dt.step = DT_TURN_TOWARD;
    dt.heading = -20;
    (void)at(&dt, 0, now);
    CHECK(dt_wants_reading(&dt, now), "lost: reading wanted at the step start");
    dt_triggered(&dt); /* this result never comes */
    now += 10u;
    (void)at(&dt, 0, now); /* the wait starts at the next update */
    (void)at(&dt, 0, now + DT_PENDING_MAX_MS - 10u);
    CHECK(!dt_wants_reading(&dt, now + DT_PENDING_MAX_MS - 10u),
          "lost: wait for the result until the limit");
    now += DT_PENDING_MAX_MS;
    (void)at(&dt, 0, now);
    CHECK(dt_wants_reading(&dt, now), "lost: at the limit, stop waiting and ping again");
    for (int deg = 0; dt.step == DT_TURN_TOWARD && deg > -200; --deg) {
        now += 10u;
        (void)at(&dt, deg, now);
        if (dt_wants_reading(&dt, now)) {
            dt_triggered(&dt);
            dt_reading(&dt, NR, now);
        }
    }
    CHECK(dt.step == DT_STRAIGHT && dt.returning && dt.fail == DT_FAIL_NONE,
          "lost: the return turn ends at the return heading, not the limit");
    CHECK(dt.heading <= -90 && dt.heading > -120, "lost: heading near -90");
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
    uint16_t omm;                /* scan distance; 0: 220 */
    bool blind;                  /* sonar: no echo after the first STRAIGHT */
    int line_straights;          /* straights done when ON_LINE began */
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
    uint8_t away_min;            /* dt_set_away_min(); 0: old rule */
    uint8_t away_seen;           /* !0: TURN_AWAY sees only its first n pings
                                  * (e7b34e07: the face drops out); 0: off */
    uint8_t away_pings;          /* TURN_AWAY pings so far */
    bool away_now;               /* the ping now is in TURN_AWAY */
    uint8_t fail;                /* dt_fail at the end */
    double first_heading;        /* robot heading at the first STRAIGHT */
    double first_clear;          /* body (half width 80) to corner (bx0, by1) there */
    bool first_seen;             /* first STRAIGHT reached */
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
            if (s->away_seen != 0u && s->away_now && s->away_pings > s->away_seen) {
                return 0xFFFFu;
            }
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
    dt_init(&dt, cfg, OBSTACLE_SIDE_LEFT, s->omm ? s->omm : 220u, 0, 0,
            s->t);
    dt_set_away_min(&dt, s->away_min);
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
            s->line_straights = dt.straights;
        }
        hash_add(s, (uint32_t)dt.step | (uint32_t)w << 8 |
                        (uint32_t)(uint16_t)dt.heading << 24);
        hash_add(s, (uint32_t)dt.returning);
        if (dt.step == DT_STRAIGHT && dt.straights == 0u && !s->first_seen) {
            const double ex = s->bx0 - s->x, ey = s->by1 - s->y;
            s->first_seen = true;
            s->first_heading = s->th;
            s->first_clear = ex * sin_d(s->th) - ey * cos_d(s->th) - 80.0;
        }
        if (!s->echo_due && dt_wants_reading(&dt, s->t)) {
            dt_triggered(&dt);
            s->away_now = dt.step == DT_TURN_AWAY;
            if (s->away_now && s->away_pings < 255u) {
                ++s->away_pings;
            }
            s->echo = s->blind && dt.straights >= 1u
                          ? 0xFFFFu
                          : scene_echo(s, dt_servo_deg(w));
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
    s->fail = dt.fail;
    return dt_step(&dt);
}

static scene_t fc2_like(void) {
    /* FC_2 (2026-11-17): the line leaves the stop at about -7 deg; the box
     * is on the line: its left (detour side) wall at the left line edge, its
     * body right of the line (operator 2026-11-17). */
    return (scene_t){.lx = 119.0, .ly = 7.0, .la = -7.0,
                     .bx0 = 290.0, .bx1 = 435.0, .by0 = -110.0, .by1 = -10.0};
}

static scene_t d2_like(void) {
    /* FX_1 detour 2 (2026-11-17): STRAIGHT2 ends with the pair near the
     * line; the return turn crosses it at about -25..-34 degrees. */
    scene_t s = fc2_like();
    s.la = -3.0;
    s.ly = 20.0;
    return s;
}

static scene_t fc1_like(void) {
    /* FC_1: box 100 x 145 mm on the line. */
    return (scene_t){.lx = 119.0, .ly = 7.0, .la = -7.0,
                     .bx0 = 290.0, .bx1 = 435.0, .by0 = -50.0, .by1 = 50.0};
}

/* Return turn after the second STRAIGHT from heading h0 to h1 (whole
 * degrees, S frame), line seen from line_from (degrees, absolute) on.
 * Returns the step at the end. */
static dt_step_t sweep_line(dt_t *dt, int h0, int h1, int line_from,
                            uint32_t *now) {
    dt->step = DT_TURN_TOWARD;
    dt->straights = 2u;
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
    dt.straights = 2u;
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

    /* (8) After the first STRAIGHT only: the robot can be beside the
     * obstacle; the line does not count. */
    dt_init(&dt, &cfg, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    (void)at(&dt, 0, 50u);
    now = 100u;
    (void)sweep_line(&dt, -12, -12, 0, &now); /* sets straights 2 */
    dt.l0 = dt.l;
    dt.r0 = dt.r;
    dt.straights = 1u;
    for (int h = -13; h >= -80; --h) {
        now += 10u;
        (void)dt_update(&dt, true, dt.l0 - (h + 12) * 10,
                        dt.r0 + (h + 12) * 10, now);
    }
    CHECK(dt_step(&dt) == DT_TURN_TOWARD,
          "turn after STRAIGHT 1: line at -45..-80 ignored");

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

    /* (9) Sonar miss beside the obstacle: box 100 x 300 mm on the line,
     * 140 mm ahead; no echo after the first STRAIGHT. The turn after the
     * first STRAIGHT crosses the line beside the box. The line must not
     * count there: the trace is the old-rule trace. */
    for (uint16_t mm = 100u; mm <= 180u; mm = (uint16_t)(mm + 40u)) {
        scene_t a = fc2_like(), b;
        a.omm = mm;
        a.blind = true;
        a.bx0 = 65.0 + mm;
        a.bx1 = a.bx0 + 300.0;
        b = a;
        cfg.line_turn_deg = 0u;
        (void)scene_run(&a, &cfg);
        cfg.line_turn_deg = 45u;
        (void)scene_run(&b, &cfg);
        CHECK(b.hash == a.hash &&
                  !(b.on_line_from == DT_TURN_TOWARD && b.line_straights < 2),
              "sonar miss: no line in the turn after STRAIGHT 1");
    }
}

/* The robot value (vehicle_robot_config.h), not a test copy. */
static void test_robot_value(void) {
    const dt_config_t *rc = &DETOUR_CFG.dt;
    dt_t dt;
    uint32_t now = 100u;
    scene_t s;
    (void)LINE_CFG;
    (void)SEARCH_CFG;
    (void)SCAN_CFG;
    /* (10) Return turn after STRAIGHT 2 from -23; line from -27. The
     * helper turns 20 counts per degree; line_turn_deg is the robot's. */
    dt_config_t unit = *rc;
    unit.counts_per_deg_x100 = 2000u;
    dt_init(&dt, &unit, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    (void)dt_update(&dt, false, 0, 0, 50u);
    CHECK(sweep_line(&dt, -23, -90, -27, &now) == DT_ON_LINE &&
              dt.heading == -27,
          "robot value: line at -27 in the return turn counts");
    /* (11) D2-like scene: the line is crossed at a shallow heading. */
    s = d2_like();
    CHECK(scene_run(&s, rc) == DT_DONE && s.on_line_from == DT_TURN_TOWARD &&
              s.line_straights >= 2 && s.line_heading <= -20 &&
              s.line_heading >= -35,
          "robot value: D2-like rejoins from the return turn");
    {
        const double d = -(s.x - s.lx) * sin_d(s.la) +
                         (s.y - s.ly) * cos_d(s.la);
        CHECK(d > -30.0 && d < 30.0, "robot value: D2-like axle on the line");
    }
    /* (12) FC_2-like and FC_1-like keep their results. */
    s = fc2_like();
    CHECK(scene_run(&s, rc) == DT_DONE && s.on_line_from == DT_TURN_TOWARD &&
              s.line_heading <= -45,
          "robot value: FC_2-like still rejoins in the turn past -45");
    s = fc1_like();
    CHECK(scene_run(&s, rc) == DT_DONE && s.hash == 0x391dc490u,
          "robot value: FC_1-like trace identical to the old rule");
    /* (13) Post-STRAIGHT-1 guard with the robot value. */
    for (uint16_t mm = 100u; mm <= 180u; mm = (uint16_t)(mm + 40u)) {
        dt_config_t off = *rc;
        scene_t a = fc2_like(), b;
        a.omm = mm;
        a.blind = true;
        a.bx0 = 65.0 + mm;
        a.bx1 = a.bx0 + 300.0;
        b = a;
        off.line_turn_deg = 0u;
        (void)scene_run(&a, &off);
        (void)scene_run(&b, rc);
        CHECK(b.hash == a.hash, "robot value: sonar miss, old-rule trace");
    }
}


/* ---- Minimum turn before LOST (docs/s5-obstacle/OBSTACLE.md). ---- */

/* One robot sweep, replayed: wheel counts (L, R) at run-relative times from
 * robot/state, linearly interpolated, and the robot/sonar results (trigger
 * time, packed result; the result comes at the trigger time). After the
 * last sample the turn goes on at turn_rate counts per 10 ms (S frame),
 * every further reading is far_echo, until coast_ms; then the counts stop. */
typedef struct {
    uint32_t t;
    int32_t l, r;
} count_sample_t;
typedef struct {
    uint32_t t;
    ultrasonic_result_t echo;
} ping_sample_t;
typedef struct {
    uint32_t hash;           /* FNV-1a of (step, word, heading, sweep end) */
    int lost_trig_x100;      /* turn x100 of the trigger of the last reading before LOST */
    int first_end_heading;   /* heading at the end of TURN_AWAY */
    uint8_t first_end;       /* sweep_last_end at the end of TURN_AWAY */
    uint8_t step_after;      /* step after TURN_AWAY */
    bool trimming;           /* TURN_TOWARD after TURN_AWAY is the trim */
    bool lost_at_start;      /* LOST accepted with the turn below 10 degrees */
    bool turn_on;            /* the step after TURN_AWAY turns away (S2) */
    int straight_heading;    /* heading of the first STRAIGHT; -999: none */
    uint8_t fail;            /* dt_fail at the end */
} replay_t;

static int32_t lerp(int32_t a, int32_t b, uint32_t t, uint32_t t0, uint32_t t1) {
    return a + (int32_t)((int64_t)(b - a) * (int64_t)(t - t0) / (int64_t)(t1 - t0));
}

/* After the last count sample the counts follow the command word of the
 * last update (robot frame): a turn (sweep, trim, turn-on) turns
 * sweep_rate counts per 10 ms, a back-off 32 counts (131 mm/s), a straight
 * 16 counts per wheel (131 mm/s). Readings after the log are seen (250 mm)
 * while the TURN_AWAY trigger turn is below seen_to_deg, else far_echo. The
 * replay stops 200 ms into the first STRAIGHT, or at end_ms. */
static replay_t replay_to(obstacle_side_t side, uint16_t mm, unsigned int away_min,
                          const dt_config_t *rc,
                          const count_sample_t *cs, unsigned int nc,
                          const ping_sample_t *ps, unsigned int np,
                          int32_t sweep_rate, ultrasonic_result_t far_echo,
                          int seen_to_deg, uint32_t end_ms) {
    dt_t dt;
    replay_t out = {2166136261u, 0, 0, 0u, 0u, false, false, false, -999, 0u};
    int32_t l = cs[0].l, r = cs[0].r;
    dt_init(&dt, rc, side, mm, l, r, 0u);
    dt_set_away_min(&dt, away_min);
    unsigned int next = 0u;
    int last_trig = 0;
    bool in_away = true;
    dt_word_t w = 0u;
    uint32_t straight_at = 0u;
    for (uint32_t t = 10u; t <= end_ms && dt_step(&dt) < DT_DONE; t += 10u) {
        if (t <= cs[nc - 1u].t) {
            unsigned int i = 0u;
            while (i + 1u < nc && cs[i + 1u].t < t) ++i;
            if (i + 1u < nc) {
                l = lerp(cs[i].l, cs[i + 1u].l, t, cs[i].t, cs[i + 1u].t);
                r = lerp(cs[i].r, cs[i + 1u].r, t, cs[i].t, cs[i + 1u].t);
            }
        } else if (dt_move(w) == DT_MOVE_TURN) {
            /* As detour_run.c: every turn but a back-off is at sweep speed. */
            const bool sweep = dt.step == DT_TURN_AWAY ||
                               (dt.step == DT_TURN_TOWARD && !dt.backing);
            const int32_t d = sweep ? sweep_rate : 32;
            l -= dt_dir(w) * (d - d / 2);
            r += dt_dir(w) * (d / 2);
        } else if (dt_move(w) == DT_MOVE_STRAIGHT) {
            l += dt_dir(w) * 16;
            r += dt_dir(w) * 16;
        }
        w = dt_update(&dt, false, l, r, t);
        if (dt_wants_reading(&dt, t)) {
            const bool replayed = next < np && t >= ps[next].t;
            const bool beyond = next >= np && t > cs[nc - 1u].t;
            if (replayed || beyond) {
                dt_triggered(&dt);
                last_trig = dt.trig_turn_x100;
                ultrasonic_result_t e = far_echo;
                if (!replayed && dt.step == DT_TURN_AWAY &&
                    dt.trig_turn_x100 < seen_to_deg * (int)rc->counts_per_deg_x100) {
                    e = (ultrasonic_result_t)250u;
                }
                dt_reading(&dt, replayed ? ps[next].echo : e, t);
                if (replayed) ++next;
            }
        }
        if (dt.sweep_end == DT_SWEEP_LOST && out.lost_trig_x100 == 0) {
            out.lost_trig_x100 = last_trig;
        }
        if (in_away && dt_step(&dt) != DT_TURN_AWAY) {
            in_away = false;
            out.first_end_heading = dt.heading;
            out.first_end = dt.sweep_last_end;
            out.step_after = dt.step;
            out.trimming = dt.trimming && !dt.turn_on;
            out.turn_on = dt.trimming && dt.turn_on;
            out.lost_at_start = dt.sweep_last_end == DT_SWEEP_LOST &&
                                out.lost_trig_x100 < 10 * (int)rc->counts_per_deg_x100;
        }
        if (dt_step(&dt) == DT_STRAIGHT && out.straight_heading == -999) {
            out.straight_heading = dt.heading;
            straight_at = t;
        }
        const uint32_t v[2] = {(uint32_t)dt.step | (uint32_t)w << 8 |
                                   (uint32_t)(uint16_t)dt.heading << 24,
                               dt.sweep_last_end};
        for (unsigned int k = 0u; k < 2u; ++k)
            for (int b = 0; b < 4; ++b) {
                out.hash ^= (v[k] >> (8 * b)) & 0xFFu;
                out.hash *= 16777619u;
            }
        if (straight_at != 0u && t >= straight_at + 200u) {
            break;
        }
    }
    out.fail = dt.fail;
    return out;
}

static replay_t replay(obstacle_side_t side, uint16_t mm, unsigned int away_min,
                       const count_sample_t *cs, unsigned int nc,
                       const ping_sample_t *ps, unsigned int np,
                       int32_t turn_rate, ultrasonic_result_t far_echo,
                       uint32_t end_ms) {
    return replay_to(side, mm, away_min, &DETOUR_CFG.dt, cs, nc, ps, np, turn_rate,
                     far_echo, 0, end_ms);
}

/* Centre-line distance (mm) of a corner at (x, y) mm from the axle at the
 * detour start, to the path at heading deg: + on the obstacle side. */
static double corner_clear(double x, double y, int deg) {
    return x * sin_d(deg) - y * cos_d(deg);
}

#define ECHO(mm) ((ultrasonic_result_t)(mm))
#define NO_RET ((ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN << ULTRASONIC_STATUS_SHIFT))

/* Boot e7b34e07 (2026-11-19, image 15, FAILED TOO_NEAR): detour start at
 * uptime 22180 ms; robot/state counts and robot/sonar results relative
 * to it. Scan: closest 230, left free step 6 (30 deg), right blocked:
 * side LEFT. Seen 278/278 before the wheels moved, 290 at about 1 deg,
 * then NO_RETURN x2 at about 2-4 deg: today LOST, trim back to 0. */
static const count_sample_t E7_COUNTS[] = {
    {0u, 3267, 3358}, {26u, 3267, 3358}, {144u, 3267, 3358}, {265u, 3266, 3358},
    {386u, 3233, 3400}, {505u, 3216, 3423},
};
static const ping_sample_t E7_PINGS[] = {
    {161u, ECHO(278)}, {230u, ECHO(278)}, {301u, ECHO(290)}, {371u, NO_RET}, {441u, NO_RET},
};
/* Boot c2c31955 (same obstacle, DONE_LINE): start at uptime 16180 ms.
 * Scan: closest 233, right free step 4 (20 deg), left blocked: side RIGHT.
 * The obstacle is seen on every ping to about 45 degrees, then lost. */
static const count_sample_t C2_COUNTS[] = {
    {0u, 1410, 1218}, {6u, 1410, 1218}, {124u, 1410, 1218}, {245u, 1410, 1218},
    {365u, 1439, 1186}, {486u, 1461, 1179}, {605u, 1485, 1179}, {725u, 1520, 1135},
    {846u, 1577, 1074}, {965u, 1634, 1015}, {1325u, 1804, 825}, {1685u, 1976, 648},
    {2046u, 1984, 631},
};
static const ping_sample_t C2_PINGS[] = {
    {161u, ECHO(250)}, {230u, ECHO(263)}, {301u, ECHO(250)}, {371u, ECHO(250)},
    {441u, ECHO(258)}, {511u, ECHO(254)}, {581u, ECHO(245)}, {651u, ECHO(240)},
    {721u, ECHO(241)}, {791u, ECHO(237)}, {861u, ECHO(238)}, {931u, ECHO(239)},
    {1001u, ECHO(236)}, {1071u, ECHO(242)}, {1141u, ECHO(243)}, {1211u, ECHO(242)},
    {1281u, ECHO(248)}, {1351u, ECHO(250)}, {1421u, ECHO(249)}, {1491u, ECHO(259)},
    {1561u, ECHO(262)}, {1631u, NO_RET}, {1701u, NO_RET},
};
#define N_OF(a) (sizeof(a) / sizeof((a)[0]))

static void test_away_min(void) {
    const int cpd = (int)DETOUR_CFG.dt.counts_per_deg_x100;
    /* 40 deg/s: 0.4 deg per 10 ms = 9.6 counts (S frame). */
    const int32_t rate = 10;
    /* (1) Old rule (0): the replay gives the robot's result (fidelity). */
    replay_t old = replay(OBSTACLE_SIDE_LEFT, 230u, 0u, E7_COUNTS, N_OF(E7_COUNTS),
                          E7_PINGS, N_OF(E7_PINGS), rate, NO_RET, 1500u);
    CHECK(old.first_end == DT_SWEEP_LOST && old.lost_at_start &&
              old.first_end_heading >= 2 && old.first_end_heading <= 6 &&
              old.step_after == DT_TURN_TOWARD && old.trimming,
          "e7b34e07 old rule: LOST at about 5 deg, then the trim (robot result)");
    /* (2) Minimum 30 (left free step 6 x 5): NO_RETURN x2 at 2-4 deg do not
     * give LOST. The turn goes on; LOST is accepted only from 30 deg. */
    replay_t fix = replay(OBSTACLE_SIDE_LEFT, 230u, 30u, E7_COUNTS, N_OF(E7_COUNTS),
                          E7_PINGS, N_OF(E7_PINGS), rate, NO_RET, 6000u);
    CHECK(!fix.lost_at_start && fix.lost_trig_x100 >= 30 * cpd,
          "e7b34e07 min 30: no LOST below 30 deg");
    CHECK(fix.first_end == DT_SWEEP_LOST && fix.first_end_heading >= 30 &&
              fix.first_end_heading < (int)DETOUR_CFG.dt.away_max_deg,
          "e7b34e07 min 30: LOST past 30 deg, not the turn limit");
    /* (3) Boot c2c31955, minimum 20 (right free step 4 x 5): LOST at about
     * 45 deg, past the minimum: the same steps as the old rule. */
    replay_t c_old = replay(OBSTACLE_SIDE_RIGHT, 233u, 0u, C2_COUNTS, N_OF(C2_COUNTS),
                            C2_PINGS, N_OF(C2_PINGS), 0, NO_RET, 2600u);
    replay_t c_new = replay(OBSTACLE_SIDE_RIGHT, 233u, 20u, C2_COUNTS, N_OF(C2_COUNTS),
                            C2_PINGS, N_OF(C2_PINGS), 0, NO_RET, 2600u);
    CHECK(c_old.first_end == DT_SWEEP_LOST && c_old.first_end_heading >= 40 &&
              c_old.first_end_heading <= 52,
          "c2c31955 old rule: LOST at about 47 deg (robot result)");
    CHECK(c_new.hash == c_old.hash && c_new.first_end_heading == c_old.first_end_heading &&
              c_new.step_after == c_old.step_after,
          "c2c31955 min 20: steps identical to the old rule");
    /* (4) Seen only from the start pose (seen x2 at turn 0, then no echo):
     * old rule LOST at the start; minimum 30: LOST at 30 deg, no TURN_LIMIT. */
    static const ping_sample_t START_ONLY[] = {{161u, ECHO(250)}, {230u, ECHO(250)}};
    static const count_sample_t STILL[] = {{0u, 0, 0}, {250u, 0, 0}};
    replay_t so_old = replay(OBSTACLE_SIDE_LEFT, 230u, 0u, STILL, N_OF(STILL),
                             START_ONLY, N_OF(START_ONLY), rate, NO_RET, 6000u);
    replay_t so_new = replay(OBSTACLE_SIDE_LEFT, 230u, 30u, STILL, N_OF(STILL),
                             START_ONLY, N_OF(START_ONLY), rate, NO_RET, 6000u);
    CHECK(so_old.first_end == DT_SWEEP_LOST && so_old.lost_at_start,
          "start pose only, old rule: LOST at the start");
    CHECK(so_new.first_end == DT_SWEEP_LOST && so_new.lost_trig_x100 >= 30 * cpd &&
              so_new.first_end_heading >= 30 && so_new.first_end_heading <= 36,
          "start pose only, min 30: LOST at 30 deg, no TURN_LIMIT");
    /* (5) Never seen: TURN_LIMIT at away_max_deg, with and without the minimum. */
    replay_t ns_old = replay(OBSTACLE_SIDE_LEFT, 230u, 0u, STILL, N_OF(STILL),
                             START_ONLY, 0u, rate, NO_RET, 12000u);
    replay_t ns_new = replay(OBSTACLE_SIDE_LEFT, 230u, 30u, STILL, N_OF(STILL),
                             START_ONLY, 0u, rate, NO_RET, 12000u);
    CHECK(ns_old.first_end == DT_SWEEP_LIMIT && ns_new.hash == ns_old.hash,
          "never seen: TURN_LIMIT as today, identical");
    /* (6) Clamp and default. */
    dt_t dt;
    dt_init(&dt, &DETOUR_CFG.dt, OBSTACLE_SIDE_LEFT, 230u, 0, 0, 0u);
    CHECK(dt.away_min_deg == 0u, "dt_init: minimum off");
    dt_set_away_min(&dt, 30u);
    CHECK(dt.away_min_deg == 30u, "set 30");
    dt_set_away_min(&dt, 500u);
    CHECK(dt.away_min_deg == DETOUR_CFG.dt.away_max_deg, "clamped to away_max_deg");
    /* (7) Stepped looks (sweep_deg_s 0): an unseen look below the minimum
     * after seen_once turns one more step; at the minimum it ends. */
    dt_config_t stepped = CFG;
    stepped.sweep_deg_s = 0u;
    dt_init(&dt, &stepped, OBSTACLE_SIDE_LEFT, 250u, 0, 0, 0u);
    dt_set_away_min(&dt, 10u);
    (void)at(&dt, 0, 200u);
    reading(&dt, 250u, 400u); /* seen at 0 */
    CHECK(dt.seen_once && dt.target_deg == 5u, "stepped: seen at 0, look at 5");
    (void)at(&dt, 5, 600u);
    reading(&dt, NR, 800u);
    CHECK(dt.step == DT_TURN_AWAY && dt.target_deg == 10u,
          "stepped: unseen at 5 (below 10): one more step");
    (void)at(&dt, 10, 1000u);
    reading(&dt, NR, 1200u);
    CHECK(dt.step != DT_TURN_AWAY && dt.heading == 10,
          "stepped: unseen at 10: past the edge");
    /* (8) Robot scenes with a small minimum (free step 1, 5 deg): the step
     * traces are identical to the old rule. */
    const uint32_t old_fc1 = 0x391dc490u;
    scene_t s = fc1_like();
    s.away_min = 5u;
    CHECK(scene_run(&s, &ROBOT) == DT_DONE && s.hash == old_fc1,
          "min 5: FC_1-like trace identical");
    scene_t a = fc2_like(), b = fc2_like();
    b.away_min = 5u;
    (void)scene_run(&a, &DETOUR_CFG.dt);
    (void)scene_run(&b, &DETOUR_CFG.dt);
    CHECK(a.hash == b.hash, "min 5: FC_2-like robot trace identical");
}

/* ---- S2: corner on the scan free ray when the edge is below the minimum. ---- */

/* Smallest whole heading in 0..max (S frame) whose path passes (x, y) by
 * clear mm (centre line); max when none. Same rule as trim_target, doubles. */
static int clear_heading(double x, double y, double clear, int max) {
    for (int th = 0; th <= max; ++th) {
        if (-corner_clear(x, y, th) <= -clear) return th;
    }
    return max;
}

static void test_scan_corner(void) {
    const dt_config_t *rc = &DETOUR_CFG.dt;
    const int32_t rate = 10;
    /* Scan free-ray corner of e7b34e07: axle (65 + 230, 230 tan 30). */
    const double fx = 295.0, fy = 230.0 * sin_d(30) / cos_d(30);
    const int c45 = clear_heading(fx, fy, rc->first_clear_mm, rc->away_max_deg);
    CHECK(c45 == 45, "free-ray corner (295, 133): clear heading 45");
    /* (1) e7b34e07, minimum 30: LOST at 33, edge at 1 (below 30): turn on
     * to the clear heading, STRAIGHT at 45. */
    replay_t e = replay_to(OBSTACLE_SIDE_LEFT, 230u, 30u, rc, E7_COUNTS, N_OF(E7_COUNTS),
                           E7_PINGS, N_OF(E7_PINGS), rate, NO_RET, 0, 8000u);
    CHECK(e.first_end == DT_SWEEP_LOST && e.first_end_heading >= 30 &&
              e.first_end_heading <= 36 && e.turn_on && !e.trimming,
          "e7b34e07 S2: LOST at about 33, then turn on (not the trim to 0)");
    CHECK(e.straight_heading == c45, "e7b34e07 S2: first STRAIGHT at 45");
    /* Centre-line clearance at 45 for the free-ray corner and the two scan
     * corners (+25: 282 mm, +20: 248 mm along the beam from the sensor). */
    const double m_free = corner_clear(fx, fy, e.straight_heading);
    const double m_25 = corner_clear(65.0 + 282.0 * cos_d(25), 282.0 * sin_d(25), e.straight_heading);
    const double m_20 = corner_clear(65.0 + 248.0 * cos_d(20), 248.0 * sin_d(20), e.straight_heading);
    CHECK(m_free >= 110.0 && m_25 >= 110.0 && m_20 >= 110.0,
          "e7b34e07 S2: every scan corner clears by >= 110 mm (centre line)");
    printf("S2 e7b34e07: LOST heading %d, straight %d, clear free %.1f +25 %.1f +20 %.1f mm\n",
           e.first_end_heading, e.straight_heading, m_free, m_25, m_20);
    /* (2) Edge at/after the minimum (face seen to 45): today's trim from
     * the last seen echo (250 mm at about 44, ray 14). */
    replay_t at45 = replay_to(OBSTACLE_SIDE_LEFT, 230u, 30u, rc, E7_COUNTS, N_OF(E7_COUNTS),
                              E7_PINGS, N_OF(E7_PINGS), rate, NO_RET, 45, 8000u);
    CHECK(at45.first_end == DT_SWEEP_LOST && at45.trimming && !at45.turn_on &&
              at45.straight_heading < at45.first_end_heading,
          "edge past the minimum: today's trim (down)");
    /* (3) Seen only from the start pose: no TURN_LIMIT, turn on to 45. */
    static const ping_sample_t START_ONLY[] = {{161u, ECHO(250)}, {230u, ECHO(250)}};
    static const count_sample_t STILL[] = {{0u, 0, 0}, {250u, 0, 0}};
    replay_t so = replay_to(OBSTACLE_SIDE_LEFT, 230u, 30u, rc, STILL, N_OF(STILL),
                            START_ONLY, N_OF(START_ONLY), rate, NO_RET, 0, 8000u);
    CHECK(so.fail == DT_FAIL_NONE && so.turn_on && so.straight_heading == c45,
          "start pose only: no TURN_LIMIT, turn on, STRAIGHT at 45");
    /* (4) Cap: away_max_deg 40 limits the turn-on target to 40. */
    dt_config_t capped = *rc;
    capped.away_max_deg = 40u;
    replay_t cap = replay_to(OBSTACLE_SIDE_LEFT, 230u, 30u, &capped, E7_COUNTS, N_OF(E7_COUNTS),
                             E7_PINGS, N_OF(E7_PINGS), rate, NO_RET, 0, 8000u);
    CHECK(cap.fail == DT_FAIL_NONE && cap.turn_on && cap.straight_heading == 40,
          "cap: turn on stops at away_max_deg 40, then STRAIGHT");
    /* (5) Old rule (minimum 0) and c2c31955 (edge at 43, past 20): identical. */
    replay_t e0a = replay_to(OBSTACLE_SIDE_LEFT, 230u, 0u, rc, E7_COUNTS, N_OF(E7_COUNTS),
                             E7_PINGS, N_OF(E7_PINGS), rate, NO_RET, 0, 8000u);
    CHECK(e0a.trimming && !e0a.turn_on && e0a.straight_heading <= 1,
          "minimum 0: today's trim to about 0 (the robot result)");
    replay_t c0 = replay_to(OBSTACLE_SIDE_RIGHT, 233u, 0u, rc, C2_COUNTS, N_OF(C2_COUNTS),
                            C2_PINGS, N_OF(C2_PINGS), 0, NO_RET, 0, 4000u);
    replay_t c20 = replay_to(OBSTACLE_SIDE_RIGHT, 233u, 20u, rc, C2_COUNTS, N_OF(C2_COUNTS),
                             C2_PINGS, N_OF(C2_PINGS), 0, NO_RET, 0, 4000u);
    CHECK(c0.hash == c20.hash && c20.trimming && !c20.turn_on,
          "c2c31955: trace identical, today's trim");
    /* (5b) Return leg, e7b34e07-like scene: face 230 mm ahead of the
     * sensor, left corner at (295, 133) (scan free at +30), 150 mm deep,
     * line along x. TURN_AWAY sees its first 3 pings, then the face drops
     * out. Old rule: trim to about 8, FAILED TOO_NEAR. S2: STRAIGHT at
     * 45, body clears the corner, the return turn and the returning
     * STRAIGHT (-90) find the line: DONE. */
    {
        scene_t a = {.lx = 119.0, .ly = 0.0, .la = 0.0, .omm = 230u, .away_seen = 3u,
                     .bx0 = 295.0, .bx1 = 445.0, .by0 = -300.0, .by1 = 133.0};
        scene_t b = a;
        b.away_min = 30u;
        CHECK(scene_run(&a, rc) == DT_FAILED && a.fail == DT_FAIL_TOO_NEAR,
              "e7-like scene, old rule: FAILED TOO_NEAR (the robot result)");
        CHECK(scene_run(&b, rc) == DT_DONE && b.on_line_from == DT_STRAIGHT &&
                  b.first_heading > 44.0 && b.first_heading < 46.5 && b.first_clear >= 30.0 &&
                  b.line_heading <= -85 && b.line_heading >= -95,
              "e7-like scene, S2: STRAIGHT at 45, corner clear, line found at -90, DONE");
        printf("S2 scene: first STRAIGHT %.1f deg, body clearance %.1f mm, line at %d deg\n",
               b.first_heading, b.first_clear, b.line_heading);
    }
    /* (6) Trim down from the free-ray corner: minimum 10 (corner (295, 41),
     * clear heading 30), edge at 0, LOST at 40: trim down 10. */
    dt_config_t unit = *rc;
    unit.counts_per_deg_x100 = 2000u;
    dt_t dt;
    dt_init(&dt, &unit, OBSTACLE_SIDE_LEFT, 230u, 0, 0, 0u);
    dt_set_away_min(&dt, 10u);
    (void)at(&dt, 0, 200u);
    reading(&dt, 250u, 200u);
    reading(&dt, 250u, 260u);
    CHECK(dt.seen_once && dt.edge_deg == 0, "trim down: edge at 0");
    (void)at(&dt, 40, 400u);
    reading(&dt, NR, 400u);
    reading(&dt, NR, 460u);
    (void)at(&dt, 40, 470u);
    (void)at(&dt, 40, 500u);
    const int c10 = clear_heading(295.0, 230.0 * sin_d(10) / cos_d(10), unit.first_clear_mm, 90);
    CHECK(dt.step == DT_TURN_TOWARD && dt.trimming && !dt.backing && dt.heading == 40 &&
              dt.trim_deg == 40 - c10,
          "trim down: from 40 to the free-ray clear heading");
    printf("S2 trim down: clear heading %d, trim %d\n", c10, dt.trim_deg);
}

int main(void) {
    test_controller();
    test_convergence();
    test_seen_and_coast(OBSTACLE_SIDE_LEFT);
    test_seen_and_coast(OBSTACLE_SIDE_RIGHT);
    test_away_and_return();
    test_line_in_return_turn();
    test_lost_result();
    test_scenes();
    test_robot_value();
    test_away_min();
    test_scan_corner();
    if (failures) return 1;
    puts("test_detour_sweep: all tests passed");
    return 0;
}
