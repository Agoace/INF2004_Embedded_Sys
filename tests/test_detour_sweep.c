/* Pure sweep/controller tests. Link with src/obstacle_detour.c only. */
#include <stdio.h>
#include "obstacle_detour.h"

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

int main(void) {
    test_controller();
    test_convergence();
    test_seen_and_coast(OBSTACLE_SIDE_LEFT);
    test_seen_and_coast(OBSTACLE_SIDE_RIGHT);
    test_away_and_return();
    if (failures) return 1;
    puts("test_detour_sweep: all tests passed");
    return 0;
}
