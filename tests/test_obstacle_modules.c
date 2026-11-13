/*
 * Host unit test for the SCAN and DETOUR modules (app_program/subsystem5/servo_scan.c,
 * app_program/subsystem5/detour_run.c), alone and in the shell with LINE (app_program/vehicle/vehicle_sm.c).
 * It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
 *      tests/test_obstacle_modules.c app_program/subsystem5/servo_scan.c app_program/subsystem5/detour_run.c \
 *      app_program/subsystem5/obstacle_detour.c app_program/subsystem5/obstacle_profile.c app_program/subsystem3/line_trace.c \
 *      app_program/subsystem3/line_search.c app_program/subsystem3/line_steer.c app_program/subsystem5/obstacle.c app_program/vehicle/vehicle_sm.c \
 *      app_program/subsystem3/line_position.c -o build/test_obstacle_modules
 *   ./build/test_obstacle_modules
 */
#include <stdio.h>

#include "../app_program/subsystem2/motor_ctl.h"
#include "../app_program/subsystem5/detour_run.h"
#include "../app_program/subsystem3/line_search.h"
#include "../app_program/subsystem3/line_trace.h"
#include "../app_program/subsystem5/servo_scan.h"

static int failures;

#define CHECK(cond, ...)                                                                  \
    do {                                                                                  \
        if (!(cond)) {                                                                    \
            printf("FAIL line %d: ", __LINE__);                                           \
            printf(__VA_ARGS__);                                                          \
            printf("\n");                                                                 \
            ++failures;                                                                   \
        }                                                                                 \
    } while (0)

static ultrasonic_result_t echo_mm(uint16_t mm) {
    return (ultrasonic_result_t)(mm << ULTRASONIC_DISTANCE_SHIFT);
}
static ultrasonic_result_t echo_none(void) {
    return (ultrasonic_result_t)((unsigned)ULTRASONIC_STATUS_NO_RETURN << ULTRASONIC_STATUS_SHIFT |
                                 (ULTRASONIC_DISTANCE_MAX_MM << ULTRASONIC_DISTANCE_SHIFT));
}
static ultrasonic_result_t echo_fault(uint8_t streak) {
    return (ultrasonic_result_t)(((unsigned)ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT) |
                                 ((unsigned)streak << ULTRASONIC_STREAK_SHIFT));
}

/* detour_test v1.8 settings (src/detour_test.c). */
static int back_off_for(int turned_deg) {
    const int half = turned_deg / 2;
    return half > 30 ? 30 : half;
}
static const dr_config_t DR = {
    .dt = {.back_off_fn = back_off_for, .return_heading_deg = 90u, .return_max_mm = 1000u,
           .sweep_deg_s = 20u, .sweep_ping_ms = 60u, .sweep_seen_count = 2u,
           .line_return_only = true, .turn_step_deg = 5u, .away_max_deg = 90u,
           .toward_max_deg = 150u, .max_straights = 5u, .back_off_deg = 25u, .look_ms = 150u,
           .vehicle_mm = 350u, .sweep_mm = 150u, .margin_mm = 30u, .sensor_ahead_mm = 65u,
           .line_ahead_mm = 123u, .seen_mm = 400u, .counts_per_deg_x100 = 2405u,
           .total_max_ms = 40000u},
    .straight_mm_s = 131,
    .turn_mm_s = 131,
    .pause_ms = 250u,
};
static const ss_config_t SS = {300u};

/* Sensor geometry for full reattempt tests: one free side is visible after a reverse. */
static ultrasonic_result_t scan_view(unsigned int index, unsigned int attempt,
                                     unsigned int free_after) {
    return attempt >= free_after && index >= 9u ? echo_none() : echo_mm(200u);
}

/* Wheel counts follow commanded straight speed: 10 ms x 12.414 counts/mm. */
static void reverse_wheels(int32_t counts[2], int32_t milli[2], const vsm_drive_t *d) {
    const int16_t v[2] = {d->left_mm_s, d->right_mm_s};
    for (unsigned int w = 0u; w < 2u; ++w) {
        milli[w] += (int32_t)v[w] * 124;
        counts[w] += milli[w] / 1000;
        milli[w] %= 1000;
    }
}

/* One step of a module. The drive starts with junk wheel speeds: the
 * modules set every field themselves (the shell gives 0, servo last). */
static uint16_t ping;
static vsm_outcome_t step(void *m, vsm_outcome_t (*fn)(void *, const vsm_inputs_t *, uint32_t, vsm_drive_t *),
                          const vsm_inputs_t *in, uint32_t now, vsm_drive_t *d) {
    *d = (vsm_drive_t){77, 77, 9, VSM_ACT_SPEED};
    ping = VSM_PING_DEFAULT_MS;
    return fn(m, in, now, d);
}

/* ---------------------------------------------------------------- SCAN */

static void test_payload(void) {
    const uint16_t p = ss_payload(OBSTACLE_SIDE_LEFT, 511u);
    CHECK(p == (511u | 2u << 9) && ss_payload_side(p) == OBSTACLE_SIDE_LEFT &&
              ss_payload_distance(p) == 511u && p <= VSM_OUT_PAYLOAD_MAX,
          "payload 0x%x", p);
    const uint16_t q = ss_payload(OBSTACLE_SIDE_RIGHT, 0x4FFu); /* distance too wide: masked */
    CHECK(ss_payload_side(q) == OBSTACLE_SIDE_RIGHT && ss_payload_distance(q) == 0xFFu, "masked 0x%x", q);
}

/* The robot's view: a box at 200 mm from index lo to hi, free elsewhere. */
static ultrasonic_result_t view(unsigned int index, unsigned int lo, unsigned int hi) {
    return index >= lo && index <= hi ? echo_mm(200u) : echo_none();
}

/* Run a scan with a box from lo to hi. Returns the outcome; *t_end the
 * time of the outcome. */
static vsm_outcome_t run_scan(ss_t *ss, unsigned int lo, unsigned int hi, uint32_t *t_end, int *bad) {
    vsm_inputs_t in = {0, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    vsm_drive_t d;
    ss_start(ss, 0u, 0u);
    bool in_flight = false;
    unsigned int at = 0u;
    *bad = 0;
    for (uint32_t t = 0u; t < 10000u; t += 10u) {
        /* The glue: a ping asked in the last step goes out now; its echo
         * comes in the same step. */
        in.ping_sent = in_flight;
        in.echo_new = in_flight;
        in.echo = view(at, lo, hi);
        const vsm_outcome_t o = step(ss, ss_step, &in, t, &d);
        if (o != 0u || ss->phase == SS_REVERSE) {
            *t_end = t;
            return o;
        }
        if (d.left_mm_s != 0 || d.right_mm_s != 0) ++*bad;
        /* The servo angle is where the next ping looks. */
        at = (unsigned int)((d.servo_deg + 30) / 5);
        in_flight = ping == VSM_PING_ASAP;
        if (ping != 0u && ping != VSM_PING_ASAP) ++*bad;
    }
    *t_end = 0u;
    return 0u;
}

static void test_scan(void) {
    ss_t ss;
    ss_init(&ss, &SS);
    vsm_inputs_t in = {0, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    vsm_drive_t d;
    ss_start(&ss, 123u, 0u);

    /* WAIT: stopped, servo centre, no pings; an echo with no ping is ignored. */
    in.echo_new = true;
    in.echo = echo_mm(100u);
    vsm_outcome_t o = step(&ss, ss_step, &in, 290u, &d);
    CHECK(o == 0u && d.left_mm_s == 0 && d.right_mm_s == 0 && d.servo_deg == 0 && ping == 0u &&
              ss.phase == SS_WAIT && ss.profile.point[0] == OBSTACLE_SCAN_POINT_EMPTY,
          "wait: out %u servo %d ping %u phase %u", o, d.servo_deg, ping, ss.phase);
    in.echo_new = false;
    /* After stop_ms: point 0 (30 right), settle 150 ms, no pings yet. */
    o = step(&ss, ss_step, &in, 300u, &d);
    CHECK(ss.phase == SS_SETTLE && d.servo_deg == -30 && ping == 0u, "point 0: phase %u servo %d ping %u",
          ss.phase, d.servo_deg, ping);
    o = step(&ss, ss_step, &in, 449u, &d);
    CHECK(ss.phase == SS_SETTLE && ping == 0u, "settle 149 ms: phase %u", ss.phase);
    o = step(&ss, ss_step, &in, 450u, &d);
    CHECK(ss.phase == SS_MEASURE && ping == VSM_PING_ASAP && d.servo_deg == -30,
          "measure: phase %u ping %u", ss.phase, ping);
    /* An echo without a ping in MEASURE: not counted. */
    in.echo_new = true;
    in.echo = echo_mm(150u);
    o = step(&ss, ss_step, &in, 460u, &d);
    CHECK(ss.readings == 0u && ss.profile.point[0] == OBSTACLE_SCAN_POINT_EMPTY, "stray echo counted");
    /* Ping, echo later. */
    in.echo_new = false;
    in.ping_sent = true;
    o = step(&ss, ss_step, &in, 470u, &d);
    in.ping_sent = false;
    CHECK(ss.pending && ss.readings == 0u, "pending %d", ss.pending);
    in.echo_new = true;
    in.echo = echo_mm(300u);
    o = step(&ss, ss_step, &in, 480u, &d);
    CHECK(!ss.pending && ss.readings == 1u && ultrasonic_result_distance(ss.profile.point[0]) == 300u,
          "reading 1: %u readings", ss.readings);
    /* Second reading, nearer: kept; point done, next point settles 60 ms. */
    in.ping_sent = true;
    in.echo = echo_mm(250u);
    o = step(&ss, ss_step, &in, 540u, &d);
    CHECK(ss.index == 1u && ss.readings == 0u && ss.phase == SS_SETTLE && d.servo_deg == -25 &&
              ping == 0u && ultrasonic_result_distance(ss.profile.point[0]) == 250u,
          "point 1: index %u phase %u servo %d point0 %u", ss.index, ss.phase, d.servo_deg,
          ultrasonic_result_distance(ss.profile.point[0]));
    in.ping_sent = false;
    in.echo_new = false;
    o = step(&ss, ss_step, &in, 599u, &d);
    CHECK(ss.phase == SS_SETTLE, "settle step 59 ms");
    o = step(&ss, ss_step, &in, 600u, &d);
    CHECK(ss.phase == SS_MEASURE, "settle step 60 ms");
    /* A ping in SETTLE (not asked) is not counted. */
    ss.phase = SS_SETTLE;
    in.ping_sent = true;
    in.echo_new = true;
    o = step(&ss, ss_step, &in, 610u, &d);
    CHECK(!ss.pending && ss.readings == 0u, "settle ping counted");
    /* Fault streak: FAILED. */
    ss.phase = SS_MEASURE;
    in.echo = echo_fault(ULTRASONIC_FAULT_STREAK);
    o = step(&ss, ss_step, &in, 620u, &d);
    CHECK(o == vsm_outcome(VSM_OUT_FAILED, SS_FAIL_ECHO), "fault: out 0x%x", o);
    /* One fault reading below the streak: counted as no information. */
    ss_start(&ss, 0u, 0u);
    ss.phase = SS_MEASURE;
    in.echo = echo_fault(1u);
    o = step(&ss, ss_step, &in, 10u, &d);
    CHECK(o == 0u && ss.readings == 1u && ss.profile.point[0] == OBSTACLE_SCAN_POINT_EMPTY,
          "single fault: out 0x%x readings %u", o, ss.readings);

    /* Last point: RETURN, servo centre, the outcome after 150 ms. */
    ss_start(&ss, 0u, 0u);
    ss.index = OBSTACLE_SCAN_POINTS - 1u;
    ss.readings = OBSTACLE_SCAN_READINGS - 1u;
    ss.phase = SS_MEASURE;
    in.ping_sent = true;
    in.echo = echo_mm(300u);
    o = step(&ss, ss_step, &in, 1000u, &d);
    CHECK(o == 0u && ss.phase == SS_RETURN && d.servo_deg == 0 && ping == 0u,
          "return: out 0x%x phase %u servo %d", o, ss.phase, d.servo_deg);
    in.ping_sent = false;
    in.echo_new = false;
    o = step(&ss, ss_step, &in, 1149u, &d);
    CHECK(o == 0u && d.servo_deg == 0, "return 149 ms: out 0x%x", o);
    o = step(&ss, ss_step, &in, 1150u, &d);
    CHECK(o == 0u && ss.phase == SS_REVERSE && ss.notice == SS_NOTICE_REVERSE_BEGIN,
          "blocked synthetic profile reverses: out 0x%x phase %u", o, ss.phase);
    /* A new start clears the readings of a stopped scan. */
    ss.readings = 1u;
    ss_start(&ss, 0u, 0u);
    CHECK(ss.readings == 0u && ss.index == 0u && ss.phase == SS_WAIT, "restart: readings %u", ss.readings);

    /* Whole scans. Box 3..8 (15 right to 10 left): left is free nearer. */
    uint32_t t_end;
    int bad;
    o = run_scan(&ss, 3u, 8u, &t_end, &bad);
    CHECK(o == vsm_outcome(VSM_OUT_SIDE, ss_payload(OBSTACLE_SIDE_LEFT, 200u)) && bad == 0,
          "box 3..8: out 0x%x bad %d", o, bad);
    CHECK(obstacle_profile_left(ss.word) == 3u && obstacle_profile_right(ss.word) == 4u,
          "free steps L %u R %u", obstacle_profile_left(ss.word), obstacle_profile_right(ss.word));
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        CHECK(ss.profile.point[i] == view(i, 3u, 8u), "point %u 0x%x", i, ss.profile.point[i]);
    }
    /* Time: wait 300, first settle 150, 12 x 60 settle, return 150, and
     * 2 readings per point at one step per ping (asked, sent, read). */
    CHECK(t_end > 300u + 150u + 12u * 60u + 150u && t_end < 3000u, "scan took %lu ms",
          (unsigned long)t_end);
    const uint16_t scans = ss.scans;
    o = run_scan(&ss, 4u, 10u, &t_end, &bad);
    CHECK(o == vsm_outcome(VSM_OUT_SIDE, ss_payload(OBSTACLE_SIDE_RIGHT, 200u)) && ss.scans == scans + 1u,
          "box 4..10: out 0x%x", o);
    o = run_scan(&ss, 0u, 12u, &t_end, &bad);
    CHECK(o == 0u && ss.phase == SS_REVERSE && bad == 0,
          "blocked first scan must reattempt: out 0x%x phase %u bad %d", o, ss.phase, bad);
}

static void test_reverse_math(void) {
    CHECK(SS_REVERSE_COUNTS == 1241u, "100 mm at 12414 counts/m: %u", SS_REVERSE_COUNTS);
    ss_t ss;
    ss_init(&ss, &SS);
    ss_start(&ss, 0u, 0u);
    ss.phase = SS_REVERSE;
    ss.wait_ms = SS_REVERSE_TIMEOUT_MS;
    ss.reverse_origin[0] = 2000;
    ss.reverse_origin[1] = -2000;
    vsm_inputs_t in = {0, VC_EVENT_NONE, {2000 - 1240, -2000 - 1240}, false,
                       0u, &ping, false};
    vsm_drive_t d;
    CHECK(step(&ss, ss_step, &in, 10u, &d) == 0u && ss.phase == SS_REVERSE &&
              d.left_mm_s == -SS_REVERSE_MM_S && d.right_mm_s == -SS_REVERSE_MM_S,
          "1240 counts each side is short: phase %u", ss.phase);
    in.counts[0] = 2000 - 1241;
    in.counts[1] = -2000 - 1241;
    CHECK(step(&ss, ss_step, &in, 20u, &d) == 0u && ss.phase == SS_WAIT &&
              ss.notice == SS_NOTICE_REVERSE_END && d.left_mm_s == 0 && d.right_mm_s == 0,
          "1241 counts mean ends reverse: phase %u notice %u", ss.phase, ss.notice);
    ss.phase = SS_REVERSE;
    ss.since_ms = 0u;
    ss.reverse_origin[0] = 0;
    ss.reverse_origin[1] = 0;
    in.counts[0] = -2482;
    in.counts[1] = 0;
    CHECK(step(&ss, ss_step, &in, 30u, &d) == 0u && ss.phase == SS_WAIT,
          "mean of both wheels, not each wheel: phase %u", ss.phase);
    CHECK(step(&ss, ss_step, &in, 329u, &d) == 0u && ss.phase == SS_WAIT &&
              d.left_mm_s == 0 && d.servo_deg == 0 && ping == 0u,
          "settle after reverse: phase %u", ss.phase);
    ss.attempt = 1u;
    CHECK(step(&ss, ss_step, &in, 330u, &d) == 0u && ss.phase == SS_SETTLE &&
              ss.notice == SS_NOTICE_RESCAN_BEGIN && d.servo_deg == -30,
          "rescan starts a full sweep: phase %u notice %u servo %d", ss.phase,
          ss.notice, d.servo_deg);
}

static vsm_outcome_t run_reattempts(ss_t *ss, unsigned int free_after, int *bad,
                                    unsigned int *notices, unsigned int *wait_pings) {
    vsm_inputs_t in = {0, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    vsm_drive_t d = {0};
    int32_t milli[2] = {0, 0};
    bool in_flight = false;
    unsigned int at = 0u;
    unsigned int completed = 0u;
    *bad = 0;
    *notices = 0u;
    *wait_pings = 0u;
    ss_start(ss, 0u, 0u);
    for (uint32_t t = 0u; t < 30000u; t += 10u) {
        in.ping_sent = in_flight;
        in.echo_new = in_flight;
        in.echo = scan_view(at, ss->attempt, free_after);
        const uint8_t before_phase = ss->phase;
        const vsm_outcome_t o = step(ss, ss_step, &in, t, &d);
        if (ss->notice != SS_NOTICE_NONE) {
            *notices = (*notices << 2u) | ss->notice;
        }
        if (ss->completed_scans != completed) {
            completed = ss->completed_scans;
            if (ss->completed_attempt + 1u != completed) {
                ++*bad;
            }
        }
        if (o != 0u) {
            return o;
        }
        /* After a reverse the robot settles before the sweep: no pings. */
        if (before_phase == SS_WAIT && (in.ping_sent || ping != 0u)) {
            ++*wait_pings;
        }
        if (ss->phase == SS_REVERSE) {
            if (d.left_mm_s != -SS_REVERSE_MM_S || d.right_mm_s != -SS_REVERSE_MM_S ||
                d.servo_deg != 0 || ping != 0u) {
                ++*bad;
            }
        } else if (d.left_mm_s != 0 || d.right_mm_s != 0) {
            ++*bad;
        }
        reverse_wheels(in.counts, milli, &d);
        at = (unsigned int)((d.servo_deg + 30) / 5);
        in_flight = ping == VSM_PING_ASAP;
    }
    ++*bad;
    return 0u;
}

static void test_reattempts(void) {
    ss_t ss;
    ss_init(&ss, &SS);
    int bad;
    unsigned int notices;
    unsigned int wait_pings;
    vsm_outcome_t o = run_reattempts(&ss, 1u, &bad, &notices, &wait_pings);
    CHECK(o == vsm_outcome(VSM_OUT_SIDE, ss_payload(OBSTACLE_SIDE_LEFT, 200u)) && bad == 0 &&
              ss.completed_scans == 2u && ss.completed_attempt == 1u && ss.scans == 2u &&
              notices == ((1u << 4u) | (2u << 2u) | 3u) && wait_pings == 0u,
          "blocked->reverse->free: out 0x%x bad %d scans %u attempt %u notices 0x%x "
          "wait pings %u", o, bad, ss.completed_scans, ss.completed_attempt, notices,
          wait_pings);
    ss_init(&ss, &SS);
    o = run_reattempts(&ss, 9u, &bad, &notices, &wait_pings);
    CHECK(o == vsm_outcome(VSM_OUT_SIDE, ss_payload(OBSTACLE_SIDE_NONE, 200u)) && bad == 0 &&
              ss.completed_scans == 3u && ss.completed_attempt == SS_MAX_REATTEMPTS &&
              ss.attempt == SS_MAX_REATTEMPTS && wait_pings == 0u && ss.scans == 3u,
          "three blocked: out 0x%x bad %d scans %u attempt %u wait pings %u", o, bad,
          ss.completed_scans, ss.completed_attempt, wait_pings);

    ss_start(&ss, 0u, 0u);
    ss.phase = SS_REVERSE;
    ss.since_ms = 100u;
    ss.wait_ms = SS_REVERSE_TIMEOUT_MS;
    vsm_inputs_t in = {0, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    vsm_drive_t d;
    o = step(&ss, ss_step, &in, 100u + SS_REVERSE_TIMEOUT_MS - 1u, &d);
    CHECK(o == 0u && d.left_mm_s == -SS_REVERSE_MM_S, "before timeout: out 0x%x", o);
    o = step(&ss, ss_step, &in, 100u + SS_REVERSE_TIMEOUT_MS, &d);
    CHECK(o == vsm_outcome(VSM_OUT_FAILED, SS_FAIL_REVERSE_TIMEOUT) && d.left_mm_s == 0 &&
              d.right_mm_s == 0 && d.servo_deg == 0 && ping == 0u,
          "timeout fail-safe: out 0x%x drive %d/%d", o, d.left_mm_s, d.right_mm_s);
    ss_init(&ss, &SS);
    ss_start(&ss, 0u, 0u);
    ss.phase = SS_REVERSE;
    ss.since_ms = 0u;
    ss.wait_ms = SS_REVERSE_TIMEOUT_MS;
    in.counts[0] = -1241;
    in.counts[1] = -1241;
    o = step(&ss, ss_step, &in, SS_REVERSE_TIMEOUT_MS, &d);
    CHECK(o == 0u && ss.phase == SS_WAIT,
          "distance reached on timeout tick: 0x%x phase %u", o, ss.phase);
}

static void test_reverse_safety(void) {
    ss_t ss;
    ss_init(&ss, &SS);
    vsm_t vsm;
    vsm_init(&vsm, VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_set_module(&vsm, VSM_MOD_SCAN, ss_module(&ss));
    CHECK(vsm_start(&vsm, VSM_MOD_SCAN, 0u, 0u), "scan start");
    ss.phase = SS_REVERSE;
    ss.since_ms = 0u;
    ss.wait_ms = SS_REVERSE_TIMEOUT_MS;
    vsm_inputs_t in = {0, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    vsm_drive_t d = vsm_step(&vsm, &in, 0u, 10u);
    CHECK(d.left_mm_s == -SS_REVERSE_MM_S && d.action == VSM_ACT_SPEED, "reverse drive");
    d = vsm_step(&vsm, &in, MC_FAULT_ESTOP, 20u);
    CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_SAFETY && vsm.fault_detail == MC_FAULT_ESTOP &&
              d.action == VSM_ACT_STOP && d.left_mm_s == 0,
          "ESTOP mid-reverse: state %u fault %u action %u", vsm.state, vsm.fault, d.action);
    vsm_reset(&vsm);
    CHECK(vsm_start(&vsm, VSM_MOD_SCAN, 0u, 30u), "restart");
    ss.phase = SS_REVERSE;
    ss.since_ms = 30u;
    ss.wait_ms = SS_REVERSE_TIMEOUT_MS;
    d = vsm_step(&vsm, &in, 0u, 40u);
    vsm_stop(&vsm);
    d = vsm_step(&vsm, &in, 0u, 50u);
    CHECK(vsm.state == VSM_IDLE && d.action == VSM_ACT_STOP && d.left_mm_s == 0,
          "operator stop mid-reverse: state %u action %u", vsm.state, d.action);
}

/* -------------------------------------------------------------- DETOUR */

static void test_turn_speed(void) {
    /* 20 deg/s: 240.5 counts/s per wheel x 0.0806 mm = 19.4 mm/s. */
    CHECK(dr_turn_mm_s(&DR.dt, 20u) == 19, "20 deg/s: %d", dr_turn_mm_s(&DR.dt, 20u));
    CHECK(dr_turn_mm_s(&DR.dt, 0u) == 0, "0 deg/s");
    /* 40 deg/s: the robot sweep; 38 mm/s did not stick on the mat. */
    CHECK(dr_turn_mm_s(&DR.dt, 40u) >= 38, "40 deg/s: %d", dr_turn_mm_s(&DR.dt, 40u));
    CHECK(dr_turn_mm_s(&DR.dt, 135u) == 131, "135 deg/s: %d", dr_turn_mm_s(&DR.dt, 135u));
    dr_t dr;
    dr_init(&dr, &DR);
    CHECK(dr.sweep_mm_s == 19, "sweep %d", dr.sweep_mm_s);
}

/* Wheel counts follow the asked speeds (12.414 counts per mm). */
static void move_wheels(int32_t counts[2], int32_t milli[2], const vsm_drive_t *d) {
    const int v[2] = {d->left_mm_s, d->right_mm_s};
    for (unsigned int w = 0u; w < 2u; ++w) {
        milli[w] += v[w] * 124; /* 10 ms x 12.414 counts/mm x 1000 */
        counts[w] += milli[w] / 1000;
        milli[w] %= 1000;
    }
}

static void test_detour(void) {
    lp_t lp;
    lp_init(&lp);
    dr_t dr;
    dr_init(&dr, &DR);
    vsm_inputs_t in = {&lp, VC_EVENT_NONE, {5000, 7000}, false, 0u, &ping, false};
    vsm_drive_t d;

    /* Side NONE: FAILED NO_SIDE in the first step. */
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_NONE, 200u), 0u);
    vsm_outcome_t o = step(&dr, dr_step, &in, 0u, &d);
    CHECK(o == vsm_outcome(VSM_OUT_FAILED, DT_FAIL_NO_SIDE), "no side: out 0x%x", o);

    /* Detour LEFT (obstacle right), 300 mm: TURN_AWAY, no BACK. */
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_LEFT, 300u), 0u);
    CHECK(dr.detours == 2u && !dr.started, "start: detours %u", dr.detours);
    o = step(&dr, dr_step, &in, 1000u, &d);
    CHECK(o == 0u && dr.started && dt_step(&dr.dt) == DT_TURN_AWAY && dr.dt.side == 1 &&
              dr.dt.l0 == 5000 && dr.dt.r0 == 7000,
          "init: step %u side %d", dt_step(&dr.dt), dr.dt.side);
    /* Servo 30 right (toward the obstacle); pause, then the sweep. */
    CHECK(d.servo_deg == -30 && d.left_mm_s == 0 && d.right_mm_s == 0 && ping == 0u,
          "first step: servo %d wheels %d/%d ping %u", d.servo_deg, d.left_mm_s, d.right_mm_s, ping);
    o = step(&dr, dr_step, &in, 1160u, &d); /* servo settled (40 + 4 x 30 ms) */
    CHECK(ping == VSM_PING_ASAP && d.left_mm_s == 0, "settled: ping %u wheels %d", ping, d.left_mm_s);
    o = step(&dr, dr_step, &in, 1249u, &d);
    CHECK(d.left_mm_s == 0 && d.right_mm_s == 0, "pause 249 ms: %d/%d", d.left_mm_s, d.right_mm_s);
    o = step(&dr, dr_step, &in, 1250u, &d);
    CHECK(d.left_mm_s == -19 && d.right_mm_s == 19 && d.servo_deg == -30,
          "sweep left: %d/%d servo %d", d.left_mm_s, d.right_mm_s, d.servo_deg);

    /* Readings: ping and echo in one step are this ping (seen twice, then
     * lost twice: past the edge). */
    int32_t milli[2] = {0, 0};
    uint32_t t = 1260u;
    const ultrasonic_result_t seq[] = {echo_mm(250u), echo_mm(260u), echo_none(), echo_none()};
    unsigned int next = 0u;
    bool asked = ping == VSM_PING_ASAP;
    for (; t < 4000u && dt_step(&dr.dt) == DT_TURN_AWAY; t += 10u) {
        move_wheels(in.counts, milli, &d);
        in.ping_sent = asked && next < 4u;
        in.echo_new = in.ping_sent;
        in.echo = in.ping_sent ? seq[next++] : 0u;
        o = step(&dr, dr_step, &in, t, &d);
        asked = ping == VSM_PING_ASAP;
        CHECK(o == 0u, "turn away: out 0x%x", o);
        CHECK(d.left_mm_s == -d.right_mm_s && (d.right_mm_s == 19 || d.right_mm_s == 0),
              "turn away wheels %d/%d at %lu", d.left_mm_s, d.right_mm_s, (unsigned long)t);
    }
    CHECK(dt_step(&dr.dt) == DT_STRAIGHT && dr.dt.seen_once && dr.dt.sweep_last_end == DT_SWEEP_LOST &&
              dr.dt.heading > 0,
          "after turn away: step %u end %u heading %d", dt_step(&dr.dt), dr.dt.sweep_last_end,
          dr.dt.heading);
    in.ping_sent = false;
    in.echo_new = false;
    /* STRAIGHT: pause, then 131/131, servo still toward the obstacle. The
     * wheels are not matched to a count difference (no heading hold). */
    in.counts[0] += 40;
    o = step(&dr, dr_step, &in, t, &d);
    CHECK(d.left_mm_s == 0 && d.right_mm_s == 0, "straight pause: %d/%d", d.left_mm_s, d.right_mm_s);
    in.counts[0] += 40;
    o = step(&dr, dr_step, &in, t + 250u, &d);
    CHECK(d.left_mm_s == 131 && d.right_mm_s == 131 && d.servo_deg == -30,
          "straight: %d/%d servo %d", d.left_mm_s, d.right_mm_s, d.servo_deg);

    /* Backing in TURN_TOWARD is not a sweep: turn_mm_s, to the detour side. */
    dr.dt.step = DT_TURN_TOWARD;
    dr.dt.backing = true;
    dr.dt.back_x100 = 1000000;
    o = step(&dr, dr_step, &in, t + 260u, &d);
    o = step(&dr, dr_step, &in, t + 510u, &d);
    CHECK(d.left_mm_s == -131 && d.right_mm_s == 131, "back off: %d/%d", d.left_mm_s, d.right_mm_s);
    /* TURN_TOWARD sweep: toward the obstacle (right), sweep speed. */
    dr.dt.backing = false;
    o = step(&dr, dr_step, &in, t + 520u, &d);
    CHECK(d.left_mm_s == 0 && d.right_mm_s == 0, "turn toward pause: %d/%d", d.left_mm_s, d.right_mm_s);
    o = step(&dr, dr_step, &in, t + 770u, &d);
    CHECK(d.left_mm_s == 19 && d.right_mm_s == -19, "turn toward: %d/%d", d.left_mm_s, d.right_mm_s);
    /* A look (STOP) stops the wheels but keeps the motion: no new pause. */
    dr.dt.sweep_end = DT_SWEEP_SEEN;
    dr.dt.sweep_stop_ms = dr.dt.sweep_quiet_ms = t + 780u;
    o = step(&dr, dr_step, &in, t + 780u, &d);
    CHECK(d.left_mm_s == 0 && d.right_mm_s == 0 && ping == 0u, "look: %d/%d ping %u", d.left_mm_s,
          d.right_mm_s, ping);
    dr.dt.sweep_end = DT_SWEEP_NONE;
    o = step(&dr, dr_step, &in, t + 790u, &d);
    CHECK(d.left_mm_s == 19, "after look: %d (no pause)", d.left_mm_s);

    /* Echo fault streak: FAILED SENSOR. */
    dr.dt.step = DT_STRAIGHT;
    in.ping_sent = true;
    in.echo_new = true;
    in.echo = echo_fault(ULTRASONIC_FAULT_STREAK);
    o = step(&dr, dr_step, &in, t + 800u, &d);
    CHECK(o == vsm_outcome(VSM_OUT_FAILED, DT_FAIL_SENSOR), "sensor: out 0x%x", o);
    in.ping_sent = false;
    in.echo_new = false;

    /* DONE: line memory reset, one step stopped, then the outcome. */
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_LEFT, 300u), 0u);
    o = step(&dr, dr_step, &in, 0u, &d);
    dr.dt.step = DT_DONE;
    lp.flags |= LP_FLAG_LINE_SEEN;
    lp.position = 40;
    o = step(&dr, dr_step, &in, 10u, &d);
    CHECK(o == 0u && dr.done && !lp_line_seen(&lp) && lp.position == 0 && d.left_mm_s == 0 &&
              d.right_mm_s == 0 && d.servo_deg == 0 && ping == 0u,
          "done: out 0x%x seen %d pos %d servo %d", o, lp_line_seen(&lp), lp.position, d.servo_deg);
    o = step(&dr, dr_step, &in, 20u, &d);
    CHECK(o == vsm_outcome(VSM_OUT_DONE_NO_LINE, VC_LINE_SIDE_RIGHT), "no line: out 0x%x", o);
    /* Detour right: search left. Line seen after the reset: DONE_LINE. */
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_RIGHT, 300u), 0u);
    o = step(&dr, dr_step, &in, 0u, &d);
    CHECK(d.servo_deg == 30, "detour right: servo %d", d.servo_deg);
    dr.dt.step = DT_DONE;
    o = step(&dr, dr_step, &in, 10u, &d);
    o = step(&dr, dr_step, &in, 20u, &d);
    CHECK(o == vsm_outcome(VSM_OUT_DONE_NO_LINE, VC_LINE_SIDE_LEFT), "no line right: out 0x%x", o);
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_RIGHT, 300u), 0u);
    o = step(&dr, dr_step, &in, 0u, &d);
    dr.dt.step = DT_DONE;
    o = step(&dr, dr_step, &in, 10u, &d);
    lp.flags |= LP_FLAG_LINE_SEEN; /* the detector read the line */
    o = step(&dr, dr_step, &in, 20u, &d);
    CHECK(o == vsm_outcome(VSM_OUT_DONE_LINE, 0u), "line: out 0x%x", o);

    /* No ultrasonic in the glue (NULL): no crash, same drive. */
    in.ping_ms = 0;
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_LEFT, 300u), 0u);
    o = step(&dr, dr_step, &in, 0u, &d);
    CHECK(o == 0u && ping == VSM_PING_DEFAULT_MS, "null ping: out 0x%x ping %u", o, ping);
}

/* BACK: obstacle inside the turn circle. Reverse first. */
static void test_back(void) {
    lp_t lp;
    lp_init(&lp);
    dr_t dr;
    dr_init(&dr, &DR);
    vsm_inputs_t in = {&lp, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    vsm_drive_t d;
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_LEFT, 100u), 0u); /* 65 + 100 < 150 + 30 */
    vsm_outcome_t o = step(&dr, dr_step, &in, 0u, &d);
    o = step(&dr, dr_step, &in, 250u, &d);
    CHECK(o == 0u && dt_step(&dr.dt) == DT_BACK && d.left_mm_s == -131 && d.right_mm_s == -131,
          "back: step %u %d/%d", dt_step(&dr.dt), d.left_mm_s, d.right_mm_s);
}

/* ---------------------------------------------------------------- shell */

static void test_shell(void) {
    lp_t lp;
    lp_init(&lp);
    lt_t lt;
    ls_t ls;
    ss_t ss;
    dr_t dr;
    const lt_config_t LT = {{146, 85u, 85u, 62, 378, 50u, 340u}, 450, 20};
    const ls_config_t LS = {131, 316};
    lt_init(&lt, &LT);
    ls_init(&ls, &LS);
    ss_init(&ss, &SS);
    dr_init(&dr, &DR);
    vsm_t vsm;
    vsm_init(&vsm, VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_set_module(&vsm, VSM_MOD_LINE, lt_module(&lt));
    (void)vsm_set_module(&vsm, VSM_MOD_SEARCH, ls_module(&ls));
    (void)vsm_set_module(&vsm, VSM_MOD_SCAN, ss_module(&ss));
    (void)vsm_set_module(&vsm, VSM_MOD_DETOUR, dr_module(&dr));
    CHECK(vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u), "start");

    vsm_inputs_t in = {&lp, VC_EVENT_NONE, {0, 0}, false, 0u, &ping, false};
    lp.flags |= LP_FLAG_LINE_SEEN;
    uint32_t t = 10u;
    bool asked = false;
    unsigned int pings = 0u;
    uint32_t since = 0u;
    vsm_drive_t d = {0, 0, 0, 0};
    for (; t < 30000u && vsm.state == VSM_RUNNING && vsm.running != VSM_MOD_DETOUR; t += 10u) {
        /* The glue: default period, a ping when it is due. */
        in.ping_sent = asked || (ping != 0u && ping != VSM_PING_ASAP && t - since >= ping);
        if (in.ping_sent) {
            since = t;
            ++pings;
        }
        in.echo_new = in.ping_sent;
        /* A box 200 mm ahead; the scan sees it from 15 right to 10 left. */
        const int at = vsm.running == VSM_MOD_SCAN ? (d.servo_deg + 30) / 5 : 6;
        in.echo = view((unsigned int)at, 3u, 8u);
        ping = VSM_PING_DEFAULT_MS;
        d = vsm_step(&vsm, &in, 0u, t);
        asked = ping == VSM_PING_ASAP;
    }
    CHECK(vsm.state == VSM_RUNNING && vsm.running == VSM_MOD_DETOUR && vsm.handoffs == 3u &&
              ss.scans == 1u && dr.detours == 1u,
          "to detour: state %u running %u handoffs %lu", vsm.state, vsm.running,
          (unsigned long)vsm.handoffs);
    CHECK(vsm.last.from == VSM_MOD_SCAN &&
              vsm.last.outcome == vsm_outcome(VSM_OUT_SIDE, ss_payload(OBSTACLE_SIDE_LEFT, 200u)),
          "last handoff %u 0x%x", vsm.last.from, vsm.last.outcome);
    CHECK(dr.started && dr.dt.side == 1 && d.servo_deg == -30 && d.left_mm_s == 0,
          "detour first step: side %d servo %d", dr.dt.side, d.servo_deg);
    CHECK(pings >= 2u * OBSTACLE_SCAN_POINTS, "pings %u", pings);

    /* DONE with the line: back to LINE; the line module drives again. */
    dr.dt.step = DT_DONE;
    d = vsm_step(&vsm, &in, 0u, t);
    lp.flags |= LP_FLAG_LINE_SEEN;
    d = vsm_step(&vsm, &in, 0u, t + 10u);
    CHECK(vsm.running == VSM_MOD_LINE && vsm.last.outcome == vsm_outcome(VSM_OUT_DONE_LINE, 0u) &&
              d.left_mm_s == 146 && d.right_mm_s == 146 && d.servo_deg == 0,
          "back to line: running %u drive %d/%d servo %d", vsm.running, d.left_mm_s, d.right_mm_s,
          d.servo_deg);

    /* DONE without the line: SEARCH, toward the obstacle (right first). */
    vsm.running = VSM_MOD_DETOUR;
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_LEFT, 300u), t);
    d = vsm_step(&vsm, &in, 0u, t + 20u);
    dr.dt.step = DT_DONE;
    d = vsm_step(&vsm, &in, 0u, t + 30u);
    d = vsm_step(&vsm, &in, 0u, t + 40u);
    CHECK(vsm.running == VSM_MOD_SEARCH && d.left_mm_s == 131 && d.right_mm_s == -131,
          "search: running %u drive %d/%d", vsm.running, d.left_mm_s, d.right_mm_s);

    /* Detour FAILED: FAULT with the reason. */
    vsm.running = VSM_MOD_DETOUR;
    dr_start(&dr, ss_payload(OBSTACLE_SIDE_NONE, 300u), t);
    d = vsm_step(&vsm, &in, 0u, t + 50u);
    CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_FAILED && vsm.fault_detail == DT_FAIL_NO_SIDE &&
              d.action == VSM_ACT_STOP,
          "failed: state %u fault %u detail %u", vsm.state, vsm.fault, vsm.fault_detail);
}

int main(void) {
    test_payload();
    test_scan();
    test_reverse_math();
    test_reattempts();
    test_reverse_safety();
    test_turn_speed();
    test_detour();
    test_back();
    test_shell();
    if (failures != 0) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("test_obstacle_modules: all checks passed\n");
    return 0;
}
