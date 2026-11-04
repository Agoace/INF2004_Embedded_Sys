/*
 * Host unit test for the line tracing and line search modules
 * (src/line_trace.c, src/line_search.c), alone and in the shell
 * (src/vehicle_sm.c). It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
 *      tests/test_line_trace.c src/line_trace.c src/line_search.c \
 *      src/line_steer.c src/obstacle.c src/vehicle_sm.c src/line_position.c \
 *      -o build/test_line_trace
 *   ./build/test_line_trace
 */
#include <stdio.h>

#include "line_search.h"
#include "line_trace.h"
#include "motor_ctl.h"

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

/* Locked line tracing in mm/s (docs/LINE_STEER_MM.md). */
static const lt_config_t LT = {{171, 100u, 100u, 62, 378, 50u, 400u}, 450, 20};
static const ls_config_t LS = {131, 316};

static ultrasonic_result_t echo_mm(uint16_t mm) {
    return (ultrasonic_result_t)(mm << ULTRASONIC_DISTANCE_SHIFT);
}
static ultrasonic_result_t echo_fault(uint8_t streak) {
    return (ultrasonic_result_t)(((unsigned)ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT) |
                                 ((unsigned)streak << ULTRASONIC_STREAK_SHIFT));
}

static vsm_outcome_t step(void *m, vsm_outcome_t (*fn)(void *, const vsm_inputs_t *, uint32_t, vsm_drive_t *),
                          const vsm_inputs_t *in, vsm_drive_t *d) {
    *d = (vsm_drive_t){0, 0, 9, VSM_ACT_SPEED};
    return fn(m, in, 0u, d);
}

static void test_trace(void) {
    lp_t lp;
    lp_init(&lp);
    lt_t lt;
    lt_init(&lt, &LT);
    vsm_inputs_t in = {&lp, VC_EVENT_NONE, {1000, 1000}, false, 0u};
    vsm_drive_t d;
    lt_start(&lt, 0u, 0u);

    lp.position = 0;
    vsm_outcome_t o = step(&lt, lt_step, &in, &d);
    CHECK(o == 0u && d.left_mm_s == 171 && d.right_mm_s == 171 && d.servo_deg == 0,
          "centred: out %u, %d/%d servo %d", o, d.left_mm_s, d.right_mm_s, d.servo_deg);
    lp.position = 20;
    o = step(&lt, lt_step, &in, &d);
    CHECK(d.left_mm_s == 215 && d.right_mm_s == 127, "+20: %d/%d", d.left_mm_s, d.right_mm_s);

    /* Side hint from the counts: left - right > 20 -> LEFT; < -20 -> RIGHT. */
    in.counts[0] += 20;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lp.hint == VC_LINE_SIDE_UNKNOWN && lt.heading == 20, "hint at +20: %u", lp.hint);
    in.counts[1] += 20;
    (void)step(&lt, lt_step, &in, &d);
    in.counts[0] += 21;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lp.hint == VC_LINE_SIDE_LEFT && lt.heading == 21, "hint left: %u, heading %ld", lp.hint,
          (long)lt.heading);
    in.counts[1] += 21;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lp.hint == VC_LINE_SIDE_UNKNOWN && lt.heading == 0, "hint none: %u %ld", lp.hint,
          (long)lt.heading);
    in.counts[1] += 20;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lp.hint == VC_LINE_SIDE_UNKNOWN, "hint at -20: %u", lp.hint);
    in.counts[1] += 1;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lp.hint == VC_LINE_SIDE_RIGHT, "hint right: %u", lp.hint);
    lp.flags |= LP_FLAG_CENTRED;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lp.hint == VC_LINE_SIDE_UNKNOWN && lt.heading == 0, "centred clears heading: %ld",
          (long)lt.heading);
    lp.flags &= (uint16_t)~LP_FLAG_CENTRED;

    /* Junction hold: straight (base on both) for 450 counts of travel. */
    lp.position = 60;
    lp.flags |= LP_FLAG_JUNCTION_HOLD;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lt.straight && d.left_mm_s == 171 && d.right_mm_s == 171, "junction straight %d/%d",
          d.left_mm_s, d.right_mm_s);
    in.counts[0] += 449;
    in.counts[1] += 449;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lt.straight, "straight at 449");
    in.counts[0] += 1;
    in.counts[1] += 1;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(!lt.straight && d.left_mm_s > d.right_mm_s, "steer at 450: %d/%d", d.left_mm_s,
          d.right_mm_s);
    lp.flags &= (uint16_t)~LP_FLAG_JUNCTION_HOLD;
    (void)step(&lt, lt_step, &in, &d);
    lp.flags |= LP_FLAG_JUNCTION_HOLD; /* a new hold starts a new straight */
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lt.straight, "second junction straight");
    /* lt_start() in a hold: the straight starts again from here. */
    in.counts[0] += 600;
    in.counts[1] += 600;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(!lt.straight, "hold past 450");
    lt_start(&lt, 0u, 0u);
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lt.straight, "start in a hold: straight again");
    lp.flags &= (uint16_t)~LP_FLAG_JUNCTION_HOLD;

    /* LINE_LOST: outcome with the side; no drive change. */
    in.line_event = vc_event(VC_EVENT_LINE_LOST, VC_LINE_SIDE_RIGHT);
    o = step(&lt, lt_step, &in, &d);
    CHECK(vsm_outcome_type(o) == VSM_OUT_LINE_LOST && vsm_outcome_payload(o) == VC_LINE_SIDE_RIGHT,
          "lost: 0x%04x", o);
    in.line_event = vc_event(VC_EVENT_LINE_FOUND, 0u);
    o = step(&lt, lt_step, &in, &d);
    CHECK(o == 0u, "FOUND is ignored while tracing: 0x%04x", o);
    in.line_event = VC_EVENT_NONE;

    /* Forward watch: STOP after the confirm count at <= 258 mm. */
    lt_start(&lt, 0u, 0u);
    in.echo_new = true;
    in.echo = echo_mm(200u);
    unsigned int n = 0u;
    do {
        o = step(&lt, lt_step, &in, &d);
        ++n;
    } while (o == 0u && n < 10u);
    CHECK(vsm_outcome_type(o) == VSM_OUT_OBSTACLE && vsm_outcome_payload(o) == 200u &&
              n == OBSTACLE_CONFIRM_COUNT,
          "obstacle: 0x%04x after %u", o, n);
    /* No new reading: the watch is not updated. */
    lt_start(&lt, 0u, 0u);
    in.echo_new = false;
    for (n = 0u; n < 5u; ++n) o = step(&lt, lt_step, &in, &d);
    CHECK(o == 0u, "stale echo used: 0x%04x", o);
    /* lt_start() resets the watch: one reading before is forgotten. */
    lt_init(&lt, &LT);
    lt_start(&lt, 0u, 0u);
    in.echo_new = true;
    (void)step(&lt, lt_step, &in, &d);
    lt_start(&lt, 0u, 0u);
    o = step(&lt, lt_step, &in, &d);
    CHECK(o == 0u, "watch not reset by start: 0x%04x", o);
    /* Sensor fault. */
    lt_start(&lt, 0u, 0u);
    in.echo = echo_fault(ULTRASONIC_FAULT_STREAK);
    o = step(&lt, lt_step, &in, &d);
    CHECK(vsm_outcome_type(o) == VSM_OUT_FAILED && vsm_outcome_payload(o) == LT_FAIL_ECHO,
          "echo fault: 0x%04x", o);
    in.echo_new = false;

    /* lt_start(): D and heading forgotten, counts taken at the first step. */
    (void)step(&lt, lt_step, &in, &d);
    in.counts[0] += 50;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(lt.heading == 50, "heading before restart %ld", (long)lt.heading);
    lt_start(&lt, 0u, 0u);
    lp.position = 40;
    in.counts[0] = 5000;
    in.counts[1] = 3000;
    (void)step(&lt, lt_step, &in, &d);
    CHECK(d.left_mm_s == 215 && d.right_mm_s == 127 && lt.heading == 0,
          "restart: %d/%d heading %ld", d.left_mm_s, d.right_mm_s, (long)lt.heading);

    /* Limits applied in lt_init(). */
    const lt_config_t big = {{171, 100u, 0u, 0, 900, 50u, 0u}, 450, 20};
    lt_init(&lt, &big);
    CHECK(lt.cfg.steer.max_mm_s == STEER_MM_MAX, "max limit %d", lt.cfg.steer.max_mm_s);
}

static void test_search(void) {
    lp_t lp;
    lp_init(&lp);
    ls_t ls;
    ls_init(&ls, &LS);
    vsm_inputs_t in = {&lp, VC_EVENT_NONE, {0, 0}, false, 0u};
    vsm_drive_t d;

    /* Line last seen right: turn right first (left forward). */
    ls_start(&ls, VC_LINE_SIDE_RIGHT, 0u);
    vsm_outcome_t o = step(&ls, ls_step, &in, &d);
    CHECK(o == 0u && d.left_mm_s == 131 && d.right_mm_s == -131 && d.servo_deg == 0,
          "right first: %d/%d", d.left_mm_s, d.right_mm_s);
    in.counts[0] = 315; /* angle (0 - 315)/2 * -1 = 157 */
    in.counts[1] = -316;
    (void)step(&ls, ls_step, &in, &d);
    CHECK(ls.step == 0u && d.left_mm_s == 131, "still first side at 315");
    in.counts[0] = 316;
    (void)step(&ls, ls_step, &in, &d);
    CHECK(ls.step == 1u && d.left_mm_s == -131 && d.right_mm_s == 131, "turn back: %d/%d",
          d.left_mm_s, d.right_mm_s);
    in.counts[0] = -315;
    in.counts[1] = 316;
    o = step(&ls, ls_step, &in, &d);
    CHECK(o == 0u && ls.step == 1u, "not past -316 yet: 0x%04x", o);
    in.counts[0] = -316;
    o = step(&ls, ls_step, &in, &d);
    CHECK(vsm_outcome_type(o) == VSM_OUT_FAILED && vsm_outcome_payload(o) == LS_FAIL_NOT_FOUND,
          "failed: 0x%04x", o);

    /* Unknown or left: left first; start counts taken at the first step. */
    in.counts[0] = 1000;
    in.counts[1] = 1000;
    ls_start(&ls, VC_LINE_SIDE_UNKNOWN, 0u);
    (void)step(&ls, ls_step, &in, &d);
    CHECK(d.left_mm_s == -131 && d.right_mm_s == 131 && ls.searches == 2u, "left first: %d/%d",
          d.left_mm_s, d.right_mm_s);
    in.counts[0] = 1000 - 316;
    in.counts[1] = 1000 + 316;
    (void)step(&ls, ls_step, &in, &d);
    CHECK(ls.step == 1u && d.left_mm_s == 131, "left then right");
    ls_start(&ls, VC_LINE_SIDE_LEFT, 0u);
    (void)step(&ls, ls_step, &in, &d);
    CHECK(ls.step == 0u && d.left_mm_s == -131, "start resets step: %u %d", ls.step, d.left_mm_s);

    in.line_event = vc_event(VC_EVENT_LINE_FOUND, 0u);
    o = step(&ls, ls_step, &in, &d);
    CHECK(vsm_outcome_type(o) == VSM_OUT_FOUND, "found: 0x%04x", o);
    in.line_event = vc_event(VC_EVENT_LINE_LOST, 1u);
    o = step(&ls, ls_step, &in, &d);
    CHECK(o == 0u, "LOST is ignored while searching: 0x%04x", o);
}

/* In the shell: LINE -> LINE_LOST -> SEARCH (same tick) -> FOUND -> LINE;
 * an obstacle with no SCAN module stops the run. */
static void test_shell(void) {
    lp_t lp;
    lp_init(&lp);
    static lt_t lt;
    static ls_t ls;
    lt_init(&lt, &LT);
    ls_init(&ls, &LS);
    vsm_t v;
    vsm_init(&v, VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    CHECK(vsm_set_module(&v, VSM_MOD_LINE, lt_module(&lt)), "set LINE");
    CHECK(vsm_set_module(&v, VSM_MOD_SEARCH, ls_module(&ls)), "set SEARCH");
    CHECK(vsm_start(&v, VSM_MOD_LINE, 0u, 0u), "start");
    vsm_inputs_t in = {&lp, VC_EVENT_NONE, {0, 0}, false, 0u};
    vsm_drive_t d = vsm_step(&v, &in, 0u, 10u);
    CHECK(d.action == VSM_ACT_SPEED && d.left_mm_s == 171, "line in shell: %d", d.left_mm_s);
    in.line_event = vc_event(VC_EVENT_LINE_LOST, VC_LINE_SIDE_LEFT);
    d = vsm_step(&v, &in, 0u, 20u);
    CHECK(v.running == VSM_MOD_SEARCH && d.left_mm_s == -131 && d.right_mm_s == 131,
          "search same tick: %u %d/%d", v.running, d.left_mm_s, d.right_mm_s);
    in.line_event = VC_EVENT_NONE;
    d = vsm_step(&v, &in, 0u, 30u);
    in.line_event = vc_event(VC_EVENT_LINE_FOUND, 0u);
    lp.position = -10;
    d = vsm_step(&v, &in, 0u, 40u);
    CHECK(v.running == VSM_MOD_LINE && d.left_mm_s < d.right_mm_s, "back to line: %u %d/%d",
          v.running, d.left_mm_s, d.right_mm_s);
    in.line_event = VC_EVENT_NONE;
    /* Search failed -> FAULT (FAILED). */
    in.line_event = vc_event(VC_EVENT_LINE_LOST, VC_LINE_SIDE_LEFT);
    (void)vsm_step(&v, &in, 0u, 50u);
    in.line_event = VC_EVENT_NONE;
    in.counts[0] = -316;
    in.counts[1] = 316;
    (void)vsm_step(&v, &in, 0u, 60u);
    in.counts[0] = 316;
    in.counts[1] = -316;
    d = vsm_step(&v, &in, 0u, 70u);
    CHECK(v.state == VSM_FAULT && v.fault == VSM_FAULT_FAILED &&
              v.fault_detail == LS_FAIL_NOT_FOUND && d.action == VSM_ACT_STOP,
          "search failed: state %u fault %u", v.state, v.fault);
    /* Obstacle with no SCAN module: FAULT NO_MODULE (stop). */
    vsm_reset(&v);
    CHECK(vsm_start(&v, VSM_MOD_LINE, 0u, 100u), "restart");
    in.echo_new = true;
    in.echo = echo_mm(150u);
    (void)vsm_step(&v, &in, 0u, 110u);
    d = vsm_step(&v, &in, 0u, 120u);
    CHECK(v.state == VSM_FAULT && v.fault == VSM_FAULT_NO_MODULE && v.fault_detail == VSM_MOD_SCAN &&
              v.last.from == VSM_MOD_LINE &&
              vsm_outcome_type(v.last.outcome) == VSM_OUT_OBSTACLE && d.action == VSM_ACT_STOP,
          "obstacle stop: state %u fault %u detail %u", v.state, v.fault, v.fault_detail);
    /* Stall: SAFETY. */
    vsm_reset(&v);
    in.echo_new = false;
    CHECK(vsm_start(&v, VSM_MOD_LINE, 0u, 200u), "restart 2");
    d = vsm_step(&v, &in, MC_FAULT_STALL_LEFT, 210u);
    CHECK(v.fault == VSM_FAULT_SAFETY && d.action == VSM_ACT_STOP, "stall: %u", v.fault);
}

int main(void) {
    test_trace();
    test_search();
    test_shell();
    if (failures == 0) {
        printf("test_line_trace: all tests passed\n");
        return 0;
    }
    printf("test_line_trace: %d failure(s)\n", failures);
    return 1;
}
