/* Independent module test. See docs/BRANCH_LAYOUT.md for build commands. */
#include <assert.h>
#include "line_trace.h"
int main(void) {
    lp_t lp;
    lp_init(&lp);
    const lt_config_t cfg = {{171, 100u, 100u, 62, 378, 50u, 400u}, 450, 20};
    lt_t m;
    lt_init(&m, &cfg);
    lt_start(&m, 0u, 0u);
    vsm_inputs_t in = {.line = &lp};
    vsm_drive_t d = {0};
    assert(lt_step(&m, &in, 0u, &d) == 0u);
    assert(d.left_mm_s == 171 && d.right_mm_s == 171);
    lp.position = 20;
    assert(lt_step(&m, &in, 10u, &d) == 0u);
    assert(d.left_mm_s == 215 && d.right_mm_s == 127);
    in.line_event = vc_event(VC_EVENT_LINE_LOST, VC_LINE_SIDE_RIGHT);
    const vsm_outcome_t o = lt_step(&m, &in, 20u, &d);
    assert(vsm_outcome_type(o) == VSM_OUT_LINE_LOST);
    assert(vsm_outcome_payload(o) == VC_LINE_SIDE_RIGHT);
    lt_start(&m, 0u, 30u);
    in.line_event = VC_EVENT_NONE;
    in.echo_new = true;
    in.echo = (ultrasonic_result_t)(200u << ULTRASONIC_DISTANCE_SHIFT);
    vsm_outcome_t obstacle = 0u;
    for (unsigned int i = 0; i < OBSTACLE_CONFIRM_COUNT; ++i)
        obstacle = lt_step(&m, &in, 40u, &d);
    assert(vsm_outcome_type(obstacle) == VSM_OUT_OBSTACLE);
    return 0;
}
