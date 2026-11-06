/* Independent module test. See docs/BRANCH_LAYOUT.md for build commands. */
#include <assert.h>
#include "line_search.h"
int main(void) {
    ls_t m;
    const ls_config_t cfg = {131, 316};
    ls_init(&m, &cfg);
    ls_start(&m, VC_LINE_SIDE_RIGHT, 0u);
    vsm_inputs_t in = {0};
    vsm_drive_t d = {0};
    assert(ls_step(&m, &in, 0u, &d) == 0u);
    assert(d.left_mm_s == 131 && d.right_mm_s == -131);
    in.counts[0] = 316;
    in.counts[1] = -316;
    assert(ls_step(&m, &in, 10u, &d) == 0u);
    assert(d.left_mm_s == -131 && d.right_mm_s == 131);
    in.counts[0] = -316;
    in.counts[1] = 316;
    assert(ls_step(&m, &in, 20u, &d) == vsm_outcome(VSM_OUT_FAILED, LS_FAIL_NOT_FOUND));
    ls_start(&m, VC_LINE_SIDE_LEFT, 30u);
    assert(ls_step(&m, &in, 30u, &d) == 0u);
    assert(d.left_mm_s == -131 && d.right_mm_s == 131);
    in.line_event = vc_event(VC_EVENT_LINE_FOUND, 0u);
    assert(vsm_outcome_type(ls_step(&m, &in, 40u, &d)) == VSM_OUT_FOUND);
    return 0;
}
