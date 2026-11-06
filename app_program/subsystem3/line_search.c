#include "line_search.h"

void ls_init(ls_t *ls, const ls_config_t *cfg) {
    *ls = (ls_t){0};
    ls->cfg = *cfg;
    ls->direction = 1;
}

void ls_start(void *ctx, uint16_t payload, uint32_t now_ms) {
    (void)now_ms;
    ls_t *ls = ctx;
    ls->direction = payload == VC_LINE_SIDE_RIGHT ? -1 : 1;
    ls->step = 0u;
    ls->have_start = false;
    ++ls->searches;
}

vsm_outcome_t ls_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive) {
    (void)now_ms;
    ls_t *ls = ctx;
    if (vc_event_type(in->line_event) == VC_EVENT_LINE_FOUND) {
        return vsm_outcome(VSM_OUT_FOUND, 0u);
    }
    if (!ls->have_start) {
        ls->start[0] = in->counts[0];
        ls->start[1] = in->counts[1];
        ls->have_start = true;
    }
    const int32_t angle =
        ((in->counts[1] - ls->start[1]) - (in->counts[0] - ls->start[0])) / 2 * ls->direction;
    if (ls->step == 0u && angle >= ls->cfg.turn_counts) {
        ls->step = 1u;
    } else if (ls->step == 1u && angle <= -ls->cfg.turn_counts) {
        return vsm_outcome(VSM_OUT_FAILED, LS_FAIL_NOT_FOUND);
    }
    const int turn = ls->step == 0u ? ls->direction : -ls->direction; /* +1 = left */
    drive->left_mm_s = (int16_t)(-turn * ls->cfg.speed_mm_s);
    drive->right_mm_s = (int16_t)(turn * ls->cfg.speed_mm_s);
    drive->servo_deg = 0;
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}
