#include "line_trace.h"

void lt_init(lt_t *lt, const lt_config_t *cfg) {
    *lt = (lt_t){0};
    lt->cfg = *cfg;
    steer_mm_init(&lt->steer, &cfg->steer);
    lt->cfg.steer = lt->steer.cfg; /* with the limits applied */
    obstacle_init(&lt->watch);
}

void lt_start(void *ctx, uint16_t payload, uint32_t now_ms) {
    (void)payload;
    (void)now_ms;
    lt_t *lt = ctx;
    steer_mm_reset(&lt->steer);
    obstacle_init(&lt->watch);
    lt->have_counts = false;
    lt->heading = 0;
    lt->was_hold = false;
    lt->straight = false;
}

vsm_outcome_t lt_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive) {
    (void)now_ms;
    lt_t *lt = ctx;
    if (vc_event_type(in->line_event) == VC_EVENT_LINE_LOST) {
        return vsm_outcome(VSM_OUT_LINE_LOST, vc_event_payload(in->line_event));
    }
    if (in->echo_new) {
        const obstacle_event_word_t ev = obstacle_update(&lt->watch, in->echo);
        if (obstacle_event(ev) == OBSTACLE_EVENT_STOP) {
            return vsm_outcome(VSM_OUT_OBSTACLE, obstacle_event_distance(ev));
        }
        if (obstacle_event(ev) == OBSTACLE_EVENT_SENSOR_FAULT) {
            return vsm_outcome(VSM_OUT_FAILED, LT_FAIL_ECHO);
        }
    }

    if (!lt->have_counts) {
        lt->last[0] = in->counts[0];
        lt->last[1] = in->counts[1];
        lt->have_counts = true;
    }
    lt->heading += (in->counts[0] - lt->last[0]) - (in->counts[1] - lt->last[1]);
    lt->last[0] = in->counts[0];
    lt->last[1] = in->counts[1];
    if (lp_centred(in->line)) {
        lt->heading = 0;
    }
    lp_set_hint(in->line, lt->heading > lt->cfg.hint_counts    ? VC_LINE_SIDE_LEFT
                          : lt->heading < -lt->cfg.hint_counts ? VC_LINE_SIDE_RIGHT
                                                               : VC_LINE_SIDE_UNKNOWN);
    const bool hold = lp_junction_hold(in->line);
    const int32_t travel = (in->counts[0] + in->counts[1]) / 2;
    if (hold && !lt->was_hold) {
        lt->straight_from = travel;
    }
    lt->was_hold = hold;
    lt->straight = hold && travel - lt->straight_from < lt->cfg.straight_counts;
    const steer_mm_word_t w =
        lt->straight ? steer_mm_straight(&lt->steer) : steer_mm_update(&lt->steer, in->line->position);
    drive->left_mm_s = steer_mm_left(w);
    drive->right_mm_s = steer_mm_right(w);
    drive->servo_deg = 0;
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}
