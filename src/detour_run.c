#include "detour_run.h"

#include "servo_scan.h"

int16_t dr_turn_mm_s(const dt_config_t *dt, unsigned int deg_s) {
    /* Each wheel: counts_per_deg_x100 / 200 counts per degree. */
    const uint64_t num = (uint64_t)deg_s * dt->counts_per_deg_x100 * DT_WHEEL_CIRCUMFERENCE_UM;
    const uint64_t den = 200u * (uint64_t)DT_COUNTS_PER_WHEEL_TURN * 1000u;
    const uint64_t v = (num + den / 2u) / den;
    return (int16_t)(v > 0x7FFFu ? 0x7FFFu : v);
}

void dr_init(dr_t *dr, const dr_config_t *cfg) {
    *dr = (dr_t){0};
    dr->cfg = *cfg;
    dr->sweep_mm_s = dr_turn_mm_s(&cfg->dt, cfg->dt.sweep_deg_s);
}

void dr_start(void *ctx, uint16_t payload, uint32_t now_ms) {
    (void)now_ms;
    dr_t *dr = ctx;
    dr->payload = payload;
    dr->started = false;
    dr->done = false;
    dr->move = (uint8_t)DT_MOVE_STOP; /* the first motion always pauses */
    ++dr->detours;
}

static void wheels(vsm_drive_t *drive, int left, int right) {
    drive->left_mm_s = (int16_t)left;
    drive->right_mm_s = (int16_t)right;
}

vsm_outcome_t dr_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive) {
    dr_t *dr = ctx;
    wheels(drive, 0, 0);
    if (!dr->started) {
        dt_init(&dr->dt, &dr->cfg.dt, ss_payload_side(dr->payload), ss_payload_distance(dr->payload),
                in->counts[0], in->counts[1], now_ms);
        dr->started = true;
    }
    if (dr->done) {
        if (lp_line_seen(in->line)) {
            return vsm_outcome(VSM_OUT_DONE_LINE, 0u);
        }
        return vsm_outcome(VSM_OUT_DONE_NO_LINE,
                           dr->dt.side > 0 ? VC_LINE_SIDE_RIGHT : VC_LINE_SIDE_LEFT);
    }
    if (in->ping_sent) {
        dt_triggered(&dr->dt);
    }
    if (in->echo_new) {
        dt_reading(&dr->dt, in->echo, now_ms);
    }
    const dt_word_t w = dt_update(&dr->dt, lp_line_seen(in->line), in->counts[0], in->counts[1], now_ms);
    const dt_step_t step = dt_step(&dr->dt);
    if (step == DT_FAILED) {
        return vsm_outcome(VSM_OUT_FAILED, (uint16_t)dr->dt.fail);
    }
    drive->servo_deg = (int8_t)dt_servo_deg(w);
    if (step == DT_DONE) {
        lp_reset_tracking(in->line);
        dr->done = true;
        vsm_ask_ping(in, 0u);
        return vsm_outcome(VSM_OUT_RUNNING, 0u);
    }
    vsm_ask_ping(in, dt_wants_reading(&dr->dt, now_ms) ? VSM_PING_ASAP : 0u);

    const dt_move_t move = dt_move(w);
    const int dir = dt_dir(w);
    if (move == DT_MOVE_STOP) { /* a look: the next motion of the same turn needs no pause */
        return vsm_outcome(VSM_OUT_RUNNING, 0u);
    }
    if ((uint8_t)move != dr->move || dir != dr->dir) {
        dr->move = (uint8_t)move;
        dr->dir = (int8_t)dir;
        dr->pause_until_ms = now_ms + dr->cfg.pause_ms;
    }
    if ((int32_t)(now_ms - dr->pause_until_ms) < 0) {
        return vsm_outcome(VSM_OUT_RUNNING, 0u);
    }
    if (move == DT_MOVE_TURN) {
        const bool sweep = dr->cfg.dt.sweep_deg_s != 0u && !dr->dt.backing &&
                           (step == DT_TURN_AWAY || step == DT_TURN_TOWARD);
        const int v = sweep ? dr->sweep_mm_s : dr->cfg.turn_mm_s;
        wheels(drive, -dir * v, dir * v);
    } else {
        wheels(drive, dir * dr->cfg.straight_mm_s, dir * dr->cfg.straight_mm_s);
    }
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}
