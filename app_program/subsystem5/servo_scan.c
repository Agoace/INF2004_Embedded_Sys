#include "servo_scan.h"

void ss_init(ss_t *ss, const ss_config_t *cfg) {
    *ss = (ss_t){0};
    ss->cfg = *cfg;
}

static void phase(ss_t *ss, ss_phase_t next, uint32_t wait_ms, uint32_t now_ms) {
    ss->phase = (uint8_t)next;
    ss->wait_ms = wait_ms;
    ss->since_ms = now_ms;
}

void ss_start(void *ctx, uint16_t payload, uint32_t now_ms) {
    (void)payload;
    ss_t *ss = ctx;
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        ss->profile.point[i] = OBSTACLE_SCAN_POINT_EMPTY;
    }
    ss->index = 0u;
    ss->readings = 0u;
    ss->pending = false;
    ss->word = 0u;
    phase(ss, SS_WAIT, ss->cfg.stop_ms, now_ms);
    ++ss->scans;
}

vsm_outcome_t ss_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive) {
    ss_t *ss = ctx;
    if (in->ping_sent && ss->phase == SS_MEASURE) {
        ss->pending = true;
    }
    if (in->echo_new && ss->pending) {
        ss->pending = false;
        if (ultrasonic_result_fault(in->echo)) {
            return vsm_outcome(VSM_OUT_FAILED, SS_FAIL_ECHO);
        }
        ss->profile.point[ss->index] = obstacle_scan_combine(ss->profile.point[ss->index], in->echo);
        if (++ss->readings >= OBSTACLE_SCAN_READINGS) {
            ss->readings = 0u;
            if (++ss->index < OBSTACLE_SCAN_POINTS) {
                phase(ss, SS_SETTLE, OBSTACLE_SCAN_SETTLE_STEP_MS, now_ms);
            } else {
                phase(ss, SS_RETURN, OBSTACLE_SCAN_SETTLE_RETURN_MS, now_ms);
            }
        }
    }

    const bool waited = now_ms - ss->since_ms >= ss->wait_ms;
    switch ((ss_phase_t)ss->phase) {
    case SS_WAIT:
        if (waited) {
            phase(ss, SS_SETTLE, OBSTACLE_SCAN_SETTLE_FIRST_MS, now_ms);
        }
        break;
    case SS_SETTLE:
        if (waited) {
            ss->phase = SS_MEASURE;
        }
        break;
    case SS_RETURN:
        if (waited) {
            ss->word = obstacle_profile_build(&ss->profile);
            const obstacle_side_t side = obstacle_profile_side(ss->word, &ss->profile);
            return vsm_outcome(VSM_OUT_SIDE, ss_payload(side, obstacle_profile_distance(ss->word)));
        }
        break;
    case SS_MEASURE:
        break;
    }

    drive->left_mm_s = 0;
    drive->right_mm_s = 0;
    drive->servo_deg = (int8_t)(ss->phase == SS_WAIT || ss->phase == SS_RETURN
                                    ? 0
                                    : obstacle_scan_angle_deg(ss->index));
    vsm_ask_ping(in, ss->phase == SS_MEASURE ? VSM_PING_ASAP : 0u);
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}
