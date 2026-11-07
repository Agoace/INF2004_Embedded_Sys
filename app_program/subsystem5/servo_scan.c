/* Pure SCAN module: full profiles, at most two encoder-based reattempts. */
#include "servo_scan.h"

static void phase(ss_t *ss, ss_phase_t next, uint32_t wait_ms,
                  uint32_t now_ms)
{
    ss->phase = (uint8_t)next;
    ss->wait_ms = wait_ms;
    ss->since_ms = now_ms;
}

/* A retry must not mix a new profile with readings from the old pose. */
static void clear_profile(ss_t *ss)
{
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        ss->profile.point[i] = OBSTACLE_SCAN_POINT_EMPTY;
    }
    ss->index = 0u;
    ss->readings = 0u;
    ss->pending = false;
    ss->word = 0u;
}

/* Signed modulo-2^32 delta without signed overflow or bad casts. */
static int64_t encoder_delta(int32_t current, int32_t origin)
{
    const uint32_t delta = (uint32_t)current - (uint32_t)origin;

    if (delta <= INT32_MAX) {
        return (int64_t)delta;
    }
    return (int64_t)delta - INT64_C(4294967296);
}

/* True when the mean of both wheels moved back by the reverse target. */
static bool reverse_complete(const ss_t *ss, const vsm_inputs_t *in)
{
    const int64_t left = encoder_delta(in->counts[0], ss->reverse_origin[0]);
    const int64_t right = encoder_delta(in->counts[1], ss->reverse_origin[1]);

    return -(left + right) >= (int64_t)SS_REVERSE_COUNTS * 2;
}

static vsm_outcome_t fail(ss_t *ss, uint8_t reason)
{
    ss->phase = SS_FAILED;
    ss->failure = reason;
    ss->pending = false;
    return vsm_outcome(VSM_OUT_FAILED, reason);
}

void ss_init(ss_t *ss, const ss_config_t *cfg)
{
    *ss = (ss_t){0};
    ss->cfg = *cfg;
}

void ss_start(void *ctx, uint16_t payload, uint32_t now_ms)
{
    ss_t *ss = ctx;

    (void)payload;
    clear_profile(ss);
    ss->attempt = 0u;
    ss->completed_attempt = 0u;
    ss->notice = SS_NOTICE_NONE;
    ss->failure = 0u;
    phase(ss, SS_WAIT, ss->cfg.stop_ms, now_ms);
    ++ss->scans;
}

/* Add one echo to the current point; returns false on a fault streak. */
static bool take_echo(ss_t *ss, const vsm_inputs_t *in, uint32_t now_ms)
{
    ss->pending = false;
    if (ultrasonic_result_fault(in->echo)) {
        return false;
    }
    ss->profile.point[ss->index] =
        obstacle_scan_combine(ss->profile.point[ss->index], in->echo);
    if (++ss->readings >= OBSTACLE_SCAN_READINGS) {
        ss->readings = 0u;
        if (++ss->index < OBSTACLE_SCAN_POINTS) {
            phase(ss, SS_SETTLE, OBSTACLE_SCAN_SETTLE_STEP_MS, now_ms);
        } else {
            phase(ss, SS_RETURN, OBSTACLE_SCAN_SETTLE_RETURN_MS, now_ms);
        }
    }
    return true;
}

/* A profile is complete: decide, or start one more reverse. */
static vsm_outcome_t finish_profile(ss_t *ss, const vsm_inputs_t *in,
                                    uint32_t now_ms)
{
    obstacle_side_t side;

    ss->word = obstacle_profile_build(&ss->profile);
    side = obstacle_profile_side(ss->word, &ss->profile);
    ss->completed_attempt = ss->attempt;
    ++ss->completed_scans;
    if ((side != OBSTACLE_SIDE_NONE) || (ss->attempt >= SS_MAX_REATTEMPTS)) {
        const uint16_t mm = obstacle_profile_distance(ss->word);

        return vsm_outcome(VSM_OUT_SIDE, ss_payload(side, mm));
    }
    ++ss->attempt;
    ss->reverse_origin[0] = in->counts[0];
    ss->reverse_origin[1] = in->counts[1];
    ss->pending = false;
    phase(ss, SS_REVERSE, SS_REVERSE_TIMEOUT_MS, now_ms);
    ss->notice = SS_NOTICE_REVERSE_BEGIN;
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}

vsm_outcome_t ss_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms,
                      vsm_drive_t *drive)
{
    ss_t *ss = ctx;
    vsm_outcome_t out = vsm_outcome(VSM_OUT_RUNNING, 0u);
    bool waited;

    /* Every step, also a failure: wheels 0, servo centre, no sonar. */
    *drive = (vsm_drive_t){.action = VSM_ACT_SPEED};
    vsm_ask_ping(in, 0u);
    ss->notice = SS_NOTICE_NONE;
    if (ss->phase == SS_FAILED) {
        return vsm_outcome(VSM_OUT_FAILED, ss->failure);
    }
    if (in->ping_sent && (ss->phase == SS_MEASURE)) {
        ss->pending = true;
    }
    if (in->echo_new && ss->pending && !take_echo(ss, in, now_ms)) {
        return fail(ss, SS_FAIL_ECHO);
    }

    waited = now_ms - ss->since_ms >= ss->wait_ms;
    switch ((ss_phase_t)ss->phase) {
    case SS_WAIT:
        if (waited) {
            phase(ss, SS_SETTLE, OBSTACLE_SCAN_SETTLE_FIRST_MS, now_ms);
            if (ss->attempt > 0u) {
                ++ss->scans;
                ss->notice = SS_NOTICE_RESCAN_BEGIN;
            }
        }
        break;
    case SS_SETTLE:
        if (waited) {
            ss->phase = SS_MEASURE;
        }
        break;
    case SS_RETURN:
        if (waited) {
            out = finish_profile(ss, in, now_ms);
        }
        break;
    case SS_REVERSE:
        if (reverse_complete(ss, in)) {
            /* Stop and settle again before the new full sweep. */
            clear_profile(ss);
            phase(ss, SS_WAIT, ss->cfg.stop_ms, now_ms);
            ss->notice = SS_NOTICE_REVERSE_END;
        } else if (waited) {
            return fail(ss, SS_FAIL_REVERSE_TIMEOUT);
        }
        break;
    case SS_MEASURE:
    case SS_FAILED:
        break;
    }
    if (vsm_outcome_type(out) != VSM_OUT_RUNNING) {
        return out;
    }

    if (ss->phase == SS_REVERSE) {
        drive->left_mm_s = -SS_REVERSE_MM_S;
        drive->right_mm_s = -SS_REVERSE_MM_S;
    }
    if ((ss->phase == SS_SETTLE) || (ss->phase == SS_MEASURE)) {
        drive->servo_deg = (int8_t)obstacle_scan_angle_deg(ss->index);
    }
    if (ss->phase == SS_MEASURE) {
        vsm_ask_ping(in, VSM_PING_ASAP);
    }
    return out;
}

_Static_assert(SS_MAX_REATTEMPTS <= 3u, "scan attempt uses two bits");
_Static_assert(SS_FAILED <= 7, "scan phase uses three bits");
