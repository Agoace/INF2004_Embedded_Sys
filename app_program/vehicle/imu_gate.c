/* Pure Vehicle gate: port of smojjofy 48b5f9b/038ff23 with explicit ACK. */
#include "imu_gate.h"
#include <stddef.h>
#include <string.h>

void ig_init(ig_t *g)
{
    memset(g, 0, sizeof(*g));
}

static uint8_t effective(const ig_t *g, uint32_t now)
{
    return imu_safety_evaluate(&g->report.value, g->have, now,
                               IMU_SAFETY_MAX_AGE_MS);
}

static bool safe_pose(const ig_t *g, uint32_t now)
{
    const uint8_t flags = IMU_SAFETY_FLAG_VALID | IMU_SAFETY_FLAG_UPRIGHT |
                           IMU_SAFETY_FLAG_NEAR_ONE_G;
    return g->have && effective(g, now) != IMU_SAFETY_IMU_UNAVAILABLE &&
           (g->report.value.flags & flags) == flags &&
           g->report.value.clear_blockers == 0u;
}

uint8_t ig_refusal(const ig_t *g, uint32_t now)
{
    if (!g->have) { return IG_NO_BOARD; }
    const uint8_t cause = imu_safety_unavailable_cause(
        &g->report.value, g->have, now, IMU_SAFETY_MAX_AGE_MS);
    if (cause == IMU_SAFETY_CAUSE_STALE) { return IG_STALE; }
    if (cause != IMU_SAFETY_CAUSE_NONE) { return IG_INVALID; }
    const imu_safety_output_t *v = &g->report.value;
    if ((v->flags & IMU_SAFETY_FLAG_UPRIGHT) == 0u) { return IG_POSE; }
    if ((v->flags & IMU_SAFETY_FLAG_NEAR_ONE_G) == 0u) { return IG_ONE_G; }
    if ((v->clear_blockers & IMU_SAFETY_BLOCK_WHEELS_MOVING) != 0u) {
        return IG_WHEELS;
    }
    return IG_WINDOW;
}

ig_action_t ig_step(ig_t *g, const imu_safety_report_t *report,
                     uint32_t now, bool active)
{
    ig_action_t action = {0};
    if (report != NULL) {
        g->report = *report;
        g->have = true;
    }
    const uint8_t trigger = effective(g, now);
    if (g->pending) {
        if ((int32_t)(now - g->deadline) >= 0) {
            action.recovery = IG_CANCELLED;
        } else if (!safe_pose(g, now)) {
            action.recovery = ig_refusal(g, now);
        } else if (g->token != 0u &&
                   g->report.applied_request == g->token &&
                   trigger == IMU_SAFETY_NONE) {
            g->pending = false;
            g->latched = IMU_SAFETY_NONE;
            g->cause = IMU_SAFETY_CAUSE_NONE;
            action.cleared = true;
            return action;
        } else if (g->report.value.clear_ms < IMU_SAFETY_CLEAR_MS) {
            action.recovery = IG_WINDOW;
        } else if ((int32_t)(now - g->retry_at) >= 0) {
            action.request_clear = true;
            g->retry_at = now + IG_RETRY_MS;
        }
        if (action.recovery != 0u) { g->pending = false; }
    }
    if (g->latched == IMU_SAFETY_NONE && trigger != IMU_SAFETY_NONE &&
        (trigger != IMU_SAFETY_IMU_UNAVAILABLE || active)) {
        g->latched = trigger;
        g->pending = false;
        g->cause = imu_safety_unavailable_cause(
            &g->report.value, g->have, now, IMU_SAFETY_MAX_AGE_MS);
        action.stop_edge = true;
    }
    return action;
}

bool ig_start_allowed(const ig_t *g, uint32_t now)
{
    return !g->pending && g->latched == IMU_SAFETY_NONE && g->have &&
           effective(g, now) == IMU_SAFETY_NONE &&
           (g->report.value.flags & IMU_SAFETY_FLAG_UPRIGHT) != 0u;
}

bool ig_stop_required(const ig_t *g)
{
    return g->pending || g->latched != IMU_SAFETY_NONE ||
           (g->have && g->report.value.latched_trigger != IMU_SAFETY_NONE);
}

bool ig_press(ig_t *g, uint32_t now)
{
    if (!ig_stop_required(g) || g->pending || !safe_pose(g, now) ||
        g->report.value.clear_ms < IMU_SAFETY_CLEAR_MS) {
        return false;
    }
    g->pending = true;
    g->token = 0u;
    g->deadline = now + IG_INTENT_MS;
    g->retry_at = now + IG_RETRY_MS;
    return true;
}

void ig_queued(ig_t *g, uint32_t token, uint32_t now)
{
    if (g->pending && token != 0u) { g->token = token; }
    g->retry_at = now + IG_RETRY_MS;
}
