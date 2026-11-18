/* Port of smojjofy 48b5f9b/038ff23 recovery scenarios plus receipt guards. */
#include <assert.h>
#include <stddef.h>
#include "../app_program/vehicle/imu_gate.h"

static imu_safety_report_t safe(uint32_t now)
{
    return (imu_safety_report_t){.value = {
        .observation_ms = now, .clear_ms = IMU_SAFETY_CLEAR_MS,
        .flags = IMU_SAFETY_FLAG_VALID | IMU_SAFETY_FLAG_UPRIGHT |
                 IMU_SAFETY_FLAG_NEAR_ONE_G}};
}
int main(void)
{
    ig_t g;
    ig_init(&g);
    assert(!ig_step(&g, NULL, 10u, false).stop_edge);
    assert(!ig_start_allowed(&g, 10u)); /* Cold boot refusal, not a fault. */
    assert(ig_step(&g, NULL, 20u, true).stop_edge);
    assert(!ig_step(&g, NULL, 30u, true).stop_edge); /* No repeated ESTOP. */
    imu_safety_report_t r = safe(1000u);
    (void)ig_step(&g, &r, 1000u, false);
    assert(ig_press(&g, 1000u));
    ig_queued(&g, 7u, 1000u);
    /* Vehicle-only UNAVAILABLE latch: old NONE must not fake an ACK. */
    assert(!ig_step(&g, &r, 1001u, false).cleared);
    r.applied_request = 6u;
    r.value.observation_ms = 1010u;
    assert(!ig_step(&g, &r, 1010u, false).cleared);
    r.applied_request = 7u;
    r.value.observation_ms = 1020u;
    assert(ig_step(&g, &r, 1020u, false).cleared);
    assert(ig_start_allowed(&g, 1020u));
    r.value.latched_trigger = IMU_SAFETY_LIFT;
    assert(ig_step(&g, &r, 1020u, true).stop_edge);
    assert(ig_stop_required(&g));
    assert(!ig_step(&g, &r, 1020u, true).stop_edge);
    assert(ig_press(&g, 1020u));
    ig_queued(&g, 8u, 1020u);
    r.value.observation_ms = 1120u;
    assert(ig_step(&g, &r, 1120u, false).request_clear);
    ig_queued(&g, 9u, 1120u);
    r.value.latched_trigger = IMU_SAFETY_NONE;
    r.applied_request = 8u; /* Previous retry receipt is not this attempt. */
    assert(!ig_step(&g, &r, 1120u, false).cleared);
    r.applied_request = 9u;
    r.value.flags &= (uint8_t)~IMU_SAFETY_FLAG_UPRIGHT;
    assert(!ig_step(&g, &r, 1120u, false).cleared);
    assert(!g.pending && ig_stop_required(&g)); /* Unsafe ACK cancels. */
    r = safe(1200u);
    (void)ig_step(&g, &r, 1200u, false);
    assert(ig_press(&g, 1200u));
    ig_queued(&g, 10u, 1200u);
    r.value.observation_ms = 2200u;
    assert(ig_step(&g, &r, 2200u, false).recovery == IG_CANCELLED);
    assert(!ig_start_allowed(&g, 2200u));
    /* A fresh board receipt still cannot clear when its age is negative. */
    r = safe(2300u);
    (void)ig_step(&g, &r, 2300u, false);
    assert(ig_press(&g, 2300u));
    ig_queued(&g, 11u, 2300u);
    r = safe(2400u);
    r.applied_request = 11u;
    assert(!ig_step(&g, &r, 2310u, false).cleared);
    assert(ig_stop_required(&g));
    assert(!ig_start_allowed(&g, 2400u));
    return 0;
}
