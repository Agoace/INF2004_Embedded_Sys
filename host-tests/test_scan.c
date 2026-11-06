/* Independent module test. See docs/BRANCH_LAYOUT.md for build commands. */
#include <assert.h>
#include "../app_program/subsystem5/servo_scan.h"
int main(void) {
    ss_t m;
    const ss_config_t cfg = {300u};
    ss_init(&m, &cfg);
    ss_start(&m, 0u, 0u);
    uint16_t ping = VSM_PING_DEFAULT_MS;
    vsm_inputs_t in = {.ping_ms = &ping};
    vsm_drive_t d = {77, 77, 9, VSM_ACT_SPEED};
    assert(ss_step(&m, &in, 290u, &d) == 0u);
    assert(d.left_mm_s == 0 && d.right_mm_s == 0 && d.servo_deg == 0 && ping == 0u);
    assert(ss_step(&m, &in, 300u, &d) == 0u);
    assert(m.phase == SS_SETTLE && d.servo_deg == -30);
    assert(ss_step(&m, &in, 450u, &d) == 0u);
    assert(m.phase == SS_MEASURE && ping == VSM_PING_ASAP);
    in.ping_sent = true;
    in.echo_new = true;
    in.echo = (ultrasonic_result_t)(200u << ULTRASONIC_DISTANCE_SHIFT);
    assert(ss_step(&m, &in, 460u, &d) == 0u);
    assert(m.readings == 1u);
    in.echo = (ultrasonic_result_t)(((unsigned)ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT) |
                       (ULTRASONIC_FAULT_STREAK << ULTRASONIC_STREAK_SHIFT));
    assert(ss_step(&m, &in, 520u, &d) == vsm_outcome(VSM_OUT_FAILED, SS_FAIL_ECHO));
    const uint16_t p = ss_payload(OBSTACLE_SIDE_LEFT, 511u);
    assert(ss_payload_side(p) == OBSTACLE_SIDE_LEFT && ss_payload_distance(p) == 511u);
    return 0;
}
