/*
 * Independent module test. See docs/architecture/BRANCH_LAYOUT.md for build
 * commands.
 */
#include <assert.h>
#include "../app_program/subsystem5/detour_run.h"
#include "../app_program/subsystem5/servo_scan.h"
int main(void) {
    const dr_config_t cfg = {
        .dt = {.counts_per_deg_x100 = 2405u, .sweep_deg_s = 20u,
               .total_max_ms = 1000u, .away_max_deg = 90u},
        .straight_mm_s = 131, .turn_mm_s = 131, .pause_ms = 250u
    };
    assert(dr_turn_mm_s(&cfg.dt, 0u) == 0);
    assert(dr_turn_mm_s(&cfg.dt, 20u) == 19);
    assert(dr_turn_mm_s(&cfg.dt, 135u) == 131);
    dr_t m;
    dr_init(&m, &cfg);
    assert(m.sweep_mm_s == 19);
    lp_t lp;
    lp_init(&lp);
    uint16_t ping = VSM_PING_DEFAULT_MS;
    vsm_inputs_t in = {.line = &lp, .ping_ms = &ping};
    vsm_drive_t d = {0};
    dr_start(&m, ss_payload(OBSTACLE_SIDE_LEFT, 200u), 0u);
    assert(dr_step(&m, &in, 0u, &d) == 0u);
    assert(m.started && m.detours == 1u);
    assert(d.left_mm_s == 0 && d.right_mm_s == 0);
    assert(vsm_outcome_type(dr_step(&m, &in, 1001u, &d)) == VSM_OUT_FAILED);
    dr_start(&m, ss_payload(OBSTACLE_SIDE_RIGHT, 200u), 2000u);
    assert(!m.started && !m.done && m.detours == 2u);
    return 0;
}
