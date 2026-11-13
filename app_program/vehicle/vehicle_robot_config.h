/* Extracted from src/vc_line_test.c: fixed P7 D85, detour v1.8.
 * Only NULL is spelled 0; numeric settings are not retyped. */
#ifndef VEHICLE_ROBOT_CONFIG_H
#define VEHICLE_ROBOT_CONFIG_H
#include "../subsystem3/line_trace.h"
#include "../subsystem3/line_search.h"
#include "../subsystem5/servo_scan.h"
#include "../subsystem5/detour_run.h"
static const lt_config_t LINE_CFG = {
    .steer = {.base_mm_s = 146, .gain_pct = 85u, .kd_pct = 85u, .min_mm_s = 62,
              .max_mm_s = 378, .far_from = 50u, .far_gain_pct = 340u},
    .straight_counts = 450,
    .hint_counts = 20,
};
/* Search: 20 % -> 131 mm/s; an eighth wheel turn each way (2535 / 8). */
static const ls_config_t SEARCH_CFG = {.speed_mm_s = 131, .turn_counts = 2535 / 8};

/* Scan: the robot stands still 300 ms first (detour_test STOP_SETTLE_MS). */
static const ss_config_t SCAN_CFG = {.stop_ms = 300u};

/* Detour v1.8 (src/detour_test.c, docs/archive/OBSTACLE_DETOUR_HANDOFF.md) with
 * wheel speeds: straights and the non-sweep turns 131 mm/s (20 %); the
 * sweeps 40 deg/s = 39 mm/s per wheel, held by the motor controller (no
 * duty rate control). No heading hold on the straights (operator
 * 2026-11-04). Back off: a fixed back_off_deg 25 degrees (operator
 * 2026-11-05, after run R1; before: half of the turn, at most 30). */
static const dr_config_t DETOUR_CFG = {
    .dt = {.back_off_fn = 0, /* fixed back_off_deg */
           .return_heading_deg = 90u,
           .return_max_mm = 1000u,
           .sweep_deg_s = 40u, /* pivot bench 2026-11-13: 19 mm/s sticks */
           .sweep_ping_ms = 60u,
           .sweep_seen_count = 2u,
           .sweep_min_duty = 0u, /* duty rate control: not used */
           .sweep_max_duty = 0u,
           .line_return_only = true,
           .turn_step_deg = 5u,
           .away_max_deg = 90u,
           .toward_max_deg = 150u,
           .max_straights = 5u,
           .back_off_deg = 25u, /* operator 2026-11-05 */
           .look_ms = 150u,
           .vehicle_mm = 400u, /* operator 2026-11-05: 350 touched the box (R2), 450 went too far (R3) */
           .sweep_mm = 150u,
           .margin_mm = 30u,
           .sensor_ahead_mm = 65u,
           .line_ahead_mm = 123u,
           .seen_mm = 400u,
           .counts_per_deg_x100 = 2405u,
           .total_max_ms = 40000u},
    .straight_mm_s = 131,
    .turn_mm_s = 131,
    .pause_ms = 250u,
};

/* Spin calibration (as src/line_spin.c, with wheel speeds). */
#define CAL_SPEED_MM_S 131 /* line_spin.c: 20 % */
#define CAL_STEP_COUNTS (2535 / 4)
#define CAL_STEP_TIMEOUT_MS 6000u
#define CAL_PAUSE_MS 200u
#define CAL_CENTRE_STOP 900u
#define CAL_CENTRE_MIN 500u
#define CAL_PEAK_DROP 50u


#endif
