#ifndef SIM_DETOUR_HOOKS_H
#define SIM_DETOUR_HOOKS_H
#include "line_position.h"
#include "line_steer.h"
#include "obstacle.h"
#include "obstacle_detour.h"
#include "obstacle_scan.h"
void sim_hook_dt_init(dt_t *dt, const dt_config_t *cfg, obstacle_side_t side,
    uint16_t mm, int32_t left, int32_t right, uint32_t now);
dt_word_t sim_hook_dt_update(dt_t *dt, bool seen, int32_t left, int32_t right, uint32_t now);
void sim_hook_dt_reading(dt_t *dt, ultrasonic_result_t result, uint32_t now);
obstacle_event_word_t sim_hook_obstacle_update(obstacle_monitor_t *watch, ultrasonic_result_t result);
bool sim_hook_obstacle_scan_start(void);
obstacle_scan_event_t sim_hook_obstacle_scan_service(void);
vc_event_t sim_hook_lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now);
steer_word_t sim_hook_steer_update(steer_t *steer, int8_t position);
steer_word_t sim_hook_steer_straight(steer_t *steer);
void sim_hook_steer_reset(steer_t *steer);
void sim_hook_obstacle_init(obstacle_monitor_t *watch);
#endif
