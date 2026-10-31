#include "detour_hooks.h"
#include "sim_runtime.h"
#include <inttypes.h>

static void record_dt(const char *call, const dt_t *dt, unsigned int word) {
    sim_event("hook", "\"call\":\"%s\",\"step\":%u,\"fail\":%u,\"heading\":%d,"
        "\"x_mm\":%" PRId32 ",\"y_mm\":%" PRId32 ",\"word\":%u",
        call, (unsigned int)dt_step(dt), (unsigned int)dt_fail(dt), (int)dt->heading,
        dt->x, dt->y, word);
}
void sim_hook_dt_init(dt_t *dt, const dt_config_t *cfg, obstacle_side_t side,
    uint16_t mm, int32_t left, int32_t right, uint32_t now) {
    dt_init(dt, cfg, side, mm, left, right, now);
    record_dt("dt_init", dt, 0u);
}
dt_word_t sim_hook_dt_update(dt_t *dt, bool seen, int32_t left, int32_t right, uint32_t now) {
    const dt_word_t word = dt_update(dt, seen, left, right, now);
    record_dt("dt_update", dt, word);
    return word;
}
void sim_hook_dt_reading(dt_t *dt, ultrasonic_result_t result, uint32_t now) {
    dt_reading(dt, result, now);
    record_dt("dt_reading", dt, result);
}
obstacle_event_word_t sim_hook_obstacle_update(obstacle_monitor_t *watch, ultrasonic_result_t result) {
    const obstacle_event_word_t word = obstacle_update(watch, result);
    sim_event("hook", "\"call\":\"obstacle_update\",\"result\":%u", (unsigned int)word);
    return word;
}
bool sim_hook_obstacle_scan_start(void) {
    const bool started = obstacle_scan_start();
    sim_event("hook", "\"call\":\"obstacle_scan_start\",\"result\":%s", started ? "true" : "false");
    return started;
}
obstacle_scan_event_t sim_hook_obstacle_scan_service(void) {
    const obstacle_scan_event_t event = obstacle_scan_service();
    sim_event("hook", "\"call\":\"obstacle_scan_service\",\"result\":%u", (unsigned int)event);
    return event;
}
vc_event_t sim_hook_lp_update(lp_t *lp, const uint16_t raw[LP_SENSORS], uint32_t now) {
    const vc_event_t event = lp_update(lp, raw, now);
    sim_event("hook", "\"call\":\"lp_update\",\"result\":%u,\"seen\":%s,\"position\":%d",
        (unsigned int)event, lp_line_seen(lp) ? "true" : "false", (int)lp->position);
    return event;
}
steer_word_t sim_hook_steer_update(steer_t *steer, int8_t position) {
    const steer_word_t word = steer_update(steer, position);
    sim_event("hook", "\"call\":\"steer_update\",\"result\":%u", (unsigned int)word);
    return word;
}
steer_word_t sim_hook_steer_straight(steer_t *steer) {
    const steer_word_t word = steer_straight(steer);
    sim_event("hook", "\"call\":\"steer_straight\",\"result\":%u", (unsigned int)word);
    return word;
}
void sim_hook_steer_reset(steer_t *steer) {
    steer_reset(steer);
    sim_event("hook", "\"call\":\"steer_reset\"");
}
void sim_hook_obstacle_init(obstacle_monitor_t *watch) {
    obstacle_init(watch);
    sim_event("hook", "\"call\":\"obstacle_init\"");
}
