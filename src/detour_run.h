/*
 * Detour module for the Vehicle Controller shell (module DETOUR,
 * src/vehicle_sm.h). See docs/VC_DETOUR_RUN.md.
 *
 * Runs the detour logic of src/obstacle_detour.c (v1.8) unchanged and
 * turns its command word into wheel speeds:
 *
 *   STOP      wheels 0 (a look, or the coast at the end of a sweep)
 *   TURN      on the spot. In a sweep (TURN_AWAY, TURN_TOWARD before the
 *             obstacle is seen): dt.sweep_deg_s as a wheel speed, so the
 *             motor controller holds the rate. Else turn_mm_s.
 *   STRAIGHT  straight_mm_s on both wheels. No heading hold: the motor
 *             controller holds the wheel speeds (operator 2026-11-04).
 *   A new motion (move or direction) starts after pause_ms with the
 *   wheels at 0, as detour_test.
 *
 * Readings: the module asks for a ping when dt_wants_reading() is true
 * (VSM_PING_ASAP), else no pings. ping_sent gives dt_triggered(), echo_new
 * gives dt_reading(), in that order.
 *
 * The start payload is the SCAN payload (src/servo_scan.h): side and
 * obstacle distance. The detour starts on the first step (it needs the
 * wheel counts).
 *
 * Outcomes:
 *   DONE_LINE     the detour ended (ALIGN done), and the line is seen
 *   DONE_NO_LINE  the same, line not seen; payload: side to search
 *                 (vc_line_side_t, toward the obstacle side)
 *   FAILED        payload: dt_fail_t
 *   At DONE the module resets the line memory (lp_reset_tracking) and
 *   waits one step, stopped, so that the line detector reads the sensors
 *   again before the module decides.
 *
 * Pure logic: host-tested by tests/test_obstacle_modules.c.
 */
#ifndef DETOUR_RUN_H
#define DETOUR_RUN_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_detour.h"
#include "vehicle_inputs.h"
#include "vehicle_sm.h"

typedef struct {
    dt_config_t dt;        /* detour settings (sweep_min/max_duty not used) */
    int16_t straight_mm_s; /* STRAIGHT, BACK, ON_LINE */
    int16_t turn_mm_s;     /* turns that are not sweeps (back off, ALIGN) */
    uint16_t pause_ms;     /* wheels 0 before a new motion */
} dr_config_t;

typedef struct {
    dr_config_t cfg;
    int16_t sweep_mm_s; /* from dt.sweep_deg_s and the wheel geometry */
    dt_t dt;
    uint16_t payload;   /* start payload */
    bool started;       /* dt_init() done */
    bool done;          /* DONE: line memory reset, decide on the next step */
    uint8_t move;       /* dt_move_t of the last motion */
    int8_t dir;         /* direction of the last motion */
    uint32_t pause_until_ms;
    uint16_t detours;   /* starts since dr_init() */
} dr_t;

/* Wheel speed for a turn on the spot at deg_s, mm/s, rounded. */
int16_t dr_turn_mm_s(const dt_config_t *dt, unsigned int deg_s);

void dr_init(dr_t *dr, const dr_config_t *cfg);
void dr_start(void *ctx, uint16_t payload, uint32_t now_ms);
vsm_outcome_t dr_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive);

static inline vsm_module_t dr_module(dr_t *dr) {
    return (vsm_module_t){dr_start, dr_step, dr};
}

#endif
