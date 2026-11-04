/*
 * Detector data for the Vehicle Controller modules (src/vehicle_sm.h,
 * docs/VEHICLE_STATE_MACHINE.md). The application glue fills one copy
 * before each vsm_step() and the shell passes it to the running module.
 *
 * Detectors never decide (docs/ADR_VEHICLE_CONTROLLER.md, rule 1): each
 * field is a measurement or a detector event; the running module
 * interprets it for its job.
 *
 * Pure types: no hardware. Host-tested through the module tests.
 */
#ifndef VEHICLE_INPUTS_H
#define VEHICLE_INPUTS_H

#include <stdbool.h>
#include <stdint.h>

#include "line_position.h"
#include "ultrasonic.h"
#include "vehicle_controller.h"

struct vsm_inputs {
    /* Line detector (updated every 1 ms by the glue). Not const: a module
     * may ask it for a setting (rule 3), for example lp_set_hint(). */
    lp_t *line;
    /* The last LINE_LOST or LINE_FOUND event since the previous step, or
     * VC_EVENT_NONE. */
    vc_event_t line_event;
    /* Wheel encoder counts, forward positive (motor controller). */
    int32_t counts[2];
    /* A new ultrasonic reading since the previous step (servo straight
     * ahead unless a module turned it). */
    bool echo_new;
    ultrasonic_result_t echo;
};

#endif
