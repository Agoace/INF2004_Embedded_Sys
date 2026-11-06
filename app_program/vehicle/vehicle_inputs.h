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

#include "../subsystem3/line_position.h"
#include "../subsystem5/ultrasonic.h"
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
    /* Ultrasonic ping period (rule 3). Before each step the glue sets it
     * to its default (VSM_PING_DEFAULT_MS); a running module may change
     * it in its step. The glue then triggers a reading when this many ms
     * passed since the last trigger (the driver keeps at least 60 ms).
     * 0: no readings; VSM_PING_ASAP: at the next allowed time. NULL: the
     * glue has no ultrasonic sensor. */
    uint16_t *ping_ms;
    /* A reading was triggered since the previous step. The glue does not
     * trigger while an unread result waits, so when ping_sent and
     * echo_new are both true, the echo is the result of this ping. */
    bool ping_sent;
};

#define VSM_PING_DEFAULT_MS 100u
#define VSM_PING_ASAP 1u

/* Set the ping period, if the glue has a sensor. */
static inline void vsm_ask_ping(const struct vsm_inputs *in, uint16_t period_ms) {
    if (in->ping_ms) {
        *in->ping_ms = period_ms;
    }
}

#endif
