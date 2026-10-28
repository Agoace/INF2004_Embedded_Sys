#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Vehicle Controller, step 1: line following only.
 *
 * vc_handle() takes one event word and the time, and returns one motion
 * command word. Call it for every event from mbf_ctrl_evt, and with
 * VC_EVENT_NONE when the 10 ms mailbox wait times out, so timeouts run.
 *
 * Pure logic: no hardware or RTOS calls. The same code runs on the Pico
 * and in the host unit test. See docs/VEHICLE_CONTROLLER.md.
 */

/* ---------------------------------------------------------------------
 * States: 4 bits. The numbers follow the 13-state list in the Week 8
 * notes (section 33), so later steps can add states without renumbering.
 * Step 1 uses only IDLE, LINE_FOLLOW, LINE_SEARCH, and FAULT.
 * ------------------------------------------------------------------- */
typedef enum {
    VC_STATE_INIT = 0,
    VC_STATE_CALIBRATE = 1,
    VC_STATE_IDLE = 2,
    VC_STATE_LINE_FOLLOW = 3,
    VC_STATE_LINE_SEARCH = 4,
    VC_STATE_EXECUTE_NAV = 5,
    VC_STATE_HUMP_TRAVERSE = 6,
    VC_STATE_HUMP_OBST_VALIDATE = 7,
    VC_STATE_OBSTACLE_STOP = 8,
    VC_STATE_BYPASS = 9,
    VC_STATE_LINE_REACQUIRE = 10,
    VC_STATE_MISSION_COMPLETE = 11,
    VC_STATE_FAULT = 12,
} vc_state_t;

/* ---------------------------------------------------------------------
 * Event word (into the controller): 16 bits.
 *
 *   bits 0-4    event type (5 bits; about 20 events in the full design)
 *   bits 5-13   payload (9 bits, meaning depends on the type)
 *   bits 14-15  spare, always 0
 * ------------------------------------------------------------------- */
typedef uint16_t vc_event_t;

typedef enum {
    VC_EVENT_NONE = 0,       /* Mailbox timeout: check timers only. */
    VC_EVENT_START = 1,      /* Telemetry CMD start. */
    VC_EVENT_STOP = 2,       /* Telemetry CMD stop. */
    VC_EVENT_ESTOP = 3,      /* Telemetry CMD estop or E-stop button. */
    VC_EVENT_RESET = 4,      /* Telemetry CMD reset. */
    VC_EVENT_LINE_LOST = 5,  /* Payload: vc_line_side_t. Already filtered
                                by the Line task (more than 50 ms). */
    VC_EVENT_LINE_FOUND = 6,
    VC_EVENT_MOTION_DONE = 7, /* The last ROTATE finished. */
    VC_EVENT_STALL = 8,       /* Motion task: a wheel stalled. */
} vc_event_type_t;

/* LINE_LOST payload: where the line was last seen. 2 bits. */
typedef enum {
    VC_LINE_SIDE_UNKNOWN = 0,
    VC_LINE_SIDE_LEFT = 1,
    VC_LINE_SIDE_RIGHT = 2,
} vc_line_side_t;

#define VC_EVENT_TYPE_SHIFT 0u
#define VC_EVENT_TYPE_MASK 0x001Fu
#define VC_EVENT_PAYLOAD_SHIFT 5u
#define VC_EVENT_PAYLOAD_MASK 0x3FE0u

static inline vc_event_t vc_event(vc_event_type_t type, uint16_t payload) {
    return (vc_event_t)(((uint32_t)type << VC_EVENT_TYPE_SHIFT) |
                        (((uint32_t)payload << VC_EVENT_PAYLOAD_SHIFT) &
                         VC_EVENT_PAYLOAD_MASK));
}

static inline vc_event_type_t vc_event_type(vc_event_t e) {
    return (vc_event_type_t)((e & VC_EVENT_TYPE_MASK) >> VC_EVENT_TYPE_SHIFT);
}

static inline uint16_t vc_event_payload(vc_event_t e) {
    return (uint16_t)((e & VC_EVENT_PAYLOAD_MASK) >> VC_EVENT_PAYLOAD_SHIFT);
}

/* ---------------------------------------------------------------------
 * Motion command word (out of the controller): 16 bits.
 *
 *   bits 0-2    command type (3 bits)
 *   bits 3-11   parameter (9 bits)
 *                 FOLLOW: 0 = cruise speed, 1 = slow speed
 *                 ROTATE: signed degrees, two's complement, -256..255;
 *                         positive = left (counter-clockwise from above)
 *   bits 12-15  spare, always 0
 *
 * A command is sent only when it changes. VC_CMD_NONE means "keep doing
 * the current command".
 * ------------------------------------------------------------------- */
typedef uint16_t vc_command_t;

typedef enum {
    VC_CMD_NONE = 0,
    VC_CMD_STOP = 1,
    VC_CMD_FOLLOW = 2,
    VC_CMD_ROTATE = 3,
} vc_command_type_t;

#define VC_FOLLOW_CRUISE 0u
#define VC_FOLLOW_SLOW 1u

#define VC_CMD_TYPE_SHIFT 0u
#define VC_CMD_TYPE_MASK 0x0007u
#define VC_CMD_PARAM_SHIFT 3u
#define VC_CMD_PARAM_MASK 0x0FF8u

static inline vc_command_type_t vc_command_type(vc_command_t c) {
    return (vc_command_type_t)((c & VC_CMD_TYPE_MASK) >> VC_CMD_TYPE_SHIFT);
}

static inline uint16_t vc_command_param(vc_command_t c) {
    return (uint16_t)((c & VC_CMD_PARAM_MASK) >> VC_CMD_PARAM_SHIFT);
}

/* ROTATE parameter as signed degrees. */
static inline int vc_command_degrees(vc_command_t c) {
    const int raw = (int)vc_command_param(c);
    return raw >= 256 ? raw - 512 : raw;
}

/* ---------------------------------------------------------------------
 * Tuning values. Provisional; tune on the track.
 * ------------------------------------------------------------------- */
#define VC_AUTO_START_MS 5000u    /* Design review: 5 s auto-start. */
#define VC_SEARCH_TIMEOUT_MS 6000u
/* Search sweep: heading offsets from where the line was lost, in degrees.
 * Positive = toward the side where the line was last seen. */
#define VC_SEARCH_STEPS 6u
#define VC_SEARCH_OFFSETS_DEG {20, -20, 45, -45, 90, -90}

/* Fault mask: one bit per independent fault. 3 bits in step 1. */
#define VC_FAULT_ESTOP 0x1u
#define VC_FAULT_STALL 0x2u
#define VC_FAULT_SEARCH 0x4u /* Line search used all steps or timed out. */

/* ---------------------------------------------------------------------
 * Controller state, private to the Vehicle Controller Task.
 *
 * bits (16-bit word):
 *   bits 0-3    state (vc_state_t)
 *   bits 4-6    search step: next sweep offset index, 0-6
 *   bit  7      search mirror: 1 = sweep right first
 *   bit  8      auto-start armed
 *   bits 9-11   fault mask
 *   bits 12-15  spare, always 0
 *
 * entered_ms is the time the current state started. It is a native
 * 32-bit millisecond count because it is compared with the RTOS clock.
 * ------------------------------------------------------------------- */
typedef struct {
    uint16_t bits;
    uint32_t entered_ms;
} vc_t;

#define VC_STATE_SHIFT 0u
#define VC_STATE_MASK 0x000Fu
#define VC_SEARCH_STEP_SHIFT 4u
#define VC_SEARCH_STEP_MASK 0x0070u
#define VC_SEARCH_MIRROR_BIT 0x0080u
#define VC_AUTO_START_BIT 0x0100u
#define VC_FAULT_SHIFT 9u
#define VC_FAULT_MASK 0x0E00u

/* Start in IDLE with auto-start armed. INIT and CALIBRATE come in a later
 * step. Returns the first command (STOP). */
vc_command_t vc_init(vc_t *vc, uint32_t now_ms);

vc_command_t vc_handle(vc_t *vc, vc_event_t event, uint32_t now_ms);

static inline vc_state_t vc_state(const vc_t *vc) {
    return (vc_state_t)((vc->bits & VC_STATE_MASK) >> VC_STATE_SHIFT);
}

static inline unsigned int vc_faults(const vc_t *vc) {
    return (vc->bits & VC_FAULT_MASK) >> VC_FAULT_SHIFT;
}

#endif
