/*
 * Vehicle state machine: the Vehicle Controller shell
 * (docs/ADR_VEHICLE_CONTROLLER.md, docs/VEHICLE_STATE_MACHINE.md).
 *
 * It runs one module at a time. Each module does its job, interprets the
 * shared detectors for that job, and returns RUNNING or an outcome. The
 * shell maps (running module, outcome type) to the next module through a
 * route table, starts it with the outcome payload, and steps it in the
 * same tick. It holds no job details.
 *
 * Safety: vsm_step() takes the latched motor controller faults. Any
 * latched fault stops the run (FAULT) whatever module is running.
 *
 * The shell is pure logic: it returns a drive command and does not call
 * the motor controller. The application glue passes the command to
 * mcp_set_speed() / mcp_stop() / mcp_set_servo(); that glue is the only
 * caller of the motor controller.
 *
 * vehicle_controller.h (step 1) stays: its event word is used by the line
 * detector (line_position.h).
 *
 * Host-tested by tests/test_vehicle_sm.c.
 */
#ifndef VEHICLE_SM_H
#define VEHICLE_SM_H

#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------
 * Module numbers. 0 is "no module".
 * ------------------------------------------------------------------- */
typedef enum {
    VSM_MOD_NONE = 0,
    VSM_MOD_LINE = 1,   /* line tracing */
    VSM_MOD_SEARCH = 2, /* line search */
    VSM_MOD_SCAN = 3,   /* obstacle scan */
    VSM_MOD_DETOUR = 4, /* obstacle detour */
    VSM_MOD_COUNT = 5,
} vsm_module_id_t;

/* ---------------------------------------------------------------------
 * Outcome word (module -> shell): 16 bits.
 *
 *   bits 0-3    outcome type (4 bits)
 *   bits 4-15   payload (12 bits, 0..4095; meaning depends on the type,
 *               the shell passes it to the next module unchanged)
 * ------------------------------------------------------------------- */
typedef uint16_t vsm_outcome_t;

typedef enum {
    VSM_OUT_RUNNING = 0,      /* the job goes on */
    VSM_OUT_OBSTACLE = 1,     /* payload: distance mm */
    VSM_OUT_SIDE = 2,         /* scan result; payload: module-defined (side, distance) */
    VSM_OUT_DONE_LINE = 3,    /* detour done, line seen */
    VSM_OUT_DONE_NO_LINE = 4, /* detour done, no line; payload: side to search */
    VSM_OUT_LINE_LOST = 5,    /* payload: last side (vc_line_side_t) */
    VSM_OUT_FOUND = 6,        /* line found */
    VSM_OUT_FAILED = 7,       /* payload: module reason code */
    VSM_OUT_TYPES = 8,
} vsm_outcome_type_t;

#define VSM_OUT_TYPE_MASK 0x000Fu
#define VSM_OUT_PAYLOAD_SHIFT 4u
#define VSM_OUT_PAYLOAD_MAX 0x0FFFu

static inline vsm_outcome_t vsm_outcome(vsm_outcome_type_t type, uint16_t payload) {
    const uint32_t p = payload > VSM_OUT_PAYLOAD_MAX ? VSM_OUT_PAYLOAD_MAX : payload;
    return (vsm_outcome_t)(((uint32_t)type & VSM_OUT_TYPE_MASK) | (p << VSM_OUT_PAYLOAD_SHIFT));
}
static inline vsm_outcome_type_t vsm_outcome_type(vsm_outcome_t o) {
    return (vsm_outcome_type_t)(o & VSM_OUT_TYPE_MASK);
}
static inline uint16_t vsm_outcome_payload(vsm_outcome_t o) {
    return (uint16_t)(o >> VSM_OUT_PAYLOAD_SHIFT);
}

/* ---------------------------------------------------------------------
 * Drive command (module -> shell -> glue).
 * Before each step the shell sets the wheels to 0 and the servo to its
 * last angle, so a module that does not touch the servo keeps it.
 * ------------------------------------------------------------------- */
typedef enum {
    VSM_ACT_SPEED = 0, /* glue: mcp_set_speed(left, right) and mcp_set_servo(servo) */
    VSM_ACT_STOP = 1,  /* glue: mcp_stop() (IDLE, or FAULT from a module) */
} vsm_action_t;

typedef struct {
    int16_t left_mm_s;
    int16_t right_mm_s;
    int8_t servo_deg;
    uint8_t action; /* vsm_action_t */
} vsm_drive_t;

/* Detector data: defined by the application, passed to the modules as is. */
typedef struct vsm_inputs vsm_inputs_t;

typedef struct {
    /* Called when the shell hands over to this module. */
    void (*start)(void *ctx, uint16_t payload, uint32_t now_ms);
    /* Called every 10 ms while the module runs. Fill *drive; return
     * VSM_OUT_RUNNING or an outcome. */
    vsm_outcome_t (*step)(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive);
    void *ctx;
} vsm_module_t;

/* Route: in module `from`, outcome `outcome` starts module `to`. */
typedef struct {
    uint8_t from;    /* vsm_module_id_t */
    uint8_t outcome; /* vsm_outcome_type_t */
    uint8_t to;      /* vsm_module_id_t */
} vsm_route_t;

/* The first route table (ADR). FAILED and any outcome without a route
 * stop the run (FAULT). */
extern const vsm_route_t VSM_DEFAULT_ROUTES[];
extern const unsigned int VSM_DEFAULT_ROUTE_COUNT;

typedef enum {
    VSM_IDLE = 0,
    VSM_RUNNING = 1,
    VSM_FAULT = 2,
} vsm_state_t;

typedef enum {
    VSM_FAULT_NONE = 0,
    VSM_FAULT_SAFETY = 1,       /* detail: motor controller fault bits */
    VSM_FAULT_FAILED = 2,       /* detail: FAILED payload (module reason) */
    VSM_FAULT_NO_ROUTE = 3,     /* detail: outcome word */
    VSM_FAULT_NO_MODULE = 4,    /* detail: module number with no functions */
    VSM_FAULT_HANDOFF_LOOP = 5, /* detail: outcome word of the last handoff */
} vsm_fault_t;

/* Handoffs allowed in one vsm_step(): a new module may finish in its
 * first step. One more is a FAULT (HANDOFF_LOOP). */
#define VSM_MAX_HANDOFFS_PER_STEP 3u

typedef struct {
    uint32_t t_ms;
    uint8_t from;
    uint8_t to;
    vsm_outcome_t outcome;
} vsm_handoff_t;

typedef struct {
    vsm_module_t modules[VSM_MOD_COUNT];
    const vsm_route_t *routes;
    unsigned int route_count;
    uint8_t state;   /* vsm_state_t */
    uint8_t running; /* vsm_module_id_t */
    uint8_t fault;   /* vsm_fault_t */
    uint16_t fault_detail;
    int8_t servo_deg;
    uint32_t handoffs;       /* since vsm_init() */
    vsm_handoff_t last;      /* the last handoff (also to FAULT) */
} vsm_t;

void vsm_init(vsm_t *vsm, const vsm_route_t *routes, unsigned int route_count);

/* Register the functions of one module. False for VSM_MOD_NONE or out of range. */
bool vsm_set_module(vsm_t *vsm, vsm_module_id_t id, vsm_module_t module);

/* Start a run with module `first` (operator start). Only from IDLE.
 * False if not IDLE or the module has no functions. */
bool vsm_start(vsm_t *vsm, vsm_module_id_t first, uint16_t payload, uint32_t now_ms);

/* Operator stop: IDLE from RUNNING. A FAULT stays until vsm_reset(). */
void vsm_stop(vsm_t *vsm);

/* Leave FAULT: IDLE. The glue clears the motor controller faults too. */
void vsm_reset(vsm_t *vsm);

/* Every 10 ms. mc_faults: latched motor controller fault bits
 * (MC_FAULT_LATCHED part of mc_faults()). Returns the drive command. */
vsm_drive_t vsm_step(vsm_t *vsm, const vsm_inputs_t *in, uint8_t mc_faults, uint32_t now_ms);

#endif
