#include "vehicle_controller.h"

static const int search_offsets[VC_SEARCH_STEPS] = VC_SEARCH_OFFSETS_DEG;

/* ---- packed-field helpers -------------------------------------------- */

static void set_bits(vc_t *vc, uint16_t mask, unsigned int shift,
                     unsigned int value) {
    vc->bits = (uint16_t)((vc->bits & ~mask) | ((value << shift) & mask));
}

static unsigned int search_step(const vc_t *vc) {
    return (vc->bits & VC_SEARCH_STEP_MASK) >> VC_SEARCH_STEP_SHIFT;
}

static bool flag(const vc_t *vc, uint16_t bit) {
    return (vc->bits & bit) != 0u;
}

static void set_flag(vc_t *vc, uint16_t bit, bool on) {
    vc->bits = (uint16_t)(on ? (vc->bits | bit) : (vc->bits & ~bit));
}

static void enter(vc_t *vc, vc_state_t state, uint32_t now_ms) {
    set_bits(vc, VC_STATE_MASK, VC_STATE_SHIFT, (unsigned int)state);
    vc->entered_ms = now_ms;
}

/* ---- command builders ------------------------------------------------ */

static vc_command_t command(vc_command_type_t type, unsigned int param) {
    return (vc_command_t)(((uint32_t)type << VC_CMD_TYPE_SHIFT) |
                          (((uint32_t)param << VC_CMD_PARAM_SHIFT) &
                           VC_CMD_PARAM_MASK));
}

static vc_command_t rotate(int degrees) {
    /* 9-bit two's complement. */
    return command(VC_CMD_ROTATE, (unsigned int)degrees & 0x1FFu);
}

#define CMD_NONE command(VC_CMD_NONE, 0u)
#define CMD_STOP command(VC_CMD_STOP, 0u)
#define CMD_FOLLOW command(VC_CMD_FOLLOW, VC_FOLLOW_CRUISE)

/* ---- transitions ----------------------------------------------------- */

static vc_command_t to_fault(vc_t *vc, unsigned int fault, uint32_t now_ms) {
    set_bits(vc, VC_FAULT_MASK, VC_FAULT_SHIFT, vc_faults(vc) | fault);
    set_flag(vc, VC_AUTO_START_BIT, false);
    enter(vc, VC_STATE_FAULT, now_ms);
    return CMD_STOP;
}

static vc_command_t to_follow(vc_t *vc, uint32_t now_ms) {
    set_flag(vc, VC_AUTO_START_BIT, false);
    enter(vc, VC_STATE_LINE_FOLLOW, now_ms);
    return CMD_FOLLOW;
}

/* Heading offset of a search step, in degrees, mirrored if the line was
 * last seen on the right. */
static int offset_at(const vc_t *vc, unsigned int step) {
    const int offset = step < VC_SEARCH_STEPS ? search_offsets[step] : 0;
    return flag(vc, VC_SEARCH_MIRROR_BIT) ? -offset : offset;
}

/* Rotate from the current sweep position to the next one. */
static vc_command_t next_sweep(vc_t *vc, uint32_t now_ms) {
    const unsigned int step = search_step(vc);
    if (step >= VC_SEARCH_STEPS) {
        return to_fault(vc, VC_FAULT_SEARCH, now_ms);
    }
    const int from = step == 0u ? 0 : offset_at(vc, step - 1u);
    set_bits(vc, VC_SEARCH_STEP_MASK, VC_SEARCH_STEP_SHIFT, step + 1u);
    return rotate(offset_at(vc, step) - from);
}

static vc_command_t to_search(vc_t *vc, vc_line_side_t side, uint32_t now_ms) {
    enter(vc, VC_STATE_LINE_SEARCH, now_ms);
    set_bits(vc, VC_SEARCH_STEP_MASK, VC_SEARCH_STEP_SHIFT, 0u);
    set_flag(vc, VC_SEARCH_MIRROR_BIT, side == VC_LINE_SIDE_RIGHT);
    return next_sweep(vc, now_ms);
}

/* ---- public ---------------------------------------------------------- */

vc_command_t vc_init(vc_t *vc, uint32_t now_ms) {
    vc->bits = 0u;
    set_flag(vc, VC_AUTO_START_BIT, true);
    enter(vc, VC_STATE_IDLE, now_ms);
    return CMD_STOP;
}

vc_command_t vc_handle(vc_t *vc, vc_event_t event, uint32_t now_ms) {
    const vc_event_type_t type = vc_event_type(event);
    const vc_state_t state = vc_state(vc);
    const uint32_t in_state_ms = now_ms - vc->entered_ms;

    /* Events that act the same in every state except FAULT. */
    if (state != VC_STATE_FAULT) {
        if (type == VC_EVENT_ESTOP) {
            return to_fault(vc, VC_FAULT_ESTOP, now_ms);
        }
        if (type == VC_EVENT_STALL && state != VC_STATE_IDLE) {
            return to_fault(vc, VC_FAULT_STALL, now_ms);
        }
    }

    switch (state) {
    case VC_STATE_IDLE:
        if (type == VC_EVENT_START) {
            return to_follow(vc, now_ms);
        }
        if (type == VC_EVENT_STOP) {
            set_flag(vc, VC_AUTO_START_BIT, false);
            return CMD_NONE;
        }
        if (flag(vc, VC_AUTO_START_BIT) && in_state_ms >= VC_AUTO_START_MS) {
            return to_follow(vc, now_ms);
        }
        return CMD_NONE;

    case VC_STATE_LINE_FOLLOW:
        if (type == VC_EVENT_STOP) {
            enter(vc, VC_STATE_IDLE, now_ms);
            return CMD_STOP;
        }
        if (type == VC_EVENT_LINE_LOST) {
            return to_search(vc, (vc_line_side_t)(vc_event_payload(event) & 0x3u),
                             now_ms);
        }
        return CMD_NONE;

    case VC_STATE_LINE_SEARCH:
        if (type == VC_EVENT_STOP) {
            enter(vc, VC_STATE_IDLE, now_ms);
            return CMD_STOP;
        }
        if (type == VC_EVENT_LINE_FOUND) {
            return to_follow(vc, now_ms);
        }
        if (in_state_ms >= VC_SEARCH_TIMEOUT_MS) {
            return to_fault(vc, VC_FAULT_SEARCH, now_ms);
        }
        if (type == VC_EVENT_MOTION_DONE) {
            return next_sweep(vc, now_ms);
        }
        return CMD_NONE;

    case VC_STATE_FAULT:
        if (type == VC_EVENT_RESET) {
            set_bits(vc, VC_FAULT_MASK, VC_FAULT_SHIFT, 0u);
            enter(vc, VC_STATE_IDLE, now_ms);
            return CMD_STOP;
        }
        return CMD_NONE;

    default:
        /* States of later steps are not reachable in step 1. Stop safely;
         * no fault bit fits, so the mask does not change. */
        return to_fault(vc, 0u, now_ms);
    }
}
