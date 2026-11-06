#include "vehicle_sm.h"

const vsm_route_t VSM_DEFAULT_ROUTES[] = {
    {VSM_MOD_LINE, VSM_OUT_OBSTACLE, VSM_MOD_SCAN},
    {VSM_MOD_SCAN, VSM_OUT_SIDE, VSM_MOD_DETOUR},
    {VSM_MOD_DETOUR, VSM_OUT_DONE_LINE, VSM_MOD_LINE},
    {VSM_MOD_DETOUR, VSM_OUT_DONE_NO_LINE, VSM_MOD_SEARCH},
    {VSM_MOD_LINE, VSM_OUT_LINE_LOST, VSM_MOD_SEARCH},
    {VSM_MOD_SEARCH, VSM_OUT_FOUND, VSM_MOD_LINE},
    {VSM_MOD_SEARCH, VSM_OUT_OBSTACLE, VSM_MOD_SCAN},
};
const unsigned int VSM_DEFAULT_ROUTE_COUNT = sizeof VSM_DEFAULT_ROUTES / sizeof VSM_DEFAULT_ROUTES[0];

/* Module 0 can never be registered, so it has no functions. */
static bool has_module(const vsm_t *vsm, unsigned int id) {
    return id < VSM_MOD_COUNT && vsm->modules[id].start && vsm->modules[id].step;
}

static vsm_drive_t stop_drive(const vsm_t *vsm) {
    return (vsm_drive_t){0, 0, vsm->servo_deg, VSM_ACT_STOP};
}

static void record(vsm_t *vsm, uint32_t now_ms, unsigned int from, unsigned int to, vsm_outcome_t o) {
    vsm->last = (vsm_handoff_t){now_ms, (uint8_t)from, (uint8_t)to, o};
    vsm->handoffs++;
}

static void to_fault(vsm_t *vsm, vsm_fault_t fault, uint16_t detail, uint32_t now_ms, vsm_outcome_t o) {
    record(vsm, now_ms, vsm->running, VSM_MOD_NONE, o);
    vsm->state = VSM_FAULT;
    vsm->fault = (uint8_t)fault;
    vsm->fault_detail = detail;
    vsm->running = VSM_MOD_NONE;
}

void vsm_init(vsm_t *vsm, const vsm_route_t *routes, unsigned int route_count) {
    for (unsigned int i = 0u; i < VSM_MOD_COUNT; ++i) {
        vsm->modules[i] = (vsm_module_t){0, 0, 0};
    }
    vsm->routes = routes;
    vsm->route_count = route_count;
    vsm->state = VSM_IDLE;
    vsm->running = VSM_MOD_NONE;
    vsm->fault = VSM_FAULT_NONE;
    vsm->fault_detail = 0u;
    vsm->servo_deg = 0;
    vsm->handoffs = 0u;
    vsm->last = (vsm_handoff_t){0u, 0u, 0u, 0u};
}

bool vsm_set_module(vsm_t *vsm, vsm_module_id_t id, vsm_module_t module) {
    if (id == VSM_MOD_NONE || (unsigned int)id >= VSM_MOD_COUNT) {
        return false;
    }
    vsm->modules[id] = module;
    return true;
}

bool vsm_start(vsm_t *vsm, vsm_module_id_t first, uint16_t payload, uint32_t now_ms) {
    if (vsm->state != VSM_IDLE || !has_module(vsm, first)) {
        return false;
    }
    record(vsm, now_ms, VSM_MOD_NONE, first, vsm_outcome(VSM_OUT_RUNNING, payload));
    vsm->state = VSM_RUNNING;
    vsm->running = (uint8_t)first;
    vsm->modules[first].start(vsm->modules[first].ctx, payload, now_ms);
    return true;
}

void vsm_stop(vsm_t *vsm) {
    if (vsm->state == VSM_RUNNING) {
        vsm->state = VSM_IDLE;
        vsm->running = VSM_MOD_NONE;
    }
}

void vsm_reset(vsm_t *vsm) {
    if (vsm->state == VSM_FAULT) {
        vsm->state = VSM_IDLE;
        vsm->fault = VSM_FAULT_NONE;
        vsm->fault_detail = 0u;
    }
}

static unsigned int route(const vsm_t *vsm, unsigned int from, vsm_outcome_type_t type) {
    for (unsigned int i = 0u; i < vsm->route_count; ++i) {
        if (vsm->routes[i].from == from && vsm->routes[i].outcome == type) {
            return vsm->routes[i].to;
        }
    }
    return VSM_MOD_NONE;
}

vsm_drive_t vsm_step(vsm_t *vsm, const vsm_inputs_t *in, uint8_t mc_faults, uint32_t now_ms) {
    if (vsm->state == VSM_RUNNING && mc_faults != 0u) {
        to_fault(vsm, VSM_FAULT_SAFETY, mc_faults, now_ms, 0u);
    }
    if (vsm->state != VSM_RUNNING) {
        return stop_drive(vsm);
    }
    for (unsigned int n = 0u;; ++n) {
        const vsm_module_t *m = &vsm->modules[vsm->running];
        vsm_drive_t drive = {0, 0, vsm->servo_deg, VSM_ACT_SPEED};
        const vsm_outcome_t o = m->step(m->ctx, in, now_ms, &drive);
        const vsm_outcome_type_t type = vsm_outcome_type(o);
        if (type == VSM_OUT_RUNNING) {
            drive.action = VSM_ACT_SPEED;
            vsm->servo_deg = drive.servo_deg;
            return drive;
        }
        if (type == VSM_OUT_FAILED) {
            to_fault(vsm, VSM_FAULT_FAILED, vsm_outcome_payload(o), now_ms, o);
            return stop_drive(vsm);
        }
        const unsigned int next = route(vsm, vsm->running, type);
        if (next == VSM_MOD_NONE) {
            to_fault(vsm, VSM_FAULT_NO_ROUTE, o, now_ms, o);
            return stop_drive(vsm);
        }
        if (!has_module(vsm, next)) {
            to_fault(vsm, VSM_FAULT_NO_MODULE, (uint16_t)next, now_ms, o);
            return stop_drive(vsm);
        }
        if (n >= VSM_MAX_HANDOFFS_PER_STEP) {
            to_fault(vsm, VSM_FAULT_HANDOFF_LOOP, o, now_ms, o);
            return stop_drive(vsm);
        }
        record(vsm, now_ms, vsm->running, next, o);
        vsm->running = (uint8_t)next;
        vsm->modules[next].start(vsm->modules[next].ctx, vsm_outcome_payload(o), now_ms);
    }
}
