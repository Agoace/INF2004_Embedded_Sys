/* Host test for app_program/vehicle/vehicle_sm.c.
 * Build: cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
 *        tests/test_vehicle_sm.c app_program/vehicle/vehicle_sm.c -o build/test_vehicle_sm
 */
#include <stdio.h>

#include "../app_program/vehicle/vehicle_sm.h"

static int failures;
#define CHECK(cond, ...)                                                                \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            ++failures;                                                                 \
            printf("FAIL line %d: ", __LINE__);                                         \
            printf(__VA_ARGS__);                                                        \
            printf("\n");                                                               \
        }                                                                               \
    } while (0)

/* The application's detector data (opaque to the shell). */
struct vsm_inputs {
    int tag;
};

/* A fake module: returns a scripted outcome, drives a fixed speed. */
typedef struct {
    int starts;
    int steps;
    uint16_t start_payload;
    uint32_t start_ms;
    vsm_outcome_t next; /* returned by the next step, then RUNNING */
    int repeat;         /* if > 0: return `next` this many more steps */
    int16_t speed;
    int8_t servo;
    bool set_servo;
    int seen_tag;
} fake_t;

static fake_t fakes[VSM_MOD_COUNT];

static void fake_start(void *ctx, uint16_t payload, uint32_t now_ms) {
    fake_t *f = ctx;
    f->starts++;
    f->start_payload = payload;
    f->start_ms = now_ms;
}

static vsm_outcome_t fake_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *d) {
    (void)now_ms;
    fake_t *f = ctx;
    f->steps++;
    f->seen_tag = in ? in->tag : -1;
    d->left_mm_s = f->speed;
    d->right_mm_s = (int16_t)-f->speed;
    if (f->set_servo) d->servo_deg = f->servo;
    d->action = VSM_ACT_STOP; /* the shell must override this for RUNNING */
    const vsm_outcome_t o = f->next;
    if (f->repeat > 0) {
        f->repeat--;
    } else {
        f->next = VSM_OUT_RUNNING;
    }
    return o;
}

static vsm_t vsm;
static const struct vsm_inputs IN = {42};

static void setup(const vsm_route_t *routes, unsigned int n) {
    vsm_init(&vsm, routes, n);
    for (unsigned int i = 1u; i < VSM_MOD_COUNT; ++i) {
        fakes[i] = (fake_t){0, 0, 0u, 0u, VSM_OUT_RUNNING, 0, (int16_t)(100 * i), 0, false, 0};
        CHECK(vsm_set_module(&vsm, (vsm_module_id_t)i, (vsm_module_t){fake_start, fake_step, &fakes[i]}),
              "set module %u", i);
    }
}

int main(void) {
    /* 1. Outcome word. */
    {
        const vsm_outcome_t o = vsm_outcome(VSM_OUT_OBSTACLE, 230u);
        CHECK(vsm_outcome_type(o) == VSM_OUT_OBSTACLE && vsm_outcome_payload(o) == 230u, "pack 230");
        CHECK(o == (vsm_outcome_t)(1u | (230u << 4u)), "bit layout 0x%04x", o);
        CHECK(vsm_outcome_payload(vsm_outcome(VSM_OUT_SIDE, 4095u)) == 4095u, "payload 4095");
        CHECK(vsm_outcome_payload(vsm_outcome(VSM_OUT_SIDE, 5000u)) == 4095u &&
                  vsm_outcome_type(vsm_outcome(VSM_OUT_SIDE, 5000u)) == VSM_OUT_SIDE,
              "payload clamp 4095");
        CHECK(vsm_outcome_type(vsm_outcome((vsm_outcome_type_t)0x13u, 0u)) == (vsm_outcome_type_t)3u &&
                  vsm_outcome_payload(vsm_outcome((vsm_outcome_type_t)0x13u, 0u)) == 0u,
              "type masked to 4 bits, payload not touched");
    }

    /* 2. Registration and start. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    CHECK(!vsm_set_module(&vsm, VSM_MOD_NONE, (vsm_module_t){fake_start, fake_step, 0}), "no module 0");
    CHECK(!vsm_set_module(&vsm, VSM_MOD_COUNT, (vsm_module_t){fake_start, fake_step, 0}), "no module 5");
    CHECK(vsm.state == VSM_IDLE, "init idle");
    {
        const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 0u);
        CHECK(d.action == VSM_ACT_STOP && d.left_mm_s == 0 && d.right_mm_s == 0, "idle: stop");
    }
    CHECK(!vsm_start(&vsm, VSM_MOD_NONE, 0u, 0u), "start: module 0 refused");
    CHECK(!vsm_start(&vsm, VSM_MOD_COUNT, 0u, 0u), "start: module 5 refused");
    CHECK(vsm_start(&vsm, VSM_MOD_LINE, 7u, 100u), "start line");
    CHECK(fakes[VSM_MOD_LINE].starts == 1 && fakes[VSM_MOD_LINE].start_payload == 7u &&
              fakes[VSM_MOD_LINE].start_ms == 100u,
          "line started with payload 7 at 100");
    CHECK(!vsm_start(&vsm, VSM_MOD_LINE, 0u, 100u), "start: not twice");
    CHECK(vsm.last.from == VSM_MOD_NONE && vsm.last.to == VSM_MOD_LINE && vsm.handoffs == 1u, "start recorded");

    /* 3. RUNNING: the module's speeds, action SPEED, inputs passed. */
    {
        const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 110u);
        CHECK(d.action == VSM_ACT_SPEED && d.left_mm_s == 100 && d.right_mm_s == -100, "line drives 100/-100");
        CHECK(fakes[VSM_MOD_LINE].seen_tag == 42, "inputs passed");
    }

    /* 4. Servo: kept when a module does not set it; set when it does. */
    fakes[VSM_MOD_LINE].set_servo = true;
    fakes[VSM_MOD_LINE].servo = 25;
    (void)vsm_step(&vsm, &IN, 0u, 120u);
    fakes[VSM_MOD_LINE].set_servo = false;
    {
        const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 130u);
        CHECK(d.servo_deg == 25, "servo kept at 25 (%d)", d.servo_deg);
    }

    /* 5. Handoff: line OBSTACLE(230) -> scan, started with 230 and stepped
     * in the same tick (scan's speeds come out). */
    fakes[VSM_MOD_LINE].next = vsm_outcome(VSM_OUT_OBSTACLE, 230u);
    {
        const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 140u);
        CHECK(vsm.running == VSM_MOD_SCAN, "obstacle -> scan (%u)", vsm.running);
        CHECK(fakes[VSM_MOD_SCAN].starts == 1 && fakes[VSM_MOD_SCAN].start_payload == 230u &&
                  fakes[VSM_MOD_SCAN].start_ms == 140u,
              "scan started with 230 at 140");
        CHECK(fakes[VSM_MOD_SCAN].steps == 1 && d.left_mm_s == 300 && d.action == VSM_ACT_SPEED,
              "scan stepped in the same tick");
        CHECK(vsm.last.from == VSM_MOD_LINE && vsm.last.to == VSM_MOD_SCAN && vsm.last.t_ms == 140u &&
                  vsm.last.outcome == vsm_outcome(VSM_OUT_OBSTACLE, 230u) && vsm.handoffs == 2u,
              "handoff recorded");
    }

    /* 6. The whole default table. */
    {
        struct {
            vsm_module_id_t from;
            vsm_outcome_type_t out;
            vsm_module_id_t to;
        } const cases[] = {
            {VSM_MOD_LINE, VSM_OUT_OBSTACLE, VSM_MOD_SCAN},
            {VSM_MOD_SCAN, VSM_OUT_SIDE, VSM_MOD_DETOUR},
            {VSM_MOD_DETOUR, VSM_OUT_DONE_LINE, VSM_MOD_LINE},
            {VSM_MOD_DETOUR, VSM_OUT_DONE_NO_LINE, VSM_MOD_SEARCH},
            {VSM_MOD_LINE, VSM_OUT_LINE_LOST, VSM_MOD_SEARCH},
            {VSM_MOD_SEARCH, VSM_OUT_FOUND, VSM_MOD_LINE},
            {VSM_MOD_SEARCH, VSM_OUT_OBSTACLE, VSM_MOD_SCAN},
        };
        for (unsigned int k = 0u; k < sizeof cases / sizeof cases[0]; ++k) {
            setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
            (void)vsm_start(&vsm, cases[k].from, 0u, 0u);
            fakes[cases[k].from].next = vsm_outcome(cases[k].out, (uint16_t)(k + 1u));
            (void)vsm_step(&vsm, &IN, 0u, 10u);
            CHECK(vsm.state == VSM_RUNNING && vsm.running == cases[k].to &&
                      fakes[cases[k].to].start_payload == k + 1u,
                  "route %u: %u -> %u (got %u)", k, cases[k].from, cases[k].to, vsm.running);
        }
    }

    /* 7. FAILED: FAULT with the reason, stop. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_start(&vsm, VSM_MOD_DETOUR, 0u, 0u);
    fakes[VSM_MOD_DETOUR].next = vsm_outcome(VSM_OUT_FAILED, 9u);
    {
        const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 10u);
        CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_FAILED && vsm.fault_detail == 9u,
              "failed: fault, reason 9");
        CHECK(d.action == VSM_ACT_STOP && d.left_mm_s == 0, "failed: stop");
        CHECK(vsm.last.from == VSM_MOD_DETOUR && vsm.last.to == VSM_MOD_NONE && vsm.running == VSM_MOD_NONE,
              "failed recorded");
    }
    /* FAULT stays: start refused, stop does not clear, steps stop. */
    CHECK(!vsm_start(&vsm, VSM_MOD_LINE, 0u, 20u), "fault: start refused");
    vsm_stop(&vsm);
    CHECK(vsm.state == VSM_FAULT, "fault: stop does not clear");
    CHECK(vsm_step(&vsm, &IN, 0u, 30u).action == VSM_ACT_STOP, "fault: stop each step");
    vsm_reset(&vsm);
    CHECK(vsm.state == VSM_IDLE && vsm.fault == VSM_FAULT_NONE && vsm.fault_detail == 0u, "reset: idle");
    CHECK(vsm_start(&vsm, VSM_MOD_LINE, 0u, 40u), "reset: start again");

    /* 8. Outcome with no route (scan FOUND): FAULT NO_ROUTE. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_start(&vsm, VSM_MOD_SCAN, 0u, 0u);
    fakes[VSM_MOD_SCAN].next = vsm_outcome(VSM_OUT_FOUND, 0u);
    (void)vsm_step(&vsm, &IN, 0u, 10u);
    CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_NO_ROUTE &&
              vsm.fault_detail == vsm_outcome(VSM_OUT_FOUND, 0u),
          "no route: fault (%u)", vsm.fault);

    /* 9. Route to a module with no functions: FAULT NO_MODULE. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_set_module(&vsm, VSM_MOD_SCAN, (vsm_module_t){0, 0, 0});
    CHECK(!vsm_start(&vsm, VSM_MOD_SCAN, 0u, 0u), "start: module without functions refused");
    (void)vsm_set_module(&vsm, VSM_MOD_SEARCH, (vsm_module_t){fake_start, 0, 0});
    CHECK(!vsm_start(&vsm, VSM_MOD_SEARCH, 0u, 0u), "start: module without step refused");
    (void)vsm_set_module(&vsm, VSM_MOD_DETOUR, (vsm_module_t){0, fake_step, 0});
    CHECK(!vsm_start(&vsm, VSM_MOD_DETOUR, 0u, 0u), "start: module without start refused");
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u);
    fakes[VSM_MOD_LINE].next = vsm_outcome(VSM_OUT_OBSTACLE, 0u);
    (void)vsm_step(&vsm, &IN, 0u, 10u);
    CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_NO_MODULE && vsm.fault_detail == VSM_MOD_SCAN,
          "no module: fault");

    /* 10. Safety: a latched motor fault stops any module. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_start(&vsm, VSM_MOD_DETOUR, 0u, 0u);
    {
        const int steps = fakes[VSM_MOD_DETOUR].steps;
        const vsm_drive_t d = vsm_step(&vsm, &IN, 0x02u, 10u);
        CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_SAFETY && vsm.fault_detail == 0x02u,
              "safety: fault, bits 0x02");
        CHECK(d.action == VSM_ACT_STOP && fakes[VSM_MOD_DETOUR].steps == steps, "safety: module not stepped");
    }
    /* Bit 0x01 (left stall) too. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u);
    (void)vsm_step(&vsm, &IN, 0x01u, 10u);
    CHECK(vsm.state == VSM_FAULT && vsm.fault_detail == 0x01u, "safety: bit 0x01");
    /* A fault bit while IDLE does not make a FAULT. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_step(&vsm, &IN, 0x01u, 10u);
    CHECK(vsm.state == VSM_IDLE, "safety while idle: stays idle");

    /* 11. Operator stop from RUNNING. */
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u);
    vsm_stop(&vsm);
    CHECK(vsm.state == VSM_IDLE && vsm.running == VSM_MOD_NONE, "stop: idle");
    CHECK(vsm_step(&vsm, &IN, 0u, 10u).action == VSM_ACT_STOP, "stop: drive stop");
    vsm_reset(&vsm);
    CHECK(vsm.state == VSM_IDLE, "reset from idle: idle");
    /* Reset while running does nothing. */
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 20u);
    vsm_reset(&vsm);
    CHECK(vsm.state == VSM_RUNNING && vsm.running == VSM_MOD_LINE, "reset while running: still running");

    /* Init clears the servo angle. */
    vsm.servo_deg = 9;
    setup(VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u);
    CHECK(vsm_step(&vsm, &IN, 0u, 10u).servo_deg == 0, "init: servo 0");

    /* 12. Handoff limit: 3 handoffs in one step pass, the 4th is a FAULT.
     * Ping-pong table line <-> search. */
    {
        static const vsm_route_t ping[] = {
            {VSM_MOD_LINE, VSM_OUT_LINE_LOST, VSM_MOD_SEARCH},
            {VSM_MOD_SEARCH, VSM_OUT_FOUND, VSM_MOD_LINE},
        };
        /* line -> search -> line -> search (3 handoffs), then search runs. */
        setup(ping, 2u);
        (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u);
        fakes[VSM_MOD_LINE].next = vsm_outcome(VSM_OUT_LINE_LOST, 1u);
        fakes[VSM_MOD_LINE].repeat = 1;
        fakes[VSM_MOD_SEARCH].next = vsm_outcome(VSM_OUT_FOUND, 0u);
        {
            const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 10u);
            CHECK(vsm.state == VSM_RUNNING && vsm.running == VSM_MOD_SEARCH && vsm.handoffs == 4u &&
                      d.left_mm_s == 200,
                  "3 handoffs in one step (handoffs %u, running %u)", vsm.handoffs, vsm.running);
        }
        /* One more handoff in the same step: FAULT. */
        setup(ping, 2u);
        (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, 0u);
        fakes[VSM_MOD_LINE].next = vsm_outcome(VSM_OUT_LINE_LOST, 1u);
        fakes[VSM_MOD_LINE].repeat = 1;
        fakes[VSM_MOD_SEARCH].next = vsm_outcome(VSM_OUT_FOUND, 0u);
        fakes[VSM_MOD_SEARCH].repeat = 1;
        {
            const vsm_drive_t d = vsm_step(&vsm, &IN, 0u, 10u);
            CHECK(vsm.state == VSM_FAULT && vsm.fault == VSM_FAULT_HANDOFF_LOOP &&
                      vsm.fault_detail == vsm_outcome(VSM_OUT_FOUND, 0u) && d.action == VSM_ACT_STOP,
                  "4th handoff: fault (%u)", vsm.fault);
            CHECK(fakes[VSM_MOD_LINE].starts == 2 && fakes[VSM_MOD_SEARCH].starts == 2,
                  "loop: starts 2 and 2 (%d, %d)", fakes[VSM_MOD_LINE].starts, fakes[VSM_MOD_SEARCH].starts);
        }
    }

    if (failures) {
        printf("test_vehicle_sm: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_vehicle_sm: all tests passed\n");
    return 0;
}
