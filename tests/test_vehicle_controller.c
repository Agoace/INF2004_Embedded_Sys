/*
 * Host unit test for src/vehicle_controller.c (step 1: line following).
 * It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_vehicle_controller.c src/vehicle_controller.c \
 *      -o build/test_vehicle_controller
 *   ./build/test_vehicle_controller
 */
#include <stdio.h>

#include "vehicle_controller.h"

static int failures;
static const char *test_name;
static int step;

/* Expected command: type and, for FOLLOW/ROTATE, the parameter. */
#define NONE VC_CMD_NONE, 0
#define STOP VC_CMD_STOP, 0
#define FOLLOW VC_CMD_FOLLOW, VC_FOLLOW_CRUISE
#define ROT(d) VC_CMD_ROTATE, (d)

static void check(vc_t *vc, vc_event_t event, uint32_t now_ms,
                  vc_command_type_t want_type, int want_param,
                  vc_state_t want_state) {
    const vc_command_t c = vc_handle(vc, event, now_ms);
    const vc_command_type_t type = vc_command_type(c);
    const int param = type == VC_CMD_ROTATE ? vc_command_degrees(c)
                                            : (int)vc_command_param(c);
    ++step;
    if (type != want_type || param != want_param || vc_state(vc) != want_state ||
        (c & 0xF000u) != 0u || (vc->bits & 0xF000u) != 0u) {
        printf("FAIL %s step %d: cmd %d/%d state %d; want cmd %d/%d state %d\n",
               test_name, step, type, param, vc_state(vc), want_type, want_param,
               want_state);
        ++failures;
    }
}

static void begin(const char *name, vc_t *vc, uint32_t now_ms) {
    test_name = name;
    step = 0;
    const vc_command_t c = vc_init(vc, now_ms);
    if (vc_command_type(c) != VC_CMD_STOP || vc_state(vc) != VC_STATE_IDLE) {
        printf("FAIL %s: init\n", name);
        ++failures;
    }
}

#define EV(type) vc_event(VC_EVENT_##type, 0u)
#define LOST(side) vc_event(VC_EVENT_LINE_LOST, VC_LINE_SIDE_##side)
#define S(name) VC_STATE_##name

int main(void) {
    vc_t vc;

    begin("start command", &vc, 1000u);
    check(&vc, EV(NONE), 1010u, NONE, S(IDLE));
    check(&vc, EV(START), 1020u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, EV(LINE_FOUND), 1030u, NONE, S(LINE_FOLLOW));
    check(&vc, EV(STOP), 1040u, STOP, S(IDLE));

    begin("auto-start after 5 s, once only", &vc, 0u);
    check(&vc, EV(NONE), 4999u, NONE, S(IDLE));
    check(&vc, EV(NONE), 5000u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, EV(STOP), 6000u, STOP, S(IDLE));
    check(&vc, EV(NONE), 20000u, NONE, S(IDLE));

    begin("stop in IDLE cancels auto-start", &vc, 0u);
    check(&vc, EV(STOP), 100u, NONE, S(IDLE));
    check(&vc, EV(NONE), 9000u, NONE, S(IDLE));
    check(&vc, EV(START), 9100u, FOLLOW, S(LINE_FOLLOW));

    begin("search left first, then found", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 100u, ROT(20), S(LINE_SEARCH));
    check(&vc, EV(NONE), 200u, NONE, S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 300u, ROT(-40), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 500u, ROT(65), S(LINE_SEARCH));
    check(&vc, EV(LINE_FOUND), 600u, FOLLOW, S(LINE_FOLLOW));

    begin("search right first when the line was last on the right", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(RIGHT), 100u, ROT(-20), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 300u, ROT(40), S(LINE_SEARCH));

    begin("unknown side searches left first", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(UNKNOWN), 100u, ROT(20), S(LINE_SEARCH));

    begin("full sweep without the line is a fault", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 0u, ROT(20), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 100u, ROT(-40), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 200u, ROT(65), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 300u, ROT(-90), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 400u, ROT(135), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 500u, ROT(-180), S(LINE_SEARCH));
    check(&vc, EV(MOTION_DONE), 600u, STOP, S(FAULT));
    if (vc_faults(&vc) != VC_FAULT_SEARCH) {
        printf("FAIL full sweep: fault mask %u\n", vc_faults(&vc));
        ++failures;
    }

    begin("search timeout is a fault", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 1000u, ROT(20), S(LINE_SEARCH));
    check(&vc, EV(NONE), 6999u, NONE, S(LINE_SEARCH));
    check(&vc, EV(NONE), 7000u, STOP, S(FAULT));

    begin("line found wins over a late timeout", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 0u, ROT(20), S(LINE_SEARCH));
    check(&vc, EV(LINE_FOUND), 7000u, FOLLOW, S(LINE_FOLLOW));

    begin("stop during search", &vc, 0u);
    check(&vc, EV(START), 0u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 0u, ROT(20), S(LINE_SEARCH));
    check(&vc, EV(STOP), 50u, STOP, S(IDLE));

    begin("estop from every state, reset to IDLE", &vc, 0u);
    check(&vc, EV(ESTOP), 10u, STOP, S(FAULT));
    check(&vc, EV(START), 20u, NONE, S(FAULT));
    check(&vc, EV(NONE), 9000u, NONE, S(FAULT));
    check(&vc, EV(RESET), 9100u, STOP, S(IDLE));
    check(&vc, EV(NONE), 20000u, NONE, S(IDLE)); /* no auto-start after reset */
    check(&vc, EV(START), 20100u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, EV(ESTOP), 20200u, STOP, S(FAULT));
    check(&vc, EV(RESET), 20300u, STOP, S(IDLE));
    check(&vc, EV(START), 20400u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 20500u, ROT(20), S(LINE_SEARCH));
    check(&vc, EV(ESTOP), 20600u, STOP, S(FAULT));
    if (vc_faults(&vc) != VC_FAULT_ESTOP) {
        printf("FAIL estop: fault mask %u\n", vc_faults(&vc));
        ++failures;
    }

    begin("stall while moving is a fault; ignored in IDLE", &vc, 0u);
    check(&vc, EV(STALL), 10u, NONE, S(IDLE));
    check(&vc, EV(START), 20u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, EV(STALL), 30u, STOP, S(FAULT));
    check(&vc, EV(ESTOP), 40u, NONE, S(FAULT));
    if (vc_faults(&vc) != VC_FAULT_STALL) {
        printf("FAIL stall: fault mask %u\n", vc_faults(&vc));
        ++failures;
    }

    begin("events that do not apply are ignored", &vc, 0u);
    check(&vc, EV(LINE_LOST), 10u, NONE, S(IDLE));
    check(&vc, EV(MOTION_DONE), 20u, NONE, S(IDLE));
    check(&vc, EV(RESET), 30u, NONE, S(IDLE));
    check(&vc, EV(START), 40u, FOLLOW, S(LINE_FOLLOW));
    check(&vc, EV(MOTION_DONE), 50u, NONE, S(LINE_FOLLOW));
    check(&vc, LOST(LEFT), 60u, ROT(20), S(LINE_SEARCH));
    check(&vc, LOST(LEFT), 70u, NONE, S(LINE_SEARCH));

    begin("clock wrap-around", &vc, 0xFFFFF000u);
    check(&vc, EV(NONE), 0xFFFFF000u + 4999u, NONE, S(IDLE));
    check(&vc, EV(NONE), 0xFFFFF000u + 5000u, FOLLOW, S(LINE_FOLLOW));

    /* Word layouts. */
    if (vc_event(VC_EVENT_LINE_LOST, VC_LINE_SIDE_RIGHT) != (5u | (2u << 5)) ||
        vc_event_payload(vc_event(VC_EVENT_LINE_LOST, 0x1FFu)) != 0x1FFu) {
        printf("FAIL event word layout\n");
        ++failures;
    }

    if (failures == 0) {
        printf("test_vehicle_controller: all tests passed\n");
        return 0;
    }
    printf("test_vehicle_controller: %d failure(s)\n", failures);
    return 1;
}
