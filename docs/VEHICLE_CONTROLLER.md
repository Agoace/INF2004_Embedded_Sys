# Vehicle Controller

This record describes the Vehicle Controller state machine. It is built one
step at a time on the `vehicle-controller` branch.

- Code: [`src/vehicle_controller.c`](../src/vehicle_controller.c), [`src/vehicle_controller.h`](../src/vehicle_controller.h)
- Host unit test: [`tests/test_vehicle_controller.c`](../tests/test_vehicle_controller.c)
- Design: [system statechart](../design-review/system_statechart.mmd), [task interaction](../design-review/task_interaction.mmd)
- Bit widths: [Week 8 notes, sections 33–38](week8_change_requirement_1_notes.md#33-vehicle-controller-state)

## Steps

| Step | Scope | Status |
| --- | --- | --- |
| 1 | Line following only: IDLE, LINE_FOLLOW, LINE_SEARCH, FAULT | Done: host-tested |
| Later | INIT and CALIBRATE, obstacle path, navigation, hump, mission complete | Not started |

## Step 1: line following

```text
             START, or 5 s auto-start (first boot only)
     IDLE ─────────────────────────────▶ LINE_FOLLOW ◀─────────┐
      ▲  ◀──────── STOP ─────────────────   │                  │ LINE_FOUND
      │                                    │ LINE_LOST         │
   RESET                                   ▼                   │
      │                               LINE_SEARCH ─────────────┘
    FAULT ◀── ESTOP (any state) ────────── STOP ──▶ IDLE
          ◀── STALL (when not IDLE)
          ◀── search failed (all sweep steps or 6 s)
```

The controller decides only *what* the robot does. The Motion and Line
tasks do the steering (line PID) and the encoder-based rotation.

### How it is called

`vc_handle(&vc, event, now_ms)` takes one event word and returns one motion
command word. The Vehicle Controller Task calls it for every event from
`mbf_ctrl_evt`, and with `VC_EVENT_NONE` when the 10 ms mailbox wait times
out. The timeout call runs the auto-start and search timers. The module has
no hardware or RTOS calls.

### Rules

| Rule | Reason |
| --- | --- |
| Auto-start runs once, 5 s after boot | Design review. STOP in IDLE cancels it. It is not armed again after STOP or RESET, so the robot does not start by itself after an operator stops it. |
| ESTOP goes to FAULT from every state | Design review |
| STALL goes to FAULT, but not in IDLE | The wheels do not turn in IDLE, so a stall report there is not real |
| LINE_LOST is already filtered | The Line task sends it after more than 50 ms without the line (statechart) |
| The search sweeps toward the side where the line was last seen first | The line usually leaves on that side. Unknown side: left first. |
| LINE_FOUND wins over the search timeout | A found line is the better result |
| RESET clears all faults and goes to IDLE | "Condition cleared" checks come with the real fault sources |
| A command is sent only when it changes | NONE means "keep doing the current command" |

### Line search

The sweep turns to these heading offsets from where the line was lost. A
positive offset is toward the side where the line was last seen.

| Step | Offset | ROTATE sent (last seen left) |
| ---: | ---: | ---: |
| 1 | +20° | +20° |
| 2 | −20° | −40° |
| 3 | +45° | +65° |
| 4 | −45° | −90° |
| 5 | +90° | +135° |
| 6 | −90° | −180° |

Each ROTATE starts after MOTION_DONE for the previous one. After step 6, or
6 s after LINE_LOST, the search fails: FAULT with the SEARCH bit.

## Data representation

Week 8 Change Requirement 1 applies. All words use explicit masks and
shifts.

Event word (into the controller):

| Bits | Field | Width |
| --- | --- | ---: |
| 0–4 | Event type | 5 |
| 5–13 | Payload | 9 |
| 14–15 | Spare | 2 |

| Code | Event | Payload |
| ---: | --- | --- |
| 0 | NONE (mailbox timeout) | — |
| 1 | START | — |
| 2 | STOP | — |
| 3 | ESTOP | — |
| 4 | RESET | — |
| 5 | LINE_LOST | Last-seen side: 0 unknown, 1 left, 2 right (2 bits) |
| 6 | LINE_FOUND | — |
| 7 | MOTION_DONE | — |
| 8 | STALL | — |

The 5-bit type leaves room for the about 20 events of the full design. The
9-bit payload fits a later ultrasonic distance or PROFILE field.

Motion command word (out of the controller):

| Bits | Field | Width | Values |
| --- | --- | ---: | --- |
| 0–2 | Command type | 3 | NONE, STOP, FOLLOW, ROTATE; 4–7 for FWD, REV, TURN later |
| 3–11 | Parameter | 9 | FOLLOW: 0 cruise, 1 slow. ROTATE: signed degrees, −256 to +255, positive = left |
| 12–15 | Spare | 4 | Always 0 |

Controller state (private to the task):

| Bits | Field | Width |
| --- | --- | ---: |
| 0–3 | State | 4 (13 states in the full design) |
| 4–6 | Search step | 3 (0–6) |
| 7 | Search mirror (sweep right first) | 1 |
| 8 | Auto-start armed | 1 |
| 9–11 | Fault mask: ESTOP, STALL, SEARCH | 3 |
| 12–15 | Spare | 4 |

The time the current state started is a native 32-bit millisecond count,
because it is compared with the RTOS clock. The elapsed-time subtraction
is correct across a clock wrap.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
   tests/test_vehicle_controller.c src/vehicle_controller.c \
   -o build/test_vehicle_controller
./build/test_vehicle_controller
```

It checks 15 cases: START, auto-start once, STOP cancels auto-start, the
search in both directions and with an unknown side, the full sweep, the
search timeout, LINE_FOUND at the timeout, STOP during search, ESTOP from
every state, STALL, ignored events, clock wrap, and the event word layout.

Result on 2026-10-27: all tests pass. Five deliberate rule changes (search
direction, auto-start after a fault, timeout boundary, sweep calculation,
STALL in IDLE) each make the test fail.

## Open items

- Tune the sweep offsets, the 6 s search timeout, and the auto-start delay
  on the track.
- Agree the LINE_LOST payload with the Line task owner.
- Agree the ROTATE sign convention and MOTION_DONE with the Motion task
  owner.
- Next step: choose with the team (INIT and CALIBRATE, or the obstacle
  path).
