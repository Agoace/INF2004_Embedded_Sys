# ADR: the Vehicle Controller runs one module at a time and maps outcomes to the next module

| Item | Value |
| --- | --- |
| Status | Accepted, 2026-11-04 (operator) |
| Scope | Vehicle Controller, task modules, shared detectors |
| Related | [ADR: motor controller](ADR_MOTOR_CONTROLLER.md), [ADR: heartbeat](ADR_HEARTBEAT.md), [Vehicle Controller step 1](VEHICLE_CONTROLLER.md) |
| Branch for the work | `yx-vehicle-controller` owns the interface; `yx-mod-*` own modules; merge only into `yx-staging` |

## Context

- [Vehicle Controller step 1](VEHICLE_CONTROLLER.md) is one flat state
  machine (the 13-state list of the Week 8 notes). The controller itself
  holds the details of line following and line search, and its output is
  a motion command word (for example ROTATE). After the
  [motor controller ADR](ADR_MOTOR_CONTROLLER.md), a command such as
  ROTATE is intent in the motor controller.
- `detour_test.c` does the handoffs by hand: forward watch STOP → scan →
  detour → follow, or search if no line. The handoff rules are mixed into
  the test program.
- At the detour handoff, `detour_test.c` must call `lp_reset_tracking()`,
  because the line detector is updated only in some phases.
- Several jobs need the same triggers: line tracing and line search both
  need the obstacle watch; line search and the detour both need "line
  seen".

## Decision

### Four layers

```text
 sensors ──► SHARED DETECTORS        sense and report only; run all the time
              obstacle watch · line detector · obstacle profile · motor faults
                     │  latest values and events
                     ▼
              RUNNING MODULE         one at a time; does its job; interprets the
              (line tracing, search,  detectors for its job; returns an outcome
               scan, detour, …)       + payload, and the wheel speeds and servo
                     │                angle it wants
                     ▼
              VEHICLE CONTROLLER     outcome → table → next module; the only caller
                     │                of the motor controller; acts on safety
                     ▼                faults directly
              MOTOR CONTROLLER       executes speeds and servo angle; measures; protects
```

| Layer | Does | Does not |
| --- | --- | --- |
| Shared detector | Senses and reports (for example "obstacle at 230 mm", "line lost, last seen right"). Runs on the heartbeat all the time, so its data is current when a new module starts | Decide what a reading means; start a module |
| Running module | Its job. Reads the detectors and decides if a reading ends its job. Returns an outcome with a payload, and the wheel speeds and servo angle it wants | Start another module; call the motor controller |
| Vehicle Controller | Starts the first module; maps each outcome to the next module through a table; passes the payload on; sends the module's speeds and servo angle to the motor controller; stops on safety faults | Hold job details (line search, detour steps) |
| Motor controller | As in its [ADR](ADR_MOTOR_CONTROLLER.md) | Intent |

### Rules

1. **Detectors never decide.** They report values and events only.
2. **The running module interprets the detectors** for its own job. The
   same reading can end one job and be normal in another:

   | Detector report | Line tracing | Line search | Detour |
   | --- | --- | --- | --- |
   | Obstacle STOP | Ends the job: OBSTACLE | Ends the job: OBSTACLE | Normal (the box is near on purpose) |
   | Line seen | Normal input | Ends the job: FOUND | Counts only on the return (`line_return_only`, run D20) |
   | Line lost | Ends the job: LINE_LOST | Normal input | Not used |

3. **A module that needs a different detector setting asks the detector**
   for it (for example a nearer obstacle limit); it does not detect again
   itself.
4. **Modules never start each other.** They return an outcome; the
   Vehicle Controller chooses the next module.
5. **Safety faults go to the Vehicle Controller directly** (stall,
   emergency stop, stale command). They mean "stop" in every module; no
   module interprets them.
6. **The Vehicle Controller is the only caller of the motor controller**
   (`mc_set_speed()`, `mc_set_servo()`).
7. **Timing:** the Vehicle Controller steps the running module in the main
   loop on the 10 ms heartbeat flag ([ADR: heartbeat](ADR_HEARTBEAT.md)).

### Outcome table (first version)

| Running | Outcome (payload) | Next module (input) |
| --- | --- | --- |
| Line tracing | OBSTACLE (distance mm) | Scan |
| Scan | SIDE (left or right, nearest mm) | Detour (side, distance) |
| Detour | DONE, line seen | Line tracing |
| Detour | DONE, no line (side to search) | Line search (side) |
| Line tracing | LINE_LOST (last side) | Line search (side) |
| Line search | FOUND | Line tracing |
| Line search | OBSTACLE (distance mm) | Scan |
| Any | FAILED (reason) | Stop (FAULT) |
| Any (safety fault) | — | Stop (FAULT) |

### Module interface (sketch; details in the implementation)

| Call | Use |
| --- | --- |
| `start(payload)` | The Vehicle Controller starts the module with the payload of the last outcome |
| `step(detectors, now)` → result | Every 10 ms. Result: RUNNING or an outcome word (type + payload), plus the wheel speeds (mm/s) and servo angle the module wants |

The outcome word uses the packed event format of the Week 8 work
(type and payload in one 16-bit word,
[Week 8 notes](week8_change_requirement_1_notes.md#33-vehicle-controller-state)).

## Consequences

- Vehicle Controller step 1 changes shape: the line-following and
  line-search details move into modules; the controller keeps the table
  and the safety stop. The shell is a new module,
  [vehicle state machine](VEHICLE_STATE_MACHINE.md) (`vehicle_sm`); step 1
  (`vehicle_controller.[ch]`) stays because the line detector uses its
  event word. Its motion command word output (ROTATE and others)
  is replaced by wheel speeds from the running module.
- The handoff logic in `detour_test.c` moves into the table and becomes
  host-testable.
- The line detector runs all the time, so the handoff reset
  (`lp_reset_tracking()` in `detour_test.c`) is checked again when the
  modules are built.
- A new job is a new module and new rows in the table.
- Existing shared detectors: `obstacle.c` (forward watch),
  `line_position.c` (line detector), `obstacle_scan.c` and
  `obstacle_profile.c` (profile), `motor_ctl.c` (motor faults).

## Alternatives considered

| Option | Why not chosen |
| --- | --- |
| One flat state machine (step 1 shape) | The controller holds every job detail; each new job changes the big machine |
| Modules start the next module themselves | Handoffs are spread over the modules; no single table to read or test |
| The Vehicle Controller interprets all detector reports | The meaning of a report depends on the job (obstacle in the detour, line in the detour); the controller would hold job details again |
| Each module has its own copy of the obstacle and line detection | Duplicate code; the copies can differ |
