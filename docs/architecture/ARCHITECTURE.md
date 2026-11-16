# Current robot architecture

## What it is

The staging robot runs micro T-Kernel 3.0 on Pico W. The Vehicle Controller
runs one navigation module at a time. Sensor tasks publish observations;
modules return wheel-speed requests and outcomes. Motion alone writes motor
and servo PWM. A successful build is not physical acceptance.

This is the implemented system, not the older two-core proposal. The
[branch rules](PROJECT_STATUS.md#branch-layout-and-integration-rules) define the integration boundary. The three
[decision records](#decision-records) stay as whole, separate ADRs:
[actuator ownership](#adr-the-motor-controller-is-the-only-owner-of-the-actuators),
[time-base ownership](#adr-one-hardware-heartbeat-short-work-in-the-interrupt-decisions-in-the-main-loop) and
[module routing](#adr-the-vehicle-controller-runs-one-module-at-a-time-and-maps-outcomes-to-the-next-module).

## How it works

```text
encoder / echo edges -> GPIO dispatcher -> motion / sonar observations
1 ms ADC interrupt -> sample queue -> line service -> calibrated position
10 ms motion tick -> vehicle wake -> one module -> motion request
                                         |
                                         +-> bounded telemetry messages
read-only observation boards -> Wi-Fi task -> MQTT -> laptop dashboard
```

### Subsystems and ownership

| Area | Current responsibility | Limit |
| --- | --- | --- |
| S1 telemetry | RAM log/dump, bounded network queues, MQTT codecs and health | No network call in a control task |
| S2 motion | Speed loop, encoders, motor/servo PWM, stale-command stop | Sole actuator owner; sole watchdog feeder |
| S3 line/barcode | ADC acquisition, pair-mode line learning and observations | Barcode is raw analog only; no robot barcode decoder |
| S4 IMU | Robot/TEST observer: stationary bias, filtered/compensated pitch and relative height | No driving action, terrain events, MQTT/KR1 fields or physical step-2 acceptance |
| S5 obstacle | Asynchronous echo service, monitor, scan and detour modules | Sensor service does not choose a route |
| Vehicle | Calibration/run phases, buttons, module shell and routing | No ADC polling, echo wait, network call or dump loop |

### Tasks and cores

Lower task priority numbers run first. Registered application tasks use
`TP_PRC1`: processor 1 in kernel numbering, physical core 0. The robot
profile uses `SMP=0`. A default SMP build does not move the robot or Wi-Fi
tasks to core 1. The earlier proposed core split is not implemented.

| Task | Priority | Release / role |
| --- | ---: | --- |
| Motion | 10 | 10 ms cyclic; control tick and watchdog feed |
| Line | 11 | Wake for queued 1 ms ADC samples |
| Sonar | 12 | Requests, echo edges and 10 ms deadline checks |
| Vehicle | 13 | Motion after-tick notification, 10 ms |
| IMU, robot/TEST only | 14 | Own 10 ms cyclic, bounded sensor/count reads and observation board |
| Wi-Fi, when enabled | 20 | Sole CYW43/lwIP/MQTT owner; requested 2 ms sleep |
| USB console | 21 | Interrupt wake or 10 ms fallback |
| Telemetry | 30 | Nonblocking producer messages, RAM storage and dump |
| Health, MQTT only | 31 | 1 s wake, bounded board and stack observations |
| Idle meter, when enabled | 32 | Lowest-priority CPU-load reference |

TEST moves its runner to priority 20 after startup. IMU_STREAM runs only
S4 tests, including an explicitly operator-triggered GP21 straight-drive
trial bounded by600 mm/10 s, second press and faults; height never commands
it. VIN and lifted-first stop checks are required. BENCH is exclusive with
the vehicle program. Task stacks and
kernel object sizes are in the [kernel record](../kernel/KERNEL.md).

### Interrupt and polling rules

| Work | Implemented method | Rule |
| --- | --- | --- |
| Encoder and echo edges | Core-0 GPIO13 dispatcher, IRQ level 1 requested, 0 on hardware ([F-NVIC](../kernel/KERNEL.md#interrupt-priorities-f-nvic)) | Record counts/timestamps, set state and wake a task |
| ADC sampling | TIMER alarm 0 / IRQ0, level 2 requested, 0 on hardware ([F-NVIC](../kernel/KERNEL.md#interrupt-priorities-f-nvic)); 1 ms absolute targets | Eight conversions per channel, timestamped 128-slot queue |
| Motion scheduling | Kernel cyclic, 10 ms | Complete one tick; do not replay old releases |
| IMU observation | Separate kernel cyclic, 10 ms, task14 | Bounded bus/count reads; no motion wake-client replacement or driving decision |
| Missing echo / stalled wheel | Bounded deadline or progress check | A missing edge cannot generate its own interrupt |
| GP21 operator button | Poll every 20 ms, then immediate motion ESTOP | Not a hardware ESTOP interrupt |

Keep interrupt work short. Do not print, sleep, allocate memory or use
floating point there. Kernel-facing code owns IRQs, clocks and scheduling;
SDK headers stay in isolated `*_hw.c` register layers. Do not install SDK
runtime or IRQ ownership beside the kernel.

`volatile` alone is not a publication contract. Use the existing short
kernel DI/EI sections for core-local interrupt state, inherited mutexes for
service copies, and the sequence/word observation boards for bounded public
reads. A busy board read returns unavailable; it must not spin. Bare-metal
SDK interrupt-mask examples in the archive are not kernel API instructions.

## Interface

### Vehicle Controller shell

The pure `vsm_*` shell consumes `vsm_inputs_t` observations and returns
`vsm_drive_t`: STOP, or wheel speeds in mm/s plus servo degrees. Glue sends
this to motion. A module has `start(ctx, payload, now_ms)` and
`step(ctx, in, now_ms, drive)`. Detectors do not decide transitions.

| State | Drive | Exit |
| --- | --- | --- |
| IDLE | STOP | `vsm_start()` |
| RUNNING | Current module request | Stop, routed outcome or fault |
| FAULT | STOP | `vsm_reset()`; start/stop do not clear a fault |

The shell checks latched motor faults before stepping a running module.
A completed module can start and step its successor in the same tick.
Three handoffs are allowed; a fourth faults. Missing routes/modules and
FAILED outcomes fault. Each handoff records time, endpoints and outcome.

### Route table

| Running module | Outcome | Next module |
| --- | --- | --- |
| LINE | OBSTACLE | SCAN |
| LINE | LINE_LOST | SEARCH |
| SCAN | SIDE | DETOUR |
| DETOUR | DONE_LINE | LINE |
| DETOUR | DONE_NO_LINE | SEARCH |
| SEARCH | FOUND | LINE |
| SEARCH | OBSTACLE | SCAN |

A route entry does not prove that every module emits that outcome. Current
SEARCH does not run a separate forward obstacle watch. Outcomes use a
16-bit word: type in bits 0–3, payload in bits 4–15, clamped to 0–4095.
Types 0–7 are RUNNING, OBSTACLE, SIDE, DONE_LINE, DONE_NO_LINE, LINE_LOST,
FOUND and FAILED. Other terminal outcomes have no route and fault.

## Settings and safety

| Setting | Current rule |
| --- | --- |
| Navigation/control period | 10 ms; vehicle runs after motion, not the old bare-metal phase-9 schedule |
| Sensor requests | Sonar generation/ID/angle/deadline checked before use; one outstanding one-shot |
| Data publication | Bounded value copies; no sensor event consumed by telemetry |
| Commands | Latest-value motion request with timestamp expiry; no drive backlog |
| Watchdog | 100 ms hardware timeout; fed only after completed motion ticks |
| Network failure | Must not stop or delay driving; application command acceptance is separate from MQTT PUBACK |
| ADC pins | C GP26, L GP27, barcode analog GP28; old digital GP7 unused |
| GP28 supply sensing | Keep the VBAT sense jumper open; VIN/2 must not enter the barcode signal |

Detailed age/deadline limits remain in the kernel and subsystem records.
Never replace a missing echo with a clear path. Keep calibrated values,
physical measurements and model assumptions separate.

## Results

| Evidence | Result and scope |
| --- | --- |
| Shell host checks, 2026-11-04 | Route/fault/handoff tests pass; 36/36 mutations caught |
| M1-S7, 2026-11-06 | Basic line/search run; 32019 samples, no missed/dropped samples, no motion overruns |
| M2-S7, 2026-11-06 | SCAN to DETOUR to SEARCH exercised; 101929 samples, no missed/dropped samples; robot left the tiles, so line reacquisition was not tested |
| S4-U03, image 6ed89ad | Continuous TEST IMU stream passed; not a robot/hump result |
| M2 reattempt, image 48c69d3 | Bounded SCAN reverse/rescan passed; not full detour acceptance |

## Limits and records

No gyro is present in the GY-511. IMU calibration/hump integration and the
analog barcode decoder are not implemented. Physical stop latency and each
new image's cold-power and track checks need operator evidence. Do not
carry a PASS from an earlier image to a new one.

- [Integration procedures and results](../../testing/integration/KERNEL_ROBOT.md).
- [System test log](../../testing/integration/TEST_LOG.md).
- [Line service test log](../../testing/subsystem3/TEST_LOG.md).
- [Ultrasonic service test log](../../testing/subsystem5/TEST_LOG.md).
- [IMU record](../s4-imu/IMU.md) and [S4 log](../../testing/subsystem4/TEST_LOG.md).
- The original interrupt record, shell record, older control proposal and
  multicore plan are in the [architecture history](../archive/architecture/HISTORY.md#files);
  they retain every earlier number and result.

## Decision records

Each ADR keeps its full text, status and date.

### ADR: the motor controller is the only owner of the actuators

| Item | Value |
| --- | --- |
| Status | Accepted, 2026-11-04 (operator) |
| Scope | Motor PWM, servo PWM, wheel encoders |
| Branch for the work | `mtk3-motion` (from `yx`) |

#### Context

In `yx` (2026-11-04) seven files write the motors or the servo directly:
`drive.c`, `line_spin.c`, `detour_test.c`, `steer_test.c`,
`bypass_test.c`, `obstacle_scan.c` and `servo.c`. The motors run open loop:
the callers set PWM duty and the encoders are used only for stall stops,
the heading hint, the junction distance and the detour pose
([Line steering](../s3-line-barcode/LINE.md)).

So each caller must know the robot's hardware quirks:

| Quirk | Where it is handled now | Evidence |
| --- | --- | --- |
| Left and right motors differ | `right_trim_pct` +1 in `line_steer` | [Line steering](../s3-line-barcode/LINE.md) |
| Dead zone: a loaded wheel does not turn below about 12 % | `min_pct` 12 in `line_steer` | Bench run 2026-10-29 |
| Reverse is weaker (left wheel) | Detour sweep minimum 14 % | D19, D27 ([detour handoff](../archive/OBSTACLE_DETOUR_HANDOFF.md)) |
| Battery voltage drop | Not handled | Second-detour stalls at 14–15 % (D31, D33, D35) |
| Encoder signs, 2535 counts per wheel turn | `drive.c` | [Calibration results](../s2-motion/MOTION.md#hardware-wiring-and-calibration) |
| 24.05 counts per degree of turn | `detour_test.c` | Calculated; slip not measured |
| Servo centre and safe range (1100–1900 µs) | `servo.h` | [Servo obstacle scan](../s5-obstacle/OBSTACLE.md) |
| Servo settle time before a ping | `obstacle_scan.c` timing | [Servo obstacle scan](../s5-obstacle/OBSTACLE.md) |

The team Motion module (Buddy 2, branch `motor-line-manager`, now in tag `archive/yx-line-abstraction`,
`src/motion/`, `docs/MOTION_CONTROL.md`) already owns the motors and has
a closed-loop speed control. But it also holds intent: `motion_set_steer()`
(steer mixing), `motion_set_slow()` (a line-following "calm" mode), a
FOLLOW state, and `MOTION_CMD_ROTATE` / `MOTION_CMD_FWD` (it decides when
a move is complete). It does not own the servo.

#### Decision

1. **One owner.** The motor controller is the only module that writes
   motor PWM or servo PWM, or reads the wheel encoders.
2. **Values in, measurements out.** Callers send values; the controller
   executes them and reports measurements.

   | In (commands) | Out (measurements) |
   | --- | --- |
   | Left and right wheel speed, mm/s, signed | Wheel travel (mm) and speed (mm/s) |
   | Servo angle, degrees | Heading change, degrees (from the encoder counts) |
   | Stop; emergency stop | Servo "settled at angle X" |
   | — | Faults: stall, stale command |

3. **No intent in the controller.** It does not mix a steer value, has
   no line-following or "calm" mode, no FOLLOW state, and does not decide
   when a move is complete. Example: the heading change is a measurement
   (controller); "turn until −90°" is intent (detour).
4. **Hardware quirks stay inside the controller.** All items in the
   Context table: left/right difference, dead zone, weak reverse, battery
   voltage drop (closed-loop speed), encoder signs and counts, counts per
   degree, servo centre, limits and settle time.
5. **Protection stays inside the controller.** It is the same in every
   situation, so it is not intent: stall stop, emergency stop, stop when
   no new command arrives for 100 ms, acceleration limit.
6. **Callers stay pure.** Line steering, detour, obstacle scan and search
   use only physical values (mm/s, degrees), with no robot numbers.
7. **Servo "settled" is an estimate.** The servo gives no feedback: the
   controller calculates the settle time from the size of the move and a
   measured settle rate.

#### Consequences

| Area | Change |
| --- | --- |
| Line steering | Output becomes left and right wheel speed. `min_pct`, `max_pct` and `right_trim_pct` go to the controller. Gains must be tuned again |
| Detour | The sweep rate control (duty 14–30 %) becomes a turn-rate request. The sweep stall (D27, D31, D33, D35) is handled by the closed-loop speed control |
| Obstacle scan | Sends an angle and waits for "settled"; no pulse widths or settle timing |
| Search, spin calibration | Send wheel speeds, not duty |
| Host tests and simulator | Callers need no robot numbers; the simulator can replace the controller with a model |
| Accuracy | The caller stops a turn from the measured heading. With a 20 ms controller period, a 45°/s turn can overshoot by about 1° |
| Tuning | A new battery, motor or wheel changes only the controller |

#### Alternatives considered

| Option | Why not chosen |
| --- | --- |
| Callers send duty % (no speed control) | No intent, but every caller must handle the hardware quirks |
| Callers send speed + steer + calm (Motion today) | The controller holds line-following intent |
| Callers send moves (rotate 90°, forward 350 mm) | The controller decides when a move is complete (intent) |

#### Work rules

- One branch for each module. Merge module branches only into `mtk3-staging`.
  Copy needed code from other people's branches by hand; do not merge those branches.
- Code to reuse:

  | Source | What |
  | --- | --- |
  | `origin/motor-line-manager`, `src/motion/` (Buddy 2) | Speed control (feed-forward and correction, 20 ms), stall and emergency stop, stale-command timeout, encoder handling |
  | `yx`, `src/drive.c` | Pins, encoder signs, 20 kHz PWM |
  | `yx`, `src/servo.c`, `src/servo.h` | Servo pulse, safe range |
  | `yx`, motor calibration targets | `motor_duty_calibration.c`, `motor_speed_calibration.c`, `motor_rotation_drift.c` |

- Remove the intent parts when code is copied from Motion
  (`motion_set_steer`, `motion_set_slow`, the FOLLOW state, move completion).

### ADR: one hardware heartbeat; short work in the interrupt, decisions in the main loop

| Item | Value |
| --- | --- |
| Status | Accepted, 2026-11-04 (operator: option B); revised 2026-11-04: owner is a separate platform module (decision 7) |
| Scope | Timing of all modules on core 0 |
| Related | [ADR: motor controller](#adr-the-motor-controller-is-the-only-owner-of-the-actuators), [motor controller](../s2-motion/MOTION.md), [interrupt conventions](#interrupt-and-polling-rules), [control architecture](../archive/architecture/HISTORY.md#files) |

#### Context

- Each test program (`detour_test`, `steer_test`, …) has its own main
  loop with its own timing. All read the same clock, but nothing gives a
  common rhythm.
- `mc_tick()` is the motor controller's heartbeat: commands only store
  values; the tick computes and gives out the duty. If the tick stops,
  the PWM keeps its last duty and the stale stop (inside the tick) cannot
  work.
- Encoder edges and ultrasonic echo timing use interrupts. A long
  interrupt handler makes them wait.

#### Decision

1. **One heartbeat:** a hardware timer interrupt every **1 ms** on core 0.
   Every periodic module runs on a multiple of it.
2. **Short, exact work runs inside the heartbeat interrupt:**
   - line sensor ADC sample (1 ms);
   - `mc_tick()` (every 10 ms) and the PWM write of its output;
   - restart of the hardware watchdog;
   - "due" flags for the main loop.
3. **Decisions run in the main loop**, started by the flags: line
   position, steering, `mc_set_speed()`, detour, obstacle watch, buttons,
   Wi-Fi log, `printf`.
4. **Order in each 10 ms step: sense → decide → act.** The decision for a
   step is ready before the next `mc_tick()`; the delay from a reading to
   the motors is fixed.
5. **Safety:**
   - Main loop hangs: no new `mc_set_speed()`; the heartbeat still runs
     `mc_tick()`; stale stop after 100 ms.
   - Heartbeat stops (chip hangs): the hardware watchdog, restarted only
     by the heartbeat, resets the chip; the PWM pins go to 0. Watchdog
     time: set in step 2 (about 50 ms).
6. **Rules for the interrupt:** no `printf`, Wi-Fi, sleep, or blocking
   call; measure the worst-case handler time; count overruns. Shared
   data between the main loop and the handler (for example
   `mc_set_speed()` against `mc_tick()`) uses a short interrupt-off
   section ([interrupt conventions](#interrupt-and-polling-rules)).

| Every | Where | Work |
| --- | --- | --- |
| 1 ms | Interrupt | Line sensor ADC sample; watchdog restart |
| 10 ms | Interrupt | `mc_tick()`, PWM write |
| 10 ms | Main loop | Line position → steering → `mc_set_speed()`; detour step |
| 20 ms | Main loop | Buttons |
| 50 ms | Main loop | Wi-Fi log row |
| 60 ms | Main loop | Ultrasonic ping start (echo timing stays on its own interrupt) |
| — | Hardware | Servo pulse (50 Hz PWM) |

7. **Owner: a separate platform module, `heartbeat`** (revised
   2026-11-04, operator). It owns the 1 ms timer, the hardware watchdog
   and the "due" flags, and it is not part of any module that it drives.
   The users hook into it: the motor controller's Pico layer registers
   `mc_tick()` for every 10th tick, the line sensors register the ADC
   sample for every tick. Reason: the heartbeat serves every layer
   ([ADR: Vehicle Controller](#adr-the-vehicle-controller-runs-one-module-at-a-time-and-maps-outcomes-to-the-next-module)); if one of
   the driven modules owned it, the others would depend on that module.
8. **Not on the heartbeat:** encoder edges and the ultrasonic echo
   (events, their own interrupts) and the motor and servo pulses
   (hardware PWM).

```text
  heartbeat (1 ms timer, watchdog, flags)          ← platform module
     │ interrupt hooks            │ flags
     ▼                            ▼
  ADC sample, mc_tick       detectors → running module → Vehicle Controller
                                                           → mc_set_speed (used at the next mc_tick)
```

Delay from a reading to the motors: at most one 10 ms frame, and always
the same.

#### Alternatives considered

| Option | Why not chosen |
| --- | --- |
| A. All module work inside the heartbeat interrupt | Exact timing, but encoder and echo interrupts wait during the long handler; no logging; one slow module makes the motors late too |
| No common heartbeat (each program keeps its own loop) | `mc_tick()` stops when the loop hangs; the stale stop cannot work |

#### Consequences

- A new platform module `heartbeat` owns the timer, the watchdog and the
  flags (decision 7). The motor controller's Pico layer (step 2) hooks
  `mc_tick()` into it; it does not own the timer.
- With μT-Kernel (team firmware, `motion_task_utkernel.c`), the RTOS tick
  takes the place of the heartbeat; the split stays the same.
- Core 1 (planned in [control architecture](../archive/architecture/HISTORY.md#files))
  keeps the peripherals that tolerate delay.

### ADR: the Vehicle Controller runs one module at a time and maps outcomes to the next module

| Item | Value |
| --- | --- |
| Status | Accepted, 2026-11-04 (operator) |
| Scope | Vehicle Controller, task modules, shared detectors |
| Related | [ADR: motor controller](#adr-the-motor-controller-is-the-only-owner-of-the-actuators), [ADR: heartbeat](#adr-one-hardware-heartbeat-short-work-in-the-interrupt-decisions-in-the-main-loop), [Vehicle Controller step 1](../archive/VEHICLE_CONTROLLER.md) |
| Branch for the work | `mtk3-vehicle` owns the interface; `mtk3-*` own modules; merge only into `mtk3-staging` |

#### Context

- [Vehicle Controller step 1](../archive/VEHICLE_CONTROLLER.md) is one flat state
  machine (the 13-state list of the Week 8 notes). The controller itself
  holds the details of line following and line search, and its output is
  a motion command word (for example ROTATE). After the
  [motor controller ADR](#adr-the-motor-controller-is-the-only-owner-of-the-actuators), a command such as
  ROTATE is intent in the motor controller.
- `detour_test.c` does the handoffs by hand: forward watch STOP → scan →
  detour → follow, or search if no line. The handoff rules are mixed into
  the test program.
- At the detour handoff, `detour_test.c` must call `lp_reset_tracking()`,
  because the line detector is updated only in some phases.
- Several jobs need the same triggers: line tracing and line search both
  need the obstacle watch; line search and the detour both need "line
  seen".

#### Decision

##### Four layers

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
| Motor controller | As in its [ADR](#adr-the-motor-controller-is-the-only-owner-of-the-actuators) | Intent |

##### Rules

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
   loop on the 10 ms heartbeat flag ([ADR: heartbeat](#adr-one-hardware-heartbeat-short-work-in-the-interrupt-decisions-in-the-main-loop)).

##### Outcome table (first version)

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

##### Module interface (sketch; details in the implementation)

| Call | Use |
| --- | --- |
| `start(payload)` | The Vehicle Controller starts the module with the payload of the last outcome |
| `step(detectors, now)` → result | Every 10 ms. Result: RUNNING or an outcome word (type + payload), plus the wheel speeds (mm/s) and servo angle the module wants |

The outcome word uses the packed event format of the Week 8 work
(type and payload in one 16-bit word,
[Week 8 notes](CR1.md#vehicle-controller-state-week-8-33)).

#### Consequences

- Vehicle Controller step 1 changes shape: the line-following and
  line-search details move into modules; the controller keeps the table
  and the safety stop. The shell is a new module,
  [vehicle state machine](#vehicle-controller-shell) (`vehicle_sm`); step 1
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

#### Alternatives considered

| Option | Why not chosen |
| --- | --- |
| One flat state machine (step 1 shape) | The controller holds every job detail; each new job changes the big machine |
| Modules start the next module themselves | Handoffs are spread over the modules; no single table to read or test |
| The Vehicle Controller interprets all detector reports | The meaning of a report depends on the job (obstacle in the detour, line in the detour); the controller would hold job details again |
| Each module has its own copy of the obstacle and line detection | Duplicate code; the copies can differ |
