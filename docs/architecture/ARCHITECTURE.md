# Current robot architecture

## What it is

The staging robot runs micro T-Kernel 3.0 on Pico W. The Vehicle Controller
runs one navigation module at a time. Sensor tasks publish observations;
modules return wheel-speed requests and outcomes. Motion alone writes motor
and servo PWM. A successful build is not physical acceptance.

This is the implemented system, not the older two-core proposal. The
[branch rules](BRANCH_LAYOUT.md) define the integration boundary. Keep the
three ADRs separate: [actuator ownership](ADR_MOTOR_CONTROLLER.md),
[time-base ownership](ADR_HEARTBEAT.md) and
[module routing](ADR_VEHICLE_CONTROLLER.md).

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
S4 tests. BENCH is exclusive with the vehicle program. Task stacks and
kernel object sizes are in the [kernel record](../kernel/KERNEL.md).

### Interrupt and polling rules

| Work | Implemented method | Rule |
| --- | --- | --- |
| Encoder and echo edges | Core-0 GPIO13 dispatcher, IRQ level 1 | Record counts/timestamps, set state and wake a task |
| ADC sampling | TIMER alarm 0 / IRQ0, level 2; 1 ms absolute targets | Eight conversions per channel, timestamped 128-slot queue |
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
- [Original interrupt record](../archive/architecture/INTERRUPT_CONVENTIONS.md), [shell record](../archive/architecture/VEHICLE_STATE_MACHINE.md), [older control proposal](../archive/architecture/CONTROL_ARCHITECTURE.md) and [multicore plan](../archive/architecture/robo_pico_rtos_multicore_plan.md) retain every earlier number and result.
