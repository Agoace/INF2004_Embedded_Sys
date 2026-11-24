# Team and subsystems

## 1. Owners

| Subsystem | Responsibility | Code |
| --- | --- | --- |
| 1 | Telemetry RAM log, MQTT Wi-Fi-task publication, health31 and laptop protocol | `app_program/subsystem1/` |
| 2 | Motion: motors, servo, encoders and exclusive floor bench | `app_program/subsystem2/` |
| 3 | Line acquisition/filter/calibration and Code 39 barcode reader; LIVE commands in the demo | `app_program/subsystem3/` |
| 4 | GY-511 robot hump report and lift/overturn safety detector; TEST probes | `app_program/subsystem4/` |
| 5 | Ultrasonic request/echo service and pure obstacle modules | `app_program/subsystem5/` |

The template demos are removed. GP19 is free. Keep subsystem init/start
order in `app_main.c`. Shared platform and configuration changes require
team agreement. The current demo is tag `demo-kernel-v2`; the dual-core
split is parked. See [the release record](docs/release/DEMO_KERNEL_V2.md).

## 2. Pins

| Pins | Owner | Use |
| --- | --- | --- |
| GP8 / GP9 | S2 | Left motor forward / reverse PWM |
| GP10 / GP11 | S2 | Right motor forward / reverse PWM |
| GP16 / GP17 | S2 | Left encoder A / B; count sign +1 |
| GP4 / GP5 | S2 | Right encoder A / B; count sign -1 |
| GP15 | S2 | Servo PWM |
| GP20 / GP21 | Vehicle, exclusive S2 bench or S4 TEST stream | Dump/MARK; normal calibrate-start/ESTOP, STREAM bounded drive/stop; active low |
| GP27 / GP26 | S3 | Left / centre line sensor |
| GP28 | S3 | Barcode analog ADC2, Code 39 reader; VBAT jumper open |
| GP2 / GP3 | S5 | Ultrasonic echo / trigger |
| GP23 / GP24 / GP25 / GP29 | Reserved | Pico W radio |
| GP0 / GP1 | S4 robot/TEST | I2C0 SDA / SCL, GY-511; USB only, CRASH_REPORT=0. BENCH has no IMU; disconnect it before a UART demo. |
| GP7 | S4 | Accelerometer INT1, pulled-down input; unused by FIFO reader, no IRQ |
| GP19 | Unclaimed | Old LED demo removed |

## 3. Tasks and peripherals

The UP demo (`SMP=0`) runs all tasks on physical core 0 (`TP_PRC1`).
SMP placement code remains, but the dual-core split is parked.

| Task | Priority | Release / responsibility | Stack bytes |
| --- | ---: | --- | ---: |
| Kernel radio service | 20 | Sole radio/lwIP/MQTT owner; below driving tasks | 8320 with lwIP (8192 + 128 kernel bytes) |
| Kernel USB service | 21 | Template console owner, no application TinyUSB calls; below driving tasks and radio | 2048 UP / 4096 SMP, plus 128 kernel bytes |
| motion_task | 10 | 10 ms cyclic, motor/servo PWM, stale stop and watchdog | 3072 |
| line_task | 11 | Every 1 ms sample wake; raw queue, lp and calibration snapshot | 2048 |
| sonar_task | 12 | Request/edge wakes plus 10 ms deadline cyclic | 2048 |
| vehicle_task | 13 | Motion after-tick wake, navigation, junction actions, IMU safety gate, buttons, commands | 4096 |
| imu_task, robot/TEST | 14 | Separate 10 ms cyclic; FIFO reads, bias/filter, hump and safety boards | 2048 |
| telemetry_task | 30 | MBF receive, RAM log and paced GP20 dump | 4096 |
| health_task, MQTT only | 31 | 1 s cyclic; bounded health board, app-stack scan | 2048 |
| idle_meter, MQTT only | 32 | CPU idle-loop measurement | 768 |
| motion_bench, exclusive BENCH | 11 | Earlier bench; never with vehicle or TEST | 4096 |

The ADC ISR retains eight-conversion means per channel. The line task
processes every timestamped sample. It passes L/C/zero to pure lp logic;
the barcode producer also feeds the Code 39 window reader. S3 owns ADC0/1/2;
the template `adca` device stays unopened and ADC3/GP29 is never selected.
The source requests IRQ0 priority 2 and GPIO IRQ13 priority 1 for
encoders/echo. Hardware observations show IRQ0, IRQ5 (USB) and IRQ13 at 0;
SysTick/PendSV use 3. See [F-NVIC](docs/kernel/KERNEL.md#interrupt-priorities-f-nvic).
Do not enable application ADC/GPIO events on core 1. Motion owns PWM slices 4/5 and servo slice7.

SCAN keeps existing servo-settle logic/settings; motion keeps servo PWM.
The service only pings. This approved difference from plan section7 avoids
changing pure navigation logic or adding a second settle delay. Priority12
allows the sonar service to run before navigation13. Per-sample line wakes
avoid the 10 ms kernel tick granularity. Revision-scoped ISR/task costs
are recorded in [the kernel results](testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06).

## 4. Interfaces

| ID | Owner | Consumer | Contract |
| --- | --- | --- | --- |
| IF-01 | S2 | Vehicle or bench | motion_* inherited-mutex API; exclusive after-tick listener |
| IF-02 | S3 | Vehicle | line_snapshot_read; inherited mutex; raw L/C/raw_bc, lp, validity/time/counters; only LINE age >10 ms stops active motion |
| IF-03 | S5 | Vehicle | sonar_request MBF TMO_POL; correlated result mutex + event flag; WATCH/ONCE/STOP, generation/ID/angle/deadline; ping_sent means real trigger |
| IF-04 | S1 | Vehicle producers/operator | telemetry_send MBF TMO_POL; by-value rows/events/info/dump; lost count on full buffer, never blocks |
| IF-05 | S4 | Vehicle and TEST/operator | Bounded IMU snapshot, hump and safety boards; Vehicle reports humps and stops/refuses starts through its safety gate; TEST MARK remains available |

No sensor hardware access, queue draining or dump loop remains in vehicle;
buttons may stay there. Ultrasonic results use their own deadline/age,
not the LINE 10 ms limit. No echo is unknown, never clear. No application
control task may call radio, lwIP or TinyUSB directly. MQTT runs only in the
existing Wi-Fi task; sensors know no transport. See [MQTT design](docs/s1-telemetry/TELEMETRY.md).

## 5. Objects, measurement and acceptance

The base robot uses six application tasks, including IMU. Motion, line
and sonar use inherited mutexes. Sonar requests use a 256-byte message
buffer and result event flag; telemetry uses a 4096-byte message buffer.
TEST starts five service/motion tasks, neither button task;
BENCH starts only motion/bench and has no IMU task/stack reserve. S4 TEST
adds a temporary 20 ms observer cyclic for its 30 s report; IMU_STREAM=1
keeps it until power-off with 50 Hz samples and acknowledged GP20 MARK.
STREAM alone permits GP21 equal100 mm/s to600 mm/10 s, second press/fault
stop, with checked stop-pending retries. VIN/lifted-first checks required;
the STREAM procedure is separate from robot navigation.
The IMU task owns its persistent 10 ms cyclic and bounded boards. Hump
height is report-only; Vehicle uses the safety board to stop on lift,
overturn or unavailable data and to guard starts. Summary v3 and event
kinds 9/10 report humps and safety. See [IMU](docs/s4-imu/IMU.md) and
[the safety port](docs/s4-imu/IMU_SAFETY_PORT.md).
MQTT adds a health task/cyclic and three bounded event/command MBFs.
Approved shared template changes are listed in [the ledger](docs/architecture/TEMPLATE_CHANGES.md).
Every task loops forever or exits through
`tk_ext_tsk()`, never returns.

Application stacks use TA_USERBUF, 8-byte alignment and a fill pattern;
the base robot reserves 17408 bytes in six regions, including IMU.
MQTT health/idle add 2816 bytes, for 20224; lwIP Wi-Fi adds 8320 bytes,
for 28544. The 4096-byte bench region exists only in BENCH builds.
These sizes come from `app_program/platform/app_task.c`. TEST/GP20 report
high-water/free bytes, including IMU; measured IMU results are in
[the kernel stack record](docs/kernel/KERNEL.md#application-stacks).
Initial budgets are not physical measurements;
require at least 256 untouched bytes after full load/dump. The template USB
stack is outside this report; the registered Wi-Fi stack is included. Vehicle deadline overruns and line/ISR
elapsed-work counters are also reported. The task-split TEST checks,
M1/M2 repeats and CPU/ISR/stack measurements **passed on 2026-11-06** (see
[step 7 results](testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06)).
Independent physical stop latency remains unmeasured. Later full-course
runs are recorded in [the current release](docs/release/DEMO_KERNEL_V2.md);
the early R5 procedure is not a claim of complete acceptance of every gate. See
[task/object and plan-vs-port tables](docs/kernel/KERNEL.md).

## Hardware watchdog ownership

Only completed motion ticks feed the 100 ms hardware watchdog, after PWM
writes and mutex release. Failed clock/task/cyclic startup does not enable
it or driving. Debug halt pauses it. Stale-command stop remains separate.
S2-U05 runs LAST in TEST, uses scratch0/1 for a one-time result, and never
repurposes SDK/bootrom scratch4..7. The earlier watchdog image passed
S2-U05 on 2026-11-06 (99 ms, no repeat) and M1. The task-split TEST image
also passed S2-U05 on both tested revisions; see the step 7 results above.
Read [watchdog procedure](docs/kernel/KERNEL.md).

## STEP 8 interfaces and acceptance

- S2/3/5 publish bounded, non-consuming read-only boards; S5 retains 16
  service results with sequence/miss detection. Wi-Fi never waits on them.
- Vehicle owns its board and is the sole live event producer. Its command
  queues are eight ordinary records plus a separate one-slot ESTOP queue.
- S1 owns MQTT/health, versioned binary packing and paced log transport.
  [Wire layout](docs/s1-telemetry/MQTT_WIRE.md); [broker runbook](docs/s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard).
- Busy reads, network queues and disconnects count losses, never motor or
  vehicle faults. Boot and RAM logging do not require the network.
- Valid configured token is required for every command. Without a valid
  token, only shape-valid ESTOP is accepted; its token field is unchecked.
- Commands apply to NORMAL robot only. TEST/BENCH do not start vehicle.
  GP21 remains the real ESTOP in its normal/bench roles; lift wheels for TEST.
- Earlier STEP 8 MQTT checks ran on 2026-11-07; coverage and limits are in
  [the S1 log](testing/subsystem1/TEST_LOG.md). M1/M2 remain independent of
  MQTT acceptance. Later course runs do not prove all network-loss gates.
