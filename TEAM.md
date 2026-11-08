# Team and subsystems

## 1. Owners

| Subsystem | Responsibility | Code |
| --- | --- | --- |
| 1 | Telemetry RAM log, MQTT Wi-Fi-task publication, health31 and laptop protocol | `app_program/subsystem1/` |
| 2 | Motion: motors, servo, encoders and exclusive floor bench | `app_program/subsystem2/` |
| 3 | Line acquisition/filter/calibration snapshot and sampled-only barcode raw | `app_program/subsystem3/` |
| 4 | TEST-only GY-511 raw probe; no robot integration | `app_program/subsystem4/` |
| 5 | Ultrasonic request/echo service and pure obstacle modules | `app_program/subsystem5/` |

The template demos are removed. GP19 is free. Keep subsystem init/start
order in `app_main.c`. Shared platform and configuration changes require
team agreement. STEP 7 keeps pure modules/settings unchanged.

## 2. Pins

| Pins | Owner | Use |
| --- | --- | --- |
| GP8 / GP9 | S2 | Left motor forward / reverse PWM |
| GP10 / GP11 | S2 | Right motor forward / reverse PWM |
| GP16 / GP17 | S2 | Left encoder A / B; count sign +1 |
| GP4 / GP5 | S2 | Right encoder A / B; count sign -1 |
| GP15 | S2 | Servo PWM |
| GP20 / GP21 | Vehicle or exclusive S2 bench | Dump request / calibrate-start or ESTOP; pull-up, active low |
| GP27 / GP26 | S3 | Left / centre line sensor |
| GP28 | S3 | Barcode analog ADC2, sampled only; VBAT jumper open |
| GP2 / GP3 | S5 | Ultrasonic echo / trigger |
| GP23 / GP24 / GP25 / GP29 | Reserved | Pico W radio |
| GP0 / GP1 | S4 TEST | I2C0 SDA / SCL, GY-511; requires USB only, CRASH_REPORT=0. Disconnect IMU before a UART demo. |
| GP19 | Unclaimed | Old LED demo removed |

## 3. Tasks and peripherals

All application tasks run on physical core 0 (`TP_PRC1`).

| Task | Priority | Release / responsibility | Stack bytes |
| --- | ---: | --- | ---: |
| Kernel radio service | 20 | Sole radio/lwIP/MQTT owner; below driving tasks | 8192 with lwIP |
| Kernel USB service | 21 | Template console owner, no application TinyUSB calls; below driving tasks and radio | Template |
| motion_task | 10 | 10 ms cyclic, motor/servo PWM, stale stop and watchdog | 3072 |
| line_task | 11 | Every 1 ms sample wake; raw queue, lp and calibration snapshot | 2048 |
| sonar_task | 12 | Request/edge wakes plus 10 ms deadline cyclic | 2048 |
| vehicle_task | 13 | Motion after-tick wake, modules, spin behaviour, buttons, commands | 4096 |
| telemetry_task | 30 | MBF receive, RAM log and paced GP20 dump | 4096 |
| health_task, MQTT only | 31 | 1 s cyclic; bounded health board, app-stack scan | 2048 |
| motion_bench, exclusive BENCH | 11 | Earlier bench; never with vehicle or TEST | 4096 |

The ADC ISR retains eight-conversion means per channel. The line task
processes every timestamped sample. It passes L/C/zero to pure lp logic;
barcode raw is a separate observation, with no decoder. S3 owns ADC0/1/2;
the template `adca` device stays unopened and ADC3/GP29 is never selected.
ADC IRQ0 uses priority 2. GPIO13 uses priority 1 for encoders/echo; USB IRQ5
keeps priority 0, SysTick/PendSV priority 3. Do not enable application
ADC/GPIO events on core 1. Motion owns PWM slices 4/5 and servo slice7.

SCAN keeps existing servo-settle logic/settings; motion keeps servo PWM.
The service only pings. This approved difference from plan section7 avoids
changing pure navigation logic or adding a second settle delay. Priority12
allows the sonar service to run before navigation13. Per-sample line wakes
avoid the 10 ms kernel tick granularity; measured CPU cost is still owed.

## 4. Interfaces

| ID | Owner | Consumer | Contract |
| --- | --- | --- | --- |
| IF-01 | S2 | Vehicle or bench | motion_* inherited-mutex API; exclusive after-tick listener |
| IF-02 | S3 | Vehicle | line_snapshot_read; inherited mutex; raw L/C/raw_bc, lp, validity/time/counters; only LINE age >10 ms stops active motion |
| IF-03 | S5 | Vehicle | sonar_request MBF TMO_POL; correlated result mutex + event flag; WATCH/ONCE/STOP, generation/ID/angle/deadline; ping_sent means real trigger |
| IF-04 | S1 | Vehicle producers/operator | telemetry_send MBF TMO_POL; by-value rows/events/info/dump; lost count on full buffer, never blocks |

No sensor hardware access, queue draining or dump loop remains in vehicle;
buttons may stay there. Ultrasonic results use their own deadline/age,
not the LINE 10 ms limit. No echo is unknown, never clear. No application
control task may call radio, lwIP or TinyUSB directly. MQTT runs only in the
existing Wi-Fi task; sensors know no transport. See [MQTT design](docs/s1-telemetry/KERNEL_MQTT.md).

## 5. Objects, measurement and acceptance

Robot uses five application tasks, three inherited mutexes, two cyclic
handlers, two message buffers (256/4096 bytes) and one result event flag.
TEST starts four service/motion tasks, neither button task; BENCH starts
only motion/bench. S4 TEST creates one temporary 10 ms cyclic for its 30 s
raw IMU check, then deletes it; IMU_STREAM=1 keeps it until power-off and
uses GP20 for debounced MARK lines. No robot task or object is added.
Existing kernel limits suffice. See [IMU bring-up](docs/s4-imu/IMU.md).
MQTT adds a health task/cyclic and three bounded event/command MBFs.
Approved shared template changes are listed in [the ledger](docs/architecture/TEMPLATE_CHANGES.md).
Every task loops forever or exits through
`tk_ext_tsk()`, never returns.

Application stacks use TA_USERBUF, 8-byte alignment and a fill pattern;
19456 bytes reserve six regions without MQTT; health adds 2048, for 21504. TEST/GP20 report high-water/free
bytes, including motion. Initial budgets are not physical measurements;
require at least 256 untouched bytes after full load/dump. Template service
stacks are outside this report. Vehicle deadline overruns and line/ISR
elapsed-work counters are also reported. The task-split TEST checks,
M1/M2 repeats and CPU/ISR/stack measurements **passed on 2026-11-06** (see
[step 7 results](testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06)).
Physical stop latency and the full track (R5) are not run. See
[task/object and plan-vs-port tables](docs/kernel/KERNEL.md).

## Hardware watchdog ownership

Only completed motion ticks feed the 100 ms hardware watchdog, after PWM
writes and mutex release. Failed clock/task/cyclic startup does not enable
it or driving. Debug halt pauses it. Stale-command stop remains separate.
S2-U05 runs LAST in TEST, uses scratch0/1 for a one-time result, and never
repurposes SDK/bootrom scratch4..7. The earlier watchdog image passed
S2-U05 on 2026-11-06 (99 ms, no repeat) and M1. This task-split image must
repeat the checks. Read [watchdog procedure](docs/kernel/KERNEL.md).

## STEP 8 interfaces and acceptance

- S2/3/5 publish bounded, non-consuming read-only boards; S5 retains 16
  service results with sequence/miss detection. Wi-Fi never waits on them.
- Vehicle owns its board and is the sole live event producer. Its command
  queues are eight ordinary records plus a separate one-slot ESTOP queue.
- S1 owns MQTT/health, binary v1 packing and paced log transport.
  [Wire layout](docs/s1-telemetry/MQTT_WIRE.md); [broker runbook](docs/s1-telemetry/MQTT_RUNBOOK.md).
- Busy reads, network queues and disconnects count losses, never motor or
  vehicle faults. Boot and RAM logging do not require the network.
- Valid configured token is required for every command. Without a valid
  token, only shape-valid ESTOP is accepted; its token field is unchecked.
- Commands apply to NORMAL robot only. TEST/BENCH do not start vehicle.
  GP21 remains the real ESTOP in its normal/bench roles; lift wheels for TEST.
- S1-U04/U05/U06, S2-U01 Wi-Fi20 and M1/M2 with MQTT are **Not run** on the
  STEP 8 image. M1/M2 are independent of MQTT acceptance.
