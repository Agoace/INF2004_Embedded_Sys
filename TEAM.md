# Team and subsystems

## 1. Owners

| Subsystem | Responsibility | Code |
| --- | --- | --- |
| 1 | Wi-Fi and telemetry | `app_program/subsystem1/` |
| 2 | Motion: motors, servo, encoders and floor bench | `app_program/subsystem2/` |
| 3 | Line and barcode | `app_program/subsystem3/` |
| 4 | IMU | `app_program/subsystem4/` |
| 5 | Ultrasonic and obstacle | `app_program/subsystem5/` |

The template demo tasks, tests and interfaces are removed. GP19 is free.
Empty subsystems create no objects. Keep the init/start order in `app_main.c`.
Only the owner changes a subsystem. Shared platform and configuration changes
need team agreement.

## 2. Pins

| Pins | Owner | Use |
| --- | --- | --- |
| GP8 / GP9 | S2 | Left motor forward / reverse PWM |
| GP10 / GP11 | S2 | Right motor forward / reverse PWM |
| GP16 / GP17 | S2 | Left encoder A / B; count sign +1 |
| GP4 / GP5 | S2 | Right encoder A / B; count sign -1 |
| GP15 | S2 | Servo PWM |
| GP20 / GP21 | S2 bench | Print saved run / start or emergency stop; pull-up, active low |
| GP27 / GP26 | S3 | Left / centre line sensor |
| GP2 / GP3 | S5 | Ultrasonic echo / trigger |
| GP23 / GP24 / GP25 / GP29 | Reserved | Pico W radio |
| GP0 / GP1 | Console when enabled | UART0 TX / RX; used with CRASH_REPORT=1 or CONSOLE=uart |
| GP19 | Unclaimed | Old LED demo removed |

## 3. Tasks and peripherals

| Task | Priority | Processor | Responsibility |
| --- | --- | --- | --- |
| Kernel USB service | 3 | 1 (core 0) | Console; no application TinyUSB calls |
| Kernel radio service, when enabled | 4 | 1 (core 0) | Radio and network stack |
| `motion_task` on motion branch | 10 | 1 (`TP_PRC1`, core 0) | 10 ms controller, motor and servo PWM |
| `motion_bench` on motion branch | 11 | 1 (`TP_PRC1`, core 0) | Button polling and RAM log; not started in TEST builds |

`motion_task` is the highest-priority application task. `cyc_motion` only
wakes it. GPIO IRQ 13 and all encoder state have one owner: core 0. The
shared dispatcher uses priority 1; USB IRQ 5 keeps reset priority 0.
SysTick and PendSV use priority 3. Do not enable these GPIO events on core 1.

The motion owner uses PWM slices 4 and 5 for motors, slice 7 for the servo.
The kernel ADC driver remains the ADC owner; line sampling is not ported yet.
No application task may call radio, lwIP or TinyUSB functions directly.

## 4. Interfaces

| ID | Owner | Users | Contract |
| --- | --- | --- | --- |
| IF-01 | S2 | Bench and future vehicle task | `motion_*` API; values in, one consistent status out; `mtx_motion` uses priority inheritance |

The old LED/producer/consumer/monitor IF-01 through IF-05 are removed. This
IF-01 is a new interface, not the old sample mailbox.

## 5. Object budget

Motion uses one inherited mutex, one 10 ms cyclic handler, and one task.
The non-TEST bench adds one task. Empty subsystems use no kernel objects.
Kernel limits are in `config/config.h`; the USB and radio services also
use objects. Hardware tests and the floor run must be recorded in
`testing/subsystem2/TEST_LOG.md`; build success is not a hardware result.

## Hardware watchdog ownership

S2 owns the 100 ms hardware watchdog. Only a completed `motion_task` tick
feeds it, after PWM writes and mutex release. Startup order is verified
clock, task, cyclic, watchdog; failed motion init starts no watchdog and
leaves motor PWM zero. Debug halt pauses it. Stale-command stop remains a
separate live-task safety check. Dumps show the cached last reset reason.

S2-U05 is TEST-only, runs last after S1..S5 and intentionally stops feeds.
It resets with wheels lifted, then reports the previous timeout in a later
TEST line on the next boot. Scratch0/1 are reserved for its one-time marker
and elapsed observation; SDK/bootrom scratch4..7 are not repurposed.
S2-U05 and physical reset/stop timing are **Not run**. Read
[the watchdog record](docs/KERNEL_WATCHDOG.md) before this destructive test.
