# Motion and motor controller

## What it is

The motor controller is the only owner of motor PWM, servo PWM and wheel
encoder counts. PWM means pulse-width modulation: duty sets the fraction
of each cycle for which a driver output is on. Navigation requests wheel
speed and servo angle, not PWM pins. See the
[ownership decision](../architecture/ADR_MOTOR_CONTROLLER.md),
[hardware/calibration](HARDWARE.md), and [kernel tasks](../kernel/KERNEL.md).

## How it works

```text
vehicle / exclusive bench -> motion_* API -> inherited mutex
10 ms cyclic wake -> motion10 -> mc_tick(counts, time) -> PWM writes
                                             unlock -> watchdog feed -> client wake
encoder GPIO edges -> signed counts; no controller work in GPIO interrupt
```

The pure [motor_ctl.c](../../app_program/subsystem2/motor_ctl.c) computes
commands. [motion.c](../../app_program/subsystem2/motion.c) owns kernel work;
[drv_motor_hw.c](../../app_program/subsystem2/drv_motor_hw.c) owns hardware.
An inherited mutex serializes ticks and public calls. A computed old duty
cannot overwrite a later ESTOP. Discard/count extra queued wakes rather
than replaying stale ticks. `tick_max_us` includes mutex wait through PWM
write; gap extrema are task start-to-start, not execution duration.

### Interface

All `motion_*` calls are task-context APIs returning `E_OK` or a negative
kernel error. Do not call them from an interrupt.

| Call | Rule |
| --- | --- |
| `motion_init()` / `motion_start()` | Initialize zero PWM and verify time; start task, cyclic, watchdog in order; failure prevents driving |
| `motion_set_speed(l,r)` | Signed mm/s, forward positive; refresh within 100 ms |
| `motion_set_servo(deg,&accepted)` | Positive left; clamp, optional accepted angle; next motor tick writes pulse |
| `motion_stop()` | Immediate zero requested speed and PWM; not latched |
| `motion_estop()` | Immediate zero PWM and latched fault |
| `motion_clear_faults()` | Clear latches but keep wheels stopped until a new command |
| `motion_status(&s)` | Consistent last-tick counts, travel, speed, duty, heading, servo, faults and timing |
| `motion_snapshot_try_read()` | One bounded read-only observation; no control mutex; see [boards](../s1-telemetry/TELEMETRY.md#interfaces-boards-and-queues) |
| `motion_notify_after_tick(tid)` | One client; positive ID registers, zero disables; keep client alive until disabled |

The underlying pure API is `mc_init`, `mc_set_speed`, `mc_set_servo`,
`mc_stop`, `mc_estop`, `mc_clear_faults`, `mc_tick` and measurement getters
in [motor_ctl.h](../../app_program/subsystem2/motor_ctl.h).

### Settings

[motor_ctl_robot.h](../../app_program/subsystem2/motor_ctl_robot.h) is the
current configuration source. The loop adds an estimated duty for the
requested speed (feed-forward), a proportional correction (P), and an
accumulated error correction (I).

| Setting | Current value / rule |
| --- | --- |
| Tick | 10 ms |
| Distance | 12414 counts/metre, from 2535 counts/revolution and nominal 65 mm diameter |
| Heading | `(right-left)` counts; 24.05 counts/degree, 111 mm track; left positive |
| Feed-forward, L forward/reverse | Offset 48/59 permille; slope 9.66/8.89 permille per count/tick |
| Feed-forward, R forward/reverse | Offset 56/62; slope 9.39/8.67, same units |
| P / I | 6.0 /0.5 per tick; I limited to ±400 permille |
| I hold | While saturated in the error direction, while set point rises, or during startup kick |
| Rise/fall | Rise at most 8 counts/tick per tick (about 6400 mm/s²); fall at once |
| Start/direction-change kick | At least 12% until 10 counts; no movement by 250 ms latches stall |
| Stall | Duty at least 15%, no count for 500 ms |
| Duty / speed limit | 80% / ±460 mm/s |
| Stale command | Stop at 100 ms without refresh |
| Fault bits | Left stall1, right stall2, ESTOP4 (latched); stale8 |
| Servo | Centre 1500 µs; 1100..1900 µs, ±36°; about 11.1 µs/degree |
| Servo settle estimate | 42 ms +3.6 ms/degree; fitted to 150 ms for30°, 60 ms for5° |

The 12% current loaded kick supersedes the old unloaded 10% bench policy.
A zero `max_speed_mm_s` in the pure API means no speed clamp; the robot
configuration deliberately uses 460. Stop is **coast**, not active braking.
At 20 kHz, PWM compare changes apply within one 50 µs wrap; this is not a
measurement of physical wheel stop time.

## Hardware and time boundary

| Item | Current owner / setting |
| --- | --- |
| Motor PWM | Slices4/5, divider1, wrap6249 at 125 MHz =20 kHz; clear opposite pin before duty |
| Servo | GP15, slice7B, divider125, wrap19999 =50 Hz and1 µs/count |
| Encoders | Both edges, no pulls; left GP16/17 sign+1, right GP4/5 sign-1 |
| GPIO interrupt | Shared core-0 IRQ13 dispatcher; acknowledge edge bits before callbacks; interrupt priority1 |
| Time | `drv_time_us()` reads TIMER; millisecond read uses a stable high/low pair, not truncated microseconds |
| Startup | Kernel selects the 1 µs watchdog tick before devices/radio/tasks; application verification is idempotent |
| Clock checks | Production error limit2%; cold-power S0-U01 limit1%; warm boot alone is insufficient |

Kernel-facing files contain no SDK headers. SDK/register adapters contain
no kernel headers. The motor owner releases PWM reset once before clients
start, including when kernel `USE_PTMR=0`. It does not reset TIMER or take
SDK IRQ ownership. Full startup/watchdog contract: [KERNEL.md](../kernel/KERNEL.md).

### Heartbeat

The **normal kernel robot** uses a 10 ms cyclic wake and motion-only
watchdog feeding; it does not call the old heartbeat interrupt to drive
motors. Do not confuse that design with the separate CMake benches.

The retained bare-metal bench library [heartbeat.c](../../src/heartbeat.c)
is a 1 ms time base. It supports four interrupt hooks and eight main-loop
flags, fixed before start. Hooks run before flags; period/phase spreads work.
`hb_take()` returns a wrapping 16-bit posted-minus-taken count. Its Pico
adapter restarts its watchdog in the heartbeat interrupt: a main-loop hang
there does not reset while interrupts continue. This **bench-only** rule
must not be used to describe the kernel watchdog.

| Bench counter | Meaning |
| --- | --- |
| `gap_overruns` | Start gap >1500 µs, excluding first tick |
| `isr_overruns` | Interrupt work >=1000 µs |
| `isr_max_us` | Longest measured interrupt work |
| `missed_frames` | Extra posts found by a late flag consumer |

## Duty map

The loaded floor map converts old duty settings to an initial speed request;
it is not closed-loop accuracy or a guarantee at another voltage/load.
DM1: 2026-11-04, VIN, boot `5d51`, revision `7a65c0e`. All 23 levels ran
without stall; 6524/6541 rows arrived. The original printed summary overflowed
from 80%; these values come from the 10 ms rows, not that summary.

| Duty % | Forward L/R mm/s | Reverse L/R mm/s | Mean magnitude mm/s |
| ---: | --- | --- | ---: |
| 5 | 0/0 | 0/0 | 0 |
| 6 | 0/0 | 0/0 | 0 |
| 7 | 0/0 | -4/-1 | 1 |
| 8 | 24/24 | -29/-26 | 26 |
| 9 | 34/33 | -39/-36 | 36 |
| 10 | 43/42 | -48/-45 | 45 |
| 11 | 52/51 | -56/-53 | 53 |
| 12 | 60/59 | -65/-63 | 62 |
| 14 | 78/77 | -82/-80 | 79 |
| 16 | 96/95 | -98/-99 | 97 |
| 18 | 114/114 | -116/-117 | 115 |
| 20 | 129/129 | -131/-134 | 131 |
| 25 | 168/171 | -171/-175 | 171 |
| 30 | 211/214 | -214/-217 | 214 |
| 35 | 253/257 | -256/-262 | 257 |
| 40 | 296/299 | -299/-304 | 300 |
| 45 | 332/334 | -335/-342 | 336 |
| 50 | 376/378 | -379/-380 | 378 |
| 60 | 464/463 | -463/-462 | 463 |
| 70 | 561/554 | -548/-543 | 552 |
| 80 | 654/637 | -617/-618 | 632 |
| 90 | 730/706 | -690/-697 | 706 |
| 100 | 807/781 | -759/-734 | 770 |

Do not tune from 80..100%: wheelies occurred, and90..100% speed still rose
within the300 ms window. Running minimum was8% after a12% start kick;
5..6% stopped and7% nearly stopped. The8..70% region was approximately
8.5 mm/s per duty percent, with wheels/directions within about 3%.
DM1 heartbeat max56 µs, no overruns. The source method/settle windows are
preserved in the [full map](../archive/s2-motion/DUTY_MAP.md).

## Results

B3 is the current rise-limit setting, measured2026-11-04 on VIN, boot `ede2`.
All 17 steps completed without fault; rows7..15 were lost over Wi-Fi.
Overshoot uses a50 ms average; ripple is speed standard deviation (sd).

| B3 step | Overshoot L/R % | Time to 90% L/R ms | Ripple L/R mm/s | Heading change |
| --- | --- | --- | --- | --- |
| Forward150 | 6/11 | 140/130 | 2.5/2.4 | +0.5° |
| Reverse150 | 9/12 | 180/170 | 2.6/3.1 | -1.3° |
| Forward300 | 10/9 | 160/150 | 5.0/4.5 | +0.2° |
| Reverse300 | 8/9 | 200/180 | 7.5/7.7 | -2.2° |
| Spin left 20 | 44/36 | 70/50 | About3 | +59.0° |
| Spin right 20 | 100/116 | 50/50 | About3 | -58.9° |
| Forward40 | 8/24 | 210/70 | 1.4/1.8 | +1.3° |
| Reverse40 | 12/24 | 190/60 | About2 | -1.6° |

| Other recorded check | Result / limit |
| --- | --- |
| B3 timing | Controller hook max85 µs; heartbeat max99 µs; no overruns |
| B2 tuning, kept except rise limit | 150/300 mm/s overshoot4..8%; straight heading within 0.7°; slow-start kick accepted; full comparison in archive |
| Kernel B4 cold VIN | 2110 rows, faults0, gaps9952..10046 µs; functional pass; strict B3 speed/heading comparison missed on slow steps |
| S2-U04 | Operator saw immediate stop and about 3 s stopped; fault4 latched, gaps9961..10031 µs, late/missed0 |
| Bare-metal heartbeat bench, 45 s | 1 ms/10 ms hook gaps1000/10000 µs; work max30 µs; overruns/missed0; main-loop hang produced450 missed flags, not reset |

Measured rows are in [S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md).
The old heartbeat GP21 test observed a watchdog reboot within 2 s of log time;
it did not independently measure its50 ms request. Do not substitute it for
the kernel S2-U05 result.

## SDK hardware subset

The app-owned [app_sdk_hw.mk](../../build_make/app_sdk_hw.mk) is the current
exhaustive include/source list, invoked by the
[application hook](../../build_make/mtkernel_3/app_program/subdir.mk).
Selected runtime-free GPIO/PWM/reset/ADC/tick/watchdog operations are allowed.
ADC, ticks and watchdog source files are compiled; radio-off adds GPIO,
radio builds reuse the template GPIO object. PWM/reset operations are inline.
The robot/TEST IMU observer uses I2C0 registers and the I2C header path, not SDK I2C
runtime; TEST links libm for tilt calculation.

Do not add SDK crt0, stdlib/runtime initialization, IRQ registration, time,
sync, multicore or clock runtime sources. Do not call SDK GPIO IRQ, timer
alarm, exclusive-handler or spinlock APIs. The kernel owns interrupts and
synchronization. SDK headers are `-isystem`; application strict warnings
remain errors. Link-only proof and command:
[BUILD.md](../kernel/BUILD.md#host-tests-and-link-proof).

## Limits, tests and full evidence

Wheel slip, tyre compression and startup kick affect speed/heading. No
encoder estimate is a ground-truth position. The accepted slow-turn kick
is not a precise low-speed transient. No active brake, burst-drive scheme,
new steering loop or alternative timer-ISR controller is implemented.

Host tests cover the pure loop, feed-forward/P/I hold, stale/stall boundaries,
servo limits, task ordering and immediate ESTOP. They do not prove physical
stopping or interrupt latency. Use [BUILD.md](../kernel/BUILD.md#host-tests-and-link-proof),
[S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md) and
[integration TEST_LOG](../../testing/integration/TEST_LOG.md).
All B1/B2/B3 comparisons, mutation results, older interface/SDK measurements
and ideas remain reachable in the
[motor archive](../archive/s2-motion/MOTOR_CONTROLLER.md),
[heartbeat archive](../archive/s2-motion/HEARTBEAT.md) and
[duty-map archive](../archive/s2-motion/DUTY_MAP.md).
