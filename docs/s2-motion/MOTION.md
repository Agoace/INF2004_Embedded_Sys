# Motion and motor controller

## What it is

The motor controller is the only owner of motor PWM, servo PWM and wheel
encoder counts. PWM means pulse-width modulation: duty sets the fraction
of each cycle for which a driver output is on. Navigation requests wheel
speed and servo angle, not PWM pins. See the
[ownership decision](../architecture/ARCHITECTURE.md#adr-the-motor-controller-is-the-only-owner-of-the-actuators),
[hardware/calibration](#hardware-wiring-and-calibration), and [kernel tasks](../kernel/KERNEL.md).

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
| GPIO interrupt | Shared core-0 IRQ13 dispatcher; acknowledge edge bits before callbacks; interrupt priority 1 requested, 0 on hardware ([F-NVIC](../kernel/KERNEL.md#interrupt-priorities-f-nvic)) |
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

## Hardware, wiring and calibration

The robot uses a Raspberry Pi Pico W on a Cytron Robo Pico, two encoder
motors, a positional servo, HC-SR04+ sonar, line ADC sensors and a GY-511.
This record separates measured ratios from assumed ground distance.
The [motion settings](#settings) use these values; source and wiring must agree
before a powered test. Historical calibration used Pico SDK2.3.0.

### Pins and interface

| Function | Pico pin | Current connection / owner |
| --- | --- | --- |
| GY-511 SDA/SCL | GP0/GP1 | I2C0, robot/TEST IMU observer; USB console, crash mirror off |
| Sonar echo/trigger | GP2/GP3 | HC-SR04+, sonar service |
| Right/motor 2 encoder A/B | GP4/GP5 | Motion; forward raw count decreases, sign-1 |
| GY-511 DRDY (magnetometer) | GP6 | Connected, unused; not the accelerometer data-ready signal |
| GY-511 INT1 (accelerometer) | GP7 | CTRL_REG3 I1_DRDY1; polled before/after reads, input with weak pull-down; no IRQ |
| Left/motor 1 driver | GP8/GP9 | Forward/reverse PWM |
| Right/motor 2 driver | GP10/GP11 | Forward/reverse PWM |
| Servo | GP15 | Motion owns PWM; positive angle turns left |
| Left/motor 1 encoder A/B | GP16/GP17 | Forward raw count increases, sign+1 |
| Dump / start-ESTOP buttons | GP20/GP21 | Active low with pull-ups; normal vehicle samples20 ms |
| Centre line ADC | GP26/ADC0 | Current two-sensor line service |
| Left line ADC | GP27/ADC1 | Current two-sensor line service |
| Barcode raw ADC | GP28/ADC2 | Sample only; no barcode decoder; VBAT jumper open |

Early three-sensor logs used different L/C/R labels. Do not copy those
labels into the current two-sensor wiring. GP29/ADC3 is not selected.
The GY-511 scan found accelerometer0x19 and magnetometer0x1E. Current
accelerometer TEST bring-up and scale limits are in [IMU.md](../s4-imu/IMU.md);
the raw magnitude is not calibrated gravity. No magnetometer navigation
is implied. Grove 7 exposes GP7 and GP28: GP7 now carries accelerometer
INT1 while GP28 remains the barcode analog input. Do not assign GP7 to
barcode output or enable a GPIO IRQ. Step 2c checks INT1 HIGH before and
LOW after the accel read; that transition is still a pending hardware gate.

### Geometry and distance settings

| Value | Value used / measured | Limit |
| --- | --- | --- |
| Wheel track | 111 mm | Operator measurement2026-10-30 |
| Wheel diameter | 65 mm nominal | Effective loaded rolling diameter not calibrated |
| Servo axis to rear axle | 65 mm | Operator measurement2026-10-30 |
| Maximum width | 160 mm | Includes left barcode-reader mount |
| Overall length | 190 mm | Operator measurement2026-10-30 |
| Sensor face to front | About58 mm | Echo mean57.9 mm, not ruler; short-range reading about 11 mm high |
| Rear axle to front line sensors | 123 mm =65+58 | Derived from uncertain echo offset |
| Front-corner radius | 146.7 mm =sqrt(123²+80²) | Conservative full-width calculation; detour sweep uses 150 mm |
| Counts per wheel revolution | 2535 shared | 50-turn unloaded visual-alignment measurement |
| Counts per metre | 12414 | Derived from 2535 and nominal65 mm; not measured ground travel |
| Differential turn scale | 24.05 counts/degree, constant2405 per degree×100 | 111 mm track; wheel slip not measured |

If the true sensor-front offset is about 47 mm rather than 58 mm, front reach
becomes112 mm and corner radius about 138 mm. Measure with a ruler before
using the old offset as precise geometry. The body without the left reader
mount is narrower than 160 mm. Encoders cannot measure wheel slip or tyre
compression; precise gear ratio does not make precise ground distance.

### Encoder ratio and drift method

Each motor ran alone, wheel lifted, forward50 revolutions at 10% duty,
then coasted and was jogged **forward only** to a wheel/chassis alignment
mark. The encoder interrupt made the stop decision, deliberately300 counts
short of the expected mark. Final absolute count plus signed forward jog,
then magnitude, divided by50 gives counts/revolution. A reverse jog would
introduce gearbox backlash (lost motion at a direction change).

| Result | Motor 1 (left) | Motor 2 (right) |
| --- | ---: | ---: |
| Count at visual alignment | 126750 | 126747 |
| Revolutions | 50 | 50 |
| Counts/revolution | 2535.000 | 2534.940 |
| Shared selected value | 2535 | 2535 |
| Measured unloaded coast at 10% | 27–28 counts | 23 counts |
| Mean revolution time, USB test | 2491 ms | 2433 ms |
| Mean speed in that test | 24.08 RPM | 24.66 RPM |

The0.06-count difference is within the estimated±0.06-count/revolution
precision. The non-multiple-of-four ratio is consistent with an encoder
before a non-integer gearbox; do not assume a round gear ratio.

| Jog / old-method result | Observation |
| --- | --- |
| Fine3-count request | 3..7 actual counts |
| Coarse40-count request | 56..57 actual counts |
| Time-based8..10 ms pulse | Did not move this motor reliably |
| Starting revolution | About60..90 ms slower; wheels warmed and sped up |
| Old single-revolution visual precision | About±20 counts; could not resolve a5-count ratio error |
| Old motor 1/motor 2 ratios | 2540/2539 superseded by2535 |
| Encoder-calibration stop | 2500 counts retained, with2535-35 brake compensation |

The old2540 value was0.197% high, about 2 cm systematic error over10 m.
Coast is per-motor, speed/load dependent, not a new shared correction.
The50-turn tests were unloaded; loaded coast, slip and effective rolling
radius remain unmeasured. The full signed arithmetic, method and older
repeatability evidence are in `MOTOR_ROTATION_DRIFT.md` in the
[motion history](../archive/s2-motion/HISTORY.md#files).

### Servo and sonar

| Setting / observation | Result |
| --- | --- |
| Unmounted servo behavior | Positional, not continuous rotation; 500/1500/2500 µs endpoints/centre, about 180° total; no strain observed in that test |
| Mounted collision points | About1040/1960 µs, roughly41° right/left |
| Current safe clamp / scan | 1100..1900 µs (about±36°); scan±30° |
| Direction | Below1500 µs turns right, above turns left, viewed from behind |
| Module | HC-SR04+ on reverse silkscreen, despite HC-SR04 front marking |
| Logic supply | 3.3 V; echo follows that supply; no level conversion fitted |
| Classic5 V HC-SR04 | Can drive5 V echo; not interchangeable without level conversion; carrier digital input limit3.6 V |
| Early stationary echo | About177 mm; functional observation, not distance calibration |
| Approximate beam | ±20°, operator measured2026-11-07; not a hard angular boundary |

Pulse mapping is about 11.1 µs/degree around1500 µs, positive left. The old
unmounted500..2500 µs test does **not** authorize that range on the mounted
robot. Ultrasound gives approximate presence/range, not obstacle width.
Tilted, narrow or soft targets can fail to return an echo. No echo is not
proof of an empty route. The standard sound-speed constant assumes about
20°C and varies about 0.17%/°C. Current sensor logic and measured optics:
[ultrasonic driver](../s5-obstacle/OBSTACLE.md#ultrasonic-driver-and-service) and
[scan](../s5-obstacle/OBSTACLE.md).

### Motor duty and speed results

These unloaded benches are not the loaded controller tuning. Both wheels
were lifted, approximately20 kHz PWM. The VIN test used5 V; motors/rail
voltage change the result.

| Starting/running check | Recorded result |
| --- | --- |
| USB, 0..5% | No movement |
| USB, 6% | Motor 1 one count, no sustained movement; motor 2 none |
| USB, 7% | Both sustained after about 80 ms |
| 5 V VIN unloaded start | Motor 1 at 6%, about 70 ms to 10 counts; motor 2 at 7%, about 80 ms |
| Old common unloaded policy | 8% minimum; 10% kick to 10 counts, 250 ms limit |
| Current loaded controller | 12% kick; loaded duty map and limits in [Duty map](#duty-map) |

#### Unloaded motor speed at 5 V VIN

The following are unchanged recorded RPM values. They used old ratios2540
(left) and2539 (right): left reads about 0.197% low, right about 0.158% low.
Apply2540/2535 or2539/2535 when comparing; new firmware uses 2535 directly.
A1 s settle and5 s measurement followed a10% start kick.

| Duty % | VIN left RPM | VIN right RPM | Earlier USB left RPM | Earlier USB right RPM |
| ---: | ---: | ---: | ---: | ---: |
| 10 | 26.253 | 26.556 | 23.815 | 24.094 |
| 20 | 53.881 | 54.352 | 49.412 | 49.885 |
| 30 | 81.486 | 81.948 | 74.598 | 75.516 |
| 40 | 109.006 | 109.328 | 99.699 | 100.570 |
| 50 | 136.648 | 136.527 | 124.209 | 125.350 |
| 60 | 163.417 | 163.136 | 147.992 | 149.576 |
| 70 | 190.842 | 188.965 | Not valid | Not valid |
| 80 | 217.029 | 214.298 | Not valid | Not valid |
| 90 | 242.664 | 239.560 | Not valid | Not valid |
| 100 | 260.829 | 257.860 | Not valid | Not valid |

The VIN motors differed by less than 0.5 RPM through 60%; left was about 1..3 RPM
faster above 70%. Earlier USB speeds were about 8.5..10.5% lower over10..60%.
The USB run ended in over-current during70% settling; no70..100% results
exist. These results do not authorize a power configuration or full-duty
loaded test.

### Power record status and limits

**USB_POWER_SAFETY is obsolete by operator decision.** Its old limits,
exceptions and recovery instructions are archived, not current operating
requirements. Do not silently apply its exceptions to a new setup.
The [full obsolete record](../archive/s2-motion/USB_POWER_SAFETY.md) preserves
the reported1.2 A Pi400 budget, SSD incident and unmarked-bank observations.
No replacement current-budget measurement was made for this rewrite.

Hardware facts retained there: carrier input3.6..6 V, unregulated motor rail,
and USB-only motor rail described as VUSB minus0.4 V. Supply voltage affects
every speed table. Clear the mechanism and lift wheels for TEST; use the
operator-approved supply arrangement for the actual hardware.

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
DM1 heartbeat max56 µs, no overruns. B2 held 150 mm/s at 220 permille; this
map gives 22% for about 150 mm/s.

DM1 method (`src/duty_map_bench.c`): both wheels the same duty, forward then
reverse; 23 levels (5–12% by 1, 14–20% by 2, 25–50% by 5, 60–100% by 10).
Start from rest with a 12% kick until 10 counts (at most 250 ms). Settle +
measure: 5–20% 500 + 500 ms; 25–40% 400 + 400; 45–50% 300 + 400; 60–100%
300 + 300. Speed = mean counts per 10 ms tick x 8.055 mm/s (12414 counts/m).
500 ms rest at 0 between drives. Log `build/steer_logs/text_5d51.txt`. The
original record is `DUTY_MAP.md` in the
[motion history](../archive/s2-motion/HISTORY.md#files).

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

## Same-tick retarget (opt-in)

Build switch `MOTION_SAME_TICK=1` (image suffix `_sametick`). Default 0:
the image is byte-identical (hash76 MATCH against `b9a47b5`).

| Item | Default | `MOTION_SAME_TICK=1` |
| --- | --- | --- |
| New target reaches the motor | next tick, about 8 ms after the vehicle step | in the same period, at `motion_set_speed()` |
| Loop delay, line read to motor (10 ms period) | about 10 ms | about 2 ms |
| Control steps per period | 1 | 1 (the retarget redoes the step, it does not add one) |

How it works:

1. The motion task saves the wheel state, then runs `mc_tick()`. If no
   fault is latched and STALE is not set, the window opens.
2. The vehicle task calls `motion_set_speed()`. Under `mtx_motion` it sets
   the target, then `mc_retarget()` restores the saved state, keeps the
   tick's measured speed and the new target, and runs the control step
   again. The result is written to the motor at once.
3. The result equals a tick run with the new target: one rise step and
   one integral step per period. A second set in the same period redoes
   the same step; it does not add a step.

Safety and timing:

- Stop, ESTOP, fault clear and init close the window before they act. They
  use the same mutex as before, so the order against ESTOP does not change.
  A latched fault also blocks the retarget. A stall found by the retarget
  latches and gives duty 0.
- Only a completed motion tick feeds the watchdog. The retarget never feeds.
- The task schedule does not change; the wake, response and overrun
  measurements keep their meaning. The vehicle step gets longer by the
  retarget time (one control step under the mutex).
- `KERNEL_RAM=1` adds [`same_tick.list`](../../tools/kernel_ram/same_tick.list)
  (`motion_set_speed`, `mc_retarget`) only with this switch. The robot
  demo build `KERNEL_RAM=1 MOTION_SAME_TICK=1` gives `RAM_PROOF PASS listed 77`.
- `BENCH_KERNEL=1` rejects the switch. The sim and Webots paths do not use it.

Files: [`motor_ctl_same_tick.h`](../../app_program/subsystem2/motor_ctl_same_tick.h)
(interface), [`motor_ctl_same_tick_impl.h`](../../app_program/subsystem2/motor_ctl_same_tick_impl.h)
and [`motion_same_tick.h`](../../app_program/subsystem2/motion_same_tick.h).
The make rules force-include (`-include`) the last two into `motor_ctl.o`
and `motion.o`, so `motor_ctl.c` and `motion.c` do not change. Tests:
[`test_motor_ctl_sametick.c`](../../tests/test_motor_ctl_sametick.c)
(20000-step comparison against set-then-tick, rise, integral, ESTOP, stop,
stale and stall cases) and `test_motion_task_sametick` (motor write in
`motion_set_speed`, window closed by stop, clear and ESTOP).

Status: built and host-tested. Not run on the robot. SIL reference (on
`mtk3-hil`): with the same-tick model, 0..3 s wheel RMS 19.6 vs 30.1.

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
Other logs: [S3 TEST_LOG](../../testing/subsystem3/TEST_LOG.md) (learned line
calibration; old three-sensor values are not two-sensor data) and
[S5 TEST_LOG](../../testing/subsystem5/TEST_LOG.md) (sonar and mounted scan).
All B1/B2/B3 comparisons, mutation results, older interface/SDK measurements,
historical ADC/line values, ten-cycle drift tables, 50-turn conditions and
the STEP 8 motion timing investigation are in the
[motion history](../archive/s2-motion/HISTORY.md#files).

No new hardware measurement was made in these records. Ground
distance/slip, loaded coast, independent physical stop time and an
externally corrected path remain open.
