# Motor controller

The only owner of the motor PWM, the servo PWM and the wheel encoders. The
decision and its reasons: [ADR: motor controller](ADR_MOTOR_CONTROLLER.md).
Branch: `yx-motor-controller`.

## Status

| Step | What | State |
| --- | --- | --- |
| 1 | Pure core [`src/motor_ctl.c`](../src/motor_ctl.c), [`src/motor_ctl.h`](../src/motor_ctl.h); robot settings [`src/motor_ctl_robot.h`](../src/motor_ctl_robot.h); host test [`tests/test_motor_ctl.c`](../tests/test_motor_ctl.c) | Done 2026-11-04 |
| 2 | Pico layer [`src/motor_ctl_pico.c`](../src/motor_ctl_pico.c), [`src/motor_ctl_pico.h`](../src/motor_ctl_pico.h): PWM, encoder interrupt, servo; hooks `mc_tick()` into the [heartbeat](HEARTBEAT.md) every 10 ms (the heartbeat owns the timer and the watchdog: [ADR: heartbeat](ADR_HEARTBEAT.md)). CMake library `motor_ctl_pico` | Done 2026-11-04: strict ARM compile clean; not yet run on hardware (step 3) |
| 3 | Floor bench target `motor_ctl_bench` ([`src/motor_ctl_bench.c`](../src/motor_ctl_bench.c)): step response, both directions, slow turn; tune P, I and feed-forward | In progress: built, not run |
| 4 | Callers move to wheel speeds (line steering, detour, scan, search) | Not started; one branch each |

## Interface

| Call | Use |
| --- | --- |
| `mc_init(mc, cfg, counts, now)` | Start; travel and heading count from here |
| `mc_set_speed(mc, left_mm_s, right_mm_s, now)` | Wheel speeds, signed, forward positive. Send again within 100 ms, or the wheels stop (stale) |
| `mc_set_servo(mc, deg, now)` | Servo angle, positive = left; clamped to the safe range; returns the angle set |
| `mc_stop(mc)` | Duty 0 at once, not latched |
| `mc_estop(mc)` / `mc_clear_faults(mc)` | Latched stop; clear (the wheels stay stopped until a new speed) |
| `mc_tick(mc, counts, now, &out)` | Every 10 ms: encoder counts in; motor duty (permille) and servo pulse (µs) out |
| `mc_travel_mm`, `mc_speed_mm_s`, `mc_heading_cdeg` | Measurements: travel, speed, heading change (0.01°, left positive) |
| `mc_servo_settled(mc, now)`, `mc_servo_deg(mc)` | Servo settle estimate and angle |
| `mc_faults(mc)` | Bits: stall left 0x01, stall right 0x02, emergency stop 0x04 (latched); stale 0x08 |

## Speed loop

Adapted from Motion (Buddy 2), `origin/motor-line-manager`,
`src/motion/motion_control.c`, `wheel_control()`. Intent parts not copied:
steer mixing, calm mode, FOLLOW state, move completion, command queue.

| Part | Rule | Setting ([`motor_ctl_robot.h`](../src/motor_ctl_robot.h)) |
| --- | --- | --- |
| Feed-forward | offset + slope × set point, **per wheel and per direction** | Offset 85 ‰ (right reverse 90); slope 9.0 ‰ per count/tick (left reverse 12.0, right reverse 9.4). Rough fit to the track logs of 2026-11-03/04 |
| P | kp × (set point − measured) | 6.0 ‰ per count/tick (Motion 3.0 at 20 ms, scaled) |
| I | ki × error each tick; not while the output is at the limit in the error's direction; limit ±400 ‰ | 0.5 per tick |
| Rise limit | Set point rises at most 2 counts/tick per tick (about 1600 mm/s²); falls at once | — |
| Start kick | From rest or on a direction change: at least 12 % until 10 counts; none in 250 ms = stall | Start policy in [calibration results](CALIBRATION_RESULTS.md) |
| Stall | Duty ≥ 15 % and no count for 500 ms | Latched |
| Duty limit | 80 % | — |
| Units | 12414 counts per metre (2535 counts, 65 mm wheel); 24.05 counts per degree (track 111 mm) | [Robot geometry](CALIBRATION_RESULTS.md#robot-geometry) |

Note: Motion's `motion_config.h` has wheelbase 120 mm (MEASURE); this
repository measured 111 mm.

Servo: 1500 µs centre, 1100–1900 µs (±36°), 11.1 µs per degree. Settle
estimate 42 ms + 3.6 ms per degree, fitted to the scan settle times
(150 ms for 30°, 60 ms for 5°; [servo obstacle scan](OBSTACLE_SCAN.md)).

## Pico layer

```text
 main loop                         heartbeat interrupt, every 10 ms (hook)
 ─────────                         ───────────────────────────────────────
 mcp_set_speed(l, r) ─┐            lock
 mcp_set_servo(deg)  ─┤ lock        read encoder counts (sign per wheel)
 mcp_stop / estop    ─┤ store       mc_tick(counts, ms) → duty, servo pulse
 mcp_status(&s)      ─┘ copy        write motor PWM and servo PWM
                                   unlock
 encoder edges: own GPIO interrupt (quadrature table, from drive.c)
```

| Part | Rule |
| --- | --- |
| Pins | Left PWM GP8/GP9, encoder GP16/GP17 (+1); right PWM GP10/GP11, encoder GP4/GP5 (−1); servo GP15. Copied from `drive.c` and `servo.c`; do not link those files in the same target |
| Motor PWM | 20 kHz, wrap 6249; level = 6250 × \|duty‰\| / 1000 on the forward or the reverse pin; the other pin 0 |
| Servo PWM | 50 Hz, 1 count = 1 µs; written only when the pulse changes |
| Hook | Added by `mcp_init()` with period `tick_ms` (10 heartbeat ticks), phase 0. `mcp_init()` refuses a `tick_ms` that is not a whole number of heartbeat ticks |
| Lock | `critical_section_t` (interrupts off on this core and a hardware spin lock), so calls are safe from both cores. The hook also holds it while it writes the PWM, so a duty computed before an `mcp_estop()` cannot overwrite it |
| `mcp_stop()`, `mcp_estop()` | Also write PWM 0 at once; they do not wait for the next tick |
| Clock | `mcp_now_ms()` = `time_us_64() / 1000`, used by the hook and the main-loop calls. (`time_us_32()` in ms would wrap after 71 minutes.) |
| `mcp_status()` | One consistent copy: counts, travel, speed, duty, heading, servo, faults, tick count, longest hook time |
| Size | 1480 bytes of code (`-Os`, Cortex-M0+), core and heartbeat not included |

The servo pulse changes at the next tick, so up to 10 ms after
`mcp_set_servo()`. The settle estimate starts at the call; the error is
small against the 42 ms base.

## Floor bench (step 3)

`motor_ctl_bench` drives a fixed list of wheel speed steps on the floor
and records one row per 10 ms tick in RAM (up to 24 s). After the run it
prints a summary and all rows over USB and Wi-Fi (`text_<boot>.txt` from
`scripts/steer_log_receiver.py`).

| Step | Speed L/R (mm/s) | Time | Checks |
| --- | --- | --- | --- |
| fwd 150, rev 150 | ±150 | 1.5 s each | Rise, overshoot, steady error, reverse against forward |
| fwd 300, rev 300 | ±300 | 1.0 s each | Higher speed, duty limit |
| spin L 20, spin R 20 | ∓20 / ±20 | 3.0 s each (about 60°) | The slow-turn case (detour sweep, 20°/s): does the I term hold a speed below the dead zone? |
| fwd 40, rev 40 | ±40 | 2.0 s each | The slowest steady speed seen in the track logs |

Each step is followed by 0.7 s at speed 0 (coast). The list is 21.4 s; the
net travel is about 0.

| Control | Action |
| --- | --- |
| GP21 | Start after 1 s; a second press is an emergency stop |
| GP20 | Print the last run again |

The run stops early on a latched fault (stall or emergency stop).
Space: about 40 cm clear ahead and behind, and room to turn.

Row format: `MB i step tgtL tgtR spdL spdR dutyL dutyR cntL cntR faults`
(mm/s, permille, encoder counts). The summary gives, per step, the mean
speed and duty over the second half and the heading change from the
counts, plus the longest hook time and the heartbeat counters.

### Run B1 (2026-11-04, boot 5bf1, `ad2404c`, floor, VIN)

Log `build/steer_logs/text_5bf1.txt`; report
`python3 scripts/motor_bench_report.py build/steer_logs/text_5bf1.txt`.
All 17 steps done, no fault, 2101 of 2110 rows received (rows 7–15, in
the first rest, lost over Wi-Fi). Settings: `MC_ROBOT_CONFIG` as in
step 1 (untuned).

| Step | Wheel | Time to 90 % (ms) | Overshoot (50 ms avg) | Mean, 2nd half (mm/s) | Ripple sd (mm/s) | Duty, 2nd half (‰) |
| --- | --- | --- | --- | --- | --- | --- |
| fwd 150 | L / R | 150 / 180 | 18 % / 17 % | +151 / +149 | 2.2 / 2.8 | 219..242 / 223..241 |
| rev 150 | L / R | 150 / 170 | 34 % / 24 % | −152 / −150 | 4.1 / 2.5 | 214..233 / 211..230 |
| fwd 300 | L / R | 230 / 230 | 11 % / 12 % | +310 / +310 | 4.0 / 5.2 | 406..432 / 402..427 |
| rev 300 | L / R | 220 / 240 | 26 % / 17 % | −325 / −317 | 18.1 / 12.2 | 403..441 / 390..417 |
| spin L 20 | L / R | 60 / 60 | 100 % / 60 % | −20 / +19 | 3.8 / 3.3 | 82..115 / 75..103 |
| spin R 20 | L / R | 50 / 50 | 124 % / 100 % | +21 / −20 | 2.6 / 2.7 | 69..97 / 85..108 |
| fwd 40 | L / R | 90 / 60 | 52 % / 40 % | +40 / +40 | 1.1 / 1.6 | 81..95 / 86..99 |
| rev 40 | L / R | 70 / 70 | 56 % / 48 % | −41 / −40 | 1.4 / 2.0 | 83..99 / 84..98 |

| Item | Result |
| --- | --- |
| Steady speed | Within 2 mm/s of the target at 20, 40 and 150 mm/s, both wheels and both directions. **20 mm/s holds** (the detour sweep case), at 75–115 ‰ duty |
| Spin | 65.3° and −66.1° for a 3 s step at 20 mm/s (62° from the target alone; the extra is the start overshoot) |
| Straight drives | Heading change −0.3° (fwd 150), +3.2° (rev 150), +0.2° (fwd 300), +6.7° (rev 300): the left wheel overshoots more in reverse |
| Overshoot | 11–34 % at 150 and 300 mm/s; 40–124 % at 20 and 40 mm/s (start kick and quantisation: 1 count per tick = 8 mm/s). At 300 mm/s the 1 s step ends before the overshoot is gone |
| Cause of the overshoot (fwd 150, left) | The wheel lags the set point by about 100 ms. During the lag P and I rise; the duty peaks at 320 ‰ against 230 ‰ steady, then the I term unwinds over about 300 ms |
| Feed-forward (steady points) | Forward and reverse are now alike (the weak left reverse from the track logs is not seen). Straight-line fit, counts per tick: L fwd 48 + 9.66·v, L rev 59 + 8.89·v, R fwd 56 + 9.39·v, R rev 62 + 8.67·v. The step-1 values are too high (L rev slope 12.0: 308 ‰ against 223 ‰ needed at 150 mm/s). Below 5 counts per tick the duty is flat at about 80–97 ‰ (dead zone) |
| Timing | Hook at most 88 µs; heartbeat interrupt at most 102 µs (10 % of a tick); no overruns. Missed 424 frames: the main loop stopped during the row print after the run (motors at 0) |

## Ideas (not implemented)

**Burst drive for the dead zone** (operator idea, 2026-11-04). At 20 kHz the
motor sees the average voltage, so a low duty is a steady weak push below
the static friction. Short strong pushes with rests between them can
break the friction and give a lower average speed. Reason: the slowest
steady speed is about 40–50 mm/s (14 %), but a 20°/s detour sweep needs
about 19 mm/s per wheel; the sweeps ran at 40°/s.

| Way | How | Cost |
| --- | --- | --- |
| Lower PWM frequency | About 100–500 Hz | Audible whine |
| Bursts | 20 kHz at about 25 % for N ms, then off for M ms | Step-like motion; counts come in bunches |
| Dither | A small fast change added to the duty | Smaller effect |
| Brake mode (driver chip) | Brake instead of coast between pulses (`drive.c` coasts now) | Needs the Robo Pico driver chip datasheet |

If built: the speed loop must average over a full burst cycle. Bench
test first: the slowest steady speed on the floor for each way.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_motor_ctl.c src/motor_ctl.c -o build/test_motor_ctl
./build/test_motor_ctl
```

The test drives a simulated wheel pair (dead zone about 9 %, first-order
lag) and checks: 150 mm/s reached within 5 %; the same with a 30 % battery
drop, a 20 % weaker left motor, a weak left reverse, and a slow turn on the
spot (±40 mm/s, the detour sweep case) with no stall; rise limit; start
kick and kick on a direction change; stale stop at exactly 100 ms; stop;
emergency stop and clear; stall at exactly 500 ms (both duties 0); speed
sent while latched has no effect; late tick; exact feed-forward, P, clamp
and anti-windup values; travel, heading; servo pulse, clamp and settle
time.

Mutation check 2026-11-04: 35 of 36 caught. The survivor is equivalent
(`<=` against `<` in the rise limit: both give the asked set point). Four
redundant lines found by the check were removed.

Pico compile check: `arm-none-eabi-gcc` with `-Wall -Wextra -Wconversion
-Wshadow -Wformat=2 -Werror`: clean; 1.9 KB of code.
