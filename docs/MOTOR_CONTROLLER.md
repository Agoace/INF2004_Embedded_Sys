# Motor controller

The only owner of the motor PWM, the servo PWM and the wheel encoders. The
decision and its reasons: [ADR: motor controller](ADR_MOTOR_CONTROLLER.md).
Branch: `yx-motor-controller`.

## Status

| Step | What | State |
| --- | --- | --- |
| 1 | Pure core [`src/motor_ctl.c`](../src/motor_ctl.c), [`src/motor_ctl.h`](../src/motor_ctl.h); robot settings [`src/motor_ctl_robot.h`](../src/motor_ctl_robot.h); host test [`tests/test_motor_ctl.c`](../tests/test_motor_ctl.c) | Done 2026-11-04 |
| 2 | Pico layer [`src/motor_ctl_pico.c`](../src/motor_ctl_pico.c), [`src/motor_ctl_pico.h`](../src/motor_ctl_pico.h): PWM, encoder interrupt, servo; hooks `mc_tick()` into the [heartbeat](HEARTBEAT.md) every 10 ms (the heartbeat owns the timer and the watchdog: [ADR: heartbeat](ADR_HEARTBEAT.md)). CMake library `motor_ctl_pico` | Done 2026-11-04: strict ARM compile clean; not yet run on hardware (step 3) |
| 3 | Floor bench target `motor_ctl_bench` ([`src/motor_ctl_bench.c`](../src/motor_ctl_bench.c)): step response, both directions, slow turn; tune P, I and feed-forward | Done 2026-11-04: runs B1 and B2; settings of B2 kept (`1cf08a1`) |
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
| Feed-forward | offset + slope × set point, **per wheel and per direction** | Offset / slope (‰, ‰ per count/tick): L fwd 48 / 9.66, L rev 59 / 8.89, R fwd 56 / 9.39, R rev 62 / 8.67. Fit to floor bench run B1 (2026-11-04). Step 1 used a rough fit to the track logs (offset 85–90, slope 9.0–12.0) |
| P | kp × (set point − measured) | 6.0 ‰ per count/tick (Motion 3.0 at 20 ms, scaled) |
| I | ki × error each tick; limit ±400 ‰. **Held** (not integrated) while the output is at the limit in the error's direction, **while the set point still rises, and while the start kick runs** (added after run B1: the wheel lags then, and I wound up and overshot) | 0.5 per tick |
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

### Run B2 (2026-11-04, boot 8460, `1cf08a1`, floor, VIN)

Changes from B1: feed-forward refit to B1, and the I term held while the
set point rises and while the kick runs. Same step list. Log
`build/steer_logs/text_8460.txt`. All 17 steps done, no fault; the same
9 rows (7–15) lost over Wi-Fi.

| Step | Overshoot B1 → B2 (L / R) | Time to 90 % B1 → B2 (ms, L / R) | Mean, 2nd half B2 (mm/s) | Heading change B1 → B2 |
| --- | --- | --- | --- | --- |
| fwd 150 | 18/17 % → **7/8 %** | 150/180 → 170/180 | +149 / +150 | −0.3° → +0.5° |
| rev 150 | 34/24 % → **7/8 %** | 150/170 → 200/200 | −152 / −151 | +3.2° → −0.7° |
| fwd 300 | 11/12 % → **5/6 %** | 230/230 → 260/260 | +304 / +304 | +0.2° → +0.5° |
| rev 300 | 26/17 % → **4/4 %** | 220/240 → 290/290 | −304 / −306 | +6.7° → −0.5° |
| spin L 20 | 100/60 % → 116/28 % | 60/60 → 50/60 | −20 / +20 | 65.3° → 59.8° |
| spin R 20 | 124/100 % → 116/116 % | 50/50 → 40/50 | +21 / −19 | −66.1° → −59.1° |
| fwd 40 | 52/40 % → **12/24 %** | 90/60 → 160/70 | +39 / +39 | −0.6° → +0.5° |
| rev 40 | 56/48 % → **20/28 %** | 70/70 → 130/120 | −39 / −39 | +0.9° → +0.1° |

| Item | Result |
| --- | --- |
| 150 and 300 mm/s | Overshoot 4–8 % (was 11–34 %); ripple sd 1.8–2.7 mm/s (was up to 18); the 300 mm/s mean is now within 6 mm/s in the 1 s step. Rise is 20–70 ms slower: the I term no longer helps during the rise |
| Straight drives | Heading change at most 0.7° in every straight step (was up to 6.7°): the two wheels now match |
| Slow spin (20 mm/s) | Still about 2× the target speed in the first 50 ms. Cause: the start kick (120 ‰ until 10 counts) drives the wheel to 40–56 mm/s in 3 ticks; after the kick the speed drops to 8–16 mm/s for about 200 ms while I builds from 0. The turn is 59–60° against 62° from the target |
| Feed-forward refit to B2 | Offset 52–60 ‰, slope 8.7–9.3: close to the values in use, so no change |
| Timing | Hook at most 85 µs, heartbeat interrupt at most 99 µs, no overruns |

Decision (2026-11-04, operator): keep the B2 settings; the slow-start
kick is accepted (about 100 ms at the start of a slow turn; the spin is
within 3° of the target). Options not taken are under *Ideas*.

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


Slow-start kick (from run B2, not taken):

| Option | Effect | Cost |
| --- | --- | --- |
| Shorter kick (`kick_counts` 10 → about 4) | Smaller burst at the start of a slow turn | A weak start may not break the static friction; needs a bench run |
| Start I at the kick duty, not 0 | Removes the 200 ms slow patch after the kick | One more rule in the loop; needs a bench run |

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
and anti-windup values; I held while the set point rises and while the
kick runs, and I grows at the target; travel, heading; servo pulse, clamp
and settle time.

Mutation check 2026-11-04: 35 of 36 caught. The survivor is equivalent
(`<=` against `<` in the rise limit: both give the asked set point). Four
redundant lines found by the check were removed.

Mutation check after the I hold (2026-11-04): 38 of 39 caught (3 new
mutants for the hold rule), the same equivalent survivor. The check first
showed that the anti-windup test had become empty (the kick held I); the
test now ends the kick after one tick.

Pico compile check: `arm-none-eabi-gcc` with `-Wall -Wextra -Wconversion
-Wshadow -Wformat=2 -Werror`: clean; 1.9 KB of code.
