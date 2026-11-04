# Motor controller

The only owner of the motor PWM, the servo PWM and the wheel encoders. The
decision and its reasons: [ADR: motor controller](ADR_MOTOR_CONTROLLER.md).
Branch: `yx-motor-controller`.

## Status

| Step | What | State |
| --- | --- | --- |
| 1 | Pure core [`src/motor_ctl.c`](../src/motor_ctl.c), [`src/motor_ctl.h`](../src/motor_ctl.h); robot settings [`src/motor_ctl_robot.h`](../src/motor_ctl_robot.h); host test [`tests/test_motor_ctl.c`](../tests/test_motor_ctl.c) | Done 2026-11-04 |
| 2 | Pico layer [`src/motor_ctl_pico.c`](../src/motor_ctl_pico.c), [`src/motor_ctl_pico.h`](../src/motor_ctl_pico.h): PWM, encoder interrupt, servo; hooks `mc_tick()` into the [heartbeat](HEARTBEAT.md) every 10 ms (the heartbeat owns the timer and the watchdog: [ADR: heartbeat](ADR_HEARTBEAT.md)). CMake library `motor_ctl_pico` | Done 2026-11-04: strict ARM compile clean; not yet run on hardware (step 3) |
| 3 | Bench target: step response, both directions, slow turn; tune P, I and feed-forward | Not started |
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
