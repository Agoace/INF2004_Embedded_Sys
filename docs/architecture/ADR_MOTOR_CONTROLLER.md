# ADR: the motor controller is the only owner of the actuators

| Item | Value |
| --- | --- |
| Status | Accepted, 2026-11-04 (operator) |
| Scope | Motor PWM, servo PWM, wheel encoders |
| Branch for the work | `mtk3-motion` (from `yx`) |

## Context

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
| Encoder signs, 2535 counts per wheel turn | `drive.c` | [Calibration results](../s2-motion/CALIBRATION_RESULTS.md) |
| 24.05 counts per degree of turn | `detour_test.c` | Calculated; slip not measured |
| Servo centre and safe range (1100–1900 µs) | `servo.h` | [Servo obstacle scan](../s5-obstacle/OBSTACLE.md) |
| Servo settle time before a ping | `obstacle_scan.c` timing | [Servo obstacle scan](../s5-obstacle/OBSTACLE.md) |

The team Motion module (Buddy 2, branch `motor-line-manager`, now in tag `archive/yx-line-abstraction`,
`src/motion/`, `docs/MOTION_CONTROL.md`) already owns the motors and has
a closed-loop speed control. But it also holds intent: `motion_set_steer()`
(steer mixing), `motion_set_slow()` (a line-following "calm" mode), a
FOLLOW state, and `MOTION_CMD_ROTATE` / `MOTION_CMD_FWD` (it decides when
a move is complete). It does not own the servo.

## Decision

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

## Consequences

| Area | Change |
| --- | --- |
| Line steering | Output becomes left and right wheel speed. `min_pct`, `max_pct` and `right_trim_pct` go to the controller. Gains must be tuned again |
| Detour | The sweep rate control (duty 14–30 %) becomes a turn-rate request. The sweep stall (D27, D31, D33, D35) is handled by the closed-loop speed control |
| Obstacle scan | Sends an angle and waits for "settled"; no pulse widths or settle timing |
| Search, spin calibration | Send wheel speeds, not duty |
| Host tests and simulator | Callers need no robot numbers; the simulator can replace the controller with a model |
| Accuracy | The caller stops a turn from the measured heading. With a 20 ms controller period, a 45°/s turn can overshoot by about 1° |
| Tuning | A new battery, motor or wheel changes only the controller |

## Alternatives considered

| Option | Why not chosen |
| --- | --- |
| Callers send duty % (no speed control) | No intent, but every caller must handle the hardware quirks |
| Callers send speed + steer + calm (Motion today) | The controller holds line-following intent |
| Callers send moves (rotate 90°, forward 350 mm) | The controller decides when a move is complete (intent) |

## Work rules

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
