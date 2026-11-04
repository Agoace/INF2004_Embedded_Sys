# Vehicle Controller line run (`vc_line_test`)

The first target in the new layers
([ADR: Vehicle Controller](ADR_VEHICLE_CONTROLLER.md)): line tracing and
line search as modules in the [shell](VEHICLE_STATE_MACHINE.md), wheel
speeds through the [motor controller](MOTOR_CONTROLLER.md). Branch:
`yx-line-steer`. Status: built and host-tested; **not run on the track**.
The goal of the first run (P5) is to compare with P4 of the
[locked line tracing](TWO_SENSOR_LINE_TRACING.md).

- Line tracing module: [`src/line_trace.c`](../src/line_trace.c), [`src/line_trace.h`](../src/line_trace.h)
- Line search module: [`src/line_search.c`](../src/line_search.c), [`src/line_search.h`](../src/line_search.h)
- Detector data for modules: [`src/vehicle_inputs.h`](../src/vehicle_inputs.h)
- Pico glue (target): [`src/vc_line_test.c`](../src/vc_line_test.c)
- Host unit test: [`tests/test_line_trace.c`](../tests/test_line_trace.c)

## Layers

```text
 1 ms flag ─▶ line sensors ─▶ lp_update ─▶ last LOST / FOUND event ─┐
 100 ms    ─▶ ultrasonic ping ─▶ new reading ───────────────────────┤ vsm_inputs
 10 ms     ─▶ mcp_status: wheel counts, latched faults ─────────────┘
                                                                    │
 10 ms flag ─▶ vsm_step ─▶ LINE or SEARCH module ─▶ wheel mm/s, servo ─▶ mcp_set_speed
 heartbeat interrupt: mc_tick every 10 ms (PWM, encoders), watchdog 100 ms
```

| Layer | Here |
| --- | --- |
| Detectors | Line position every 1 ms; an ultrasonic reading every 100 ms (servo straight); wheel counts. They never decide |
| Modules | LINE and SEARCH. SCAN and DETOUR are not registered: an obstacle stops the run (shell FAULT NO_MODULE, detail 3) |
| Shell | `vsm_step()` on the 10 ms flag at phase 9 (1 ms before the next `mc_tick`; phase 1 until P5), routes `VSM_DEFAULT_ROUTES` |
| Motor controller | The only writer of motor and servo PWM. A latched stall stops the run (shell FAULT SAFETY) |

Deviation from the [heartbeat ADR](ADR_HEARTBEAT.md): the line sample and
`lp_update()` run in the main loop on a 1 ms flag (as in `detour_test`),
not in the interrupt. A late main loop is counted as missed frames.

## Modules

| Module | Does (same as `detour_test`) | Outcomes |
| --- | --- | --- |
| LINE | `steer_mm_update()` on the position ([mm/s steering](LINE_STEER_MM.md)); straight for 450 counts across a junction hold; side hint to the line detector when the turn (left − right counts since centred) is above 20; forward watch (`obstacle_update()`) on each new reading. `start()`: D, heading, watch and junction memory reset | LINE_LOST (payload: side), OBSTACLE (payload: mm), FAILED 1 (ultrasonic fault) |
| SEARCH | Turn on the spot at 131 mm/s toward the last side (RIGHT: right first, else left) to 316 counts ((right − left) / 2), then the other way to 316 past the start | FOUND, FAILED 1 (not found) |

Settings, mapped once through the [duty map](DUTY_MAP.md):

| Setting | `detour_test` (duty) | `vc_line_test` (mm/s) |
| --- | --- | --- |
| Line base / min / max | 25 / 12 / 50 % | 171 / 62 / 378 |
| Gain, D, far gain | 100, 100, 400 above 50 | the same |
| Right trim | +1 | none (motor controller) |
| Search turn | 20 % | 131 |
| Spin calibration | 20 % | 131 |
| Stall stop | Own: no count for 500 ms | Motor controller: ≥ 150 ‰ for 500 ms |

## Spin calibration

`line_spin.c` drives the motors with `drive.c`, which cannot share a
target with the motor controller. `vc_line_test.c` has the same steps
with wheel speeds: a quarter wheel turn left, through the start to a
quarter turn right (200 ms still between), then back to the start and
stop on the line (C at 90 %, or 5 % below its peak after 50 %). Changes:
both wheels start together (the 50 ms delay was for USB power); a motor
fault or 6 s per step stops it.

## Log

The [steering Wi-Fi log](STEER_WIFI_LOG.md) every 50 ms, live and
replayed after the run. Header flag bit 4 (`SL_FLAG_SPEED`): the two wheel
fields are the asked speeds in 4 mm/s units; the receiver shows mm/s.
Modes: FOLW, SRCH, STRT. The summary gives the stop reason, the shell
fault, the last handoff, handoffs, searches, pings, heartbeat health and
the motor controller hook time.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_line_trace.c src/line_trace.c src/line_search.c \
   src/line_steer.c src/obstacle.c src/vehicle_sm.c src/line_position.c \
   -o build/test_line_trace
./build/test_line_trace
```

| Check | Result |
| --- | --- |
| LINE: centred 171/171; +20: 215/127; servo 0 | Pass |
| LINE: hint at ±20 none, at ±21 a side; centred clears the heading | Pass |
| LINE: junction straight for 449 counts, steer at 450; a new hold and a start in a hold start a new straight | Pass |
| LINE: LINE_LOST with the side; FOUND ignored; obstacle after 2 readings with the distance; no new reading: no update; start resets the watch; ultrasonic fault: FAILED 1 | Pass |
| SEARCH: right first or left first by payload; turn back at 316, FAILED at −316; start counts at the first step; start resets the step; FOUND; LOST ignored | Pass |
| Shell: LINE → LINE_LOST → SEARCH in the same tick → FOUND → LINE; search failed: FAULT FAILED; obstacle: FAULT NO_MODULE; stall: FAULT SAFETY | Pass |

Mutation check (2026-11-04): 35 of 35 caught. The first pass found four
gaps, now closed: the hint at exactly +20, the watch reset in `start()`
(the test reused a watch that had already stopped), the junction memory
and the heading reset in `start()`.

ARM size (`-Os`): `line_trace.c` 340 bytes, `line_search.c` 154 bytes.

## Track procedure (P5)

1. Restart the receiver (it needs the `SL_FLAG_SPEED` change).
2. VIN on, USB off; wait for the LED and the start text.
3. C sensor over the line, no box on the track.
4. Press GP21: spin calibration, then the run after 1 s. Press again to
   stop; run limit 90 s.
5. Keep it on for about 2 s after it stops (replay).

## Run P5 (2026-11-04, boot 61ae)

Log `build/steer_logs/run_20261104-165030_boot61ae_run1.txt`. The operator
stopped it at 87.6 s (GP21): **wobbly**. No faults; heartbeat ISR max
103 µs, missed frames 0, motor controller hook max 86 µs.

| Measure (follow mode) | P4 (duty) | P5 (mm/s) |
| --- | --- | --- |
| Side changes (position past ±20) | 2.1 per s | **4.8 per s** |
| Position at ±45 or more (one sensor only) | 52 % | **81 %** |
| Searches | 37 per min | **60 per min** |
| Mean speed (counts) | 146 mm/s | 115 mm/s |
| Wheel difference follows the command after (best match) | 50 ms (0.90) | **100 ms (0.86)** |
| Measured / asked wheel difference (spread) | — | 109 / 204 mm/s |

Finding: the settings map correctly on average (asked mean 172 mm/s,
base 171), but the wheels follow the steering about **50 ms later** than
with duty, and reach only about half the asked difference. The same
gains with more delay make the steering swing. Sources of the delay:

| Source | Delay |
| --- | --- |
| Speed-up limit `accel_cpt_q8` 512 (16 mm/s per 10 ms): the faster wheel needs about 70 ms for a +112 mm/s change (slowing is not limited) | up to 70 ms |
| Shell step 1 ms after `mc_tick`; its command waits for the next tick | 9 ms |

Changes after P5 (2026-11-04):

| Change | Effect |
| --- | --- |
| Motor controller rise limit 2 → 8 counts/tick per tick (about 6400 mm/s²) | +112 mm/s in about 20 ms instead of 70 ms. Check overshoot on the bench (B3) before P6 |
| Shell step at phase 9 instead of 1 | The command reaches the motors about 1 ms later instead of 9 ms |

## Run P6 (2026-11-04, boot 3f9e run 2)

Changes from P5: rise limit 8, shell step at phase 9. Log
`build/steer_logs/run_20261104-171448_boot3f9e_run2.txt` (run 1 of this
boot was lost: the laptop left the hotspot). The operator stopped it at
25.7 s (GP21): **still wobbly, a little less**. No faults; heartbeat ISR
max 103 µs, missed frames 0.

| Measure (follow mode) | P4 (duty) | P5 | P6 |
| --- | --- | --- | --- |
| Wheel difference follows the command after (best match) | 50 ms (0.90) | 100 ms (0.86) | **50 ms (0.85)** |
| Side changes (position past ±20) | 2.1 per s | 4.8 per s | **5.5 per s** |
| Position at ±45 or more | 52 % | 81 % | 78 % |
| Position spread (sd) | 37 | 54 | 50 |
| Searches (run summary) | 37 per min | 82 per min | **12 per min** |
| Mean speed (counts) | 146 mm/s | 115 mm/s | **166 mm/s** |
| Measured wheel difference per position unit | 1.59 mm/s | 1.21 mm/s | **1.86 mm/s** |

Finding: the delay is now the same as P4, and the line is lost less
often. But the robot drives **14 % faster** and turns **17 % harder**
for the same position than in P4. The duty map was measured in straight
runs; in P4 the open-loop wheels did not reach the mapped speeds while
turning (load), and the speed loop now does. The swing is faster (about
2.7 per s).

Change for P7 (operator, 2026-11-04): match what P4 really did.

| Setting | P5, P6 | P7 |
| --- | --- | --- |
| Base | 171 mm/s | **146 mm/s** (P4 mean speed) |
| Gain / D / far gain | 100 / 100 / 400 | **85 / 85 / 340** (× 0.85: P4 1.59 against P6 1.86 mm/s per position unit) |
| Min, max, far from | 62, 378, 50 | the same |

## Run P7 (2026-11-04, boot 95bc run 1)

Settings as above (base 146, gains × 0.85). Log
`build/steer_logs/run_20261104-172140_boot95bc_run1.txt`, 1205 of 1205
entries. The operator stopped it at 60.3 s (GP21): **better**. No
faults; heartbeat ISR max 103 µs, missed frames 0.

| Measure (follow mode) | P4 (duty) | P6 | P7 |
| --- | --- | --- | --- |
| Side changes (position past ±20) | 2.1 per s | 5.5 per s | **3.6 per s** |
| Position at ±45 or more | 52 % | 78 % | **58 %** |
| Position spread (sd) | 37 | 50 | **44** |
| Searches (run summary) | 45 per min | 12 per min | **33 per min** |
| Mean speed (counts) | 146 mm/s | 166 mm/s | **142 mm/s** |
| Measured wheel difference per position unit | 1.59 mm/s | 1.86 mm/s | **1.40 mm/s** |

Result: close to P4, with fewer searches. The side changes are still
more frequent than in P4 although the turn is now 12 % weaker, so turn
strength is not the only cause.

### D test (after P7)

Operator (2026-11-04): P7 swings less far but makes more small swings.
The log shows the same main swing as P4 (about 3 Hz) with 30 % larger
position steps per 50 ms (sd 40 against 31); the inner wheel stops and
starts as often as in P4 (about 3 per s). Test whether the steering is
under-damped: only D changes.

GP20 (idle) steps D through 85 (P7) → 130 → 50 → 85 and prints
`[cfg] steering D …`; the run summary prints the D in use. GP20 no longer
starts a calibration only.

| D higher (130) | Meaning | Next |
| --- | --- | --- |
| Swings less | Under-damped | Try more D |
| Swings more | D amplifies the position noise | Try D 50 |
| No change | Not the steering | Wheel speed loop (overshoot 6–12 %, bench B3) |

#### Runs P7b and P8 (2026-11-04, boot 2c0b runs 1 and 2)

P7b repeated D 85 (the GP20 press was not registered); P8 used D 130.
Operator after P7b: it calms, then falls into a stronger swing. The log
shows the swings come **on the straights after a curve** (1 s windows:
position sd 6–22 in curves, 40–49 on the straight after; correlation of
turn rate with sd −0.59), then die out in 2–3 s.

| Run | D | Side changes | Position sd, straight windows (turn < 15 °/s) | sd, curve windows (≥ 40 °/s) | Searches |
| --- | --- | --- | --- | --- | --- |
| P4 (duty) | 100 (duty) | 2.1 per s | **26** (max 49) | 17 | 46 per min |
| P7 | 85 | 3.6 per s | 37 (max 55) | 16 | 33 per min |
| P7b | 85 | 4.1 per s | 35 (max 49) | 17 | 30 per min |
| P8 | 130 | 3.9 per s | 36 (max 52) | 16 | 35 per min |

Result: D 85 → 130 changes nothing. In 53–58 % of the time the line is
under one sensor only, where the position stays near ±50 and its change
(the D input) is near 0, so D has little to act on. The difference to
P4 is on the straights only (sd 36 against 26); curves are the same.
