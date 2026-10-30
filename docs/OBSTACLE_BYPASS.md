# Obstacle bypass

Status: version 2, host-tested and built on 2026-10-30 after track run 1
hit the obstacle ([track tests](#track-tests)). Not yet tried on the track.

This record describes how the robot drives around one obstacle on the line
and finds the line again. The operator's name for the plan is the "Blind
Bicycle Algorithm" (BBA). The first plan used arcs and wheel-count targets.
The operator and the assistant changed it to the simpler edge following
below: the sensor follows the edge of the obstacle, and the robot steers so
that the edge passes a fixed clearance from its centre line. After run 1,
the operator agreed to one wheel-count check: the straight PASS past the
obstacle, because the sensor cannot see beside the robot.

## Assumptions

- One obstacle sits on the line. Nothing else is near it.
- The scan (±30°, [OBSTACLE_SCAN.md](OBSTACLE_SCAN.md)) sees past at least
  one side of it.
- The robot finds the line again after the obstacle. No map is used; the
  only distance is the PASS travel.

## Steps

```text
 FOLLOW ──STOP (forward watch, 258 mm)──▶ wait 300 ms ──▶ SCAN
                                                          │ side S, obstacle mm
                                                          ▼
                                   PIVOT (servo 30° toward the obstacle)
                                     │ obstacle in the beam, then 2 free
                                     ▼
            ┌──────────────▶ PASS (straight) ──travel done──▶ WRAP (20° back)
            │ clear at 30°     │ hit                             │ hit
            │                  ▼                                 │
            └─────────────── EDGE ◀──────────────────────────────┘
                     (steer = hit angle + asin(clearance / d) toward S)

 Any AROUND phase ──line seen again──▶ FOLLOW
 PIVOT / AROUND ──time limit / fault──▶ FAILED (stop)
```

| Step | What the robot does | Ends when |
| --- | --- | --- |
| STOP | Motors off (coast) | 300 ms have passed |
| SCAN | One 13-point scan. The PROFILE word picks the side S with the smaller free step (tie → more room at 30°, then LEFT). The obstacle is on the other side | The scan is done |
| PIVOT | Turns on the spot toward S at 20 % duty. The servo looks 30° toward the obstacle | The obstacle has gone **through the beam and out**: one reading that is not free, then 2 free readings → AROUND, PASS. Free = no echo, or an echo at least 150 mm past the obstacle (the scan's own rule) |
| AROUND | Drives forward in phase PASS, WRAP or EDGE (below) | The line is seen again → FOLLOW |
| FAILED | Stops the run | — |

Why the pivot rule is "through the beam and out" (run 1): "free straight
ahead" only means that the narrow beam misses the obstacle, and in run 1
the robot turned only about 13°. "Free at 30°" is no better: the run 1 scan
already read no echo at +30° before the pivot. When the obstacle has gone
out of a beam that looks 30° toward it, the obstacle edge is about 40° off
the heading (30° plus about 10° of beam).

### AROUND phases

| Phase | Steering | Servo | Ends when |
| --- | --- | --- | --- |
| **PASS** | Straight (0°) | Stays at 30° toward the obstacle | Mean wheel travel = last distance × cos 30° + `pass_extra_mm` → WRAP. A hit → EDGE |
| **WRAP** | `wrap_deg` (20°) toward the obstacle side, back toward the line | Stays at 30° | A hit → EDGE |
| **EDGE** | Hit angle θ + asin(`clearance_mm` / d) toward S, at most ±30° | Rubber band (below) | Clear at 30° → PASS, with a new pass distance |

The steering rule in EDGE: a point at distance d passes `clearance_mm` from
the robot's centre line when the heading is asin(clearance / d) away from
it. With 110 mm (80 mm half width + 30 mm margin):

| d | 250 mm | 200 mm | 186 mm | 168 mm | 155 mm | 110 mm or less |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| asin(110 / d) | 27° | 34° | 37° | 41° | 46° | 89–90° (clamped to 30°) |

Run 1 hits with the old fixed 20° offset and with this rule (bypass right,
so "away" is negative):

| Hit | Old steering | New steering |
| --- | ---: | ---: |
| +20° at 168 mm | 0° | −21° |
| +25° at 155 mm | +5° | −21° |

The pass distance: the last point seen at 30° is d × cos 30° ahead of the
sensor. `pass_extra_mm` (130 mm) adds 65 mm from the servo to the rear axle
and 65 mm for the depth of the obstacle past that point. Counts =
mm × 2535 / 204.204 (wheel diameter 65 mm), 12.41 counts per mm.

### Rubber band (EDGE)

| Reading at servo angle θ | Servo next | Steering angle |
| --- | --- | --- |
| **Hit** (echo nearer than `hit_mm`) | θ + `bounce` toward S | θ + asin(`clearance_mm` / d) toward S |
| **Clear** (no echo, or echo at `hit_mm` or farther) | θ + `step` toward the obstacle | No change |
| Clear at 30° toward the obstacle | Stays | 0°: PASS starts |
| No information (NO_RISE, STUCK_HIGH, no fault streak) | Same angle, read again | No change |

Angles are positive to the left, as in the scan. A hit in PASS or WRAP
starts EDGE in the same way.

### Settings

The bench values are in [`src/bypass_test.c`](../src/bypass_test.c)
(`BYPASS`). They are first values, to tune on the track.

| Setting | Value | Notes |
| --- | ---: | --- |
| `hit_mm` | 250 | About the stop distance (258 mm). Larger keeps the robot farther away |
| `clearance_mm` | 110 | Robot width 160 mm: 80 mm half width + 30 mm margin |
| `pass_extra_mm` | 130 | Servo to rear axle (65 mm) + obstacle depth (65 mm) |
| `wrap_deg` | 20 | Turn back toward the line after PASS |
| `limit_deg` | 30 | Servo and steering limit each side (the scan range) |
| `step_deg` | 5 | Servo step while clear |
| `bounce_deg` | 10 | Servo step back after a hit |
| `line_arm_ms` | 200 | The line must be lost this long before it can end the bypass. The robot starts on the line |
| `pivot_max_ms` | 4000 | Pivot time limit |
| `around_max_ms` | 10000 | AROUND time limit |

Servo settle time before each reading: 40 ms + 4 ms per degree moved
(60 ms for 5°, 80 ms for 10°, 160 ms for the first 30° move). The
ultrasonic driver also keeps 60 ms between triggers.

## Steering

The robot is treated as a bicycle. The servo is the front wheel and the
rear axle is the back wheel. Operator measurements, 2026-10-30 (about):

| Size | Value |
| --- | ---: |
| Track W (wheel contact to wheel contact) | 111 mm |
| Wheelbase L (servo axis to rear axle) | 65 mm |
| Wheel diameter | 65 mm |
| Robot width (widest point) | 160 mm |

At steering angle δ the rear axle turns on radius R = L / tan δ. Each wheel
then runs at speed × (1 ∓ (W/2)·tan δ / L). `line_steer` with gain 100 %
gives speed × (1 ± position / 100). So:

position = −100 · (W/2) · tan δ / L = −85.4 · tan δ (negative = left)

| δ | 0° | ±5° | ±10° | ±15° | ±20° | ±30° |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| position | 0 | ∓7 | ∓15 | ∓23 | ∓31 | ∓49 |

`bp_steer_position()` uses a 31-entry table of tan × 1000 (the RP2040 has
no floating-point hardware) and rounds. At 30° the inner wheel runs at
51 % of the base speed above `min_pct`. The bypass uses base 25 %, gain
100 %, no damping, max 50 %, right trim +1 % (`STEER_BYPASS`).

## Command word (Change Requirement 1)

`bp_update()` returns one 16-bit word:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–1 | Motion: 0 STOP, 1 PIVOT, 2 STEER | 2 |
| 2–7 | Steering angle + 30, 0–60 (PIVOT: 60 left, 0 right) | 6 |
| 8–13 | Servo angle + 30, 0–60 | 6 |
| 14–15 | Spare, 0 | 2 |

Angles are whole degrees in ±30, so 6 bits each (61 values) are enough.

## Failures

| Reason | When |
| --- | --- |
| No free side | The scan saw both sides blocked |
| Pivot time limit | The obstacle did not go through the beam and out in 4 s |
| Line not found | AROUND ran 10 s |
| Ultrasonic fault | Fault streak (5 faults in a row) |
| Obstacle too near ahead | TOO_NEAR (under 60 mm) within ±10° of straight ahead in AROUND |

`bypass_test` stops the run on each failure.

## Bench test: `bypass_test`

[`src/bypass_test.c`](../src/bypass_test.c) follows the line with the calm
settings of `steer_test` (base 25 %, max 50 %), with the same junction
straight and line search. It does the steps above when the forward watch
gives STOP.

Pins: ultrasonic trigger GP3, echo GP2, servo GP15. Buttons as `steer_test`.

Procedure (power from VIN, USB unplugged, [USB_POWER_SAFETY.md](USB_POWER_SAFETY.md)):

1. Start the receiver ([STEER_WIFI_LOG.md](STEER_WIFI_LOG.md#use)).
2. Put an obstacle on a straight part of the line, at least 60 cm ahead of
   the robot. Keep 30 cm free on both sides.
3. VIN on, wait for the LED. Put the robot on the line and press GP21. It
   spin-calibrates, waits 1 s, then drives.
4. The run stops after 30 s, on GP21, or on a failure. Keep the robot on
   for about 2 s after it stops so the replay is sent.

The Wi-Fi log gets the 50 ms entries (mode 3 is `BYPS`: STOP, SCAN or
bypass; its position column is the steering angle in degrees) and one
40-bit record per bypass reading, with the state and the AROUND phase
([packet format](STEER_WIFI_LOG.md#packet-format)). The receiver writes the
readings as a table at the end of the run file. The text shows each state
and phase change with the steering angle, the last distance and the pass
distance.

### What to watch in the first tests

| Watch | If it happens |
| --- | --- |
| The side or rear corner touches the obstacle | Larger `clearance_mm` or `pass_extra_mm` |
| The robot goes very wide and misses the line | Smaller `clearance_mm`, larger `wrap_deg` |
| The robot crosses the line steeply and follows it back toward the obstacle | Smaller `wrap_deg`, so it meets the line at a flatter angle |
| A steep crossing reads as a junction | The bypass already ignores readings with all three sensors on the line; check the log |
| The pivot never ends (wall behind the obstacle) | Move the obstacle, or check the scan |

## Track tests

### Run 1: 2026-10-30, first try (hit the bottle)

Log: `build/steer_logs/run_20261030-140353_bootaaa6_run1.txt` (not in Git).
Obstacle: a bottle on a straight part of the line.

| Time | What happened |
| ---: | --- |
| 2845 ms | Forward watch STOP at 201 mm. The robot coasted about 50 ms |
| 2.9–4.95 s | Scan: `NR NR NR NR NR NR 190 186 186 206 NR NR NR` (bottle at 0° to +15°). Free step R 1, L 4 → bypass RIGHT |
| 4.95–5.14 s | PIVOT right: 2 free readings straight ahead after about 150 counts per wheel |
| 5.14–5.9 s | AROUND: hits at +20° (168 mm) and +25° (155 mm) set steering 0° and +5° |
| 5.93 s on | Clear at the limit +30°: steering +10°, toward the bottle (the wrap rule) |
| 6.4 s | Wheels stop against the bottle; run stopped by "right wheel stalled" at 6.9 s |

Causes:

1. **The pivot ends too early.** "Free straight ahead" means only that the
   narrow sensor beam missed the bottle, not that the robot body misses it.
   The bottle edge was about 13° off the heading, at 186 mm: about 42 mm to
   the side, less than the 56 mm half track.
2. **The fixed offset is too small near the obstacle.** To pass a point at
   distance d with side clearance c, the heading must be asin(c / d) away
   from it. For c = 100 mm: 24° at 250 mm, 37° at 168 mm, 40° at 155 mm.
   The fixed 20° drove at the bottle.
3. **The wrap starts too early.** "Clear at +30°" happened while the bottle
   was still beside the front of the robot; the sensor cannot see beside
   the robot, so the robot turned into it.

The stall stop worked as a collision stop. The text line for PIVOT →
AROUND was not printed: that change happens in `bp_reading()`, and
`bypass_test` printed state changes only after `bp_update()`.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_obstacle_bypass.c src/obstacle_bypass.c -o build/test_obstacle_bypass
./build/test_obstacle_bypass
```

[`tests/test_obstacle_bypass.c`](../tests/test_obstacle_bypass.c) checks: the
clearance angle and millimetre-to-count helpers, no free side, the pivot
direction and servo angle, the settle times, ignored readings without a
trigger, the pivot rule (free before the obstacle is seen does not count;
the 409 / 410 mm boundary for a 260 mm obstacle), the pass distance, PASS
→ WRAP by wheel travel (also from a negative count), hits in PASS and WRAP,
the rubber band, the run 1 hits, the steering clamp, TOO_NEAR ahead and at
the side, NO_RISE as no information, the fault streak, the line arming and
the junction rule, the time limits, the time-counter wrap, the word layout,
and the steering-angle to position table.

Result on 2026-10-30 (version 2): all tests pass. 30 rule changes in
`src/obstacle_bypass.c` each make a test fail; one (no rounding in the
millimetre-to-count helper) first passed and needed a test value that
rounds up. One change cannot be found by a test: the servo angle after "no
free side" is never sent, because the word is then STOP.

Version 1 (same day): 22 rule changes each made a test fail; four of them
first passed and needed new tests (a hit at exactly `hit_mm`, no wait after
a reading with no information, and the settle time and pivot time limit
without the wrap-safe subtraction).
