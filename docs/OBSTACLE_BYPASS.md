# Obstacle bypass

Status: first version, host-tested and built on 2026-10-30. Not yet tried on
the track.

This record describes how the robot drives around one obstacle on the line
and finds the line again. The operator's name for the plan is the "Blind
Bicycle Algorithm" (BBA). The first plan used arcs and wheel-count targets.
The operator and the assistant changed it to the simpler edge following
below: the sensor follows the edge of the obstacle, and the robot steers to
keep a fixed angle away from that edge.

## Assumptions

- One obstacle sits on the line. Nothing else is near it.
- The scan (±30°, [OBSTACLE_SCAN.md](OBSTACLE_SCAN.md)) sees past at least
  one side of it.
- The robot finds the line again after the obstacle. No odometry and no
  map are used.

## Steps

```text
 FOLLOW ──STOP (forward watch, 258 mm)──▶ wait 300 ms ──▶ SCAN
                                                          │ side S, obstacle mm
                                                          ▼
 FOLLOW ◀──line seen again── AROUND ◀──2 free readings── PIVOT
                               │                           │
                               └──── time limit / fault ───┴──▶ FAILED (stop)
```

| Step | What the robot does | Ends when |
| --- | --- | --- |
| STOP | Motors off (coast) | 300 ms have passed |
| SCAN | One 13-point scan. The PROFILE word picks the side S with the smaller free step (tie → more room at 30°, then LEFT). The obstacle is on the other side | The scan is done |
| PIVOT | Turns on the spot toward S at 20 % duty, servo straight ahead | 2 **free** readings in a row → AROUND. Free = no echo, or an echo at least 150 mm past the obstacle (the scan's own rule) |
| AROUND | Drives forward. The servo moves like a **rubber band** (below). The robot steers at the hit angle, moved `offset` toward S | The line is seen again → FOLLOW |
| FAILED | Stops the run | — |

Why "free" in PIVOT uses the scan rule: the robot stops about 258 mm from
the obstacle. If "clear" meant "at least 250 mm", the pivot would end at
once, still facing the obstacle.

### Rubber band

The servo looks for the edge of the obstacle on each reading:

| Reading at servo angle θ | Servo next | Steering angle |
| --- | --- | --- |
| **Hit** (echo nearer than `hit_mm`) | θ + `bounce` toward S | θ + `offset` toward S |
| **Clear** (no echo, or echo at `hit_mm` or farther) | θ + `step` toward the obstacle | No change |
| Clear at the servo limit on the obstacle side | Stays at the limit | Limit + `offset` toward S: the obstacle is beside the robot, so the robot turns back toward it (wraps around it) |
| No information (NO_RISE, STUCK_HIGH, no fault streak) | Same angle, read again | No change |

Angles are positive to the left, as in the scan. Example, bypass on the
left (obstacle on the right), offset 20°:

| Reading | Servo | Steering |
| --- | ---: | ---: |
| Start of AROUND | −5° | 0° |
| Clear at −5° | −10° | 0° |
| Hit at −10° | 0° | +10° (away from the obstacle) |
| Clear at 0°, −5°, … −30° | steps right | +10° |
| Clear at −30° (obstacle beside the robot) | −30° | −10° (back toward it) |

### Settings

The bench values are in [`src/bypass_test.c`](../src/bypass_test.c)
(`BYPASS`). They are first values, to tune on the track.

| Setting | Value | Notes |
| --- | ---: | --- |
| `hit_mm` | 250 | About the stop distance (258 mm). Larger keeps the robot farther away |
| `limit_deg` | 30 | Servo and steering limit each side (the scan range) |
| `step_deg` | 5 | Servo step while clear |
| `bounce_deg` | 10 | Servo step back after a hit |
| `offset_deg` | 20 | Start large; make it smaller if the robot goes too wide |
| `line_arm_ms` | 200 | The line must be lost this long before it can end the bypass. The robot starts on the line |
| `pivot_max_ms` | 4000 | Pivot time limit |
| `around_max_ms` | 10000 | AROUND time limit |

Servo settle time before each reading: 40 ms + 4 ms per degree moved
(60 ms for 5°, 80 ms for 10°). The ultrasonic driver also keeps 60 ms
between triggers.

## Steering

The robot is treated as a bicycle. The servo is the front wheel and the
rear axle is the back wheel. Operator measurements, 2026-10-30 (about):

| Size | Value |
| --- | ---: |
| Track W (wheel contact to wheel contact) | 111 mm |
| Wheelbase L (servo axis to rear axle) | 65 mm |

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
| Pivot time limit | Never 2 free readings ahead in 4 s |
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
40-bit record per bypass reading ([packet format](STEER_WIFI_LOG.md#packet-format)).
The receiver writes the readings as a table at the end of the run file.

### What to watch in the first tests

| Watch | If it happens |
| --- | --- |
| The rear corner touches the obstacle | Larger `offset_deg` or `hit_mm` |
| The robot goes very wide and misses the line | Smaller `offset_deg` |
| The robot crosses the line steeply and follows it back toward the obstacle | Smaller `offset_deg`, so it meets the line at a flatter angle |
| A steep crossing reads as a junction | The bypass already ignores readings with all three sensors on the line; check the log |
| The pivot never ends (wall behind the obstacle) | Move the obstacle, or check the scan |

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_obstacle_bypass.c src/obstacle_bypass.c -o build/test_obstacle_bypass
./build/test_obstacle_bypass
```

[`tests/test_obstacle_bypass.c`](../tests/test_obstacle_bypass.c) checks: no
free side, pivot direction, the settle times, ignored readings without a
trigger, the pivot free rule and its boundary (409 / 410 mm for a 260 mm
obstacle), NO_RISE as no information, the fault streak, the rubber band in
both directions, the steering clamp, TOO_NEAR ahead and at the side, the
line arming and the junction rule, the time limits, the time-counter wrap,
the word layout, and the steering-angle to position table.

Result on 2026-10-30: all tests pass. 22 rule changes in
`src/obstacle_bypass.c` each make a test fail. Four of them first passed
and needed new tests: a hit at exactly `hit_mm`, no wait after a reading
with no information, and the settle time and the pivot time limit written
without the wrap-safe subtraction.
