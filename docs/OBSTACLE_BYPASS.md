# Obstacle bypass

Status: **shelved on 2026-10-30** at version 4.1, by the operator, after
track runs 15–18 ([open problems](#open-problems-when-shelved)). The code
stays in the repository and its host test passes; `bypass_test` is the
last firmware on the robot. Version 4 (runs 12–14) spiralled in: it turned back after about
25 mm of each leg. Earlier versions ([track tests](#track-tests)): version 2 drove a
fixed straight and arc (runs 3–4); version 3 steered at the last safe angle
+ 20° (runs 5–8); version 3.1 also turned harder when the obstacle was out
of view, and its front corner hit a box (run 11).

This record describes how the robot drives around one obstacle on the line
and finds the line again. The operator's name for the plan is the "Blind
Bicycle Algorithm" (BBA). The robot only drives forward when it knows where
the obstacle's edge is. When it loses the obstacle, it stops, turns on the
spot to find it again, and finds the edge with the servo. Then it steers so
that the front of the robot passes the edge with a fixed clearance, and it
drives that path to its closest point (by the wheel counts) before it may
turn back to look again.

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
                                                        PIVOT (toward S, until the
                                                          │    obstacle has gone
                                                          │    through the beam and out)
                                                          ▼
          ┌──────────────── AROUND ─────────────────────────────────┐
          │  DRIVE: steer = edge + clear angle ──lost at 30°──▶ TURN │
          │    ▲    (servo: rubber band)               (on the spot, │
          │    │                                      toward the    │
          │    └── EDGE: stopped; servo steps ◀──seen── obstacle)   │
          │         toward S until 2 free readings                  │
          └───────────────────────── line seen ─────────────────────┘
                                         ▼
                                      FOLLOW
```

| Step | What the robot does | Ends when |
| --- | --- | --- |
| STOP | Motors off (coast) | 300 ms have passed |
| SCAN | One 13-point scan. The PROFILE word picks the side S with the smaller free step (tie → more room at 30°, then LEFT). The obstacle is on the other side | The scan is done |
| PIVOT | Turns on the spot toward S at 20 % duty. The servo looks 30° toward the obstacle | The obstacle has gone **through the beam and out**: one seen reading, then 2 free readings → AROUND, DRIVE |
| AROUND | DRIVE, TURN and EDGE (below), at base 20 % | The line is seen again → FOLLOW |
| FAILED | Stops the run | — |

**Seen** and **free** use the scan's rule: a valid echo nearer than the
obstacle distance from the scan + 150 mm (`free_mm`) is seen; no echo, or
an echo at `free_mm` or farther, is free.

Why the pivot rule is "through the beam and out" (run 1): "free straight
ahead" only means that the narrow beam misses the obstacle; in run 1 the
robot turned only about 13°.

### AROUND phases (version 4)

| Phase | Motors | Servo | Reading → what happens |
| --- | --- | --- | --- |
| **DRIVE** | Forward, steering angle | Rubber band | **Seen** at θ, d mm: steer = θ + clear angle(d) toward S, and a new leg starts; servo jumps `bounce` (10°) toward S. **Free** at θ: servo steps `step` (5°) toward the obstacle; steering does not change. **Free at the 30° limit** toward the obstacle: lost → TURN, **but only when the leg is done**; before that the servo reads the limit again |
| **TURN** | Pivot on the spot toward the obstacle (20 %) | Stays at the limit | Seen → EDGE (servo steps 5° toward S). Free → keep turning |
| **EDGE** | Stopped | Steps 5° toward S per seen reading | Free → read the same angle again. **2 free at one angle** = the edge e: steer = e + clear angle(last d), new leg → DRIVE. Seen at the 30° limit toward S → edge = the limit → DRIVE |

AROUND starts in DRIVE with the edge at the limit (where the pivot's last
free readings were) and the pivot's last seen distance.

**Leg** (version 4.1, runs 12–14): along the steering direction, the
sensor end is nearest to the point (θ, d) after √(d² − c²) mm. The leg is
that distance in mean wheel counts (12.41 counts per mm), counted from the
first `bp_update()` after the leg starts. Turning back before the end of
the leg (version 4: after about 25 mm) undid the turn away and the robot
spiralled in.

| d | 250 mm | 230 mm | 200 mm | 180 mm | ≤ 150 mm |
| --- | ---: | ---: | ---: | ---: | ---: |
| Clear angle (c = 150) | 37° | 41° | 49° | 57° | 90° |
| Leg (c = 150) | 200 mm | 174 mm | 132 mm | 99 mm | 0 |

**Clear angle(d)** = the smallest whole angle a with sin a ≥ c / d, where c
is `clearance_mm` (150 = 80 mm half robot width + 70 mm, because the front
corner is beside the sensor; version 4 used 110). In the
bicycle model the servo end of the robot moves in the steering direction,
so steering at the edge + a passes the edge point with c to spare. Near
obstacles get a larger angle:

| d (c = 110, version 4) | 300 mm | 250 mm | 220 mm | 200 mm | 183 mm | 150 mm | ≤ 110 mm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Clear angle | 22° | 27° | 30° | 34° | 37° | 48° | 89–90° |

The steering is clamped to ±30°.

Why version 4 (runs 5–11): the servo only sees ±30° (the mount allows
about ±36°). When the obstacle is beside the robot it is out of view, so
steering toward it "until seen" while driving forward (version 3.1) is a
blind turn: the box corner was hit. Version 4 only turns toward the
obstacle on the spot. A fixed offset (version 3) is also a problem: if the
obstacle is always kept within 30° of the nose and the robot drives forward,
it spirals in. The clear angle grows as the obstacle gets nearer.

Why two free readings in EDGE (run 11): a slanted flat face can miss an
echo, and the box gave no echo at −20° to −30° at about 180 mm.

### Settings

The bench values are in [`src/bypass_test.c`](../src/bypass_test.c)
(`BYPASS`, `STEER_BYPASS`). They are first values, to tune on the track.

| Setting | Value | Notes |
| --- | ---: | --- |
| `clearance_mm` | 150 | Edge to the sensor's path. Larger: wider path. Version 4: 110 (corner hit) |
| `limit_deg` | 30 | Servo and steering limit each side (the scan range) |
| `step_deg` | 5 | Servo step: DRIVE toward the obstacle, EDGE toward S |
| `bounce_deg` | 10 | DRIVE: servo step toward S after a seen reading |
| `line_arm_ms` | 200 | The line must be lost this long before it can end the bypass. The robot starts on the line |
| `pivot_max_ms` | 4000 | Time limit of the pivot and of each TURN |
| `around_max_ms` | 20000 | AROUND time limit (version 4 stops to look, so it is slower) |
| Base duty | 20 % | Slower than line following (25 %), operator choice after run 4 |

Servo settle time before each reading: 40 ms + 4 ms per degree moved
(40 ms for a reading again, 60 ms for 5°, 80 ms for 10°, 160 ms for the
first 30° move). The ultrasonic driver also keeps 60 ms between triggers.

## Steering

The robot is treated as a bicycle. The servo is the front wheel and the
rear axle is the back wheel. Operator measurements, 2026-10-30 (about):

| Size | Value |
| --- | ---: |
| Track W (wheel contact to wheel contact) | 111 mm |
| Wheelbase L (servo axis to rear axle) | 65 mm |
| Wheel diameter | 65 mm (versions 2 and 4.1: wheel counts to mm) |
| Robot width (widest point) | 160 mm (versions 2, 4 and 4.1: clearance) |
| Robot length (overall) | 190 mm (given after the bypass was shelved; not used in the code) |

See the main [robot geometry record](CALIBRATION_RESULTS.md#robot-geometry) for sources, calculated values, and limits.

At steering angle δ the rear axle turns on radius R = L / tan δ. Each wheel
then runs at speed × (1 ∓ (W/2)·tan δ / L). `line_steer` with gain 100 %
gives speed × (1 ± position / 100). So:

position = −100 · (W/2) · tan δ / L = −85.4 · tan δ (negative = left)

| δ | 0° | ±5° | ±10° | ±15° | ±20° | ±30° |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| position | 0 | ∓7 | ∓15 | ∓23 | ∓31 | ∓49 |

`bp_steer_position()` uses a 31-entry table of tan × 1000 (the RP2040 has
no floating-point hardware) and rounds. At 30° the inner wheel runs at
51 % of the base speed above `min_pct`.
The bypass uses base 20 %, gain 100 %, no damping, min 6 % (the duty where
a turning wheel stops; run 3), max 50 %, right trim +1 % (`STEER_BYPASS`).

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
| Pivot time limit | The obstacle did not go through the beam and out in 4 s, or one TURN did not see it in 4 s |
| Line not found | AROUND ran 20 s |
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
and phase change with the steering angle, the servo angle and the last
distance.

### What to watch in the first tests

| Watch | If it happens |
| --- | --- |
| A corner touches the obstacle | Larger `clearance_mm` |
| The robot goes very wide and misses the line | Smaller `clearance_mm` |
| TURN never sees the obstacle (time limit) | A slanted face gives no echo: check the bypass table |
| Very slow progress (many TURN and EDGE cycles) | Expected for a first version; note the time per cycle |
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

### Run 3: 2026-10-30, version 2 (around the bottle, wide)

Log: `build/steer_logs/run_20261030-144234_bootf169_run3.txt` (not in Git).
Run 2 on the same boot was a 30 s line-following run with no obstacle.
Operator: "it works kinda", but the path went across about half the
circuit.

| Time | What happened |
| ---: | --- |
| 3401 ms | STOP at 191 mm |
| 5.5 s | Scan: bottle at −20° to 0°, nearest 182 mm. Free step R 5, L 1 → bypass LEFT |
| 5.53–5.94 s | PIVOT left: bottle seen 3 times at −30° (185–187 mm), then 2 free. Mean wheel travel 342 counts: about 28° |
| 5.94–7.74 s | PASS: 3612 counts (291 mm) straight, no hits |
| 7.74–10.24 s | WRAP at −20°: duties 29/22 %, counts 117/91 per 50 ms. The robot turned only about 50° in 412 mm |
| 10.24 s | DONE: the line found, then a junction straight and a search; line following again |
| 18.3 s | A second STOP at the bottle after a lap; bypass LEFT; DONE during PASS at 21.5 s (line found after 640 ms) |
| 24.7 s | Search failed |

Cause of the wide path: the steering used `min_pct` 12 % as speed zero.
That is the duty at which a stopped wheel starts; a turning wheel slows to
zero only at about 5.6 % (speed table: 20 % → 74, 25 % → 101, 33 % → 141
counts per 50 ms). The planned 20° WRAP is a 179 mm turn radius (rear axle);
the robot drove a 467 mm radius, 2.6 times wider. The 30° steering limit
was not the cause.

Fix (run 4): `STEER_BYPASS.min_pct` 6. A 20° WRAP is then 31/20 % (about
131/74 counts, speed ratio 1.77 for a planned 1.89); 30° is 34/17 %.
Line following keeps `min_pct` 12 (tuned on the track).

### Run 4: 2026-10-30, version 2 with min 6 %

Log: `build/steer_logs/run_20261030-145054_bootcda6_run2.txt` (not in Git).
Run 1 of the same boot found a line 220 ms into PASS (the right sensor, at
25.3 s) and then failed a search.

| Time | What happened |
| ---: | --- |
| 8301 ms | STOP at 197 mm; scan nearest 185 mm, bypass LEFT |
| 10.43–10.96 s | PIVOT: bottle seen at −30° (192–241 mm), then 2 free |
| 10.96–13.07 s | PASS: 4196 counts (338 mm) straight |
| 13.07–14.28 s | WRAP at −20°: duties 31/20 %; 45° in 191 mm, a 243 mm radius (planned 179 mm; run 3: 467 mm) |
| 14.28 s | DONE: the line found; line following again |

Operator review: turn as much as the sensor shows is safe plus the offset,
drive slower, and keep the sensor ranging the obstacle all the time (the
first plan). Version 3 does this: PASS and WRAP are removed, the rubber
band runs during all of AROUND, the steering is the safe angle + offset,
and the base duty is 20 %.

### Runs 5–8: 2026-10-30, version 3 (offset 20°, base 20 %)

Logs: `build/steer_logs/run_20261030-150809_boot3759_run1.txt` to
`run_20261030-150947_boot3759_run4.txt` (boot 3759, runs 1–4; not in Git).
Path from the wheel counts (dead reckoning, start at the STOP point, line
along x). The bottle centre is taken as 65 + nearest + 33 mm ahead of the
rear axle.

| Run | STOP | Side | AROUND | Path | Closest robot centre to bottle centre | End |
| --- | ---: | --- | ---: | --- | ---: | --- |
| 5 | 201 mm | LEFT | 5.85 s | x 596, y 262 mm; heading 45° → 7° | 173 mm | Line found; time limit later |
| 6 | 211 mm | LEFT | 0.79 s | x 69, y 70 mm | — (before the bottle) | A line under L at 6.0 s; search failed |
| 7 | 201 mm | LEFT | 0.62 s | x 51, y 55 mm | — (before the bottle) | A line under R at 12.2 s; search failed |
| 8 | 221 mm | LEFT | 3.84 s | x 448, y 172 mm; heading 39° → 3° | 149 mm | Line found; search failed later |

Findings:

1. **The rubber band never started.** All AROUND readings in the four
   runs were NO_RETURN at −30°. The pivot ends when the bottle leaves the
   −30° beam; then, while the robot drives forward, the bottle moves
   further behind (its bearing grows). The robot must turn toward it faster
   than that to see it again: about a 440 mm radius or less at 220 mm and
   30°. AROUND was a fixed −10° arc.
2. **The −10° arc was wide.** Duties 22/19 % gave 82/71 counts per 50 ms
   (ratio 1.15; planned 1.35). From the wheel counts the turn radius was
   about 800 mm (planned 369 mm).
3. No contact: the robot body (80 mm half width) passed the bottle
   (about 33 mm radius) with about 35–60 mm to spare (dead reckoning; an
   estimate).
4. Runs 6 and 7 found a line about 100 mm after the pivot, before the
   bottle, and the line search failed after it. The line at that point is
   not known (another part of the track near the bottle?).

Proposal: `offset_deg` 10, so clear at the limit steers −20°. Operator
review: the sensor must keep looking for the obstacle during the arc, and
when it cannot see it even at 30° the robot should turn more. Version 3.1
does this (not seen at the limit: 5° more steering per reading); the
offset stays 20°.

### Runs 9–11: 2026-10-30, version 3.1 (turn more when not seen)

Logs: `build/steer_logs/run_20261030-152229_boot8a6c_run1.txt`,
`run_20261030-152326_boot8a6c_run2.txt`, `run_20261030-152605_boot8a6c_run3.txt`
(boot 8a6c; not in Git).

| Run | Obstacle | AROUND | Hits in AROUND | End |
| --- | --- | ---: | --- | --- |
| 9 | Bottle | 12.83–16.38 s | 1 (117 mm at −30°) | Line found (steer −30); time limit |
| 10 | Bottle | 11.19–14.65 s; again 21.59–22.19 s | 1 (159 mm); 0 | Line found both times; time limit |
| 11 | Box | 9.32 s – | 1 (183 mm at −30°, 10.03 s) | **Front corner hit the box**; left wheel stalled at 12.07 s |

Run 11 readings: not seen at −30° from 9.31 s, so the steering went to
−30° in 0.24 s. One echo at 10.03 s (183 mm) gave steer 0, then no echo at
−20°, −25° and −30°, and the steering went back to −30° until the stall.
The box face was close but gave no echo at a slant.

Operator review: the turn-more rule corrects, but it turns back toward the
obstacle too eagerly. Operator proposal: when the obstacle is lost at the
limit, turn on the spot toward it until it is seen; then step the servo
back until it is not seen (the edge); then steer at that edge + offset.
The assistant pointed out the spiral (a fixed offset with the obstacle
always within 30° of the nose) and the slanted face; the operator chose
option B: the offset is the clear angle for 110 mm at the measured
distance, and EDGE needs two free readings. That is version 4.

### Runs 12–14: 2026-10-30, version 4 (spiral in, corner hit)

Logs: `build/steer_logs/run_20261030-155756_boot1597_run1.txt`,
`run_20261030-155915_boot1597_run2.txt`, `run_20261030-160004_boot1597_run3.txt`
(boot 1597; not in Git). All three ended with "left wheel stalled": the
front corner against the obstacle.

Distance at each EDGE (first seen reading after TURN):

| Run | Cycle 1 | 2 | 3 | 4 | 5 | 6 | 7 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 12 | 230 | 204 | 179 | 136 | 131 | 83 | — |
| 13 | 248 | 205 | 176 | 157 | 119 | 92 | — |
| 14 (stopped at 112 mm) | 126 | 102 | 83 | — | — | — | — |

Findings (run 12):

1. Each cycle took about 0.5–0.8 s: DRIVE about 190 ms (three readings,
   about 25 mm), TURN 120–300 ms, EDGE 120–250 ms.
2. In DRIVE the obstacle was free at −20°, −25° and −30° within about
   200 ms, although EDGE had just seen it at −25°: the turn away moves it
   out of view at once, and TURN then turns the robot back. From the wheel
   counts the heading went 0° → −46° (toward the obstacle) in 4 s.
3. The edge is found only 5–10° from where TURN first saw it.
4. The clearance is from the sensor. The robot corner is about 80 mm to
   the side of the sensor, so a 110 mm sensor clearance is not body
   clearance when the obstacle is beside the front.

Operator: "not turning away enough". Version 4.1 (operator chose both):
A, drive each leg to its closest point before TURN is allowed; B,
clearance 150 mm.

### Runs 15–18: 2026-10-30, version 4.1 (leg rule, clearance 150)

Logs: `build/steer_logs/run_20261030-160955_boota411_run1.txt`,
`run_20261030-161118_boota411_run2.txt`, `run_20261030-161226_boota411_run3.txt`,
`run_20261030-161302_boota411_run4.txt` (boot a411; not in Git). Box.

| Run | First leg | TURN 1 | EDGE 1 | TURN 2 | EDGE 2 | End |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 15 | 2483 counts (200 mm), 1.8 s | 85° | 201 mm | — | — | Line found at 16.2 s; right wheel stalled at 16.9 s |
| 16 | 2334 counts (188 mm), 1.6 s | 86° | 166 mm | 64° | 82 mm | Close cycles at 79–109 mm, leg 0; line found at 10.8 s; search failed |
| 17 | 2334 counts, 1.6 s | 82° | 198 mm | 57° | 88 mm | Left wheel stalled at 9.2 s (contact) |
| 18 | 2383 counts, 1.7 s | 84° | 186 mm | — | — | Line found at 7.2 s; search failed |

TURN angles from the wheel counts.

Findings:

1. The leg rule worked: the first leg was about 190–200 mm (version 4:
   about 25 mm).
2. After a full leg the obstacle is far out of view, so each TURN turned
   the robot back by 57–86° on the spot. Each TURN brought the sensor
   much nearer: about 245 → 166–201 mm after TURN 1, about 170 → 82–88 mm
   after TURN 2. A pivot on the spot swings the front of the robot (the
   sensor and the front corners) toward an obstacle beside it.
3. The line was found in three runs, but the line follower lost it again
   at once (the robot arrives at a steep angle, with steering +30°).

### Open problems when shelved

| Problem | Seen in |
| --- | --- |
| The servo sees only ±30° (mount limit about ±36°), so the obstacle is out of view when it is beside the robot | Runs 5–18 |
| Turning back on the spot swings the front toward the obstacle | Runs 15–18 |
| Flat faces at a slant give no echo | Runs 11, 12 |
| The robot meets the line at a steep angle and the line follower loses it | Runs 15, 16, 18 |
| Version 2 (fixed straight + arc by wheel counts) went around the bottle with no contact | Runs 3, 4 |

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_obstacle_bypass.c src/obstacle_bypass.c -o build/test_obstacle_bypass
./build/test_obstacle_bypass
```

[`tests/test_obstacle_bypass.c`](../tests/test_obstacle_bypass.c) checks: no
free side, the pivot direction and servo angle, the settle times, ignored
readings without a trigger, the pivot rule (free before the obstacle is
seen does not count; the 409 / 410 mm boundary for a 260 mm obstacle), the
clear-angle table and its boundaries, the start steering of AROUND, the
leg length (integer square root) and the counts per mm, the leg rule (not
lost before the leg ends, the 3463 / 3464 count boundary, a seen reading
starts a new leg, a new leg counts from the next update), a full
DRIVE → TURN → EDGE → DRIVE cycle in both directions (the TURN direction,
the EDGE steps, the read-again rule and its restart, the edge steering, the
rubber band and the seen / free boundary in DRIVE), EDGE seen across the
whole view, the steering clamp, the TURN time limit, TOO_NEAR ahead and at
the side, NO_RISE as no information, the fault streak, the line arming and
the junction rule, the time limits, the time-counter wrap, the word layout,
and the steering-angle to position table.

Result on 2026-10-30 (version 4.1): all tests pass. 44 rule changes in
`src/obstacle_bypass.c` each make a test fail.

Version 4: 35 rule changes. Version 3.1: 25. Version 3: 20.
Version 2 (same day): 30 rule changes each made a test fail. Version 1:
22, four of them after new tests (a hit at exactly `hit_mm`, no wait after
a reading with no information, and the settle time and pivot time limit
without the wrap-safe subtraction).
