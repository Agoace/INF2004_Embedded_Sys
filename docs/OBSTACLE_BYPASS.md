# Obstacle bypass

Status: version 3.1, host-tested and built on 2026-10-30. Not yet tried on
the track. Version 3 (runs 5–8) passed the bottle but never saw it again
after the pivot. Version 2 (PASS straight, then WRAP) went around the bottle in
runs 3 and 4; the operator chose this simpler rule after run 4
([track tests](#track-tests)).

This record describes how the robot drives around one obstacle on the line
and finds the line again. The operator's name for the plan is the "Blind
Bicycle Algorithm" (BBA). The sensor follows the edge of the obstacle all
the time, and the robot steers at the edge of what the sensor sees as safe,
plus a fixed offset away from the obstacle. No wheel-count distance is used.

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
 FOLLOW ◀──line seen again── AROUND ◀──through the beam── PIVOT
                         (rubber band;     and out        (servo 30° toward
                          steer = safe                     the obstacle)
                          angle + offset)
                               │                           │
                               └──── time limit / fault ───┴──▶ FAILED (stop)
```

| Step | What the robot does | Ends when |
| --- | --- | --- |
| STOP | Motors off (coast) | 300 ms have passed |
| SCAN | One 13-point scan. The PROFILE word picks the side S with the smaller free step (tie → more room at 30°, then LEFT). The obstacle is on the other side | The scan is done |
| PIVOT | Turns on the spot toward S at 20 % duty. The servo looks 30° toward the obstacle | The obstacle has gone **through the beam and out**: one reading that is not free, then 2 free readings → AROUND. Free = no echo, or an echo at least 150 mm past the obstacle (the scan's own rule) |
| AROUND | Drives forward at base 20 %. The servo moves like a **rubber band** and the robot steers at the **safe angle + offset** (below) | The line is seen again → FOLLOW |
| FAILED | Stops the run | — |

Why the pivot rule is "through the beam and out" (run 1): "free straight
ahead" only means that the narrow beam misses the obstacle; in run 1 the
robot turned only about 13°. "Free at 30°" is no better: the run 1 scan
read no echo at +30° before the pivot. In runs 3 and 4 the pivot turned
about 28°.

### Rubber band and steering (AROUND)

The servo never stops ranging. Each reading at servo angle θ:

| Reading | Servo next | Safe angle |
| --- | --- | --- |
| **Hit** (echo nearer than `hit_mm`) | θ + `bounce` toward S | θ + `bounce` toward S |
| **Clear** (no echo, or echo at `hit_mm` or farther) | θ + `step` toward the obstacle (stays at the 30° limit) | θ |
| No information (NO_RISE, STUCK_HIGH, no fault streak) | Same angle, read again | No change |

**Steering = safe angle + `offset` toward S**, at most ±30°. AROUND starts
with the safe angle at the limit, where the pivot's last free readings
were.

**Not seen even at the limit** (a clear reading with the servo at the 30°
limit toward the obstacle): the steering turns one `step` (5°) more toward
the obstacle for each such reading, up to 30°, until the obstacle is in
the beam again (runs 5–8: the obstacle went out of view behind the beam
and a fixed −10° arc never brought it back). At one reading per 60 ms,
−10° to −30° takes about 0.24 s.

Example, bypass on the left (obstacle on the right), offset 20°:

| Reading | Servo | Safe | Steering |
| --- | ---: | ---: | ---: |
| Start of AROUND | −30° | −30° | −10° (back toward the obstacle) |
| Clear at −30° (not seen at the limit) | −30° | −30° | −15° (one step more) |
| Clear at −30° again | −30° | −30° | −20° |
| Hit at −30° | −20° | −20° | 0° |
| Clear at −20° | −25° | −20° | 0° |
| Hit at −25° | −15° | −15° | +5° (away) |

So the robot keeps the obstacle's edge about `offset` beyond its heading.
When the obstacle is out of view, the robot turns harder toward it until
the beam finds it again, so it follows the obstacle round and meets the
line.

### Settings

The bench values are in [`src/bypass_test.c`](../src/bypass_test.c)
(`BYPASS`, `STEER_BYPASS`). They are first values, to tune on the track.

| Setting | Value | Notes |
| --- | ---: | --- |
| `hit_mm` | 250 | About the stop distance (258 mm). Larger keeps the robot farther away |
| `offset_deg` | 20 | Steering beyond the safe angle, toward S. Larger: wider path; smaller: closer |
| `limit_deg` | 30 | Servo and steering limit each side (the scan range) |
| `step_deg` | 5 | Servo step while clear; also the extra steering for each reading not seen at the limit |
| `bounce_deg` | 10 | Servo step back after a hit |
| `line_arm_ms` | 200 | The line must be lost this long before it can end the bypass. The robot starts on the line |
| `pivot_max_ms` | 4000 | Pivot time limit |
| `around_max_ms` | 10000 | AROUND time limit |
| Base duty | 20 % | Slower than line following (25 %), operator choice after run 4 |

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
| Wheel diameter | 65 mm (version 2 only) |
| Robot width (widest point) | 160 mm (version 2 only) |

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
40-bit record per bypass reading
([packet format](STEER_WIFI_LOG.md#packet-format)). The receiver writes the
readings as a table at the end of the run file. The text shows each state
change with the steering angle and the last distance.

### What to watch in the first tests

| Watch | If it happens |
| --- | --- |
| The side or rear corner touches the obstacle | Larger `offset_deg` or `hit_mm` |
| The robot goes very wide and misses the line | Smaller `offset_deg` |
| The robot crosses the line steeply and follows it back toward the obstacle | Larger `offset_deg` (a gentler turn back) |
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
start steering of AROUND, the rubber band and the safe-angle steering in
both directions, the extra turn per reading not seen at the limit (and its
stop at 30°), the `hit_mm` boundary, the steering and servo clamps,
TOO_NEAR ahead and at the side, NO_RISE as no information, the fault
streak, the line arming and the junction rule, the time limits, the
time-counter wrap, the word layout, and the steering-angle to position
table.

Result on 2026-10-30 (version 3.1): all tests pass. 25 rule changes in
`src/obstacle_bypass.c` each make a test fail (version 3: 20, before the
turn-more rule).

Version 2 (same day): 30 rule changes each made a test fail. Version 1:
22, four of them after new tests (a hit at exactly `hit_mm`, no wait after
a reading with no information, and the settle time and pivot time limit
without the wrap-safe subtraction).
