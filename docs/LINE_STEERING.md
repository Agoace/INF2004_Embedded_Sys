# Line steering

This record describes proportional line steering on the `yx-line_tracing`
branch: the [normalised line position](LINE_POSITION.md) sets the left and
right wheel duty.

- Code: [`src/line_steer.c`](../src/line_steer.c), [`src/line_steer.h`](../src/line_steer.h)
- Host unit test: [`tests/test_line_steer.c`](../tests/test_line_steer.c)
- Bench test: [`src/steer_test.c`](../src/steer_test.c)
- Shared bench parts: [`src/drive.c`](../src/drive.c) (motors and encoders),
  [`src/line_sensors.c`](../src/line_sensors.c) (IR reads),
  [`src/line_spin.c`](../src/line_spin.c) (spin calibration)

## How it works

Steering works on wheel speed, not raw duty. A loaded wheel only starts to
turn at about `min_pct` (12 %), so `min_pct` is speed zero:

```text
speed = base − min
turn  = gain × position + kd × (position − previous position)
left  = speed + speed × turn / 10000        (wheel speeds)
right = speed − speed × turn / 10000
duty  = min + wheel speed, or 0 if the wheel speed is 0 or less
```

The correction is a share of the base speed, so it is the same ratio at
any speed. Above the start duty our motors are close to linear
([unloaded speed test](CALIBRATION_RESULTS.md#unloaded-motor-speed-at-5-v-vin)).

| Position | Meaning | Wheels L / R (base 20 %, gain 150 %, min 12 %, max 30 %) | Before (raw duty, gain 100 %) |
| ---: | --- | --- | --- |
| 0 | Line under the centre sensor | 20 / 20: straight | 20 / 20 |
| +25 | Line a little right of centre | 23 / 17 | 25 / 15 |
| +50 | Line between centre and right | 26 / 14 | **30 / 0** |
| +66 | | 28 / 0: inner wheel stops | 30 / 0 |
| +100 | Line under the right sensor | 30 / 0: pivot right | 30 / 0 |

Gain 100 % means: with the line under an outer sensor, the inner wheel
stops. Gain 150 % stops it at ±67.

Why: with the raw-duty rule a duty below `min_pct` was set to 0, so the
inner wheel dropped from 12 % to 0 at position 41. Run 4 below showed the
steering jumping between "straight" (20/21, 36 % of the time) and "pivot"
(0/30, 15 %), with about 3 jumps of 10 % duty or more each second. Now the
inner wheel slows by at most 1 % per step of position until it stops
(13 % to 0, where the wheel hardly turns).

## Trim

`right_trim_pct` is added to the duty of the right wheel when it turns,
before the limits (a negative value adds to the left wheel). The team's drive-straight trim is
+1 % on the right motor. It agrees with the measurement: at 20 % duty the
left wheel ran about 7 % faster, and because a wheel only starts to turn
at about 6 % duty, one more percent at 20 % is about 7 % more speed.

## Limits

Applied in this order, after the trim:

| Limit | Rule | Why |
| --- | --- | --- |
| `max_pct` | A wheel above it is set to it; the other wheel is lowered by the same amount | Keeps the turn when the outer wheel saturates |
| `min_pct` and no reverse | A wheel below `min_pct` (for example after the max limit lowered it) is set to 0 | A loaded wheel below about 12 % does not turn; it only uses current. No pivot on the spot while following |

## The D part

`kd_pct` reacts to the change of position since the last update (10 ms).
It works against fast movement of the line, which reduces zig-zag. While
the line moves back toward the centre, D turns against the turn, so the
robot does not swing past the line. In a steady curve the position does
not change, so D does not weaken the turn.

The change is limited to ±25 per update (`STEER_MAX_CHANGE`): a jump of
the position between sensors gives only a small kick.

> Update 2026-10-29: `steer_test` uses kd 200 %. In run 6 (D off, base
> 25 %, max 50 %) the robot swung from 0/50 to 50/0 and back for about
> 0.8 s after each junction. Example with the `steer_test` settings: line
> at −30 gives 17/34; the next update at −12 (line coming back fast)
> gives 27/24, so the robot already turns back; at −12 again, 22/29.
> Damping from the wheel counts (turn rate) was not used: it also works
> against the steady turn in a curve, and run 5 lost the line because the
> turn was too weak.

## Straight across a junction

`steer_straight()` gives both wheels the base duty (with the trim) and
forgets the previous position, so the next update has no D part.
`steer_test` uses it while `lp_junction_hold()` is true (up to 300 ms
after all three sensors saw the line), for at most 450 counts of mean wheel
travel (since 2026-10-29, after run 7; a distance does not grow with the
speed). In run 6 the side arm of the T
junction under the right sensor pulled the robot right for about 200 ms
before the swing began. The log shows these updates as mode `STRT`.

The [bench data](LINE_POSITION.md#bench-test) showed that the position is
mostly 0 or ±100: when only the centre sensor sees the line the position
stays 0, and the overlap zones between sensors are narrow. So the robot
can drift inside the centre zone without correction, then gets a large
correction at its edge. Expect some weave.

## Data representation

Steer word: 14 bits used.

| Bits | Field |
| --- | --- |
| 0–6 | Left duty, 0–100 % |
| 7–13 | Right duty, 0–100 % |
| 14–15 | Spare |

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
   tests/test_line_steer.c src/line_steer.c -o build/test_line_steer
./build/test_line_steer
```

It checks straight, both turn directions, the pivot at ±100, rounding, the
ratio at another base, gain 50, the max limit in both directions, the
100 % ceiling, the min limit, the D part (first update, moving, still,
moving back, reset, the ±25 change limit, braking while the line comes
back), straight across a junction (base and trim, no D before or after),
the trim (straight, turning, not on a stopped wheel, max limit, negative
trim), speed-based steering (min as speed zero, gain 150 % points, and no
step of more than 1 % in the inner wheel for positions 1 to 100 until it
stops), and the word layout.

Result on 2026-10-27: all tests pass. Six deliberate rule changes (turn
direction swapped, max without lowering the other wheel, no min limit, D
on the first update, no 100 % ceiling, no rounding) each make the test
fail. Three trim rule changes (no left trim, no right trim, negative trim
on the right wheel) each make the test fail.

Speed-based steering, 2026-10-29: all tests pass. Four rule changes (no
min offset, speed from base instead of base − min, trim on a stopped
wheel, the old 12 % cliff) each make the test fail. The trim rule change
first passed, because with min 12 the extra 1 % is set back to 0; a test
with min 0 was added.

D limit and straight, 2026-10-29: all tests pass. Five rule changes (no
upper change limit, no lower change limit, D sign swapped, no reset
before or after the straight word) each make the test fail.

## Bench test

```sh
cmake --build build --target steer_test
picotool load -f -x build/steer_test.uf2
```

| Control | Action |
| --- | --- |
| GP21 | Spin calibration, then line following after 1 s; press again to stop |
| GP20 | Spin calibration only |

Settings: base 33 %, gain 200 %, kd 200 %, min 12 % (speed zero), max 75 %,
right trim +1 %. Max is an operator choice (30 %, then 50 %, then 75 %
on 2026-10-29); base 33 % is chosen so the outer wheel reaches max at
±100: 12 + 3 × 21 = 75.

| Position | Left / right duty (base 33 %) |
| ---: | --- |
| 0 | 33 / 34 |
| +10 | 37 / 30 |
| +25 | 44 / 23 |
| +40 | 50 / 17 |
| +50 | 54 / 0 |
| +100 | 75 / 0 |

While following, the encoders give the line position module its side
hint: left minus right wheel counts since the line was last centred; more
than +20 means the robot turns right, so the line moves left.

On LINE_LOST the robot searches on the spot at 20 % duty: an eighth of a
wheel turn toward the last side (left if unknown), then through the start
to an eighth on the other side. LINE_FOUND resumes following. This is a
bench stand-in for the Vehicle Controller LINE_SEARCH state.

A run stops on GP21, a failed search, a stalled wheel (no encoder count
for 500 ms), or the 60 s time limit. The log keeps the last 20 s of the
run, and the summary covers the whole run. The Wi-Fi live log covers the
whole run.

> Update 2026-10-29: one button (GP21) now calibrates and starts the run,
> and the run stops after 60 s (operator choices). From run 3 to run 6
> there was no time limit, and GP20 then GP21 were pressed.

This is a driving test, so use 5 V VIN with USB disconnected
([USB power safety](USB_POWER_SAFETY.md)). The run is logged every 50 ms
(position, duties, raw readings, side and its source, hint, wheel counts)
and saved to the last two flash sectors when the robot stops, so a restart
does not lose it. Connect USB and open the serial port: the summary and
log print once, then the summary repeats every 5 s. The same text and log
also go over Wi-Fi ([steering Wi-Fi log](STEER_WIFI_LOG.md)).

Procedure:

1. Flash with USB. Turn on VIN, then disconnect USB.
2. Put the robot on the line, centre sensor over it. Press GP21 and move
   your hand away. The robot spins to calibrate, stops on the line, and
   starts 1 s later.
3. It stops after 60 s, or press GP21 again.
4. After it stops, connect USB and read the report.

### Run 1: 2026-10-27, VIN

Base 25 %, max 50 %. The robot turned away from the line and lost it after
197 ms:

| t (ms) | Position | Left / right duty |
| ---: | ---: | --- |
| 0 | −33 | 17 / 33 |
| 50 | −96 | 0 / 49 |
| 100 | −100 | 0 / 50 |

The spin's first step turns left as coded, so the motor sides are correct.
The operator confirmed that the sensor sides were swapped: GP26 is the
robot's right sensor and GP28 its left. Fixed in
[`src/line_sensors.c`](../src/line_sensors.c). A second run also turned the
wrong way; its report was lost when the Pico restarted, so the run is now
saved to flash.

Settings after run 1: base 20 %, max 30 % (operator choices).

### Run 2: 2026-10-27, VIN

Sensor sides fixed. 605 ms, LINE_LOST with unknown side. The position was
0 for 88 % of the run: only the centre sensor saw the line, so there was
no steering. The left wheel ran 881 counts and the right 820 at the same
duty, so the robot drifted right and the line went into the gap between C
and L, where no sensor sees it. See
[the gaps between the sensors](LINE_POSITION.md#the-gaps-between-the-sensors)
for the measurement and the side tracking that followed.

### Run 3: 2026-10-27, VIN

Side tracking, local search, spin stop at the centre peak (906 per
mille). No trim yet.

| Item | Result |
| --- | --- |
| Duration | 20 s: stopped by the time limit, not by a lost line |
| Wheels | L 29192, R 28543 counts (about 11.5 wheel turns) |
| LINE_LOST | 3 times, the line was found again 3 times |
| Mean \|pos\| | 21; 0 for 37 % of the steering updates |
| Side source | Outer sensor 43 %, gap rise 12 %, wheel hint about 1 % |

- Most corrections were to the left (line in the C–L gap): the left wheel
  is faster at the same duty, so the robot drifts right.
- At 14.0 s the robot crossed a junction (all sensors high). 150 ms later
  a side arm of the junction was still near L (+54 counts) while the line
  had gone right: the gap rise gave LEFT, the search turned left first,
  and the line was found only on the way back. The gap rise is now not
  used for 300 ms after a junction
  ([junction hold](LINE_POSITION.md#the-gaps-between-the-sensors)).
- Next run: right trim +1 %.

### Run 4: 2026-10-29, VIN, Wi-Fi log

Trim +1 %, side memory through the centre, raw-duty steering (gain 100 %,
min 12 %). The previous run (91 s, 10 LINE_LOST, all found again) gave
the "before" numbers from its last 20 s.

| Measure | Before side memory | Run 4 (66 s) |
| --- | ---: | ---: |
| One wheel stopped | 17.1 % | 15.4 % |
| Position 0 | 25.4 % | 22.0 % |
| Mean change of the inner duty per 50 ms | 4.08 | 3.39 |
| Jumps of 10 % duty or more | 37 per 10 s | 31 per 10 s |
| LINE_LOST | — | 5, all found again |

- The operator saw jerky steering in tight turns, "jumping between two
  steering points". The log agrees: 20/21 for 36 % and 0/30 for 15 % of
  the time, and nothing between 12/29 and 0/30.
- The side memory helped a little; the remaining jumps come from the
  12 % to 0 step. Speed-based steering (above) was added after this run.

### Run 5: 2026-10-29, VIN, Wi-Fi log

Speed-based steering, gain 150 %, max 30 %. 98 s, stopped by GP21. The
Wi-Fi log received 1927 of 1962 entries.

| Measure | Run 4 | Run 5 |
| --- | ---: | ---: |
| Jumps of 10 % duty or more | 31 per 10 s | 26.5 per 10 s |
| One wheel stopped | 15.4 % | 12.9 % |
| Mean change of the inner duty per 50 ms | 3.39 | 2.85 |
| LINE_LOST | 5 in 66 s | **30 in 98 s**, all found again |
| \|pos\| 51–99 | about 7 % | 18 % |

- Smoother, but too weak in tight curves: at position −50 the wheels were
  14/27, the line went out past the outer sensor, and the search turned
  back to it (about 0.1 s). The searches came in groups at the tight
  curves.
- The track has one ramp per lap (about 10 s). On every climb the right
  sensor read 1200–1600 raw (about 200 on the flat) for 2–4 s. There was
  no LINE_LOST on the ramps, and the wheel speed did not change at 20 %
  duty. The floor learning followed the higher right reading, so it was
  not taken as the line; on the ramp the right sensor is less sensitive.
- Next run: gain 200 % (inner wheel stops at ±50), max 50 %, base 25 %
  (speed above the start duty 13 instead of 8, about 60 % faster), on a
  lap without the ramp.

### Run 6: 2026-10-29, VIN, Wi-Fi log

Base 25 %, gain 200 %, max 50 %. 59 s, stopped by GP21. The Wi-Fi log
received 1157 of 1177 entries. At about 51 s the left wheel came off.

| Measure (until the wheel came off, 51 s) | Run 5 | Run 6 |
| --- | ---: | ---: |
| Mean wheel speed | 72 counts per 50 ms | 99 counts per 50 ms |
| LINE_LOST | 2.7 per 10 s | 1.4 per 10 s, all found again |
| One wheel stopped | 12.9 % | 20.3 % |
| Jumps of 10 % duty or more | 26.5 per 10 s | 51.3 per 10 s |

- About 38 % faster, with half the line losses.
- The duty jumps are larger because the duty range is larger: at
  49.8 s the wheels went from 0/50 to 50/0 in 100 ms.
- From 51.3 s the sensor readings did not change (the robot did not
  move) while the left wheel turned at 214 counts per 50 ms at 38 %
  duty. On the track that duty gives about 129, so the motor turned
  with no wheel load. The stall guard did not stop the run, because the
  motor turned. At 55 s the robot was lifted.
- All 30 swings from one side (|position| ≥ 50) to the other within
  300 ms started less than 1.5 s after one of the 7 junctions. Run 5 (max
  30 %) had none. Next run: straight across junctions and kd 200 %.

### Run 7: 2026-10-29, VIN, Wi-Fi log

Straight across junctions, kd 200 %, base 25 %, gain 200 %, max 50 %. One
button, stopped by the 60 s time limit. The Wi-Fi log received 1179 of
1199 entries.

| Measure | Run 6 (first 51 s) | Run 7 (60 s) |
| --- | ---: | ---: |
| Mean wheel speed | 99 counts per 50 ms | 101 counts per 50 ms |
| LINE_LOST | 1.4 per 10 s | 0.8 per 10 s, all found again |
| Swings from ±50 to ∓50 within 300 ms | 30 | 5 |
| One wheel stopped | 20.3 % | 11.1 % |
| Jumps of 10 % duty or more | 51.3 per 10 s | 29.0 per 10 s |
| Mean change of the inner duty per 50 ms | 6.39 | 4.03 |

- Straight mode (`STRT`) started at all 8 junction readings and lasted
  300 ms. The 5 swings left are at junctions: about 50 ms before the
  junction reading the side arm under R gives +50 (38/0); the robot then
  drives straight and the main line shows at −50. After the straight
  part the robot was back near 0 within about 0.2 s (run 6: about 0.8 s).
- One junction (34.75 s) ended in LINE_LOST: during the straight part
  the line moved to +50 and C faded; after it the robot turned right,
  swung past the line to −51 and found it again with a search.
- The receiver was started before `STRT` was added and wrote mode 2 as
  `?`; the saved file was corrected to `STRT`.
- Next run: max 75 % (operator), base 33 %, gain 200 % and kd 200 % kept
  (the same turn ratios; D grows with the speed because the position
  changes faster), straight part at most 450 counts.

### Run 8: 2026-10-29, VIN, Wi-Fi log

Base 33 %, max 75 %, gain 200 %, kd 200 %. Stopped by the 60 s time
limit. The Wi-Fi log received 1179 of 1199 entries. The operator saw
oscillation.

| Measure | Run 7 (max 50 %) | Run 8 (max 75 %) |
| --- | ---: | ---: |
| Speed when centred | 103 counts per 50 ms | 145 counts per 50 ms |
| Swings from ±50 to ∓50 within 300 ms | 5 | 126 (10 bursts of 3 or more) |
| \|position\| 100 | 1 % | 19 % |
| One wheel stopped | 11.1 % | 34.2 % |
| LINE_LOST | 5 | 7, all found again |

- About half of the swings came after junctions; the other half on
  straight parts with no junction (for example 10 s and 28–29.6 s).
- In a burst the wheels went 0/75 ↔ 75/0 about every 125 ms: −100 →
  +100 → +7 → −100. The line crossed all three sensors in about 50 ms,
  so the position was nearly only ±100 or 0. The D part (limited to 25 per
  update) braked at the centre (24/43 at +7), but the robot still turned
  past the line in the next 50 ms.
- Duty 0 lets a wheel coast (both driver inputs low), so a 75 % pivot
  keeps turning after the duty changes.

## Open items

- Tune `base_pct` and `gain_pct` on the track.
- Junctions: the robot goes straight for 300 ms. The Vehicle Controller decides later.
- Tune `kd_pct` on the track.
- Stop the run when a wheel turns with no load (a wheel came off in run 6)
  or the readings do not change while the wheels turn.
- Find the lowest duty at which a loaded wheel turns, to set `min_pct`.
- Send the steer word to the Motion task when the RTOS wiring is done.
