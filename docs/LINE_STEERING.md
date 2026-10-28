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

```text
turn  = gain × position + kd × (position − previous position)
left  = base + base × turn / 10000
right = base − base × turn / 10000
```

The correction is a share of the base duty, so it is the same ratio at any
speed. Our motors are close to linear between 10 % and 60 % duty
([unloaded speed test](CALIBRATION_RESULTS.md#unloaded-motor-speed-at-5-v-vin)),
so a duty ratio is close to a speed ratio.

| Position | Meaning | Wheels (base 20 %, gain 100 %, max 30 %, min 12 %) |
| ---: | --- | --- |
| 0 | Line under the centre sensor | 20 / 20: straight |
| +25 | Line a little right of centre | 25 / 15: turn right |
| +50 | Line between centre and right | 30 / 0: 30 / 10 after max, 10 is below min |
| +100 | Line under the right sensor | 30 / 0: pivot right |
| −50 | Line between left and centre | 0 / 30: turn left |

Gain 100 % means: with the line under an outer sensor, the inner wheel stops.

## Trim

`right_trim_pct` is added to the right wheel before the limits (a
negative value adds to the left wheel). The team's drive-straight trim is
+1 % on the right motor. It agrees with the measurement: at 20 % duty the
left wheel ran about 7 % faster, and because a wheel only starts to turn
at about 6 % duty, one more percent at 20 % is about 7 % more speed.

## Limits

Applied in this order, after the trim:

| Limit | Rule | Why |
| --- | --- | --- |
| `max_pct` | A wheel above it is set to it; the other wheel is lowered by the same amount | Keeps the turn when the outer wheel saturates |
| No reverse | A wheel below 0 is set to 0 | No pivot on the spot while following |
| `min_pct` | A wheel below it is set to 0 | A loaded wheel below about 12 % does not turn; it only uses current |

## The D part

`kd_pct` reacts to the change of position since the last update (10 ms).
It works against fast movement of the line, which reduces zig-zag. It is
0 (off) at first. Turn it on if the robot zig-zags on a straight line.

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
moving back, reset), the trim (straight, turning, inner wheel stop, max
limit, negative trim), and the word layout.

Result on 2026-10-27: all tests pass. Six deliberate rule changes (turn
direction swapped, max without lowering the other wheel, no min limit, D
on the first update, no 100 % ceiling, no rounding) each make the test
fail. Three trim rule changes (no left trim, no right trim, negative trim
on the right wheel) each make the test fail.

## Bench test

```sh
cmake --build build --target steer_test
picotool load -f -x build/steer_test.uf2
```

| Control | Action |
| --- | --- |
| GP20 | Spin calibration |
| GP21 | Start line following after 1 s / stop |

Settings: base 20 %, gain 100 %, kd 0, min 12 %, max 30 % (operator choices).

While following, the encoders give the line position module its side
hint: left minus right wheel counts since the line was last centred; more
than +20 means the robot turns right, so the line moves left.

On LINE_LOST the robot searches on the spot at 20 % duty: an eighth of a
wheel turn toward the last side (left if unknown), then through the start
to an eighth on the other side. LINE_FOUND resumes following. This is a
bench stand-in for the Vehicle Controller LINE_SEARCH state.

A run stops on GP21, a failed search, a stalled wheel (no encoder count
for 500 ms), or after 20 s.

This is a driving test, so use 5 V VIN with USB disconnected
([USB power safety](USB_POWER_SAFETY.md)). The run is logged every 50 ms
(position, duties, raw readings, side and its source, hint, wheel counts)
and saved to the last two flash sectors when the robot stops, so a restart
does not lose it. Connect USB and open the serial port: the summary and
log print once, then the summary repeats every 5 s.

Procedure:

1. Flash with USB. Turn on VIN, then disconnect USB.
2. Put the robot on the line, centre sensor over it. Press GP20.
3. Press GP21 and move your hand away. The robot starts after 1 s.
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

## Open items

- Tune `base_pct` and `gain_pct` on the track.
- Junctions: the robot goes straight. The Vehicle Controller decides later.
- Turn on `kd_pct` if the robot zig-zags.
- Find the lowest duty at which a loaded wheel turns, to set `min_pct`.
- Send the steer word to the Motion task when the RTOS wiring is done.
