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

| Position | Meaning | Wheels (base 25 %, gain 100 %) |
| ---: | --- | --- |
| 0 | Line under the centre sensor | 25 / 25: straight |
| +50 | Line between centre and right | 38 / 13: turn right |
| +100 | Line under the right sensor | 50 / 0: pivot right |
| −50 | Line between left and centre | 13 / 38: turn left |

Gain 100 % means: with the line under an outer sensor, the inner wheel stops.

## Limits

Applied in this order:

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
moving back, reset), and the word layout.

Result on 2026-10-27: all tests pass. Six deliberate rule changes (turn
direction swapped, max without lowering the other wheel, no min limit, D
on the first update, no 100 % ceiling, no rounding) each make the test
fail.

## Bench test

```sh
cmake --build build --target steer_test
picotool load -f -x build/steer_test.uf2
```

| Control | Action |
| --- | --- |
| GP20 | Spin calibration |
| GP21 | Start line following after 1 s / stop |

Settings: base 25 %, gain 100 %, kd 0, min 12 %, max 50 %.

A run stops on GP21, LINE_LOST, a stalled wheel (no encoder count for
500 ms), or after 20 s. There is no line search yet.

This is a driving test, so use 5 V VIN with USB disconnected
([USB power safety](USB_POWER_SAFETY.md)). The run is logged in RAM every
50 ms. After the robot stops, connect USB and open the serial port. The
summary and log print once, then the summary repeats every 5 s.

Procedure:

1. Flash with USB. Turn on VIN, then disconnect USB.
2. Put the robot on the line, centre sensor over it. Press GP20.
3. Press GP21 and move your hand away. The robot starts after 1 s.
4. After it stops, connect USB and read the report.

Results: not yet recorded.

## Open items

- First run on the track; tune `base_pct` and `gain_pct`.
- Turn on `kd_pct` if the robot zig-zags.
- Find the lowest duty at which a loaded wheel turns, to set `min_pct`.
- Send the steer word to the Motion task when the RTOS wiring is done.
