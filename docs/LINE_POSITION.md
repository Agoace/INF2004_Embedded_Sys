# Normalised line position

This record describes the line position module on the `yx-line_tracing`
branch. It replaces the fixed ADC thresholds of the first line detector
(`src/line_detector.c`: black ≤ 1200, white ≥ 2600) with values that each
sensor learns. The first detector is not changed.

- Code: [`src/line_position.c`](../src/line_position.c), [`src/line_position.h`](../src/line_position.h)
- Host unit test: [`tests/test_line_position.c`](../tests/test_line_position.c)
- Bench test: [`src/line_test.c`](../src/line_test.c)
- Sensor pins: [`src/ir_module_pins.h`](../src/ir_module_pins.h): left GP26, centre GP27, right GP28
- Events go to the [Vehicle Controller](VEHICLE_CONTROLLER.md)

## Why

| Fixed thresholds | Normalised |
| --- | --- |
| One pair of ADC values for all sensors | Each sensor learns its own floor and line values |
| Must be changed for a new surface, light, or sensor height | Calibrates at each start and follows slow changes |
| Assumes the line reads low | Polarity is found from the calibration data |
| A sensor on the line edge is "unknown", so steering is invalid | An edge reading gives a value between 0 and 100 and moves the position smoothly |
| Steering error −2 to +2 | Position −100 to +100 |

## How it works

```text
raw ADC ──▶ n = (raw − floor) / (line − floor), 0–100 per sensor
        ──▶ position = 100 × (nR − nL) / (nL + nC + nR), −100…+100
```

| Position | Meaning |
| ---: | --- |
| −100 | Line under the left sensor |
| −50 | Line between the left and centre sensors |
| 0 | Line under the centre sensor |
| +50 | Line between the centre and right sensors |
| +100 | Line under the right sensor |

### A. Calibration sweep

The sensors cross the line for 4 s. For now, slide the robot by hand. Later,
the CALIBRATE state turns the robot left and right over the line. Each
sensor records its minimum and maximum.

Polarity: the floor is seen most of the time, so the average reading is
nearer the floor end. The module compares the average with both ends for
all three sensors and decides once whether the line reads higher or lower.

A sensor whose range is less than 200 ADC counts (about 0.16 V) fails the
calibration. This is a sanity limit, not a black/white threshold: the
sensor is broken or did not cross the line. On failure the previous
calibration stays in use.

### B. Learning while running

| Reading | What is learned |
| --- | --- |
| Below 20 % | The floor value moves 1/256 of the way to the reading |
| Above 80 % | The line value moves 1/256 of the way to the reading |
| 20–80 % | Nothing |

At 1 kHz this follows a change in about 0.25 s. An end only learns when the
sensor clearly sees it. A plain running minimum and maximum would slowly
shrink the range of a sensor that does not see the line for a long time;
then noise would look like the line. Learning also never moves an end so
far that the range drops below the 200-count sanity limit.

## Thresholds

All are percentages of each sensor's own range, not ADC values:

| Name | Value | Meaning |
| --- | ---: | --- |
| `LP_LOST_PCT` | 30 % | All sensors below this: the line is not seen |
| `LP_FOUND_PCT` | 50 % | Any sensor at or above this: the line is seen again |
| `LP_JUNCTION_PCT` | 50 % | All sensors at or above this: junction |
| `LP_SIDE_PCT` | 20 | Position beyond ±20 gives a side for LINE_LOST |
| `LP_LOST_MS` | 50 ms | Design review: LINE_LOST after more than 50 ms |

The gap between 30 % and 50 % stops the seen/lost state from changing on
every reading.

## Events for the Vehicle Controller

`lp_update()` returns a [Vehicle Controller event word](VEHICLE_CONTROLLER.md#data-representation):

| Event | When | Payload |
| --- | --- | --- |
| LINE_LOST | The line was not seen for 50 ms | Last side: left, right, or unknown (line was near the centre) |
| LINE_FOUND | Any sensor reaches 50 % after LINE_LOST | — |

After a calibration the module assumes the line is under the robot. If it
is not, LINE_LOST comes after 50 ms.

## Data representation

Line word for other tasks and telemetry:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–7 | Position, two's complement, −100 to +100 | 8 |
| 8 | Line seen | 1 |
| 9 | Junction | 1 |
| 10–11 | Last side | 2 |
| 12–15 | Spare | 4 |

Each normalised reading is 0–100 (7 bits). The learned ends are 12-bit ADC
values with 8 fraction bits for the slow learning; they are private to the
line task.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
   tests/test_line_position.c src/line_position.c \
   -o build/test_line_position
./build/test_line_position
```

It checks both polarities with different values per sensor, seven
positions including edge readings, junction, no calibration, a sensor that
never saw the line, LINE_LOST at 50 ms with the side, LINE_FOUND at 50 %,
a short gap, the unknown side, a start off the line, a 600-count light
drift over 10 s, the range guard, and the line word.

Result on 2026-10-27: all tests pass. Six deliberate rule changes (fixed
polarity, no range guard, no 50 ms wait, found at 30 %, no floor learning,
not seen after calibration) each make the test fail. The range-guard test
was first written wrongly and did not detect its rule change; it was
corrected.

## Bench test

```sh
cmake --build build --target line_test
picotool load -f -x build/line_test.uf2
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
```

| Control | Action |
| --- | --- |
| GP20 | Calibrate again |
| GP21 | Pause or resume the 100 ms output |

The motors are not driven.

Procedure:

1. Put the robot on the track. During the 4 s calibration, slide it slowly
   left and right across the line two or three times.
2. Check the calibration report: all three sensors OK, and the polarity.
3. Put the line under each sensor in turn. Expect about −100, 0, +100, and
   values between them on the edges.
4. Lift the line away to the left. Expect LINE_LOST with LEFT. Bring it
   back. Expect LINE_FOUND.

Results: not yet recorded.

## Open items

- Record the bench test and the measured floor and line values.
- Confirm the polarity. Board diagnostics read about 180–220 mV on all
  three sensors; the surface under them was not recorded.
- Tell the Vehicle Controller the line state at START, so that a robot
  started off the line searches at once.
- Send JUNCTION to the Vehicle Controller in a later step.
- Agree this approach with the line detector owner.
