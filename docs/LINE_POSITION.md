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

The sensors cross the line: the robot turns on the spot left and right
over the line (spin calibration), or is slid across it by hand for 4 s.
Each sensor records its minimum and maximum. The spin is the model for the
Vehicle Controller CALIBRATE state.

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

Nothing moves until a button is pressed.

| Control | Action |
| --- | --- |
| GP20 | Spin calibration (motors run) |
| GP21 | Hand calibration: slide the robot across the line for 4 s |

Spin calibration, at 20 % duty, on the spot:

```text
start on the line ─▶ left ¼ wheel turn ─▶ right ¼ wheel turn past the start
                  ─▶ calibrate ─▶ turn back until the line is under the centre sensor
```

- All distances are counted from the start position, so coasting does not
  add up from step to step.
- The last step stops on the line, not on the encoder count. Tyres slip
  when the robot turns on the spot.
- USB power limits: see the
  [spin-calibration exception](USB_POWER_SAFETY.md#bounded-spin-calibration-exception).

Procedure:

1. Put the robot on the track with the centre sensor over the line.
2. Press GP20. Check the calibration report: all three sensors OK.
3. Put the line under each sensor in turn. Expect about −100, 0, +100, and
   values between them on the edges.
4. Lift the line away to the left. Expect LINE_LOST with LEFT. Bring it
   back. Expect LINE_FOUND.

### Run 1: 2026-10-27, spin calibration, USB power

15 % duty stalled (motor 2 stopped after 746 of 2535 counts). Raised to
20 %. Whole and half wheel turns were too far; a quarter turn (633 counts)
was enough. The first version counted each step from the previous stop
and did not return to the line. Final version:

| Step | Time | Encoders from start (M1, M2) |
| --- | ---: | --- |
| Left | 807 ms | −633, −650 |
| Right | 1633 ms | +641, +633 |
| Back to the line | 712 ms | +60, −22: stopped on the line |

| Sensor | Floor | Line | Range |
| --- | ---: | ---: | ---: |
| L | 191 | 3424 | 3233 |
| C | 185 | 2615 | 2430 |
| R | 202 | 1645 | 1443 |

- The line reads **higher** than the floor. The fixed-threshold detector
  assumes the opposite (black ≤ 1200).
- One fixed "line ≥ 2600" value would never detect the line on R. Each
  sensor needs its own range.
- After the stop: C at 58 %, position 0.
- The encoders were 60 and −22 counts from the start at the stop, so a
  count-only return would not reliably end on the line.

Procedure steps 3 and 4 (robot moved by hand, 2026-10-27):

| Check | Result |
| --- | --- |
| Each sensor reaches 100 % | Yes: L, C and R |
| Line under L, C, R | Position −100, 0, +100 |
| Line between L and C | L 50 %, C 67 %: position −42 |
| Slide off, line leaves through R (2 times) | LINE_LOST RIGHT |
| Slide off, line leaves through L | LINE_LOST LEFT |
| Lift the robot | LINE_LOST unknown side (correct: the line did not leave sideways) |
| Put back on the line | LINE_FOUND, once per return |

A lift and a line that leaves sideways through the centre sensor look the
same: only C fades. Using the recent movement of the position to choose a
side was considered and not added, because it gives a wrong side after a
lift. Look at it again if real driving gives "unknown" when the line leaves
sideways.

## Open items

- Find out why R reaches only 1645 on the line (sensor height, angle, or
  only part of the line crossed it).
- Tell the line detector owner: the line reads high on this robot.
- Tell the Vehicle Controller the line state at START, so that a robot
  started off the line searches at once.
- Send JUNCTION to the Vehicle Controller in a later step.
- Agree this approach with the line detector owner.
