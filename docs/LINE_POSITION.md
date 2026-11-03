# Normalised line position

This record describes the line position module on the `yx-line_tracing`
branch. It replaces the fixed ADC thresholds of the first line detector
(`src/line_detector.c`: black ≤ 1200, white ≥ 2600) with values that each
sensor learns. The first detector is not changed.

- Code: [`src/line_position.c`](../src/line_position.c), [`src/line_position.h`](../src/line_position.h)
- Host unit test: [`tests/test_line_position.c`](../tests/test_line_position.c)
- Bench test: [`src/line_test.c`](../src/line_test.c); spin calibration in [`src/line_spin.c`](../src/line_spin.c)
- Steering from the position: [Line steering](LINE_STEERING.md)
- Sensor pins, robot's own sides: left **GP28**, centre GP27, right **GP26**, set in
  [`src/line_sensors.c`](../src/line_sensors.c). [`src/ir_module_pins.h`](../src/ir_module_pins.h)
  names them the other way round.
- Events go to the [Vehicle Controller](VEHICLE_CONTROLLER.md)

> **Update 2026-11-03:** the course allows only two line sensors. The robot
> now uses **pair mode**: left **GP27** (the old left sensor), centre
> **GP26** (the old centre sensor), right sensor disabled. See
> [Pair mode](#pair-mode-two-sensors-2026-11-03). The three-sensor mode
> below is unchanged and selected with `LINE_SENSORS_PAIR 0`.

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
raw ADC ──▶ n = (raw − floor) / (line − floor), 0–1000 per sensor
        ──▶ side (where the line is going) ──▶ position −100…+100
```

With a known side: position = side × 50 × ((1000 − nC) + nOuter) / 1000.
With the line under an outer sensor alone: ±100. With no known side, or
with both outer sensors on the line: the weighted average
100 × (nR − nL) / (nL + nC + nR).

| Position | Meaning |
| ---: | --- |
| −100 | Line under the left sensor |
| −50 | Line between the left and centre sensors (in the gap) |
| 0 | Line under the centre sensor |
| +50 | Line between the centre and right sensors |
| +100 | Line under the right sensor |

### The gaps between the sensors

> **Update 2026-11-03:** the operator says these gap numbers were not a
> controlled measurement. The sensors are 14.5 mm apart and about 9 mm
> high, on purpose, so that the spots are wide and a line between two
> sensors reaches both. Pair mode does not use the gap rise.

On 2026-10-27 the line, placed between C and L, gave almost nothing:

| | L | C | R |
| --- | ---: | ---: | ---: |
| Line in the C–L gap | 178 | 222 | 198 |
| Bare floor, same area | 151 | 188 | 200 |
| Rise | +27 | +34 | −2 |

The noise is about ±2 counts, so the rise is real, but it is under 1 % of
the range, and the floor itself varies by about 100 counts across the mat.
The sensors are already at the highest and closest mount position. So the
gap rise cannot say "the line is here", but it can say which side the line
went to: each outer sensor is compared with its own reading a moment
earlier, while the line was centred.

Side tracking, newest evidence first:

| Source | Rule |
| --- | --- |
| Sensor | One outer sensor reads at least 5 % |
| Gap rise | One outer sensor is at least 0.3 % above its baseline and at least 0.3 % above the other |
| Wheel hint | From the caller: the left wheel turned at least 20 counts more than the right since the line was centred, so the robot turns right and the line moves left (or the opposite) |

- Baseline: the outer sensors' readings while the line is centred (C at
  least 80 %, outer sensors under 5 %), moving 1/16 per reading, used after
  16 readings. It must not learn while C fades: then it absorbs the rise
  it must detect. The host test found this.
- The side is kept until a newer sensor or gap rise replaces it. Since
  2026-10-29 it is also kept when the line is centred, as a memory
  (source "none"): on a curve the line leaves on the same side again, so
  the position follows C as soon as C fades (C 90 % gives ±5). Before,
  the side was cleared when centred and the position stayed 0 until the
  line was half into the gap; then the steering pulled hard. In the last
  20 s of a track run one wheel was stopped 17 % of the time (jerky
  turns). The cost: on a straight line that drifts to the other side, the
  robot first steers the wrong way, a little, until the gap rise on the
  other side replaces the memory.
- The hint is used only when no side is kept, because the robot's own
  correction turns the wheel difference the other way.
- If the line goes dark from the middle (position under ±75), it is
  probably in a gap: LINE_LOST waits 300 ms instead of 50 ms. The side can
  come late, in the dark, when the gap rise appears; then the position
  becomes ±50 and the steering can bring the line back. A lift is also
  reported after 300 ms.
- If the line goes dark after a position of ±75 or more, it left outward
  past an outer sensor: the position stays and LINE_LOST comes after 50 ms.
- Junction hold: for 300 ms after the last junction reading (all sensors
  at least 50 %), the gap rise is not used and the baseline does not
  learn, because a side arm of the junction can stay near an outer sensor.
  The side from before stays. Steering Run 3 found this.

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

All are percentages of each sensor's own range, not ADC values. Readings
are per mille (0–1000), so the code compares `LP_PERMILLE(pct)`:

| Name | Value | Meaning |
| --- | ---: | --- |
| `LP_LOST_PCT` | 30 % | All sensors below this: the line is not seen |
| `LP_FOUND_PCT` | 50 % | Any sensor at or above this: the line is seen again |
| `LP_JUNCTION_PCT` | 50 % | All sensors at or above this: junction |
| `LP_SIDE_PCT` | 20 | Position beyond ±20 gives a side for LINE_LOST |
| `LP_LOST_MS` | 50 ms | Design review: LINE_LOST after more than 50 ms |
| `LP_GAP_LOST_MS` | 300 ms | LINE_LOST when the line goes dark from the middle |
| `LP_CENTRED_PCT` | 80 % | C at or above, outer sensors dark: centred |
| `LP_OUTER_SEEN_PCT` | 5 % | An outer sensor at or above this sees the line |
| `LP_GAP_RISE_PERMILLE` | 0.3 % | Gap rise above the baseline (about 11 ADC counts) |
| `LP_OUTWARD_POSITION` | 75 | Dark after this: the line left outward |
| `LP_JUNCTION_HOLD_MS` | 300 ms | No gap rise and no baseline learning after a junction |

The gap between 30 % and 50 % stops the seen/lost state from changing on
every reading.

## Events for the Vehicle Controller

`lp_update()` returns a [Vehicle Controller event word](VEHICLE_CONTROLLER.md#data-representation):

| Event | When | Payload |
| --- | --- | --- |
| LINE_LOST | The line was not seen for 50 ms after an outer sensor, or 300 ms from the middle | Last side: left, right, or unknown |
| LINE_FOUND | Any sensor reaches 50 % after LINE_LOST | — |

After a calibration the module assumes the line is under the robot. If it
is not, LINE_LOST comes after 300 ms.

The design review asks for LINE_LOST after more than 50 ms. The 300 ms
wait from the middle is a deliberate change for the sensor gaps; at 20 %
duty the robot moves about 4 cm in that time.

## Data representation

Line word for other tasks and telemetry:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–7 | Position, two's complement, −100 to +100 | 8 |
| 8 | Line seen | 1 |
| 9 | Junction | 1 |
| 10–11 | Last side | 2 |
| 12–15 | Spare | 4 |

Each normalised reading is 0–1000 (10 bits). The learned ends are 12-bit ADC
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

Side tracking: per-mille resolution, gap left (side, position −20 then
−50, no LINE_LOST at 50 ms, LINE_LOST LEFT at 300 ms), gap right, equal
rise on both sides (no side, LINE_LOST at 50 ms), baseline not ready, the
hint (used, kept while the wheels turn back), the side remembered when
centred (C 90 % then gives −5), a gap rise and then a sensor replacing
it, a gap rise on the other side replacing a remembered side, the junction hold (a side
arm near L keeps RIGHT, the gap rise works again after the hold), and no
baseline learning during the hold.

Result on 2026-10-27: all tests pass. Six deliberate rule changes (fixed
polarity, no range guard, no 50 ms wait, found at 30 %, no floor learning,
not seen after calibration) each make the test fail. The range-guard test
was first written wrongly and did not detect its rule change; it was
corrected.

Side tracking, 2026-10-27: all tests pass. Eight deliberate rule changes
(baseline learns while fading, gap rise without the difference check,
hint replaces a kept side, no 300 ms gap wait, no outward hold, side not
cleared when centred, baseline never ready, outer-only not ±100) each make
the test fail. Three junction-hold rule changes (baseline learns in the
hold, gap rise used in the hold, no hold time) each make the test fail.

Side memory, 2026-10-29: the old rule (clear the side when centred)
makes 4 checks fail. The "side not cleared when centred" rule change
above is now the intended rule.

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

> **Update 2026-10-27:** Run 1 and the hand-movement checks below used the
> pin names from `ir_module_pins.h` (L = GP26, R = GP28). The first steering
> run turned away from the line, and the operator confirmed that GP26 is the
> robot's **right** sensor and GP28 its **left**. In the tables below, "L"
> is the physical right sensor and "R" the physical left sensor. The code
> now uses the physical sides.

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

## Pair mode (two sensors), 2026-11-03

Course rule: two line sensors. The wiring changed: the old left sensor
moved from GP28 to **GP27** (left), the old centre sensor moved from GP27
to **GP26** (centre). The old right sensor is disabled.
[`src/line_sensors.c`](../src/line_sensors.c) reads only GP27 and GP26 when
`LINE_SENSORS_PAIR` is 1 (the default in
[`src/line_sensors.h`](../src/line_sensors.h)) and returns 0 for the right
input. Every target that calls `lp_init()` then calls
`lp_set_pair(&lp, LINE_SENSORS_PAIR != 0, 0)`.

| Value | Source |
| --- | --- |
| Sensor spacing, centre to centre | 14.5 mm (operator, 2026-11-03) |
| Sensor height | About 9 mm (operator, 2026-11-03), high on purpose for wide, overlapping spots |
| Line width | 18 mm (operator, 2026-11-03) |
| Spot width | Not measured |

### Rules

| Readings | Position |
| --- | --- |
| Both sensors see the line (≥ 5 %) | (C − L − target) / 10, clamped to ±100. Target 0: the line centred between L and C |
| Only one sensor sees the line | The line left the pair on that side. Keep the largest value on that side until both see the line again: a fading sensor must not reduce the steering |
| Neither reaches 30 % (dark) | Keep the last position. LINE_LOST after 50 ms (after an outward value, \|position\| ≥ 75) or 300 ms |

- The line moves right: L fades and C rises, so the position goes up and
  the robot steers right. The same sign holds when the line is narrower
  than the spacing (C rises first).
- No junction flag: two sensors cannot tell a junction from the line. The
  junction straight-through in `steer_test` and `detour_test` does not
  run; at a cross the readings stay equal, so the robot drives straight.
- Centred: both see the line and |position| ≤ 20.
- Calibration checks contrast and polarity on L and C only.
- With target 0 the robot runs about 7 mm right of the line centre (half
  the spacing). `lp_set_pair()` takes another target (per mille of C − L)
  for edge following.

### Host test

The pair block in [`tests/test_line_position.c`](../tests/test_line_position.c)
checks: three-sensor mode rejects the disabled right input, pair
calibration ignores it, `lp_init` turns pair mode off, the right reading
is cleared, centred and its ±20 limit, both signs, no junction, the 5 %
limit, the dark limit, the one-sensor hold on both sides, LINE_LOST at
50 ms with the side, LINE_FOUND, the 300 ms wait, the target and both
clamps, a LOW line, and low contrast on L.

Result 2026-11-03: all tests pass. Mutation check of the pair code: 26 of
26 rule changes make the test fail (after 6 survivors led to new checks).

### Simulator results, 2026-11-03

`detour_test` with pair mode, scenarios
[`two_sensor_track`](../sim/scenarios/two_sensor_track.json) (the operator's
test-track loop with curves and crosses, no obstacle, 60 s) and
[`two_sensor_straight`](../sim/scenarios/two_sensor_straight.json). Spacing
14.5 mm; spot width 16 mm (guess); the recorded-gap check is off
(`gap_check: false`).

| Case | Result |
| --- | --- |
| Straight | Steady 6.9 mm right of the line centre; no oscillation |
| Track, spot 16 mm | About 2.5 laps in 55 s through curves and crosses; line never dark; no contact |
| Track, spot 4, 8, 12, 24 mm | Same: line never dark |
| Track, line 14 mm and 10 mm | Same: line never dark |

These are simulator results with an ideal sensor model (uniform spot, no
noise). They are not robot results.

Rerun 2026-11-03 with the fitted model (line 12.5 mm, spot 8 mm):

| Case | Result |
| --- | --- |
| Straight | Steady 7.1 mm right of the line centre; L 36 %, C 39 % |
| Track | Same path as before; line never dark; no contact |
| Spot 5 or 11 mm, line 14 mm | Line never dark |
| Line 11 mm | Midway reading 28 %, below the 30 % dark limit: 4.4 % dark samples, repeated LINE_LOST and search turns; the robot still goes round |

The target (midway) is where both readings are lowest, about 8 points
above the dark limit on the robot.

### Robot check: spin recording, 2026-11-03, boot 5209

`detour_test` GP20 (spin calibration only) with the raw recording of
[`src/line_spin.c`](../src/line_spin.c): L and C every 2 ms, 1634 samples,
all received over Wi-Fi. Calibration: L floor 172, line 2487; C floor 158,
line 2611. Copy on the laptop: `build/steer_logs/spin_raw_boot5209.csv`
(not committed).

| Crossing | Midway between L and C | Lowest stronger reading |
| --- | --- | --- |
| Step 2, line from C to L | L 30 %, C 44 % then L 40 %, C 26 % | 40 % |
| Step 3, line from L to C | L 38 %, C 38 % | 38 % |

- Both sensors see a line midway between them, so pair mode does not go
  dark in the middle. The lowest reading (38 %) is above the 30 % dark limit;
  the margin is small.
- The position goes from +100 to −100 only in a narrow band of the
  crossing; outside it one sensor is full and the other is at the floor,
  and the position stays at ±100.
- Simulator fit (14.5 mm spacing as the ruler, flat-spot model of
  `sim/core/world.c`): **effective line 12.5 mm, spot 8 mm** gives 38 % on
  each sensor midway, as recorded. A fit of L against C alone cannot
  tell 13.6/3 mm from 11.2/11.5 mm; the wheel counts in each sweep select
  12–13 mm and 7–9 mm. The fit fixes only the ratios.
- **Update:** the operator confirms the line is 18 mm wide. Scaled to an
  18 mm line, the fit gives **sensing points 20.9 mm apart and an 11.5 mm
  spot**. This does not agree with the 14.5 mm measured on the board; the
  cause is open (for example, the sensors look slightly outward). The
  readings are the same; only the distances change. Both two-sensor
  scenarios use 18 / 20.9 / 11.5 mm and the calibration ADC values above.
  On a straight the robot then drives 10.3 mm right of the line centre.
- The old 50 ms run logs (2026-10-29 to 11-03) show a dark gap instead.
  The left sensor read 1911 on the line then and 2487 now, so the sensor
  mount changed. Use only data from after the change.

## Open items

- Find out why R reaches only 1645 on the line (sensor height, angle, or
  only part of the line crossed it).
- Tell the line detector owner: the line reads high on this robot.
- Check the gap rise and the side sources on the track (`steer_test` log).

### Gap check: 2026-10-27, `steer_test` idle, USB

The line was centred (C 90 %, side cleared), then slid slowly into the
C–L gap. Side L was found from the gap rise:

| | L raw | C raw | R raw |
| --- | ---: | ---: | ---: |
| Centred | 163 | 2300 | ~205 |
| In the gap | 168–172 | ~276 | ~199 |
| Rise | +5 to +9 | | −6 |

- The rise was smaller than the first measurement (+27): the line was
  only just past the edge of C. +5 to +9 counts is 2.5–4.5 per mille,
  just above the 3 per mille trigger.
- LINE_LOST came 50 ms after C went dark, before the rise appeared, so
  the position stayed 0. The wait from the middle was then changed to
  300 ms for every dark start, not only one with a known side.
- A first try showed side L from the start: it was left from the spin,
  which stopped with C at 48 %, so the line was never centred and the side
  was never cleared. The spin now stops at the peak of C.
- Tell the Vehicle Controller the line state at START, so that a robot
  started off the line searches at once.
- Send JUNCTION to the Vehicle Controller in a later step.
- Agree this approach with the line detector owner.
