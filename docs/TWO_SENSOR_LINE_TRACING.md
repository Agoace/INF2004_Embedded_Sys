# Two-sensor line tracing (locked)

**Status: locked on 2026-11-03** by the operator after track run P4
("good enough"). Locked firmware: commit `e4ba9ec`, Git tag
`line-tracing-2sensor-v1`, target `detour_test`. Change this design only
with a new track run and a new record.

The course allows two line sensors. This record states the locked
design in one place. The detailed records are
[Line position, pair mode](LINE_POSITION.md#pair-mode-two-sensors-2026-11-03)
and [Line steering, far gain](LINE_STEERING.md#far-gain-two-line-sensors-2026-11-03).

## Hardware

| Item | Value |
| --- | --- |
| Left sensor (L) | GP27 (the old left sensor) |
| Centre sensor (C) | GP26 (the old centre sensor, at the robot centre line) |
| Right sensor | Disabled; `line_sensors_read()` returns 0 for it |
| Build switch | `LINE_SENSORS_PAIR` 1 (default, `src/line_sensors.h`); every `lp_init()` caller calls `lp_set_pair(&lp, LINE_SENSORS_PAIR != 0, 0)` |
| Line width | 18 mm (operator) |
| Sensor spacing | 14.5 mm measured on the board; the spin recording with the 18 mm line gives about 21 mm between the sensing points (open, see Limits) |
| Sensor height | About 9 mm, high on purpose: a line midway reaches both sensors |

## How it works

The spin calibration before each run learns each sensor's floor and line
value; readings are per mille (0–1000) of that range.

```text
line:      far left ─ under L ─ midway ─ under C ─ far right
position:   −100   ←   −50   ←   0    →   +50   →   +100
            L fading    (C − L) / 20       C fading
```

| Readings | Position |
| --- | --- |
| Both see the line (≥ 5 %) | (C − L) / 20 |
| One sees the line, its reading rising | The same formula (line coming in from the middle) |
| One sees the line, below its peak | Past that sensor: C only (2 × peak − C − L) / 20; L only (C − (2 × peak − L)) / 20 |
| Neither reaches 30 % (dark) | Keep the position and the peak. LINE_LOST after 50 ms if one sensor saw the line last, else after 300 ms |
| L + C ≥ 125 % (junction) | Keep the position and the peak; not centred |

Steering (`src/line_steer.c`, `detour_test` settings):

| Setting | Value |
| --- | --- |
| Base duty | 25 % |
| Gain (P) | 100 |
| D | 100 |
| Far gain | 400 above \|position\| 50 (full turn at about ±60) |
| Min / max duty | 12 % / 50 % |
| Right trim | +1 % |

Target 0 holds the line midway between L and C, so the robot runs about
10 mm right of the line centre.

## Evidence

| Test | Result |
| --- | --- |
| Spin recording, boot 5209 | A line midway reads 38–40 % on each sensor; plain line L + C at most 102 % |
| Track P1 (gain 100) | Calm straight; drove off a curve (too little turn) |
| Track P2 (gain 200) | Swung from side to side; a wheel stopped 54 % of the time |
| Track P3 (far gain) | 65 s; a wheel stopped 9 %; 2.9 turn swaps/s; one loss after a junction (fixed in `e4ba9ec`) |
| Track P4 (locked firmware) | 57 s, stopped by GP21. A wheel stopped 10 %; 2.6 turn swaps/s. 35 searches, all 50–100 ms, each in the correct direction first; position kept through every junction |
| Host tests | `tests/test_line_position.c`, `tests/test_line_steer.c`; mutation checks 35/36 and 11/11 (the survivor is equivalent) |
| Simulator | `sim/scenarios/two_sensor_track.json`: line never dark, no wheel stopped |

P4 log: `build/steer_logs/run_20261103-165858_boot2a9f_run1.txt` (laptop
only, not committed).

## Limits

- **Sharp right curves:** a full turn (inner wheel stopped, outer 50 %)
  is not always enough. C goes dark past the sensor, and a 50–100 ms
  search turn finds the line. Accepted at lock.
- **Small margin midway:** 38 % against the 30 % dark limit. A higher
  mount or a narrower line can make the midway point dark.
- **Spacing:** 14.5 mm measured against about 21 mm from the recording.
  The behaviour does not depend on it; distances in millimetres do.
- **Other targets:** `steer_test` and `bypass_test` use pair mode with
  gain 200 / D 200 and no far gain. They are not part of the locked
  design.
- **Simulator:** its motors turn more easily than the real ones; it did
  not show the weak turn of P1.
