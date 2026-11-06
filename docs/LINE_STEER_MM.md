# Line steering in wheel speed (mm/s)

The locked two-sensor line steering
([two-sensor line tracing](TWO_SENSOR_LINE_TRACING.md), tag
`line-tracing-2sensor-v1`) moved from motor duty to wheel speed, so the
`motor controller` holds the wheel speeds. Branch:
`mtk3-line`; integration: `mtk3-staging`. Status: code and host tests done; used by `vc_line_test`; **not run on the track**.
The lock rule applies: a track run and a new record are needed before
this replaces the duty version.

- Code: `steer_mm_*` in [`app_program/subsystem3/line_steer.c`](../app_program/subsystem3/line_steer.c), [`app_program/subsystem3/line_steer.h`](../app_program/subsystem3/line_steer.h)
- Host unit test: [`tests/test_line_steer.c`](../tests/test_line_steer.c)
- Speed values from the `duty map` (run DM1, floor)

## Rule

The same rule as `steer_update()`: one shared function computes both.
Only the units and the trim differ.

| Part | Duty version (locked) | mm/s version |
| --- | --- | --- |
| Base, min, max | % duty | mm/s |
| Gain, D, far gain | no unit (turn per position) | the same |
| Trim | `right_trim_pct` | **none**: the motor controller makes the wheels match (hardware quirks stay in the controller, [ADR](ADR_MOTOR_CONTROLLER.md)) |
| Min | Duty at which a loaded wheel starts to turn (a motor limit) | **Steering setting**: the inner wheel slows to min, then stops. Kept so that the first track run compares with P4 |
| Output | Steer word: 7 + 7 bits (duty %) | Steer mm word: 9 + 9 bits (0–511 mm/s), bits 18–31 spare |

## Settings for the first track run (mapped one to one)

Each duty setting is converted once through the duty map. The steering
code has no duty table.

| Setting | Locked (duty) | Duty map | mm/s version |
| --- | --- | --- | --- |
| Base | 25 % | 171 mm/s | `base_mm_s` 171 |
| Min | 12 % | 62 mm/s | `min_mm_s` 62 |
| Max | 50 % | 378 mm/s | `max_mm_s` 378 |
| Gain | 100 | — | 100 |
| D | 100 | — | 100 |
| Far gain | 400 above 50 | — | 400 above 50 |
| Right trim | +1 | — | none |

Why the result matches: the duty map is close to a straight line from
8 % to 70 % (about 8.4 mm/s per % between 12 % and 50 %). The rule is
linear in (value − min), so a straight-line map changes the result only
by its slope. Example at position +20 after 0 (turn 4000): duty 12 + 13 + 5 = 30 % /
12 + 13 − 5 = 20 % (trim off) → map 214 / 131 mm/s; mm/s rule 215 / 127
mm/s.

The motor controller limits the asked speed to 460 mm/s; the max 378 is
below it.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_line_steer.c app_program/subsystem3/line_steer.c -o build/test_line_steer
./build/test_line_steer
```

| Check | Result |
| --- | --- |
| Same rule: duty numbers used as mm/s (trim 0) give the same output as `steer_update()` over a sequence with D and far gain | Pass |
| Locked mm/s settings: centred 171/171; +20 after 0: 215/127; +100: 378/0; +40 without D: 215/127 | Pass |
| Straight: base on both wheels, D forgotten | Pass |
| Max above 511 (512, 2000) set to 511; 511 kept; word layout and field masks | Pass |
| Max clamp pushes the other wheel below min: it stops (shared rule; this case had no test before) | Pass |
| Duty version against the locked code (tag `line-tracing-2sensor-v1`): 20000 random settings × 40 updates, with straight drives | 800000 updates, 0 differences |
| Simulator (`ctest --test-dir sim/build`, firmware `detour_test`) | 6 of 6 pass |

Mutation check (2026-11-04): 17 of 17 caught (14 on the mm/s code, 3 on
the shared rule). Three gaps found by the first pass were closed: max
512, word fields with the spare bits set, and the clamp-below-min case.

ARM size (`-Os`, Cortex-M0+): `line_steer.c` 636 bytes (locked: 412).
