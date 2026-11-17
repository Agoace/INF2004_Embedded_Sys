# Demo course figure-8 (2026-11-17)

The final demo line track, as a Webots course. Only the line track is
fixed. Obstacle, ramp and barcode are free on the day, so they are
parameters (default: none placed).

## Source and labels

| Item | Value | Label |
| --- | --- | --- |
| Tile grid, track, loose arcs | Operator photos IMG_1960, IMG_1961, IMG_1962 (local only, not in the repository) | PHOTO-ESTIMATED |
| Tile, line, arc | 315 mm tile, 18 mm line, arc radius 157.5 mm (MEASUREMENTS "Course tile") | ASSUMED |
| Obstacle box | 170 × 120 mm footprint (STEP 8 box); 120 mm high | MEASURED / PHOTO-ESTIMATED |
| Ramp | 195 mm long, 10 mm high (test hump); 52 mm wide | MEASURED / PHOTO-ESTIMATED |
| Barcode sheet | SIL sheet 400 × 50 mm, 40 mm bars, 77 mm left of the line centre | ASSUMED |
| Start pose | `L1:1190` (top straight, heading west) | ASSUMED |

## Tile map

Source of truth: [`course_figure8_tilemap.txt`](../../sim/webots/courses/course_figure8_tilemap.txt).
North is up, as in IMG_1962. `*` = loose arc, not joined to the track.

| | c0 | c1 | c2 | c3 | c4 | c5 | c6 | c7 | c8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| r0 | . | SE | - | SW | SW* | B | . | . | . |
| r1 | SE | NW | B | \| | SW* | B | . | . | . |
| r2 | \| | B | B | \| | NW* | SW* | . | . | . |
| r3 | NE | - | - | X | - | - | - | SW | . |
| r4 | NW* | NW* | B | \| | B | B | SW* | \| | B |
| r5 | B | B | NW* | \| | B | SW* | NW* | \| | B |
| r6 | . | NE* | B | \| | B | SE | SW | \| | B |
| r7 | . | . | . | NE | - | NW | NE | NW | B |

| Check (tool) | Result |
| --- | --- |
| Tiles | 57 (29 track, 11 loose arcs, 17 blank) |
| Loop 1 (L1), with the S-jog on its left side | 12 tiles, 3442 mm |
| Loop 2 (L2), with the S-wiggle on its bottom side | 18 tiles, 5197 mm |
| Crossing | r3 c3, the only 4-way tile |
| Loose arcs | 11 counted in the photos (Main estimated about 14); 2 pairs form half circles |

The c8 column (r4–r7) and the left edge of rows r4–r6 are the least
certain cells: perspective in IMG_1962 and the image edge in IMG_1961.

## Placements

A placement is `LOOP:MM[:SIDE]`:
- `LOOP`: `L1` leaves the crossing north, `L2` leaves it east. Add `r` (`L1r`, `L2r`) for the other drive direction.
- `MM`: distance along the line from the crossing centre, in the drive direction.
- `SIDE`: `centre` (default), `left` or `right` (56 mm, one wheel). A barcode is always 77 mm left of the line.

Straight runs (distance in mm along L1 / L2):

| Loop | Straights | Arcs |
| --- | --- | --- |
| L1 | 0–788 N, 1035–1350 W, 2092–2407 S, 2655–3442 E | 788–1035, 1350–2092 (S-jog), 2407–2655 |
| L2 | 0–1102 E, 1350–2295 S, 3532–3847 W, 4094–5197 N | 1102–1350, 2295–3532 (S-wiggle), 3847–4094 |

Example placements (not measured). They are alternatives: do not place all at once (for example `L2:1800` overlaps the `L2:1700` barcode). The render uses `L1:400` obstacle, `L2:600` ramp, `L2:1700` barcode.

| Object | Placement | Where |
| --- | --- | --- |
| Obstacle | `L1:400` | Loop 1 straight north of the crossing |
| Obstacle | `L2:1800` | Loop 2 right-side straight |
| Obstacle | `L1:3150` | Just before the crossing (loop 1 bottom straight) |
| Barcode | `L2:1700` | Loop 2 right-side straight, after the corner |
| Barcode | `L1r:250` | Loop 1 driven the other way, after the crossing |
| Ramp | `L2:600` | Loop 2 top straight, centred |
| Ramp | `L1:2250:left` | Loop 1 left straight, under the left wheel |

## Build

```sh
python3 sim/webots/tools/course8_tilemap.py \
  --out sim/webots/courses/course_figure8_photo.json        # track only
python3 sim/webots/tools/course8_tilemap.py --obstacle L1:400 \
  --ramp L2:600 --barcode L2:1700 --out build/course8/example.json
python3 -m unittest sim/webots/tests/test_course8_tilemap.py
```

The tool writes schema-1 course JSON. The Webots world comes from the
course generator on `mtk3-hil` (`sim/webots/pil/world.py`, owned by SW8),
which is not on this branch: in a `mtk3-hil` checkout run
`cd sim/webots && python3 -m pil.world --course <json> --robot calibration/robot_assumed.json --output <that checkout>/build/<name>`.
The generator reports 18 open ports: the free ends of the loose arcs
(`allow_open_ports` is true for this reason).

## Status

Built and host-checked: 8 unit checks; world generated on `mtk3-hil`
448fcef. The top-down render was compared with IMG_1962. No Webots
robot run yet.
