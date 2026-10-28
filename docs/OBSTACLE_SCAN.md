# Servo obstacle scan

This record describes the second part of the Obstacle Scan Task: the servo
scan. After OBST_STOP, the Vehicle Controller sets SCAN_REQ. The scan turns
the ultrasonic sensor through 13 points and records the nearest echo at each
point. The profile analysis then chooses the shorter way around the
obstacle. It does not estimate the obstacle width.

- Code: [`src/obstacle_scan.c`](../src/obstacle_scan.c), [`src/obstacle_scan.h`](../src/obstacle_scan.h)
- Servo: [`src/servo.c`](../src/servo.c), [`src/servo.h`](../src/servo.h)
- Profile analysis: [`src/obstacle_profile.c`](../src/obstacle_profile.c), [`src/obstacle_profile.h`](../src/obstacle_profile.h)
- Host unit tests: [`tests/test_obstacle_scan.c`](../tests/test_obstacle_scan.c), [`tests/test_obstacle_profile.c`](../tests/test_obstacle_profile.c)
- Bench test: [`src/scan_test.c`](../src/scan_test.c)
- Related: [forward obstacle watch](OBSTACLE_MONITOR.md), [ultrasonic driver](ULTRASONIC_DRIVER.md)

## Mounted servo limits

Measured on 2026-10-27 with `servo_calibration`, servo mounted on the robot:

| Item | Value |
| --- | --- |
| Pulse below 1500 µs | Sensor turns **right** (seen from behind the robot) |
| Pulse above 1500 µs | Sensor turns **left** |
| Right touch point | About 1040 µs (about 41° right) |
| Left touch point | About 1960 µs (about 41° left) |
| Software clamp in `servo.c` | 1100–1900 µs (about 36° each side) |
| Scan range | ±30°, about 11° from each touch point |

The ±15° limit in the [Week 8 notes](week8_change_requirement_1_notes.md#20-mounted-servo-limit)
was an estimate. The measured range is wider.

Not yet confirmed: that 1500 µs points the sensor straight ahead.

## Scan points

Angle: negative is right, positive is left. Pulse = 1500 µs + angle ×
1000/90 µs, rounded.

| Index | Angle | Pulse |
| ---: | ---: | ---: |
| 0 | 30° right | 1167 µs |
| 1 | 25° right | 1222 µs |
| 2 | 20° right | 1278 µs |
| 3 | 15° right | 1333 µs |
| 4 | 10° right | 1389 µs |
| 5 | 5° right | 1444 µs |
| 6 | Straight ahead | 1500 µs |
| 7 | 5° left | 1556 µs |
| 8 | 10° left | 1611 µs |
| 9 | 15° left | 1667 µs |
| 10 | 20° left | 1722 µs |
| 11 | 25° left | 1778 µs |
| 12 | 30° left | 1833 µs |
| 13–15 | Not used | — |

The index is 4 bits. The first version used 7 points in 10° steps (3-bit
index); 5° steps were chosen on 2026-10-27 for a finer profile. The order is right to left; the servo then returns to
the centre so the forward watch looks straight ahead.

The HC-SR04+ beam is about 30° wide, so each point also sees about 15° to
each side. The scan therefore covers about ±45°. Neighbouring points
overlap, so the scan cannot find an obstacle edge precisely. It can tell
which side is nearer.

## Sequence and timing

```text
start --> move to idx 0, wait 150 ms --> 2 readings --> move 5°, wait 60 ms
      --> 2 readings --> ... idx 12 --> move to centre, wait 150 ms --> DONE
```

| Item | Value | Reason |
| --- | --- | --- |
| First settle | 150 ms | Centre to 30° right |
| Step settle | 60 ms | One 5° step; hidden by the 60 ms trigger interval |
| Return settle | 150 ms | 30° left to centre |
| Readings per point | 2 | The driver needs 60 ms between triggers |
| Expected scan time | About 1.9 s | Confirm with the bench test |

Every call to `obstacle_scan_service()` returns at once. During a scan the
scan module owns the ultrasonic driver; no other code may trigger it.

## Combining the readings at one point

| Readings | Point result | Reason |
| --- | --- | --- |
| Two VALID | The nearer distance | The nearer value is safer for a bypass |
| VALID + NO_RETURN | The VALID distance | Something is there |
| VALID + failure | The VALID distance | |
| NO_RETURN + NO_RETURN or failure | NO_RETURN | Nothing within 511 mm |
| Two failures | No data (`0x0200`) | Unknown |

A failure is a NO_RISE or STUCK_HIGH result below the fault streak. A fault
streak of 5 stops the scan, sends the servo to the centre, and returns
`OBSTACLE_SCAN_FAULT`.

## Data representation

Each profile point uses the ultrasonic result layout without the streak:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–8 | Distance code (0 = TOO_NEAR when VALID) | 9 |
| 9–10 | Status: VALID, NO_RETURN, or NO_RISE (no data) | 2 |
| 11–15 | Always 0 | — |

Logical size: 11 bits per point, 143 bits for the 13-point profile. The
profile stays inside the Obstacle Scan Task. Only the PROFILE word below
goes to the Vehicle Controller.

## Profile analysis and the PROFILE event

Decision 2026-10-27: the Vehicle Controller does not need the obstacle
width. The project brief does not ask for it so far, and the scan cannot
measure it reliably: the beam blurs edges, a flat face reflects the sound
away at more than about 20–25°, and a wide obstacle extends past ±30°.
BYPASS needs only the shorter way around.

For each side, the analysis starts at straight ahead and steps outward 5° at
a time. The **free step** is the first step where the beam is past the
obstacle:

| Point | Free? |
| --- | --- |
| NO_RETURN | Yes |
| VALID, at least 150 mm farther than the obstacle | Yes: the beam sees past the edge |
| VALID, less than 150 mm farther | No: still the obstacle (run 1 corners were 20–35 mm farther) |
| No data | Never |

- The free point and the next point outward must both be free, so one stray
  NO_RETURN is not an edge. The 30° point may be free alone.
- Obstacle distance = the nearest VALID point (511 when there is none).
- Clearer side = the smaller free step. On a tie, the side with more room at
  30°. On a full tie, LEFT. Both sides blocked = NONE: reverse and rescan.

NO_RETURN counts as free here, but it never releases a STOP in the
[forward watch](OBSTACLE_MONITOR.md). Without this, the robot could not
choose a side in open space. A flat face angled away can also give
NO_RETURN, so the forward watch must run during BYPASS. It stops the robot
if the scan missed part of the obstacle.

PROFILE word:

| Bits | Field | Width | Values |
| --- | --- | ---: | --- |
| 0–8 | Obstacle distance | 9 | Ultrasonic distance code; 0 = TOO_NEAR; 511 = no VALID point |
| 9–11 | Right free step | 3 | 0–6 = free at 0–30°; 7 = blocked |
| 12–14 | Left free step | 3 | 0–6 = free at 0–30°; 7 = blocked |
| 15 | Spare | 1 | Always 0 |

Logical size: 15 bits. The turn angle for BYPASS is the free step × 5° plus
a margin for the robot half-width; the Vehicle Controller chooses the
margin.

Run 1 gives `0x5CA2`: obstacle 162 mm, right free at 30°, left free at 25°,
go LEFT.

### Run 2: profile on the robot

2026-10-27. Firmware with the profile analysis. Box ahead, slightly to the
right.

| Idx | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| mm | NR | 383 | 201 | 197 | 197 | 197 | 197 | 197 | 201 | 214 | NR | NR | NR |

Result: `0x4AC5`, obstacle 197 mm, right free at 25°, left free at 20°,
go LEFT. Scan time 1833 ms.

- At 25° right the beam saw past the box to an object at 383 mm. This is at
  least 150 mm farther than the box, so the gap rule marked it free. This is
  the first bench check of the gap rule.
- The corner readings (201 and 214 mm) stayed below the gap and were not
  taken as edges.
- The decision matches a hand check of the rules.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
   tests/test_obstacle_scan.c -o build/test_obstacle_scan
./build/test_obstacle_scan
```

It checks the angle and pulse tables, that every pulse is inside the safe
clamp, and the combine rules.

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
   tests/test_obstacle_profile.c src/obstacle_profile.c \
   -o build/test_obstacle_profile
./build/test_obstacle_profile
```

It checks 12 profiles, including run 1, open space, a wall, a stray
NO_RETURN, no data, TOO_NEAR, and both tie rules. Three deliberate rule
changes (no data as free, one free point as an edge, no 150 mm gap) each
make the test fail.

Result on 2026-10-27: all tests pass.

## Bench test

```sh
cmake --build build --target scan_test
picotool load -f -x build/scan_test.uf2
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
```

| Control | Action |
| --- | --- |
| GP20 | Run one scan |
| GP21 | Repeat a scan every 2 s on or off |

The servo moves. Keep fingers away from the sensor mount. The motors are
not driven.

Procedure:

1. Open space: expect NO_RETURN at most points.
2. Box straight ahead at about 200 mm: expect the nearest point at idx 6,
   with several neighbours also VALID because of the beam width.
3. Move the box to the right side: the nearest point moves to idx 0–5.
4. Move the box to the left side: the nearest point moves to idx 7–12.
5. Record the scan time.

### Run 1: object straight ahead

2026-10-27. Robot still, servo mounted, one object in front. The object type
and width were not recorded.

| Idx | Angle | Result |
| ---: | ---: | --- |
| 0 | 30° right | NO_RETURN |
| 1 | 25° right | 187 mm |
| 2 | 20° right | 175 mm |
| 3 | 15° right | 166 mm |
| 4 | 10° right | 166 mm |
| 5 | 5° right | 162 mm |
| 6 | Ahead | 166 mm |
| 7 | 5° left | 167 mm |
| 8 | 10° left | 171 mm |
| 9 | 15° left | 183 mm |
| 10 | 20° left | 196 mm |
| 11 | 25° left | NO_RETURN |
| 12 | 30° left | NO_RETURN |

Scan time: **1829 ms** (expected about 1.9 s).

Conclusions:

- The scan sequence, the servo direction, and the timing work.
- The object face is about 165 mm from the sensor (about 107 mm from the
  robot front). Idx 3–7 agree within one sensor step (about 4 mm), so the
  nearest point (idx 5) does not show the object centre.
- The object is seen over 45° (25° right to 20° left). This is the object
  width plus the beam width. With the object width known, the effective
  beam width = 45° − object angular width. The profile step needs this
  value.
- The readings increase toward both ends because the beam edge sees the
  object corners, which are farther away.

## Open items

- Repeat run 1 with an object of known width and distance; calculate
  the effective beam width.
- Record the right-side and left-side runs (procedure steps 3 and 4).
- Confirm that 1500 µs points straight ahead.
- Check whether 60 ms is enough settle time; if readings at a point
  disagree, increase it.
- Bench-test the profile with the obstacle on the left (expect go RIGHT),
  facing a wall (expect NONE), and in open space.
- Check the 150 mm gap rule with an obstacle in front of a wall.
- The Vehicle Controller must keep the forward watch running during
  BYPASS.
