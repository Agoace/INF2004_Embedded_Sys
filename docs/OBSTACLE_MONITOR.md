# Forward obstacle watch

This record describes the first part of the Obstacle Scan Task: the forward
watch. It turns each packed ultrasonic result into events for the Vehicle
Controller. The servo scan and the obstacle profile are later steps.

- Code: [`src/obstacle.c`](../src/obstacle.c), [`src/obstacle.h`](../src/obstacle.h)
- Host unit test: [`tests/test_obstacle.c`](../tests/test_obstacle.c)
- Bench test: [`src/obstacle_test.c`](../src/obstacle_test.c)
- Input: the 16-bit result from the [ultrasonic driver](ULTRASONIC_DRIVER.md)

## Where it fits

```text
ultrasonic driver        forward watch (this record)          Vehicle Controller
16-bit result     --->   MONITORING / APPROACHING /    --->   OBST_NEAR  -> slow
every 100 ms             CONFIRMED / SENSOR_FAULT             OBST_STOP  -> stop, SCAN_REQ
                                                              OBST_CLEAR -> cruise
                                                              SENSOR_FAULT -> FAULT
```

The module has no hardware or RTOS calls. The same code runs on the Pico and
in the host unit test. One task owns the monitor; other tasks receive only
the event word.

## Thresholds

All distances are at the sensor face. The robot front is about 58 mm in
front of the sensor ([ultrasonic run 7](ULTRASONIC_DRIVER.md#run-7-sensor-to-robot-front-offset)).

| Name | Value | Meaning |
| --- | ---: | --- |
| `OBSTACLE_STOP_MM` | 258 mm | Enter STOP below this: 200 mm from the robot front + 58 mm |
| `OBSTACLE_NEAR_MM` | 400 mm | Enter NEAR below this: about 342 mm from the robot front |
| `OBSTACLE_HYSTERESIS_MM` | 20 mm | Dead band: leave STOP at 278 mm or more; leave NEAR at 420 mm or more |
| `OBSTACLE_CONFIRM_COUNT` | 2 | Readings in a row that confirm every change (design review: "<20 cm x2") |

The dead band is necessary. A target on a threshold changes by one sensor
step (about 4 mm) between readings. Without the dead band, the state would
change on almost every reading.

Confirm the 58 mm offset with a ruler, then change
`OBSTACLE_SENSOR_TO_FRONT_MM` if necessary.

## States and events

```text
                 2 x < 400                 2 x < 258
  MONITORING  -------------> APPROACHING -------------> CONFIRMED
              <-------------             <-------------
               2 x >= 420 or               2 x VALID >= 278
               2 x NO_RETURN               (never NO_RETURN)

  MONITORING --- 2 x < 258 ---> CONFIRMED   (fast approach)
  any state  --- fault streak 5 ---> SENSOR_FAULT (latched)
```

| Event | Code | Sent when the state changes to |
| --- | ---: | --- |
| NONE | 0 | No change |
| OBST_CLEAR | 1 | MONITORING |
| OBST_NEAR | 2 | APPROACHING |
| OBST_STOP | 3 | CONFIRMED |
| SENSOR_FAULT | 4 | SENSOR_FAULT |

Rules:

- **TOO_NEAR is a STOP reading.** Distance code 0 is below every threshold.
- **NO_RETURN is unknown, not clear.** It can end APPROACHING, because the
  target can move out of range. It can never release a STOP. Only two valid
  echoes at 278 mm or more release a STOP. Reason: a soft or angled
  obstacle can absorb or deflect the echo. Without this rule, the robot
  could drive into it.
- **In open space a STOP stays latched.** When the obstacle is removed and
  nothing is within 511 mm, no valid echo returns, so the STOP cannot
  release. This is intended (decision 2026-10-27). The Vehicle Controller
  calls `obstacle_init()` after its servo scan (STOP → scan → bypass). On
  the bench, press GP20. A timed release after 10 NO_RETURN results was
  considered and rejected.
- **A STOP is released in two steps.** CONFIRMED goes to APPROACHING first.
  OBST_CLEAR needs its own confirmation.
- **A single sensor fault is ignored.** A NO_RISE or STUCK_HIGH result with a
  streak below 5 does not change the counts. A streak of 5 latches
  SENSOR_FAULT. Only `obstacle_init()` clears it.
- **"No result yet" (`0x0200`) is ignored.**
- **A reading in a dead band resets the counts.** For example, 265 mm after
  one 250 mm reading does not confirm a STOP.

## Data representation

Week 8 Change Requirement 1 applies. Both words use explicit masks and
shifts.

Event word, sent to the Vehicle Controller:

| Bits | Field | Width | Values |
| --- | --- | ---: | --- |
| 0–8 | Distance code of the reading that caused the event | 9 | Same as the ultrasonic distance code; 0 for TOO_NEAR, NO_RETURN, and SENSOR_FAULT |
| 9–11 | Event | 3 | 0–4; 5–7 unused |
| 12–15 | Spare | 4 | Always 0 |

Example: `0x06ED` = OBST_STOP at 237 mm.

Monitor word, private to the owning task:

| Bits | Field | Width | Values |
| --- | --- | ---: | --- |
| 0–1 | State | 2 | MONITORING, APPROACHING, CONFIRMED, SENSOR_FAULT |
| 2–3 | Stop count | 2 | 0–2 |
| 4–5 | Near count | 2 | 0–2 |
| 6–7 | Clear count | 2 | 0–2 |
| 8–9 | Release count | 2 | 0–2 |
| 10–15 | Spare | 6 | Always 0 |

The logical size is 10 bits. The state needs 2 bits for 4 values. Each
count needs 2 bits for 0–2.

## Host unit test

The test runs on the development computer and needs no hardware:

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
   tests/test_obstacle.c src/obstacle.c -o build/test_obstacle
./build/test_obstacle
```

It checks 13 cases: approach and stop, a single reading, a fast approach,
TOO_NEAR, NO_RETURN during STOP, the two-step release, NO_RETURN during a
release, both dead bands, NO_RETURN in APPROACHING, a single fault, the
fault latch, "no result yet", and the event word layout.

Result on 2026-10-27: all tests pass. A deliberate change to the NO_RETURN
rule makes the test fail, so the test detects that error.

## Bench test

```sh
cmake --build build --target obstacle_test
picotool load -f -x build/obstacle_test.uf2
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
```

The test does not drive the motors or the servo.

| Control | Action |
| --- | --- |
| GP21 | Show every reading on or off |
| GP20 | Reset the monitor; this also clears SENSOR_FAULT |

Events start with `>>>`. When every-reading output is off, the test prints a
status line every second.

Procedure:

1. Start with nothing in front of the sensor. Expect MONITORING.
2. Move a flat target slowly from 600 mm to 100 mm. Expect OBST_NEAR near
   400 mm, then OBST_STOP near 258 mm.
3. Hold the target at about 258 mm. Expect no repeated events.
4. Remove the target quickly. Expect no event: NO_RETURN does not release a
   STOP. The state stays CONFIRMED until GP20 is pressed.
5. Show the target at 300 mm, then remove it. Expect OBST_NEAR, then
   OBST_CLEAR.
6. Disconnect the trigger wire. Expect SENSOR_FAULT after about 0.5 s.
   Reconnect it and press GP20.

Results on 2026-10-27 (hand-held target, user report):

| Check | Result |
| --- | --- |
| Open space | MONITORING, NO_RETURN |
| Approach to 106 mm | OBST_STOP; state CONFIRMED |
| Target removed | Stays CONFIRMED with NO_RETURN, as intended; GP20 resets |
| General behaviour | Works as specified |

Steps 3, 5, and 6 were not recorded one by one.

## Open items

- Record steps 3, 5, and 6 of the bench test one by one.
- Measure the sensor-to-front offset with a ruler.
- Agree the NEAR threshold and the SLOW speed with the Vehicle Controller
  owner.
- Decide how the Vehicle Controller asks for a reset after SENSOR_FAULT.
- The Vehicle Controller must call `obstacle_init()` after the servo scan,
  or a STOP stays latched in open space.
- Ignore these events during HUMP_TRAVERSE (design review). The Vehicle
  Controller does this, not this module.
- Next steps: servo scan on SCAN_REQ, then the obstacle profile event.
