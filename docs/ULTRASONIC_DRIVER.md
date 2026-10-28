# Ultrasonic driver and bench test

Status: Implemented; bench-tested; mounted range test pending

This record describes the non-blocking HC-SR04+ driver for the Spatial
Optimizer (Buddy 5) and the `ultrasonic_test` bench target. The data format
follows [Change Requirement 1](week8_change_requirement_1_notes.md#13-buddy-5--ultrasonic-distance).

## Wiring

| Signal | GPIO | Note |
| --- | --- | --- |
| Echo | GP2 | HC-SR04+ on 3.3 V; no divider needed. See [calibration results](CALIBRATION_RESULTS.md#hc-sr04). |
| Trigger | GP3 | 10 µs HIGH pulse |

## Files

| File | Purpose |
| --- | --- |
| [`src/ultrasonic.h`](../src/ultrasonic.h) | Public API, status codes, packed result layout |
| [`src/ultrasonic.c`](../src/ultrasonic.c) | Echo interrupt, timeout detection, conversion, publication |
| [`src/ultrasonic_test.c`](../src/ultrasonic_test.c) | Bench test: live readings and 50-reading batch statistics |

The driver has no RTOS dependency. An RTOS build can release `sem_echo` from
the optional echo hook.

## How a measurement works

```text
Obstacle Task / main loop           Echo interrupt (GP2)            Shared
-------------------------           --------------------            ------
ultrasonic_trigger()
  echo already HIGH? -> STUCK_HIGH (fault)  -------------------->   result word
  arm, 10 µs pulse on GP3
                                    rising edge:  record time
                                    falling edge: record time,
                                                  call hook
ultrasonic_service()  (any time later; never waits)
  echo complete     -> VALID + distance  ----------------------->   result word
  no rise in 2 ms   -> NO_RISE (fault)  ------------------------>   result word
  HIGH for > 3 ms   -> NO_RETURN  ------------------------------>   result word
```

- The interrupt only records a timer value and changes the phase. This follows
  the [interrupt conventions](INTERRUPT_CONVENTIONS.md#event-driven-work).
- Timeouts are found by polling, because a missing edge produces no interrupt.
  This is the [no-event polling](INTERRUPT_CONVENTIONS.md#polling-for-no-event)
  pattern.
- The edge times come from the interrupt. A late `ultrasonic_service()` call
  does not change the measured distance.
- The driver refuses a new trigger within 60 ms of the last one. A late echo
  from the previous ping cannot then match the new ping.
- The driver uses a raw GPIO interrupt handler, so it can share the GPIO
  interrupt with the encoder callback.

## Packed result

One 16-bit word. Fixed masks and shifts define the layout; C bit-fields are
not used.

```text
 15 14 | 13 12 11 | 10  9 | 8 7 6 5 4 3 2 1 0
 spare |  streak  | status|   distance code
```

| Field | Producer | Consumer | Valid range | Resolution | Reserved meaning | Bits |
| --- | --- | --- | --- | --- | --- | ---: |
| Distance code | `ultrasonic_service()` | Obstacle Task | 60–511 mm | 1 mm | `0` with VALID = TOO_NEAR (less than 60 mm); 1–59 unused; `0` when status is not VALID | 9 |
| Status | `ultrasonic_service()`, `ultrasonic_trigger()` | Obstacle Task | VALID, NO_RISE, STUCK_HIGH, NO_RETURN | 4 states | NO_RISE with streak 0 = no result yet | 2 |
| Fault streak | `ultrasonic_service()` | Obstacle Task | 0–5 | 1 | `5` = sensor fault; saturates | 3 |

Status codes:

| Code | Status | Meaning | Streak |
| ---: | --- | --- | --- |
| 0 | VALID | Echo returned from 0–511 mm. Distance code 0 means TOO_NEAR. | Reset to 0 |
| 1 | NO_RISE | Fault: Echo did not go HIGH within 2 ms of the trigger | +1 |
| 2 | STUCK_HIGH | Fault: Echo was HIGH before the trigger | +1 |
| 3 | NO_RETURN | Sensor pinged (Echo rose), but no echo returned from 0–511 mm | Reset to 0 |
| Spare | — | — | — | — | Always 0 | 2 |

Rules:

- **No echo is unknown, not clear.** Treat every status except VALID as
  unknown.
- **TOO_NEAR:** a VALID echo from less than 60 mm has distance code `0`.
  Something is in front of the sensor, but the exact distance is not
  reliable. Treat it as the nearest possible obstacle. Use
  `ultrasonic_result_too_near()`. Below about 40 mm the sensor reports wrong
  distances (run 6). The robot front is about 58 mm from the sensor
  (run 7), so an obstacle that touches the front gives TOO_NEAR or about
  60 mm. Both mean "stop".
- The field uses the 9-bit range up to 511 mm. A 9-bit field holds 512
  values, so the extra 111 mm of look-ahead beyond the planned 400 mm costs
  no extra bits. An echo from more than 511 mm is reported as NO_RETURN.
- History: the first version used 0–400 mm and reported 401–515 mm as code
  `511`. The bench test showed that a target at 400 mm flipped between `400`
  and `511`, and the far code gave no decision value. The range was then
  changed to 0–511 mm. Runs 6 and 7 then added the 60 mm floor and the
  TOO_NEAR code. This replaces the 0–400 mm
  range in the [Week 8 notes](week8_change_requirement_1_notes.md#13-buddy-5--ultrasonic-distance).
- NO_RETURN is not a fault. The sensor answered the trigger, so it works.
  NO_RETURN is still unknown: open space, fabric, a tilted face, or a narrow
  object all give NO_RETURN. The Vehicle Controller owner must decide how to
  drive on NO_RETURN.
- The streak counts consecutive NO_RISE and STUCK_HIGH results. A VALID or
  NO_RETURN result resets it. A streak of 5 means a real sensor fault.
- Before the first measurement, the word is `0x0200` (NO_RISE, streak 0).
  A real fault always has a streak of 1 or more, so this word cannot come
  from a measurement. Use `ultrasonic_result_is_none()`.
- Change from the Week 8 notes: [section 17](week8_change_requirement_1_notes.md#17-ultrasonic-result-status)
  lists three outcomes. The bench test showed that the HC-SR04+ holds Echo
  HIGH when no echo returns, so open space looked like a fault. The fourth
  code separates that case. This change needs team agreement.
- Synchronisation: the word has one writer, the task that calls
  `ultrasonic_service()`. An aligned 16-bit store is atomic on the RP2040, so
  a reader on either core sees a whole word.
- Raw echo time stays in the driver. `ultrasonic_debug_latest()` exposes it
  for the bench test only.

### Wider transient arithmetic

The driver uses 32-bit timer values and 32-bit arithmetic for the pulse width
and millimetre conversion. It converts the result to the 9-bit field before
publication. This matches
[Change Requirement 1, section 4](week8_change_requirement_1_notes.md#4-wider-arithmetic-can-still-be-valid).

Conversion: `distance_mm = (pulse_us × 343 + 1000) / 2000`, rounded to the
nearest mm. The constant assumes about 20 °C.

## Timing constants

| Constant | Value | Reason |
| --- | ---: | --- |
| Trigger pulse | 10 µs | HC-SR04 family minimum |
| Rise timeout | 2 ms | About 9 times the measured 212–217 µs trigger-to-rise latency. It does not limit range. |
| Echo timeout | 3 ms | `3000 µs × 343 m/s ÷ 2 ≈ 515 mm`. A 511 mm echo takes about 2.98 ms. The timeout only stops the wait; the 511 mm limit is applied to the converted distance. |
| Minimum trigger interval | 60 ms | Lets the previous ping fade before the next ping |
| Test trigger period | 100 ms | Same as the planned Obstacle Task |

With these timeouts, the results mean:

| Target | Result |
| --- | --- |
| Less than 60 mm | VALID + `0` (TOO_NEAR) |
| 60–511 mm | VALID + distance |
| More than 511 mm, or no echo | NO_RETURN (unknown) |

The HC-SR04+ holds Echo HIGH by itself when no echo returns. The echo timeout
ends the measurement early. In the open-space test, the sensor always
released Echo before the next trigger 100 ms later.

## Run the bench test

```sh
cmake --build build --target ultrasonic_test
picotool load -f -v -x build/ultrasonic_test.uf2
picocom /dev/ttyACM0
```

| Control | Action |
| --- | --- |
| GP20 | Record 50 readings and print counts, mean, standard deviation, minimum, maximum, and trigger-to-echo latency |
| GP21 | Turn the live per-reading output on or off |

A live line looks like this:

```text
#42 VALID      word 0x00B1 streak 0  177 mm  pulse 1032 us  rise 215 us
```

## First bench results

Date: 2026-10-27. Sensor unmounted, on the bench.

### Run 1: 30 ms timeouts, three status codes

This run used 30 ms rise and echo timeouts and the original three status
codes. At that time STUCK_HIGH also covered an echo that stayed HIGH too long.

| Observation | Result |
| --- | --- |
| Target at about 120 mm | 117, 122, and 126 mm; all VALID |
| Trigger-to-rise latency | 212–217 µs, very stable |
| Pulse-width steps | About 25 µs (one 40 kHz cycle), so about 4.3 mm per step |
| Target at about 1 m, 50-reading batch | 50 VALID. 48 readings: mean 1016.3 mm, std 10.6 mm, range 999–1056 mm |
| Single short readings | 2 of 50 near 480 mm; an object at the beam edge |
| STUCK_HIGH | 2 single results while the target moved; the next reading was VALID |

Conclusions:

- The packed word decoded correctly for VALID, beyond-400 mm (`0x01FF`, a
  code that the driver no longer uses), and
  STUCK_HIGH with streak 1 (`0x0C00`).
- The HC-SR04+ resolution is about 4.3 mm, not 1 mm. Use this when the
  distance resolution is decided.
- A single reading can come from a different object. Confirm an obstacle with
  two consecutive readings, as the design review requires for `OBST_STOP`.
- When no echo returns, the HC-SR04+ holds Echo HIGH. A missing echo was
  therefore reported as STUCK_HIGH, not NO_RISE.

### Run 2: 2 ms and 3 ms timeouts, open space

The sensor pointed at open space with nothing within about 500 mm. The
firmware still had three status codes.

| Observation | Result |
| --- | --- |
| Status | 60 of 60 STUCK_HIGH (echo too long), word `0x2C00` |
| Trigger-to-rise latency | 213–215 µs on every reading |
| Echo HIGH before the next trigger | Never seen |
| Streak | Saturated at 5; `FAULT` shown |

Conclusions:

- The sensor worked on every ping, but the driver reported a fault. Open
  space would trigger a false sensor fault on every clear straight.
- The HC-SR04+ releases Echo within 100 ms, so it is ready for the next ping.
- Result: the driver now reports this case as NO_RETURN, which resets the
  streak. STUCK_HIGH now means only "Echo HIGH before the trigger".

### Run 3: four status codes, open space

| Observation | Result |
| --- | --- |
| Status | 50 of 50 NO_RETURN, word `0x0600` |
| Trigger-to-rise latency | 212–216 µs |
| Streak and fault | Streak 0 on every reading; no `FAULT` |

### Run 4: Trigger wire (GP3) disconnected and reconnected

| Step | Results |
| --- | --- |
| Connected, open space | NO_RETURN, streak 0 (`0x0600`) |
| Wire disconnected | NO_RISE with streak 1, 2, 3, 4 (`0x0A00`, `0x1200`, `0x1A00`, `0x2200`) |
| Fifth miss | NO_RISE, streak 5, `FAULT` (`0x2A00`), about 0.5 s after disconnection |
| Held disconnected | 108 readings at streak 5 |
| Wire pushed in | 1 VALID (59 mm, hand near sensor), then 3 NO_RISE from contact bounce |
| Wire seated | NO_RETURN, streak 0 |

Conclusions:

- The fault path works: NO_RISE increases the streak, and the fifth
  consecutive fault sets `FAULT`.
- One VALID or NO_RETURN result clears the streak.
- Three misses from contact bounce did not reach `FAULT`. The threshold of 5
  filters short glitches.

### Run 5: distance accuracy, hand-held target

A flat target was held by hand at each distance. The GP20 batch summaries
were lost: two recorder processes read the serial port at the same time. The
values below come from the per-reading lines that remained, grouped by their
sequence numbers. Only stable plateaus are included; readings taken while the
target moved are excluded. The firmware still reported echoes from 401 to
515 mm as code `511`; the raw millimetre values are used here.

| Target | Readings | Mean | Std | Min | Max | Drop-outs | Note |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 20 mm | 45 | 49.3 | 7.1 | 41 | 74 | 15 NO_RETURN | Unstable; confirmed by run 6 |
| 50 mm (unconfirmed) | 54 | 61.4 | 2.6 | 56 | 68 | 16 NO_RETURN | |
| 100 mm | 270 | 96.8 | 1.6 | 87 | 108 | 0 | |
| 200 mm | 93 | 210.5 | 3.0 | 196 | 217 | 0 | |
| 300 mm | 83 | 316.1 | 4.1 | 312 | 336 | 1 NO_RETURN | |
| 400 mm | 146 | 398.1 | 5.2 | 385 | 422 | 0 | Flipped between `400` and `511` |
| 600 mm | — | — | — | — | — | All NO_RETURN | Beyond the 515 mm echo timeout |

The two near plateaus were not labelled during the test. Run 6 confirmed the
20 mm row. The 50 mm row is still the order of the procedure, not a
confirmed record.

Conclusions:

- The shortest reading seen was 41 mm. Below about 60 mm the readings are
  unstable and about 1 in 4 is NO_RETURN. Treat 60 mm as the provisional
  reliable minimum.
- From 100 mm to 400 mm the standard deviation is 1.6–5.2 mm with almost
  no drop-outs.
- The mean errors (−3, +11, +16, −2 mm) do not grow in proportion to
  distance, so a speed-of-sound error is not the likely cause. Hand placement
  and the measurement reference point are more likely. Repeat with a fixed
  target before using these errors as a correction.
- A target at 400 mm flipped between `400` and `511`. Code `511` gave no
  decision value. The field is now 0–511 mm with no special codes, and an
  echo beyond 511 mm is NO_RETURN.

### Run 6: near-range check at 20 mm

A flat target was held by hand at 20 mm. The firmware used the 0–511 mm
field. One recorder process read the port.

| Observation | Result |
| --- | --- |
| Most readings | VALID, 40–55 mm |
| Other readings | Often 60–90 mm; some 100–250 mm, probably from the hand or the table |
| Lowest reported value | 40 mm |
| Status | Mostly VALID; the sensor does not flag the error |

Conclusions:

- The sensor has a blind zone of about 40 mm. The 8-cycle 40 kHz burst
  lasts about 200 µs, and a 40 mm echo returns after about 233 µs. An
  echo from a nearer target arrives while the transducer still rings.
- In the blind zone the sensor gives a wrong VALID distance, not NO_RETURN.
  The driver cannot detect this.
- A 20 mm target reads about 45 mm. This is still far below the 200 mm stop
  threshold, so the stop decision is not affected.
- The driver now reports a VALID echo below 60 mm as TOO_NEAR (code `0`)
  and not as a distance.

### Run 7: sensor-to-robot-front offset

The sensor was mounted on the robot. A flat target touched the front of the
robot. Firmware: 0–511 mm field, before the 60 mm floor.

| Observation | Result |
| --- | --- |
| Readings | 80 of 80 VALID |
| Mean | 57.9 mm, std 2.2 mm |
| Values | 56 mm ×45, 60 mm ×21, 61 mm ×14 |
| Pulse | 327 or 353 µs; one 25 µs sensor step apart |

Conclusions:

- The echo distance from the sensor to the robot front is about 58 mm.
  At about 50 mm the hand-held test read about 11 mm high, so measure the
  true offset with a ruler before use.
- The sensor blind zone (about 40 mm) is inside the robot body.
- The stop threshold needs a stated reference point:

| Stop rule | Sensor threshold |
| --- | ---: |
| 200 mm from the sensor | 200 mm |
| 200 mm from the robot front | 200 + 58 ≈ 258 mm |

- The 511 mm field reaches about 453 mm in front of the robot.

## Measurements to record

Use a flat, hard target square to the sensor unless the row says otherwise.
Measure the true distance from the sensor face. Do not open a second serial
reader while a recorder is running.

### Behaviour without a target

| Test | Status seen | Note |
| --- | --- | --- |
| Open space, nothing within about 500 mm | NO_RETURN | Done in runs 2 and 3. The HC-SR04+ holds Echo HIGH for more than 3 ms and releases it within 100 ms. |
| Target removed during live output | | |

### Material and angle

| Target at 200 mm | VALID / 50 | Mean |
| --- | ---: | ---: |
| Flat hard face, 0° | | |
| Flat hard face, 15° | | |
| Flat hard face, 30° | | |
| Fabric or foam | | |
| Narrow object (pen or table leg) | | |

## Open items

- Repeat the distance test with a fixed target and the sensor mounted on the
  robot. Confirm the 50 mm behaviour.
- Complete the material and angle table.
- Get team agreement on the four status codes and on how the Vehicle
  Controller drives on NO_RETURN.
- Decide whether the Obstacle Task needs the 1 mm resolution
  (see [notes, section 50](week8_change_requirement_1_notes.md#50-resolution-is-a-separate-decision-from-range)).
