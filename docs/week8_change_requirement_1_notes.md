# Week 8 Tribunal — Change Requirement 1: Minimum-Bit Data Representation

## Purpose

This file consolidates the discussion, calculations, decisions, caveats, and open measurements arising from **Change Requirement 1: Minimum-Bit Data Representation** during the Week 8 Tribunal.

It covers the system-wide interpretation of the requirement, the decisions for Buddies 1–5, the detailed Spatial Optimizer analysis, concurrency implications, and the measurements still needed before final bit widths are frozen.

---

# 1. Injected requirement

The injected requirement says that application data must use only the number of bits required to represent its valid range.

It applies to:

- motion control and encoders;
- line and barcode sensing;
- obstacle detection;
- hump estimation;
- vehicle/controller states;
- task-to-task shared data;
- ISR-to-task shared data;
- inter-task messages;
- MQTT telemetry and command payloads.

Other explicit requirements:

- use C bit-fields or explicit masks/shifts where appropriate;
- independent faults should be individual bits in a shared fault mask;
- MQTT application data should be compact binary, not individual text values;
- packed shared data must remain race-safe;
- Week 13 must include:
  - a data representation table;
  - RAM usage before/after;
  - message size before/after;
  - correctness/performance evidence.

Source: `Change Requirement 1.docx`.

---

# 2. Main design interpretation

The main conclusion was:

> **Size application fields from the range that can actually affect robot behaviour, not blindly from the maximum capability of the hardware.**

Examples:

- an ultrasonic sensor may physically measure metres, but if obstacle avoidance only cares about 0–400 mm, the shared profile does not need a multi-metre range;
- once clearance exceeds the robot's required traversal envelope, additional clearance does not change the pass/fail decision;
- once an obstacle extends beyond the scanner's observable field of view, exact width is no longer observable or useful and can be represented by a saturation state such as `extends beyond FOV / wall`.

This lets the system satisfy the bit requirement without throwing away information that is actually needed.

---

# 3. Hardware range vs application-valid range

These are different.

## Hardware range

What the device/peripheral can physically or electrically represent.

Examples:

- full servo pulse range;
- full ultrasonic measurement range;
- hardware timer width;
- ADC width.

## Application-valid range

What the firmware must preserve to meet its required behaviour.

Examples:

- obstacle distance only up to the avoidance horizon;
- clearance only up to the safe-pass threshold;
- only the servo scan positions actually used;
- only real controller states.

The injected requirement is best handled by making shared/application data follow the **application-valid range**.

Low-level drivers may still temporarily use wider native values for arithmetic or peripheral APIs.

---

# 4. Wider arithmetic can still be valid

A useful distinction is:

- **stored/shared application representation** should be minimum-bit;
- **transient arithmetic** can use wider native types when needed for correctness.

Example:

- timer API may return a 64-bit timestamp;
- subtract timestamps safely using the native type;
- convert the result into a bounded application value before publishing it.

Likewise, a 9-bit distance field can still be computed using 32-bit arithmetic.

This should be documented as:

> compact storage representation with wider transient arithmetic where required for correctness.

If the lecturer intends the requirement to apply even to all local transient variables, that interpretation should be clarified.

---

# 5. C bit-fields vs masks/shifts

C bit-fields are convenient for local structures but bit ordering/layout is compiler dependent.

Reasonable use:

- local structures compiled by one known toolchain.

Prefer explicit masks/shifts for:

- MQTT binary packets;
- fixed inter-module formats;
- anything whose exact byte/bit layout must be documented.

For wire formats, define explicit bit positions.

---

# 6. Packed shared data and races

Packing multiple values into one byte/word creates read-modify-write hazards.

Example:

- Task A reads a word and modifies bit 0;
- Task B reads the same old word and modifies bit 1;
- the later write can overwrite the first update.

Safe patterns:

1. one writer owns the packed word;
2. protect updates with a mutex/critical section;
3. use suitable atomics;
4. send complete immutable messages instead of independently editing shared bits.

For ISR/task interaction, avoid making ISR and task independently modify different bits of the same packed word unless the update is atomic.

A good obstacle pattern is:

- Echo ISR captures timing/status and signals `sem_echo`;
- Obstacle Task owns higher-level obstacle/profile state;
- Vehicle Controller owns controller flags/state.

---

# 7. RTOS object width vs logical bits

Some kernel objects have fixed native width.

Example logical mode bits:

- RUN
- LINE_EN
- SCAN_REQ
- SLOW

That is **4 logical application bits**, even if the micro-T-Kernel event flag object is physically a larger machine word.

Document the logical bits; do not attempt to shrink kernel internals.

---

# 8. Buddy 1 — MQTT / telemetry

## Impact

Existing JSON/text-style telemetry is incompatible with the new final requirement.

Textual state names, direction strings, duplicated `has_*` fields, etc. should not be used as the final MQTT representation.

## Decision

Keep:

- MQTT;
- mutex-protected snapshot;
- 110 ms telemetry period, subject to testing.

Change:

- JSON/text payload → packed binary;
- incoming commands → packed binary;
- each subsystem defines field range/resolution;
- Buddy 1 defines the final MQTT bit/byte layout.

## Validation

Need to prove:

- message size decreases;
- pack/unpack is correct;
- telemetry still meets timing under full RTOS load.

---

# 9. Buddy 2 — motion / encoders

## 9.1 Speed

Current Tribunal approach:

> signed encoder counts per 20 ms PID period

Calibration evidence cited:

- about 261 RPM at 100% duty;
- about 221 counts per 20 ms.

A signed 9-bit value represents:

-256 to +255

So the current decision is:

**speed = 9 bits, provisional**

Final action:

- log the real peak 20 ms count delta under actual driving/load before freezing the field.

---

## 9.2 Motor duty

Use one signed value rather than:

- duty magnitude;
- direction string;
- presence flags.

For −100% to +100%:

201 logical values.

`ceil(log2(201)) = 8`

Therefore:

**signed motor duty = 8 bits per motor**

---

## 9.3 Motion command type

Current macro commands:

1. FOLLOW
2. TURN
3. FWD
4. REV
5. ROTATE
6. STOP

Six possibilities require:

`ceil(log2(6)) = 3`

Therefore:

**motion command type = 3 bits**

The current Tribunal proposal is:

**3-bit command type + 9-bit parameter**

But the final parameter width should ideally be checked per command:

- TURN angle;
- FWD distance;
- REV distance;
- ROTATE parameter.

A type-specific payload may save more bits than one generic parameter.

---

## 9.4 Encoder counters

Original idea:

- 32-bit counters grow for the whole run.

Revised decision:

- use wrapping counters;
- encoder ISR is sole writer;
- readers store previous count;
- readers calculate `now - previous`;
- readers do not reset the shared counter.

Modular subtraction must work through:

- forward motion;
- reverse motion;
- wrap-around.

The exact final counter width is still an engineering decision. Either:

- keep a justified native-width wrapping counter; or
- reduce width based on maximum count rate and maximum interval between reads.

---

## 9.5 Cruise speed / wheel diameter

Exact cruise speed still requires wheel diameter measurement.

This matters to Buddy 5 because speed affects:

- stopping distance;
- obstacle stop threshold;
- distance travelled between ultrasonic polls.

Worked example discussed:

- 0.25 m/s;
- two 100 ms polls + 30 ms latency.

`0.25 × 0.230 = 0.0575 m`

≈ **5.75 cm ≈ 5.8 cm**

Alternative 60 ms poll example:

`0.25 × (2×0.060 + 0.030) = 0.0375 m`

≈ **3.75 cm**

Changing polling should still be based on measured system requirements.

---

# 10. Buddy 3 — line + barcode

## 10.1 Three analogue line sensors

Each ADC reading is 12 bits.

Therefore:

- sensor 1 = 12 bits;
- sensor 2 = 12 bits;
- sensor 3 = 12 bits.

Total raw analogue information:

**36 bits**

Full precision is retained for line calculations.

---

## 10.2 Thresholded line pattern

Three binary black/white results:

**3 bits**

---

## 10.3 Digital barcode sample

Digital sample:

**1 bit**

---

## 10.4 Barcode/action code

The discussion requires a 3-bit shared code.

Reasons:

- multiple barcode actions;
- `000` reserved for internal line-lost/recovery semantics;
- five intended actions have been discussed, including Reverse.

Three bits allow up to 8 codes.

Therefore:

**barcode/action code = 3 bits**

Open validation:

- Reverse barcode still needs definition;
- final GP7 wiring must be confirmed;
- decoding at driving speed needs testing.

---

## 10.5 Line sensor validation

The design uses three analogue line sensors plus a separate digital barcode sensor.

Current test firmware does not yet prove the full three-sensor analogue array in final conditions.

Therefore retain 12-bit raw readings until calibration proves which processed representation is safe for consumers.

---

# 11. Buddy 4 — IMU / hump data

## 11.1 Proposed packed snapshot

Current change table proposes two 32-bit words.

### Word 1

- pitch = 8 bits
- heading = 8 bits
- turn rate = 8 bits
- hump state = 2 bits
- collision = 1 bit
- valid = 1 bit
- spare = 4 bits

Total = 32 bits.

### Word 2

- height = 8 bits
- peak = 8 bits
- age/sequence = 16 bits

Total = 32 bits.

So this exact design is:

**64 bits = 8 bytes**

---

## 11.2 Document inconsistency to fix

One Curveball row says the IMU snapshot shrinks from about 20–24 B to about **6 B**.

But two 32-bit words are:

**8 B**

So the final Tribunal/documentation should not claim both 6 B and 8 B.

If 6 B is intended, a different layout must be defined.

---

## 11.3 Heading quantisation issue

If heading is published in 8 bits over 360°:

`360 / 256 = 1.40625° per LSB`

At 90°/s over 13 ms:

`90 × 0.013 = 1.17°`

So the real change can be smaller than one published heading LSB.

Decision:

- retain higher internal precision;
- publish quantised heading if desired;
- do not derive fine turn rate only from the quantised 8-bit heading;
- BYPASS / LINE_REACQUIRE should not depend only on published heading;
- encoder odometry remains important.

This is a good example of **minimum-bit storage not meaning minimum-bit internal computation**.

---

# 12. Buddy 5 — Spatial Optimizer

Buddy 5 scope here is primarily:

- ultrasonic sensing;
- servo scan;
- obstacle profiling;
- compact profile representation.

The main challenge was:

> Should the Spatial Optimizer represent the full sensor/actuator range, or only the range that can affect avoidance decisions?

Decision:

**use decision-relevant application bounds.**

The ±15° mounted servo and 400 mm profile range were a worked example, not the core issue.

---

# 13. Buddy 5 — ultrasonic distance

> **Update 2026-10-27 (bench test):** the implemented range is **0–511 mm**,
> still 9 bits. A 9-bit field holds 512 values, so the range uses every code
> at no extra cost. An echo beyond 511 mm is NO_RETURN. See
> [Ultrasonic driver](ULTRASONIC_DRIVER.md#packed-result).

## Chosen application range

Current working profile range:

**0–400 mm**

Resolution:

**1 mm**

Number of values:

401

Required bits:

`ceil(log2(401)) = 9`

Therefore:

**profile/obstacle distance = 9 bits**

This is a current design decision, not a claim about the sensor's maximum physical range.

---

# 14. Ultrasonic timing within the 100 ms task window

Speed of sound at about 20 °C:

≈ **343 m/s**

The pulse travels to the target and back.

Theoretical one-way distance corresponding to a 100 ms round trip:

`d = (343 × 0.100) / 2`

≈ **17.15 m**

Useful comparisons:

### 400 mm target

`2 × 0.4 / 343 ≈ 2.33 ms`

### 4 m target

`2 × 4 / 343 ≈ 23.32 ms`

### Current 30 ms echo timeout

`343 × 0.030 / 2 ≈ 5.145 m`

Conclusion:

the 100 ms obstacle period is not limited by acoustic travel time at the intended 400 mm avoidance range.

At 400 mm, echo flight is only about **2.3 ms**.

---

# 15. HC-SR04+ / diagnostic behaviour discussed

Current diagnostics include:

- 10 µs trigger pulse;
- 30 ms echo timeout;
- separate no-rising-edge and stuck-high/too-long error cases;
- pulse-duration-to-mm conversion.

Important observed/known behaviour:

- no echo must **not** automatically mean clear;
- soft/fabric surfaces can absorb sound;
- narrow objects may be missed;
- tilt can cause no echo;
- mounted reliable min/max range still needs physical measurement.

A stationary reading around 177 mm was seen in diagnostics, but that is **not a range calibration**.

---

# 16. Echo pulse duration

If raw echo duration itself is shared:

range:

0–30,000 µs

values:

30,001

`ceil(log2(30001)) = 15`

Therefore:

**echo duration = 15 bits**

if it truly must be stored/shared.

Preferred approach:

- keep raw timing local;
- publish compact distance + status instead.

---

# 17. Ultrasonic result status

> **Update 2026-10-27 (bench test):** the implemented status has **four**
> codes in 2 bits: VALID, NO_RISE (fault), STUCK_HIGH (fault: Echo HIGH
> before trigger), and NO_RETURN (sensor works, no echo within 511 mm). The
> HC-SR04+ holds Echo HIGH when no echo returns, so with three codes open
> space was reported as a fault. NO_RETURN is still unknown, not clear. See
> [Ultrasonic driver](ULTRASONIC_DRIVER.md#packed-result).

At least three outcomes were discussed:

1. valid echo;
2. no rising edge;
3. stuck high / too long.

Three states require:

**2 bits**

This is cleaner than negative distance sentinels.

---

# 18. No-echo streak counter

Fault threshold:

five consecutive failed/no-rise triggers.

Counter states:

0,1,2,3,4,5

Six possibilities require:

**3 bits**

---

# 19. Servo calibration

Unobstructed calibration:

- min = 500 µs
- centre = 1500 µs
- max = 2500 µs
- period = 20,000 µs
- frequency = 50 Hz
- about ±90° around centre
- calibration utility step = 10 µs

Important:

the 10 µs calibration step does **not** automatically mean final application resolution must be 10 µs.

---

# 20. Mounted servo limit

With surrounding hardware installed, practical movement is about:

**−15° to +15°**

This is the application-relevant motion unless the mechanical arrangement is changed.

---

# 21. Servo scan index

If final scan points are:

- −15°
- −10°
- −5°
- 0°
- +5°
- +10°
- +15°

there are 7 positions.

`ceil(log2(7)) = 3`

Therefore:

**scan position = 3-bit index**

Recommended mapping:

| Index | Angle |
|---:|---:|
| 0 | −15° |
| 1 | −10° |
| 2 | −5° |
| 3 | 0° |
| 4 | +5° |
| 5 | +10° |
| 6 | +15° |
| 7 | reserved |

The Obstacle Task can locally map the index to PWM.

---

# 22. Mounted servo PWM range

From the linear unloaded calibration:

- −90° ≈ 500 µs
- 0° = 1500 µs
- +90° ≈ 2500 µs

Scale:

`1000 / 90 ≈ 11.111 µs/°`

At ±15°:

≈ ±166.7 µs around centre.

Approximate pulses:

- −15° ≈ 1333 µs
- 0° = 1500 µs
- +15° ≈ 1667 µs

If deliberately using 10 µs application steps:

~1340–1660 µs

Number of values:

`(1660-1340)/10 + 1 = 33`

33 values require:

**6 bits**

But a 3-bit scan index is still better if only seven positions are used.

---

# 23. Obstacle width geometry

> **Update 2026-10-27:** the calculation below uses 400 mm. If the profile
> uses the full 0–511 mm range, the observable width becomes
> `2 × 511 × tan(15°) ≈ 274 mm`. That exceeds an 8-bit 1 mm field (0–255).
> Either cap the width profile at 400 mm, or use a 9-bit width field or a
> coarser width resolution. This is a team decision.

Working assumptions:

- forward profile range = 400 mm;
- scan half-angle = 15°;
- approximately flat obstacle face.

Maximum fully observable width:

`W = 2D tan(theta)`

`W = 2 × 400 × tan(15°)`

≈ **214.36 mm**

Therefore about:

**214 mm**

is the maximum fully observable width under this geometry.

Anything wider extends beyond the scan field.

---

# 24. Width geometry caveat

214 mm assumes **400 mm is forward/perpendicular distance**.

If 400 mm is instead the slant/radial range to the edge rays:

`W = 2R sin(15°)`

≈ **207.06 mm**

Current Tribunal wording uses about 214 mm.

The final documentation should state the geometry assumption.

---

# 25. Obstacle width representation

At 1 mm resolution:

- numeric widths 0–214 mm;
- plus an overflow/wall meaning.

This still fits in:

**8 bits**

Possible encoding:

- `0–214` = measured mm;
- `255` = `>=214 mm / extends beyond FOV / wall`;
- `215–254` reserved.

Alternative:

- make the threshold code itself mean "214 mm or more".

Core semantic rule:

> if the obstacle extends beyond the observable field, do not pretend its exact width is known.

---

# 26. Coarser width resolution

If later validated:

- 5 mm resolution could be around 6 bits;
- 10 mm resolution could be around 5 bits.

Do not reduce precision solely to save bits unless functionality/accuracy requirements are still met.

Current conservative value:

**8 bits at 1 mm.**

---

# 27. Left/right clearance

The system does not need arbitrary large clearance values.

Once clearance is sufficient for safe passage, larger values are decision-equivalent.

Therefore clearance should be a **saturating field**.

The limit should be based on:

- physical robot width;
- turning sweep;
- protruding hardware;
- bypass trajectory;
- safety margin.

Height is irrelevant unless overhead clearance is being checked.

Length can matter indirectly through swept turning footprint.

---

# 28. Clearance measurement required

The final clearance threshold should **not be guessed**.

Measure:

1. intended bypass manoeuvre;
2. maximum swept lateral width;
3. protruding hardware;
4. add agreed safety margin.

Let threshold be:

`C_limit_mm`

At 1 mm resolution, if the top value means "`C_limit_mm` or greater":

required bits:

`ceil(log2(C_limit_mm + 1))`

Example only:

if threshold = 180 mm:

`ceil(log2(181)) = 8`

So 8 bits would be enough in that example.

Final value requires measurement.

---

# 29. Spatial Optimizer state

Current state design has seven states:

1. MONITORING
2. APPROACHING
3. OBSTACLE_CONFIRMED
4. COARSE_SCAN
5. FINE_SCAN
6. BUILD_PROFILE
7. SENSOR_FAULT

Seven states require:

**3 bits**

If the scan design changes because of the mounted ±15° limit, recalculate from the final state count.

---

# 30. Existing scan design vs mounted hardware

Original design assumed roughly:

- coarse scan −90° to +90° in 30° steps;
- fine scan around detected edges.

Mounted servo only permits about ±15°.

Therefore the original broad scan is physically inconsistent unless hardware is changed.

Possible outcomes:

1. redesign profiling around narrow ±15° scan;
2. remount the servo;
3. supplement with robot/body rotation.

For Change Requirement 1, the final bit width must reflect the scan positions actually implemented.

---

# 31. Rescan counter

Current controller allows up to three rescans.

Values:

0–3

Four possibilities:

**2 bits**

---

# 32. Obstacle/mode flags

Logical booleans require one bit each.

Examples:

- obstacle-near active = 1 bit;
- obstacle-stop active = 1 bit;
- SCAN_REQ = 1 bit;
- SLOW = 1 bit;
- auto-start armed = 1 bit, if retained.

Strict interpretation of the requirement means these should be packed rather than separate byte/word-sized booleans.

---

# 33. Vehicle Controller state

Current flattened state set discussed:

1. INIT
2. CALIBRATE
3. IDLE
4. LINE_FOLLOW
5. LINE_SEARCH
6. EXECUTE_NAV
7. HUMP_TRAVERSE
8. HUMP_OBST_VALIDATE
9. OBSTACLE_STOP
10. BYPASS
11. LINE_REACQUIRE
12. MISSION_COMPLETE
13. FAULT

Thirteen states require:

**4 bits**

---

# 34. Mode flags

Current logical mode flags:

- RUN
- LINE_EN
- SCAN_REQ
- SLOW

Total:

**4 logical bits**

The RTOS object itself may still be wider.

---

# 35. Small enums

## Pending navigation

If only:

- none
- left
- straight
- right

four possibilities:

**2 bits**

If more actions are added, recalculate.

## Barcode/action

Four basic commands could fit in 2 bits, but with recovery/none/reverse/reserved semantics the current design uses:

**3 bits**

---

# 36. Fault mask

Independent faults should use one bit each.

Examples:

- motor stall;
- encoder fault;
- IMU fault;
- ultrasonic no-echo/sensor fault;
- Wi-Fi loss;
- collision if represented as fault;
- other subsystem faults.

Final mask width depends on the final system-wide fault enumeration.

---

# 37. Controller event messages

Current events discussed include roughly:

### Motion
- MOTION_DONE
- STALL
- self-test/encoder status

### Line/barcode
- BARCODE
- JUNCTION
- LINE_LOST
- LINE_FOUND
- sensor status

### Obstacle
- OBST_NEAR
- OBST_CLEAR
- OBST_STOP
- PROFILE
- SENSOR_FAULT
- echo self-test

### IMU
- HUMP_START
- HUMP_END
- COLLISION
- SENSOR_FAULT
- self-test/calibration status

### Telemetry
- START
- STOP
- ESTOP
- RESET

A flat list of about 20+ unique events would likely need around:

**5 bits**

but the final event enumeration is not fixed.

Alternative:

- subsystem/source ID;
- event ID within subsystem;
- type-specific compact payload.

---

# 38. Message design principle

Do not make every message contain every possible field.

Prefer:

**event/command type + type-specific payload**

Examples:

- `OBST_CLEAR` may need no payload;
- `OBST_NEAR` may need bounded distance;
- `PROFILE` carries compact profile;
- `SENSOR_FAULT` carries compact fault subtype.

Likewise Motion can use:

**3-bit command type + command-specific parameter**

---

# 39. Raw hardware values should stay local

Values that usually do not need to propagate:

- absolute timer timestamps;
- raw servo PWM;
- echo edge timestamps;
- direction strings;
- duplicated `has_*` flags;
- wide intermediate arithmetic.

Prefer publishing:

- bounded distance;
- compact status;
- scan index;
- compact width/clearance;
- state;
- fault bits.

---

# 40. Peripheral register width is not application width

The PWM peripheral can require a larger native register while application data remains a 3-bit scan index.

Recommended flow:

1. application/shared data stores scan index;
2. Obstacle Task converts index to calibrated pulse;
3. PWM peripheral receives native hardware value.

Likewise, the ultrasonic driver can use native timer widths internally and publish only bounded distance/status.

---

# 41. No echo is not clear

A missing echo can mean:

- no obstacle;
- obstacle outside beam;
- absorptive material;
- bad angle;
- narrow object;
- sensor/electrical fault.

Therefore:

**timeout/no echo = unknown/error**

not automatically:

**clear path**

Compression must preserve this semantic distinction.

---

# 42. Current Spatial Optimizer representation table

| Field | Proposed range | Resolution/states | Bits | Status |
|---|---:|---:|---:|---|
| Profile distance | 0–400 mm | 1 mm | 9 | Provisional design decision |
| Ultrasonic result status | valid / no-rise / stuck-high | 3 states | 2 | Recommended |
| Echo duration if shared | 0–30,000 µs | 1 µs | 15 | Prefer local |
| No-echo streak | 0–5 | integer | 3 | Current design |
| Scan position | −15°…+15° in 5° steps | 7 positions | 3 | Conditional on final grid |
| Arbitrary mounted PWM at 10 µs | ~1340–1660 µs | 33 values | 6 | Usually unnecessary |
| Obstacle width | 0–214 mm + overflow/wall | 1 mm | 8 | 400 mm forward-range assumption |
| Left clearance | 0…safe threshold | 1 mm | TBD | Measurement required |
| Right clearance | 0…safe threshold | 1 mm | TBD | Measurement required |
| Spatial Optimizer state | 7 current states | enum | 3 | Recalculate if redesigned |
| Rescan count | 0–3 | integer | 2 | Current design |
| OBST_NEAR | false/true | 2 states | 1 | Known |
| OBST_STOP | false/true | 2 states | 1 | Known |
| SCAN_REQ | false/true | 2 states | 1 | Known |
| SLOW | false/true | 2 states | 1 | Known |

---

# 43. Current system-wide compact fields

| Field | Reasoning | Bits/status |
|---|---|---:|
| Vehicle Controller state | 13 states | 4 |
| Mode flags | RUN, LINE_EN, SCAN_REQ, SLOW | 4 logical bits |
| Motion command type | 6 commands | 3 |
| Motion speed | signed counts / 20 ms, current peak ~221 | 9 provisional |
| Motor duty | −100…+100% | 8 |
| Three line ADCs | 12-bit each | 36 total |
| Thresholded line pattern | 3 binary sensors | 3 |
| Barcode raw sample | binary | 1 |
| Barcode/action code | multi-action + reserved/recovery | 3 |
| IMU proposed snapshot | 2 × 32-bit words | 64 bits / 8 B |
| Rescan count | 0–3 | 2 |
| Pending nav | none/L/S/R if unchanged | 2 |
| Faults | 1 bit each | final count TBD |

---

# 44. Buddy 5 Tribunal contribution — final framing

The main Buddy 5 challenge should be described as:

> **Whether Spatial Optimizer data should represent full hardware ranges or only decision-relevant application ranges under the minimum-bit requirement.**

Self-challenge/review showed that:

- distance beyond the useful avoidance horizon adds no decision information;
- clearance beyond the safe traversal threshold is simply "enough";
- width beyond the observable profile can saturate;
- raw echo timing and PWM do not need to leave the subsystem unless another consumer needs them.

The ±15° scan and 400 mm range are supporting examples.

---

# 45. Buddy 5 Resource and Assumption Review

## Item challenged

Whether Spatial Optimizer data should represent full hardware ranges or only decision-relevant ranges.

## Evidence

- new minimum-bit requirement;
- current 0–400 mm useful profile range;
- mounted servo ≈ ±15°;
- at 400 mm forward distance, ±15° gives ≈214 mm fully observable width;
- clearance beyond the safe traversal envelope does not change the passability decision.

## Decision

Use bounded/saturated application fields:

- distance → useful avoidance range;
- width → observable/useful maximum + wall/overflow semantic;
- clearance → safe traversal threshold;
- raw timing/PWM remains local where possible.

## Impact

Buddy 1:
- smaller bounded binary telemetry fields.

Buddy 2:
- compact profile/passability information rather than oversized raw sensor values.

Buddy 5:
- must document valid range, resolution, saturation rule, bit width, and validation evidence.

---

# 46. Buddy 5 Curveball Response — concrete version

## Immediate impact

Using the current Spatial Optimizer design:

- profile distance 0–400 mm at 1 mm → **9 bits**;
- ±15° scan in 5° steps → **7 positions → 3 bits**;
- max fully observable width at 400 mm forward range → **~214 mm**;
- width + saturation state → **8 bits**;
- clearance threshold still unknown because vehicle swept width + safety margin has not been measured.

## Adaptation

- store profile distance in 9 bits;
- store servo scan as 3-bit index, not PWM;
- store width in 8 bits with explicit `extends beyond FOV / wall` semantics;
- measure robot maximum swept lateral width during bypass;
- add agreed safety margin;
- use that as left/right clearance saturation threshold;
- at 1 mm, clearance bits = `ceil(log2(C_limit_mm + 1))`;
- keep raw echo timing and PWM local unless another task needs them.

---

# 47. Measurements still required

## Buddy 2 / Motion

- wheel diameter;
- actual cruise speed;
- maximum encoder delta per 20 ms;
- stopping distance on representative surface/load;
- final TURN/FWD/REV parameter ranges.

## Buddy 3 / Line + barcode

- calibrate all three analogue sensors;
- final line thresholds;
- barcode reliability at driving speed;
- Reverse barcode definition if retained.

## Buddy 4 / IMU

- exact scaling/ranges for packed fields;
- internal vs published heading representation;
- turn-rate range;
- height/peak scaling;
- whether final snapshot is 8 B or a genuine 6 B redesign.

## Buddy 5 / Spatial Optimizer

- mounted HC-SR04+ reliable min/max range;
- representative obstacle material/angle behaviour;
- final mounted servo safe angle/pulse endpoints;
- final scan grid;
- robot maximum swept lateral width during bypass;
- final safety margin;
- final left/right clearance threshold;
- stopping threshold after cruise speed/stopping distance are measured.

---

# 48. Hardware specs are provisional, measurements are final

Generic datasheet/common module values can help establish provisional bounds.

But mounted behaviour depends on:

- exact module;
- supply voltage;
- mounting;
- target material;
- target angle;
- robot motion.

Therefore:

**specification → provisional bound**

**physical measurement → final application-valid bound**

For Buddy 5, the shared profile is intentionally capped at 400 mm anyway, but mounted reliability still needs testing.

---

# 49. Saturation is valid only when decision meaning is unchanged

Saturation is valid when every larger value has the same meaning.

Examples:

## Clearance
Any `>= safe threshold` = passable.

## Width
Anything beyond observable FOV = exact width unknown / effectively wall.

## Distance
If controller ignores everything beyond 400 mm, larger values need not be distinguished in that application field.

Do not saturate if later logic needs the larger values.

---

# 50. Resolution is a separate decision from range

Example:

0–400 mm at 1 mm → 9 bits.

If 10 mm resolution were proven sufficient:

0,10,20,...,400 gives 41 values → **6 bits**.

But resolution must be justified from:

- thresholds;
- accuracy requirements;
- sensor noise;
- downstream calculations;
- test criteria.

Do not reduce precision just to save bits without proving functionality is preserved.

---

# 51. Week 13 representation table — recommended columns

Use:

| Field | Producer | Consumer | Valid range | Resolution | Saturation/reserved meaning | Required bits | Implementation | Synchronisation | Evidence |
|---|---|---|---|---|---|---:|---|---|---|

This is more meaningful than only listing C types.

---

# 52. Week 13 RAM comparison

Measure actual compiled RAM before/after.

Possible evidence:

- linker map;
- `size` output;
- `sizeof()` logs for key structures;
- static/global RAM totals.

Do not assume logical bit totals equal C structure size. Bit-fields can still occupy larger storage units depending on compiler/layout.

---

# 53. Week 13 message-size comparison

Compare before/after for:

- `mbf_ctrl_evt`;
- `mbf_motion_cmd`;
- shared snapshot;
- obstacle profile;
- MQTT payload.

Useful examples:

- JSON vs packed binary;
- float/int32 IMU snapshot vs packed snapshot;
- string direction vs signed duty;
- generic obstacle profile vs bounded packed profile.

---

# 54. Week 13 correctness/performance evidence

## Motion
- signed speed correct;
- forward/reverse correct;
- wrapping counter correct;
- PID timing unchanged.

## Line/barcode
- line tracking still correct;
- 12-bit sensor information preserved where needed;
- barcode action codes correct;
- 1 ms task timing still met.

## IMU
- internal calculations retain enough precision;
- published quantisation does not break control;
- shared access race-safe.

## Spatial Optimizer
- distance thresholds unchanged;
- no-echo still means unknown/error, not clear;
- scan index maps to correct physical angle;
- width saturation behaves as intended;
- clearance threshold matches measured vehicle envelope;
- task timing remains acceptable.

## Telemetry
- binary payload encodes/decodes correctly;
- message size decreases;
- 110 ms behaviour remains acceptable.

---

# 55. Concurrency tests worth performing

Test:

- simultaneous snapshot updates;
- ISR activity during task reads;
- fault bit set/clear collisions;
- telemetry copy while producers update;
- controller mode flags changing while tasks sample them.

Prefer clear ownership:

- one writer per packed word/field;
- mutex for shared snapshot;
- RTOS signalling for events;
- immutable queue/message payload after send.

---

# 56. Relevant current architecture

Task timing/priority discussed:

- Motion: priority 10, 20 ms;
- Line & Barcode: 11, 1 ms;
- Obstacle: 12, 100 ms + SCAN_REQ;
- Vehicle Controller: 13, event-driven;
- IMU: 20, ~13 ms behaviour;
- Telemetry: 30, 110 ms.

Relevant interfaces:

- `mbf_motion_cmd`
- `mbf_ctrl_evt`
- `flg_mode`
- `sem_estop`
- `sem_echo`
- encoder counts
- `robot_snapshot_t`
- `steer_corr`

The curveball affects both data representation and safe sharing through these interfaces.

---

# 57. Ownership rules that help packing

Current architecture ownership:

- Motion Task = sole motor PWM writer;
- IMU Task = sole I2C owner;
- Line Task = sole `steer_corr` writer;
- Vehicle Controller = mission-state owner;
- Vehicle Controller = owner of RUN/LINE_EN/SCAN_REQ/SLOW changes;
- Obstacle Task = obstacle scan/profile owner;
- Echo ISR = timing/signalling only;
- snapshot = mutex-protected.

Single-writer ownership reduces packed-data race risk.

---

# 58. Design text that needs updating

## Telemetry

Any final design prose still saying ~200-byte JSON telemetry is obsolete.

Final MQTT must be compact binary.

## Servo scan

Any final design claiming a mounted −90°…+90° servo scan is inconsistent with the observed ≈±15° installation unless hardware is changed.

## Generic data types

Review final structures for:

- 32-bit tiny enums;
- strings for states/directions;
- separate byte-sized booleans;
- duplicated `has_*` flags.

---

# 59. Repository evidence discussed

## Servo calibration
- 500 µs minimum
- 1500 µs centre
- 2500 µs maximum
- 20 ms period
- 10 µs calibration adjustment step

## Ultrasonic diagnostics
- 30 ms timeout
- explicit no-rise/stuck-high errors
- pulse-duration-based mm conversion
- HC-SR04+ used at 3.3 V
- mounted range not fully calibrated

## Telemetry
Current test/baseline code contains wider/string fields and is not suitable as the final packed MQTT format.

## Final architecture implementation
Several final event/message/profile structures are still design-level decisions, so Week 13 documentation must be checked against the actual final code.

---

# 60. Open questions before implementation freeze

1. Is 400 mm definitely the final Spatial Optimizer profile cap?
2. Is 1 mm distance resolution required?
3. Is the final servo grid exactly seven 5° positions?
4. Will the servo be remounted for larger FOV?
5. Will body rotation supplement the narrow scan?
6. What is the measured maximum swept bypass width?
7. What safety margin will be used?
8. What is the final clearance bit width?
9. What is the exact `mbf_ctrl_evt` event list?
10. What are the exact motion-command parameter ranges?
11. What is the final fault-bit allocation?
12. Is the IMU snapshot 8 B, or is a true 6 B format intended?
13. Which native-width values are justified implementation temporaries/exceptions?
14. How will MQTT byte order and bit order be specified?

---

# 61. Recommended implementation philosophy

## Sensors/drivers
Use native widths as needed for correct hardware interaction and arithmetic.

## Subsystem output
Convert raw values into bounded application information.

## Shared data
Pack fields using actual valid ranges.

## Inter-task messages
Send only information the consumer needs.

## Controller
Use compact state/event/flag encodings.

## MQTT
Use explicit binary packet layout and masks/shifts.

## Documentation
Record scale, saturation, invalid codes, and bit order.

## Synchronisation
Use ownership, mutexes, atomics, or RTOS primitives to prevent packed-field races.

---

# 62. Final short summary

The curveball does **not** fundamentally change the robot architecture.

It mainly forces a redesign of **data representation and interfaces**.

Main consequences:

- oversized application fields must be bounded;
- textual MQTT becomes binary;
- faults become bit flags;
- states/enums get explicit minimum widths;
- raw hardware values stay local unless needed elsewhere;
- saturation is used when larger values do not change decisions;
- internal precision is retained where computations require it;
- packed shared data must remain race-safe;
- Week 13 must prove RAM/message savings and no functional/timing regression.

For Buddy 5 specifically:

- **distance: 0–400 mm → 9 bits at 1 mm**
- **scan position: 7 positions → 3 bits**, if the current 5° grid is retained
- **obstacle width: ~214 mm max useful observable width → 8 bits including saturation/wall semantics**
- **clearance: final bit width requires measuring swept traversal envelope + safety margin**
- **raw echo timing/PWM stays local unless another subsystem genuinely needs it**

---

# 63. Source material

- `Change Requirement 1.docx`
- Week 8 Tribunal / Changes tab
- current Design Review architecture
- repository calibration/diagnostic files discussed during the Tribunal
- servo and ultrasonic calibration/diagnostic results
- calculations and decisions made during the Change Requirement 1 discussion

---

# 64. Status labels for future use

- **LOCKED** — measured/agreed and ready for final implementation.
- **PROVISIONAL** — reasonable current choice but still needs validation.
- **MEASUREMENT REQUIRED** — do not invent the value.
- **TEAM DECISION REQUIRED** — depends on interface/state-machine agreement.
- **LECTURER CLARIFICATION** — requirement interpretation remains ambiguous.
