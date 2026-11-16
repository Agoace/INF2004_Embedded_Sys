# Measurements

One table of the measured or assumed physical and timing values in this project.
Network values (addresses, secrets) are not included.

Rules:

- Check this file before you ask the operator for a number.
- When you record a new value in a document, add a row here in the same commit.
- If two values do not agree, keep both rows with their dates. Do not delete the older row.
- `mtk3-hil:path` sources are on branch `mtk3-hil` only. Read them with
  `git show origin/mtk3-hil:path`. Do not merge that branch for this.
- `569672a:path` sources were removed from the tree when documents were
  merged. Read them with `git show 569672a:path`. The area `HISTORY.md`
  under `docs/archive/` lists them.

## Labels

| Label | Meaning |
| --- | --- |
| `MEASURED(who,date)` | A person or the robot measured it. `who` is operator, bench, robot log, trace or build. The date is in the label when the source gives it. |
| `ASSUMED` | A design setting, guess, datasheet value or simulator input. Not measured. |
| `PHOTO-ESTIMATED` | Read from a photo with a ruler. |
| `DERIVED` | Calculated or fitted from other values. |

Date column: the run date when the source gives one. `doc ≤ YYYY-MM-DD` means
the source does not give a date; the value was in the document on that date
(last change of that line).

## Values that do not agree

These rows are not resolved, except where the table says so. The tables
below give the sources.

| Quantity | Value A (date) | Value B (date) | Other |
| --- | --- | --- | --- |
| Sonar face to robot front | 58 mm, echo mean 57.9 mm (2026-10-27) | 47 mm if the echo reads 11 mm high (doc ≤ 2026-11-08) | Not resolved. Both values use the same reference: transducer face to a flat target touching the robot front, read by echo (Run 7, 80/80 VALID); 47 mm only subtracts an assumed 11 mm short-range over-read. Ruler-derived 42.5 mm = 127.5 − 85 (2026-11-16) uses the module front face and the robot front from the ruler session |
| Line sensors ahead of the wheel axle | 123 mm = 65 + 58, calculated (2026-10-30) | 119 mm, operator (2026-11-15) | Sim guess 119 mm (doc ≤ 2026-11-08). **Ruler about 122 mm** to the GP26/27 centre (2026-11-16); both older values are within 3 mm of it |
| Counts per wheel revolution | 2540 / 2539 (2026-10-27) | 2535 (doc ≤ 2026-11-08) | Old motor 2 test 2532.9 / 2532.3 |
| Robot mass | 1 kg SIL placeholder (doc ≤ 2026-11-16) | 671.6 g, operator (2026-11-16) | **Resolved: 671.6 g** |
| Robot length | 190 mm overall, operator (2026-10-30, MOTION.md) | 180 mm, operator ruler (2026-11-16) | **Current: 180 mm** (ruler). The 190 mm value may include a bumper or sensor overhang; its reference is unknown |
| Wheel diameter | 65 mm, "Operator" (2026-10-30, archive) | 65 mm nominal, not calibrated (doc ≤ 2026-11-08) | **Operator confirmed 65 mm** (2026-11-16); loaded rolling diameter still not calibrated |
| Wheel track | 111 mm, operator (2026-10-30) | 120 mm in old `motion_config.h` (doc ≤ 2026-11-06) | IMU notes "rough 115 mm". **Resolved: 111-112 mm** contact centre to contact centre, operator ruler (2026-11-16); the old 120 mm code value is superseded |
| Wheel axle to caster | 80 mm, operator | 79 mm, offline fit | **Resolved: 82 mm**, operator ruler (2026-11-16), confirmed after the first "?"; 80 and 79 mm are older |
| Line sensor height | about 9 mm (2026-11-03) | about 8 mm (2026-11-15) | **Photo supports about 9 mm** (LED tips, ±1 mm, 2026-11-16); the 8 mm value is an older SIL input |
| Line sensor spacing | 14.5 mm on the board (2026-11-03) | 20.9 mm optical fit (2026-11-03) | Sim guess 22 mm (doc ≤ 2026-11-08). **Photo supports 14.5 mm** (14.7 / 14.2 mm, 2026-11-16); 20.9 mm is an optical fit, not a position |
| GP28 height | 5.5 mm lowest possible (doc ≤ 2026-11-09) | 5–6 mm (2026-11-15) | Model uses 5.5 mm. **Photo about 5.2 mm** (LED tips, ±1 mm, 2026-11-16), consistent with 5.5 mm |
| GP28 forward from rear axle | about 80 mm +-5, photo estimate (2026-11-11) | 73 mm to the diode, operator ruler (2026-11-16) | **Current: 73 mm** (ruler) |
| GP28 lateral (left of robot centre line) | 77 mm, operator ruler (2026-11-11) | about 73 mm to the diode centre, operator ruler (2026-11-16) | About 76.5 mm DERIVED (module edge minus half housing). **Current: 73 mm** (ruler, diode centre) |
| Floor speed at 100 % duty | 740 mm/s, recollection (2026-10-30) | 770..807 mm/s, still rising (doc ≤ 2026-11-10) | — |
| Narrowest bar | 5 mm large, 2–3 mm small, planning (2026-11-06) | 5.8 mm (2x F), 2.9 mm (course F) (doc ≤ 2026-11-09) | — |

## Operator ruler session 2026-11-16

The operator measured these with a ruler on 2026-11-16 (reported through
Main_Worker). Values as given:

- Outer wheel to outer wheel: 128 mm.
- Robot length: 180 mm.
- Robot height: 134 mm.
- IMU height: 132 mm.
- IR barcode sensor (GP28 module) protrudes 17-18 mm beyond the left outer wheel face.
- Rear wheel axle to robot rear: 52-53 mm.
- Wheel track, contact centre to contact centre: 111-112 mm.
- Wheel width: 16 mm.
- Rear axle to caster: 82 mm (first marked "?"; confirmed by the operator later on 2026-11-16).
- Rear axle to back of ultrasonic module: about 70 mm +-3 mm.

Derived from these values:

- Robot front ahead of rear axle: 180 - 52.5 = 127.5 mm.
- Wheel centre half-track: 111.5 / 2 = 55.75 mm.
- Outer wheel face from centre: 128 / 2 = 64 mm.

Photo estimates: 3 operator photos with a steel ruler beside the sensors
(same depth, ruler end = floor), 2026-11-16:

- Line sensors GP26/27: LED tips about 9 mm above floor (housing bottom about 10.5 mm), +-1 mm.
- GP28: LED tips about 5.2 mm above floor (housing bottom about 6.8 mm), +-1 mm.
- GP26/27 module centre spacing about 14.7 mm; emitter/receiver midpoint spacing about 14.2 mm (supports the 14.5 mm board value).
- TCRT housing width about 10.6 mm (scale check against the 10.2 mm datasheet value).

Operator answers, 2026-11-16:

- Rear axle to line sensor (GP26/27) centre: about 122 mm.
- GP28 diode centre about 73 mm left of the robot centre line.
- Rear axle to GP28 diode: 73 mm forward.
- Wheel diameter: 65 mm, confirmed.
- Rear axle to caster: 82 mm, confirmed.
- Ultrasonic module depth: 15 mm.

Derived from the answers:

- Rear axle to sonar front face: 70 (+-3) + 15 = 85 mm +-3 mm.
- Sonar front face behind robot front: 127.5 - 85 = 42.5 mm.

Check (DERIVED, no new value): the operator confirms the GP28 diode sits
at the module outer edge. Derived lateral = 64 + 17.5 - about 5 (half the
TCRT housing) = about 76.5 mm; this assumes the LED pair centre is about
5 mm inside the module edge. The direct 73 mm implies about 8.5 mm.

## Geometry

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| Caster radius (SIL) | 10 mm | doc ≤ 2026-11-15 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Course humps | 12.7 mm and 13.4 mm | 2026-11-11 | MEASURED(operator,2026-11-11) | [SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11) |
| Course tile | 315 x 315 mm, 10 mm thick; curve radius 157.5 mm (assumed) | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#published-track-specification](../archive/simulation/SIMULATION.md#published-track-specification) |
| Curve stroke radii | inner / outer 148.5 / 166.5 mm (centre radius 157.5) | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/S3_SENSOR_MODEL.md#shared-geometry` |
| Front-corner circle (rear axle to farthest corner) | 146.7 mm calculated; 150 mm in code | 2026-10-30 | DERIVED | [OBSTACLE_DETOUR_HANDOFF.md#values-to-measure](../archive/OBSTACLE_DETOUR_HANDOFF.md#values-to-measure) |
| Front-corner radius | 146.7 mm = sqrt(123^2 + 80^2) (138 mm if offset 47 mm) | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Line mount distance from axle (supersedes 123 mm) | 119 mm | 2026-11-15 | MEASURED(operator,2026-11-15) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Line sensor position (old statement) | at the very front of the robot | 2026-10-30 | MEASURED(operator,2026-10-30) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#robot-geometry` |
| Line sensors ahead of rear axle | 123 mm (calculated) | 2026-10-30 | DERIVED | [OBSTACLE_DETOUR_HANDOFF.md#values-to-measure](../archive/OBSTACLE_DETOUR_HANDOFF.md#values-to-measure) |
| Maximum width (incl. left barcode mount) | 160 mm | doc ≤ 2026-11-08 | MEASURED(operator) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Outer wheel face from centre | 128 / 2 = 64 mm | 2026-11-16 | DERIVED | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Outer wheel to outer wheel | 128 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Overall length | 190 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Rear axle to caster | 82 mm (confirmed) | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Rear axle to front line sensors | 123 mm = 65 + 58 | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Rear wheel axle to robot rear | 52-53 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Robot corner to the side of sonar | about 80 mm | 2026-10-30 | DERIVED | [OBSTACLE_BYPASS.md#runs-1214-2026-10-30-version-4-spiral-in-corner-hit](../archive/OBSTACLE_BYPASS.md#runs-1214-2026-10-30-version-4-spiral-in-corner-hit) |
| Robot front ahead of rear axle | 180 - 52.5 = 127.5 mm | 2026-11-16 | DERIVED | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Robot height | 134 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Robot length | 180 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Robot length (overall) | 190 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | [OBSTACLE_BYPASS.md#steering](../archive/OBSTACLE_BYPASS.md#steering) |
| Robot width (widest point) | 160 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | [OBSTACLE_BYPASS.md#steering](../archive/OBSTACLE_BYPASS.md#steering) |
| Servo axis to rear axle | 65 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Servo axis to rear axle | 65 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | `569672a:docs/architecture/week8_change_requirement_1_notes.md#27-leftright-clearance` |
| SIL example box / hump | box 100x180x100 mm; hump 80x30x10 mm | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| SIL other PROTO geometry | wheel width 16 mm; caster width 6 / trail 8 mm; head 30x35x20 mm; servo torque 0.1 Nm | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Sim front / IR board tip x | 123 mm = 65 + 58 (calculated) | doc ≤ 2026-11-08 | DERIVED | [SIMULATION.md#geometry](../archive/simulation/SIMULATION.md#geometry) |
| Sim front with 47 mm true sonar offset | front x 112 mm; corner radius about 138 mm | doc ≤ 2026-11-08 | DERIVED | [SIMULATION.md#geometry](../archive/simulation/SIMULATION.md#geometry) |
| Sim IR sensing-point x | 119 mm = 123 - 4 (guess) | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Sim rear outline x | -67 mm = 123 - 190 | doc ≤ 2026-11-08 | DERIVED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Sonar face to front | about 58 mm (echo mean 57.9 mm, not ruler) | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Sonar face to front, alternative | about 47 mm (echo reads about 11 mm high) | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Track (wheel contact to wheel contact) | 111 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | `569672a:docs/architecture/week8_change_requirement_1_notes.md#27-leftright-clearance` |
| Wheel axle to caster (wheelbase) | 80 mm | doc ≤ 2026-11-15 | MEASURED(operator) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Wheel centre half-track | 111.5 / 2 = 55.75 mm | 2026-11-16 | DERIVED | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Wheel diameter | 65 mm nominal; loaded rolling diameter not calibrated | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Wheel diameter (archive label) | 65 mm, "Operator, 2026-10-30" | 2026-10-30 | MEASURED(operator,2026-10-30) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#robot-geometry` |
| Wheel diameter (operator confirmed) | 65 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Wheel track | 111 mm | 2026-10-30 | MEASURED(operator,2026-10-30) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Wheel track, contact centre to contact centre | 111-112 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Wheel width | 16 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Wheelbase in Motion's motion_config.h (old) | 120 mm (MEASURE) vs 111 mm here | doc ≤ 2026-11-06 | ASSUMED | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#speed-loop` |
| Wheelbase, offline fit | 79 mm | doc ≤ 2026-11-15 | DERIVED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |

## Mass

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| Body height / COM (SIL) | 35 mm box, centre Z 25 mm; COM x 30 / Z 20 mm | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Derived body mass (after assumed parts) | 551.6 g | 2026-11-16 | DERIVED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| SIL placeholder mass (completed runs) | 1 kg | doc ≤ 2026-11-16 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Total robot mass | 671.6 g | 2026-11-16 | MEASURED(operator,2026-11-16) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Wheel / caster / head mass | 40 g each / 20 g / 20 g | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |

## Motors and duty map (DM1)

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| B1 hook / heartbeat ISR max | 88 / 102 us | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b1-2026-11-04-boot-5bf1-ad2404c-floor-vin` |
| B1 slow speed hold | within 2 mm/s at 20/40/150 mm/s; 20 mm/s at 75-115 permille | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b1-2026-11-04-boot-5bf1-ad2404c-floor-vin` |
| B1 step fwd/rev 150 overshoot L/R | 18/17 %, 34/24 % | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b1-2026-11-04-boot-5bf1-ad2404c-floor-vin` |
| B1 step fwd/rev 300 overshoot L/R | 11/12 %, 26/17 %; rev 300 ripple sd 18.1/12.2 mm/s | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b1-2026-11-04-boot-5bf1-ad2404c-floor-vin` |
| B1 wheel lag behind set point | about 100 ms; duty peak 320 vs 230 permille steady | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b1-2026-11-04-boot-5bf1-ad2404c-floor-vin` |
| B2 feed-forward refit | offset 52-60 permille, slope 8.7-9.3 | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b2-2026-11-04-boot-8460-1cf08a1-floor-vin` |
| B2 overshoot 150/300 | 7/8, 7/8, 5/6, 4/4 % (fwd150, rev150, fwd300, rev300) | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b2-2026-11-04-boot-8460-1cf08a1-floor-vin` |
| B2 tuning 150/300 mm/s | overshoot 4..8 %; straight heading within 0.7 deg | 2026-11-04 | MEASURED(bench) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| B2/B3 hook / heartbeat ISR max | 85 / 99 us | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b3-2026-11-04-boot-ede2-rise-limit-8-countstick-per-tick-floor-vin` |
| B3 (2026-11-04) step fwd / rev 40 mm/s | 8/24 % and 12/24 % overshoot; t90 210/70 and 190/60 ms | 2026-11-04 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| B3 (2026-11-04) step fwd 150 mm/s: overshoot L/R, t90 L/R, ripple, heading | 6/11 %, 140/130 ms, 2.5/2.4 mm/s, +0.5 deg | 2026-11-04 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| B3 (2026-11-04) step fwd 300 mm/s | 10/9 %, 160/150 ms, 5.0/4.5 mm/s, +0.2 deg | 2026-11-04 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| B3 (2026-11-04) step rev 150 mm/s | 9/12 %, 180/170 ms, 2.6/3.1 mm/s, -1.3 deg | 2026-11-04 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| B3 (2026-11-04) step rev 300 mm/s | 8/9 %, 200/180 ms, 7.5/7.7 mm/s, -2.2 deg | 2026-11-04 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| B3 feed-forward refit | offset 53-68 permille, slope 8.4-9.2 | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#run-b3-2026-11-04-boot-ede2-rise-limit-8-countstick-per-tick-floor-vin` |
| B3 reference spin L/R 20 (3 s) | heading +5900 / -5887 cdeg | 2026-11-04 | MEASURED(operator,2026-11-04) | [TEST_LOG.md#b4-reference-b3-measured-values](../../testing/subsystem2/TEST_LOG.md#b4-reference-b3-measured-values) |
| B3 spin left / right 20 | 44/36 % and 100/116 % overshoot; +59.0 / -58.9 deg | doc ≤ 2026-11-16 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| B4 floor speeds vs B3 | within 1-2 mm/s (0.3-10.5 %); heading diffs 25..220 cdeg | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#b4-floor-cold-vin-power-on](../../testing/subsystem2/TEST_LOG.md#b4-floor-cold-vin-power-on) |
| B4-lifted (USB) spin heading | about 69 deg lifted vs about 59 deg floor | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#b4-lifted-usb-after-flash](../../testing/subsystem2/TEST_LOG.md#b4-lifted-usb-after-flash) |
| Bypass arc radius | planned 179 mm at 20 deg; driven 467 mm (2.6 x) | 2026-10-30 | MEASURED(robot log,2026-10-30) | [OBSTACLE_BYPASS.md#run-3-2026-10-30-version-2-around-the-bottle-wide](../archive/OBSTACLE_BYPASS.md#run-3-2026-10-30-version-2-around-the-bottle-wide) |
| DM1 12 / 20 / 25 % | mean 62 / 131 / 171 mm/s | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| DM1 30 / 40 / 50 / 60 / 70 % | mean 214 / 300 / 378 / 463 / 552 mm/s | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| DM1 80 / 90 / 100 % (wheelies, do not use) | mean 632 / 706 / 770 mm/s | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| DM1 heartbeat interrupt max | 56 us | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| DM1 loaded duty map (fwd L/R, rev L/R) 8 % | +24/+24, -29/-26 mm/s | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| DM1 running minimum | 8 % after start kick; 5-6 % stops, 7 % almost | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| Duty / speed cap | 80 % / +-460 mm/s | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Duty-mode speed table (floor) | 20 % -> 74, 25 % -> 101, 33 % -> 141 counts per 50 ms; wheel stops at about 5.6 % | 2026-10-30 | MEASURED(robot log,2026-10-30) | [OBSTACLE_BYPASS.md#run-3-2026-10-30-version-2-around-the-bottle-wide](../archive/OBSTACLE_BYPASS.md#run-3-2026-10-30-version-2-around-the-bottle-wide) |
| Feed-forward L fwd/rev | offset 48/59 permille; slope 9.66/8.89 permille per count/tick | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Floor 100 % duty speed | about 770..807 mm/s, still rising, wheelies | doc ≤ 2026-11-10 | MEASURED(operator) | [CR1.md#a2-planned-shared-16-bit-modular-encoder-values](CR1.md#a2-planned-shared-16-bit-modular-encoder-values) |
| Host model wheel dead zone | about 9 % | doc ≤ 2026-11-06 | ASSUMED | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#host-unit-test` |
| Loaded duty to speed, 8..70 % | about 8.5 mm/s per duty %; wheels within about 3 %; 5..6 % stop, 7 % nearly | doc ≤ 2026-11-16 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Loaded steady speed at 14 % | about 40-50 mm/s; 20 deg/s sweep needs about 19 mm/s per wheel | 2026-11-04 | MEASURED(operator,2026-11-04) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#ideas-not-implemented` |
| Motor PWM | 20 kHz (wrap 6249 at 125 MHz) | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#hardware-and-time-boundary](../s2-motion/MOTION.md#hardware-and-time-boundary) |
| Pi 400 USB peripheral limit | 1.2 A shared | doc ≤ 2026-09-20 | ASSUMED | [USB_POWER_SAFETY.md#limit](../archive/s2-motion/USB_POWER_SAFETY.md#limit) |
| Pico current, idle / radio 1 Hz | about 30-50 mA / 70-90 mA | doc ≤ 2026-09-21 | ASSUMED | [USB_POWER_SAFETY.md#conditional-behaviour-of-the-unmarked-power-bank](../archive/s2-motion/USB_POWER_SAFETY.md#conditional-behaviour-of-the-unmarked-power-bank) |
| PWM frequency | 20 kHz | doc ≤ 2026-11-04 | ASSUMED | [ARCHITECTURE.md#work-rules](ARCHITECTURE.md#work-rules) |
| Retained-origin count limit | 32768 counts, about 2.64 m | doc ≤ 2026-11-10 | DERIVED | [CR1.md#a2-planned-shared-16-bit-modular-encoder-values](CR1.md#a2-planned-shared-16-bit-modular-encoder-values) |
| Reverse weaker (left wheel); detour sweep minimum | 14 % duty; second-detour stalls at 14-15 % | doc ≤ 2026-11-08 | MEASURED(operator) | [ARCHITECTURE.md#context](ARCHITECTURE.md#context) |
| Rise limit | 8 counts/tick per tick (about 6400 mm/s^2) | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Rise limit history | 2 counts/tick per tick (1600 mm/s^2) until 2026-11-04, then 8 | 2026-11-04 | ASSUMED | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#speed-loop` |
| Robo Pico input range; motor rail on USB | 3.6-6 V; VUSB - 0.4 V | doc ≤ 2026-09-21 | ASSUMED | [USB_POWER_SAFETY.md#conditional-behaviour-of-the-unmarked-power-bank](../archive/s2-motion/USB_POWER_SAFETY.md#conditional-behaviour-of-the-unmarked-power-bank) |
| S2-U04 ESTOP during fwd 300 | 330 mm/s, duty 408/403; stopped at once | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#s2-u04-run-2026-11-06](../../testing/subsystem2/TEST_LOG.md#s2-u04-run-2026-11-06) |
| SIL contact / inertia / torque | friction 0.8; wheel inertia 2.1125e-5 kg m^2; torque cap 0.12 Nm | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| SIL high-duty model | linear extension of 60-70 % slope to 80 % (unvalidated) | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| SIL motor response | drive/coast 50 ms; static threshold 12 % | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Sim loaded speed at 100 % (operator recollection) | 740 mm/s | 2026-10-30 | ASSUMED | [OBSTACLE_DETOUR_HANDOFF.md#evidence](../archive/OBSTACLE_DETOUR_HANDOFF.md#evidence) |
| Sim wheel time constant | loaded tau 50 ms (assumed); unloaded fit 24.74 / 20.47 ms | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#implemented-motion-equations-and-coast-fit](../archive/simulation/SIMULATION.md#implemented-motion-equations-and-coast-fit) |
| Speed cap basis | 460 mm/s = DM1 60 % point | 2026-11-04 | DERIVED | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#speed-loop` |
| Speed loop P / I | 6.0 / 0.5 per tick; I limit +-400 permille | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Speed quantum | 1 count per 10 ms tick = 8.055 mm/s | doc ≤ 2026-11-06 | DERIVED | `569672a:docs/archive/s2-motion/DUTY_MAP.md#run-dm1-2026-11-04-boot-5d51-7a65c0e-floor-vin` |
| Speed safety bound / command cap / duty cap | 1000 mm/s (12414 counts/s) / 460 mm/s / 80 % | doc ≤ 2026-11-10 | ASSUMED | [CR1.md#a2-planned-shared-16-bit-modular-encoder-values](CR1.md#a2-planned-shared-16-bit-modular-encoder-values) |
| Stale command stop | 100 ms | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Stall detect | duty >= 15 %, no count for 500 ms | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Start kick | at least 12 % until 10 counts; no move by 250 ms = stall | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Turn overshoot at 45 deg/s, 20 ms period | about 1 deg | doc ≤ 2026-11-04 | DERIVED | [ARCHITECTURE.md#consequences](ARCHITECTURE.md#consequences) |
| Unloaded coast at 10 % (USB, lifted) | 27-28 counts left, 23 right | doc ≤ 2026-11-08 | MEASURED(operator) | [SIMULATION.md#motors-counts-and-coast](../archive/simulation/SIMULATION.md#motors-counts-and-coast) |
| Unloaded speed at 5 V VIN, 10..100 % duty L | 26.3, 53.9, 81.5, 109.0, 136.6, 163.4, 190.8, 217.0, 242.7, 260.8 RPM (old ratio 2540) | 2026-10-27 | MEASURED(bench) | [MOTION.md#unloaded-motor-speed-at-5-v-vin](../s2-motion/MOTION.md#unloaded-motor-speed-at-5-v-vin) |
| Unloaded speed at 5 V VIN, 10..100 % duty R | 26.6, 54.4, 81.9, 109.3, 136.5, 163.1, 189.0, 214.3, 239.6, 257.9 RPM (old ratio 2539) | 2026-10-27 | MEASURED(bench) | [MOTION.md#unloaded-motor-speed-at-5-v-vin](../s2-motion/MOTION.md#unloaded-motor-speed-at-5-v-vin) |
| Unloaded speed USB vs VIN | USB 8.5..10.5 % lower over 10..60 %; USB over-current at 70 % | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#unloaded-motor-speed-at-5-v-vin](../s2-motion/MOTION.md#unloaded-motor-speed-at-5-v-vin) |
| Unloaded start duty (VIN) | 6 % left, 7 % right; recommend 8 % min, 10 % kick | doc ≤ 2026-11-08 | MEASURED(operator) | [SIMULATION.md#motors-counts-and-coast](../archive/simulation/SIMULATION.md#motors-counts-and-coast) |
| Unloaded start, 5 V VIN | M1 6 % (about 70 ms to 10 counts); M2 7 % (about 80 ms) | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#motor-duty-and-speed-results](../s2-motion/MOTION.md#motor-duty-and-speed-results) |
| Unloaded start, USB | 0..5 % none; 6 % M1 one count; 7 % both after about 80 ms | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#motor-duty-and-speed-results](../s2-motion/MOTION.md#motor-duty-and-speed-results) |
| USB floor test | 15 % stalled on floor; 20 % completed | doc ≤ 2026-10-28 | MEASURED(bench) | [USB_POWER_SAFETY.md#bounded-spin-calibration-exception](../archive/s2-motion/USB_POWER_SAFETY.md#bounded-spin-calibration-exception) |
| USB over-current event | at 70 % duty settling, both motors | 2026-09-20 | MEASURED(operator,2026-09-20) | [USB_POWER_SAFETY.md#observed-incident](../archive/s2-motion/USB_POWER_SAFETY.md#observed-incident) |
| Webots replay PWM map (from DM1) | PWM 1000/6250 = 16 % -> L/R +96/+95 mm/s; 1250 = 20 % -> +129/+129 mm/s | 2026-11-15 | DERIVED | `mtk3-hil:docs/simulation/WEBOTS_DRIVING_REPLAY_20261115.md#scope-and-one-mapping` |
| Week 8 calibration citation | about 261 RPM at 100 % duty; about 221 counts per 20 ms | 2026-10-27 | MEASURED(operator) | `569672a:docs/architecture/week8_change_requirement_1_notes.md#91-speed` |
| Wheel circumference | 204.204 mm (65 mm nominal diameter) | doc ≤ 2026-11-08 | DERIVED | [SIMULATION.md#motors-counts-and-coast](../archive/simulation/SIMULATION.md#motors-counts-and-coast) |
| Wheel radius (SIL) | 32.5 mm | doc ≤ 2026-11-13 | DERIVED | `mtk3-hil:docs/simulation/S3_SENSOR_MODEL.md#mount-and-motion-limits` |
| Wheelies | at 80..100 % duty | doc ≤ 2026-11-16 | MEASURED(bench) | [MOTION.md#pins-and-interface](../s2-motion/MOTION.md#pins-and-interface) |

## Encoders

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| Brake travel at 10 % from 2500 stop | 40 counts (single); typical 33-39 counts | doc ≤ 2026-09-20 | MEASURED(bench) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#superseded-single-revolution-measurement` |
| Counts per metre | 12414 (2535 and nominal 65 mm; not ground travel) | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Counts per wheel revolution | 2535 (L 2535.000, R 2534.940; 50 turns, unloaded) | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Differential turn scale | 24.05 counts/deg (2405 per deg x100), 111 mm track | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#geometry-and-distance-settings](../s2-motion/MOTION.md#geometry-and-distance-settings) |
| Encoder 10-cycle drift, run 1 | dir A 2535.7 / B 2532.4 counts; signed -33 counts (-4.68 deg encoder, about 1 deg visual) | doc ≤ 2026-09-20 | MEASURED(bench) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#ten-cycle-repeatability-test` |
| Encoder 10-cycle drift, run 2 | dir A 2532.9 / B 2532.3 counts; signed -6 counts (-0.85 deg) | doc ≤ 2026-09-19 | MEASURED(bench) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#ten-cycle-repeatability-test-1` |
| Encoder-calibration stop | 2500 counts, with 2535-35 brake compensation | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| First revolution slower (warm-up) | 60-90 ms; about 2602 -> 2530 ms/rev | doc ≤ 2026-09-21 | MEASURED(bench) | `569672a:docs/archive/s2-motion/MOTOR_ROTATION_DRIFT.md#speed-observations` |
| Jog request 3 / 40 counts gives | 3..7 / 56..57 counts | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| Mean revolution time at 10 %, USB, L/R | 2491 / 2433 ms (24.08 / 24.66 RPM) | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| Old counts per revolution L/R | 2540 / 2539 (superseded; 0.197 % high) | doc ≤ 2026-11-08 | MEASURED(bench, old) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| Old motor 2 10-cycle test (10 %) | 2532.9 / 2532.3 counts per rev (A/B); drift -6 counts (-0.85 deg); superseded by 2535 | doc ≤ 2026-11-08 | MEASURED(bench) | `569672a:docs/archive/kernel/DEVELOPMENT.md#unloaded-motor-speed` |
| Unloaded coast at 10 % duty L/R | 27-28 / 23 counts | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |

## Line sensors and junction

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| Adaptation | 1/256 per sample below 20 % / above 80 %; about 0.25 s at 1 kHz | doc ≤ 2026-10-28 | DERIVED | [LINE_POSITION.md#b-learning-while-running](../archive/s3-line-barcode/LINE_POSITION.md#b-learning-while-running) |
| Both sensors see the line (share of entries) | 12 % (P4), 6 % (P7, P9) | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p9-2026-11-04-boot-d8fa-run-1](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p9-2026-11-04-boot-d8fa-run-1) |
| Calibration minimum contrast | 200 ADC counts (about 0.16 V) | doc ≤ 2026-10-28 | ASSUMED | [LINE_POSITION.md#a-calibration-sweep](../archive/s3-line-barcode/LINE_POSITION.md#a-calibration-sweep) |
| Course line width | 18 mm | 2026-11-03 | MEASURED(operator,2026-11-03) | [LINE_POSITION.md#pair-mode-two-sensors-2026-11-03](../archive/s3-line-barcode/LINE_POSITION.md#pair-mode-two-sensors-2026-11-03) |
| Duty-to-mm/s port | 25 / 12 / 50 % = 171 / 62 / 378 mm/s; about 8.4 mm/s per % (12-50 %) | doc ≤ 2026-11-06 | DERIVED | [LINE_STEER_MM.md#settings-for-the-first-track-run-mapped-one-to-one](../archive/s3-line-barcode/LINE_STEER_MM.md#settings-for-the-first-track-run-mapped-one-to-one) |
| Effective line / spot (first fit, 14.5 mm ruler) | 12.5 mm / 8 mm | 2026-11-03 | DERIVED | [LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209](../archive/s3-line-barcode/LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209) |
| Floor / line ADC (P4-P9 logs) | floor 190/160, line 2600/2450 (L/C) | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p9-2026-11-04-boot-d8fa-run-1](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p9-2026-11-04-boot-d8fa-run-1) |
| Floor variation across mat / ADC noise | about 100 counts / about +-2 counts | doc ≤ 2026-10-28 | MEASURED(robot log) | [LINE_POSITION.md#the-gaps-between-the-sensors](../archive/s3-line-barcode/LINE_POSITION.md#the-gaps-between-the-sensors) |
| Gap-rise threshold | 0.3 % (about 11 ADC counts) | doc ≤ 2026-10-28 | ASSUMED | [LINE_POSITION.md#thresholds](../archive/s3-line-barcode/LINE_POSITION.md#thresholds) |
| GP26/GP27 line sensors: x from axle / clearance | 119 mm / about 8 mm (replaces 9 mm) | 2026-11-15 | MEASURED(operator,2026-11-15) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| GP26/GP27 sensor separation | 14.5 mm (fitted 20.9 mm is not a survey) | 2026-11-03 | MEASURED(operator,2026-11-03) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| IR module board | about 32 x 14 mm, long axis forward | doc ≤ 2026-11-08 | PHOTO-ESTIMATED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Junction action limits | bar exit 70 mm / 1.5 s; C completion 160 mm; spin stall 500 ms; wrong direction > 3 deg; translation > 15 mm | doc ≤ 2026-11-10 | ASSUMED | [JUNCTION_ACTION.md#phases-and-bounds](JUNCTION_ACTION.md#phases-and-bounds) |
| Junction approach travel | 132 mm; bar clearance 33 mm | doc ≤ 2026-11-10 | ASSUMED | [JUNCTION_VALIDATION.md#operator-supplied-evidence--2026-11-10](../kernel/JUNCTION_VALIDATION.md#operator-supplied-evidence--2026-11-10) |
| Junction events (L+C >= 125 %) on track | 11 short events, about 1 per 5-10 s | 2026-11-03 | MEASURED(robot log,2026-11-03) | [LINE_POSITION.md#junction-fix-after-track-run-p3-2026-11-03](../archive/s3-line-barcode/LINE_POSITION.md#junction-fix-after-track-run-p3-2026-11-03) |
| Junction side-arm contact | about 100 mm behind centre; arm contact about 30-40 deg | doc ≤ 2026-11-10 | DERIVED | [JUNCTION_ACTION.md#phases-and-bounds](JUNCTION_ACTION.md#phases-and-bounds) |
| Kernel M1-S7 | three initial losses recovered in 20-110 ms | doc ≤ 2026-11-08 | MEASURED(operator) | [LINE.md#results](../s3-line-barcode/LINE.md#results) |
| Kernel S3-U01/U02 (3f889a1) | 1009 samples; age 993/983 us; task max 84/152 us; ISR max 110/108 us | doc ≤ 2026-11-08 | MEASURED(operator) | [LINE.md#results](../s3-line-barcode/LINE.md#results) |
| Left wheel faster at 20 % duty | about 7 % (right trim +1 %) | 2026-10-27 | MEASURED(robot log,2026-10-27) | [LINE_STEERING.md#trim](../archive/s3-line-barcode/LINE_STEERING.md#trim) |
| Line acquisition | 1 ms TIMER alarm; 8 ADC conversions per channel; 128-slot queue | doc ≤ 2026-11-08 | ASSUMED | [LINE.md#acquisition-and-calibration](../s3-line-barcode/LINE.md#acquisition-and-calibration) |
| Line sensor (GP26/27) LED tip height above floor (photo) | about 9 mm (housing bottom about 10.5 mm), +-1 mm | 2026-11-16 | PHOTO-ESTIMATED(operator photos,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Line sensor (GP26/27) spacing (photo) | module centres about 14.7 mm; emitter/receiver midpoints about 14.2 mm | 2026-11-16 | PHOTO-ESTIMATED(operator photos,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Line sensor fitted Gaussian sigma | 15.92 mm (coverage 0.39 at 7.25 mm offset) | doc ≤ 2026-11-15 | DERIVED | `mtk3-hil:docs/simulation/S3_ASSUMED_LINE_PROFILES.md#original-linear-fit-comparison-a` |
| Line sensor height above floor | about 9 mm | 2026-11-03 | MEASURED(operator,2026-11-03) | [LINE_POSITION.md#pair-mode-two-sensors-2026-11-03](../archive/s3-line-barcode/LINE_POSITION.md#pair-mode-two-sensors-2026-11-03) |
| Line sensor spacing, centre to centre | 14.5 mm | 2026-11-03 | MEASURED(operator,2026-11-03) | [LINE_POSITION.md#pair-mode-two-sensors-2026-11-03](../archive/s3-line-barcode/LINE_POSITION.md#pair-mode-two-sensors-2026-11-03) |
| LINE zigzag on straights (current) | about +-40..54 deg/s, period about 0.4 s | 2026-11-16 | MEASURED(robot log,2026-11-16) | [PROJECT_STATUS.md#areas](PROJECT_STATUS.md#areas) |
| LINE_LOST delay / gap delay / junction hold | 50 ms / 300 ms / 300 ms | doc ≤ 2026-10-28 | ASSUMED | [LINE_POSITION.md#thresholds](../archive/s3-line-barcode/LINE_POSITION.md#thresholds) |
| Lost / found / junction / centred / outer-seen thresholds | 30 % / 50 % / 50 % (L+C >= 125 % in pair mode) / 80 % / 5 % | doc ≤ 2026-10-28 | ASSUMED | [LINE_POSITION.md#thresholds](../archive/s3-line-barcode/LINE_POSITION.md#thresholds) |
| M1 calibration ADC | L floor/line 178/3595; C 150/3577 | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL_ROBOT.md#basic-checks-on-two-tiles-2026-11-06](../../testing/integration/KERNEL_ROBOT.md#basic-checks-on-two-tiles-2026-11-06) |
| Midway reading, line between L and C | 38-40 % each; plain-line L+C max 102 % | 2026-11-03 | MEASURED(robot log,2026-11-03) | [LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209](../archive/s3-line-barcode/LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209) |
| Offset from line centre with target 0 | about 7 mm (sim 6.9-7.1 mm; 10.3 mm with 20.9 mm fit) | 2026-11-03 | DERIVED | [LINE_POSITION.md#rules](../archive/s3-line-barcode/LINE_POSITION.md#rules) |
| Optical sensing-point spacing / spot (fit, 18 mm line) | 20.9 mm / 11.5 mm (about 21 mm) | 2026-11-03 | DERIVED | [LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209](../archive/s3-line-barcode/LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209) |
| P2 swing (gain 200 / D 200) | +50 to -50..-84 every 50-100 ms; wheel stopped 54 % | 2026-11-03 | MEASURED(robot log,2026-11-03) | [LINE_POSITION.md#track-run-p2-2026-11-03-gain-200--d-200-boot-2928](../archive/s3-line-barcode/LINE_POSITION.md#track-run-p2-2026-11-03-gain-200--d-200-boot-2928) |
| P3 (far gain) | 65 s; wheel stopped 9 %; 2.9 turn swaps/s | 2026-11-03 | MEASURED(robot log,2026-11-03) | [TWO_SENSOR_LINE_TRACING.md#evidence](../archive/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md#evidence) |
| P4 (duty lock) | 57 s; wheel stopped 10 %; 2.6 swaps/s; 35 searches of 50-100 ms; mean 146 mm/s | 2026-11-03 | MEASURED(robot log,2026-11-03) | [LINE_STEERING.md#far-gain-two-line-sensors-2026-11-03](../archive/s3-line-barcode/LINE_STEERING.md#far-gain-two-line-sensors-2026-11-03) |
| P4 / P7 main swing | about 3 Hz; position step sd 31 vs 40 per 50 ms | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#d-test-after-p7](../archive/s3-line-barcode/VC_LINE_RUN.md#d-test-after-p7) |
| P4 object stop | object at 73 mm at 54.6 s | 2026-11-03 | MEASURED(robot log,2026-11-03) | [LINE_STEERING.md#far-gain-two-line-sensors-2026-11-03](../archive/s3-line-barcode/LINE_STEERING.md#far-gain-two-line-sensors-2026-11-03) |
| P5 delay budget | rise limit about 70 ms for +112 mm/s; shell step 9 ms | 2026-11-04 | DERIVED | [VC_LINE_RUN.md#run-p5-2026-11-04-boot-61ae](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p5-2026-11-04-boot-61ae) |
| P5 wheel response delay | 100 ms (corr 0.86); mean speed 115 mm/s; asked mean 172 mm/s | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p5-2026-11-04-boot-61ae](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p5-2026-11-04-boot-61ae) |
| P5/P6 heartbeat ISR max | 103 us; controller hook max 86 us | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p5-2026-11-04-boot-61ae](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p5-2026-11-04-boot-61ae) |
| P6 | delay 50 ms (0.85); mean 166 mm/s; 1.86 mm/s per position unit; +-45: 78 % | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p6-2026-11-04-boot-3f9e-run-2](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p6-2026-11-04-boot-3f9e-run-2) |
| P7 | base 146 mm/s, gains 85/85/340; side changes 3.6/s; mean 142 mm/s; 1.40 mm/s per unit; 33 searches/min | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p7-2026-11-04-boot-95bc-run-1](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p7-2026-11-04-boot-95bc-run-1) |
| P9 (I off) | side changes 3.2/s; straight spread 35; 37 searches/min; 136 mm/s | 2026-11-04 | MEASURED(robot log,2026-11-04) | [VC_LINE_RUN.md#run-p9-2026-11-04-boot-d8fa-run-1](../archive/s3-line-barcode/VC_LINE_RUN.md#run-p9-2026-11-04-boot-d8fa-run-1) |
| Rear axle to line sensor (GP26/27) centre (ruler) | about 122 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Run 10 calm mode | mean speed 113 counts per 50 ms; calm 55 % of time; wheel stopped 13.2 % | 2026-10-30 | MEASURED(robot log,2026-10-30) | [LINE_STEERING.md#run-10-2026-10-30-vin-wi-fi-log](../archive/s3-line-barcode/LINE_STEERING.md#run-10-2026-10-30-vin-wi-fi-log) |
| Run 3 (2026-10-27) wheels | L 29192 / R 28543 counts (about 11.5 turns) | 2026-10-27 | MEASURED(robot log,2026-10-27) | [LINE_STEERING.md#run-3-2026-10-27-vin](../archive/s3-line-barcode/LINE_STEERING.md#run-3-2026-10-27-vin) |
| Run 6 / 7 mean wheel speed | 99 / 101 counts per 50 ms | 2026-10-29 | MEASURED(robot log,2026-10-29) | [LINE_STEERING.md#run-7-2026-10-29-vin-wi-fi-log](../archive/s3-line-barcode/LINE_STEERING.md#run-7-2026-10-29-vin-wi-fi-log) |
| Run 7 / 8 / 9 speed when centred | 103 / 145 / 141 counts per 50 ms; wheel stopped 11.1 / 34.2 / 24.6 % | 2026-10-29 | MEASURED(robot log,2026-10-29) | [LINE_STEERING.md#run-9-2026-10-29-vin-wi-fi-log](../archive/s3-line-barcode/LINE_STEERING.md#run-9-2026-10-29-vin-wi-fi-log) |
| Run 8 burst swing period | 0/75 <-> 75/0 about every 125 ms; line crossed 3 sensors in about 50 ms | 2026-10-29 | MEASURED(robot log,2026-10-29) | [LINE_STEERING.md#run-8-2026-10-29-vin-wi-fi-log](../archive/s3-line-barcode/LINE_STEERING.md#run-8-2026-10-29-vin-wi-fi-log) |
| Runs 4-6 (2026-10-29) one wheel stopped | 17.1 % -> 15.4 % -> 12.9 % -> 20.3 % | 2026-10-29 | MEASURED(robot log,2026-10-29) | [LINE_STEERING.md#run-4-2026-10-29-vin-wi-fi-log](../archive/s3-line-barcode/LINE_STEERING.md#run-4-2026-10-29-vin-wi-fi-log) |
| S3-U01 line samples | 1009 samples in 1009603 us, 0 missed | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#s2-u05-result-2026-11-06](../../testing/subsystem2/TEST_LOG.md#s2-u05-result-2026-11-06) |
| Sim IR spacing (early guess) | 22 mm (+22/0/-22); footprint 3 mm | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Spin calibration / search | 131 mm/s; 633 counts each side; 200 ms pause; 6 s timeout; search 316 counts | doc ≤ 2026-11-08 | ASSUMED | [LINE.md#acquisition-and-calibration](../s3-line-barcode/LINE.md#acquisition-and-calibration) |
| Spin calibration run 1 (USB) | 15 % stalled (746 of 2535 counts); 20 %: quarter turn 633 counts; L 807 ms, R 1633 ms, back 712 ms | 2026-10-27 | MEASURED(operator,2026-10-27) | [LINE_POSITION.md#run-1-2026-10-27-spin-calibration-usb-power](../archive/s3-line-barcode/LINE_POSITION.md#run-1-2026-10-27-spin-calibration-usb-power) |
| Steering start duty (duty mode) | about 6 % wheel start; loaded below about 12 % does not turn | 2026-10-29 | MEASURED(robot log) | [LINE_STEERING.md#trim](../archive/s3-line-barcode/LINE_STEERING.md#trim) |
| STEP 7 S3-U02 TEST image | age 945/952 us; task max 207/104 us per sample (13 permille); ISR max 114 us (75 permille) | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL_ROBOT.md#test-image](../../testing/integration/KERNEL_ROBOT.md#test-image) |
| TCRT housing width (photo scale check) | about 10.6 mm (datasheet 10.2 mm) | 2026-11-16 | PHOTO-ESTIMATED(operator photos,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Travel during 300 ms gap wait at 20 % duty | about 4 cm | doc ≤ 2026-10-28 | DERIVED | [LINE_POSITION.md#events-for-the-vehicle-controller](../archive/s3-line-barcode/LINE_POSITION.md#events-for-the-vehicle-controller) |

## GP28 and barcode

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| 0cd2c03 six passes | stop at 433 mm; wheel endpoints 437/429 mm | doc ≤ 2026-11-09 | MEASURED(operator) | [TEST_LOG.md#0cd2c03-six-powered-passes-at-430-mm--100-mms](../../testing/subsystem3/TEST_LOG.md#0cd2c03-six-powered-passes-at-430-mm--100-mms) |
| 2x F sheet | about 285 mm; narrow 5.8 mm, wide 17.5 mm | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#decoder-rules-and-fixtures](../s3-line-barcode/BARCODE.md#decoder-rules-and-fixtures) |
| A4 barcode sheet | 297 x 210 mm | 2026-11-16 | MEASURED(not stated) | `mtk3-hil:docs/simulation/SIL_JUNCTION_BARCODE_RESULTS_20261116.md#version-and-protocol` |
| Barcode bar height | 160 +-10 mm | 2026-11-16 | PHOTO-ESTIMATED | `mtk3-hil:docs/simulation/SIL_JUNCTION_BARCODE_RESULTS_20261116.md#version-and-protocol` |
| Barcode guard (placeholder) | 150 mm, not verified | doc ≤ 2026-11-10 | ASSUMED | [LIVE_CALIBRATION.md#shadow-hardware-procedure](../s3-line-barcode/LIVE_CALIBRATION.md#shadow-hardware-procedure) |
| Barcode length estimate (3 chars, 1 / 3 copies) | 2 mm bar: 90 / 270 mm; 5 mm bar: 225 / 680 mm | 2026-11-06 | DERIVED | [BARCODE_READING_IDEAS.md#barcode-length](../archive/s3-line-barcode/BARCODE_READING_IDEAS.md#barcode-length) |
| Barcode print body (2x F) | 29 elements = 273.5 mm; quiet envelope 389.5 mm | doc ≤ 2026-11-13 | DERIVED | `mtk3-hil:docs/simulation/S3_SENSOR_MODEL.md#shared-geometry` |
| Barcode-to-line gap | 20 mm | doc ≤ 2026-11-09 | ASSUMED | [BARCODE.md#decoder-rules-and-fixtures](../s3-line-barcode/BARCODE.md#decoder-rules-and-fixtures) |
| Capture RAM | ADC 6144 x 4 B (12.288 s at 2 ms); motion 512 x 12 B (10.24 s at 20 ms); static 158676 B | doc ≤ 2026-11-09 | DERIVED | [BARCODE.md#capture-ram-and-diagnostics](../s3-line-barcode/BARCODE.md#capture-ram-and-diagnostics) |
| Course-size F sheet | about 141 mm; narrow 2.9 mm, wide 8.7 mm | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#decoder-rules-and-fixtures](../s3-line-barcode/BARCODE.md#decoder-rules-and-fixtures) |
| Dark floor beyond barcode map | after about 450 mm | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#scope-and-evidence](../s3-line-barcode/BARCODE.md#scope-and-evidence) |
| Encoder geometry (barcode) | 12414 counts/m | doc ≤ 2026-11-09 | DERIVED | [BARCODE.md#build-and-settings](../s3-line-barcode/BARCODE.md#build-and-settings) |
| GP28 at L40/R217 mm/s | body-forward about 5.72 mm/s, lateral about 127.57 mm/s (mean 128.5) | doc ≤ 2026-11-13 | DERIVED | `mtk3-hil:docs/simulation/S3_SENSOR_MODEL.md#mount-and-motion-limits` |
| GP28 clearance (operator) | approximately 5-6 mm; model stays 5.5 mm | 2026-11-15 | MEASURED(operator,2026-11-15) | `mtk3-hil:docs/simulation/S3_SENSOR_MODEL.md#status-and-limits` |
| GP28 diode centre left of robot centre line (ruler) | about 73 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| GP28 forward offset | about 80 mm (+-5) ahead of wheel axle | 2026-11-11 | PHOTO-ESTIMATED | [SHADOW_HARDWARE_RESULTS.md#findings](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#findings) |
| GP28 forward travel formula | d_s = 1.194 dL - 0.194 dR | 2026-11-11 | DERIVED | [SHADOW_HARDWARE_RESULTS.md#findings](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#findings) |
| GP28 lateral offset | 77 mm left of centre line | 2026-11-11 | MEASURED(operator ruler) | [SHADOW_HARDWARE_RESULTS.md#findings](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#findings) |
| GP28 LED tip height above floor (photo) | about 5.2 mm (housing bottom about 6.8 mm), +-1 mm | 2026-11-16 | PHOTO-ESTIMATED(operator photos,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| GP28 optical fit (sensor-X profile) | white 164 ADC, contrast 1335.44 ADC, sigma 2.125 mm, power 6; noise robust sigma 4.4478 ADC | doc ≤ 2026-11-13 | DERIVED | `mtk3-hil:docs/simulation/S3_SENSOR_MODEL.md#tracked-fit-and-independent-held-out-check` |
| GP28 pose: x / y left / clearance | about 80 mm / 77 mm / 5-6 mm (model 5.5) | 2026-11-15 | MEASURED(operator,2026-11-15) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| GP28 sensor height (lowest possible) | 5.5 mm; touches floor on a 10 mm hump | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#scope-and-evidence](../s3-line-barcode/BARCODE.md#scope-and-evidence) |
| GP28 sensor height (planning note) | about 5.5 mm or more | 2026-11-06 | ASSUMED | [BARCODE_READING_IDEAS.md#known-facts](../archive/s3-line-barcode/BARCODE_READING_IDEAS.md#known-facts) |
| GP28 sensor scrape | about 5.5 mm high; scraped on 10 mm hump in all floor drives | doc ≤ 2026-11-09 | MEASURED(operator) | [IMU.md#line-short-course-result-on-d51d27b](../s4-imu/IMU.md#line-short-course-result-on-d51d27b) |
| GP28 static contrast (B1 70caef0) | paper 151..170, floor 155..172, black 1334..1660 ADC (black HIGH) | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#scope-and-evidence](../s3-line-barcode/BARCODE.md#scope-and-evidence) |
| Hand-push capture encoder travel | 0..70 mm (back-driven wheels slip) | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#scope-and-evidence](../s3-line-barcode/BARCODE.md#scope-and-evidence) |
| IR barcode sensor (GP28 module) beyond the left outer wheel face | 17-18 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Large F first floor capture | full black about 1500, white about 150; 5.8 mm bars peak 600-900, appear 2-3 mm wide; contrast 1839 | 2026-11-09 | MEASURED(operator,2026-11-09) | [TEST_LOG.md#supplied-first-floor-2x-f-capture-and-optical-correction](../../testing/subsystem3/TEST_LOG.md#supplied-first-floor-2x-f-capture-and-optical-correction) |
| Large F frame span | about 73-348 mm of the pass | 2026-11-09 | MEASURED(trace) | [TEST_LOG.md#supplied-first-floor-2x-f-capture-and-optical-correction](../../testing/subsystem3/TEST_LOG.md#supplied-first-floor-2x-f-capture-and-optical-correction) |
| Live calibration gen 1 | 2070 samples; range 152..1820; high hold 70003 us; max gap 1027 us | doc ≤ 2026-11-10 | MEASURED(operator) | [LIVE_CALIBRATION.md#first-home-mat-result-and-policy-correction](../s3-line-barcode/LIVE_CALIBRATION.md#first-home-mat-result-and-policy-correction) |
| Live calibration run 3 (clean) | 1980 samples; range 152..3178; high hold 61000 us; max sample gap 1024 us | doc ≤ 2026-11-10 | MEASURED(operator) | [LIVE_CALIBRATION.md#first-home-mat-result-and-policy-correction](../s3-line-barcode/LIVE_CALIBRATION.md#first-home-mat-result-and-policy-correction) |
| Live reader corrected widths (old, before 2026-11-16) | narrow 4.6-7.0 mm; wide 13.5-21.5 mm; limits 5 s / 360 mm / 35 mm | doc ≤ 2026-11-10 | ASSUMED | [LIVE_WINDOW_READER.md#the-course-window-not-a-trailing-quiet-zone-repair](../s3-line-barcode/LIVE_WINDOW_READER.md#the-course-window-not-a-trailing-quiet-zone-repair) |
| Live reader gaps | ADC 1500 us; motor 30000 us | doc ≤ 2026-11-10 | ASSUMED | [LIVE_WINDOW_READER.md#inputs-and-bounded-work](../s3-line-barcode/LIVE_WINDOW_READER.md#inputs-and-bounded-work) |
| Live reader size gates (size-tolerant) | narrow 4.0-7.0 mm fitted from start star; wide by ratio 1.6-3.4; run ceiling min(35, 1.5w+\|bias\|) mm after the star; 5 s / 360 mm kept | 2026-11-16 | ASSUMED | [LIVE_WINDOW_READER.md#size-extension-acceptance-gates](../s3-line-barcode/LIVE_WINDOW_READER.md#size-extension-acceptance-gates) |
| Live reader white candidates | 120..220 ADC | doc ≤ 2026-11-10 | ASSUMED | [LIVE_WINDOW_READER.md#frozen-adc-thresholds-and-leading-white](../s3-line-barcode/LIVE_WINDOW_READER.md#frozen-adc-thresholds-and-leading-white) |
| Live thresholds (5c34854) | rise = white + 150, fall = white + 75, rise cap 350 | 2026-11-11 | ASSUMED | [SHADOW_HARDWARE_RESULTS.md#gp28-spin-calibration-home-mat-a883455](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#gp28-spin-calibration-home-mat-a883455) |
| Live window context / result | 988 B / 64 B | doc ≤ 2026-11-10 | MEASURED(build) | [LIVE_WINDOW_READER.md#historical-evidence-and-limits](../s3-line-barcode/LIVE_WINDOW_READER.md#historical-evidence-and-limits) |
| MQTT mat proxies | MAT 178 / 2321 ADC (unlocated, not calibration) | 2026-11-16 | ASSUMED | `mtk3-hil:docs/simulation/SIL_JUNCTION_BARCODE_RESULTS_20261116.md#version-and-protocol` |
| Narrow bar duration at 100 mm/s | about 29 / 58 ms (14 / 29 samples) | doc ≤ 2026-11-09 | DERIVED | [BARCODE.md#decoder-rules-and-fixtures](../s3-line-barcode/BARCODE.md#decoder-rules-and-fixtures) |
| Narrowest bar (planning note) | about 5 mm large, 2-3 mm small | 2026-11-06 | ASSUMED | [BARCODE_READING_IDEAS.md#known-facts](../archive/s3-line-barcode/BARCODE_READING_IDEAS.md#known-facts) |
| Old 600 mm setting stop travel | 603 / 602 mm | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#scope-and-evidence](../s3-line-barcode/BARCODE.md#scope-and-evidence) |
| Pass distance / speed / hard limit | 430 mm / 100 mm/s (25..200) / 450 mm | doc ≤ 2026-11-09 | ASSUMED | [BARCODE.md#build-and-settings](../s3-line-barcode/BARCODE.md#build-and-settings) |
| Pass stop travel (0cd2c03) / coast | 433 mm for 430 mm target; coast about 3 mm at 100 mm/s | doc ≤ 2026-11-09 | MEASURED(operator) | [BARCODE.md#scope-and-evidence](../s3-line-barcode/BARCODE.md#scope-and-evidence) |
| Quiet margin needed (2x F) | at least 58 mm each end | doc ≤ 2026-11-09 | DERIVED | [BARCODE.md#driven-procedure](../s3-line-barcode/BARCODE.md#driven-procedure) |
| Quiet noise SD (1 mm means) | about 1.3-2.4 counts | doc ≤ 2026-11-09 | MEASURED(trace) | [BARCODE.md#b1c-evidence-and-remaining-physical-tests](../s3-line-barcode/BARCODE.md#b1c-evidence-and-remaining-physical-tests) |
| Quiet-bin analysis | 792 quiet bins: no maxima reached gate; white returns about 156-179 | doc ≤ 2026-11-09 | MEASURED(trace) | [TEST_LOG.md#approved-real-fixtures-and-analysis](../../testing/subsystem3/TEST_LOG.md#approved-real-fixtures-and-analysis) |
| Real replay fitted widths, large F | n 5.766 / w 17.551 mm and n 5.776 / w 17.715 mm | 2026-11-16 | DERIVED | [REAL_WINDOW_REPLAY.md#native-retained-data-comparison](../s3-line-barcode/REAL_WINDOW_REPLAY.md#native-retained-data-comparison) |
| Rear axle to GP28 diode (ruler) | 73 mm forward | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Shadow full circuit (b405aef) | 90 s, 9.9 m, 146 mm/s; 45 losses; 7 junctions; 3 attempts 0 frames | 2026-11-11 | MEASURED(operator,2026-11-11) | [SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11) |
| SIL barcode along-sigma | 1 / 4 / 8 mm | 2026-11-16 | ASSUMED | `mtk3-hil:docs/simulation/SIL_JUNCTION_BARCODE_RESULTS_20261116.md#version-and-protocol` |
| SIL barcode sheet | 400x50 mm paper, 40 mm bar height, 389.5 mm quiet length | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |

## Sonar, servo and obstacle

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| 511 mm field reach in front of robot | about 453 mm | doc ≤ 2026-10-28 | DERIVED | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-7-sensor-to-robot-front-offset` |
| Box (M2 reattempt) | 12 cm wide, 17 cm tall; stops at about 220 mm; edge echo about 20 deg off axis | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#result-m2-reattempt-2026-11-07-mqtt-image-48c69d3-pass` |
| Box (R2-R4) | 160 x 250 mm faces | 2026-11-05 | MEASURED(operator,2026-11-05) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r3-2026-11-05-boot-0491-run-1-straight-450-mm` |
| Box flat-face echo loss angle | 20-25 deg (sim assumes 20 deg) | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#servo-and-ultrasonic](../archive/simulation/SIMULATION.md#servo-and-ultrasonic) |
| Bypass forward-watch stop / coast | STOP at 201 mm; coasted about 50 ms | 2026-10-30 | MEASURED(robot log,2026-10-30) | [OBSTACLE_BYPASS.md#run-1-2026-10-30-first-try-hit-the-bottle](../archive/OBSTACLE_BYPASS.md#run-1-2026-10-30-first-try-hit-the-bottle) |
| Bypass servo settle model | 40 ms + 4 ms per degree | 2026-10-30 | ASSUMED | [OBSTACLE_BYPASS.md#settings](../archive/OBSTACLE_BYPASS.md#settings) |
| Dashboard sonar cone | approximate +-20 deg | 2026-11-07 | MEASURED(operator,2026-11-07) | [TELEMETRY.md#ultrasonic-fan](../s1-telemetry/TELEMETRY.md#ultrasonic-fan) |
| Detour straight setting | 350 (2026-11-13; was 400, was 450) mm; sweep 150 mm; margin 30 mm | 2026-11-13 | ASSUMED | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| Detour sweep at 14 % duty | 40-43 deg/s (target 20 deg/s); 14 % does not always turn the left wheel in reverse | 2026-11-03 | MEASURED(robot log,2026-11-03) | [OBSTACLE_DETOUR_HANDOFF.md#two-sensor-runs-2026-11-03-firmware-e4ba9ec-v17](../archive/OBSTACLE_DETOUR_HANDOFF.md#two-sensor-runs-2026-11-03-firmware-e4ba9ec-v17) |
| Detour sweep speed | 40 deg/s, about 39 mm/s per wheel (was 20 deg/s, 19 mm/s) | 2026-11-13 | DERIVED | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| Early stationary echo | about 177 mm (functional, not calibration) | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Echo lost on tilted targets | above about 15 deg | doc ≤ 2026-09-21 | MEASURED(bench) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#hc-sr04` |
| Echo timeout range | 3 ms = about 515 mm; 511 mm echo = about 2.98 ms (343 m/s, about 20 C) | doc ≤ 2026-10-27 | DERIVED | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#timing-constants` |
| Encoder turn scale | 24.05 difference counts/degree | doc ≤ 2026-11-08 | DERIVED | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| Full scan time | 1829 ms (run 1); 1833 ms (run 2) | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#run-1-object-straight-ahead` |
| Hand-held 100 / 200 / 300 / 400 mm | mean 96.8 / 210.5 / 316.1 / 398.1 mm; SD 1.6 / 3.0 / 4.1 / 5.2 mm | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-5-distance-accuracy-hand-held-target` |
| Hand-held 20 / 50 mm | mean 49.3 / 61.4 mm; many NO_RETURN | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-5-distance-accuracy-hand-held-target` |
| HC-SR04+ beam width | nominal 30 deg full; scan covers about +-45 deg | doc ≤ 2026-11-08 | ASSUMED | [OBSTACLE.md#mounted-servo-limits](../s5-obstacle/OBSTACLE.md#mounted-servo-limits) |
| Kernel S5-U01 (3f889a1) fixed wall | 200 mm: medians 193/196 (188-204); 400 mm: medians 394/393 (390-407) | doc ≤ 2026-11-08 | MEASURED(operator) | [OBSTACLE.md#sonar-results](../s5-obstacle/OBSTACLE.md#sonar-results) |
| M2-S7 | STOP at 239 mm; box 218..255 mm to +20 deg; no echo at +25/+30 deg | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL_ROBOT.md#robot-image](../../testing/integration/KERNEL_ROBOT.md#robot-image) |
| Obstacle thresholds | STOP 258 mm (200 + 58); NEAR 400 mm; hysteresis 20 mm; confirm 2 | doc ≤ 2026-11-08 | DERIVED | [OBSTACLE.md#forward-monitor-thresholds](../s5-obstacle/OBSTACLE.md#forward-monitor-thresholds) |
| Pivot bench longest wheel stop | 19 mm/s: 870 ms; 28 mm/s: 330 ms; 38 mm/s: 0 ms | 2026-11-13 | MEASURED(operator,2026-11-13) | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| R1 detour speeds | sweep asked 19 got 16-18 mm/s (16.5-18.4 deg/s); straights 123-132 mm/s; back off/ALIGN 97-125 mm/s | 2026-11-05 | MEASURED(operator,2026-11-05) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#wheel-speeds-in-the-detour` |
| R1 heartbeat ISR | 103 us max, 3 missed | 2026-11-05 | MEASURED(operator,2026-11-05) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r1-2026-11-05-boot-a21a-run-1` |
| R3 closest gap (odometry, +-20 mm) | 71-95 mm | 2026-11-05 | DERIVED | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r3-2026-11-05-boot-0491-run-1-straight-450-mm` |
| R4 closest gap estimate | 42-50 mm (250 face) or 0-10 mm (160 face) | 2026-11-05 | DERIVED | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r4-2026-11-05-boot-aff0-run-1-straight-400-mm` |
| R4 false line before real line | about 48 mm; L about 2000, C about 150 | 2026-11-05 | MEASURED(robot log,2026-11-05) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r4-2026-11-05-boot-aff0-run-1-straight-400-mm` |
| Rear axle to back of ultrasonic module | about 70 mm +-3 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Rear axle to sonar front face | 70 (+-3) + 15 = 85 mm +-3 mm | 2026-11-16 | DERIVED | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Reattempt free side | 5 of 5 runs at 329-435 mm | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#result-m2-reattempt-2026-11-07-mqtt-image-48c69d3-pass` |
| Reattempt reverse centre echo change | +101..+115 mm (one +27 mm at tile edge) | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#result-m2-reattempt-2026-11-07-mqtt-image-48c69d3-pass` |
| Reverse distance | 100 mm = 1241 counts at 12414 counts/m, -100 mm/s | doc ≤ 2026-11-08 | DERIVED | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| Robot pose box | front 123, rear 67, half width 80 mm | doc ≤ 2026-11-06 | ASSUMED | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r3-2026-11-05-boot-0491-run-1-straight-450-mm` |
| S5-U01 wall (ruler) | 200 mm: medians 193/196 (188..204); 400 mm: 394/393 (390..407); about 60 mm: 19/20 not valid, 67 mm | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#s5-u01-wall-check-2026-11-06](../../testing/subsystem5/TEST_LOG.md#s5-u01-wall-check-2026-11-06) |
| Scan gap rule | free if at least 150 mm farther than obstacle; corners were 20-35 mm farther | 2026-10-27 | ASSUMED | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#profile-analysis-and-the-profile-event` |
| Scan run 1 flat-face visibility | -25..+20 deg; end readings up to 30 mm high vs ideal | 2026-10-27 | DERIVED | [SIM_ECHO_FIT.md#what-the-records-constrain](../archive/simulation/SIM_ECHO_FIT.md#what-the-records-constrain) |
| Scan run 1 object face | about 165 mm from sensor (about 107 mm from front) | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#run-1-object-straight-ahead` |
| Scan settle times | first 150 ms; step 60 ms; return 150 ms; 2 readings per point | doc ≤ 2026-10-28 | ASSUMED | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#sequence-and-timing` |
| Servo axis Z / face offset / slew (SIL) | 20 mm / 0 / 277.78 deg/s | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Servo centre 1500 us = straight ahead | not confirmed | doc ≤ 2026-11-08 | ASSUMED | [OBSTACLE.md#mounted-servo-limits](../s5-obstacle/OBSTACLE.md#mounted-servo-limits) |
| Servo datasheet range | 500-2500 us, centre 1500, 20000 us period (50 Hz), about +-90 deg | doc ≤ 2026-10-27 | ASSUMED | `569672a:docs/architecture/week8_change_requirement_1_notes.md#19-servo-calibration` |
| Servo direction | below 1500 us turns right; above turns left | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#mounted-servo-limits` |
| Servo mounted collision points | about 1040 / 1960 us (about 41 deg R/L) | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Servo mounted direction / collision | below 1500 us = right; touches at about 1040 / 1960 us (41 deg) | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s2-motion/CALIBRATION_RESULTS.md#servo` |
| Servo pulse per degree | 1000/90 us per deg | doc ≤ 2026-10-28 | ASSUMED | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#scan-points` |
| Servo pulse scale | about 11.1 us/deg around 1500 us, positive left | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Servo PWM | 50 Hz, 1 us/count (GP15) | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#hardware-and-time-boundary](../s2-motion/MOTION.md#hardware-and-time-boundary) |
| Servo PWM | 50 Hz | doc ≤ 2026-11-06 | ASSUMED | [ARCHITECTURE.md#decision-1](ARCHITECTURE.md#decision-1) |
| Servo safe clamp / scan | 1100..1900 us (about +-36 deg); scan +-30 deg | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Servo settle estimate | 42 ms + 3.6 ms/deg (150 ms for 30 deg, 60 ms for 5 deg) | doc ≤ 2026-11-08 | DERIVED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| Servo software clamp / scan range | 1100-1900 us (about +-36 deg) / +-30 deg (1167-1833 us) | doc ≤ 2026-10-28 | ASSUMED | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#mounted-servo-limits` |
| Servo touch points (mounted) | about 1040 us (41 deg right) / 1960 us (41 deg left) | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/OBSTACLE_SCAN.md#mounted-servo-limits` |
| Servo unmounted range | 500/1500/2500 us, about 180 deg | doc ≤ 2026-11-08 | MEASURED(bench) | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Sim bottle diameter | 66 mm (from 33 mm radius) | doc ≤ 2026-11-08 | DERIVED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Sim box size | 120 x 120 mm (pure guess) | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Sim clearance vs straight length (160 mm face) | 22 mm at 350; 54 mm at 400; 91 mm at 450 and 500 | 2026-11-05 | DERIVED | `569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md#run-r2-2026-11-05-boot-dcb8-run-1-fixed-25-back-off` |
| Sim narrow beam fit | beam 5 deg (+-2.5), 16 deg incidence, corner echoes to 300 mm | 2026-11-05 | DERIVED | [SIMULATION.md#narrow-beam-real-box](../archive/simulation/SIMULATION.md#narrow-beam-real-box) |
| Sim rise latency used | 215 us | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#servo-and-ultrasonic](../archive/simulation/SIMULATION.md#servo-and-ultrasonic) |
| Sim sonar face offset from servo axis | 0 mm (guess) | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#first-scenario-guesses](../archive/simulation/SIMULATION.md#first-scenario-guesses) |
| Sonar beam | about +-20 deg | 2026-11-07 | MEASURED(operator,2026-11-07) | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Sonar blind zone | about 40 mm (lowest reading 40 mm; 20 mm target reads about 45 mm) | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-6-near-range-check-at-20-mm` |
| Sonar echo quantization | about 25 us (one 40 kHz cycle) = about 4.3 mm | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-1-30-ms-timeouts-three-status-codes` |
| Sonar face ahead of pose point | 65 mm | doc ≤ 2026-11-08 | ASSUMED | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| Sonar face to robot front (echo) | mean 57.9 mm, SD 2.2 mm (80/80 VALID); about 58 mm, ruler not done | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-7-sensor-to-robot-front-offset` |
| Sonar front face behind robot front (ruler-derived) | 127.5 - 85 = 42.5 mm | 2026-11-16 | DERIVED | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| Sonar module supply / echo level | 3.3 V; carrier input limit 3.6 V | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Sonar short-reading bias | 4-7 mm short at 200 mm; 6-7 mm (about 1.7 %) at 400 mm; 30 C gives about 1.9 % | 2026-11-06 | DERIVED | [TEST_LOG.md#s5-u01-wall-check-2026-11-06](../../testing/subsystem5/TEST_LOG.md#s5-u01-wall-check-2026-11-06) |
| Sonar timing constants | trigger 10 us; rise timeout 2 ms; min trigger gap 60 ms; WATCH 100 ms | doc ≤ 2026-11-08 | ASSUMED | [OBSTACLE.md#sonar-settings](../s5-obstacle/OBSTACLE.md#sonar-settings) |
| Sonar trigger-to-rise latency | 212-217 us | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-1-30-ms-timeouts-three-status-codes` |
| Sound speed constant | assumes about 20 C; about 0.17 %/C | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#servo-and-sonar](../s2-motion/MOTION.md#servo-and-sonar) |
| Speed of sound used | about 343 m/s at about 20 C | doc ≤ 2026-10-27 | ASSUMED | `569672a:docs/architecture/week8_change_requirement_1_notes.md#14-ultrasonic-timing-within-the-100-ms-task-window` |
| STEP 8 box | 17 x 12 x 7 cm; echoes 214..290 mm at -30..+30 deg; closest 212 mm at +10 | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/MTK3_STEP8_RECORD.md#driving-observations` |
| Target at about 1 m | 48 readings mean 1016.3 mm, std 10.6 mm, range 999-1056 mm | 2026-10-27 | MEASURED(operator,2026-10-27) | `569672a:docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md#run-1-30-ms-timeouts-three-status-codes` |
| Turn-away angles needing larger toward limit | away 62/63 deg -> toward 152/153 deg | 2026-11-13 | MEASURED(robot log,2026-11-13) | [OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt](../s5-obstacle/OBSTACLE.md#4--avoidance-decision-reverse-and-reattempt) |
| Ultrasonic module depth | 15 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| v1.8 detours | 8 detours, 5 DONE, 3 stall stops | 2026-11-03 | MEASURED(robot log,2026-11-03) | [OBSTACLE_DETOUR_HANDOFF.md#two-sensor-runs-2026-11-03-firmware-e4ba9ec-v17](../archive/OBSTACLE_DETOUR_HANDOFF.md#two-sensor-runs-2026-11-03-firmware-e4ba9ec-v17) |

## IMU

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| Accel static magnitude at rest (02f7c37) | 1371-1391 mg; Z about 1375 mg, Y about 85 mg, X about 0 | doc ≤ 2026-11-08 | MEASURED(operator) | [IMU.md#operator-evidence-02f7c37](../s4-imu/IMU.md#operator-evidence-02f7c37) |
| Accel Y offset / raw level pitch | +86.5 mg / about 3.5 deg | doc ≤ 2026-11-08 | DERIVED | [IMU.md#six-face-stream-evidence-6ed89ad](../s4-imu/IMU.md#six-face-stream-evidence-6ed89ad) |
| Accel Z zero-g offset | about +300..+350 mg (half-sum +300 mg); Z scale 1076 mg/g; X/Y 1017/1030 mg/g | doc ≤ 2026-11-08 | DERIVED | [IMU.md#six-face-stream-evidence-6ed89ad](../s4-imu/IMU.md#six-face-stream-evidence-6ed89ad) |
| dfade14 hand push | corrected magnitude about 1000 mg; Z bias about +373 mg; work 1.3..1.4 ms (max 2.48); core 0.63 ms; read 0.54 ms; stack 752/2048 B | doc ≤ 2026-11-08 | MEASURED(operator) | [TEST_LOG.md#dfade14-hand-push-hardware-and-offline-analysis](../../testing/subsystem4/TEST_LOG.md#dfade14-hand-push-hardware-and-offline-analysis) |
| dfade14 hand-push encoder travel | 170..218 mm vs about 400 mm expected | doc ≤ 2026-11-08 | MEASURED(operator) | [TEST_LOG.md#dfade14-hand-push-hardware-and-offline-analysis](../../testing/subsystem4/TEST_LOG.md#dfade14-hand-push-hardware-and-offline-analysis) |
| Diagnostic C (29d57de) speed at hump peaks | about 120 mm/s | doc ≤ 2026-11-09 | MEASURED(operator) | [TEST_LOG.md#diagnostic-c-hardware-on-29d57de](../../testing/subsystem4/TEST_LOG.md#diagnostic-c-hardware-on-29d57de) |
| Diagnostic C hump peaks | 16.3 / 15.1 / 15.2 / 14.7 / 14.0 mm corrected; uncorrected 11.5 / 12.4 mm | doc ≤ 2026-11-09 | MEASURED(operator) | [IMU.md#diagnostic-c-hardware-and-operator-decision](../s4-imu/IMU.md#diagnostic-c-hardware-and-operator-decision) |
| Flat LINE lateral accel | max 36 / 61 mg | doc ≤ 2026-11-09 | MEASURED(operator) | [IMU.md#diagnostic-c-hardware-and-operator-decision](../s4-imu/IMU.md#diagnostic-c-hardware-and-operator-decision) |
| Flat runs yaw acceleration | max 4667 / 7507 mrad/s^2 | doc ≤ 2026-11-09 | MEASURED(operator) | [TEST_LOG.md#diagnostic-c-hardware-on-29d57de](../../testing/subsystem4/TEST_LOG.md#diagnostic-c-hardware-on-29d57de) |
| H0/H2 LINE runs (d51d27b) | about 146 mm/s; cpu_load 411..637 permille; stack_min_free 272..328 B; wifi_poll_gap_max up to 280190 us | 2026-11-09 | MEASURED(operator,2026-11-09) | [TEST_LOG.md#step-3-h0-and-h2-short-on-d51d27b](../../testing/subsystem4/TEST_LOG.md#step-3-h0-and-h2-short-on-d51d27b) |
| H1 (1480b37) hump peaks | 9.9 / 11.3 / 10.5 mm, count 1 each | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#step-3-h1-hardware-result](../s4-imu/IMU.md#step-3-h1-hardware-result) |
| H1 stacks | imu 1448/2048 B (600 B free); sonar 320/2048; telemetry 464/4096 | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#step-3-h1-hardware-result](../s4-imu/IMU.md#step-3-h1-hardware-result) |
| Host hump peak vs speed | 100 / 146 / 300 / 460 mm/s: 9.426 / 9.065 / 7.910 / 6.896 mm | doc ≤ 2026-11-09 | DERIVED | [IMU.md#fixture-results-synthetic-not-hardware](../s4-imu/IMU.md#fixture-results-synthetic-not-hardware) |
| Hump thresholds | ON 3.0 mm, OFF 1.5 mm; travel 30 / 30 / 600 mm; gate 40 mg; min 30 mm/s; age 50 ms | doc ≤ 2026-11-09 | ASSUMED | [IMU.md#detector](../s4-imu/IMU.md#detector) |
| I2C clock | high 126 (313 cycles) = about 399.4 kHz; SDA hold 38 cycles | doc ≤ 2026-11-08 | DERIVED | [IMU.md#transfer-bounds-and-timing](../s4-imu/IMU.md#transfer-bounds-and-timing) |
| IMU geometry defaults | effective track 111 mm (not rough 115); WB 80 mm; sensor 60 mm ahead, 130 mm high | doc ≤ 2026-11-08 | ASSUMED | [IMU.md#step-2b-support-hypotheses-and-peak-quality-now-diagnostic](../s4-imu/IMU.md#step-2b-support-hypotheses-and-peak-quality-now-diagnostic) |
| IMU height | 132 mm | 2026-11-16 | MEASURED(operator,2026-11-16) | [MEASUREMENTS.md#operator-ruler-session-2026-11-16](MEASUREMENTS.md#operator-ruler-session-2026-11-16) |
| IMU pose: ahead of axle / height | 60 mm / 130 mm | doc ≤ 2026-11-15 | MEASURED(operator) | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| IMU read wall time (02f7c37) | 529-542 us, max 624 us; max cycle gap 18975 us | doc ≤ 2026-11-08 | MEASURED(operator) | [IMU.md#transfer-bounds-and-timing](../s4-imu/IMU.md#transfer-bounds-and-timing) |
| IMU read wall time (second run) | about 530-566 us, max 592 us; max gap 17946 us | doc ≤ 2026-11-08 | MEASURED(operator) | [IMU.md#six-face-stream-evidence-6ed89ad](../s4-imu/IMU.md#six-face-stream-evidence-6ed89ad) |
| IMU stack diagnostic C | 1568/2048 B (480 B free) | doc ≤ 2026-11-09 | MEASURED(operator) | [IMU.md#diagnostic-c-hardware-and-operator-decision](../s4-imu/IMU.md#diagnostic-c-hardware-and-operator-decision) |
| INT1 check | 1600 checks: 1541 match, 59 mismatch | doc ≤ 2026-11-08 | MEASURED(operator) | [TEST_LOG.md#step-2c-hardware-on-8c5d679](../../testing/subsystem4/TEST_LOG.md#step-2c-hardware-on-8c5d679) |
| Lever accel scale | 10 rad/s^2 at 60 mm = about 61 mg = about 6.8 mm roll height | doc ≤ 2026-11-09 | DERIVED | [IMU.md#line-short-course-result-on-d51d27b](../s4-imu/IMU.md#line-short-course-result-on-d51d27b) |
| LINE runs hump peaks (146 mm/s) | 14.0 / 12.9 / 13.6 / 15.5 mm; over-read 29..55 % | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#line-short-course-result-on-d51d27b](../s4-imu/IMU.md#line-short-course-result-on-d51d27b) |
| Motor/gear vibration | about 90 Hz (lines near 25 Hz and 85..97 Hz); 4-sample mean lowers sd 40 % X, 50 % Y | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#step-2d-hardware-result](../s4-imu/IMU.md#step-2d-hardware-result) |
| Offline fit wheel-to-caster base | 79 mm | doc ≤ 2026-11-08 | DERIVED | [IMU.md#step-2b-support-hypotheses-and-peak-quality-now-diagnostic](../s4-imu/IMU.md#step-2b-support-hypotheses-and-peak-quality-now-diagnostic) |
| Offline hand-push heights | 10 mm ramp 10.2 / 11.1 mm; 5 mm object 6.0 / 6.9 mm | doc ≤ 2026-11-08 | DERIVED | [TEST_LOG.md#dfade14-hand-push-hardware-and-offline-analysis](../../testing/subsystem4/TEST_LOG.md#dfade14-hand-push-hardware-and-offline-analysis) |
| Roll filter + mean delay | 100 ms LPF + 45 ms = about 145 ms (14.5 mm at 100 mm/s) | doc ≤ 2026-11-08 | DERIVED | [IMU.md#filter-response-and-window-choice-fixtures-only](../s4-imu/IMU.md#filter-response-and-window-choice-fixtures-only) |
| SIL magnetic field | 0 / 0.3 / -0.4 gauss | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#assumptions-to-replace-with-operator-inputs` |
| Step 2b (75ff329) | work avg 1562 / max 2530 us; read 564/705 us; core 818/1053 us; static 5 mm object 5.2 mm; 10 mm ramp roll +5.4 / pitch -4.1 deg | doc ≤ 2026-11-08 | MEASURED(operator) | [TEST_LOG.md#step-2b-hardware-on75ff329](../../testing/subsystem4/TEST_LOG.md#step-2b-hardware-on75ff329) |
| Step 2c flat roll-height noise | sd 1.6..2.7 mm; median false peaks up to 5.7 mm; about 50 % variance at 12..25 Hz; 3.5 % duplicate reads | doc ≤ 2026-11-08 | MEASURED(operator) | [IMU.md#why-step-2d](../s4-imu/IMU.md#why-step-2d) |
| Step 2c hump peaks (8c5d679) | 16.8 / 13.1 / 12.4 mm | doc ≤ 2026-11-08 | MEASURED(operator) | [IMU.md#why-step-2d](../s4-imu/IMU.md#why-step-2d) |
| Step 2d bus-time estimate (n=8) | about 1.1 ms of 2 ms deadline (margin 0.37 ms); step 2b max 705, mean 564 us | doc ≤ 2026-11-08 | DERIVED | [IMU.md#read-path-and-the-2-ms-deadline](../s4-imu/IMU.md#read-path-and-the-2-ms-deadline) |
| Step 2d core_us | 1164 us average | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#cpu-and-the-d4-decision](../s4-imu/IMU.md#cpu-and-the-d4-decision) |
| Step 2d floor (0207b54) hump peaks | 10.93 / 10.85 / 10.44 mm; flat sd 0.50..0.68 mm (max 1.2) | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#step-2d-hardware-result](../s4-imu/IMU.md#step-2d-hardware-result) |
| Step 2d raw capture | about 781 samples at 400 Hz; 1 count = 4 mg; sd X 76..85, Y 170..181, Z 100..132 mg; 3.913 samples per poll (about 391 Hz) | 2026-11-09 | MEASURED(operator,2026-11-09) | [TEST_LOG.md#step-2d-hardware-on-0207b54](../../testing/subsystem4/TEST_LOG.md#step-2d-hardware-on-0207b54) |
| Step 2d read_us / work_us | 1315 (max 1610) / 2699 (max 3224) us; work about 27 % of core 0 | 2026-11-09 | MEASURED(operator,2026-11-09) | [IMU.md#step-2d-hardware-result](../s4-imu/IMU.md#step-2d-hardware-result) |
| Test hump | 10 mm high, 195 mm long; 95 mm ramps, 5 mm crest; about 6 deg | doc ≤ 2026-11-08 | MEASURED(operator) | [IMU.md#filter-response-and-window-choice-fixtures-only](../s4-imu/IMU.md#filter-response-and-window-choice-fixtures-only) |

## Timing, CPU, RAM and telemetry

| Quantity | Value | Date | Label | Source |
| --- | --- | --- | --- | --- |
| ADC conversion time (8 per channel, 3 channels) | about 48 us before software overhead | doc ≤ 2026-11-06 | DERIVED | `569672a:docs/archive/kernel/KERNEL_ROBOT.md#line-observations-and-calibration` |
| B4 floor (VIN) motion | 4862 ticks, longest 112 us, gap 9952..10046 us | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#b4-floor-cold-vin-power-on](../../testing/subsystem2/TEST_LOG.md#b4-floor-cold-vin-power-on) |
| B4-lifted lifetime | 6211 ticks, max 119 us, gap 9958..10041 us | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#b4-lifted-usb-after-flash](../../testing/subsystem2/TEST_LOG.md#b4-lifted-usb-after-flash) |
| Bare-metal heartbeat bench, 45 s | hook gaps 1000/10000 us; work max 30 us; overruns 0 | doc ≤ 2026-11-16 | MEASURED(bench) | [MOTION.md#encoder-ratio-and-drift-method](../s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| Bare-metal heartbeat idle (45 s) | 1 ms gap 1000..1000 us; 10 ms gap 10000..10000 us; ISR work max 30 us (3 %) | 2026-11-04 | MEASURED(operator) | `569672a:docs/archive/s2-motion/HEARTBEAT.md#bench-result-2026-11-04-2b64bf2-usb-power` |
| Bare-metal watchdog | reset 50 ms after heartbeat stop | 2026-11-04 | MEASURED(operator) | `569672a:docs/archive/s2-motion/HEARTBEAT.md#bench-test` |
| Bench step 0 window | 600000000 us; 2280000 edges | doc ≤ 2026-11-15 | MEASURED(bench) | `569672a:docs/kernel/BENCH_KERNEL_STEP0.md#pass-rule` |
| Cold / warm S0-U01 TIMER | warm 1009986 us; cold 1009991 us vs kernel 1010 ms | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#hardware-run-records-2026-11-06](../../testing/subsystem1/TEST_LOG.md#hardware-run-records-2026-11-06) |
| Cold-power TIMER defect | TIMER gaps 233..235 us; 500 ms rest took about 21 s | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#cold-power-timer-defect-floor-b4-blocked](../../testing/subsystem2/TEST_LOG.md#cold-power-timer-defect-floor-b4-blocked) |
| Cold-power TIMER defect (bench) | 11165 kernel ticks; TIMER gaps 233..235 us | 2026-11-06 | MEASURED(operator,2026-11-06) | `569672a:docs/archive/kernel/MTK3_MOTION_CHECKS.md#cold-power-defect-and-remaining-hardware-gate` |
| CPU idle / traffic (S1) | 117 / 139 permille; calibration 4026729 iterations/s; idle-mode 147 permille | 2026-11-07 | MEASURED(operator,2026-11-07) | [KERNEL.md#overrun-timeline-p2](../kernel/KERNEL.md#overrun-timeline-p2) |
| CR1 A1+B1 motion task gap | 9970..10025 us | 2026-11-10 | MEASURED(operator,2026-11-10) | [TEST_LOG.md#a1--b1-result-2026-11-10](../../testing/subsystem2/TEST_LOG.md#a1--b1-result-2026-11-10) |
| CR1 telemetry period | 110 ms (subject to testing) | doc ≤ 2026-10-27 | ASSUMED | `569672a:docs/architecture/week8_change_requirement_1_notes.md#decision` |
| Disabled barcode reader cost | about 73 us per 1 ms sample (about 7 % CPU); idle CPU 45 -> 54 % | 2026-11-11 | MEASURED(operator,2026-11-11) | [SHADOW_HARDWARE_RESULTS.md#findings](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#findings) |
| Full refill time (simulated) | 2.8 / 14.2 / 28.4 s at 20 / 100 / 200 ms RTT | doc ≤ 2026-11-16 | DERIVED | [TELEMETRY.md#results-and-limits](../s1-telemetry/TELEMETRY.md#results-and-limits) |
| Generator ISR per call UP / split | 5.60 / 7.43 us; 212 / 282 bp | 2026-11-15 | MEASURED(bench,2026-11-15) | `569672a:docs/kernel/SPLIT_PARKED_20261115.md#split-dual-core-placement-parked--2026-11-15` |
| GP20 dump chunk | 312 B per KR chunk; 20 ms yield every 8 rows | doc ≤ 2026-11-07 | MEASURED(operator) | `569672a:docs/archive/kernel/MTK3_USB_PRIORITY.md#consequences-checked` |
| Hardware watchdog timeout | 100 ms (SDK loads 200000 on RP2040-E1) | doc ≤ 2026-11-16 | ASSUMED | [KERNEL.md#wi-fi-stack-m9](../kernel/KERNEL.md#wi-fi-stack-m9) |
| Heading constant | 2405 counts per degree x 100 (uses 111 mm track) | doc ≤ 2026-11-07 | DERIVED | [MQTT_WIRE.md#state](../s1-telemetry/MQTT_WIRE.md#state) |
| Idle ESTOP starts travel | 322 / 354 mm | 2026-11-07 | MEASURED(operator,2026-11-07) | [TELEMETRY.md#results-and-limits](../s1-telemetry/TELEMETRY.md#results-and-limits) |
| Idle-meter stack (A1/B1 gate) | 296/512 B used, 216 B free | 2026-11-10 | MEASURED(operator,2026-11-10) | [KERNEL.md#application-stacks](../kernel/KERNEL.md#application-stacks) |
| Idle-meter startup calibration | 4000 us interrupts off | doc ≤ 2026-11-08 | ASSUMED | [KERNEL.md#watchdog](../kernel/KERNEL.md#watchdog) |
| idle_meter stack free | 216 B (staging 168d705: 240 B); idle stack then 512 -> 768 B | 2026-11-10 | MEASURED(operator,2026-11-10) | [TEST_LOG.md#a1--b1-result-2026-11-10](../../testing/subsystem2/TEST_LOG.md#a1--b1-result-2026-11-10) |
| IMU step-1 read share | 530..566 us per 10 ms = 5.3..5.7 % of one core | doc ≤ 2026-11-08 | DERIVED | `569672a:docs/kernel/RAM_BUDGET.md#cpu-estimate-and-measurement-limit` |
| IMU work share (dfade14) | 1.3..1.4 ms per 10 ms (13..14 %) | doc ≤ 2026-11-08 | MEASURED(operator) | `569672a:docs/kernel/RAM_BUDGET.md#imu-step-2b-measured-delta` |
| Kernel motion task gap (first kernel bench) | 9952..10046 us, 2110 rows | doc ≤ 2026-11-06 | MEASURED(bench) | `569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md#bench-and-tests` |
| KERNEL_RAM bench (af2e787) | line-path kernel calls 28.0 -> 14.3 us per ms; load 292 -> 243 permille; BC_CPU -34 % | doc ≤ 2026-11-16 | MEASURED(bench) | [KERNEL.md#ram-placement-kernel_ram](../kernel/KERNEL.md#ram-placement-kernel_ram) |
| KERNEL_RAM pair 1 (e06667d) | load_run 923/811; wake max 1419/1458 us; line_wake 394/539 us | doc ≤ 2026-11-16 | MEASURED(operator) | [KERNEL.md#robot-p1-pairs](../kernel/KERNEL.md#robot-p1-pairs) |
| KERNEL_RAM pair 2 (2413c4a) | load_run 947/798; wake max 1328/1459 us; line_wake 356/455 us | doc ≤ 2026-11-16 | MEASURED(operator) | [KERNEL.md#robot-p1-pairs](../kernel/KERNEL.md#robot-p1-pairs) |
| KERNEL_RAM pair 3 (b0c3a9a) | load_run 931/676; wake max 2043/1189 us; line_wake 600/136 us | doc ≤ 2026-11-16 | MEASURED(operator) | [KERNEL.md#robot-p1-pairs](../kernel/KERNEL.md#robot-p1-pairs) |
| KERNEL_RAM v2 image | ramfunc 18152 B; .data 19048 B; RAM 219/264 KB | doc ≤ 2026-11-16 | MEASURED(build) | [KERNEL.md#robot-p1-pairs](../kernel/KERNEL.md#robot-p1-pairs) |
| KERNEL_RAM v2 step max | line sample 1329 -> 854 us; barcode 1229 -> 801 us | doc ≤ 2026-11-16 | MEASURED(operator) | [KERNEL.md#robot-p1-pairs](../kernel/KERNEL.md#robot-p1-pairs) |
| Lifetime motion gap, second boot | 9965..10017 -> 9756..10247 us (TEST runner priority 20 delay < 0.25 ms) | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL_ROBOT.md#test-image](../../testing/integration/KERNEL_ROBOT.md#test-image) |
| Line snapshot age limit | 10000 us | doc ≤ 2026-11-08 | ASSUMED | [KERNEL.md#settings-and-timing](../kernel/KERNEL.md#settings-and-timing) |
| Live batch / ring | max payload 228 B (10 rows 196 B); QoS1 frame 244 of 255 B ring | doc ≤ 2026-11-07 | DERIVED | [MQTT_WIRE.md#batch-frame-kinds-rows-5-and-events-6](../s1-telemetry/MQTT_WIRE.md#batch-frame-kinds-rows-5-and-events-6) |
| M1 live/refill | 216/216 rows match USB; median age 20 ms; DUMP about 1 s | 2026-11-07 | MEASURED(operator,2026-11-07) | [TELEMETRY.md#results-and-limits](../s1-telemetry/TELEMETRY.md#results-and-limits) |
| M1-MQTT / ESTOP / M2 response max | 1470 / 1086 / 1201 us; M2 gap 9902..10099 us | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/MTK3_STEP8_RECORD.md#driving-observations` |
| M1-MQTT lifetime gap / ESTOP run gap | 9916..10087 us / 8952..11038 us (gate +38 us) | 2026-11-07 | MEASURED(operator,2026-11-07) | [TEST_LOG.md#step-8-operator-follow-up--2026-11-07](../../testing/subsystem2/TEST_LOG.md#step-8-operator-follow-up--2026-11-07) |
| M2 task gap vs bench | +-0.74 ms vs +-0.05 ms (limit 11 ms) | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL_ROBOT.md#basic-checks-on-two-tiles-2026-11-06](../../testing/integration/KERNEL_ROBOT.md#basic-checks-on-two-tiles-2026-11-06) |
| Motion BENCH tick work max / gap | 241 us; 9980..10013 us | 2026-11-10 | MEASURED(operator,2026-11-10) | [TEST_LOG.md#a1--b1-result-2026-11-10](../../testing/subsystem2/TEST_LOG.md#a1--b1-result-2026-11-10) |
| Motion board CR1 A1 | 72 -> 64 B; S2 289 -> 281 B; MQTT RAM 190672 -> 190664 B | 2026-11-10 | MEASURED(build) | [CR1.md#approved-a1-substep-motion-observation-board](CR1.md#approved-a1-substep-motion-observation-board) |
| Motion task stack high-water (before CR1) | 440 / 3072 B (2632 B free); +48 B after CR1 | 2026-11-10 | MEASURED(operator,2026-11-10) | [TEST_LOG.md#a1--b1-result-2026-11-10](../../testing/subsystem2/TEST_LOG.md#a1--b1-result-2026-11-10) |
| Motion tick | 10 ms | doc ≤ 2026-11-08 | ASSUMED | [MOTION.md#settings](../s2-motion/MOTION.md#settings) |
| motion_bench stack | 568 / 4096 B | 2026-11-10 | MEASURED(operator,2026-11-10) | [TEST_LOG.md#a1--b1-result-2026-11-10](../../testing/subsystem2/TEST_LOG.md#a1--b1-result-2026-11-10) |
| MQTT payload sizes | state v5 76 B; heartbeat v4 108 B; KR1 row 16 B; live event v2 20 B | doc ≤ 2026-11-07 | ASSUMED | [MQTT_WIRE.md#state](../s1-telemetry/MQTT_WIRE.md#state) |
| Old design review ESTOP target | stop PWM within 20 ms of safety event | doc ≤ 2026-11-06 | ASSUMED | `569672a:docs/archive/architecture/robo_pico_rtos_multicore_plan.md#11-test-matrix-and-acceptance-evidence` |
| Old UDP broadcast loss | about 75 % of live log lost (run 1) | doc ≤ 2026-10-29 | MEASURED(operator) | [STEER_WIFI_LOG.md#network](../archive/STEER_WIFI_LOG.md#network) |
| Owner CPU load estimate | approaching 80 % in normal running | doc ≤ 2026-11-12 | ASSUMED | `569672a:docs/kernel/CORE_PLACEMENT.md#performance-gate-rt-0085` |
| PIL source-to-Pico delay / expiry / endpoint | 4 ms / 5 ms / <= 3 ms | doc ≤ 2026-11-13 | ASSUMED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#wire-and-timing-boundary` |
| PIL UART link | 921600 8N1; SENSOR 66 B (71.6 %), OUTPUT 58 B (62.9 %) | doc ≤ 2026-11-13 | DERIVED | `mtk3-hil:docs/simulation/WEBOTS_PIL.md#wire-and-timing-boundary` |
| RAM log | 4501 x 16 B KR1 rows (72016 B); 512 x 12 B events (6144 B); one row per 20 ms | doc ≤ 2026-11-08 | ASSUMED | [TELEMETRY.md#ram-log-ownership](../s1-telemetry/TELEMETRY.md#ram-log-ownership) |
| Robot behind wall | 17 % ping loss | doc ≤ 2026-11-08 | MEASURED(operator) | [TELEMETRY.md#results-and-troubleshooting](../s1-telemetry/TELEMETRY.md#results-and-troubleshooting) |
| RP2040 RAM | 264 KB: 262144 B main + two 4 KB banks | doc ≤ 2026-11-08 | ASSUMED | `569672a:docs/kernel/RAM_BUDGET.md#what-it-is` |
| S0-U01 TIMER vs kernel, 1 s | 1009990 us vs 1010 ms | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#s2-u05-result-2026-11-06](../../testing/subsystem2/TEST_LOG.md#s2-u05-result-2026-11-06) |
| Shadow run stacks | all >= 496 B free; line_task 1384 B free | 2026-11-11 | MEASURED(operator,2026-11-11) | [SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11) |
| Shadow run timing | KR1 gap 9931..10066 us; heartbeat CPU up to 917 permille | 2026-11-11 | MEASURED(operator,2026-11-11) | [SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md#full-circuit-shadow-image-b405aef-school-2026-11-11) |
| Sim clock quantum | 100 us (compare 50 us) | doc ≤ 2026-11-08 | ASSUMED | [SIMULATION.md#clock-choice](../archive/simulation/SIMULATION.md#clock-choice) |
| Split core 0 / core 1 load | 621-699 / 569-626 permille; first overrun step 20, 9084 us | 2026-11-15 | MEASURED(operator,2026-11-15) | `569672a:docs/kernel/SPLIT_PARKED_20261115.md#split-dual-core-placement-parked--2026-11-15` |
| Stack free policy | at least 256 B free per named stack | doc ≤ 2026-11-10 | ASSUMED | [CR1.md#b-other-ram-savings](CR1.md#b-other-ram-savings) |
| Static NORMAL MQTT data+bss (STEP 8) | 184676 B | 2026-11-07 | MEASURED(build) | `569672a:docs/archive/MTK3_STEP8_RECORD.md#runtime-ownership-and-limits` |
| Static RAM after core merge | 204468 B (+3904 vs migration, +9328 vs staging) | doc ≤ 2026-11-12 | MEASURED(build) | `569672a:docs/kernel/RAM_BUDGET.md#core-migration-plus-staging` |
| Static RAM after Wi-Fi stack move | 192228 -> 200564 B (+8336 B) | doc ≤ 2026-11-12 | MEASURED(build) | `569672a:docs/kernel/WIFI_STACK_WATERMARK.md#ram-static-reserve-up-dynamic-allocation-down` |
| Static RAM budget table (MQTT, early) | 184784 B (70.5 %); RAM log 78160 B (4501 x 16 B = 90 s); lwIP 56881 B; stacks 22528 B; free about 60 KB | doc ≤ 2026-11-08 | MEASURED(build) | `569672a:docs/kernel/RAM_BUDGET.md#results-static-ram-at-cr1-pre-imu` |
| STEP 7 ADC work | ISR max 114 us; 69-75 permille | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL.md#overrun-timeline-p2](../kernel/KERNEL.md#overrun-timeline-p2) |
| STEP 7 S2-U01 gaps (failed gate) | 8991..11024 us and 8535..11475 us; work max 113 / 198 us | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#step-7-s2-u01-failure-and-approved-windowprobe-correction](../../testing/subsystem2/TEST_LOG.md#step-7-s2-u01-failure-and-approved-windowprobe-correction) |
| STEP 7 S2-U01 robot image | window gap 9984..10016 and 9791..10210 us; work max 41 / 28 us | 2026-11-06 | MEASURED(operator,2026-11-06) | [KERNEL_ROBOT.md#test-image](../../testing/integration/KERNEL_ROBOT.md#test-image) |
| STEP 7 S3 work / ISR share | S3 about 1.3 %, max 207 us/sample; ISR about 7.5 %, max 114 us | 2026-11-06 | MEASURED(operator,2026-11-06) | `569672a:docs/archive/kernel/KERNEL_ROBOT.md#fix-1-and-probe-a-test-only` |
| STEP 8 S0-U01 / S2-U01 | TIMER 1009753 and 1010053 us per 1010 ms; window 9514..10484 / 9520..10255 us | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/MTK3_STEP8_RECORD.md#bootsession-image-results--2026-11-07` |
| STEP 8 timing windows (motion gap) | 9508..10248 us; lifetime 9508..10491 us | 2026-11-07 | MEASURED(operator,2026-11-07) | [TEST_LOG.md#step-8-operator-follow-up--2026-11-07](../../testing/subsystem2/TEST_LOG.md#step-8-operator-follow-up--2026-11-07) |
| Task periods | motion 10 ms cyclic; line per 1 ms ADC sample; vehicle 10 ms; IMU 10 ms; GP21 poll 20 ms | doc ≤ 2026-11-08 | ASSUMED | [KERNEL.md#tasks-and-priorities](../kernel/KERNEL.md#tasks-and-priorities) |
| Task priorities | motion 10, line 11, sonar 12, vehicle 13, IMU 14, Wi-Fi 20, USB 21 | doc ≤ 2026-11-08 | ASSUMED | [ARCHITECTURE.md#tasks-and-cores](ARCHITECTURE.md#tasks-and-cores) |
| Task stack sizes | motion 3072, line 2048, sonar 2048, IMU 2048, vehicle 4096, telemetry 4096, health 2048, idle meter 512 -> 768 | doc ≤ 2026-11-08 | ASSUMED | [KERNEL.md#tasks-and-priorities](../kernel/KERNEL.md#tasks-and-priorities) |
| Telemetry fresh limits | line 10 ms, motion 20 ms, vehicle 30 ms, health 2500 ms, KR1 echo 100 ms | doc ≤ 2026-11-08 | ASSUMED | [TELEMETRY.md#interfaces-boards-and-queues](../s1-telemetry/TELEMETRY.md#interfaces-boards-and-queues) |
| Telemetry rates | RUN state 200 ms, heartbeat 1000 ms; IDLE 1000 / 5000 ms; grace 2 s | doc ≤ 2026-11-08 | ASSUMED | [TELEMETRY.md#publication-settings](../s1-telemetry/TELEMETRY.md#publication-settings) |
| UP core 0 load (P1) | 570-662 permille mean; run peak 957 | 2026-11-15 | MEASURED(operator,2026-11-15) | `569672a:docs/kernel/SPLIT_PARKED_20261115.md#split-dual-core-placement-parked--2026-11-15` |
| UP vs split | ISR max 49 vs 193 us; response max 103 vs 482 us; line sample max 132 vs 193 us; motion gap 9983..10017 vs 9909..10064 us | 2026-11-15 | MEASURED(operator,2026-11-15) | `569672a:docs/kernel/SPLIT_PARKED_20261115.md#split-dual-core-placement-parked--2026-11-15` |
| USB old priority 3 | release 9995 us; dispatch delay 4488 us; worst task gap 14467 us | doc ≤ 2026-11-13 | MEASURED(operator) | [KERNEL.md#overrun-timeline-p2](../kernel/KERNEL.md#overrun-timeline-p2) |
| USB21 (94015fd) motion gap | 9963..10041 us | doc ≤ 2026-11-16 | MEASURED(operator) | [KERNEL.md#ram-placement-kernel_ram](../kernel/KERNEL.md#ram-placement-kernel_ram) |
| USB21 dump motion gap | 9963..10041 us, 0 observations (before: 18, worst 14467 us) | doc ≤ 2026-11-07 | MEASURED(operator) | `569672a:docs/archive/kernel/MTK3_USB_PRIORITY.md#hardware-result-2026-11-07-pass` |
| Watchdog reset after last feed (S2-U05) | 99 ms (target 100 ms) | 2026-11-06 | MEASURED(operator,2026-11-06) | [TEST_LOG.md#s2-u05-two-boot-watchdog-procedure](../../testing/subsystem2/TEST_LOG.md#s2-u05-two-boot-watchdog-procedure) |
| Wi-Fi poll cadence | about 20 ms (about 50 polls/s), requested 2 ms | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/s1-telemetry/KERNEL_MQTT.md#timing-and-remaining-gates` |
| Wi-Fi task sleep | requested 2 ms; observed cadence about 20 ms | doc ≤ 2026-11-08 | MEASURED(operator) | [TELEMETRY.md#publication-settings](../s1-telemetry/TELEMETRY.md#publication-settings) |
| Wi-Fi task stack | radio only 6272 B; lwIP/MQTT 8320 B | doc ≤ 2026-11-12 | ASSUMED | `569672a:docs/kernel/WIFI_STACK_WATERMARK.md#ownership-and-creation` |
| Worst motion gap during USB dump (old USB priority 3) | 14467 us: release 9995, dispatch 4488, mutex wait 15 us; later 12783 us; lifetime 5533..14467 us | 2026-11-07 | MEASURED(operator,2026-11-07) | `569672a:docs/archive/MTK3_STEP8_RECORD.md#diag1-motion-timing-capture--2026-11-07` |
