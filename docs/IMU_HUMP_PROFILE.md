# IMU moving hump profile

Status: Implemented in the integrated `robocar_line_manager` target. Host-tested and built; not yet hardware-validated.

This record covers the IMU height profile that runs inside the line-following Manager firmware. It measures how high the front wheels climb while the robot drives forward over a hump.

## What it measures, and what it does not

The profile measures **hump height during forward travel**. It integrates pitch against real wheel travel:

```
h += distance_delta_mm * sin(pitch)
```

This is the only height source. Orientation alone cannot give a height, because a chassis on a hump and a chassis on a ramp can look the same to an accelerometer.

Two different signals do two different jobs:

- **Resultant tilt** decides that the chassis is on a hump. It is non-directional, so it catches both a front-to-back ramp, which is mostly pitch, and a one-wheel pose, which is mostly roll and which pitch alone would miss.
- **Pitch** alone feeds the height integral. Tilt never contributes height, so a roll-only pose can open an episode without inventing one.

Still out of scope for this checkpoint: a static-displacement alert as a separate API, static height from a pose, a re-calibration request API, and a shared line-calibration gate. Roll is reported in the diagnostics only, never in the snapshot. The flat reference is captured once, at start-up.

## Hardware

| Signal | Pico W pin | Note |
| --- | --- | --- |
| I2C0 SDA / SCL | GP0 / GP1 | GY-511 board pull-ups, 400 kHz |
| Data ready | GP6 | Sampling hint only; the 13 ms step is the authority |

The device is an LSM303DLHC at `0x19` (accelerometer) and `0x1E` (magnetometer). These pins do not collide with any pin the Manager, Line or Motion code already uses (GP4, GP5, GP8-GP11, GP16, GP17, GP20-GP22, GP26-GP28).

The accelerometer runs at 100 Hz, high resolution, +/-8 g. The full scale comes from one value, `IMU_CFG_ACCEL_FS_G`, which derives both the scale factor and the register bits.

## Interface

Motion stays the only owner of the encoder interrupt and the counts. The IMU never reads encoder GPIO.

```
Motion tick (20 ms, IRQ)      -> counts
Main loop (13 ms)             -> motion_consume_travel(&d_left, &d_right)
                              -> imu_feed_step(...) -> imu_publish_encoder_input()
                              -> imu_step(now_ms)
```

| Function | Owner | Purpose |
| --- | --- | --- |
| `motion_consume_travel()` | Motion | Signed per-wheel count deltas since the previous call. Wrapping-safe. Invalidated by `motion_odometer_reset()`. |
| `imu_feed_step()` | IMU | Converts counts to signed millimetres, carries the fraction, gates reverse. |
| `imu_publish_encoder_input()` | IMU | Latest-value hand-off into the 13 ms step. |
| `imu_step()` | IMU | One cycle: read, filter, classify, integrate, publish. |
| `imu_snapshot_read()` | IMU | Two-word packet, seqlock-protected. |
| `imu_event_pop()` | IMU | Discrete events: hump start, hump end, and the sensor events. |

## Distance conversion

```
distance_mm = (d_left + d_right) * circumference_um
              / (2 * counts_per_rev * 1000)
```

`MOTION_CIRCUMFERENCE_UM` and `MOTION_COUNTS_PER_REV` come from `motion_config.h`, so the 65 mm wheel and the shared 2535 counts per revolution have one definition. The IMU does not restate them.

The division keeps a remainder, in the same scaled integer unit, from one cycle to the next. Without that carry, one count of travel is 0.08 mm and would truncate to zero every cycle, so a slow climb would never accumulate height. With it, thirteen one-count cycles produce exactly 1 mm.

## Reverse gate

While the Manager is in `RECOVERY` the chassis is reversing. The feed then publishes **zero** distance and drops the carry, so a recovery can neither start nor modify a hump episode. The raw signed deltas are still published, so the diagnostic shows that the wheels really did turn.

Normal forward travel is the only travel that integrates height.

## Terrain classifier

There is one on-hump state, not a directional sequence.

| Transition | Rule | Dwell |
| --- | --- | --- |
| LEVEL to ON_HUMP | resultant tilt >= 1.5 degrees | 3 samples |
| ON_HUMP to LEVEL | resultant tilt <= 0.75 degrees | 3 samples |

An episode stays in `ON_HUMP` through the crest and the descent without changing state, and it publishes exactly one `HUMP_START` and one `HUMP_END`.

The gap between 1.5 and 0.75 degrees is the hysteresis that stops a chassis sitting near a threshold from chattering on sensor noise. Both values are **provisional**, named in `imu_config.h`, and must not be lowered further without flat-ground false-positive evidence.

A hump peak below 15 mm is a false start: it still reports `HUMP_END` with `detail=0`, but it is not retained. A one-wheel pose is exactly that case, because with pitch near zero it climbs no height, so there is nothing to retain.

The snapshot keeps its two-word ABI. The 3-bit terrain field carries `INIT`=0, `LEVEL`=1, `ON_HUMP`=2, `SENSOR_FAULT`=3, and the directional values are gone rather than kept as aliases.

## Episode phase

The public enum has one on-hump value, but the core tracks a **private** episode phase so that a broad, flat-topped crest cannot split one hump into two. The phase never leaves the core: `terr` and the snapshot only ever show `INIT`, `LEVEL`, `ON_HUMP`, `SENSOR_FAULT`.

| How the episode began | Exit rule |
| --- | --- |
| Pitch-led ascent, pitch at least +1.0 degrees at the start | Holds `ON_HUMP` across a level crest of any length until a **qualified descent**, pitch at or below -1.5 degrees for three samples, has been seen. Height and peak stay continuous through the crest. |
| Nose-down start, pitch at or below -1.0 degrees | Ends on the normal level dwell, so a descent-first episode cannot stick. |
| Pose-led, \|pitch\| below 1.0 degrees, which is the roll-only one-wheel case | Ends on the normal level dwell and never manufactures height. |

`IMU_CFG_HUMP_LEAD_DEG` sets the pitch that counts as pitch-led. The rule exists because a flat top can last far longer than the 39 ms dwell: without it, every broad hump became an ascent episode and a separate descent episode, each with its own peak, and two sub-15 mm peaks would both be rejected.

One honest consequence: a pitch-led ascent whose descent never arrives, for example because the robot is lifted off the hump and put down on flat ground, keeps `ON_HUMP` until a qualified descent happens. The state is not stuck in a broken sense, it is waiting for the geometry that opened it to close.

## Calibration

The IMU calibrates once, flat, at start-up, with the motors idle. The 2 s window measures a scalar gravity gain and a per-axis bias. The bias is the flat reference: after calibration, a level chassis reads pitch 0 degrees.

The bias is per-axis, so it applies to roll and tilt as well as pitch. Roll and tilt are therefore measured **relative to the attitude at calibration**, not to true vertical. Two consequences matter on the bench: calibrating on a slope makes that slope read as flat, and the 1.5 degree entry threshold is the same order as a plausible floor error, so the calibration surface must genuinely be flat. Confirm that `tilt` reads near 0 before driving.

The gain is accepted only inside **0.45 .. 2.20** (displayed `x1000`, so 450 to 2200). It corrects a sensor whose real sensitivity disagrees with the configured full scale; it is not an error indicator. The attached GY-511 measures about **1.83 g** for gravity, so this physical RoboCar settles near **546**. A different board will settle at its own value inside the band, and a value outside the band is rejected rather than accepted.

A fault recovery re-runs the self-test and the calibration. Park the robot flat before powering up, and let it sit still for the first two seconds.

## Diagnostics

One line at 10 Hz, separate from the 500 ms Manager report:

```
imu t=<ms> acc=<x>,<y>,<z>mg pitch=<ddg>ddg roll=<ddg>ddg tilt=<ddg>ddg
    terr=<name> h=<mm>mm hi=<mm>mm dd=<mm>mm enc=<l>/<r> gain=<x1000>
    ctrl4=0x<NN> i2c=<n> inv=<n> cal=<n> miss=<n>
```

| Field | Meaning |
| --- | --- |
| `acc` | Corrected, filtered acceleration in milli-g on the body axes (X right, Y forward, Z up). Diagnostic only |
| `roll` | Signed roll in deci-degrees. Negative when the right side drops, which is the one-wheel-on-hump pose. Diagnostic only |
| `tilt` | Non-directional resultant tilt in deci-degrees, the classifier signal. Diagnostic only |
| `terr` | Terrain state: `INIT`, `LEVEL`, `ON_HUMP`, `SENSOR_FAULT` |
| `pitch` | Filtered pitch in deci-degrees, positive nose-up. The only height input |
| `h` | Current hump height |
| `hi` | Largest retained peak for this run |
| `dd` | Distance the feed published this cycle; zero while reversing |
| `enc` | Raw signed wheel deltas, even while reversing |
| `gain` | Retained gravity gain, x1000, accepted range 450 to 2200. About 546 on this GY-511; 1000 would mean the board's scale matched the configured full scale exactly |
| `ctrl4` | Live accelerometer configuration readback, so the range can be proven |
| `i2c`, `inv` | Bus errors and rejected samples |
| `cal` | Rejected calibration windows |
| `miss` | Cycles with no encoder input |

## Bench procedure

1. Flash `build/robocar_line_manager.uf2`.
2. Park the robot on flat ground, wheels free, and leave it still for two seconds. The `imu` line must show `terr=LEVEL`, `ctrl4=0x28`, and a `cal` counter that stops increasing, with a **stable accepted gain** inside 0.45 .. 2.20 (displayed `x1000`), which is about **546** on this RoboCar because its GY-511 reports 1.83 g for gravity. A gain near 1000 is *not* the acceptance criterion for this board; a gain that keeps changing, or a `cal` that keeps incrementing, means no window has been accepted yet.
3. Start line following and drive the robot straight over a hump.
4. Drive forward over the hump. Expect `terr=ON_HUMP` while the chassis is tilted and `LEVEL` after it settles, `tilt` above 1.5 degrees on the hump and near 0 on flat ground, `pitch` rising then falling, `dd` non-zero throughout, `enc` tracking real wheel motion, and exactly one `HUMP_START` and one `HUMP_END` in the event lines.
5. Read the retained `hi` from the `HUMP_END` event and compare it with a ruler.

Error sources, all documented rather than measured away: wheel slip inflates the travelled distance, the 65 mm diameter is nominal, and a hump whose tilt is mostly roll yields little height because only pitch integrates. A one-wheel pose is therefore detected but not measured, which is the correct answer rather than a defect. A ruler comparison is the practical acceptance, not a substitute for calibration.

## Files

| File | Purpose |
| --- | --- |
| `src/imu/imu_config.h` | Every threshold, pin, gain bound and terrain limit |
| `src/imu/imu_core.c` | Filter, calibration, terrain, hump integration, collision, faults, events |
| `src/imu/imu_feed.c` | Count to millimetre conversion, carry, reverse gate |
| `src/imu/imu_snapshot.c` | Two-word packet codec with a seqlock |
| `src/imu/imu_lsm303.c` | Device configuration, readback self-test, reads |
| `src/imu/imu_hal_pico.c` | I2C0 and data-ready implementation |
| `src/robocar_line_manager.c` | 13 ms scheduling, feed wiring, 10 Hz diagnostic line |
| `tests/imu/test_imu_hump.c` | Host tests for the feed and the terrain profile |

`IMU-Telemetry/` at the workspace root remains the standalone bench image for the same sensor. It is the reference implementation for the driver and the core. The integrated copy under `src/imu/` is what ships in the line-following image, and the two must be kept in step by hand.
