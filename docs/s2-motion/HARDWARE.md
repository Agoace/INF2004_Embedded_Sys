# Hardware, wiring and calibration

## What it is

The robot uses a Raspberry Pi Pico W on a Cytron Robo Pico, two encoder
motors, a positional servo, HC-SR04+ sonar, line ADC sensors and a GY-511.
This record separates measured ratios from assumed ground distance.
[Motion settings](MOTION.md) use these values; source and wiring must agree
before a powered test. Historical calibration used Pico SDK2.3.0.

## Pins and interface

| Function | Pico pin | Current connection / owner |
| --- | --- | --- |
| GY-511 SDA/SCL | GP0/GP1 | I2C0, TEST-only IMU; USB console, crash mirror off |
| Sonar echo/trigger | GP2/GP3 | HC-SR04+, sonar service |
| Right/motor 2 encoder A/B | GP4/GP5 | Motion; forward raw count decreases, sign-1 |
| GY-511 DRDY | GP6 | Earlier level HIGH; edge behavior not established; TEST reads use polling |
| Left/motor 1 driver | GP8/GP9 | Forward/reverse PWM |
| Right/motor 2 driver | GP10/GP11 | Forward/reverse PWM |
| Servo | GP15 | Motion owns PWM; positive angle turns left |
| Left/motor 1 encoder A/B | GP16/GP17 | Forward raw count increases, sign+1 |
| Dump / start-ESTOP buttons | GP20/GP21 | Active low with pull-ups; normal vehicle samples20 ms |
| Centre line ADC | GP26/ADC0 | Current two-sensor line service |
| Left line ADC | GP27/ADC1 | Current two-sensor line service |
| Barcode raw ADC | GP28/ADC2 | Sample only; no barcode decoder; VBAT jumper open |

Early three-sensor logs used different L/C/R labels. Do not copy those
labels into the current two-sensor wiring. GP29/ADC3 is not selected.
The GY-511 scan found accelerometer0x19 and magnetometer0x1E. Current
accelerometer TEST bring-up and scale limits are in [IMU.md](../s4-imu/IMU.md);
the raw magnitude is not calibrated gravity. No magnetometer navigation
or verified DRDY transition is implied.

## Geometry and distance settings

| Value | Value used / measured | Limit |
| --- | --- | --- |
| Wheel track | 111 mm | Operator measurement2026-10-30 |
| Wheel diameter | 65 mm nominal | Effective loaded rolling diameter not calibrated |
| Servo axis to rear axle | 65 mm | Operator measurement2026-10-30 |
| Maximum width | 160 mm | Includes left barcode-reader mount |
| Overall length | 190 mm | Operator measurement2026-10-30 |
| Sensor face to front | About58 mm | Echo mean57.9 mm, not ruler; short-range reading about 11 mm high |
| Rear axle to front line sensors | 123 mm =65+58 | Derived from uncertain echo offset |
| Front-corner radius | 146.7 mm =sqrt(123²+80²) | Conservative full-width calculation; detour sweep uses 150 mm |
| Counts per wheel revolution | 2535 shared | 50-turn unloaded visual-alignment measurement |
| Counts per metre | 12414 | Derived from 2535 and nominal65 mm; not measured ground travel |
| Differential turn scale | 24.05 counts/degree, constant2405 per degree×100 | 111 mm track; wheel slip not measured |

If the true sensor-front offset is about 47 mm rather than 58 mm, front reach
becomes112 mm and corner radius about 138 mm. Measure with a ruler before
using the old offset as precise geometry. The body without the left reader
mount is narrower than 160 mm. Encoders cannot measure wheel slip or tyre
compression; precise gear ratio does not make precise ground distance.

## Encoder ratio and drift method

Each motor ran alone, wheel lifted, forward50 revolutions at 10% duty,
then coasted and was jogged **forward only** to a wheel/chassis alignment
mark. The encoder interrupt made the stop decision, deliberately300 counts
short of the expected mark. Final absolute count plus signed forward jog,
then magnitude, divided by50 gives counts/revolution. A reverse jog would
introduce gearbox backlash (lost motion at a direction change).

| Result | Motor 1 (left) | Motor 2 (right) |
| --- | ---: | ---: |
| Count at visual alignment | 126750 | 126747 |
| Revolutions | 50 | 50 |
| Counts/revolution | 2535.000 | 2534.940 |
| Shared selected value | 2535 | 2535 |
| Measured unloaded coast at 10% | 27–28 counts | 23 counts |
| Mean revolution time, USB test | 2491 ms | 2433 ms |
| Mean speed in that test | 24.08 RPM | 24.66 RPM |

The0.06-count difference is within the estimated±0.06-count/revolution
precision. The non-multiple-of-four ratio is consistent with an encoder
before a non-integer gearbox; do not assume a round gear ratio.

| Jog / old-method result | Observation |
| --- | --- |
| Fine3-count request | 3..7 actual counts |
| Coarse40-count request | 56..57 actual counts |
| Time-based8..10 ms pulse | Did not move this motor reliably |
| Starting revolution | About60..90 ms slower; wheels warmed and sped up |
| Old single-revolution visual precision | About±20 counts; could not resolve a5-count ratio error |
| Old motor 1/motor 2 ratios | 2540/2539 superseded by2535 |
| Encoder-calibration stop | 2500 counts retained, with2535-35 brake compensation |

The old2540 value was0.197% high, about 2 cm systematic error over10 m.
Coast is per-motor, speed/load dependent, not a new shared correction.
The50-turn tests were unloaded; loaded coast, slip and effective rolling
radius remain unmeasured. The full signed arithmetic, method and older
repeatability evidence are in the
[drift archive](../archive/s2-motion/MOTOR_ROTATION_DRIFT.md).

## Servo and sonar

| Setting / observation | Result |
| --- | --- |
| Unmounted servo behavior | Positional, not continuous rotation; 500/1500/2500 µs endpoints/centre, about 180° total; no strain observed in that test |
| Mounted collision points | About1040/1960 µs, roughly41° right/left |
| Current safe clamp / scan | 1100..1900 µs (about±36°); scan±30° |
| Direction | Below1500 µs turns right, above turns left, viewed from behind |
| Module | HC-SR04+ on reverse silkscreen, despite HC-SR04 front marking |
| Logic supply | 3.3 V; echo follows that supply; no level conversion fitted |
| Classic5 V HC-SR04 | Can drive5 V echo; not interchangeable without level conversion; carrier digital input limit3.6 V |
| Early stationary echo | About177 mm; functional observation, not distance calibration |
| Approximate beam | ±20°, operator measured2026-11-07; not a hard angular boundary |

Pulse mapping is about 11.1 µs/degree around1500 µs, positive left. The old
unmounted500..2500 µs test does **not** authorize that range on the mounted
robot. Ultrasound gives approximate presence/range, not obstacle width.
Tilted, narrow or soft targets can fail to return an echo. No echo is not
proof of an empty route. The standard sound-speed constant assumes about
20°C and varies about 0.17%/°C. Current sensor logic and measured optics:
[ultrasonic driver](../s5-obstacle/ULTRASONIC_DRIVER.md) and
[scan](../s5-obstacle/OBSTACLE_SCAN.md).

## Motor duty and speed results

These unloaded benches are not the loaded controller tuning. Both wheels
were lifted, approximately20 kHz PWM. The VIN test used5 V; motors/rail
voltage change the result.

| Starting/running check | Recorded result |
| --- | --- |
| USB, 0..5% | No movement |
| USB, 6% | Motor 1 one count, no sustained movement; motor 2 none |
| USB, 7% | Both sustained after about 80 ms |
| 5 V VIN unloaded start | Motor 1 at 6%, about 70 ms to 10 counts; motor 2 at 7%, about 80 ms |
| Old common unloaded policy | 8% minimum; 10% kick to 10 counts, 250 ms limit |
| Current loaded controller | 12% kick; loaded duty map and limits in [MOTION.md](MOTION.md) |

### Unloaded motor speed at 5 V VIN

The following are unchanged recorded RPM values. They used old ratios2540
(left) and2539 (right): left reads about 0.197% low, right about 0.158% low.
Apply2540/2535 or2539/2535 when comparing; new firmware uses 2535 directly.
A1 s settle and5 s measurement followed a10% start kick.

| Duty % | VIN left RPM | VIN right RPM | Earlier USB left RPM | Earlier USB right RPM |
| ---: | ---: | ---: | ---: | ---: |
| 10 | 26.253 | 26.556 | 23.815 | 24.094 |
| 20 | 53.881 | 54.352 | 49.412 | 49.885 |
| 30 | 81.486 | 81.948 | 74.598 | 75.516 |
| 40 | 109.006 | 109.328 | 99.699 | 100.570 |
| 50 | 136.648 | 136.527 | 124.209 | 125.350 |
| 60 | 163.417 | 163.136 | 147.992 | 149.576 |
| 70 | 190.842 | 188.965 | Not valid | Not valid |
| 80 | 217.029 | 214.298 | Not valid | Not valid |
| 90 | 242.664 | 239.560 | Not valid | Not valid |
| 100 | 260.829 | 257.860 | Not valid | Not valid |

The VIN motors differed by less than 0.5 RPM through 60%; left was about 1..3 RPM
faster above 70%. Earlier USB speeds were about 8.5..10.5% lower over10..60%.
The USB run ended in over-current during70% settling; no70..100% results
exist. These results do not authorize a power configuration or full-duty
loaded test.

## Power record status and limits

**USB_POWER_SAFETY is obsolete by operator decision.** Its old limits,
exceptions and recovery instructions are archived, not current operating
requirements. Do not silently apply its exceptions to a new setup.
The [full obsolete record](../archive/s2-motion/USB_POWER_SAFETY.md) preserves
the reported1.2 A Pi400 budget, SSD incident and unmarked-bank observations.
No replacement current-budget measurement was made for this rewrite.

Hardware facts retained there: carrier input3.6..6 V, unregulated motor rail,
and USB-only motor rail described as VUSB minus0.4 V. Supply voltage affects
every speed table. Clear the mechanism and lift wheels for TEST; use the
operator-approved supply arrangement for the actual hardware.

## Tests and full evidence

- [S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md): current motion/ESTOP,
  repaired encoder wiring and kernel timing checks.
- [S3 TEST_LOG](../../testing/subsystem3/TEST_LOG.md): learned line calibration;
  old three-sensor values must not be relabelled as current two-sensor data.
- [S5 TEST_LOG](../../testing/subsystem5/TEST_LOG.md): sonar and mounted scan.
- [Integration TEST_LOG](../../testing/integration/TEST_LOG.md): robot runs.
- [Original calibration](../archive/s2-motion/CALIBRATION_RESULTS.md) keeps
  every historical ADC voltage, line/floor value, direction comparison,
  motor 1/2 ten-cycle drift table and raw speed difference.
- [Original drift](../archive/s2-motion/MOTOR_ROTATION_DRIFT.md) keeps all
 50-turn conditions, jog/coast/warm-up data and limitations.

No new hardware measurement was made here. Ground distance/slip, loaded
coast, independent physical stop time and an externally corrected path
remain open. Current IMU work does not remove these encoder limits.
