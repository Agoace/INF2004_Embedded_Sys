# Hardware and calibration results

Last updated: 2026-09-20

## Hardware

- Controller: Raspberry Pi Pico W.
- Carrier: Cytron Robo Pico.
- Pico SDK used during testing: 2.3.0.

## Confirmed project wiring

| Function | GPIO | Result |
| --- | --- | --- |
| GY-511 SDA | GP0 | Connected; I²C works |
| GY-511 SCL | GP1 | Connected; I²C works |
| HC-SR04 Echo | GP2 | Working |
| HC-SR04 Trigger | GP3 | Working |
| Motor 2 encoder phase A | GP4 | Working |
| Motor 2 encoder phase B | GP5 | Working |
| GY-511 DRDY | GP6 | Electrically HIGH; functional transitions not yet confirmed |
| Robo Pico motor 1 driver | GP8 and GP9 | Working; one-revolution calibration complete |
| Robo Pico motor 2 driver | GP10 and GP11 | Working in both directions |
| Servo signal | GP15 | Working |
| Motor 1 encoder phase A | GP16 | Working |
| Motor 1 encoder phase B | GP17 | Working |
| IR sensor analog output | GP26 / ADC0 | Working |
| Programmable button 1 | GP20 | Working |
| Programmable button 2 | GP21 | Working |

The motor-to-encoder pairing is:

- Robo Pico motor 1, controlled through GP8/GP9, uses encoder GP16/GP17.
- Robo Pico motor 2, controlled through GP10/GP11, uses encoder GP4/GP5.

## GY-511

The I²C scan found both expected devices:

| Address | Device |
| --- | --- |
| `0x19` | Accelerometer |
| `0x1E` | Magnetometer |

The scan confirms power, ground, SDA, SCL, and I²C communication. The DRDY line remained HIGH with no observed edges. Configure the sensor measurement mode before testing DRDY again.

## HC-SR04

- The fitted module is an **HC-SR04+**. The front silkscreen reads HC-SR04;
  the reverse side identifies the variant as HC-SR04+.
- The sensor is powered from 3.3 V, not from the 5 V servo rail.
- Tested through `board_diagnostics`: GP3 trigger and GP2 echo both function,
  the readings are stable, and the values are sane. The sensor is verified as
  working.
- The readings were more stable than in earlier testing on the 5 V rail. The
  earlier setup was not measured for comparison.
- The initial stationary reading was approximately 177 mm. This shows that the
  sensor responds, but it is not a calibration.
- The HC-SR04 is used as a presence and approximate-range sensor, not as a
  measurement instrument. Ultrasound reflects specularly: a target tilted more
  than about 15° can return no echo, soft or fabric targets can absorb the
  pulse, and narrow objects are often missed. A missing echo cannot be told
  apart from an absent obstacle, so obstacle logic must treat a timeout as
  unknown and not as clear.
- Robo Pico digital inputs are limited to 3.6 V. A classic HC-SR04 powered at
  5 V Vcc can produce a 5 V Echo signal and requires a voltage divider.
- The HC-SR04+ Echo output follows its logic supply and is 3.3 V compatible.
  No level conversion is fitted or needed for this module.
- The standard speed-of-sound constant assumes about 20 °C and changes by
  roughly 0.17% per °C. This is negligible for obstacle avoidance but matters
  if the sensor is used for measurement or mapping.

## IR sensor

- GP26 / ADC0 produced a stable analog reading of approximately 219–223 mV.
- The initial average was approximately 220 mV.
- A changing-target test is still useful to confirm the sensor response range.

## Servo

The behavior test confirmed a positional servo, not a continuous-rotation servo.

| Position | Pulse width |
| --- | ---: |
| Endpoint A | 500 µs |
| Center | 1500 µs |
| Endpoint B | 2500 µs |

- Approximate mechanical range: 180° total, or 90° on each side of center.
- The servo did not buzz or strain at the tested endpoints.
- The servo cannot rotate continuously through 360°.

A simple angle mapping is:

```c
pulse_us = 500 + (angle_degrees * 2000 / 180);
```

## Motor 1 encoder

### Current shared calibration

The current shared value is **2535 counts per wheel revolution**. The
50-revolution forward run and count-based jog alignment are recorded in the
[Motor rotation drift record](MOTOR_ROTATION_DRIFT.md). Motor 1 measured
2535.000 counts per revolution. This supersedes the earlier 2540 and 2539
values: those values came from a chosen 2500-count stop threshold plus
observed coast and a single-revolution visual check with about +/-20 counts of
visual resolution. That method could not resolve a 5-count error.

Both `motor_encoder_calibration` and `motor_speed_calibration` now use the
shared 2535 constant. The encoder calibration still stops at 2500 counts by
design: its brake compensation is now 35 counts, so 2535 - 35 remains 2500.
This preserves the physical stop behaviour and the historical comparison.

### Superseded single-revolution measurement

Test conditions:

- Motor power: 10% PWM duty.
- PWM frequency: approximately 20 kHz.
- Driver: GP8 and GP9.
- Encoder: GP16 phase A and GP17 phase B.
- The tested direction produced negative signed counts.
- These direction-dependent results were measured before the motor 1 power leads were swapped to align its physical forward direction with motor 2.

| Value | Result |
| --- | ---: |
| Stop threshold | 2500 counts |
| Final count after braking | 2540 counts |
| Measured counts per wheel revolution (superseded) | 2540 |
| Braking travel | 40 counts |
| Test duration | Approximately 2560 ms |
| Visual error | None detected |

### Ten-cycle repeatability test

| Metric | Result |
| --- | ---: |
| Average direction A magnitude | 2535.7 counts |
| Average direction B magnitude | 2532.4 counts |
| Average directional difference per cycle | 3.3 counts |
| Final signed drift | -33 counts |
| Encoder-calculated angular drift | -4.68° |
| Average encoder-calculated drift per cycle | -0.47° |
| Visually estimated wheel drift | Approximately 1° |

The encoder reports more movement than is visible at the wheel. Gearbox backlash can absorb part of the motor-shaft movement when direction changes. Repeat this directional test after the motor 1 lead swap before using its old drift compensation.

## Motor 2 encoder

### Current shared calibration

Use the shared value **2535 counts per wheel revolution**. The 50-revolution
forward run and count-based jog alignment are recorded in the [Motor rotation
drift record](MOTOR_ROTATION_DRIFT.md). Motor 2 measured 2534.940 counts per
revolution, which is within the approximately +/-0.06-count precision of the
motor 1 result.

`motor_encoder_calibration` and `motor_speed_calibration` now use 2535 for
motor 2 as well. The encoder calibration keeps its 2500-count stop threshold:
its brake compensation is 35 counts, so the threshold remains 2535 - 35.
This keeps the physical stop behaviour unchanged while correcting the angular
drift calculation.

### Superseded single-revolution measurement

Test conditions:

- Motor power: 10% PWM duty.
- PWM frequency: approximately 20 kHz.
- Encoder: GP4 phase A and GP5 phase B.
- Direction A produced negative signed counts.
- Direction B produced positive signed counts.

### One-revolution calibration

| Value | Result |
| --- | ---: |
| Measured counts per wheel revolution (superseded) | 2539 |
| Practical rounded value | 2540 |
| Compensated stop threshold at 10% power | 2500 |
| Typical braking travel | 33–39 counts |

The braking compensation is speed-dependent. Do not assume that 2500 is the correct stop threshold at a higher PWM duty.

### Ten-cycle repeatability test

The test completed ten forward/reverse wheel-revolution pairs.

| Metric | Result |
| --- | ---: |
| Average direction A magnitude | 2532.9 counts |
| Average direction B magnitude | 2532.3 counts |
| Average directional difference per cycle | 0.6 counts |
| Final signed drift | -6 counts |
| Final calculated angular drift | -0.85° |
| Average angular drift per cycle | -0.085° |
| Visually detectable final drift | None |

The phase A and phase B edge totals remained closely matched. This result indicates that both encoder channels work and that the motor 2 forward/reverse movement is repeatable while the wheel is unloaded.

## Minimum unloaded motor duty

Both wheels were lifted. Both motors were tested together at approximately 20 kHz PWM. Each duty level ran for 10 seconds, followed by a five-second stop.

### Pico USB power

| Duty | Motor 1 | Motor 2 |
| ---: | --- | --- |
| 0–5% | No encoder movement | No encoder movement |
| 6% | One encoder count; no sustained movement | No encoder movement |
| 7% | Sustained movement after about 80 ms | Sustained movement after about 80 ms |

### 5 V VIN power with USB disconnected

| Motor | Minimum detected duty | Time to 10 counts |
| --- | ---: | ---: |
| Motor 1 | 6% | Approximately 70 ms |
| Motor 2 | 7% | Approximately 80 ms |

The VIN supply reduced motor 1's unloaded start threshold from 7% to 6%. Motor 2 remained at 7%.

- Recommended common unloaded minimum: 8% to provide a margin for both motors.
- Selected startup policy: apply a 10% duty kick until the encoder detects 10 counts, with a 250 ms timeout, then change to the requested duty.

A loaded wheel can require more starting torque than a lifted wheel. If a motor does not reach 10 encoder counts during the 250 ms kick, stop it and report a stalled-motor condition.

## Unloaded motor speed at 5 V VIN

Both wheels were lifted. Both motors ran together. Each duty level used the 10% startup kick, a one-second settling period, and a five-second encoder measurement.

These historical RPM values used the superseded 2540 count value for motor 1
and 2539 for motor 2. The motor 1 values therefore read about 0.197% low; to
correct them, multiply a recorded motor 1 RPM by `2540/2535`. The motor 2
values read about 0.158% low; use `2539/2535` for those values. Fresh runs now
use 2535 and report corrected values directly. A small difference from this
table is expected and is not a hardware change.

| Duty | Motor 1 | Motor 2 | Motor 1 minus motor 2 |
| ---: | ---: | ---: | ---: |
| 10% | 26.253 RPM | 26.556 RPM | -0.303 RPM |
| 20% | 53.881 RPM | 54.352 RPM | -0.471 RPM |
| 30% | 81.486 RPM | 81.948 RPM | -0.462 RPM |
| 40% | 109.006 RPM | 109.328 RPM | -0.322 RPM |
| 50% | 136.648 RPM | 136.527 RPM | +0.121 RPM |
| 60% | 163.417 RPM | 163.136 RPM | +0.281 RPM |
| 70% | 190.842 RPM | 188.965 RPM | +1.877 RPM |
| 80% | 217.029 RPM | 214.298 RPM | +2.731 RPM |
| 90% | 242.664 RPM | 239.560 RPM | +3.104 RPM |
| 100% | 260.829 RPM | 257.860 RPM | +2.969 RPM |

Observations:

- From 10% through 60%, the motors differ by less than 0.5 RPM.
- Above 70%, motor 1 is approximately 1–3 RPM faster.
- Maximum measured unloaded speed was approximately 261 RPM for motor 1 and 258 RPM for motor 2.
- Speed is close to linear through most of the duty range, with visible saturation near 100%.
- Encoder feedback should provide better straight-line control than a fixed PWM correction. This table can be used as the initial feed-forward estimate.

## Partial unloaded speed test on Pico USB power

Motor 1 power leads were swapped before this test so both wheels used the intended physical forward direction. Both motors were powered through the Pico USB connection.

These historical RPM values also used the superseded 2540 count value for
motor 1 and 2539 for motor 2. The motor 1 values read about 0.197% low and can
be corrected by multiplying by `2540/2535`; motor 2 uses `2539/2535` and reads
about 0.158% low. Fresh runs now use 2535 and report corrected values directly.
A small difference from this table is expected and is not a hardware change.
Do not edit the recorded measurements.

| Duty | Motor 1 | Motor 2 | Motor 1 minus motor 2 |
| ---: | ---: | ---: | ---: |
| 10% | 23.815 RPM | 24.094 RPM | -0.279 RPM |
| 20% | 49.412 RPM | 49.885 RPM | -0.473 RPM |
| 30% | 74.598 RPM | 75.516 RPM | -0.918 RPM |
| 40% | 99.699 RPM | 100.570 RPM | -0.871 RPM |
| 50% | 124.209 RPM | 125.350 RPM | -1.141 RPM |
| 60% | 147.992 RPM | 149.576 RPM | -1.584 RPM |

The test stopped while settling at 70% because the Raspberry Pi reported USB over-current. The Raspberry Pi 400 has a shared 1.2 A USB peripheral limit, which also supplies the external SSD, keyboard, Pico W, and other USB devices. The event disconnected the external SSD mounted at `/srv` and aborted its ext4 journal. No 70–100% USB-powered results are valid. Do not repeat this test with the motors powered from Pico USB.

Compared with 5 V VIN, USB motor power produced approximately 8.5–10.5% lower unloaded speed over the measured 10–60% range.

## Motor power safety finding

See the [Raspberry Pi 400 USB power safety record](USB_POWER_SAFETY.md) for the full procedure and recovery steps.

- The Raspberry Pi 400 USB ports share a maximum 1.2 A peripheral current budget.
- Power the Robo Pico motors from VIN for general motor tests.
- The `motor_rotation_drift` record documents one bounded exception: one
  unloaded motor at 10% duty with the stall guard active.
- Do not use USB motor power for multiple motors, higher duty, or a loaded
  wheel.
- For VIN-only tests, disconnect USB before starting the motors.
- Store results in RAM, stop the motors, keep VIN on, and reconnect USB only to read the result.
- If a USB over-current event occurs, turn off the Robo Pico, disconnect it, reboot the Raspberry Pi, and verify `/srv` with `findmnt /srv` before continuing.

## Remaining work

- Re-measure coast under the robot's normal load for each motor.
- Confirm the shared 2535 counts-per-revolution value under the robot's normal
  load.
- Measure wheel slip on the floor. Encoder counts cannot detect wheel slip.
- Configure the GY-511 and verify DRDY transitions.
- Test the IR sensor against targets at known distances or reflectivities.
- Measure the minimum and maximum reliable ultrasonic detection distance on the
  3.3 V supply.
- Confirm what the firmware returns when no echo arrives, because the obstacle
  logic depends on that value.
