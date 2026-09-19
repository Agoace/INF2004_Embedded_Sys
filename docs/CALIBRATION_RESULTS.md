# Hardware and calibration results

Date: 2026-09-19

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
| Robo Pico motor 1 driver | GP8 and GP9 | Board-defined connection; motor not yet calibrated |
| Robo Pico motor 2 driver | GP10 and GP11 | Working in both directions |
| Servo signal | GP15 | Working |
| Motor 1 encoder phase A | GP16 | Connection detected; not yet calibrated |
| Motor 1 encoder phase B | GP17 | Connection detected; not yet calibrated |
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

- Trigger on GP3 and Echo on GP2 produced stable readings.
- The initial stationary reading was approximately 177 mm.
- GP2 must receive no more than 3.3 V. A bare 5 V HC-SR04 Echo output needs level conversion.

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

## Motor 2 encoder

Test conditions:

- Motor power: 10% PWM duty.
- PWM frequency: approximately 20 kHz.
- Encoder: GP4 phase A and GP5 phase B.
- Direction A produced negative signed counts.
- Direction B produced positive signed counts.

### One-revolution calibration

| Value | Result |
| --- | ---: |
| Measured counts per wheel revolution | 2539 |
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

## Remaining work

- Calibrate motor 1 and its GP16/GP17 encoder.
- Test both motors under the robot's normal load.
- Measure wheel slip on the floor. Encoder counts cannot detect wheel slip.
- Configure the GY-511 and verify DRDY transitions.
- Test the IR sensor against targets at known distances or reflectivities.
- Confirm that the HC-SR04 Echo voltage at GP2 does not exceed 3.3 V.
