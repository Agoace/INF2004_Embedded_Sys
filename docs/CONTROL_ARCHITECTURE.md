# Eventual control architecture

Status: Planned

This is a planning record. It does not implement line following, use core 1,
or change an existing target.

## Workload

The eventual Pico W line-following system must carry these tasks at the same
time:

- Four IR sensors. Three sensors feed line tracing. The fourth sensor is separate.
- The line-tracing algorithm drives both motors.
- Both motor encoders remain monitored.
- An ultrasonic sensor detects obstacles.
- A GY-511 accelerometer and magnetometer have an undecided role.
- Wi-Fi telemetry runs at 1 Hz.

The main limit is **interrupt latency and timing determinism**, not CPU
throughput. The RP2040 has two cores and the arithmetic is small. The failure
mode is a blocking peripheral that stops the control loop while the robot keeps
moving.

The interrupt and shared-state rules must follow
[Interrupt and polling conventions](INTERRUPT_CONVENTIONS.md). This note does
not repeat those rules.

## Rough load estimate

These figures are estimates, not measurements on this hardware:

| Work | Estimated load | Timing form |
| --- | --- | --- |
| Two motor encoder interrupts | About 4 kHz combined at full speed | Interrupt driven; already proven |
| Three IR line sensors | 100–200 Hz | Control loop input |
| Motor control loop | 50–100 Hz | Time-critical |
| Ultrasonic sensor | 10–20 Hz | Must not block steering |
| GY-511 over I2C | 50–100 Hz if used | Tolerates some jitter |
| Wi-Fi telemetry | 1 Hz | Tolerates jitter |

The three line sensors use GP26, GP27, and GP28. These are ADC0, ADC1, and
ADC2, so each sensor has its own ADC channel. The fourth IR sensor uses a
digital output and does not use the ADC. The three ADC conversions are
sequential and are estimated at about 2 microseconds each, or about 6
microseconds total. This is not a meaningful cost in a 100–200 Hz control
loop. A single `gpio_get` for the digital sensor is effectively free.

The Robo Pico datasheet identifies GP26 and GP27 with Grove port 6, and GP26
with GP28 on Grove ports 5 and 7. These pins are already broken out on the
board.

## Blocking costs

### Ultrasonic echo

A blocking echo wait can stall for up to about 25 ms at maximum range. During
that time, a control loop on the same core is dead while the robot continues to
move. Once the sensor shares a core with steering, echo timing must be
interrupt driven or PIO based. It must never use a blocking wait in that path.

### GY-511 I2C

A full GY-511 read is six accelerometer bytes and six magnetometer bytes over a
100–400 kHz bus. The estimated cost is roughly 300–600 microseconds per full
read. Repeating it at 50–100 Hz on the control core steals time from the loop.
This is an estimate, not a measurement on this hardware.

### CYW43 Wi-Fi

The CYW43 Wi-Fi driver can block for milliseconds. At 1 Hz its total cost is
small, but one send inside a steering iteration can cause a steering glitch.
Wi-Fi must not run in the time-critical control path.

## Proposed core split

This is a proposal, not a decision:

- **Core 0:** time-critical control only: encoder interrupts, IR sampling, the
  line-tracing decision, and motor output.
- **Core 1:** peripherals that tolerate jitter: ultrasonic, I2C, Wi-Fi, and
  telemetry.

Core 1 peripherals publish their latest value into a shared structure. Core 0
reads the most recent snapshot and never calls a blocking peripheral itself.
Shared state between cores needs the same discipline as shared state between an
interrupt and the main loop. Follow
[Interrupt and polling conventions](INTERRUPT_CONVENTIONS.md).

This structure lets the sensor set change without restructuring the control
loop. If a peripheral is removed, its publisher is removed and the control
loop still reads a defined latest-value snapshot.

## GY-511 role: undecided

It is not decided whether the GY-511 participates in control. The chassis may
be magnetically noisy, so heading may be unusable. The accelerometer may still
be useful for tilt detection, or for detecting that the robot was lifted or
struck. If the device contributes nothing to steering, it belongs only in
telemetry, where its latency cost does not affect control.

The deferred test is:

1. Configure the magnetometer.
2. Log raw X, Y, and Z values while driving the motors at different duty levels.
3. Check whether the readings move with motor current.

A reading that moves with duty means motor current corrupts the field and
heading is unusable. The result depends on sensor position, so run the test
only after the sensor has its final mounting.

The publish-into-shared-state pattern keeps this question open. If the sensor
proves useless, remove one publisher and change nothing else in the control
loop.

## IR and battery wiring decision

GP28 is allocated to the third line-tracing IR sensor. The Robo Pico VBAT
sense jumper routes `VIN/2` to GP28/ADC2, so the battery sense cannot be used
at the same time. The operator chose the IR sensor instead of supply-voltage
telemetry because the current supply is a USB power bank with a regulated 5 V
output and there is little voltage variation to observe.

This decision has two important caveats:

- The power bank already cut output during `board_diagnostics` because the
  current draw was too low. The CYW43 radio keeps it on for current Wi-Fi
  targets, but this does not cover non-networked motor tests. Without a
  voltage reading a mid-run dropout cannot be distinguished from a stall or a
  crash. The stall guard stops the motor, but the diagnosis remains ambiguous.
- If the supply changes to one 18650 or 21700 cell connected directly to VIN,
  the reasoning reverses. A bare cell falls from about 4.2 V to 3.6 V during
  discharge. The motor rail is unregulated, so motor speed follows cell
  voltage. The board stops below its 3.6 V minimum with no warning. A voltage
  reading is useful in that case.

GP28 is a wiring commitment, not only a software choice. Changing this later
requires rewiring.

