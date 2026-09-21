# Pico W development environment

This repository uses CMake and the Raspberry Pi Pico C/C++ SDK. The firmware checks the exposed GPIOs for likely external connections and writes its report through USB serial.

## Requirements

Install these tools:

- CMake 3.13 or later.
- Python 3.
- An Arm GNU embedded compiler (`arm-none-eabi-gcc`).
- Git.
- Ninja or Make.
- The Raspberry Pi Pico SDK, including its Git submodules.

On Debian or Raspberry Pi OS, install the main packages:

```sh
sudo apt update
sudo apt install cmake ninja-build python3 git gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Clone the SDK outside this repository:

```sh
git clone --branch master --recurse-submodules https://github.com/raspberrypi/pico-sdk.git "$HOME/pico-sdk"
export PICO_SDK_PATH="$HOME/pico-sdk"
```

Add the `export` command to your shell configuration if you want it to apply to new terminals. You can instead pass `-DPICO_SDK_PATH=/path/to/pico-sdk` during configuration.

## Build

From the repository root, run:

```sh
cmake -S . -B build -DPICO_BOARD=pico_w
cmake --build build
```

The board is already `pico_w` by default. The explicit option makes the selected hardware clear. CMake uses an installed build tool such as Make. To use Ninja, add `-G Ninja` to the first command.

The flashable file is:

```text
build/pin_probe.uf2
```

## Flash

1. Disconnect the Pico W.
2. Hold its **BOOTSEL** button while you connect its USB cable.
3. Release **BOOTSEL** after the `RPI-RP2` storage device appears.
4. Copy `build/pin_probe.uf2` to that device.

The Pico W restarts and scans its exposed GPIOs every two seconds.

## Read USB serial output

The firmware enables USB serial and disables stdio on the default UART pins. After the board starts, connect to its serial device at any baud rate. On Linux, the device is commonly `/dev/ttyACM0`.

On Linux, read the output without an extra terminal package:

```sh
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

Press **Ctrl+C** to stop. The baud rate value is ignored by USB serial, but serial tools still require one. Alternatively, install GNU Screen with `sudo apt install screen` and run `screen /dev/ttyACM0 115200`.

## Interpret the probe

The firmware tests GPIO 0–22 and GPIO 26–28 as inputs. It applies the Pico's weak internal pull-down and pull-up resistors:

- A pin that follows both pulls is reported as open.
- A pin that remains HIGH or LOW is reported as likely connected.
- Power, ground, RUN, and internal Pico W pins are not tested.

The test does not drive a GPIO as an output. The weak pulls can still affect sensitive circuits. Disconnect any circuit that uses voltages above 3.3 V before running the probe.

A component can appear open when it has a high-impedance output, an open switch, or no path to power or ground. The result is an indication, not proof of a physical wire.

## Robo Pico diagnostics

`board_diagnostics` uses the project-specific wiring:

| Function | GPIO |
| --- | --- |
| GY-511 I²C | GP0 SDA, GP1 SCL |
| HC-SR04 | GP2 Echo, GP3 Trigger |
| Motor 2 encoder | GP4 A, GP5 B |
| GY-511 data-ready | GP6 |
| Servo | GP15 |
| Motor 1 encoder | GP16 A, GP17 B |
| IR analog output | GP26 / ADC0 |

Build and flash only this target:

```sh
cmake --build build --target board_diagnostics
picotool load -f -v -x build/board_diagnostics.uf2
```

The tool scans I²C once and then reports the ultrasonic distance, encoder edge counts, data-ready edges, and IR voltage every second. Turn each encoder slowly and check that both edge counts increase. Move an object in front of the ultrasonic and IR sensors and check that their readings change.

The diagnostic does not drive GP15 because unexpected servo movement can damage a mechanism. It cannot verify the servo signal wire without moving the servo.

A separate servo test starts at a 1.5 ms center/stop pulse, moves between 1.0 ms and 2.0 ms twice, and then returns to 1.5 ms:

```sh
cmake --build build --target servo_test
picotool load -f -v -x build/servo_test.uf2
```

Keep the mechanism clear before flashing it. A positional servo moves to a position. A continuous-rotation servo spins briefly and then stops.

The positional-servo calibration target starts GP15 at a 1500 µs center pulse. Use the Robo Pico buttons to find the usable pulse range in 10 µs steps:

- Hold GP20 to decrease the pulse width by 100 µs per second.
- Hold GP21 to increase the pulse width by 100 µs per second.
- Pressing GP20 and GP21 together returns to 1500 µs.

Build and flash it:

```sh
cmake --build build --target servo_calibration
picotool load -f -v -x build/servo_calibration.uf2
```

Read USB serial to record each pulse width. Stop moving toward an endpoint immediately if the servo stops moving, buzzes, or strains. The software clamp of 500–2500 µs is not a guarantee that the complete range is mechanically safe for a specific servo.

## Motor encoder calibration

> **Motor power warning:** Do not power the Robo Pico motors from the Raspberry Pi 400 USB ports. The Pi 400 has a shared 1.2 A USB peripheral limit. A motor test caused USB over-current and disconnected the external SSD. Follow the [USB power safety procedure](USB_POWER_SAFETY.md) and use the regulated VIN supply.

Lift the wheels before using this target. The current calibration runs a motor 1 repeatability test: tap GP20 to start ten forward/backward wheel-revolution cycles at 10% power. Each leg uses the GP16/GP17 encoder and a 2500-count stop threshold. The tool brakes between directions, prints each result, and reports cumulative signed drift. A five-second timeout limits each leg. GP21 is an emergency stop, and motor 2 stays disabled. The historical
single-revolution values were 2540 for motor 1 and 2539 for motor 2; both are
superseded by the [50-revolution measurement](MOTOR_ROTATION_DRIFT.md).

```sh
cmake --build build --target motor_encoder_calibration
picotool load -f -v -x build/motor_encoder_calibration.uf2
```

Keep hands, wires, clothing, and the USB cable clear of the wheels and drivetrain.

### 50-revolution motor rotation drift

The `motor_rotation_drift` target measures the shared counts per wheel
revolution. It runs one selected motor at 10% duty for 50 continuous forward
revolutions. The run takes about 127 seconds. GP20 starts motor 1 and GP21 is
its emergency stop. GP21 starts motor 2 and GP20 is its emergency stop.

Build and flash it:

```sh
cmake --build build --target motor_rotation_drift
picotool load -f -v -x build/motor_rotation_drift.uf2
```

The run deliberately stops 300 counts short of the ideal 126750-count total.
A drift near -277 counts is expected after unloaded coast; it includes the
deliberate undershoot and is not a motor error. Coast is reported separately.
The stall guard stops a motor after 500 ms without encoder count movement,
after a 1-second start grace period.

After a completed run, jog forward only to align a wheel-rim mark with a fixed
chassis mark. A tap of the motor button fires a 40-count coarse pulse when the
button is released. A tap of the other button fires a 3-count fine pulse on
release. Pulses use the encoder stop mechanism and have a 500 ms backstop. Do
not reverse: reverse movement engages gearbox backlash. Hold both buttons to
finish without moving. The result includes the primary 50-revolution CPR
value, a secondary 51-revolution check, and the measured jog resolution.

The firmware does not print during the motor run by design. It stores the
result in RAM and repeats it every two seconds while idle. Reconnect USB after
the motor stops to read the saved result. Follow the bounded USB exception in
the [USB power safety procedure](USB_POWER_SAFETY.md); do not generalise it
to multiple motors or higher duty.

### Minimum free-running duty

The `motor_duty_calibration` target tests both unloaded motors together. Tap GP20 to start at 0% duty. Each level runs for 10 seconds, followed by a five-second stop. Duty then increases by 1%. A motor is recorded and stopped after its encoder reaches 10 counts. GP21 stops the complete test.

```sh
cmake --build build --target motor_duty_calibration
picotool load -f -v -x build/motor_duty_calibration.uf2
```

Keep both wheels lifted. Testing both motors together includes the effect of shared power-supply load.

For a VIN-only test, disconnect USB, keep VIN power on, and press GP20. The firmware stores the completed result in RAM and repeats it every two seconds. Reconnect USB after the motors stop without switching VIN off, then open `/dev/ttyACM0` to read the saved result. Removing VIN power clears the stored result.

### Unloaded motor speed

The `motor_speed_calibration` target measures both motors from 10% through 100% duty in 10% steps. Each level uses the 10% startup kick, settles for one second, and measures encoder speed for five seconds. Results include RPM and the difference between motors. The completed table remains in RAM and repeats after USB reconnects.

```sh
cmake --build build --target motor_speed_calibration
picotool load -f -v -x build/motor_speed_calibration.uf2
```

Keep both wheels lifted. GP20 starts the test and GP21 stops it. Use VIN power because motor voltage changes the result and USB motor power can exceed the Pi 400 peripheral current limit.

A 10-cycle motor 2 test at 10% power produced an average of 2532.9 counts in direction A and 2532.3 counts in direction B. The final signed drift was -6 counts, approximately -0.85°, and no visual wheel-position drift was observed. This historical result is superseded; use the shared 2535 counts-per-revolution value in the [motor rotation drift record](MOTOR_ROTATION_DRIFT.md).

**Voltage warning:** The Robo Pico specification limits digital inputs to 3.6 V. A bare HC-SR04 can produce a 5 V Echo signal. GP2 must use a suitable level shifter or voltage divider unless the connected ultrasonic module already provides a 3.3 V Echo signal.

## Clean

```sh
rm -rf build
```

CMake caches the SDK and board paths. Delete `build/` before changing toolchains or when a configuration option appears stale.
