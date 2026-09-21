# Raspberry Pi 400 USB power safety

## Limit

The Raspberry Pi 400 provides a maximum total USB peripheral current of 1.2 A across its USB ports. This is a shared limit, not a separate 1.2 A limit for each port.

The external SSD, keyboard, Pico W, Robo Pico, and other USB devices share this budget. DC motor startup and high PWM duty can exceed the remaining current.

Source: [Raspberry Pi power supply documentation](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc#typical-power-requirements).

## Observed incident

On 2026-09-20, both Robo Pico motors were powered through the Pico USB connection during an unloaded speed test. The test completed measurements through 60% duty. During the 70% settling period, the Raspberry Pi reported USB over-current.

The event caused these effects:

- The motor test stopped.
- USB devices disconnected.
- The external SSD mounted at `/srv` went offline.
- The ext4 journal on the SSD aborted.
- The Raspberry Pi required a reboot before work continued.

The kernel log included messages similar to:

```text
usb port: over-current change
device offline error, dev sda
Aborting journal on device sda1
```

## Required motor-test procedure

1. Build and flash the firmware while USB is connected.
2. Turn off the Robo Pico.
3. Disconnect USB from the Pico W.
4. Connect the regulated 5 V VIN supply.
5. Lift the wheels and clear the mechanism.
6. Turn on the Robo Pico and run the test.
7. Wait until both motors stop.
8. Keep VIN power on so results remain in RAM.
9. Reconnect USB only after the motors stop.
10. Read the repeated saved result through USB serial.

## Bounded single-motor exception

The `motor_rotation_drift` measurement used one motor at 10% duty with the
wheel lifted. The operator determined this single-motor case was acceptable
on the Raspberry Pi 400 USB supply when the firmware stall guard was active.
The exception is bounded by both motor count and duty. Do not use it for two
motors, higher duty, or a loaded wheel. A rule that is routinely overridden is
worse than an accurate rule.

The VIN source available for this test is an unmarked power bank. It cuts its
output below roughly 100 mA and could not keep `board_diagnostics` alive. It is
therefore unusable as a standalone supply for this project. This does not make
USB power generally safe: the 1.2 A limit still supplies the external SSD and
other devices. Reconnect USB only after the motor stops so the saved result can
be read.

The Robo Pico datasheet gives an input range of 3.6–6 V. Its motor rail is
unregulated, so motor voltage equals the supply voltage. Motor speed therefore
changes with supply voltage, and each recorded speed table applies only at its
stated voltage. Under USB-only power, the motor rail is VUSB minus 0.4 V.

For all other motor tests, do not power the motors from the Raspberry Pi 400
USB ports.

## Recovery after an over-current event

1. Turn off the Robo Pico.
2. Disconnect the Pico USB cable.
3. Reboot the Raspberry Pi 400.
4. Verify that the SSD is mounted:

```sh
findmnt /srv
```

5. Check the kernel log for new storage errors:

```sh
dmesg --level=err,warn | tail -30
```

6. Do not continue repository writes if `/srv` is missing or reports I/O errors.

If the SSD does not mount cleanly, stop work and perform an offline filesystem check before continuing.
