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

### Conditional behaviour of the unmarked power bank

The unmarked USB power bank is conditional, not a generally reliable supply.
The operator confirmed this measured behaviour:

- It cut output during `board_diagnostics`, where the load was too low.
- It stayed on when a Wi-Fi target ran, including `wifi_telemetry_probe` and
  `motor_rotation_drift`.

These current values are estimates, not meter measurements. A non-networked
target draws roughly 30–50 mA. A target with the CYW43 radio associated and
sending at 1 Hz draws roughly 70–90 mA. The periodic transmit current spike,
not only the average current, is likely part of what satisfies the bank's
low-current detector. No dummy-load resistor is fitted. The radio is the only
known load that raises the current.

This does not make the bank safe for every target. It remains at risk of a
mid-test cutout when used with `board_diagnostics`,
`motor_encoder_calibration`, `motor_duty_calibration`, or
`motor_speed_calibration`; none of these targets use networking. A cutout
during a motor test looks exactly like a stall or a crash. The stall guard will
stop the motor safely, but the operator will diagnose the wrong cause. There
is no battery-voltage reading to separate these causes because GP28 is
allocated to a line-tracing IR sensor instead of the VBAT sense.

Relying on the radio is incidental, not designed. A firmware change, a
low-power radio state between packets, or a non-networked target can remove
the effect. Use the reliable fix if this supply must be used: fit a resistor
in parallel across the VIN `+` and `-` terminals. A resistor of about 42 ohms
would draw about 119 mA at 5 V and dissipate about 0.6 W. Use a 2 W or larger
part. A 1 W part at 0.6 W becomes hot enough to burn. The resistor must be
parallel with the board, never in series. A series resistor would drop the
supply voltage and starve the motors.

The bank's claimed 20 Ah capacity makes 0.6 W waste acceptable for bench
work. Power-bank capacity is normally quoted at cell voltage, however, so the
usable capacity at 5 V is lower. The resistor is **not currently fitted**.

For the four non-networked targets listed above, an unexplained stop may be a
power-bank cutout rather than a test fault. Do not continue until the supply
is confirmed. This does not make USB power generally safe: the 1.2 A limit
still supplies the external SSD and other devices. Reconnect USB only after
the motor stops so the saved result can be read.

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
