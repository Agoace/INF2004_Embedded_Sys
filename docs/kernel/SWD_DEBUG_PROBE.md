# SWD debug probe

This record tells how to connect a debug probe to the Pico W of the
robot and how to use it. Use it to look at the two cores, to read memory
while the firmware runs, and to load firmware without BOOTSEL.

## Contents

- [Probes](#probes)
- [Safety](#safety)
- [Wiring](#wiring)
- [Start OpenOCD](#start-openocd)
- [Read memory while the firmware runs](#read-memory-while-the-firmware-runs)
- [Use GDB](#use-gdb)
- [Load firmware](#load-firmware)
- [Limits](#limits)

## Probes

| Probe | Where | OpenOCD runs on | Interface file |
|---|---|---|---|
| Raspberry Pi Debug Probe | school | laptop | `interface/cmsis-dap.cfg` |
| Raspberry Pi 400 GPIO header | home | Raspberry Pi 400 | `interface/raspberrypi-swd.cfg` |

Both probes use the target file `target/rp2040.cfg`. This file shows the
two cores as two hardware threads on one GDB port (3333).

The Raspberry Pi 400 is a watcher and a debug probe only. It also runs an
unrelated project. Do not change, stop or start other services on it.
Keep debug work in its Pico folder.

## Safety

| Hazard | Cause | Rule |
|---|---|---|
| The robot drives while the cores are halted | The PWM hardware keeps the last duty when the cores stop | Halt, step or set breakpoints only with the wheels off the ground or the motor power off |
| Watchdog reset during a halt | — | The firmware enables the watchdog with pause on debug (`drv_watchdog_hw.c`). A halt does not cause a reset |
| Damage from two power sources | Probe power plus USB power | Connect only SWDIO, SWCLK and GND. Do not connect 3V3 or 5V from the probe |
| Loose wires on a moving robot | Long jumper wires | Use the probe on the bench. On the track, use memory reads only |

## Wiring

Connect three wires to the debug connector at the end of the Pico W.
Power the Pico W from its own USB cable or from the robot battery.

| Pico W debug pin | Raspberry Pi Debug Probe | Raspberry Pi 400 header |
|---|---|---|
| SWCLK | `D` port, SC | pin 22 (GPIO25) |
| SWDIO | `D` port, SD | pin 18 (GPIO24) |
| GND | `D` port, GND | pin 20 |

Keep the wires short. If the connection is not stable, decrease the
adapter speed.

## Start OpenOCD

### Raspberry Pi Debug Probe (school)

Install OpenOCD on the laptop one time:

```sh
sudo dnf install openocd
```

Start it:

```sh
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
    -c "adapter speed 5000"
```

### Raspberry Pi 400 (home)

The probe uses a start script in the Pico folder of the Raspberry Pi 400.
The script binds the servers to the loopback interface only.

```sh
ssh <pi-user>@<pi-host> <pico-folder>/openocd/bin/openocd-pico
```

On the laptop, forward the GDB port through SSH:

```sh
ssh -N -L 3333:127.0.0.1:3333 <pi-user>@<pi-host>
```

### Correct start output

```text
[rp2040.core0] Cortex-M0+ r0p1 processor detected
[rp2040.core0] Examination succeed
[rp2040.core1] Cortex-M0+ r0p1 processor detected
[rp2040.core1] Examination succeed
Listening on port 3333 for gdb connections
```

If one core is missing, examine the wires and decrease the adapter
speed. OpenOCD does not halt the cores when it starts.

## Read memory while the firmware runs

OpenOCD can read memory without a halt. Use this on the track and with
motors on. Get the address from the ELF that is on the robot:

```sh
arm-none-eabi-nm -S build_make/<image>.elf | grep <symbol>
```

Then read the word through the OpenOCD Telnet port (4444):

```text
mdw <address>
mdw <address> 8
```

Use the ELF of the image that is on the robot. Addresses change with
each build. Static symbols are also in the `nm` output.

## Use GDB

Connect to the OpenOCD GDB port:

```sh
arm-none-eabi-gdb build_make/<image>.elf \
    -ex "target extended-remote :3333"
```

| Command | Result |
|---|---|
| `info threads` | Shows the two cores |
| `thread 1` / `thread 2` | Selects a core |
| `bt` | Shows the call stack of the selected core |
| `monitor halt` / `continue` | Halts / continues the two cores |
| `monitor reset halt` | Resets and halts at the boot code |

To find a deadlock or a freeze, halt the cores and read the call stack
of each core. Then read the lock words that the stacks show.

## Load firmware

Load an ELF through the probe:

```sh
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
    -c "adapter speed 5000" \
    -c "program build_make/<image>.elf verify reset exit"
```

On the Raspberry Pi 400, copy the ELF to the Pi first and use the start
script with the same `-c "program ... verify reset exit"` argument.
Use BOOTSEL and picotool as given in [BUILD.md](BUILD.md#flash) when no
probe is connected.

## Limits

- The probe adds no information about sensors or motors. Use the USB and
  MQTT output for the robot behaviour.
- A halt stops the kernel time. Deadline counters after a halt are not
  valid for that run.
- The tested combination on 2026-11-13 is the Raspberry Pi 400 with
  OpenOCD 0.12.0 (Raspberry Pi package): the two cores were examined
  without a halt. The laptop and the Raspberry Pi Debug Probe are not
  tested yet.
