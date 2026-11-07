# IMU step 1 software validation

## Scope

Local helper `mtk3-imu`, based on `mtk3-staging` `7034501`. No push, flash,
network connection or hardware measurement was used. The TEST-only design
and pending procedures are in [IMU bring-up](IMU_BRINGUP.md).

Step 0 needs no template deviation: existing make defaults already disable
the UART mirror and free GP0/GP1. All template directories are unchanged.
`check-template-changes.py` reports the same eight deviations from `c3b0954`.

## Host checks

Commands: `cmake -S tests -B <outside-build>`, then
`make -C <outside-build> -j4 --output-sync=target` and
`ctest --test-dir <outside-build> --output-on-failure`.

- 127/127 host tests pass, including the three new strict C targets.
- `test_imu_decode`: byte order, sign extension, unused accel low bits,
  +/-2 g limits, magnetic X/Z/Y ordering and separate XY/Z scales,
  -4096 overflow, signed axis permutations, invalid maps, static tilt and
  zero-vector handling.
- `test_imu_probe`: identities and configuration/readback values; a NACK at
  every configuration stage; read timeout and unchanged output; paced
  retry including 32-bit millisecond wrap; identity/readback faults.
- `test_imu_hw`: register doubles exercise reset, disable, RX and STOP
  timeouts, NACK/abort, successful read/write and length validation. The
  fake timer never advances, so these checks also prove the finite spin
  limit exits. These are register doubles, not an electrical bus test.
- Python recorder 67/67 and dashboard 158/158 pass, including the node page
  harness. No MQTT or dashboard code changed.
- `check-docs.py`, `check-template-changes.py` and `git diff --check` pass.

## Clean firmware profiles

Each profile uses a clean build and `make -j4 --output-sync=target`.
`text`, `data` and `bss` are GNU size bytes, not a runtime stack measurement.

| Profile | text | data | bss |
| --- | ---: | ---: | ---: |
| Default (SMP1, USB, Wi-Fi) | 310468 | 220 | 126392 |
| Robot SMP0, WIFI/NET none | 63584 | 152 | 119860 |
| TEST SMP0, WIFI/NET none | 63060 | 672 | 119604 |
| BENCH SMP0, WIFI/NET none | 41966 | 148 | 95460 |
| MQTT SMP0 | 374972 | 368 | 184416 |
| MQTT-TEST SMP0 | 370528 | 888 | 183756 |

All six builds have zero warnings. The four changed C units (`imu_decode`,
`imu_probe`, `imu_hw`, `subsystem4_test`) also pass real-header ARM compile
with `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror` in each profile.
SDK retained-link proofs pass in all six profiles, with no unresolved
symbols or `__atomic_*` runtime symbols. Private validation copies and their
example secrets headers are deleted after the builds.

The normal/robot/BENCH/MQTT images retain no `imu_hw_init`, `imu_hw_read`,
`imu_probe_read` or `imu_tilt` symbols. Both TEST images retain the IMU read,
probe, decode and tilt symbols. Normal robot image sizes are unchanged.

## GP0/GP1 and UART proof

For every profile, build flags contain `BOARD_UART0_FREE=1` and
`TM_CONSOLE_UART=0`. Disassembly from each linked ELF's `knl_startup_hw`
contains the clock, TIMER check and peripheral reset loop, but **no GPIO
pin-function setup loop**. The sentinel-only pin table was optimized out.
Neither `0x40014004` nor `0x4001400c` (GP0/GP1 CTRL) occurs in that function.
The compiled `tm_com.o` disassembly contains neither a UART write routine
nor UART0 base `0x40034000`. USB console source is unchanged.

Recompiling the TEST hardware adapter with UART enabled and GP0/GP1 not
free fails at the explicit compile-time `#error`, as required. This is a
compile-only rejection check, not an alternate UART firmware build.

## Limits

S4-U01 tilt, axis orientation, absent-sensor responsiveness and measured
per-read wall time are **Not run**. S4-U02 cold-power checks are **Not run**.
A 405 us ideal-wire estimate is not a measurement. No calibration, filtering,
hump detection, driving input or MQTT integration is implemented or tested.
