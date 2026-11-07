# GY-511 TEST bring-up: steps 0 and 1

## Scope and source

The operator confirmed a GY-511 (LSM303DLHC) on I2C0: SDA GP0, SCL GP1.
It has an accelerometer at `0x19` and a magnetometer at `0x1e`. It has **no
Gyroscope**. DRDY wiring is not confirmed, so the probe polls and does not
claim a DRDY pin. Use 3.3 V logic and common ground. Check that the board has
I2C pull-ups to 3.3 V; weak Pico pull-ups do not replace suitable bus pull-ups.

The register protocol, units and axis-map ideas come from Jaison's
`origin/jaison-imu` revision `b2a6bb6`, `IMU-Telemetry/src/imu_lsm303.c`,
`imu_config.h` and `imu_core.c`. This work ports those ideas, not the branch.
It does not port Jaison's calibration, filter, producer, events or hump logic.

Only `TEST=1` calls the hardware. The robot/MQTT image does not initialize or
read this device. No driving task, priority, sensor rate or wire format changes.
No stationary bias correction, magnetic calibration or heading is claimed.

## Step 0: GP0 and GP1 ownership

Inspection found that no template change is needed. The existing make
settings default to `CRASH_REPORT=0`, pass `TM_CONSOLE_UART=0` and
`BOARD_UART0_FREE=1`, and leave GP0/GP1 un-muxed at startup. The fallback in
`config/config_tm.h` remains unchanged. `imu_hw.c` rejects TEST builds unless
these two build macros permit the IMU to own the pins.

**Do not use `CRASH_REPORT=1` or `CONSOLE=uart` with this IMU TEST image.**
Disconnect the IMU before a template UART demo. USB console behavior is
unchanged. With UART disabled, early-boot and panic output has no UART path;
USB needs a running scheduler and an enumerated host. A USB silence after a
fault does not prove that the board has no fault.

The validation retains startup disassembly for all six profiles. The startup
pin-function loop must contain no UART writes to GP0/GP1 (IO_BANK0 CTRL
addresses `0x40014004` and `0x4001400c`, FUNCSEL UART value 2).

## Step 1: files and units

| File | Responsibility |
| --- | --- |
| [imu_hw.c](../app_program/subsystem4/imu_hw.c) | SDK-only register access: I2C0 and GP0/GP1, no SDK runtime or I2C library source |
| [imu_probe.c](../app_program/subsystem4/imu_probe.c) | Identity, configuration/readback, two raw reads, faults and one-second retry policy; no SDK/kernel headers |
| [imu_decode.c](../app_program/subsystem4/imu_decode.c) | Pure raw-byte conversion, axis map, static pitch/roll |
| [subsystem4_test.c](../app_program/subsystem4/subsystem4_test.c) | TEST runner: 30 s of nominal 10 ms cycles, one USB line per 100 ms |

Accelerometer: 100 Hz, normal power, high resolution, +/-2 g, BDU enabled.
The little-endian words contain signed 12-bit values left-aligned; unused
low bits are ignored. One count is 1 mg. Magnetometer: continuous 75 Hz,
+/-1.3 gauss; big-endian X,Z,Y register order. Scale XY by 1000/1100 and Z by
1000/980 to milli-gauss, truncating toward zero. A raw magnetic axis of -4096
is an overflow: `mag_valid` is false, and that vector must not be used.
Polling at 100 Hz can repeat a 75 Hz magnetic conversion; this is expected.

The **single mounting table** is `imu_robot_axes` in `imu_decode.c`:
+X right, +Y forward, +Z up. It uses identity axis maps for both sensors, matching Jaison's default.
The 6ed89ad operator stream confirms the accelerometer signs: nose up +Y,
right side up +X and top up +Z. Magnetic orientation/calibration is not
established by the accel tilt check. Each map
must be a signed permutation (three distinct axes, signs +1 or -1).

Pitch is `atan2(Y, sqrt(X*X + Z*Z))`. Roll is `atan2(X, Z)`. Both use the raw
uncalibrated acceleration vector, as a static gravity estimate. Nose-up should
give positive pitch, and right-side-up should give positive roll. Dynamic
acceleration can make both angles wrong. `g_mg` is the vector magnitude in
mg; `pitch_ddeg` and `roll_ddeg` are tenths of a degree, truncated toward zero.
The first two conversions after startup can still be settling.

## Transfer bounds and timing

The controller uses a fixed 125 MHz system clock. SCL counts are low 187 and
high 126 (313 cycles, approximately 399.4 kHz); SDA hold is 38 cycles. This
uses the Pico SDK baud calculation but no SDK clock/time calls. Only the
`hardware_i2c` header path is added in
[app_sdk_hw.mk](../build_make/app_sdk_hw.mk). `i2c.c` is not linked. TEST adds
newlib `libm` for the static tilt calculation. Kernel code has no SDK headers.

Every read/write has one 2000 us deadline plus a total 100000-iteration guard
for a stopped timer. Each raw read queues at most seven FIFO words (depth
16), uses a repeated start, drains six bytes, and waits for STOP. TX abort
(including NACK) is a counted fault. A failed cycle invalidates its sample,
leaves output values unchanged, and retries reset/configuration after one
second. The task does not hold a mutex or disable interrupts while waiting.
A stuck bus cannot hang the test. There is no claim of electrical bus recovery
for a device that holds SDA low; repair the wiring or power-cycle it.

The TEST runner uses a 10 ms kernel cyclic wake, not an SDK alarm. It reports
queued wakes and maximum cycle gap, so USB or preemption delays stay visible.
`read_us` is the last bus-call wall time; `max_us` is its maximum and
`read_total_us` its accumulated time (32-bit, modulo). These include task
preemption, not just CPU busy time. Successful configuration has a separate
first-cycle `read_us`, is not included in the maximum/total, and is not a
sample. Retry-wait cycles report zero. Faulting read/setup work is included
in the maximum/total. `reads` counts complete successful raw read pairs.

**Measured on 02f7c37:** read wall time 529–542 us, maximum 624 us per
nominal 10 ms cycle; maximum cycle gap 18975 us, missed wakes zero. Two S4
windows each gave 2999 successful reads and no bus/config faults. This
measurement is not a result for the later continuous-stream build. Mag is
still read each cycle; no cadence or scale change was made. An absent-device
hardware check remains pending. See the
[scale investigation](IMU_SCALE_INVESTIGATION.md) for the later 6ed89ad
stream timing (about 530–566 us, max 592 us), confirmed accel axis signs and
Z-offset explanation of the 1.375 g static magnitude.

## Operator tilt check (Not run)

1. Lift both wheels before powering TEST: the existing S2 test sends a short
   motion command. Confirm the IMU and USB-UART adapter do not share GP0/GP1.
2. Build `make SMP=0 WIFI=none NET=none TEST=1 -j4 --output-sync=target` in
   `build_make`. Flash only with operator approval. Open the USB console.
3. During the 30 s S4 window, keep the mounted robot level and still for 5 s.
   Expect approximately `(0,0,+1000)` mg and `g_mg` near 1000. Save the full
   USB log; do not tune offsets from one sample.
4. Raise the nose: pitch must become positive. Lower the nose: negative.
   Raise the right side: roll must become positive. Check signs and which
   axes change; update only `imu_robot_axes` if the mounting differs.
5. Return level. Record static offsets, gravity range, raw magnetic values,
   all error counters, `read_us` range, `max_us`, `gap_max_us`, and missed
   wakes. The magnetic values should change with orientation, not be stuck
   at the -4096 overflow code. Keep magnets and motor current away for this
   check. Do not interpret the result as a calibrated compass.
6. Power off, disconnect the sensor, boot the same image and save the USB
   error/retry log. It must remain responsive for the whole S4 window.
   Reconnect only with power off and repeat the normal test.
7. Record the result in [S4 test log](../testing/subsystem4/TEST_LOG.md).
   Host success does not replace this procedure or a cold-power boot check.

## Continuous operator stream

Build from `build_make` with:

```sh
make SMP=0 WIFI=none NET=none TEST=1 IMU_STREAM=1 -j4 --output-sync=target
```

`IMU_STREAM=1` requires TEST and rejects BENCH. The image name ends in
`_test_imu_stream`; the option is also in the rebuild profile, so changing it
rebuilds all objects. It starts the normal TEST subsystem services but calls
only S4's check, indefinitely. It skips all other subsystem tests, including
the short S2 motor command and the intentional watchdog reset. The normal
motion watchdog remains active; this option does not disable safety logic.
Do not use it to drive. Default `TEST=1` without this option stays unchanged:
all tests, 30 s S4 window, then the existing destructive watchdog check.

During the stream, press and release GP20 after each position settles. A
30 ms debounced press prints `[S4] MARK n`; holding the button does not repeat.
GP20 is active-low with a pull-up, is polled once per cycle and has no IRQ.
The stream prints an S4 line every nominal 100 ms until power-off. The saved
setup readbacks `ctrl1` and `ctrl4` should be 57 and 88 (hex); they are not
additional live register reads. Counter wrap during a very long run is normal.

The stream/marker hardware procedure **passed on 6ed89ad**, including six
marked faces and confirmed accel signs. The high level magnitude is a Z
offset, not uniform gain. Per-axis bias removal is later work; do not change
scales or install constants from these hand-held means. A level yaw-flip
check and absent-device/cold-power checks remain pending; see the
[scale investigation](IMU_SCALE_INVESTIGATION.md#remaining-measurements-not-run).

## Later work: not implemented

The test hump is 10 mm high and 195 mm long, with a crest about 5 mm wide
(about 6 degrees slope). Do not import Jaison's 7-degree CLIMB or 15 mm minimum
hump thresholds: both would miss this object. Calibration, filtering,
encoder-distance integration, thresholds and MQTT integration need measured
data and separate approval. No gyro bias or gyro turn-rate estimate is possible.
