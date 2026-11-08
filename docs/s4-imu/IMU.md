# GY-511 IMU: bring-up, scale and validation

The three source records below are retained in full. Their original revision
scopes and historical Not-run statements remain attached to their evidence.

## Source: IMU_BRINGUP.md

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
| [imu_hw.c](../../app_program/subsystem4/imu_hw.c) | SDK-only register access: I2C0 and GP0/GP1, no SDK runtime or I2C library source |
| [imu_probe.c](../../app_program/subsystem4/imu_probe.c) | Identity, configuration/readback, two raw reads, faults and one-second retry policy; no SDK/kernel headers |
| [imu_decode.c](../../app_program/subsystem4/imu_decode.c) | Pure raw-byte conversion, axis map, static pitch/roll |
| [subsystem4_test.c](../../app_program/subsystem4/subsystem4_test.c) | TEST runner: 30 s of nominal 10 ms cycles, one USB line per 100 ms |

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
[app_sdk_hw.mk](../../build_make/app_sdk_hw.mk). `i2c.c` is not linked. TEST adds
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
[scale investigation](IMU.md) for the later 6ed89ad
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
7. Record the result in [S4 test log](../../testing/subsystem4/TEST_LOG.md).
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
[scale investigation](IMU.md#remaining-measurements-not-run).

## Later work: not implemented

The test hump is 10 mm high and 195 mm long, with a crest about 5 mm wide
(about 6 degrees slope). Do not import Jaison's 7-degree CLIMB or 15 mm minimum
hump thresholds: both would miss this object. Calibration, filtering,
encoder-distance integration, thresholds and MQTT integration need measured
data and separate approval. No gyro bias or gyro turn-rate estimate is possible.

## Source: IMU_SCALE_INVESTIGATION.md

# GY-511 acceleration scale investigation

## Operator evidence: 02f7c37

Two USB-only TEST windows each produced 2999 complete reads. Fault, NACK,
timeout, identity and configuration counters were zero. Read wall time was
529–542 us, maximum 624 us per nominal 10 ms cycle; maximum cycle gap was
18975 us and no queued wake was missed. Magnetic data was valid and changed
with tilt (reported X range 249–328 milli-gauss).

At rest the acceleration magnitude was 1371–1391 mg, with Z about 1375 mg,
Y about 85 mg and X near zero. Side tilts gave roll about +170/-195 tenths
of a degree and X about +400/-440 mg. The operator missed the short windows;
nose-up/down positions were not captured. This is partial evidence, not an
axis calibration or a complete S4-U01 pass. The private raw log stays private.

## Six-face stream evidence: 6ed89ad

S4-U03 **PASS**, USB-only continuous stream: 16129 successful reads; fault,
NACK, timeout, identity and configuration counters all zero. Saved setup
readbacks were `ctrl1=57`, `ctrl4=88`. Read wall time was about 530–566 us,
maximum 592 us per nominal 10 ms cycle. Maximum cycle gap was 17946 us,
missed wakes zero. Six GP20 markers were debounced with no repeats.

Means below use samples +3 through +25 after each MARK. Positions were
hand-held, not jig-exact. Values are uncorrected mg, rounded as reported by
the operator; a blank magnitude means it was not supplied.

| MARK | Position | X mg | Y mg | Z mg | Magnitude mg |
| --- | --- | ---: | ---: | ---: | ---: |
| M1 | Top up | -0.1 | 82 | 1376 | 1378 |
| M2 | Upside down | -128 | 124 | -776 | 797 |
| M3 | Nose up | 37 | 1116 | 459 | |
| M4 | Nose down | -44 | -943 | 322 | |
| M5 | Right side up | 1012 | 107 | 374 | |
| M6 | Left side up | -1022 | 115 | 315 | |

**The 1.375 g level reading is explained by a large positive Z zero-g
offset, not a uniform gain error.** Z is +315 to +459 mg when nominally
horizontal. Its opposed-face half-sum is `(1376 - 776) / 2 = +300 mg`,
consistent with an offset about +300 to +350 mg. The Z half-difference is
1076 mg/g. X and Y half-differences are about 1017 and 1030 mg/g. These
hand-held data are diagnostic estimates, not approved gain corrections.

The Y opposed-face half-sum is `(1116 - 943) / 2 = +86.5 mg`, approximately
+87 mg, matching level Y=82 mg. Thus a roughly 3.5-degree raw level pitch
must not be called mounting pitch: Y offset may explain it. A 180-degree
yaw flip on the same level support is needed to separate offset from
mounting/support tilt.

The identity axis map is **confirmed**: nose up gives +Y, right side up
+X, top up +Z. The S4-U01 sign portion therefore passes through this stream
run. This does not complete its absent-device check or cold-power/MQTT-TEST
acceptance.

## Datasheet and code audit

Reference: ST LSM303DLHC, DocID018771 Rev 2, Table 3 (page 10), Tables 26/27
(page 27), and output registers (pages 30–31).
[ST datasheet](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf),
[Pololu mirror](https://www.pololu.com/file/0J564/LSM303DLHC.pdf).

| Item | Required setting | Driver/decoder |
| --- | --- | --- |
| CTRL_REG1_A | 100 Hz, low power off, XYZ enabled | `0x57` |
| CTRL_REG4_A | BDU=1, BLE=0, FS=00 (+/-2 g), HR=1 | `0x88` |
| Register byte order | Lower address is low byte | Little-endian assembly |
| HR signed sample | 12-bit left aligned | Unsigned word divided by 16, then subtract 4096 if its sign bit is set |
| Sensitivity | Table 3: 1 mg/LSB for FS=00 | One decoded count is one mg |

The driver reads the configured registers back and refuses a mismatch.
The stream confirmed saved setup readbacks `ctrl1=57` and `ctrl4=88`, with
zero config faults and no extra per-cycle register transactions.
The host tests check positive/negative counts, ignored low bits and limits;
there is no floating-point scale factor in the accel conversion.

**No decode/configuration bug was found in this audit.** The later six-face
measurements identify the Z offset as the cause of the high level magnitude;
no 1/1.375 multiplier or full-scale change is justified.

## Calibration consequence (not implemented)

Calibration must subtract a **per-axis bias**. Jaison's stationary-level
method (mean measured vector minus expected `(0,0,1000)` mg) works for
pitch-from-level: it removes the Z offset and Y offset together. It assumes
the robot is stationary on a level surface and defines that pose as level;
it does not separately measure mounting angle, support tilt or axis gains.

Jaison's 0.90–1.10 g magnitude gate must be applied **after bias removal**,
or dropped. Applied to these raw readings it would reject calibration
indefinitely. A gate after correction alone is not proof of stationarity:
separate motion/noise checks still need a later design. No calibration code,
bias constants, scales or thresholds change in this documentation update.

## Remaining measurements (Not run)

Use the continuous stream in
[IMU bring-up](IMU.md#continuous-operator-stream), wheels lifted.
Repeat level measurements before/after a 180-degree yaw flip on the same
support to distinguish Y offset from mounting/support tilt. Better-supported
opposed faces can refine the diagnostic bias/gain estimates, but do not
install calibration constants from these hand-held means. Absent-device,
cold-power and MQTT-TEST hardware checks remain separate pending gates.

## Source: IMU_STEP1_VALIDATION.md

# IMU step 1 software validation

## Scope

Original software checks for `02f7c37`, helper `mtk3-imu`, based on
`mtk3-staging` `7034501`. The worker did not push, flash or connect to a
network. Later operator evidence and stream checks are recorded below.
The TEST-only design
and pending procedures are in [IMU bring-up](IMU.md).

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

## Continuous-stream follow-up

After the operator's two hardware windows, the helper adds `IMU_STREAM=1`,
saved setup-register output and a debounced GP20 marker. The decoder, scales,
mag cadence and default TEST sequence are unchanged. See the
[scale audit](IMU.md) for the hardware evidence and limits.

- 130/130 host tests pass. Added marker tests cover bounce, a held button,
  release/rearm and timestamp wrap. The stream-runner test proves only S4
  is called and the report/destructive watchdog test is skipped; priority
  failure also aborts without calling S4.
- All six clean profiles plus the new SMP0 USB/radio-off stream profile have
  zero warnings; strict ARM checks of six changed units and retained SDK link
  proofs pass in all seven. No atomic runtime or unresolved symbols.
- Updated sizes: TEST 63124/672/119604, MQTT-TEST 370592/888/183756;
  IMU stream 56468/672/119372 (text/data/bss bytes). Other sizes are unchanged.
- Stream ELF retains `subsystem4_test`, `imu_hw_mark_init/down` and
  `imu_mark_update`, but none of the other subsystem test functions or
  `subsystem2_watchdog_test`. Normal motion watchdog safety is unchanged.
- Startup/UART proof and compile rejection still pass in all seven profiles.
  The build-profile checker now includes the IMU stream option so objects
  cannot be reused under the wrong mode. Private build copies and generated
  SDK proof ELFs were deleted after validation.
- Subsequent operator hardware run: S4-U03 PASS on 6ed89ad, 16129 reads,
  faults zero, about 530–566 us/max 592 us, gap max 17946 us, no missed wakes,
  six debounced markers. Mag still uses one read each cycle. Accel axis signs
  pass; six-face evidence explains the high static magnitude as Z offset.
  See [scale investigation](IMU.md). No code was tuned.

## Original hardware limits at 02f7c37 handoff

S4-U01 tilt, axis orientation, absent-sensor responsiveness and measured
per-read wall time are **Not run**. S4-U02 cold-power checks are **Not run**.
A 405 us ideal-wire estimate is not a measurement. No calibration, filtering,
hump detection, driving input or MQTT integration is implemented or tested.
