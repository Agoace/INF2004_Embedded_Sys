# GY-511 IMU: observe-only step 2

## Current scope

Step 2 starts from staging `b569adc`. The robot and TEST images now run an
IMU observer. BENCH does not. The observer has no motor commands, terrain
states, hump thresholds, climb events, MQTT fields or KR1 fields. It cannot
change driving or stop the robot on an IMU fault. CR1 packing is deferred.

Bias, pure-core separation, filtering and signed height-integration ideas
are credited to Jaison, `origin/jaison-imu` `b2a6bb6`, especially
`IMU-Telemetry/src/imu_core.c`. Ideas were ported; the reference branch was
not merged. Its event system, terrain thresholds and minimum-height policy
were not ported. The LSM303DLHC has no gyro; no gyro result is claimed.

**Hardware step 2 is Not run.** The earlier S4-U03 PASS validates the older
raw stream and axis signs, not calibration, dynamic pitch or hump height.
See [pending tests](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-not-run).

## Step 2 data path and interface

```text
10 ms kernel cyclic -> IMU task14 -> bounded probe -> pure IMU core
                               ^                    |
    bounded motion count board-+                    v
    vehicle calibration epoch -+          bounded observation board
                                                    |
                                      TEST observer20 -> USB at 50 Hz
```

| File | Role |
| --- | --- |
| [imu_core.h](../../app_program/subsystem4/imu_core.h), [imu_core.c](../../app_program/subsystem4/imu_core.c) | Pure state, calibration, angles, filters and relative height; no SDK/kernel headers |
| [imu_service.c](../../app_program/subsystem4/imu_service.c) | Core-0 task14, own 10 ms cyclic, probe/motion observations and sole snapshot writer |
| [imu_service.h](../../app_program/subsystem4/imu_service.h) | `imu_snapshot_try_read()` and acknowledged operator MARK request |
| [imu_hw.c](../../app_program/subsystem4/imu_hw.c) | Bounded I2C register adapter only; still no SDK runtime/IRQ/clock owner |
| [subsystem4_test.c](../../app_program/subsystem4/subsystem4_test.c) | Observer only; no competing bus reader; TEST-only zero-duty request and USB output |
| [host checks](../../tests/imu_checks.cmake) | Core, task/board, BENCH exclusion, 50 Hz reporter and existing hardware/probe/debounce checks |

The input is identity-mapped accel mg (+X right, +Y forward, +Z up), signed
mean wheel travel and its timestamp, per-wheel stillness and a calibration
permission/epoch. The service uses coherent count deltas, not truncated
integer `travel_mm` or commanded speed:

`ds_mm = 1000 * (delta_left + delta_right) / (2 * 12414)`

The ratio comes from `MC_ROBOT_CONFIG`. Both wheel counts must be unchanged
and both actual duties zero for a stationary candidate. Equal/opposite wheel
motion is not stillness. The existing bounded motion-board read must succeed
and be at most 30000 us old. Duplicate source stamps do not generate another
velocity derivative or height update. Distinct source microsecond intervals
and their fractional remainder form the relative encoder millisecond clock;
this avoids duplicate-time jitter and handles the 32-bit microsecond wrap.

`vehicle_calibration_epoch()` is one atomic load, also without MQTT. It is
odd for the whole existing line-calibration routine, including pauses,
failures, abort and home spin. It changes on entry/exit. No existing motion
command or calibration outcome changes. The IMU reads it before/after input
collection and after core calculation. A transition during calculation
causes one rollback/replay with calibration forbidden, not a retry loop.
This prevents a candidate commit across a higher-priority vehicle action.

Snapshot reads return E_PAR for null, E_OBJ if inactive, or E_TMOUT for a
busy/empty board. Failure leaves the caller's output unchanged. There is one
writer and one bounded copy; no reader waits for the writer. The snapshot
has raw/calibrated acceleration, bias, angles in 0.1 degree, signed travel,
windowed speed, encoder acceleration, height/peak/distance, calibration,
validity, probe faults and timing/sample counters. It is not a wire packet.
Existing health stack-minimum reporting includes the new registered stack;
no heartbeat format or dedicated IMU telemetry field changes.

## Step 2 calibration and settings

| Setting | Value / meaning |
| --- | --- |
| Acquisition | 10 ms, priority14, physical core0; 2048-byte static registered stack |
| Bus / units | Unchanged 400 kHz register polling, 100 Hz HR +/-2 g, 1 mg/LSB; mag still read each cycle |
| Candidate duration | At least 1000 ms and 100 samples of continuous valid stationary input |
| Quiet gate | Per-axis sample standard deviation <=20 mg; initial engineering value, not hardware-tuned |
| Bias | Window mean minus `(0,0,1000)` mg, independently for all axes |
| Magnitude gate | Corrected whole-window envelope inside 900..1100 mg, AFTER candidate bias removal |
| Pitch/roll filter | `alpha = dt_ms / (100 + dt_ms)`; 100 ms time constant |
| Encoder speed | Signed rolling-distance window of at least 100 ms |
| Encoder acceleration | Difference of window velocities at their midpoint times; 100 ms low-pass |
| Invalid time | Zero/rollback time or interval >100 ms breaks derivative/integration history |
| Observer output | IMU_STREAM nominal 50 Hz; ordinary S4 window 30 s with 10 Hz sample lines |

Welford accumulation gives each axis mean and sample standard deviation.
The corrected minimum/maximum axis envelope is a conservative magnitude
check for the entire window; checking only the corrected mean would always
produce 1 g by construction. Neither magnitude nor quietness proves level.
The known raw magnitude near 1375 mg is NOT rejected before bias removal.
No fixed hand-held offset or `1/1.375` gain is installed.

**Boot on level support and remain still for at least one second.** Bias
removal defines that support as level. It cannot distinguish real slope,
mounting tilt or gain error from sensor offset. The operator-approved rule
also refreshes calibration after every further >=1 s stationary window.
Every accepted refresh resets height, peak and distance and clears the angle
filter. Therefore **a one-second stop on a hump can erase the segment and
redefine that slope as level**. A host regression demonstrates this limit.
Keep the robot moving through the hump, and record calibration counts; do
not count an automatic reset as a successful return-to-zero measurement.

Motion, nonzero duty, a calibration epoch change, an invalid sensor/motion
sample or a time gap breaks the stationary window. Noise/magnitude failures
at the end of a complete window increment `calibration_rejects`; interrupted
windows are discarded, not counted as completed rejects. Bias and the
calibrated flag survive faults and MARK. A failed/retry-wait probe observation
is invalid; it is not a new successful sample. Bus faults have separate
NACK, timeout, identity and configuration counters and retain paced retries.

## Step 2 angle and height formulas

Let `a = raw_accel - bias` in mg and `v` be signed encoder speed in mm/s:

- Pitch: `atan2(aY, sqrt(aX*aX + aZ*aZ))`; roll: `atan2(aX, aZ)`.
- Encoder forward acceleration: `(v_k - v_previous) / delta_seconds`,
  followed by the encoder acceleration filter above.
- Compensated pitch: low-pass
  `atan2(aY - forward_accel_mm_s2 / 9.80665, sqrt(aX*aX + aZ*aZ))`.
  One mg is 9.80665 mm/s². Forward-positive acceleration is subtracted from
  +Y; reverse does not reverse the tilt sign.
- Height: `height_mm += sin(compensated_filtered_pitch_rad) * ds_mm`.
  Peak is the largest signed relative height since calibration/MARK, with
  initial peak zero. It is not a threshold, event or maximum accepted hump.
- Distance is signed mean travel relative to the last calibration/MARK;
  it is not absolute path length. It can recover encoder distance after a
  source gap, while height deliberately omits the missing interval.

Invalid samples do not consume distance with stale pitch. They break the
integration/derivative baseline but do NOT reset the accumulated height or
peak. The first later valid encoder observation establishes a new baseline.
Only accepted calibration and operator MARK reset height/peak/distance.
Angles are reported as integer tenths of a degree, truncated toward zero;
internal filtering and integration use radians without that quantization.

This is a relative estimate, not an absolute height sensor. Encoder slip,
gear lash, turns, lever-arm acceleration and filter delay remain errors.
There is no gyro or independent dynamic attitude reference. Window midpoint
times correct derivative scale but do not remove its delay relative to the
current accelerometer sample. Even 0.1 degree residual pitch gives about
1.75 mm height error per metre. No ground-height clamp hides that drift.

## Step 2 operator checks

Build in `build_make` (do not flash without operator approval):

```sh
make SMP=0 WIFI=none NET=none TEST=1 IMU_STREAM=1 -j4 --output-sync=target
```

GP0/GP1 are reserved for I2C in robot and TEST. Use USB, `CRASH_REPORT=0`,
and remove the UART adapter. The existing compile guard now covers both
image roles. No additional template deviation is needed; the count stays 8.

IMU_STREAM skips other subsystem tests and the destructive watchdog test.
It requests zero duty once through `motion_stop()`; motion remains the only
PWM writer and the normal motion watchdog remains active. The driver sets
both forward/reverse PWM levels to zero: **COAST, not active braking**
([driver](../../app_program/subsystem2/drv_motor_hw.c)). A compare update
within 50 us is not a physical stop-time guarantee. Hand pushing is possible;
zero duty alone is not proof of stillness.

The TEST observer has its own 20 ms cyclic and prints one sample per wake.
`t_ms` is the sample millisecond timestamp; `raw_mg`/`cal_mg` are raw/bias-
removed acceleration. `raw_ddeg`, `pitch_ddeg`, `roll_ddeg`, `filt_ddeg` and
`corrected_ddeg` separate raw, calibrated, filtered and compensated angles.
Travel, distance, height and peak print in mm; speed and forward acceleration
print in mm/s and mm/s². Each line includes calibration and fault status.
One-second diagnostics show bias, standard deviations, calibration/reject
counts, probe counters, timing and registered-stack high-water marks.

GP20 remains a 30 ms debounced active-low input. MARK is requested by the
observer and applied by the sole IMU writer on its next tick. `[S4] MARK n`
prints only after the snapshot acknowledges it. Holding the button does not
repeat. The angle filter and bias do not reset on MARK. Formatting/USB work
stays at TEST priority20, never inside task14. Nominal output frequency is
not a guarantee that a slow/unplugged USB host receives every line.

Use [S4-U04/U05/U06](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-not-run).
The 10 mm, 195 mm long bring-up hump is not the course maximum. No 7-degree
CLIMB or 15 mm minimum-height rule is imported.

## Step 2 software results

Local-only validation uses public build fixtures and ARM GNU 14.3.rel1;
there was no push, flash, broker or hardware run. All **142 host tests**
pass. The core tests include known +375 mg Z bias, noise and post-bias
magnitude rejection, moving/equal-opposite wheel inhibit, phase transitions,
faults, refresh, MARK, angle/filter signs, forward/reverse/braking correction,
duplicate stamps, gaps, time wrap and the stop-on-slope reset limit. Task
checks force a phase transition at the would-be commit, a torn board copy,
source failures/staleness, signed count wrap and object/start/wake failures.
The reporter test verifies 50 sample lines per 50 x20 ms wakes and one
acknowledged debounced MARK. A moving-source MARK test checks exact zero
height/peak/distance in its acknowledged snapshot. Clang undefined-behavior
and float-cast sanitizer runs also pass for core/task tests. These are
doubles, not physical acceptance.

| Synthetic constant-speed case | Peak mm | Final height mm |
| --- | ---: | ---: |
| 5 mm / 195 mm surface-distance profile | 4.734 | -0.000 |
| 10 mm / 195 mm profile | 9.469 | -0.001 |
| 20 mm / 195 mm profile | 18.952 | -0.007 |

These profiles use 100 mm/s, a 5 mm crest and a settled moving lead-in before
MARK, followed by enough flat travel for the filters to settle. They are
not replayed sensor/encoder measurements or proof for a physical hump.

| Quantized flat start/cruise/brake/rest case | False peak mm | Largest absolute error / final height mm |
| --- | ---: | ---: |
| +/-100 mm/s, +/-500 mm/s² ramps | 0.405 | 0.431 / -0.431 |
| +/-460 mm/s, +/-500 mm/s² ramps | 1.590 | 7.994 / -7.994 |

Both directions give the listed errors. Encoders are quantized at 12414
counts/m; rest lasts 0.9 s, so no refresh erases the residual. These tests
expose compensation delay/error, especially the roughly 8 mm final error at
460 mm/s. They do NOT establish dynamic height accuracy. Keep all physical
height cases Not run until measured traces support an acceptance decision.

| Clean firmware profile | text | data | bss |
| --- | ---: | ---: | ---: |
| Default SMP1, USB/Wi-Fi | 326640 | 760 | 129592 |
| Robot SMP0, WIFI/NET none | 79716 | 688 | 123056 |
| BENCH SMP0, WIFI/NET none | 41982 | 148 | 95460 |
| MQTT SMP0 | 390892 | 904 | 187612 |
| TEST SMP0, WIFI/NET none | 72776 | 688 | 122476 |
| MQTT-TEST SMP0 | 380016 | 904 | 186636 |
| IMU_STREAM SMP0 | 66352 | 688 | 122244 |

All seven clean builds use `-j4 --output-sync=target`, with zero warnings.
Nine changed/related C units pass strict ARM compilation in every profile:
core, service, subsystem4, TEST reporter, hardware, probe, decode, vehicle
and static task registry. Flags are `-Wall -Wextra -Wconversion -Wshadow
-Wformat=2 -Werror`. All seven retained SDK link proofs pass; no unresolved
or `__atomic_*` runtime symbols. Startup disassembly still has no GP0/GP1
UART pin-function loop. UART-conflict compile rejection is checked separately.
The normal robot/MQTT and TEST images retain IMU hardware/probe; BENCH does
not. No SDK runtime, interrupt or clock owner was added.

MQTT static RAM increases **3732 B**, including the **2048 B static stack**;
see [RAM evidence and CPU estimate](../kernel/RAM_BUDGET.md#imu-step-2-measured-delta).
Stack depth and real task CPU cost remain hardware gates, not size results.

## Historical steps 0 and 1

The three source records below are retained in full. Their original revision
scopes and historical Not-run statements remain attached to their evidence.
Their TEST-only/no-calibration descriptions apply to those old revisions,
not the current observe-only step 2 above.

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
