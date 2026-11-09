# GY-511 IMU: roll-only observer, step 3 hump report

## Current scope

Step 3 adds a report-only hump detector on top of step 2d (`0207b54`).
It counts humps from the roll height during a run and reports the highest
one: USB line at run end, GP20 DUMP line, live MQTT event kind 9 and
summary v3. It never changes motor commands or navigation. See
[step 3 hump report](#step-3-hump-report). Step 3 H1 is hardware PASS on
`1480b37` (STREAM, three RIGHT-wheel drives). H0/H2-short on `d51d27b`
counted correctly but LINE height over-read by 29..55 %. Robot IMU stack
free was 592 B. Diagnostic C adds peak context, not a correction; its
hardware result and H3..H9 remain **Not run**.

Step 2d extends `8c5d679` (step 2c), which extends `75ff329` (step 2b),
originally based on `b569adc`. Step 2d changes ONLY the accelerometer input:
400 Hz FIFO samples are averaged into each 10 ms observation. See
[step 2d FIFO input](#step-2d-fifo-input). Robot and
TEST run an IMU observer; BENCH does not. The observer has no motor
commands and no KR1 fields. It cannot change normal driving. The explicit exception is a
separate operator-triggered GP21 straight-drive job in IMU_STREAM only;
its stop policy is described below. CR1 packing is deferred.

Bias, pure-core separation, filtering and signed height-integration ideas
are credited to Jaison, `origin/jaison-imu` `b2a6bb6`, especially
`IMU-Telemetry/src/imu_core.c`. Ideas were ported; the reference branch was
not merged. Its event system, terrain thresholds and minimum-height policy
were not ported. The LSM303DLHC has no gyro; no gyro result is claimed.

**Step 2d floor hardware PASS on `0207b54` for the 10 mm RIGHT-wheel hump
only:** driven peaks 10.93/10.85/10.44 mm, flat roll height sd
0.50..0.68 mm. See [step 2d hardware result](#step-2d-hardware-result).
Step 2c hardware ran on `8c5d679`:
driven hump peaks 16.8/13.1/12.4 mm over the 10 mm hump, so it is NOT a
height PASS. See [step 2c results](../../testing/subsystem4/TEST_LOG.md#step-2c-hardware-on-8c5d679).
Step 2b static RIGHT-wheel/lifted-drive
results are partial: distance/button stop PASS, driven height estimator
unusable because pitch vibrates. The new-cable session had no resets;
the cause of the earlier three resets is still unproven. Offline analysis
is not firmware height acceptance. See the current model/procedure below.
The CURRENT primary is roll-only; the step2b pitch-dependent model remains
diagnostic, not a moving-height acceptance result.
See [S4 results and pending tests](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-and-step-2b).

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
| [imu_hump.h](../../app_program/subsystem4/imu_hump.h), [imu_hump.c](../../app_program/subsystem4/imu_hump.c) | Step 3 pure hump detector and run summary; no SDK/kernel headers |
| [imu_service.c](../../app_program/subsystem4/imu_service.c) | Core-0 task14, own 10 ms cyclic, probe/motion observations and sole snapshot writer |
| [imu_service.h](../../app_program/subsystem4/imu_service.h) | `imu_snapshot_try_read()` and acknowledged operator MARK request |
| [imu_hw.c](../../app_program/subsystem4/imu_hw.c) | Bounded I2C register adapter only; still no SDK runtime/IRQ/clock owner |
| [subsystem4_test.c](../../app_program/subsystem4/subsystem4_test.c) | Observer only; no competing bus reader; TEST-only zero-duty request and USB output |
| [host checks](../../tests/imu_checks.cmake) | Core, task/board, BENCH exclusion, 50 Hz reporter and existing hardware/probe/debounce checks |

The input is identity-mapped accel mg (+X right, +Y forward, +Z up), signed
mean wheel travel, left-positive encoder yaw and their timestamp, per-wheel
stillness and a calibration permission/epoch. The service uses coherent count deltas, not truncated
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
| Bus / units | 400 kHz register polling; step 2d: 400 Hz HR +/-2 g FIFO, mean of each 10 ms batch, 1 mg/LSB; mag still read each cycle |
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
also considers refresh after every further >=1 s stationary window.
Step 2b accepts refresh only when the window mean, corrected with the
CURRENT bias, differs from level by LESS THAN the configurable 1 degree.
Larger tilt rejects refresh and increments both reject counters; a block
or stopped slope above that tilt must not become level. Boot calibration is unchanged.
Accepted near-level refresh resets the OLD integral h/peak/distance and
angle/quality windows, but preserves all NEW support peaks until MARK. The guard is not independent proof of level and can follow
small incremental changes. Record calibration counts; do not count a reset
as a successful return-to-zero measurement.

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
- Left-positive yaw: `delta_yaw = (ds_right - ds_left) / track`.
  Windowed yaw velocity/acceleration use the same timestamps as speed.
- Fake lateral acceleration: `aX_fake = -v*omega - forward_mm*alpha_yaw`.
- Fake forward acceleration: `aY_fake = dv/dt - forward_mm*omega*omega`.
  All lengths/speeds here use mm. Left turning at positive forward speed
  accelerates toward LEFT, or -X. The sensor ahead of the axle also sees
  -X angular-acceleration and -Y centripetal terms. Host tests pin both
  turn signs, both spins and an angular ramp; no silent sign inversion.
- Compensated vector: `cX=aX-aX_fake/9.80665`,
  `cY=aY-aY_fake/9.80665`, `cZ=aZ`. One mg is 9.80665 mm/s².
  Low-pass `atan2(cY,sqrt(cX*cX+cZ*cZ))` and `atan2(cX,cZ)` for pitch/roll.
  Reverse does not reverse the tilt sign. Sensor HEIGHT130 mm is a retained
  setting only: it does not enter this planar yaw model. Without a gyro,
  pitch/roll angular acceleration and lever-arm effects remain unknown.
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
It initially requests zero duty through `motion_stop()`; motion remains the only
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

Use [S4-U04/U05/U06](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-and-step-2b).
The 10 mm, 195 mm long bring-up hump is not the course maximum. No 7-degree
CLIMB or 15 mm minimum-height rule is imported.

## Step 2 software results at dfade14 (historical)

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

## Step 2b support hypotheses and peak quality (now diagnostic)

The course operator confirmed a hump under the RIGHT drive wheel only;
left wheel/caster remain on the floor. Positive roll is right-side-up,
negative pitch is nose-down. The earlier slope integral remains a secondary
long/full-width-slope diagnostic, not the primary one-wheel estimate.

| Alternative hypothesis | Signed height estimate |
| --- | --- |
| Caster on flat | `zR=-WB*sin(pitch)+(T/2)*sin(roll)`; PRIMARY right wheel |
| Caster on flat | `zL=-WB*sin(pitch)-(T/2)*sin(roll)`; consistency/secondary |
| Both drive wheels on flat | `zC=+WB*sin(pitch)`; caster-lift diagnostic |

These are alternative reference assumptions, NOT simultaneous absolute
contact heights. Common vertical translation is unobservable from tilt;
a small residual alone does not prove the caster is on the floor.
With pitch -5.0 and roll +7.1 degrees, WB80/T111 gives
zR13.832, zL0.113, zC-6.972 mm. The negative zC does not mean that the
caster is below ground; its wheels-flat hypothesis is not true in that pose.
No height is clamped. No hypothesis/detector chooses a driving action.

[imu_support.c](../../app_program/subsystem4/imu_support.c) is pure and
host-tested for left/right/both/caster blocks and alternate geometry.
[imu_settings.h](../../app_program/subsystem4/imu_settings.h) gives editable
positive geometry defaults: effective track111 mm (not rough115), WB80 mm,
sensor60 mm ahead/130 mm high, refresh tilt1 degree and consistency2 mm.
The offline fit used WB79 and remains separately labelled in the test log.

Instant zR/zL/zC use compensated filtered pitch AND roll. Sines are computed
once per axis; the pitch sine is reused for the existing integral. Stream
also shows all three peaks since MARK, `left_residual_mm=abs(zL)`, smoothed
zR and peak eligibility. The PRIMARY zR peak uses a full 10-sample boxcar
average (nominal100 ms at100 Hz), and ALL ten valid samples must have
abs(zL)<=2 mm. This configurable tolerance is a model-consistency quality
check only, not a terrain-height threshold. Raw heights, secondary peaks
and slope integral remain ungated diagnostics. Window warm-up, invalid
input, gaps and MARK cannot leak an old average into a new peak. MARK
clears support peaks/window, not the current geometric heights or bias.
Accepted near-level calibration preserves support peaks; their only resets
are MARK and boot. GP21 START does not reset them (run-start reset is later
step3 work). The old integral retains its calibration reset. Invalid
samples preserve existing peaks but invalidate the averaging window.

## Step 2b procedure at75ff329 (historical)

1. Use the same IMU_STREAM build command above. Flash only with approval.
   Boot on known level support, USB only, GP0/1 free of UART wiring.
2. Inspect RIGHT-wheel blocks first; then left, both wheels and caster.
   Keep the body still long enough to check that a >1 degree tilt rejects
   refresh rather than installing a new level. Save bias/reject counts,
   signed alternative heights, residual, eligibility and smoothed peaks.
3. Resolve unexplained resets before powered floor trials.
   **VIN is required for driving; software does not sense or prove it.**
   First lift both wheels, restrain the chassis, keep an independent way to
   remove VIN, and check the stop behavior before floor work. Zero duty is
   coast, not a brake or an immediate physical stop guarantee.
4. GP20 MARK is unchanged. A debounced GP21 press starts equal wheel-speed
   requests of100 mm/s through the motion owner, refreshed every20 ms.
   Stop at600 mm signed mean encoder travel OR10 s, whichever occurs first.
   A button held across boot must be released/repressed before driving.
   A second press, stale/unavailable IMU/motor data, missing calibration,
   sensor/motor fault (including invalid magnetometer data), missed
   observer wake or command failure stops the job.
   Cumulative probe faults prevent start until reboot. No height/consistency
   value affects this job. MARK/refresh cannot extend its distance budget.
5. A separate stop-pending latch retries `motion_stop()` then checked
   `motion_estop()` each observer tick if both fail. Starts remain forbidden
   until stop is confirmed; timeout cleanup also retains the cyclic while
   retrying. A warning says to remove VIN. Permanent API or hardware failure
   cannot prove a physical stop; do not rely on this button as an independent
   safety circuit. The normal motion watchdog/stale policy remains owned
   by motion, not the IMU task.
6. After the lifted check, make three driven RIGHT-wheel hump passes with
   MARK and full timestamped traces. Stop/start cases, geometry, slip, yaw
   correction delay and the three unexplained earlier resets remain gates.
   Report actual new work/core timing and stack high-water; the older
   13..14% wall-work measurement is NOT a step-2b bound.

New [pure drive job](../../app_program/subsystem4/imu_drive.c) is used ONLY
by the IMU_STREAM observer, not normal robot or ordinary TEST. Host cases
cover distance/time/wrap, second press, stale/missing/faulted inputs,
command failure and both stop APIs failing before recovery. All powered
and static cases were originally Not run at75ff329; see the later
[partial hardware results](../../testing/subsystem4/TEST_LOG.md#step-2b-hardware-on75ff329).

## Step 2b software validation at75ff329 (historical)

157 host cases pass, including pure support/drive, left/right turns and
spins, angular lever-arm sign, stopped-block refresh rejection, full-window
consistency gating, MARK/fault window reset, support-peak persistence across
level refresh, distance/timeout/button/fault
stops and retry after both owner stop APIs fail, including observer-timeout
cleanup. An isolated single-sample height spike confirms the boxcar peak
is an average, not a single-sample maximum. The original slope-
integral synthetic/flat-error regressions remain unchanged. Host results
are not a physical accuracy or stop-latency claim.

| Clean profile | text | data | bss |
| --- | ---: | ---: | ---: |
| Default SMP1 USB/Wi-Fi | 328032 | 760 | 129880 |
| Robot SMP0 no radio | 81108 | 688 | 123344 |
| BENCH SMP0 no radio | 41982 | 148 | 95460 |
| MQTT SMP0 | 392292 | 904 | 187908 |
| TEST SMP0 no radio | 74620 | 688 | 122772 |
| MQTT-TEST SMP0 | 381868 | 904 | 186924 |
| IMU_STREAM SMP0 no radio | 69696 | 688 | 122540 |

Clang AddressSanitizer, undefined-behavior and float-cast checks pass for
core, service, support, drive and the stream including every fault scenario.
Recorder67/dashboard180 remain green. These host checks do not prove real
motor stopping or target timing.

All seven clean `-j4 --output-sync=target` builds are warning-free. Eleven
related units/profile pass strict ARM flags (77 compilations), seven SDK
retained-link proofs pass, and no unresolved or atomic-runtime helpers
appear. The template deviation count remains8 against c3b0954. UART pin
conflicts are still compile-time errors; no SDK clock/IRQ/runtime owner is
added. Public fixtures and generated firmware/proof copies are removed.
See [current RAM/CPU evidence](../kernel/RAM_BUDGET.md#imu-step-2b-measured-delta).
New physical work_us, stack high-water, powered fault-stop timing and
static/driven accuracy were originally Not run; later step2b observations
are recorded in the test log, not inferred from these host results.

## Step 2c primary model

On a RIGHT-wheel hump, with left wheel AND caster on the floor:

- Primary height: `hR = T*sin(filtered_roll)`, effective T=111 mm.
- Expected pitch: `-asin(hR/(2*WB))`, WB=80 mm.
- Pitch consistency is diagnostic only. A configurable WIDE 10-degree
  tolerance sets a flag and the ratio matches/valid samples since MARK.
  Bad pitch NEVER gates the primary height, windows, peak or plateau.
- Use the bias-corrected roll LPF, not the yaw-corrected roll, to match the
  operator's offline roll-only evidence. The corrected angles and the
  entire step2b zR/zL/zC model remain diagnostics. Its old `eligible` bit
  describes ONLY the old pitch-dependent peak, not the new primary.
- This remains a known-contact hypothesis. Roll noise, turning acceleration,
  body flex and hand lifts can create false height; no contact detector or
  terrain event is added. No height or quality value drives the robot.

[imu_roll.c](../../app_program/subsystem4/imu_roll.c) is pure. It uses the
existing 100 ms roll LPF, a FULL 10-sample mean for the smoothed positive
peak, then a FULL trailing 20-sample median of mean outputs for the
`plateau` value (maximum median since MARK). This is a robust statistic,
not proof of a physical flat plateau. Instant signed height is not clamped.
Both maxima start at zero, persist through near-level calibration refresh,
and reset only at MARK/boot, not GP21 START. MARK also clears window and
quality counters. Invalid data/gaps/refresh clear readiness/windows but
preserve peaks; stale numeric averages must not be used when readiness is
false. Geometry and the wide tolerance are in imu_settings.h; median length
is configurable up to31 samples in imu_roll.h. Default20 is approved.

### Filter response and window choice (fixtures only)

100 Hz, dt=10 ms, alpha=10/(100+10). The roll LPF's low-frequency group
delay is100 ms; the 10-sample mean adds45 ms, total about145 ms (14.5 mm
at100 mm/s). Median is nonlinear: it has no universal group delay. For a
monotonic signal, its additional central-sample delay is about70/95/150 ms
for15/20/31 samples. These values are NOT measured hardware timing.

The actual test hump is10 mm high/195 mm long:95 mm ramp each side and
5 mm crest, ramp angle about6 degrees. Above80% lasts430 ms, above90%
240 ms at100 mm/s. The alternative fixture extends the crest to50 mm,
a500 ms flat top. Fixtures transform height to roll, apply the actual LPF
recurrence, mean10, then median. No hidden amplitude correction is applied.

| Median window | Triangle median max / bias mm | 500 ms flat-top median max mm | Flat WHITE sd2 mm: median max mm | Flat 400 ms sine sd2 mm: median max mm | Step median t90 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| 150 ms /15 | 9.364 /-0.636 | 9.979 | 1.188 | 2.173 | 370 |
| **200 ms /20 default** | **9.310 /-0.690** | **9.974** | **1.098** | **1.797** | **390** |
| 310 ms /31 | 9.143 /-0.857 | 9.961 | 0.890 | 0.975 | 450 |

Mean peak without median: triangle9.426, flat-top9.986 mm. Mean step
response t10/t50/t90=50/130/300 ms with a warmed window. Median startup
needs10+N-1 consecutive valid samples (290 ms for default20); the step
fixture starts with level/warmed windows, not cold initialization.

The noise fixtures inject sd2 mm AFTER the roll LPF (the observed driving
roll sd1.0..1.2 degrees is about2 mm). WHITE noise uses a fixed seed over
10 s; the sine has400 ms period. Neither spectrum was measured, and these
maxima are NOT expected real noise bounds. Mean-only white maximum1.717 mm.
A slow oscillation survives longer windows; max-since-MARK also grows with
observation time. Median20 trades a small additional crest bias over15 for
more noise suppression, with less crest attenuation/delay than31. The
measured noise spectrum is unknown, so retain BOTH mean peak and median
max for the next hardware run. A single20 mm raw spike becomes a2 mm mean
peak and a1 mm median20 peak; this is smoothing, not perfect artifact veto.
No fixture result establishes physical accuracy, and no fit corrects its bias.

## Step 2c procedure at 8c5d679 (historical)

1. Build IMU_STREAM as above. Boot LEVEL, USB console, GP0/1 clear of UART.
   GP7 now connects GY-511 accelerometer INT1; GP6 connects magnetometer
   DRDY and is unused. Grove7 also exposes GP28 barcode analog: leave any
   barcode digital output disconnected from GP7. Do not change GP28 mux.
2. Probe writes/readbacks CTRL_REG3_A=0x10 (I1_DRDY1), CTRL_REG6_A=0x00
   (high-active/default mode), retaining100 Hz/BDU/HR/2 g. GP7 is a SIO
   input with weak PULL-DOWN; the sensor drives it, no external pull-up is
   requested. This exposes an unwired LOW input. No GPIO IRQ/edge callback,
   FIFO, SDK clock/runtime or ninth template deviation is introduced.
3. Read path samples GP7 immediately before and after the SIX accel bytes,
   BEFORE magnetometer transfer. Successful accel reads count HIGH->LOW
   matches; other pairs count mismatches, with separate HIGH-before and
   LOW-after totals. Failed accel transfers do not count checks. A mag fault
   may occur AFTER a completed check. Levels are the last attempt. INT1 is
   only a wiring/freshness diagnostic: mismatches neither discard samples
   nor drive/stop the robot. Poll phase/ODR jitter can cause mismatches even
   when wiring works. Transition behavior is **Not run** on hardware here.
4. Keep the approved VIN/lifted-first GP21 safety procedure above. Do one
   flat600 mm drive, then THREE right-wheel hump drives, LEVEL start and
   GP20 MARK BEFORE EACH. Do not lift the body after MARK. Inspect new
   `roll_zR/avg/peak/plateau`, readiness, pitch quality/ratio, old full model,
   source timestamps and INT1 counters. Label each drive and object.
5. `[S4_VIB]` reports per-drive Welford SAMPLE std/mean of corrected pitch
   and filtered roll, accumulated from distinct valid observer snapshots
   at50 Hz, with0.1-degree input quantization. Reset on accepted GP21 start,
   not MARK; report periodically and at stop. The command window includes
   startup but not post-stop coasting. Hump pose changes also contribute:
   this is whole-drive angle spread, not isolated vibration variance. It is
   not the100 Hz sensor population or a terrain decision.
   Timestamped raw/filtered/corrected angles remain in the50 Hz stream for
   offline analysis. No embedded dominant-frequency guess is added.
6. Frequencies at/above25 Hz may alias in this observer stream. Whether
   that explains the driving noise remains OPEN. Only if actual flat data
   requires it, separately approve step2d (400 Hz/FIFO/INT1 IRQ, likely
   template deviation9); DO NOT start it as part of this change.
7. Record actual new work/core/read timing, gaps and both task stack bounds.
   Step2b work1562 us average/max2530 us is NOT a step2c measurement.
   Physical roll-only accuracy, noise, delay and stopping: see
   [step2c results](../../testing/subsystem4/TEST_LOG.md#step-2c-hardware-on-8c5d679).

## Step 2c software validation

160 host cases pass, including median15/20/31 fixtures, bad pitch that
NEVER gates the new peak, signed heights, MARK/invalid readiness, support
peak persistence across refresh, Welford sample std, all four INT1 pairs
and configuration/readback/failure exits. Recorder67/dashboard180 remain
green. New physical measurements remain Not run.

| Clean profile | text | data | bss |
| --- | ---: | ---: | ---: |
| Default SMP1 USB/Wi-Fi | 330116 | 760 | 130280 |
| Robot SMP0 no radio | 83196 | 688 | 123744 |
| BENCH SMP0 no radio | 41982 | 148 | 95460 |
| MQTT SMP0 | 394372 | 904 | 188308 |
| TEST SMP0 no radio | 77184 | 688 | 123172 |
| MQTT-TEST SMP0 | 384432 | 904 | 187324 |
| IMU_STREAM SMP0 no radio | 73084 | 688 | 122964 |

Clang AddressSanitizer/undefined-behavior/float-cast checks pass for core,
service, roll15/20/31, drive/stats, probe, register adapter and every stream
fault scenario. GPIO tests include weak pull-down and both INT1 levels.

Seven clean warning-free builds use `make -j4 --output-sync=target`.
Twelve related units/profile pass strict ARM flags (84 compilations).
Seven retained SDK proofs pass with no unresolved/atomic-runtime helpers.
UART pin guards and startup pin safety remain enforced. Template deviations
stay8 against c3b0954; no clock/IRQ/SDK runtime owner is added. See the
[current RAM/stack evidence](../kernel/RAM_BUDGET.md#imu-step-2c-measured-delta).
These results are software gates, not physical peak accuracy or CPU bounds.

## Step 2d FIFO input

### Why step 2d

Step 2c hardware on `8c5d679` read driven peaks 16.8/13.1/12.4 mm over the
10 mm RIGHT-wheel hump. On flat floor, roll-only height noise had sd
1.6..2.7 mm and the 200 ms median gave false peaks up to 5.7 mm. An
offline spectrum of the raw 50 Hz pitch/roll on flat floor put about 50 %
of the variance in 12..25 Hz, piled at the Nyquist limit. This is LIKELY
aliased motor/gear vibration; it is not proven. The INT1 poll also showed
about 3.5 % of reads with no new sample: a 100 Hz poll against a 100 Hz
sensor clock that drifts reads the same sample twice.
[Results](../../testing/subsystem4/TEST_LOG.md#step-2c-hardware-on-8c5d679).

### What changed

| Item | Step 2c | Step 2d |
| --- | --- | --- |
| Accel rate | 100 Hz, one sample read per 10 ms | 400 Hz into the 32-sample FIFO, stream mode |
| Per 10 ms | Read OUT registers once | Read FIFO_SRC once, then ONE burst of all n new samples, n<=8 (48 B) |
| Value to core | That one sample | Mean of the n samples (boxcar anti-alias filter, decimation to 100 Hz) |
| Duplicates | About 3.5 % of reads | Not possible: a FIFO read removes the sample |
| INT1/GP7 | DRDY polled before/after the read | Retired; CTRL_REG3_A=0x00, GP7 stays a pulled-down unread input |
| Core, filters, height, peaks | Unchanged | Unchanged; still a 10 ms cyclic, task14 |
| Magnetometer | 75 Hz continuous, read each cycle | Unchanged; also read in empty/reset cycles |

[imu_fifo.c](../../app_program/subsystem4/imu_fifo.c) is pure: FIFO_SRC
decode, little-endian word decode and the HR mean (each left-aligned word
divided by 16, sign-corrected, then averaged as a float in mg).
[imu_probe.c](../../app_program/subsystem4/imu_probe.c) does the bounded
read. The service feeds the mean to the unchanged core. No GPIO IRQ, no
SDK runtime, clock or IRQ owner, and no template change: the deviation
count stays 8.

### Register setup

The probe writes in this order: CTRL_REG1_A=0x00 (power down),
FIFO_CTRL_REG_A=0x00 (bypass clears the FIFO), CTRL_REG2_A=0x00,
CTRL_REG3_A=0x00, CTRL_REG4_A=0x88 (BDU, HR, +/-2 g), CTRL_REG5_A=0x40
(FIFO_EN), CTRL_REG6_A=0x00, the three unchanged magnetometer registers,
FIFO_CTRL_REG_A=0x80 (stream mode, no watermark use) and last
CTRL_REG1_A=0x77 (400 Hz, normal mode, XYZ on). It reads back every
setting after the bypass write and refuses a mismatch. `[S4_FIFO]` prints
the saved `ctrl1`, `ctrl5` and `fifo_ctrl` readbacks: expect 77, 40, 80.

### Source facts: datasheet, not hardware-verified

Sources: ST LSM303DLHC datasheet DocID018771 Rev 2 (November 2013), and
ST AN3308 Doc ID 18198 Rev 1 (January 2011). AN3308 is the LIS3DH
application note. The LSM303DLHC accelerometer has the same FIFO registers,
but its datasheet does not describe the FIFO behaviour. Thus the AN3308
rows are RELATED-DEVICE facts. No PDF is committed.

| Fact | Source | Status |
| --- | --- | --- |
| ODR 0111 = 400 Hz normal/low-power | Datasheet 7.1.1, Table 20 | Datasheet |
| HR (normal) mode bandwidth ODR/9, turn-on 7/ODR | Datasheet 3, Table 8 | Datasheet; filter shape not published |
| FIFO_EN is CTRL_REG5_A bit 6 | Datasheet 7.1.5, Tables 28/29 | Datasheet |
| FM[1:0]=10 is stream mode, 00 bypass | Datasheet 7.1.12, Tables 36/37/38 | Datasheet |
| FIFO_SRC_REG_A = WTM, OVRN_FIFO, EMPTY, FSS[4:0] | Datasheet 7.1.13, Table 39 | Datasheet |
| SUB bit 7 = 1 enables address auto-increment | Datasheet 5.1.1 | Datasheet |
| Register map 0x20..0x2F | Datasheet 6, Table 17 | Datasheet |
| FIFO holds 32 sample sets; OUT registers give the oldest set | AN3308 9.1, Tables 33/34 | Related device |
| FSS = unread count; EMPTY=1 at 0; at 32 unread OVRN=1 and FSS=31 | AN3308 9.2.3, Tables 37/38 | Related device |
| Bypass clears the FIFO; FIFO_SRC then reads 0x20 | AN3308 9.3.1 | Related device |
| Stream mode overwrites the oldest set when full | AN3308 9.3.3 | Related device |
| A burst read rolls back from 0x2D to 0x28 | AN3308 9.5 | Related device |
| I2C cost: about 29 clocks to start plus 9 per byte | AN3308 9.5 | Related device |
| LIS3DH FIFO stores 10-bit samples | AN3308 9.1 | Related device; **hardware-confirmed on 0207b54**: every LSM303DLHC raw word is a multiple of 64 |

Only the FIFO width is separately hardware-confirmed. The step 2d floor
run (readbacks 77/40/80, 3.913 samples per poll, zero resets, plausible
captures) is consistent with the other rows but does not test each one.
The 10-bit FIFO means one raw count is 4 mg; the batch mean of about 4
samples gives a finer, fractional mg value.

### Read path and the 2 ms deadline

Each ready cycle: `imu_hw_cycle_begin()`, one FIFO_SRC read, at most one
burst of `6*n` bytes from 0x28 with auto-increment (sub-address 0xA8),
one 6-byte magnetometer read, `imu_hw_cycle_end()`. One 2 ms deadline
covers the WHOLE cycle. A transfer does not start after the deadline; a
running transfer stops and the controller is disabled. A late cycle is a
counted timeout fault: the observation is invalid and the existing paced
reconfiguration (with a FIFO reset by bypass) follows one second later.
In IMU_STREAM any probe fault stops the GP21 drive and blocks new drives
until reboot. A deadline miss is thus visible, never a silent late value.

The read is single-pass. A sample that arrives during the burst stays in
the FIFO and starts the NEXT mean, so there is no gap and no duplicate.
The hardware layer keeps at most 8 read commands in flight (TX FIFO <=9
words, RX FIFO <=8 bytes) and drains RX while it queues a long burst.

| Case | Action | Counter |
| --- | --- | --- |
| n=0 with EMPTY | No observation: core not stepped, timestamp kept, MARK waits one tick | `empty` |
| n=1..8 | Mean of n to core; `reads`, `batches`, `samples`, `min`/`max` | normal |
| n>8 (backlog, e.g. a wake later than about 20 ms) | Bypass then stream reset; invalid observation | `backlog`, `reset` |
| OVRN (32 unread, data lost) | Same reset; invalid observation | `overrun`, `reset` |
| EMPTY with n>0, or no EMPTY with n=0 | Same reset; invalid observation | `bad`, `reset` |
| Bus fault or deadline | Counted fault; invalid; reconfigure after 1 s | `faults`, `timeout`/`nack` |

An invalid observation breaks the angle/height windows like any other
invalid sample; peaks are kept. During a GP21 drive, the observer stops
the drive if it sees an invalid snapshot (existing rule). A partial or old
mean never reaches the core: the mean is used only after the whole cycle,
including the magnetometer read and the deadline check, succeeded.

Bus-time ESTIMATE (AN3308 9.5 clock counts at 400 kHz, plus about 57 us
per transfer from the step 1 measurement of 530 us for two 6-byte reads):

| n | Ideal bus us | Estimate us |
| ---: | ---: | ---: |
| 4 (nominal) | 915 | about 1090 |
| 5 | 1050 | about 1220 |
| 8 (largest accepted) | 1455 | about 1630 |

These are not measurements. Preemption by tasks 10..13 adds to the wall
time (step 2b read max 705 us versus mean 564 us). With n=8 the margin
to 2 ms is only about 0.37 ms. The task busy-waits on the bus, so the read
part of `work_us` grows from about 0.56 ms to about 1.1 ms per 10 ms
(about +5 % of core 0 at priority 14). Measure `read_us`, `work_us`, gaps
and the IMU/TEST stack high-water on hardware.

### Anti-alias effect (fixtures only)

A 4-sample boxcar at 400 Hz has gain `|sin(pi*f/100)/(4*sin(pi*f/400))|`.
It is zero at 100 and 200 Hz, which fold to 0 Hz after 100 Hz decimation.
[test_imu_fifo.c](../../tests/test_imu_fifo.c) drives pure sines through
the mean and checks the folded amplitude against this formula:

| Input | Folds to at 100 Hz | One sample per 10 ms | Mean of 4 at 400 Hz |
| ---: | ---: | ---: | ---: |
| 37 Hz | 37 Hz (no fold) | 1.000 | 0.801 |
| 113 Hz | 13 Hz | 1.000 | 0.128 |
| 137 Hz | 37 Hz | 1.000 | 0.261 |
| 187 Hz | 13 Hz | 1.000 | 0.100 |

The single-sample column omits the sensor's own filter. Table 8 gives HR
bandwidth ODR/9: about 11 Hz at 100 Hz ODR, about 44 Hz at 400 Hz ODR.
Thus step 2d removes folds from near 100/200 Hz but lets MORE 11..44 Hz
content into the raw 100 Hz angles than step 2c did, if the old internal
filter worked as stated. The 100 ms roll LPF and mean/median windows still
follow. Which effect wins on this robot is a HARDWARE question; the raw
capture answers it. A fixture is not proof of real vibration rejection.

### INT1 check retired

Step 2c INT1 evidence: 1600 checks, 1541 matches, 59 mismatches. It proved
the GP7 wiring and showed the duplicate reads. Step 2d retires the check:

1. At 400 Hz DRDY changes every 2.5 ms; one level poll per 10 ms says
   nothing useful about a burst of about 4 samples.
2. FIFO_SRC gives the exact number of new samples, and a FIFO read removes
   each sample. That replaces the freshness check with a direct count.
3. A GPIO IRQ is not approved. CTRL_REG3_A=0x00 routes no event to INT1.

GP7 stays a pulled-down input that nothing reads; GP6 stays unused.

### TEST raw capture (IMU_STREAM only)

[imu_capture.c](../../app_program/subsystem4/imu_capture.c) is pure
fixed storage: 800 raw sample sets (4800 B, exact left-aligned 16-bit
words in SENSOR axes; the axis map is identity) and 256 batch records
(3072 B: time, first index, count). Total 7888 B static, ONLY in the
IMU_STREAM image; other images have no capture storage or code.

```text
IDLE -> ARMED (GP21 drive start) -> ACTIVE (next IMU14 tick)
ACTIVE -> FROZEN: 800 samples, 2 s, drive stop, error gap, or 256 batches
FROZEN -> dump after CONFIRMED stop, one batch per 20 ms -> release -> IDLE
```

- IMU14 is the only writer. It copies at most 8 sample sets per tick and
  never waits. FROZEN data does not change until the observer releases it.
- The observer dumps only when no drive is active and no stop is pending.
  A new GP21 start during a dump pauses the dump, prints
  `[S4_RAW] disabled/busy; drive unaffected` and does not overwrite the
  old capture. Capture state never changes drive permission.
- An empty cycle adds a zero-count batch. A FIFO reset or bus fault
  freezes the capture with reason GAP; it never joins data across a gap.
- The final batch can be cropped at 800; the pipeline mean never is.

Output lines, after `[S4_RAW_ARM] 800 raw16 XYZ; dump AFTER stop`:

| Line | Fields |
| --- | --- |
| `[S4_RAW_BATCH]` | `batch` index, `first` sample index, `n` count, `t_us` IMU14 read-completion time |
| `[S4_RAW]` | sample index, X, Y, Z raw words; mg = word/16 in HR, before bias |
| `[S4_RAW_END]` | `n` samples, `batches`, `reason` (1 FULL, 2 TIME, 3 STOP, 4 GAP, 5 BATCHES FULL), `odr 400` |

There is no per-sample timestamp: samples are 2.5 ms apart on the SENSOR
clock, and the last sample of a batch is at most about one sample period
plus the read time before `t_us`.

### New reported fields

`[S4_FIFO]`, once per second: `ctrl1`, `ctrl5`, `fifo_ctrl` readbacks;
last `src` and `n`; `polls` (ready read cycles), `batches`, `samples`,
`min`/`max` per accepted batch, `empty`, `overrun`, `backlog`, `bad`,
`reset`, and `mean_samples_per_poll = samples/polls`. Expect about 4.000
and `min`/`max` 3/5. Timing stays in `[S4_DIAG]`: `read_us`, `work_us`,
`core_us` and their maxima/totals, `gap_max_us`, `missed`, `wake_lost`.
CPU: the no-radio IMU_STREAM image has no idle meter (MQTT images only).
Use the `work_total_us` change over elapsed time as the wall-work fraction;
it includes preemption, so it is not exclusive CPU time.

## Step 2d current procedure

1. Build IMU_STREAM as before. Boot LEVEL, USB console only, GP0/1 free
   of UART wiring. Keep the VIN and lifted-first GP21 rules of step 2b.
2. Static: check `[S4_FIFO]` readbacks 77/40/80, mean samples per poll
   about 4, and zero overrun/backlog/bad/reset. Record `read_us` and
   `read_max_us` (estimate about 1.1 ms; deadline 2 ms).
3. Lifted wheels, one GP21 drive: wait for `[S4_RAW_END]`. Check reason
   FULL, 800 samples, continuous XYZ triplets near (0,0,+1 g raw) and
   whether every word is a multiple of 64 (10-bit FIFO).
4. Flat 600 mm drive, then THREE RIGHT-wheel 10 mm hump drives, LEVEL start
   and GP20 MARK before each. Save each raw capture. Compare roll height
   sd, median false peaks and `[S4_VIB]` std with step 2c.
5. Offline: compute the spectrum of the 400 Hz raw capture. Report the
   energy near 100/200 Hz (removed by the mean) and in 11..44 Hz (now
   passed by the sensor). Do not tune windows from one run.
6. Record `polls`, `samples`, `min`/`max`, `empty`, `overrun`, `backlog`,
   `reset`, faults/timeouts, `read_us`/`work_us`/`core_us` maxima, gaps
   and both stack high-water marks. Results:
   [step 2d hardware](../../testing/subsystem4/TEST_LOG.md#step-2d-hardware-on-0207b54).

## Step 2d software validation

Local only: no push, flash, broker or hardware. Public example settings
headers in a separate copy; generated firmware and proof ELFs removed.

- 165/165 host tests pass. New: FIFO status edges (0, 1..31, contradictory,
  OVRN/32), word decode and HR mean; alias fixtures above; probe counts
  0/1/4/8 accepted, 9 backlog, OVRN and contradictory status reset, a
  sample arriving during the burst kept for the next mean, deadline fault,
  setup readbacks 77/40/80; capture FULL/TIME/STOP/GAP/batch-full/crop;
  service EMPTY (no step, no new timestamp, MARK delayed one tick) and
  capture handoff (arm, freeze at 800, immutable read, release, stop, gap);
  observer dump only after a confirmed stop, one batch per wake, one
  release. Clang address/undefined/float-cast sanitizers pass 41 IMU tests.
- Recorder 67/67 and dashboard 180/180 pass.
- Seven clean `make -j4 --output-sync=target` builds have zero warnings.
  15 related units per profile (all subsystem4 sources, vehicle_task,
  app_task) pass `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`
  (105 compilations). Seven SDK retained-link proofs pass; no `__atomic_*`
  helper. The UART-conflict `#error` still rejects the IMU adapter.
- Template deviations stay 8 against c3b0954.

| Clean profile | text | data | bss |
| --- | ---: | ---: | ---: |
| Default SMP1 USB/Wi-Fi | 331260 | 760 | 130560 |
| Robot SMP0 no radio | 84340 | 688 | 124024 |
| BENCH SMP0 no radio | 41982 | 148 | 95460 |
| MQTT SMP0 | 395536 | 904 | 188580 |
| TEST SMP0 no radio | 78532 | 688 | 123444 |
| MQTT-TEST SMP0 | 385800 | 904 | 187604 |
| IMU_STREAM SMP0 no radio | 75752 | 688 | 131140 |

See [RAM evidence](../kernel/RAM_BUDGET.md#imu-step-2d-measured-delta).
These are software gates, not physical noise, timing or accuracy results.

## Step 2d hardware result

Floor run on 2026-11-09, `0207b54` IMU_STREAM image: 10 mm triangular
hump under the RIGHT wheel, three GP21 drives at 100 mm/s for 600 mm.
Drive 3 had no GP20 MARK, so its values are offline. An earlier attempt
that day was lost with the USB logger; it is not part of this result.
Full numbers: [S4 step 2d hardware](../../testing/subsystem4/TEST_LOG.md#step-2d-hardware-on-0207b54).

| Result | Step 2c (8c5d679) | Step 2d (0207b54) |
| --- | --- | --- |
| Hump peak, 10 mm object | 16.8/13.1/12.4 mm | 10.93/10.85/10.44 mm (drive 3 offline) |
| Plateau (200 ms median max) | Not a PASS | 10.72/10.20 mm firmware; 10.31 mm offline (drive 3) |
| Flat roll height sd | 1.6..2.7 mm | 0.50..0.68 mm, max single 1.2 mm |
| Flat median false peak | up to 5.7 mm | 0.6 mm (offline) |
| FIFO faults/resets | n/a | 0 in 45000 polls; 3.913 samples per poll, min 3, max 6 |
| read_us / work_us average (max) | n/a / 1562 us in step 2b | 1315 (1610) / 2699 (3224) us |

Operator conclusion: the main step 2c error was roughly 90 Hz motor/gear
vibration aliased into 100 Hz samples. The raw captures show lines near
25 Hz and 85..97 Hz; the 4-sample mean lowers raw sd by about 40 % (X)
and 50 % (Y). The 9..11 mm hump and <+/-1 mm flat targets are met on the
floor. **S4-U05 driven PASS for the 10 mm RIGHT-wheel hump only.** Other
heights, a mat and curves are Not run. The datasheet concern that 400 Hz
passes more 11..44 Hz content did not prevent these results.

Follow-up: work_us is about 27 % of core 0 wall time, mostly busy-wait
I2C at priority 14. Reduce it (for example an interrupt- or DMA-driven
transfer, or fewer bytes per cycle) in a separately approved step. Stack
high-water was not supplied.

## Step 3 hump report

Report only. The detector counts humps and gives the highest one. It
never changes a motor command, a navigation state or a stop decision.
All numbers in this section are synthetic fixtures or build results,
**not hardware results**, unless a line says otherwise.

### Decisions (approved D1..D5)

| ID | Decision |
| --- | --- |
| D1 | Course humps always hit the RIGHT drive wheel (operator fact). RIGHT is the primary side. A LEFT event is an unproven bonus with a side flag. No full-width estimator. |
| D2 | Thresholds below. Default source is the corrected roll height, plus the curve gate. `IMU_HUMP_SOURCE` selects the filtered roll as a fallback. Event peak is the highest 100 ms mean. |
| D3 | Event v2 gets kind 9. Summary v3 is 52 B with hump peak (0.1 mm), count, flags and gated count. No reserved barcode bytes: barcode takes v4. |
| D4 | No core_us trim in this step (see [CPU](#cpu-and-the-d4-decision)). |
| D5 | One USB line at run end, one GP20 DUMP line. DUMP rows do not grow. |

### Data path

```text
IMU task14 (10 ms) --> imu_core --> imu_hump_step --> hump board (52 B)
     ^ run_epoch (atomic)                                    |
     |                                    vehicle task13 copy (no wait)
vehicle run start: epoch odd             |-- live event kind 9 (MQTT)
vehicle run end:   summarize, epoch even |-- summary v3 (MQTT)
                                         |-- USB "[robot] hump ..." line
                                         '-- run board --> GP20 DUMP line
```

- `vehicle_run_epoch()` is odd during a run. The vehicle makes it odd at
  run start and even (+1) at run end, after it takes the summary. TEST
  images use `imu_test_run(true/false)` around the GP21 STREAM drive.
- An odd new epoch starts a run: totals clear; the 100 ms mean continues.
  An even epoch ends the run: a confirmed open event closes with OPEN.
- The IMU publishes the hump board only on a real observation. The board
  time is the IMU observation time, so a stalled sensor makes every copy
  old. A busy board keeps the reader's last copy.
- At run end the vehicle summarizes its last good copy. The copy must be
  from this run and at most 50 ms old (`IMU_HUMP_MAX_AGE_MS`), else the
  result is unavailable: peak 65535, count 0, flags IMPLEMENTED|FAULT.
  A stale copy never gives a value.
- No task waits: the vehicle reads with `imu_hump_try_read()` (E_PAR,
  E_OBJ, E_TMOUT or E_OK) and keeps going.

### Detector

Input each 10 ms: source height (RIGHT wheel up is +), encoder travel,
speed, encoder yaw rate, encoder fake lateral acceleration, validity.

| Setting | Value | Use |
| --- | ---: | --- |
| `IMU_HUMP_ON_MM` | 3.0 mm | Start level of the 100 ms mean, either sign |
| `IMU_HUMP_OFF_MM` | 1.5 mm | End level, same side as the start |
| `IMU_HUMP_ON_TRAVEL_MM` | 30 mm | Travel above ON before an event is confirmed |
| `IMU_HUMP_OFF_TRAVEL_MM` | 30 mm | Continuous travel below OFF that ends an event |
| `IMU_HUMP_MAX_TRAVEL_MM` | 600 mm | Longer events are slopes: rejected, flag LONG |
| `IMU_HUMP_GATE_LATERAL_MG` | 40 mg | Curve gate on encoder lateral acceleration |
| `IMU_HUMP_GATE_YAW_RAD_S` | 1.5 rad/s | Curve gate on encoder yaw rate |
| `IMU_HUMP_MIN_SPEED_MM_S` | 30 mm/s | Slower motion cannot start an event |
| `IMU_HUMP_MAX_AGE_MS` | 50 ms | Maximum copy age for the run result and live event |
| `IMU_HUMP_SOURCE` | 1 corrected | 2 = filtered roll fallback |

States: IDLE, CANDIDATE (above ON, travel counting), ACTIVE (confirmed),
LONG (rejected, waits for the mean to stay below OFF for 30 mm).

- A gate (curve, invalid motion or low speed) blocks a start. It is
  counted once for each ON crossing (`gated`); the count resets after the
  mean drops below OFF.
- A curve or invalid motion during an ACTIVE event freezes its peak and
  sets flag CURVE. Low speed does not freeze a peak.
- An invalid accel sample or a sample gap clears the mean window and sets
  flag FAULT during a run.
- Side: a negative mean is a LEFT event. Its magnitude is used.

| Flag bit | Name | Meaning |
| ---: | --- | --- |
| 0 | IMPLEMENTED | Always set by step 3 firmware |
| 1 | FAULT | Invalid/gap sample in the run, or result unavailable |
| 2 | OPEN | A confirmed event was still open at STOP; it is counted |
| 3 | CURVE | A curve gate froze an event peak |
| 4 | LONG | An event exceeded 600 mm and was not counted |
| 5 | LEFT | The highest event is LEFT (unproven side) |

### Outputs

| Output | Format | Image |
| --- | --- | --- |
| USB at run end | `[robot] hump highest_mm 9.4 side right count 1 flags 01 gated 0`, or `[robot] hump unavailable flags 03` | Robot, MQTT |
| GP20 DUMP, after the counts line | `[robot] hump peak_0.1mm 94 count 1 flags 01 gated 0`, or `[robot] hump no finished run` | Robot, MQTT |
| Live MQTT | Event kind 9: word peak 0.1 mm, detail number + LEFT bit 7 | MQTT |
| Summary | v3 bytes 42, 48, 50, 51; [MQTT wire](../s1-telemetry/MQTT_WIRE.md#run-summary) | MQTT |
| TEST STREAM | `[S4_HUMP] peak_0.1mm .. count .. flags .. gated .. long ..` after each GP21 drive. The 50 Hz `[S4]` line adds `hump_state`, counts, flags and `hump_avg_mm`/`hump_event_mm`/`hump_max_mm` | IMU_STREAM (`[S4]` also TEST) |

`python3 scripts/robot_log_decode.py --hump LOG` writes
`source,peak_mm,side,count,flags,gated` from the run-end, DUMP and STREAM
lines. The dashboard shows a HIGHEST HUMP card; see
[operation](../s1-telemetry/OPERATION.md#highest-hump).

### Fixture results (synthetic, not hardware)

Hump fixture: noise-free height profile 10 mm high and 195 mm long (two
ramps and a 5 mm crest) under the RIGHT wheel. The flat cases add white
noise at the step 2d flat sd; real floor noise is correlated, so H1 and H5
must confirm the false-count margin. Event peak is the highest 100 ms mean, so it
is lower than the 10 mm object at higher speed.

| Case | Result |
| --- | --- |
| 10 mm hump at 100 / 146 / 300 / 460 mm/s | peak 9.426 / 9.065 / 7.910 / 6.896 mm, count 1 |
| Flat 30 s, uniform white noise sd 0.68 mm (step 2d max) / sd 2.0 mm, one 12 mm single spike every 5 s | highest mean 1.635 / 2.479 mm, below ON 3.0, count 0 |
| 30 mg curve (300 mm/s, 1.0 rad/s), no hump | corrected mean 0.001 mm, count 0; filtered mean -3.394 mm and a false event |
| 10 mm hump on that 30 mg curve, 300 mm/s, full core | count 1, peak 7.699 mm |
| Two humps at 146 mm/s | touching humps merge into one; a 20 mm flat gap gives two (ramps below OFF add about 28 mm) |
| Long slope | rejected with flag LONG, not counted |

The 30 mg curve case is why the corrected roll is the default. Speed
attenuation at 300..460 mm/s is a property of the 100 ms mean; check it on
hardware (H2, H6) before a height pass/fail rule uses it.

### CPU and the D4 decision

Host `-Os` throughput only, not RP2040 timing: imu_core_step 117..130 ns
before (step 2d) and 130..135 ns after step 3 (1e6 updates). Compiling out
the step 2b support diagnostics saves about 8..10 ns on the host. Both are
near host noise. Step 2d hardware core_us is 1164 us average, so the
detector is expected to add a small part (estimate 20..40 us on target).
**Decision: no trim now.** H0 measures `core_us`/`work_us` with step 3;
a ROBOT-only compile-out of the step 2b diagnostics stays available as a
separate commit if H0 shows a need.

### Step 3 software validation

Local only: no push, flash, broker or hardware. Public example settings
headers in a separate copy; generated firmware and proof ELFs removed.

- 168/168 host tests pass (44 IMU). New: `test_imu_hump` (helpers, speeds,
  heights, side, flat noise, chatter and spacing, long slope, gates, run
  lifecycle, unavailable cases), `test_imu_hump_core_corrected` and
  `_filtered` (full core curve cases). Updated: service (hump board torn
  read, E_PAR/E_OBJ, FIFO-empty ageing, run epochs, fault), vehicle (run
  epochs, USB line), telemetry (DUMP line), network task (kind 9), wire
  (summary v3 and event fixtures), STREAM observer (`[S4_HUMP]`).
- Clang address/undefined/float-cast sanitizers pass all 168 tests.
- Recorder 68/68 and dashboard 185/185 pass.
- Seven clean `make -j4 --output-sync=target` builds have zero warnings.
  15 related units per profile pass `-Wall -Wextra -Wconversion -Wshadow
  -Wformat=2 -Werror` (105 compilations). Seven SDK retained-link proofs
  pass; no `__atomic_*` helper is linked.
- Template deviations stay 8 against c3b0954.

| Clean profile | text | data | bss | text / bss vs step 2d |
| --- | ---: | ---: | ---: | ---: |
| Default SMP1 USB/Wi-Fi | 334556 | 760 | 130976 | +3296 / +416 |
| Robot SMP0 no radio | 87636 | 688 | 124440 | +3296 / +416 |
| BENCH SMP0 no radio | 41982 | 148 | 95460 | 0 / 0 |
| MQTT SMP0 | 399156 | 904 | 188996 | +3620 / +416 |
| TEST SMP0 no radio | 80712 | 688 | 123788 | +2180 / +344 |
| MQTT-TEST SMP0 | 388060 | 904 | 187940 | +2260 / +336 |
| IMU_STREAM SMP0 no radio | 78564 | 688 | 131540 | +2812 / +400 |

RAM detail: [IMU step 3](../kernel/RAM_BUDGET.md#imu-step-3-measured-delta).

### Step 3 H1 hardware result

Operator-supplied results on `1480b37`, 2026-11-09: H1 PASS, floor,
10 mm RIGHT-wheel hump, 100 mm/s, 600 mm, three GP21 drives. Image:
`mtk3pico_smp0_usb_cdc_test_imu_stream.uf2`. The private log was not read.

- `[S4_HUMP]` peak_0.1mm: 99 / 113 / 105 (9.9 / 11.3 / 10.5 mm).
  Count 1 each; flags 01; gated 0; long 0. Drive 3 had no GP20 MARK;
  GP21 resets run totals, so no MARK is needed for the hump result.
- DIAG: faults 0; read_us 1262/max 1588, work_us 2730/max 3288,
  core_us 1067/max 1598; missed 0. FIFO empty/overrun/backlog/bad 0;
  mean 3.913 samples per poll.
- STACK in this STREAM image: imu_task high-water 1448/2048 B,
  free 600 B (dfade14: 752 B used); sonar 320/2048 B;
  telemetry 464/4096 B. The IMU exceeds the required 256 B free margin.
  The later ROBOT reading on `d51d27b` is 1456/2048 B, free 592 B.
  Diagnostic C needs a new named stack reading.

No separate flat-only result was supplied. H1 confirms event counting;
drive 2 at 11.3 mm does not support a blanket 9..11 mm height claim.
See [H1 record](../../testing/subsystem4/TEST_LOG.md#step-3-hump-report-h1-hardware-on-1480b37).

### LINE short-course result on d51d27b

Operator-supplied hardware evidence, 2026-11-09. MQTT robot image, seven
runs on two tiles at about 146 mm/s. Runs 1/5/6/7 crossed the 10 mm RIGHT
hump: peaks 14.0/12.9/13.6/15.5 mm, one count each. Flat runs 2/3/4 had
zero humps. IMU fault, long and left flags were zero throughout. Thus
counting passed these trials, but height over-read by 29..55 %: **not a
height PASS**. The operator pressed dashboard ESTOP after the line ended
and SEARCH began; stop_reason 2 means MQTT ESTOP, not line end.

Run 7 had CURVE|OPEN: a gate froze an ACTIVE peak at least once, and the
event had not completed its below-OFF travel at STOP. The summary counts
it by virtual closure, explaining the missing kind 9. A kind 9 time is
closure time, not peak time. Neither CURVE nor OPEN locates the error.

Same boot, GP20 after USB reconnect: robot IMU high-water 1456/2048 B,
free 592 B. Heartbeat minimum free 272 B was the 512 B idle_meter stack,
not IMU. Heartbeat has no individual IMU-stack field; GP20 calls
`app_task_report()` and prints the named `[stack] imu_task ...` line.
It measures the boot's high-water, not just the last run, and excludes
Wi-Fi/USB template stacks from the application minimum.
Full summaries, timing/network values and all named stacks:
[H0/H2-short record](../../testing/subsystem4/TEST_LOG.md#step-3-h0-and-h2-short-on-d51d27b).

Shared condition (operator-confirmed): the front-LEFT barcode sensor,
about 5.5 mm above the floor, scraped on the 10 mm hump in ALL earlier
floor drives, including step 2d and H1 STREAM. Their 10..11 mm values
already include this contact. Contact can change support/pivot geometry
and add drag/yaw, but does not alone explain the LINE/STREAM difference.
Record it in each trial. LINE zigzags slightly while climbing; STREAM
commands equal wheel speeds. Compensation lag remains a hypothesis:
100 ms encoder windows and the filtered yaw derivative can lag accel
transients. For scale only, 10 rad/s^2 at the 60 mm lever is about 61 mg,
roughly 6.8 mm apparent roll height at track 111 mm (small-angle estimate).
The 40 mg gate is a gate on estimated compensation, NOT an error bound;
filtered transients can persist after it clears. No tuning is applied.

### Diagnostic C: peak context

`imu_hump_diag.c` is a pure observer of the existing detector output.
The service calls it once after the final core step/calibration replay.
Its bounded window is separate from core's rollback copy and normal
snapshot. It adds no sine computation, IRQ, wait or driving decision.
The default source and all thresholds are unchanged; MQTT/wire and DUMP
rows are unchanged.

- Keep the context of each increase of a CANDIDATE/ACTIVE event peak.
  Equal peaks keep the first instant. Only confirmed events can become
  the run winner; cancelled candidates and LONG events cannot replace it.
  A confirmed ACTIVE event is included at STOP just like the summary.
- Paired signed corrected and uncorrected filtered-roll heights are means
  of the SAME 10 observations. The highest SELECTED source wins; this is
  not an independent highest uncorrected peak.
- Record epoch, elapsed ms from run start, accumulated absolute encoder
  travel from the first run observation, speed, yaw rate, yaw acceleration,
  lateral compensation, side, gate at the peak and gate OR over its
  window, plus window maxima of |yaw|, |yaw acceleration| and |lateral|.
  The instant is the LAST observation of the peak window, not the time
  of every acceleration sample in it. Travel includes the first observed
  encoder interval (up to a tick across the run boundary).
- Retain run-wide max |lateral| and |yaw acceleration| only while the
  sampled navigation module is active LINE. SEARCH/SCAN/DETOUR do not
  contribute. `line_samples` shows whether this measurement exists;
  it is 0 in STREAM (which has no LINE module). LINE context is updated
  on navigation transitions; it is not a measurement of wheel contact.
- An invalid/gap observation clears the parallel window. EMPTY FIFO does
  not step it or refresh its board. A new run clears winners and run
  maxima, not the height window, matching the detector.
- One bounded diagnostic board is read by the vehicle. End-of-run data
  must match the hump board's timestamp and run, be at most 50 ms old,
  and have a usable hump summary; else `valid=0` and all values clear.
  A separate last-run board serves GP20 without increasing INFO/rows.
  STREAM uses the same age/run rules. Busy reads retain the last copy.

One `[S4_HUMP_DIAG]` USB line is printed after STOP, in robot and STREAM,
and repeated at GP20 DUMP. It has no float printf fields:

| Fields | Units / meaning |
| --- | --- |
| `epoch`, `valid`, `peak_valid` | Run epoch; usable run data; a confirmed winning peak exists |
| `peak_ms`, `travel_mm` | Peak window endpoint from run start; encoder path in whole mm |
| `corrected_0.1mm`, `filtered_0.1mm` | Signed simultaneous 100 ms mean heights, 0.1 mm |
| `speed_mm_s`, `yaw_mrad_s`, `yaw_accel_mrad_s2`, `lateral_mg` | Signed motion/compensation at the peak window endpoint |
| `win_yaw_mrad_s`, `win_yaw_accel_mrad_s2`, `win_lateral_mg` | Maximum magnitudes over that window |
| `gate`, `win_gate`, `left` | Gate mask at the endpoint / OR over the window; LEFT=1 |
| `line_samples`, `line_lateral_mg`, `line_yaw_accel_mrad_s2` | LINE-only sample count and run-wide maximum magnitudes |

Gate bits: motion invalid 0x01, lateral 0x02, yaw 0x04, speed 0x08.
Low speed blocks starts only; an ACTIVE peak can legitimately have bit
0x08. `peak_valid=0` means no confirmed peak, NOT a measured zero-height
peak. `valid=1, peak_valid=0` can still report LINE-wide maxima.

Software-only checks: 169/169 host and Clang address/undefined/float-cast
tests pass; recorder 68/68, dashboard 185/185. The new test checks paired
windows, gate history, first-peak ties, lower/higher events, LEFT, rejected
candidates/LONG, virtual/actual STOP, epoch reset, invalid-window reset,
LINE-only maxima, timestamp wrap/staleness, and integer output units.
Service/vehicle/STREAM/DUMP tests check bounded reads and end-run output.
Seven clean profiles have no warnings; 126 strict ARM compilations and
7 SDK retained-link proofs pass. Template deviations remain 8.
RAM +772 B MQTT; sample frame 944 -> 1000 B (STREAM 952 -> 1016 B).
See [RAM and stack evidence](../kernel/RAM_BUDGET.md#imu-diagnostic-c-measured-delta).
New physical timing and stack high-water are **Not run**.

Approved comparison: earlier hump with at least 200 mm of flat line
AFTER clearance before SEARCH/stop (3 hump + 2 flat). Use diagnostic C
and record contact, summary and named stack lines. Keep at least 256 B
IMU free margin; rerun the named stack check on this image. If the cause
remains unclear, separately approve STREAM at 146 versus 100 mm/s. A
steering drive is a later separately approved test, not part of C.

### Hardware checks (H1 PASS; H0/H2 partial)

Use the MQTT robot image unless the row says STREAM. Record the USB
run-end line, the summary and the `[S4_DIAG]` or health timing.

| ID | Check | Pass condition |
| --- | --- | --- |
| H0 | MQTT LINE run, no hump: CPU, Wi-Fi gap, overruns | Partial on d51d27b: 3 flat runs, count 0; overruns/missed 0; IMU free 592 B. CPU/network baseline and diagnostic C stack check still needed |
| H1 | STREAM floor repeat, 10 mm RIGHT hump, 100 mm/s | PASS on 1480b37: count 1 each; peaks 9.9/11.3/10.5 mm, flags 01, gated/long 0; separate flat-only result not supplied |
| H2 | LINE run over the 10 mm hump on a straight | Partial on d51d27b: 4 humps counted once each; peaks 12.9..15.5 mm, not a height PASS; diagnostic C repeat pending |
| H3 | Curves and SEARCH, no hump | count 0; record `gated` and flags |
| H4 | Hump right after a curve | count 1; record flag CURVE and peak |
| H5 | Course mat, no hump | count 0 (mat texture is not in the fixtures) |
| H6 | Other hump heights | peak against object height; record the speed loss |
| H7 | LEFT-wheel hump | side left, flag LEFT (bonus; not required) |
| H8 | Stop on the hump (GP21) | flag OPEN, the hump is counted |
| H9 | Broker down during the run | USB line and DUMP line still correct; summary after reconnect |

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
