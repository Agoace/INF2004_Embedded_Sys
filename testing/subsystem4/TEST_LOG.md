# Subsystem 4 test log

## GY-511 steps 0 and 1

Use the [operator tilt procedure](../../docs/s4-imu/IMU.md#operator-tilt-check-not-run).
Source credit: Jaison, `origin/jaison-imu` `b2a6bb6`.

| ID | Check | Result |
| --- | --- | --- |
| S4-U01 | TEST, wheels lifted: 30 s of 10 ms polling, USB each 100 ms; confirm axes, gravity, tilt, bus/read timing and absent-device behavior | Partial hardware evidence on 02f7c37: two windows of 2999 reads, bus/config faults zero; read 529–542 us, max 624 us, gap max 18975 us, missed zero. Mag changes with tilt; side roll captured. Sign portion PASS via the later 6ed89ad stream: nose up +Y, right side up +X, top up +Z confirms identity map. Static magnitude 1371–1391 mg is explained by positive Z offset, not uniform gain. Absent-device check remains Not run; not a full S4-U01 PASS. See [investigation](../../docs/s4-imu/IMU.md). |
| S4-U03 | TEST IMU_STREAM=1: only S4 streams until power-off, USB line every 100 ms, no S2 motor command/destructive watchdog test; GP20 MARK once per stable press; record six faces, nose up/down, both sides, saved 57/88 setup register values, all counters and new read_us | PASS on 6ed89ad, USB only: 16129 reads, fault/NACK/timeout/identity/config zero, ctrl1 57 ctrl4 88; read about 530–566 us, max 592 us, gap max 17946 us, missed zero. Six debounced MARKs, no repeats. Six-face means and offset conclusion are in the investigation below. |
| S4-U02 | Cold power boot with IMU on GP0/GP1 and USB only: successful USB console and sensor configuration, no UART mux; repeat on MQTT-TEST | Not run |

Actual read wall time on 02f7c37: **529–542 us, maximum 624 us** per nominal
10 ms cycle. On 6ed89ad continuous stream: **about 530–566 us, maximum
592 us**. Mag is still read each cycle; the two-transfer workload is unchanged.

Six-face means (samples +3..+25 after each MARK, hand-held, mg):

| MARK | Pose | X | Y | Z | Magnitude |
| --- | --- | ---: | ---: | ---: | ---: |
| M1 | Top up | -0.1 | 82 | 1376 | 1378 |
| M2 | Upside down | -128 | 124 | -776 | 797 |
| M3 | Nose up | 37 | 1116 | 459 | |
| M4 | Nose down | -44 | -943 | 322 | |
| M5 | Right side up | 1012 | 107 | 374 | |
| M6 | Left side up | -1022 | 115 | 315 | |

Z offset is about +300 to +350 mg (opposed-face half-sum +300); X/Y/Z
half-differences are about 1017/1030/1076 mg/g, not jig-exact gain estimates.
Y half-sum about +87 mg matches level Y=82 mg; a 180-degree yaw flip is
needed before attributing raw level pitch to mounting. See
[scale investigation](../../docs/s4-imu/IMU.md) for the per-axis
bias-removal requirement and the corrected 0.90–1.10 g gate ordering.
The TEST output prints data and counters, not an automatic hardware PASS.
No stationary calibration, filtering, hump threshold or robot integration
has been tested or implemented in this historical steps-0/1 result.

## Observe-only step 2 (Not run)

Software starts from staging `b569adc`. See the
[current implementation and formulas](../../docs/s4-imu/IMU.md#current-scope)
and [RAM/CPU limits](../../docs/kernel/RAM_BUDGET.md#imu-step-2-measured-delta).
The local host/build checks do not change any hardware result below.
S4-U03's old raw-stream PASS does not imply that these checks pass.

| ID | Check | Result |
| --- | --- | --- |
| S4-U04 | Stationary LEVEL support, then wheels-lifted motor noise: automatic >=1 s calibration/refresh, per-axis bias and noise, post-bias magnitude, no calibration during wheel activity or CALIBRATE spin, fault responsiveness, 100 Hz task timing and stack/CPU load | **Not run** |
| S4-U05 | Three zero-duty hand pushes over the 10 mm high, 195 mm long hump, GP20 MARK before each: record calibrated/raw/filtered/compensated pitch, encoder travel/speed and height; expect about 10 mm peak and return near zero without an intervening calibration/MARK reset | **Not run** |
| S4-U06 | Later zero-duty hand pushes over measured 5 mm and 20 mm humps; record geometry, speed, trace and error, no detection/minimum-height threshold | **Not run** |

### S4-U04 procedure

1. With operator approval, use the step-2 image and save its exact revision
   and full USB log. Use USB only, `CRASH_REPORT=0`; remove GP0/GP1 UART
   wiring. Record cold/warm boot separately; retain S4-U02 as its own gate.
2. On known level support, leave both wheels and body still for >=1 s.
   Record calibration count/flag, all three bias/noise values and rejects.
   The known raw ~1375 mg magnitude must not cause perpetual rejection;
   corrected stationary acceleration should be near `(0,0,1000)` mg.
3. Repeat stationary refresh. Confirm its height/peak/distance reset is
   visible. A stationary tilted support is NOT an independent calibration:
   it defines that support as level. Do not install hand-held gain constants.
4. For motor-on noise, lift both wheels and use a separately operator-
   approved motor-test harness. IMU_STREAM itself requests zero duty and
   deliberately supplies no motor-driving path. Record the harness/revision
   and motor state. Rotation/nonzero duty and the entire line CALIBRATE
   spin/pause/home episode must prevent an IMU bias refresh. A zero mean
   wheel delta during equal/opposite rotation must not count as stillness.
5. Record a stable interval's elapsed time, work/core counter deltas, maximum
   work/core/read time, maximum gap, missed/lost wakes, motion/line/vehicle
   timing, USB drops and all task high-water/free bytes. Require at least
   the existing 256 B untouched stack margin; do not infer it from size.
   Work counters include preemption, not exclusive CPU. Compare the idle
   meter too; the documented 6..10% IMU planning estimate is not acceptance.
6. Power off before disconnect/reconnect. Confirm counted absent-device
   faults, paced retry and responsive USB with normal motion watchdog
   ownership. This remains the separately pending absent-device gate too.

### S4-U05 and S4-U06 procedure

1. Boot on level support and wait for calibration. IMU_STREAM runs only S4,
   reports at nominal 50 Hz and leaves the normal motion watchdog active.
   Zero duty drives both bridge inputs low: COAST, not active braking.
2. Check support/hump dimensions. Use GP20 MARK before each of three
   separate hand pushes. Save the acknowledged `MARK n`, calibration count,
   bias, fault counters and full timestamped trace, not just the largest
   printed number. Use a gentle, measured push speed near 100 mm/s first.
3. Do not stop for >=1 s on the slope/crest. Automatic stationary refresh
   would erase height/peak/distance and redefine the pose as level. Verify
   the calibration count does not change from MARK through the return
   measurement. A return to zero only after refresh or another MARK fails
   to establish return accuracy.
4. After descent, continue over level support so filters can settle; record
   final height BEFORE the next stationary refresh. Record travel/speed,
   sign, peak and return for all three pushes. The expectation is about
   10 mm peak and near-zero return, not an automatic PASS threshold.
5. Compare a matched flat start/stop push. Encoder acceleration correction
   has delay: synthetic flat tests already show -0.431 mm return at
   100 mm/s and -7.994 mm at 460 mm/s. Those are model results, not hardware
   error bounds. Record actual false peaks/drift and do not tune terrain
   thresholds to hide them.
6. Run S4-U06 only after the 10 mm trials, with operator approval and
   measured 5/20 mm geometry. These heights are not course limits, and
   host synthetic results are not physical passes. No driving behavior,
   detection event or MQTT/KR1 IMU field is part of these checks.
