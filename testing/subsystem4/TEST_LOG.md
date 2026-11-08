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

## Observe-only step 2 and step 2b

Software starts from staging `b569adc`. See the
[current implementation and formulas](../../docs/s4-imu/IMU.md#current-scope)
and [RAM/CPU limits](../../docs/kernel/RAM_BUDGET.md#imu-step-2-measured-delta).
The local host/build checks do not change any hardware result below.
S4-U03's old raw-stream PASS does not imply that these checks pass.

| ID | Check | Result |
| --- | --- | --- |
| S4-U04 | Stationary LEVEL support, motor noise, guarded refresh, timing/stack/CPU and fault checks | **Partial on dfade14**, stationary hand-push session below; powered noise and step-2b checks Not run |
| S4-U05 | Three hand pushes and later driven RIGHT-wheel hump trials | Hand-push finding/offline analysis below; **not height acceptance**. Step-2b driven trials Not run |
| S4-U06 | Later zero-duty hand pushes over measured 5 mm and 20 mm humps; record geometry, speed, trace and error, no detection/minimum-height threshold | **Not run** |

### dfade14 hand-push hardware and offline analysis

Operator report, private log name only:
`build/mtk_logs/s4_step2_hump_175042.txt`. No private log contents are copied.

- Calibration worked: corrected magnitude about 1000 mg, Z bias about
  +373 mg, refresh during still periods. Filtered stationary pitch noise
  about +/-0.1..0.3 degrees. Reported window: 1100 reads, zero faults.
- IMU high-water 752/2048 B. Work about 1.3..1.4 ms per 10 ms (about
  13..14% wall-work fraction), max 2.48 ms; core about 0.63 ms, read 0.54 ms.
  These replace the earlier 6..10% planning estimate, not exclusive CPU
  measurements. They apply to dfade14, not step 2b or all-load stack bounds.
- Three resets had no message. USB cable movement was suspected but is
  UNCONFIRMED; neither watchdog nor power/cable cause is established.
- The hump was under ONE RIGHT wheel. Roll +3..+7 degrees and pitch
  -2..-5 degrees indicate axle rise with caster staying down, not a long
  full-width slope. The slope integral gave negative values about
  -4/-7/-10 mm and did not estimate wheel lift correctly.
- Hand-push encoder travel was 170..218 mm versus about 400 mm expected.
  Gearmotor-wheel slip makes this hand-push distance unsuitable as truth.

Offline recalculation used WB=79 mm, T=111 mm, compensated filtered pitch
and filtered roll under the caster-flat hypothesis. These are OFFLINE
numbers, not firmware outputs. Plateau is the median of samples above
0.8 times the selected peak. References are rough/compressible, not gauges.

| Push / object | Offline zR plateau median mm | Offline peak mm | Consistency / limit |
| --- | ---: | ---: | --- |
| 1: 10 mm right-wheel ramp | 10.2 | 11.1 | zL about 0.4 mm |
| 2: roughly 5 mm one-wheel object | 6.0 | 6.9 | Selected with abs(zL)<2 mm; ungated peak13.9 mm had zL10.3 mm, a hand press/lift artifact |
| 3: notebook, rough13..14 mm, compressible | 13.9 | 14.8 | zL about1.6 mm |

Useful model evidence, not physical height acceptance. The course operator
confirmed the RIGHT wheel only, with left wheel/caster on the floor. The
step-2b defaults are WB80/T111, not the offline fit79; track115 was a rough
operator measurement, not a replacement for the effective calibrated111.

### Step 2b hardware on75ff329

Operator-supplied numbers only; private log name
`build/mtk_logs/s4_step2b_192054.txt` was NOT opened or copied.

- New cable: no resets,21400 reads,zero faults, max sample gap18.4 ms,
  missed wakes0. This does not establish the cause of the earlier resets.
- Work average1562/max2530 us; read564/705 us; core818/1053 us.
  Average wall-work is15.62% at100 Hz, not exclusive core CPU utilization.
  These are step2b measurements, NOT the new step2c budget.
- Static RIGHT-wheel objects,1 s means: rough5 mm object gave5.2/5.2 mm
  as reported;10 mm ramp had roll+5.4/pitch-4.1 degrees, roll-only
  `111*sin(roll)` about10.5 mm versus full-model firmware zR11.1 mm.
  Notebook, compressible13..14 mm: roll-only12.5 mm. Expected pitch from
  right-lift geometry was-3.7 degrees at roll5.4, measured-4.1: static
  pose is consistent with caster/left wheel staying on the floor.
- After MARK3 the body was lifted by hand: zL28 mm and full-model zR
  peak13.95 mm became polluted. A tilt-only observer cannot prove ground
  contact or reject every hand-lift pose. MARK on LEVEL before each trial.
- VIN/lifted wheels: GP21 automatic DISTANCE stop at601.6 mm; another run
  BUTTON stop at93.8 mm. Operator PASS. These are encoder distances at the
  stop decision, not verified physical ground stopping distances/braking.
- Three powered100 mm/s right-wheel hump runs started LEVEL (operator
  confirmed); no GP20 between runs, so firmware peak retained13.95 mm.
  Offline split only: corrected-pitch sd3.2..4.9 degrees versus static
  0.1..0.3, roll sd about1.0..1.2 degrees. Pitch contribution swung zR
  about+/-14 mm on FLAT floor; abs(zL)<=2 eligibility almost never held.
  The full-model primary was unusable while driving, not a height PASS.

Offline roll-only `T*sin(filtered roll)`,200 ms mean (not step2c firmware):

| Run over10 mm hump | Offline peak mm | Offline median of samples >0.8 peak mm |
| --- | ---: | ---: |
| 1 | 14.4 | 12.9 |
| 2 | 7.4 | 6.7 |
| 3 | 12.8 | 11.6 |

Earlier hand-push roll-only estimates were10.4/6.0/13.9 mm. The derived
RIGHT-only model uses roll=asin(h/T), with left wheel/caster on the floor;
pitch=-asin(h/(2*WB)) becomes a consistency diagnostic. Dynamic pitch noise
motivates step2c; offline agreement is not physical accuracy acceptance.

### Step 2b remaining checks

| ID | Check | Result |
| --- | --- | --- |
| S4-U07 | Static blocks, RIGHT first; then LEFT/both/caster | **Partial on75ff329**: RIGHT results below; LEFT/both/caster and current step2c checks Not run |
| S4-U08 | VIN, lifted GP21, then driven RIGHT-wheel hump | **Lifted distance/button PASS on75ff329**; driven estimator FINDING below, no height-accuracy PASS; hardware timeout/fault paths Not run |

### Step 2c hardware on 8c5d679

Operator-supplied numbers only; private log name
`build/mtk_logs/s4_step2c_215556.txt` was NOT opened or copied.
**Driven result: NOT a height PASS.**

| Check | Result |
| --- | --- |
| INT1 HIGH-before/LOW-after accel reads, CTRL_REG3=0x10, weak pull-down/no IRQ | 1600 checks: 1541 matches, 59 mismatches; HIGH before 1544, LOW after 1579. GP7 wiring works. About 3.5 % of reads (56 LOW-before) found no new sample: duplicate reads from sensor/kernel clock drift |
| Flat drive: roll-only noise, median false peaks | Roll-only height noise sd 1.6..2.7 mm; 200 ms median false peaks up to 5.7 mm on FLAT floor |
| Three 10 mm RIGHT-wheel hump drives | Reported peaks 16.8/13.1/12.4 mm; **NOT a height PASS** (10 mm object) |
| Per-drive `[S4_VIB]` std, three drives as supplied | Pitch 2.95/3.55/4.14 degrees; roll 2.04/1.80/2.05 degrees |
| Offline spectrum, raw 50 Hz pitch/roll, flat | About 50 % of variance in 12..25 Hz, piled at Nyquist: LIKELY aliased motor/gear vibration, not proven |
| Work/stack/gaps for step 2c | Not supplied; Not run as a recorded result |

These findings start [step 2d](../../docs/s4-imu/IMU.md#step-2d-fifo-input).
The [step 2c procedure](../../docs/s4-imu/IMU.md#step-2c-procedure-at-8c5d679-historical)
stays as history. The real hump is 10 mm high/195 mm long: about 95 mm
per 6-degree ramp, only a 5 mm flat crest. At 100 mm/s, above 80 % height
lasts about 0.43 s and above 90 % about 0.24 s. Do not assume a 0.5 s flat
plateau. Investigate the three unexplained step 2 resets before treating
any trial as reliable. Do not substitute offline gated peaks for a
firmware/hardware PASS.

### Step 2d pending hardware

Software only on the step 2d commit; see
[step 2d procedure](../../docs/s4-imu/IMU.md#step-2d-current-procedure).

| Check | Result |
| --- | --- |
| Readbacks ctrl1 77, ctrl5 40, fifo_ctrl 80; samples per poll mean/min/max; empty/overrun/backlog/bad/reset counts | **Not run** |
| `read_us`/`work_us`/`core_us` average and maximum against the 2 ms cycle deadline; gaps; IMU14/TEST20 stack high-water | **Not run** |
| Raw 400 Hz capture: FULL 800 samples, continuous triplets (burst rollback), 10-bit or 12-bit FIFO words | **Not run** |
| Flat 600 mm drive: roll height sd and median false peak versus step 2c | **Not run** |
| Three 10 mm RIGHT-wheel hump drives, LEVEL start, GP20 before each | **Not run** |

### S4-U04 procedure

1. With operator approval, use the step-2 image and save its exact revision
   and full USB log. Use USB only, `CRASH_REPORT=0`; remove GP0/GP1 UART
   wiring. Record cold/warm boot separately; retain S4-U02 as its own gate.
2. On known level support, leave both wheels and body still for >=1 s.
   Record calibration count/flag, all three bias/noise values and rejects.
   The known raw ~1375 mg magnitude must not cause perpetual rejection;
   corrected stationary acceleration should be near `(0,0,1000)` mg.
3. Repeat stationary level refresh, then hold a wheel on a block: step 2b
   must reject refresh when tilt differs by >=1 degree from current bias-
   corrected level. Record refresh rejects, unchanged bias and retained
   heights. Boot still requires LEVEL support; no hand-held gain constants.
4. For motor-on noise, lift both wheels and use the operator-approved
   step2b GP21 bounded drive trial, with VIN and verified stops. It sends
   equal100 mm/s for600 mm or10 s, never a height-dependent command.
   Record the image/revision
   and motor state. Rotation/nonzero duty and the entire line CALIBRATE
   spin/pause/home episode must prevent an IMU bias refresh. A zero mean
   wheel delta during equal/opposite rotation must not count as stillness.
5. Record a stable interval's elapsed time, work/core counter deltas, maximum
   work/core/read time, maximum gap, missed/lost wakes, motion/line/vehicle
   timing, USB drops and all task high-water/free bytes. Require at least
   the existing 256 B untouched stack margin; do not infer it from size.
   Work counters include preemption, not exclusive CPU. Compare the idle
   meter too. The dfade14 measured13..14% fraction is not a step-2b bound.
6. Power off before disconnect/reconnect. Confirm counted absent-device
   faults, paced retry and responsive USB with normal motion watchdog
   ownership. This remains the separately pending absent-device gate too.

### dfade14 S4-U05 and S4-U06 procedure (historical)

1. Boot on level support and wait for calibration. IMU_STREAM runs only S4,
   reports at nominal 50 Hz and leaves the normal motion watchdog active.
   Zero duty drives both bridge inputs low: COAST, not active braking.
2. Check support/hump dimensions. Use GP20 MARK before each of three
   separate hand pushes. Save the acknowledged `MARK n`, calibration count,
   bias, fault counters and full timestamped trace, not just the largest
   printed number. Use a gentle, measured push speed near 100 mm/s first.
3. Do not stop for >=1 s on the slope/crest. Automatic stationary refresh
   in dfade14 would erase height/peak/distance and redefine level (step2b
   guards tilted refresh). Verify
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
