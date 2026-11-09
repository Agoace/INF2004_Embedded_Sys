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
| S4-U05 | Three hand pushes and later driven RIGHT-wheel hump trials | Hand-push finding/offline analysis below; step 2b/2c driven trials were not height acceptance. **Driven PASS on 0207b54 (step 2d) for the 10 mm RIGHT-wheel hump only**; see [step 2d hardware](#step-2d-hardware-on-0207b54) |
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

### Step 2d hardware on 0207b54

Operator-supplied numbers only (2026-11-09, IMU_STREAM image, floor, VIN).
Private log name `build/mtk_logs/s4_step2d_floor_20261109_173321.txt`
was NOT opened or copied. An earlier attempt the same day was lost: the
USB logger died when the laptop rebooted. Only its drive-3 end values were
seen (peak 10.76 mm, plateau 10.14 mm); they are not part of this result.

Setup: 10 mm triangular hump (195 mm long) under the RIGHT wheel, three
GP21 drives at 100 mm/s for 600 mm, LEVEL start. Drives 1 and 2 had GP20
MARK before them. Drive 3 had NO GP20 MARK, so its values come OFFLINE
from the stream, not from firmware maxima.

| Drive | Mean peak mm (firmware unless noted) | Plateau (200 ms median max) mm |
| --- | ---: | ---: |
| 1 | 10.93 | 10.72 firmware |
| 2 | 10.85 | 10.20 firmware; offline 9.92 |
| 3 | 10.44 (offline) | 10.31 offline |

| Check | Result |
| --- | --- |
| Flat stretches 50..225 mm and 450..600 mm | `roll_zR` sd 0.50..0.68 mm, largest single sample 1.2 mm, offline 200 ms median max 0.6 mm. Step 2c: sd 1.6..2.7 mm, false peak 5.7 mm |
| `[S4_VIB]` whole-drive std, hump included | Pitch 1.38..1.54 degrees, roll 1.67..1.72 degrees (step 2c pitch 2.95..4.14, roll 1.80..2.05) |
| `[S4_FIFO]` | ctrl1 77, ctrl5 40, fifo_ctrl 80; polls 45000, samples 176085, mean 3.913 per poll, min 3, max 6; empty, overrun, backlog, bad and reset all 0 |
| `[S4_DIAG]` | reads 45000, faults 0; read_us average 1315, max 1610 (2 ms deadline held); work_us average 2699, max 3224; core_us average 1164, max 1430; gap max 18.4 ms; missed 0; still std_mg 2/2/2 |
| CPU | work_us average is about 27 % of core 0 wall time (step 2b: 1562 us, about 16 %). Most of the increase is busy-wait I2C. FOLLOW-UP item; not exclusive CPU time |
| Raw capture | Three captures of about 781 samples at 400 Hz. Every raw word is a multiple of 64: the FIFO keeps 10-bit data, so one count is 4 mg (**hardware-confirmed**) |
| Raw spectrum, offline, first 0.5 s skipped | sd X about 76..85 mg, Y 170..181 mg, Z 100..132 mg; share below 3 Hz about 0 % (X, Y); main lines near 25 Hz (X, Y) and 85..97 Hz (Y, Z). The 4-sample boxcar lowers sd X by about 40 %, Y by about 50 % |
| IMU14/TEST20 stack high-water | Not supplied |

Operator conclusion: aliasing of the roughly 90 Hz motor/gear vibration
into the 100 Hz samples was the main step 2c error. Step 2d removes it.
The hump target of 9..11 mm and the flat target of less than +/-1 mm are
met on the floor. **S4-U05 driven PASS for the 10 mm RIGHT-wheel hump
only.** Other heights, a mat surface and curves are **Not run**. The
mean 3.913 samples per poll means a sensor rate near 391 Hz against the
kernel tick (derived, not a separate measurement).

### Step 3 hump report: H1 hardware on 1480b37

Step 3 adds the report-only hump detector; see
[IMU step 3](../../docs/s4-imu/IMU.md#step-3-hump-report).
H1 PASS on `1480b37`, 2026-11-09, image
`mtk3pico_smp0_usb_cdc_test_imu_stream.uf2`. These are operator-supplied
hardware results. The private log
`build/mtk_logs/s4_step3_h1_20261109_191034.txt` was not read or copied.

Setup: floor, 10 mm hump under the RIGHT wheel, three GP21 drives at
100 mm/s for 600 mm. The operator omitted GP20 before drive 3; GP20 is
not needed for hump totals because GP21 starts a new run and resets them.
No separate flat-only drive result was supplied.

| Drive | `[S4_HUMP]` peak_0.1mm | Count | Flags | Gated | Long |
| --- | ---: | ---: | --- | ---: | ---: |
| 1 | 99 (9.9 mm) | 1 | 01 | 0 | 0 |
| 2 | 113 (11.3 mm) | 1 | 01 | 0 | 0 |
| 3 | 105 (10.5 mm) | 1 | 01 | 0 | 0 |

| Check | Hardware result |
| --- | --- |
| DIAG | faults 0; read_us 1262, max 1588; work_us 2730, max 3288; core_us 1067, max 1598; missed 0 |
| FIFO | empty, overrun, backlog and bad all 0; mean 3.913 samples per poll |
| STACK, STREAM image above | imu_task 1448/2048 B used, 600 B free (dfade14: 752 B used); sonar 320/2048 B; telemetry 464/4096 B |

The IMU stack has more than the required 256 B margin in this STREAM
image. This did not validate the ROBOT image; its later named stack
reading is recorded below with H0/H2-short. H1 confirms one event per drive;
11.3 mm in drive 2 is recorded, not hidden by a 9..11 mm height claim.

| ID | Check | Result |
| --- | --- | --- |
| H0 | MQTT LINE run, no hump: core_us/work_us, Wi-Fi gap, overruns, IMU stack high-water | Partial: 5 flat runs across d51d27b/29d57de, no false humps; diagnostic C IMU free 480 B; timing/network baseline still needed |
| H1 | STREAM floor repeat over the 10 mm RIGHT hump: `[S4_HUMP]` | PASS on 1480b37: count 1 each, peaks 9.9/11.3/10.5 mm, flags 01, gated/long 0; separate flat-only drive not supplied |
| H2 | LINE run over the 10 mm hump on a straight: USB line, kind 9, summary v3 | 9/9 counted across d51d27b/29d57de; uncorrected 11.5/12.4 mm accepted as good enough; corrected +4..6 mm under investigation |
| H3 | Full circuit, curves and SEARCH, no false hump: count 0, record `gated` | Not run; PRIORITY acceptance test when the full circuit is available |
| H4 | Hump right after a curve: count, CURVE flag | Not run |
| H5 | Course mat, no hump: count 0 | Not run |
| H6 | Other hump heights: peak against height and speed | Not run |
| H7 | LEFT-wheel hump: side LEFT (bonus) | Not run |
| H8 | GP21 stop on the hump: OPEN flag, counted | Not run |
| H9 | Broker down during the run: USB and DUMP lines, summary after reconnect | Not run |

### Step 3 H0 and H2-short on d51d27b

Operator-supplied hardware evidence, 2026-11-09. MQTT robot image
`mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`, revision `d51d27b`. Seven
runs on two tiles, LINE about 146 mm/s. USB was unplugged during the
runs; values came from the MQTT recorder. The operator confirmed the
order: 1 hump, 3 flat, 3 hump. The robot entered SEARCH at the line end;
the operator then pressed dashboard ESTOP (stop_reason 2, stop_source 2).
The line end itself is not stop_reason 2.

| Run | Object | Distance mm | Duration ms | Peak 0.1 mm | Count | Gated | Flags |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 10 mm RIGHT hump | 440 | 3170 | 140 | 1 | 4 | 09 (implemented, curve) |
| 2 | Flat | 412 | 2950 | 0 | 0 | 3 | 01 |
| 3 | Flat | 431 | 3190 | 0 | 0 | 2 | 01 |
| 4 | Flat | 416 | 2950 | 0 | 0 | 0 | 01 |
| 5 | 10 mm RIGHT hump | 408 | 2950 | 129 | 1 | 1 | 01 |
| 6 | 10 mm RIGHT hump | 447 | 3250 | 136 | 1 | 2 | 01 |
| 7 | 10 mm RIGHT hump | 469 | 4111 | 155 | 1 | 4 | 0d (implemented, curve, open) |

IMU fault, long and left flags were zero throughout. Kind 9 words
140/129/136, detail 1, arrived at run-relative 2980/2630/3200 ms for
runs 1/5/6. Run 7 had no kind 9: its ACTIVE event was virtually closed
only in the end summary (OPEN), not as a live event. These event times
are closure times, not the instants of the height peaks.

- Three flat runs: no false alarms. Four hump runs: one count each, but
  height 12.9..15.5 mm for the 10 mm object, about 29..55 % high.
  **Not a height PASS.** STREAM at 100 mm/s gave 9.9/11.3/10.5 mm.
- Heartbeat: cpu_load 411..637 permille, vehicle_overruns 0,
  motion_missed 0, stack_min_free 272..328 B; wifi_poll_gap_max up to
  280190 us, publish_ok 1071, publish_failed 1196. No comparable baseline
  was supplied: no CPU/network acceptance is inferred from these counts.
- Same boot, USB reconnected and GP20 DUMP: hump peak 155, count 1,
  flags 0d, gated 4 (run 7). KR1 diagnostics: overruns 0, lost_wakes 0,
  faults 4 (`MC_FAULT_ESTOP`, dashboard ESTOP), gap 9971..10026 us.

| Task, same robot boot | Stack B | High-water B | Free B |
| --- | ---: | ---: | ---: |
| motion | 3072 | 440 | 2632 |
| line | 2048 | 288 | 1760 |
| sonar | 2048 | 368 | 1680 |
| imu | 2048 | 1456 | 592 |
| vehicle | 4096 | 1504 | 2592 |
| telemetry | 4096 | 728 | 3368 |
| health | 2048 | 568 | 1480 |
| idle_meter | 512 | 240 | 272 |

Heartbeat stack_min_free 272 B was idle_meter, not IMU. The named robot
IMU stack margin (592 B) exceeds 256 B on d51d27b; the later diagnostic C
measurement is below. Stack marks cover the boot, not one run.

### Shared floor-test condition and diagnostic C

Operator confirmation: the front-LEFT barcode sensor (about 5.5 mm above
the floor) scraped on the 10 mm hump in ALL earlier floor drives,
including step 2d and H1 STREAM. Their 10..11 mm results already include
the scrape. Record contact in each trial. It can change support geometry
or add drag/yaw, but is not by itself the difference between STREAM and
LINE. In LINE the robot also zigzags slightly while it climbs (operator
observation); STREAM commands equal wheel speeds.

Compensation lag is a hypothesis, not measured evidence. The 100 ms
encoder window and filtered yaw derivative can lag accelerometer
transients. Diagnostic C records paired heights and motion at the actual
selected peak; see [diagnostic C](../../docs/s4-imu/IMU.md#diagnostic-c-peak-context).
The approved comparison leaves at least 200 mm of flat line after the
hump before SEARCH/stop (3 hump + 2 flat trials). That comparison and
diagnostic C results are below. Further accuracy work is now deferred by
the operator; keep corrected as the default and thresholds unchanged.

### Diagnostic C hardware on 29d57de

Operator-supplied evidence, boot ID `1600432639`, MQTT robot image
`29d57de`, LINE mode, 10 mm RIGHT-wheel hump. Measured speed at the two
captured hump peaks was about 120 mm/s. Seven runs: five hump, two flat.
The operator confirmed the wheel had physically cleared the hump before
STOP in ALL five hump runs. The private recordings were not opened.

| Run | Surface | Peak 0.1 mm | Count | Gated starts | CURVE | OPEN |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | Hump | 163 | 1 | 6 | 1 | 1 |
| 2 | Hump | 151 | 1 | 4 | 1 | 0 |
| 3 | Hump | 152 | 1 | 2 | 0 | 1 |
| 4 | Flat | 0 | 0 | Not supplied | Not supplied | Not supplied |
| 5 | Flat | 0 | 0 | Not supplied | Not supplied | Not supplied |
| 6 | Hump | 147 | 1 | 5 | 0 | 0 |
| 7 | Hump | 140 | 1 | 2 | 1 | 1 |

USB was unplugged at run end for runs 1..3, so their diagnostic lines
were lost. Only runs 4..7 have diagnostic lines; do not infer peak motion
for runs 1..3 from another run.

| Diagnostic field | Run 6 | Run 7 |
| --- | ---: | ---: |
| peak_ms / travel_mm | 1942 / 252 | 1852 / 232 |
| corrected_0.1mm / filtered_0.1mm | 147 / 124 | 140 / 115 |
| speed_mm_s | 122 | 118 |
| yaw_mrad_s / yaw_accel_mrad_s2 | 160 / -1379 | -87 / -3127 |
| lateral_mg | 6 | 20 |
| win_yaw_mrad_s / win_yaw_accel_mrad_s2 | 1241 / 8056 | 1168 / 6301 |
| win_lateral_mg | 66 | 46 |
| gate / win_gate | 00 / 02 | Endpoint mask not supplied; listed motion within limits / 02 |
| line_lateral_mg / line_yaw_accel_mrad_s2 | 66 / 8056 | 46 / 6301 |

Flat runs 4/5: run-wide LINE max |lateral| 36/61 mg and max |yaw
acceleration| 4667/7507 mrad/s^2, respectively; no hump events.
Zigzag/steering therefore occurs even on flat LINE, not only on a hump.

Findings (hardware observations, not a proven compensation diagnosis):

- Compensation adds 2.3/2.5 mm to the paired mean at the accepted peaks.
  This is consistent with the lag hypothesis, not proof of its cause.
- Both peak windows include lateral-gated samples (46..66 mg, win_gate
  02), despite no curve gate at the accepted endpoint. The current rule
  gates the endpoint only; its LPF/mean retains earlier gated content.
- Uncorrected heights remain 11.5..12.4 mm: still +1.5..2.4 mm above
  the object, so compensation does not explain all the error.
- OPEN means the signal had not completed continuous below-OFF travel at
  STOP, NOT that the wheel was still on the hump. Physical clearance was
  confirmed by the operator; filtering/steering can keep the signal open.
- Named IMU stack: free 536 B initially (high-water 1512/2048), then
  free 480 B after hump runs (high-water 1568/2048). This leaves 224 B
  above the 256 B free policy. C's sample frame is 1000 B; the new
  diagnostic step/latch frames are 56/16 B. Hump paths exercise latching
  and event processing that flat paths may not reach. The 56 B observed
  increase is consistent with the diagnostic-step frame, but not proof
  of the exact deepest call chain or interrupt contribution.

**Operator correction:** UNCORRECTED (filtered-roll) LINE readings
11.5/12.4 mm (about +1.5..2.5 mm high) are ACCEPTED as good enough.
CORRECTED +4..6 mm remains under investigation with the deferred trace
plan. Default stays CORRECTED; no default switch. Detection
across the two robot sessions is **9/9 humps** (4 on d51d27b + 5 here),
with **0 false humps in 5 flat runs** (3 + 2). This does not establish
no false humps on the full circuit. H3 (curves + SEARCH, full circuit) is
now the priority acceptance test when that circuit is available: compare
BOTH sources with two identified images. H3 and
H4..H9 remain **Not run**. This hardware-record commit changes no code or thresholds.

Accuracy follow-up is deferred; see
[deferred plan](../../docs/s4-imu/IMU.md#deferred-accuracy-follow-up).

### H3 two-image comparison plan

**Not run; priority acceptance test.** On a fuller circuit compare curves
and SEARCH with NO hump, using both MQTT robot images. The operator
accepts uncorrected 11.5/12.4 mm height, but false-hump behaviour of that
source is not established. The recorded 9/9 detections used the corrected
default, not a separate filtered-source trial. Default stays corrected. Keep all thresholds
and the diagnostic C capture unchanged; do not implement deferred trace
or window-gate work in this comparison.

| Source | Image |
| --- | --- |
| 1 corrected (default) | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` |
| 2 filtered roll | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_humpfilt.uf2` |

Use [build commands](../../docs/kernel/BUILD.md#h3-source-comparison-images).
Record filename, `[S4] hump_source N NAME` boot line, boot ID, each run
ID, source-labelled hump lines, summary/count/gates, and named stack
lines. Keep USB attached at run end; reconnecting later cannot recover a
lost peak diagnostic unless GP20 still holds that last run. Use comparable
speed, path, contact and stop/SEARCH conditions for both images. Require
zero false events/counts; record misses on any separate known-hump controls
without conflating those controls with the no-hump circuit test. Preserve
the 256 B free-stack policy. MQTT fields are unchanged and do not encode
the source; do not combine unidentified sessions. H4..H9 remain Not run.

Local source-option checks: host 171/171, sanitizers 171/171; both MQTT
robot builds warning-free, 16 strict ARM compilations, 2 SDK link proofs.
No H3 image was flashed by the worker. Public example build artifacts do
not contain the operator's broker settings; rebuild configured images
before hardware use.

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
