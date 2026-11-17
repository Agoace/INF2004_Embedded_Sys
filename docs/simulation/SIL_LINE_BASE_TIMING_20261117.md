# LINE base-speed and same-tick prediction — 2026-11-17

## Result

**Lower base speed or one less command-wait slot does not give a uniformly
stable result in this model.** Base100/20ms decays; base120/30ms retains a
small380ms oscillation. Base100/30ms enters SEARCH six times. Same-tick/20ms
enters SEARCH once and then retains a smaller280ms oscillation; same-tick/30ms
largely resembles the earlier next-tick/20ms case.

The real hardware result supplied by Main is P85/60/40 yaw RMS23.1/21.4/21.3,
absolute peaks66/66/82deg/s; P40 enters SEARCH. Real far-region occupancy is
14–15%, with mean absolute requested wheel difference124–152mm/s there.
This contradicts the earlier model's strong P60/P40 improvement at20ms and
limits confidence in these new predictions. These hardware summary values
are Main-supplied context; this task does not independently reconstruct the
real row filter, route, pose or calibration state. Its LINE-only denominators
must not be silently treated as identical to an unverified real denominator. **Do not adopt a firmware setting
from this simulation alone.** These results do not identify delay or the far
term as the unique real cause.

## Scope and source

Main approved exactly six new sequential10s physics runs,1200s wall cap each,
no retries. The two existing P85/20ms and P85/30ms recordings were analyzed
without a new physics run or changes to their records. The new cases are:

- Base120 and100, each with20and30ms explicit ADC transport; min62, P85, D85,
  far threshold50 and extra gain340 unchanged.
- Base146 with same-tick command application, each with20and30ms transport.

The four lower-base cases use SW9's real adapter and sources from read-only
Git archive `1311338c2ef24f4d3722077bc59f4bcea211319a` (`5e08d3f` adds docs).
Neither commit is merged into this branch. `line_p_near.h` changes only the
selected base in these runs. It is forced into the simulator translation unit
that owns LINE_CFG, not globally into steering/motor source. The same-tick
cases use the same source snapshot with default P85/base146 and no LINE
configuration adapter. Actual native metadata is checked in every build.
The generic `controller_config` field in these old-style SIL manifests is
unused legacy plumbing; **native metadata**, LINE_BASE and the source/build
record specify the actual control configuration, not that legacy field.

The original formula scales the complete correction:
`delta = (base-min)*turn/10000`. Relative to base146/min62, base120 scales P,
D and far correction by58/84=0.6905; base100 scales them by38/84=0.4524,
subject to original integer rounding and saturation. Lower base is therefore
**not an isolated forward-speed change**. SEARCH speed131mm/s is unchanged.

All cases retain the geometry, fixed9/8 angular footprint scaling, historical
8mm B spin calibration rows and DM1/80ms whole-loop fitted response from
[the excited P7 experiment](SIL_EXCITED_P7_20261116.md). The supplemental audit
checks exact equality of robot parameters, optical profiles, mounts and
calibration rows across all eight recordings. No physical calibration,
friction, optical fit or plant response is adjusted to obtain these results.

### What same-tick means here

The simulator's normal order is `mc_tick → shell/P7 step → mc_set_speed`.
The experimental order is `shell/P7 step → mc_set_speed → mc_tick`, in the
same10ms slot. The adapter's default path is unchanged. No RTOS, firmware task
ordering or motor algorithm is edited. This model removes **one10ms slot**;
it does not pretend that the operator's approximate8ms hardware wait was
measured as exactly10ms. No extra40ms motor transport is appended.

A native timing ABI records each tick, the command timestamp and target Q8
values actually presented to `mc_tick`. Each new run has1000 timing rows:

- Normal order:999 active-command rows, all age10ms; the first tick has no
  command yet.
- Same-tick order:1000 active-command rows, all age0ms.

Every active target matches the requested speed at the recorded command
publication timestamp using the unchanged firmware count conversion. There
are no timing/target mismatches. The default instrumented adapter and new SW9
snapshot also replay **both old10000row P85 streams exactly**, before any new
launch. This verifies no default native-output change from instrumentation.
Fault observations necessarily occur in the changed order; these experiments
have no native fault and do not establish equivalent hardware safety behavior.

## Windows and definitions

The requested0–3s window is simulated samples1..3000, immediately after the
historical calibration API finishes. There is no fabricated time0 sample.
It is placed next to2–4,4–6,6–8 and8–10s windows. A physical post-spin pose,
adaptive state or elapsed calibration sequence is not independently matched
by this historical-row calibration; similar clock labels do not prove that
real and simulated initial conditions are equal.

- Amplitude means half the95th-minus5th percentile range, not RMS or peak.
- Body yaw rate is derived from Webots heading. Wheel yaw is a10ms integer
  count-difference speed snapshot, sampled every20ms, using firmware12414
  counts/m and physical111.5mm track. It is comparable to KR odometry, not an
  independent body-heading measurement.
- Position bands are exactly `|pos|<45`, `45<=|pos|<=52`, `|pos|>52`.
  The third band is a diagnostic subset of the far term, which starts beyond
  50, not52. Mean far request is `mean(|askedL-askedR|)` in the third band.
- Primary band tables use **RUNNING/LINE20ms rows** to approach KR cadence.
  The JSON also retains all-module and LINE-only1ms/20ms denominators. Post2s
  windows include both endpoints (usually101grid rows);0–3s has150grid rows.
  Overlapping windows are not independent experiments.
- Motion amplitudes below use the full fixed window. A window containing
  SEARCH is marked MIXED and gets **no LINE-cycle period claim**. A matching
  mixed amplitude does not constitute a match to real straight P7 motion.
- Period is a finite-window autocorrelation candidate, not proof of a steady
  cycle. The long0–3s startup periods must not be described as sustained.
  Quantized wheel yaw remains4–5deg/s even when actual body motion is tiny.

## (a) Existing P85 far-share, no new run

Both existing recordings are LINE throughout. Bands and mean far request:

| Existing case / interval | Row grid, N | <45 % | 45–52 % | >52 % | Far mean request mm/s |
|---|---|---:|---:|---:|---:|
| P85/20ms,0–3s | 1ms,3000 | 86.20 | 11.37 | 2.43 | 106.0 |
| same | 20ms,150 | 86.00 | 10.67 | 3.33 | 120.4 |
| P85/20ms,0–10s | 1ms,10000 | 54.19 | 36.06 | 9.75 | 108.8 |
| same | 20ms,500 | 53.40 | 36.20 | 10.40 | 120.5 |
| P85/30ms,0–3s | 1ms,3000 | 41.43 | 48.57 | 10.00 | 106.5 |
| same | 20ms,150 | 40.00 | 48.67 | 11.33 | 113.2 |
| P85/30ms,0–10s | 1ms,10000 | 38.97 | 50.48 | 10.55 | 104.7 |
| same | 20ms,500 | 43.20 | 45.60 | 11.20 | 103.4 |

Thus the model does not reproduce the real14–15% far share and124–152mm/s
far request in the comparable first3s. A late-window far share closer to14%
must not be substituted for that first3s result. The small20ms far denominator
(5or17rows in0–3s) also makes the mean sensitive to sample phase.

For direct comparison to the new0–3s results:

| Existing case | Period ms | Body amplitude deg/s | Wheel amplitude deg/s | Raw C-L ADC | Wheel RMS / absolute peak deg/s |
|---|---:|---:|---:|---:|---:|
| P85/20ms,0–3s | 340 | 27.790 | 31.346 | 1943.5 | 19.0 /41.6 |
| P85/30ms,0–3s | 380 | 37.025 | 42.753 | 2234.0 | 30.1 /54.0 |

Existing post2s references, all LINE20ms windows with N101:

| Case / window s | Period ms | Body amplitude | Wheel amplitude | C-L ADC | <45 /45–52 />52 % | Far mean request |
|---|---:|---:|---:|---:|---|---:|
| P85/20,2–4 | 360 | 39.146 | 41.623 | 2205.0 | 55.4/33.7/10.9 | 137.1 |
| P85/20,4–6 | 320 | 37.923 | 43.678 | 2115.0 | 36.6/47.5/15.8 | 122.0 |
| P85/20,6–8 | 320 | 34.661 | 37.512 | 2009.5 | 43.6/44.6/11.9 | 115.3 |
| P85/20,8–10 | 300 | 33.440 | 39.567 | 1918.5 | 39.6/48.5/11.9 | 109.8 |
| P85/30,2–4 | 360 | 36.835 | 43.678 | 2203.0 | 39.6/48.5/11.9 | 105.7 |
| P85/30,4–6 | 360 | 36.045 | 41.623 | 2171.5 | 45.5/43.6/10.9 | 99.3 |
| P85/30,6–8 | 360 | 36.000 | 41.623 | 2171.0 | 45.5/43.6/10.9 | 100.2 |
| P85/30,8–10 | 360 | 35.911 | 41.623 | 2174.0 | 44.6/44.6/10.9 | 98.9 |

Amplitudes are deg/s; far requests are mm/s. No reference physics was rerun.

## (b/c) Six new runs

| Case | Wall s | Owned PGID, now gone | SEARCH count |
|---|---:|---:|---:|
| Base120 /20ms | 3.5785 | 40611 | 1 |
| Base120 /30ms | 3.1244 | 40752 | 0 |
| Base100 /20ms | 3.2749 | 40861 | 0 |
| Base100 /30ms | 3.2254 | 40975 | 6 |
| Same-tick base146 /20ms | 3.3767 | 41082 | 1 |
| Same-tick base146 /30ms | 3.3260 | 41202 | 0 |

All finish10000rows, exit0, matching recorded-input native C replay, final
LINE/RUNNING/fault0, no junction. Completion means the experiment finished,
not successful tracking. Exactly six physics launches occurred; no retry.

In the next table `B` is body yaw-rate amplitude, `W` is KR-like wheel yaw
amplitude, `C-L` is raw delivered-ADC amplitude. `bands` are LINE-only20ms
percentages in the order<45/45–52/>52. `N` is that LINE denominator; MIXED
means motion amplitudes include recovery, although bands exclude SEARCH.

| Case | Window s | LINE period ms | B deg/s | W deg/s | C-L ADC | bands % | N |
|---|---|---:|---:|---:|---:|---|---:|
| Base120/20 | 0–3 MIXED | — | 31.379 | 34.532 | 2310.1 | 70.7/20.4/8.8 | 147 |
| same | 2–4 MIXED | — | 41.292 | 47.789 | 2264.0 | 45.9/33.7/20.4 | 98 |
| same | 4–6 | 360 | 13.333 | 16.444 | 1170.0 | 98.0/2.0/0 | 101 |
| same | 6–8 | 340 | 12.045 | 12.333 | 1016.5 | 100/0/0 | 101 |
| same | 8–10 | 340 | 12.348 | 14.388 | 1001.5 | 100/0/0 | 101 |
| Base120/30 | 0–3 | 420 | 18.598 | 21.942 | 1765.6 | 90.7/8.0/1.3 | 150 |
| same | 2–4 | 380 | 14.282 | 16.444 | 1282.0 | 100/0/0 | 101 |
| same | 4–6 | 380 | 13.908 | 16.444 | 1244.0 | 100/0/0 | 101 |
| same | 6–8 | 380 | 14.207 | 16.444 | 1274.0 | 100/0/0 | 101 |
| same | 8–10 | 380 | 14.163 | 16.444 | 1253.0 | 100/0/0 | 101 |
| Base100/20 | 0–3 startup | 780 | 8.416 | 10.277 | 1274.5 | 86.7/12.0/1.3 | 150 |
| same | 2–4 | — | 0.539 | 4.111 | 63.5 | 100/0/0 | 101 |
| same | 4–6 | — | 0.242 | 4.111 | 44.5 | 100/0/0 | 101 |
| same | 6–8 | — | 0.138 | 4.111 | 9.0 | 100/0/0 | 101 |
| same | 8–10 | — | 0.122 | 4.111 | 11.5 | 100/0/0 | 101 |
| Base100/30 | 0–3 MIXED | — | 35.797 | 34.429 | 1987.5 | 66.0/23.6/10.4 | 144 |
| same | 2–4 MIXED | — | 48.518 | 51.643 | 2123.0 | 70.5/15.8/13.7 | 95 |
| same | 4–6 MIXED | — | 38.177 | 43.165 | 2219.5 | 66.0/20.6/13.4 | 97 |
| same | 6–8 MIXED | — | 29.017 | 32.887 | 2082.0 | 63.3/25.5/11.2 | 98 |
| same | 8–10 MIXED | — | 24.045 | 28.776 | 2014.0 | 67.7/26.3/6.1 | 99 |
| Same-tick/20 | 0–3 startup | 680 | 17.533 | 19.013 | 1286.2 | 94.7/4.7/0.7 | 150 |
| same | 2–4 MIXED | — | 43.450 | 47.789 | 2131.0 | 81.4/12.4/6.2 | 97 |
| same | 4–6 | 300 | 14.667 | 16.957 | 1047.0 | 100/0/0 | 101 |
| same | 6–8 | 280 | 16.538 | 21.068 | 1117.0 | 100/0/0 | 101 |
| same | 8–10 | 280 | 16.268 | 16.957 | 1089.5 | 100/0/0 | 101 |
| Same-tick/30 | 0–3 | 320 | 27.885 | 31.346 | 1943.5 | 86.0/10.7/3.3 | 150 |
| same | 2–4 | 360 | 39.146 | 45.734 | 2205.0 | 55.4/33.7/10.9 | 101 |
| same | 4–6 | 320 | 37.923 | 41.623 | 2115.0 | 36.6/47.5/15.8 | 101 |
| same | 6–8 | 300 | 34.661 | 39.567 | 2009.5 | 43.6/44.6/11.9 | 101 |
| same | 8–10 | 300 | 33.621 | 37.512 | 1918.5 | 39.6/48.5/11.9 | 101 |

The audit retains mean/p95/max far requests for every denominator/window.
For first3s LINE20ms far rows, means in case order are115.7,73.0,46.0,83.7,
96.0,120.4mm/s. For0–3s **all-module** wheel RMS/absolute peak:

| Case | RMS / peak deg/s |
|---|---:|
| Base120/20 MIXED | 21.3 /78.1 |
| Base120/30 | 13.0 /33.4 |
| Base100/20 | 7.9 /24.7 |
| Base100/30 MIXED | 27.0 /111.5 |
| Same-tick/20 | 11.3 /41.6 |
| Same-tick/30 | 19.6 /45.7 |

The apparent real-amplitude matches47.8or51.6deg/s occur in MIXED recovery
windows, not pure LINE oscillation. No such match is accepted as model
validation. Same-tick/30ms and previous next-tick/20ms have similar feedback
phase because adding10ms ADC transport offsets removing one10ms command-wait
slot; they are not asserted to have identical sampled wheel speeds or real
hardware dynamics.

## Evidence and reproducibility

Ignored root: `build/webots/line-base-timing-20261117T031758Z/`.
Original per-run manifests/results/sample files are unchanged. `audit.json`
is the preliminary supplement. The final `audit-v2.json` adds cross-run
geometry/optics/mount/calibration equality checks; it does not replace the
original. Final audit SHA256:

```text
3559901633e6f1d99d67672ee05c95f9af85ac874dd120ae4fb0b90d6ebd86a7
```

The audit verifies all sealed launch artifacts, ADC delay sequences, timing
rows, applied targets, sample hashes and cleanup. Each sample CSV SHA256 in
run-table order:

```text
4fbb591188bbcea559d3c426354d857f07e97bdadcd482db1f70380c86c8848e
30974121b7dcae6c4251a93ae312d46f959726375cceb160441ef7a3c7b2935a
e27e2cd5a0726b1552036bcdbc6a0faa7e55eb00c8cbead903822474a3762926
d6a88c0424a62bd89f678391847c73c74e1eac7f323564698743d93c5045d23e
9c6f0fbfe768e6dc5e87a2ed30e2010e35380bfe5b96d8570ea45051690488a8
fe7adde73f02eada81a31ab53b90c652fca9e71fe92735e04ff0538e9062fdda
```

Offline reproduction, with a fresh supplemental output:

```sh
python3 sim/webots/tools/sil_line_base_timing_audit.py \
  --root build/webots/line-base-timing-20261117T031758Z \
  --sil-build build/webots --output build/wip/base-timing-audit-check.json
PYTHONPATH=sim/webots python3 -m unittest discover \
  -s sim/webots/tests -p test_sil_line_base_timing.py
```

Prelaunch native checks pass10tests, including SW9's original base test at
146/120/100, exact timestamp/target ordering and20,000 old-row output matches.
Full Python suite passes369tests with3expected failures; fresh host build and
614native C tests pass. Frozen no-device replay matches158RX/58TX with verdict
INCOMPLETE; documentation checks pass138files with0broken links/anchors and
whitespace checks pass.
No hardware/serial access, display-server start, firmware merge or new motor
fit occurred. Hardware Gate1 remains INCOMPLETE/NOT PASS; physical GP28 remains
FAIL. This record supplies predictions, not a physical tuning recommendation.
