# Excited P7 straight-line simulation — 2026-11-16

## Answer

**The undelayed P7 cases decay. Adding20ms ADC transport produces a persistent,
slowly decaying oscillation through10s; an asymptotic steady limit cycle is not
proved. Its300–360ms period matches the real260–380ms range, but its body
amplitude33–39deg/s and comparable wheel-odometry amplitude38–44deg/s do not
match the real46–52deg/s amplitude range.**

This is a simulation prediction, not hardware or RTOS acceptance. No physical
firmware, controller gains, fitted motor response or fitted optical constants
were tuned to match the result. The explicit delay is a diagnostic assumption,
not a uniquely measured sensor-to-command transport time.

## Geometry and configuration

Geometry was committed **before** the runs: `7ea43bc` then final GP28
clarification `441cbd0`. See
[operator geometry record](ROBOT_GEOMETRY_20261116.md). All three runs use:

- Track111.5mm, wheel diameter65mm/width16mm, mass0.6716kg.
- Line forward122mm, left/centre spacing14.7mm, floor clearance9mm.
- GP28 forward73mm/left73mm, per the final operator clarification. GP28 is
  inactive in these LINE runs. No intermediate coordinate was used in a run.
- Updated body longitudinal geometry, caster82mm, sonar face85mm and IMU132mm
  floor height. Body thickness, COM, friction and component allocation remain
  assumptions, not invented consequences of the134mm total height.

The native configuration is unchanged P7: base146mm/s, gain85%, D85%, min62,
max378, far threshold50 and far gain340%. Native vehicle LINE+SEARCH and
motor control run synchronously; no scheduler or hardware timing is claimed.
All post2s windows are LINE/RUNNING, with no shell or motor fault.

The line-only fast rectangle sampler uses B optics with fixed angular
footprint scaling9/8: reference height8→9mm, sigma2.8563079834→3.2133464813mm.
This is a declared height-divergence assumption, **not a new optical fit**.
ADC endpoints and nonlinear power are unchanged; height-dependent radiometry
and tilt are not calibrated. Historical actual8mm B spin rows from
`sil-spin-B-20261115T130815Z/spin-result/calibration.json` are reused unchanged
through the native public calibration API. They are not a new9mm spin record.
There is no extra Webots spin run or GP28 numerical workload.

### Delay audit

The baseline loop already has native10ms P7/motor cadence, command application
on a following motor tick, and the existing DM1/80ms **whole-loop fitted**
response. Native command replay had already produced roughly40–60ms
command-to-wheel phase. Appending another40ms transport would double-count an
unidentified part of that response, so no such delay is added.

The third case adds exactly20ms ADC transport before native position input.
The bounded queue holds the first sample during startup. A separate fresh and
delivered ADC CSV verifies every delayed input against the original sample;
`samples.csv` raw columns are **delivered** ADC for exact native replay.
The audit finds fresh-sensor→command delay20ms and command→wheel delay60ms in
all four oscillatory windows. Correlation delay is not equivalent to a pure
transport delay; real-data phase uncertainty remains at least±40ms.

## Three authorized runs

Each ran once, sequentially, for10000ms simulated with a1200s wall cap. No retry
or additional physics run occurred. Existing DISPLAY=:0 and the approved
Xwayland authority were used; no display server or performance setting changed.

| Case | Start | Added ADC transport | Wall s | SEARCH |
|---|---|---:|---:|---:|
| offset10-yaw-10 | +10mm/−10deg | 0ms | 2.8211 | 0 |
| offset15-yaw0 | +15mm/0deg | 0ms | 2.8211 | 1, found at885ms |
| offset10-yaw-10-delay20 | +10mm/−10deg | 20ms | 2.8224 | 0 |

The +15mm start is initially outside sensor reach. Its one native SEARCH
reacquisition is retained, not described as uninterrupted P7 from startup.
All cases finish with LINE/RUNNING, fault0, junction0. Owned process groups
1123970,1124050,1124130 are gone. The recorded ADC/count inputs replay through
native C with matching outputs for all three runs; this is not repeated
physics or reproduced real timing.

## Post2s results

Amplitude is half the95th-minus5th percentile range. `Yaw rate` is finite
change of **Webots body heading**, not integer encoder speed. `Wheel yaw` uses
20ms-grid samples of10ms integer count differences, firmware12414counts/m,
and track111.5mm; it is comparable to real KR odometry. Raw C-L is delivered
ADC. No quiet or oscillatory window is removed from these tables.

| Case | Window s | Period ms | Yaw rate deg/s | Wheel yaw deg/s | Raw C-L ADC | Position amplitude |
|---|---|---:|---:|---:|---:|---:|
| +10/−10, no added delay | 2–4 | — | 0.356 | 4.625 | 24.5 | 0.5 |
| same | 4–6 | — | 0.125 | 4.625 | 11 | 0 |
| same | 6–8 | — | 0.123 | 4.625 | 10.5 | 0 |
| same | 8–10 | — | 0.148 | 4.625 | 21 | 0 |
| +15/0, no added delay | 2–4 | — | 0.894 | 4.625 | 71 | 1.5 |
| same | 4–6 | — | 0.159 | 4.625 | 35.5 | 0 |
| same | 6–8 | — | 0.102 | 4.625 | 4.5 | 0 |
| same | 8–10 | — | 0.110 | 4.625 | 7 | 0 |
| +10/−10, added20ms | 2–4 | 360 | 39.146 | 41.623 | 2205 | 53 |
| same | 4–6 | 320 | 37.923 | 43.678 | 2115 | 54.5 |
| same | 6–8 | 320 | 34.661 | 37.512 | 2009.5 | 53.5 |
| same | 8–10 | 300 | 33.440 | 39.567 | 1918.5 | 53 |

The baseline4.625deg/s wheel metric is encoder quantization; actual body yaw
is near zero. The delayed case has many repeated cycles after2s, but body and
raw-balance amplitudes still decrease, and period changes. Call it persistent
oscillation on this finite record, not proof of a fixed asymptotic limit cycle.
The delayed case's yaw-angle amplitudes across these windows are2.215,1.901,
1.712,1.562deg. Native calibration adaptation and the assumed loop remain part
of the result. No authorized extension beyond10s was run.

## Evidence, reproduction and checks

Ignored evidence root: `build/webots/taskA-excited-20261116T150638Z/`.
Original per-case manifests, process records, results, fresh/delivered ADC and
10000-row sample files are retained unchanged. Supplemental `audit.json`
checks all launch artifact hashes, final geometry, exact delay sequence,
process cleanup and comparable wheel metrics. Audit SHA256:

```text
667524458cb71158070c890e2d9d68e701127939ce48e31003747d4559b5dc3d
```

Sample CSV SHA256 in the same case order:

```text
cad9266783272ab33c5fbe13604c7b397fe27e8b3fd94692c59cf5465bb393f9
176d1990303ed1f5840071306c17d81838f792c4afde5a01c7168291aa04bf47
8da68c9c09dac52a02e1f6ef8f5982de2f2ea402f45ad090f6b69b3224b20163
```

`tools/sil_excited_line.py` accepts one named case and a fresh output path;
its cap is fixed1200s and it does not retry. Any future physics run still needs
separate authority. Offline audit does not launch Webots:

```sh
python3 sim/webots/tools/sil_excited_line_audit.py \
  --root build/webots/taskA-excited-20261116T150638Z \
  --output build/wip/taskA-audit-check.json
PYTHONPATH=sim/webots python3 -m unittest discover \
  -s sim/webots/tests -p test_sil_excited_line.py
```

Geometry/course/model checks pass24tests; targeted excitation/native motor/
SEARCH checks pass10tests. Full Python suite passes363tests with3expected
failures; the614native C tests passed in the preceding analysis delivery and
native firmware source is unchanged here. Frozen replay matches158RX/58TX,
INCOMPLETE; documentation checks pass136files with0broken links/anchors. Historical hardware Gate1 remains INCOMPLETE/NOT PASS and
physical GP28 remains FAIL. Nothing in this experiment changes that status.
