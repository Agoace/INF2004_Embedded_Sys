# S3 fast flat reference — F1

## Decision

**Accuracy PASS. Three-channel maximum-time gate FAIL. Runtime integration
remains blocked. No full-course or physical calibration claim.**

This is a host-only implementation at parent `9831991`. No firmware, wire,
decoder, action gate or frozen profile changes are included. The three
physical-model expected failures in [S3_SENSOR_MODEL.md](S3_SENSOR_MODEL.md)
remain. GP26/27 still load as UNMEASURED/null; no GP28 fallback is supplied.

## Entry points

| File | Purpose |
| --- | --- |
| [`s3_fast.py`](../../sim/webots/pil/s3_fast.py) | Compile disjoint visible rectangles; bounded runtime Gaussian integrals |
| [`s3_referee.py`](../../sim/webots/pil/s3_referee.py) | Independent offline boundary-aware `first_hit` integration |
| [`s3_f1_check.py`](../../sim/webots/pil/s3_f1_check.py) | Fixed accuracy corpus, full timing CSV, strict gates |
| [`test_s3_fast.py`](../../sim/webots/tests/test_s3_fast.py) | Analytic coverage, yaw, overlap and unsupported-domain contracts |
| [`test_s3_fast_guards.py`](../../sim/webots/tests/test_s3_fast_guards.py) | Noise, revision, invalidity and offset-point checks |
| [`test_s3_f1_check.py`](../../sim/webots/tests/test_s3_f1_check.py) | Gates retain referee uncertainty and every timing outlier |

## Method and domain

`FastScene.from_patches(patches)` compiles axis-aligned flat rectangles to
mutually exclusive visible material cells. Top physical Z wins before
coplanar layer priority. Conflicting top-layer materials retain an ambiguity
marker. CROSS/T overlap is counted once, and paper hides the line beneath it.
Compilation splits at rectangle boundaries and merges equal adjacent cells.
A 64 mm spatial index bounds lookup; excessive compile/query complexity
returns an error or explicit unsupported result, not an unbounded fallback.

`FastScene.from_course(built_course)` also marks whole CURVE/STATION tiles
and physical obstacle/hump regions unsupported. Both assumed home courses
and the school-like course are loaded for domain coverage, not course
acceptance. An immutable geometry/guard hash identifies the compiled map.
Pass `expected_revision=...` to detect a stale map at the caller boundary.
This is a static map adapter, not an automatic dynamic-scene updater.

`fast.sample(profile, origin_mm, direction, axis_u, sample_us=..., seed=...)`
uses Gaussian error-function integrals for material rectangle probabilities.
It then calls the unchanged material-response formula, applies time/channel
seeded post-eight-average noise once, quantizes and retains clipping.
No runtime `first_hit`, offline referee call, pose replay or cache build
occurs. Results use the existing `OpticalSample` fields; `rays=0`.

Normal-incidence isotropic kernels support arbitrary flat body yaw. Finite
Code39 bars and paper use exact separable rectangle coverage, including
lateral exit/re-entry. Curved strokes, their end/transition caches and
nonrectangular/rotated material polygons are not implemented. Anisotropic
profiles are explicitly unsupported. Pixel textures are not ADC inputs.

The domain guard checks a square enclosing every yaw of the four-sigma
reference footprint. A gap, unknown material, ambiguous material, different
physical plane or unsupported region inside that square invalidates the
sample. This can conservatively reject a pose whose smaller reference
footprint would be valid. It never substitutes white or a stale value.
Height/tilt extrapolation is not enabled. Sensor position remains rigid;
firmware mean encoder distance is not replaced by sensor travel.

Coverage integrates a world-aligned four-sigma square, whereas the ray
reference's square follows the sensor axes. The small Gaussian tail
contribution is included in the accuracy measurements, not added as noise.
The frozen 4.4478 ADC noise and recorded waveform failures remain separate.

## Independent fine referee and accuracy

The offline referee uses ORIGINAL public scene polygons to project all
boundary edges into the optical plane. At each integration row, original
edges split ray intervals. Actual `first_hit` selects the visible material
of each interval. The runtime's compiled material cells are never consulted.
Original edge intersections and support-edge intersections split topology
changes. Adaptive two-point Gaussian integration refines material mass;
exhausted convergence or invalid rays produce an invalid reference result.

The certification corpus uses both 1e-6 and 1e-8 material-mass tolerances.
Their ADC difference is included conservatively in the fast-path error gate.
Independent exact straight-strip checks and dense first-hit ray spots test
against shared integration mistakes; agreement of two resolutions alone is
not claimed as proof against every possible geometry.

| Fixed requirement | Observed |
| --- | ---: |
| Maximum ADC error including referee difference <=8 | 0.257002 |
| P95 ADC error including referee difference <=2 | 0.214009 |
| Maximum referee tolerance difference <=1 ADC | 0.00002195 |
| P95 referee difference <=0.25 ADC | PASS |
| Prescribed valid corpus cases | 209; all valid |
| Fast valid / referee invalid | 0 |
| Independent 65536-ray strip spots | 3; maximum difference 0.039361 ADC |

Every geometry-family subgroup passes the same 8/2 limits: straight, CROSS,
T, all 26 Code39 letters/reversals, finite paper/bar edges and tile seams.
Flat yaw includes oblique angles, not only quarter-turns. This validates the
frozen mathematical model on the fixed F1 corpus, not physical ADC accuracy.

Representative real-layout probes report: first home16 unsupported curve
poses; second home16 unsupported curves and8 valid poses; school-like16
unsupported curves,31 valid and1 unsupported obstacle. Do not discard these
unsupported results to claim a full-course simulation.

## Initial strict timing result — retain FAIL

The timer includes source-pose generation/transforms, lookup, domain guards,
coverage, response, noise, quantization and three result objects. Each batch
has frozen GP28 plus two explicitly labelled synthetic timing profiles,
not calibrated GP26/27. Synthetic positions are not mount surveys. Null/
invalid calls do not count toward a valid three-channel pass.

A short warm-up precedes three repeats of100000 source instants. All300000
batches had three valid channels. GC remained enabled. Every elapsed sample
and thread-CPU time was retained in CSV. No outliers were removed.

| Repeat | P95 us | P99 us | Maximum us | Count >500 us | Gate |
| --- | ---: | ---: | ---: | ---: | --- |
| 0 | 71.765 | 90.570 | 990.433 | 82 | **FAIL** |
| 1 | 70.824 | 88.737 | 969.855 | 81 | **FAIL** |
| 2 | 70.382 | 84.900 | 2877.243 | 98 | **FAIL** |

Limits stay P95<=150us, P99<=250us, maximum<=500us for the THREE-channel
batch. Low percentiles do not make a failing maximum pass. Of these elapsed
outliers,74/75/86 also exceeded500us of thread CPU time. The largest was
2877.243us wall /2861.560us thread CPU and overlapped GC. Most outliers cannot
be attributed to scheduler delay alone. The initial harness retains Python
metric tuples, so observer-induced GC is one candidate, not a proven sole
cause. Keep these initial reports when investigating.

Main authorized ONE bounded paired investigation after the initial FAIL
commit `4d12025`. The results below do not overwrite that FAIL or weaken500us.

## One paired refinement investigation — both modes still FAIL

The refinement uses fixed numeric measurement arrays instead of retained
per-frame metric tuples. Only zero-noise synthetic channels skip private RNG
construction; the nonzero GP28 path and frozen profile are unchanged. Pose
math, domain checks, response, noise and result construction remain inside
the timer. Python scalar and sampler-result allocations still exist: this
is **not an allocation-free sampler**. Measurement payload is33 bytes per
frame (3.3MB for100000 frames, excluding array headers). No complete
sensor-pose replay is precomputed.

The same source and generated poses ran exactly once in each mode, three
repeats of100000 batches per mode. All600000 batches had three valid channels.
GC-on is the control. The experiment explicitly collects/freezes the existing
heap and disables cyclic GC during each timed loop, then restores the prior
policy. GC setup/restore and CSV export are outside the sample timer. This
policy exists in the evidence driver only, not in the runtime sampler.
No reruns or further tuning were done after these paired measurements.

| GC on | P95 us | P99 us | Maximum us | Count >500 us | Outlier thread CPU >500 us |
| --- | ---: | ---: | ---: | ---: | ---: |
| Repeat0 | 60.003 | 75.772 | 599.038 | 48 | 36 |
| Repeat1 | 59.412 | 63.790 | 2157.581 | 49 | 32 |
| Repeat2 | 58.621 | 61.556 | 603.265 | 48 | 29 |

| Freeze + disable GC | P95 us | P99 us | Maximum us | Count >500 us | Outlier thread CPU >500 us |
| --- | ---: | ---: | ---: | ---: | ---: |
| Repeat0 | 59.532 | 71.374 | 694.256 | 57 | 44 |
| Repeat1 | 59.492 | 63.299 | 638.852 | 38 | 28 |
| Repeat2 | 58.831 | 62.217 | 568.080 | 23 | 16 |

**Every repeat fails the unchanged500us maximum.** No paired-run outlier
overlapped a GC callback. Thread CPU measures execution by the measured
thread; elapsed time also includes time when it does not execute.

In GC-on repeat1, the largest elapsed outlier was2157.581us but only
107.533us of thread CPU:2050.048us was off-CPU delay, consistent with
scheduler/OS wait, not uniquely identified as a particular cause. With
GC frozen/disabled, the largest per-repeat outliers were:

| Repeat | Wall us | Thread CPU us | Non-CPU gap us |
| --- | ---: | ---: | ---: |
| 0 | 694.256 | 656.688 | 37.568 |
| 1 | 638.852 | 632.011 | 6.841 |
| 2 | 568.080 | 542.473 | 25.607 |

These remaining maxima are mainly CPU work in the timed sampling/library
path, not cyclic GC or scheduler delay alone. The exact expensive function
has not been identified. Assigning all spikes to GC/OS scheduling would be
incorrect. Further profiling is outside this one bounded investigation.

All original and refined CSV rows/outliers are retained separately. The
freeze/disable experiment is not a production GC policy, a hard OS deadline,
a Pico timing claim or permission to integrate the runtime path.

## Reproduce and tests

From the repository root, use only the Python standard library:

```sh
python3 -S -m unittest discover -s sim/webots/tests -p 'test_s3_*.py' -v
python3 -S -m sim.webots.pil.s3_f1_check --accuracy-only \
  --output build/webots/f1/accuracy
python3 -S -m sim.webots.pil.s3_f1_check --timing-only \
  --output build/webots/f1/timing
```

The paired investigation uses separate output folders; do not overwrite the
initial `build/webots/f1/timing` evidence:

```sh
python3 -S -m sim.webots.pil.s3_f1_check --timing-only --gc-mode enabled \
  --output build/webots/f1/refinement/enabled
python3 -S -m sim.webots.pil.s3_f1_check --timing-only --gc-mode freeze-disable \
  --output build/webots/f1/refinement/freeze-disable
```

These commands document the completed one-time protocol; do not rerun it
without approval. Each report includes source/profile hashes and every
outlier's thread-CPU/elapsed split. The timing command returns nonzero when
any strict timing limit fails.
The initial S3 run has58 cases:55 passes and the same3 expected physical-model
failures. Tests cover immutable revisions/fractions, no runtime rays, exact
seeded noise, stale revision, unknown/ambiguous material, raised paper,
unsupported curves/occluders, absent floor and rigid pure-yaw bar exit/re-entry.
The merged host suite passes125 cases with the same3 expected failures:
use `PYTHONPATH="$PWD/sim/webots:$PWD"` for its existing `pil.*` imports.
The unchanged host C/PC wire interop check passes3026 comparisons at9831991.
The refinement adds3 tests: current S3 total61 and merged total128, each
retaining the same3 expected failures. Tests check buffer storage, GC-policy
restoration, zero-noise RNG avoidance and unchanged GP28 seeded values.
Full results and source hashes remain in generated evidence. Do not infer
Pico timing, stack, UART freshness or action safety from them.
