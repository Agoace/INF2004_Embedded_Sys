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

Main authorized ONE later bounded investigation: preallocated numeric
measurement buffers, a documented freeze/disable-GC experiment, and the safe
zero-noise shortcut for synthetic channels. That is a separate change and
rerun. It must not overwrite this FAIL, weaken500us, precompute sensor poses
or change the nonzero GP28 noise sequence. GC policy experiments are not a
production runtime policy or hardware timing result.

## Reproduce and tests

From the repository root, use only the Python standard library:

```sh
python3 -S -m unittest discover -s sim/webots/tests -p 'test_s3_*.py' -v
python3 -S -m sim.webots.pil.s3_f1_check --accuracy-only \
  --output build/webots/f1/accuracy
python3 -S -m sim.webots.pil.s3_f1_check --timing-only \
  --output build/webots/f1/timing
```

The timing command returns nonzero when any strict timing limit fails.
The initial S3 run has58 cases:55 passes and the same3 expected physical-model
failures. Tests cover immutable revisions/fractions, no runtime rays, exact
seeded noise, stale revision, unknown/ambiguous material, raised paper,
unsupported curves/occluders, absent floor and rigid pure-yaw bar exit/re-entry.
The merged host suite passes125 cases with the same3 expected failures:
use `PYTHONPATH="$PWD/sim/webots:$PWD"` for its existing `pil.*` imports.
The unchanged host C/PC wire interop check passes3026 comparisons at9831991.
Full results and source hashes remain in generated evidence. Do not infer
Pico timing, stack, UART freshness or action safety from them.
