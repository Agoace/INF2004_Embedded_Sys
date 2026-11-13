# S3 host sensor reference — STAGE 0

## Status and limits

**EXPLORATORY HOST REFERENCE. NOT SENSOR CALIBRATION. STOP BEFORE GATE 1.**

The fixed 1.5 mm edge check fails on the held-out capture. Keep that FAIL.
The finite-ray sampler is a slow offline reference, not a 1 kHz runtime
sampler. No firmware, decoder, acceptance gate or action permission changes
are part of this record. No Pico flash or hardware timing test was done.
GP28 remains at a nominal minimum clearance of 5.5 mm. Do not reduce it to
improve a simulated result.

## Entry points

| Path | Use |
| --- | --- |
| [`s3_geometry.py`](../../sim/webots/pil/s3_geometry.py) | Convex tile and Code39 polygons, ports, material IDs and SVG |
| [`s3_assets.py`](../../sim/webots/pil/s3_assets.py) | PNG and SVG from the same polygons; no external packages |
| [`s3_optics.py`](../../sim/webots/pil/s3_optics.py) | Structural `first_hit` adapter and explicit response/noise model |
| [`s3_kinematics.py`](../../sim/webots/pil/s3_kinematics.py) | Exact nominal differential-drive geometry and rigid sensor points |
| [`s3_replay.py`](../../sim/webots/pil/s3_replay.py) | Tracked-capture fit, pose registration and held-out check |
| [`s3_profiles.py`](../../sim/webots/pil/s3_profiles.py) | Profile loader and repeatable profile record generation |
| [`s3_sensor_profiles.json`](../../calibration/s3_sensor_profiles.json) | Frozen assumptions, file hashes, both axis variants and FAIL results |
| [`tests/`](../../sim/webots/tests/) | `test_s3_*` host tests, including three known model failures |

Python uses the standard library only. The folder name `calibration/` does
not certify the contents as a physical calibration.

## Shared geometry

World coordinates are X east, Y north, Z up, in millimetres. Body coordinates
are x forward, y left, z up. Positive yaw is counter-clockwise. Ports point
out of the tile. Convert mm to metres and degrees to radians only at the
Webots boundary.

| Item | Reference geometry / explicit assumption |
| --- | --- |
| Tile / stroke | 315 mm square / 18 mm wide |
| STRAIGHT | N/S ports at rotation zero |
| CURVE | N/E ports; centre at NE corner `(157.5,157.5)`, radius 157.5 mm |
| Curve stroke | Inner/outer radii 148.5/166.5 mm; convex quads, chord error at most 0.01 mm |
| CROSS / T / BLANK | NESW / NEW / no ports |
| Rotation | Quarter-turns counter-clockwise; polygons and ports rotate together |
| STATION | Must supply both a convex CCW marker polygon and cardinal ports; no invented station pattern |
| Barcode | `*LETTER*`, uppercase A–Z, canonical firmware Code39 masks, no checksum |
| Fixed widths | Narrow 5.8 mm, wide 17.5 mm; not fitted on either capture |
| Printed body | 29 elements: 20 narrow + 9 wide = 273.5 mm; 15 ink bars |
| Quiet envelope | Two 10-narrow spaces: 389.5 mm total; not a paper-size requirement |
| Paper / transverse bars | Paper size and bar height are explicit or unspecified; thickness defaults to an unmeasured zero-layer assumption |

A 285 mm sheet can contain the printed body without containing the full
quiet envelope. Clean mat can supply quiet space. It is not valid to assume
that a 389.5 mm envelope means a 389.5 mm sheet. `reverse=True` mirrors the
complete printed sequence; it does not substitute another letter.

All emitted patches are convex, world-space CCW polygons with unique IDs.
The line/material layers are mat 0, line 10, paper 20, ink 30. These layers
resolve coplanar ties only; they do not add physical height.

### PNG/SVG contract

`raster_rows(patches, bounds_mm, mm_per_pixel=.1)` returns grayscale rows.
`png_bytes(rows)` encodes them. PNG +Y is image top; +X is image right.
SVG negates world Y. The requested pixel spacing is an upper bound; the
actual spacing fits the supplied bounds exactly. Raster edge position has
pixel-size error. It is not optical noise.

For standalone paper, use `transparent=True` and
`png_bytes(rows, channels=2)`. These are grayscale/alpha pixel pairs. The
clear envelope is transparent, not extra painted paper. Mat black and paper
ink have the same visible grey value but different material IDs.

For zero-thickness paper, bake floor, line, paper and ink into one floor-top
texture per tile. Do not draw coplanar overlays with a hidden depth offset.
If physical paper thickness is nonzero, keep its explicit raised surface and
use a single paper/ink texture there; do not lower it onto the mat. Exclude
physically elevated box/hump surfaces from the floor texture operation.
Physical scene surfaces and the renderer still control occlusion. Camera pixels are for
inspection, not calibrated ADC input.

Example assets below use **assumed** 20 mm bars and a 285 by 30 mm sheet:

```sh
python3 -m sim.webots.pil.s3_assets \
  --output build/webots/s3/example-assumed-assets \
  --bar-height-mm 20 --paper-width-mm 285 --paper-height-mm 30
```

This writes five tile and 26 Code39 PNG/SVG pairs plus a manifest. It does
not generate a station without a supplied station specification.

## Scene and optical response

The adapter consumes SW5's frozen structural API:
`first_hit(origin_mm, direction, max_distance_mm) -> SurfaceHit`.
Direction need not be unit length; distances are physical millimetres.
The nearest physical surface wins. Highest layer applies only to coplanar
ties within 1e-7 mm. Same-material ties use the lowest surface ID.
Conflicting top-layer materials return `ambiguous_material`.

A miss/invalid hit has `valid=False`, a reason, and optional fields `None`.
The adapter never turns that into white. Unknown materials also invalidate
the sample. UV is patch-local: origin vertex zero, U along vertex zero to
one, V = normal cross U. The adapter uses world hit points, not a presumed
tile-centred UV value.

At the nominal reference height, the model integrates Gaussian spatial
coverage: a bell-shaped footprint. Sigma is its spatial standard deviation,
not a surveyed beam radius. For the all-paper 1D fit:

```text
ADC mean = white + gain * ink_coverage ** response_power
```

Ink coverage is the exact error-function integral of the fixed bar
intervals for capture replay. The 2D ray reference uses finite Gaussian
cells out to four standard deviations and renormalizes their weights.
Its numerical error is separate from the capture fit.

The material-family response preserves all four pure-material anchors.
Paper power is fitted. Mat power is assumed one; mixed-material response
is not measured. The 2673 mat-high anchor is a recorded sweep mean, not a
proven pure-material plateau; see
[LIVE_CALIBRATION.md](../s3-line-barcode/LIVE_CALIBRATION.md).
There is no measured transverse footprint or height/tilt response.
Transverse sigma equals longitudinal sigma **by assumption only**.

Outside the reference height or normal incidence, the adapter returns
`uncalibrated_height_or_tilt`. The 0.001 mm comparison is a numerical domain
check, not a measured height tolerance. Explicit `allow_extrapolation=True`
permits a geometrically projected reference, retains that assumption in the
result, and supplies no measured height gain law. Air readings are not a
known-height calibration point. Do not make the sensor follow terrain.

ADC noise is time/channel/seed-keyed, then quantized and saturated to
0..4095. A clipping flag is retained. These values already represent the
firmware's eight-conversion average; do not average them eight times again.
A computed model value is not evidence of physical calibration or live
barcode eligibility.

## Mount and motion limits

GP28 uses nominal `(forward,left)=(80,77)` mm, with approximately 5 mm
forward-position uncertainty. Current pose/height surveys are missing.
For a flat surface datum, use:

```text
mount_z = flat_surface_z + clearance - axle_world_z
```

Do not add the 32.5 mm wheel radius twice. Keep the mount rigid through
roll/pitch/yaw and terrain motion. GP26/27 forward offsets, current heights
and footprints remain `UNMEASURED/null`. Only their 14.5 mm relative spacing
is a measured reference; the historical approximately 9 mm height is not a
current survey. Inferred 119/123 mm forward offsets are not adopted.
`load_profile('GP26')` and `load_profile('GP27')` fail explicitly instead of
reusing GP28 values. Synthetic tests use labelled assumed profiles.

For context only, Main reported USB-dump floor/line readings on 2026-11-13:
L 201/3740, C 164/3712; another run L 203/3690, C 164/3526. The 2026-11-11
baseline was L 206/3402, C 165/3399. These are **OBSERVED-RUN ADC values**
from untracked USB dumps, not calibration inputs or footprint evidence.
They do not fill the null profile fields.

With nominal 111 mm track, `omega=(vR-vL)/111` and mean speed `v=(vL+vR)/2`:

```text
sensor world point = axle point + R(yaw) * (80,77)
sensor body velocity = (v - 77*omega, 80*omega)
```

At L40/R217 mm/s, mean speed is 128.5 mm/s but the sensor's body-forward
speed is about 5.72 mm/s and lateral speed about 127.57 mm/s. GP28 is outside
the 55.5 mm half-track. Sensor arc length, stripe-normal displacement and
mean wheel distance are different quantities. None replaces firmware mean
encoder distance. Tests cover straight/reverse motion, exact arcs, pure
yaw, a velocity derivative, and finite-bar exit/re-entry through real scene
queries. These prove the model arithmetic, not robot slip or print pose.

## Tracked fit and independent held-out check

| Role | Tracked file | SHA256 |
| --- | --- | --- |
| Train | [`barcode_430_large_fwd.csv`](../../tests/fixtures/barcode_430_large_fwd.csv) | `0e8c9c04d7c17d618610db8164cb8866ee9e56fe9fda61d5e93b3c0086acccd5` |
| Hold out | [`barcode_large_f_forward.csv`](../../tests/fixtures/barcode_large_f_forward.csv) | `a10e86e72b88944aeb310e1ce2d7a7a5e6c9f3fe22764e6c4d607d2ff667de0f` |

The files contain 2245 and 3080 ADC observations. Motor gaps reach
39.999/49.998 ms. The approximately 2 ms ADC dump cadence cannot prove
1 ms acquisition timing. Constant-twist interpolation is an assumption,
not a surveyed optical trajectory. Timestamp/count wrap is explicit. Unbracketed,
flagged or saturated observations are reported, not silently extrapolated.
Neither selected file has such rejected observations.

Fit only the original 0..430 mm diagnostic region. The older long trace has
later dark-floor regions beyond about 453 mm; do not label them barcode
ink. Exported CSV retains the full bracketed trace so this exclusion stays
visible. The diagnostic 320/240 crossing rule is not the firmware decoder.

Both print geometry and the check limits were fixed before hold-out
scoring: normalized RMSE at most 0.10; edge error 95th percentile at most
1.5 mm; run-width error 95th percentile at most 2 mm; exactly 30 edges.
Train fits white baseline, contrast, Gaussian sigma, bounded response power
and one global position offset. It never fits print scale. Hold-out uses the
frozen response and permits **global pose translation only**, within an
explicit +/-8 mm registration search around the first star bar. This is a
known-F waveform check, not blind letter recognition or print localization.
No decoded letter is manufactured.

| Axis / trace | RMSE ADC | Normalized RMSE | Edge P95 mm | Width P95 mm | Check |
| --- | ---: | ---: | ---: | ---: | --- |
| Nominal sensor-X / train | 151.19 | 0.11321 | 1.99060 | 2.02424 | **FAIL** |
| Nominal sensor-X / hold out | 126.39 | 0.09464 | 1.87087 | 1.83828 | **FAIL** |
| Mean wheel / train | 119.96 | 0.09058 | 1.41840 | 0.93861 | PASS, scoped training fit only |
| Mean wheel / hold out | 175.24 | 0.13233 | 3.72015 | 1.16552 | **FAIL** |

All four diagnostic traces have 30 observed/model edges. The mean-wheel
variant reaches the contrast search lower bound; the profile records this.
Do not select it merely because its training score passes.

The nominal sensor-X exploratory profile has white 164, contrast
1335.44271 ADC, sigma 2.125 mm and paper response power 6. The mean-wheel
sensitivity fit has contrast 1324.3, sigma 1.525 and power 14.25. Noise comes
only from the tracked training leading 0..30 mm region: 201 samples,
median 164, MAD 3, robust sigma `1.4826*3=4.4478` ADC (population sigma
4.15866). Full-waveform residuals are not added as white noise.

### Reproduce

Run from the repository root. Keep generated files under `build/`:

```sh
for axis in sensor_x mean_wheel; do
  python3 -m sim.webots.pil.s3_replay \
    --train tests/fixtures/barcode_430_large_fwd.csv \
    --holdout tests/fixtures/barcode_large_f_forward.csv \
    --axis "$axis" --output "build/webots/s3/final-$axis"
done
python3 -m sim.webots.pil.s3_profiles \
  --sensor-report build/webots/s3/final-sensor_x/fit_report.json \
  --mean-report build/webots/s3/final-mean_wheel/fit_report.json \
  --training-capture tests/fixtures/barcode_430_large_fwd.csv \
  --output build/webots/s3/reproduced_profiles.json
cmp calibration/s3_sensor_profiles.json build/webots/s3/reproduced_profiles.json
python3 -m unittest discover -s sim/webots/tests -p 'test_s3_*.py' -v
```

After SW5 scene integration, tests import `sim.webots.pil.scene`. Before
integration, extract its exact committed file to an ignored fixture:

```sh
mkdir -p build/webots/s3
git show 3a69191:sim/webots/pil/scene.py >build/webots/s3/scene-3a69191.py
S3_SCENE_MODULE="$PWD/build/webots/s3/scene-3a69191.py" \
  python3 -m unittest discover -s sim/webots/tests -p 'test_s3_*.py' -v
```

The integration fixture SHA256 is
`b2e3ef13c6980bb65a13befc63c737eab5d1c0be61983ccfc657b541e489c1ea`.
Do not include the SW5 file in the S3 commit. Without a merged scene or an
explicit fixture, scene tests report a skip. That run alone is not the
integration result.

The local integration run used Python with site packages disabled (`-S`):
43 cases, 40 passed, three expected failures, no skipped scene cases. This
also includes a synthetic waveform fit; it does not turn the recorded-data
failures into passes.

Three `expectedFailure` tests retain the failed sensor-X train/hold-out edge
checks and the failed mean-wheel hold-out check. An unexpected success
requires review. A normal zero test-process exit does **not** mean that the
physical model checks passed.

## Offline reference speed and numerical error

A local host benchmark used one 6-by-200 mm ink strip, a white plane, the
fitted profile and 21 scan positions. Errors below compare the ray result
with the exact 1D convolution; they exclude noise. Host wall time is not
RP2040 or UART evidence.

| Ray grid | Mean host time/channel | Maximum ADC integration error |
| --- | ---: | ---: |
| 17 by 17 | 5.46 ms | 114.24 ADC |
| 128 by 128 | 305.7 ms | 18.45 ADC |

**Neither is a runtime sampler.** Keep these errors separate from 4.4478 ADC
white noise and from renderer pixel-size error. Do not optimize this path as
part of STAGE 0. A later plan can assess analytic/cached stripe-normal
convolution for 1 kHz, with `first_hit` as a cross-check. That work needs its
own accuracy and time-budget gate.

Physical generalization remains blocked by sheet/bar measurements, station
markings/ports, rigid mount/height/optical-axis surveys, transverse footprint,
height response and slip evidence. Hardware timing, firmware stack margins,
PIL freshness and action enablement require separate approval and tests.
