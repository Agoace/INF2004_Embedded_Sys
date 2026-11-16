# S3 size-reader host-only SIL

## Status

**ASSUMED optical model. Not a hardware acceptance result. No Webots run.**

The reader is [`4667a83`](../s3-line-barcode/LIVE_WINDOW_READER.md), merged
with `--no-ff` into `mtk3-hil-sw4` at `e20ed23`, after the `mtk3-hil`
`ba1c17a` baseline merge. No code was cherry-picked. The C reader keeps the
`52244f2` GP28 projection and bar/space bias correction.

This check does not change any sensor profile, material fit, steering code,
action permission, source ABI or hardware setting. In particular,
`calibration/s3_sensor_profiles.json` is unchanged.

## Entry points and command

- [`s3_size_check.py`](../../sim/webots/pil/s3_size_check.py) builds a temporary
  C shared library from the checked-out reader and its ordinary symbol table.
  A small fixed wrapper feeds 1 kHz ADC and quantized 10 ms encoder snapshots.
- [`test_s3_size_check.py`](../../sim/webots/tests/test_s3_size_check.py) tests
  the body geometry, both directions, the real C bridge, a low-speed horizon,
  out-of-scope size and negatives. Its missing-module RED result was retained
  locally before the implementation.

From the repository root:

```sh
python3 -m unittest sim.webots.tests.test_s3_size_check
python3 -m sim.webots.pil.s3_size_check --out build/size-reader-sil/report.json
python3 -m unittest discover -s sim/webots/tests -p 'test_s3_*.py'
```

Python uses only its standard library. The bridge needs a host C compiler
named `cc`, which compiles with warnings as errors. The report records the
reader source SHA-256. Temporary build products are deleted automatically.
No serial port, Pico, Webots controller or Webots process is opened.

## Model and gates

Every barcode has all 29 ordinary Code 39 elements. Body scaling includes
both gaps; the body lengths are 274, 210 and 141 mm. No letter is substituted
to make a result pass. The corpus uses A, F, M and Z in both directions.

| Model item | Explicit assumption |
| --- | --- |
| Print | Infinite transverse ink stripes on infinite white paper; 80 mm leading white |
| ADC | Linear ink coverage, white 160, ink 1000; not a measured GP28 response fit |
| Along-print sigma | 0, 0.5, 1, 2.15 and 4 mm, all ASSUMED |
| Cross sigma | About 2.15 mm estimate; not proof of along-print resolution |
| Noise | Deterministic uniform +/-10 ADC in exploratory cases; not measured noise |
| Motion | Exact no-slip rigid differential drive, GP28 forward80/left77, track111 mm |
| Speed and yaw | Mean90/146/200 mm/s; sinusoidal0/25/55 degrees/s peak at0.7 Hz |
| Time | Enough to cross leading white, body and white proof at each speed |

Yaw cases use an independent exact nominal pose to determine GP28 world X.
The Gaussian normal marginal rotates with yaw. The reader receives original
quantized left/right counts, not the model's already-projected progress.
Infinite stripes make this marginal exact within the stated Gaussian model.
Finite bar ends, lateral visibility, paper edges, mat/paper response, height,
slip and the unknown actual sensor transfer are not represented. In particular,
this is not the finite-paper optical scene or the fitted nonlinear GP28 model.

The positive gate requires correct ordinary-letter decoding for straight
274/210 mm low-blur cases (sigma at most1 mm). The high-blur/yaw sweep is
exploratory: rejection is retained, not hidden or used to relax the gate.
Every accepted barcode must have its requested letter. All141 mm cases and
all line/junction/white negatives must reject. The negatives contain isolated
18 mm ink, separate18/35/70 mm arms, or white only; they are not surveyed mat
geometry. These rules were set before the sweep result was inspected.

The first sweep used a fixed4 s trace, which truncated the274 mm,90 mm/s
positive before its white proof. Its failed result was retained locally.
The trace horizon now depends on travel distance and speed. No decoder
threshold or acceptance requirement changed to remove that test-harness error.

## Results

MEASURED host execution of the ASSUMED cases:

| Corpus | Cases | Correct accepts | Rejections |
| --- | --- | --- | --- |
| 274 mm, four letters, both directions, five blur values | 40 | 24 | 16 |
| 210 mm, same corpus | 40 | 24 | 16 |
| 141 mm, same corpus | 40 | 0 | 40 |
| Two sizes, three speeds, three yaw peaks, two blur values, noise | 36 | 16 | 20 |
| Line/junction/white, four blur values, two yaw peaks, noise | 24 | 0 | 24 |
| Total | 180 | 64 | 116 |

Gate PASS: no wrong letter, negative decode or141 mm acceptance. The274/210
mm straight corpus accepts at sigma0/0.5/1 mm and rejects at2.15/4 mm. This
shows why a cross-axis sigma alone cannot certify210 mm physical readability.
Rejection reasons across the sweep: BW_WIDTH91, BW_START14, BW_MOTION3,
and8 cases with no completed candidate (reason0). No rejection was discarded.

The merged branch passes633/633 GCC host tests and633/633 Clang ASan/UBSan
host tests. Its S3 Python suite passes86 tests, with the same three historical
expected failures retained. These are software checks, not hardware evidence.

Local report and logs: `build/size-reader-sil/`. The report is generated data,
not a public fixture. Reader changes and the ten school attempts are recorded
in the [reader gate table](../s3-line-barcode/LIVE_WINDOW_READER.md#size-extension-acceptance-gates).
The school MQTT ADC is about5 Hz, not enough to replay1 kHz bars. This sweep
does not establish a measured false-positive rate or identify those attempts.

## Remaining gate

Wait for review before the one home straight210 mm SHADOW robot test. Record
actual body and quiet-space dimensions, full-rate GP28 and encoder data,
letter/framing, and fitted widths. The4 mm floor is ASSUMED, not a demonstrated
safe minimum. No141 mm support or real action is authorized by this result.
