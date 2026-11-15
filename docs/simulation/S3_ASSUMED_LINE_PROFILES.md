# SIL line profiles — boot-5209 fit

## Joint nonlinear spin + endpoints (opt-in B)

Label: **fitted to boot-5209 spin + endpoints**, not physical calibration.
Use `load_boot5209_line_profile(channel)` from `pil.s3_assumed_line`.
Record: `calibration/s3_boot5209_line_profiles.json`.
The old file and `load_assumed_line_profile` remain unchanged for comparison A.
Both use the same measured mount geometry; GP28 is unchanged.

| Parameter / result | GP27 left | GP26 centre |
| --- | ---: | ---: |
| Floor / centred18mm-line peak ADC |172 /2487 |158 /2611 |
| Material line level ADC (not the strip peak) |2498.366736 |2623.044321 |
| Spatial sigma / response power |2.85630798mm /3.00696460 |same |
| Midpoint after endpoint normalization |39% |39% |
| Full-spin RMSE / p95 absolute error ADC |101.73 /108.56 |110.39 /110.07 |
| Full-spin maximum absolute error ADC |1004.61 |1051.54 |

Pair maximum100.002% on a0.01mm grid; runtime ADC sweep at0.1mm stays
below102%. Floor and centred peaks are exact analytic fit constraints;
F1 finite-support integration changes peak by less than1ADC and midpoint
by less than0.1percentage point. Response is monotonic:
`floor + (line_level-floor)*coverage**power`, with Gaussian coverage over
unchanged18mm strip and14.5mm centre spacing.
Old linear model peak1698.17ADC has residuals−788.83L/−912.83C. Its best
endpoint-constrained Gaussian gives midpoint68.22% each at sigma4.48862mm,
pair136.44%; changing affine levels cannot meet the midpoint target.

All1634 rows of `tests/fixtures/spin_raw_boot5209.csv` are included, no
outlier removal or holdout. SHA256 is in the profile. Endpoint source:
[boot-5209 record](../archive/s3-line-barcode/LINE_POSITION.md#robot-check-spin-recording-2026-11-03-boot-5209).
Fit minimizes equal-weight raw-ADC squared error; endpoints and39% midpoint
are exact constraints. Four fixed starts and bounded coordinate descent
are reproducible with standard-library Python:

```sh
python3 -S sim/webots/pil/s3_spin_fit.py tests/fixtures/spin_raw_boot5209.csv
```

Pose is not uniquely known: only left encoder counts were recorded. Split
at first global count minimum/maximum (rows422/1122). Assume a rigid
119mm-forward pair on a flat straight line; fit angle=`k*(count-c0)` per
sweep. Fitted `(k rad/count,c0 counts)` are `(0.00114116,-35.8850)`,
`(0.00210496,54.5250)`, `(0.00164346,15.8809)`. These absorb unknown
right-wheel motion,50ms motor stagger and drift. They are not measured
angular rates or an encoder calibration. Pauses are not in the recording;
no constant angular rate across sample indices is assumed.
Large crossing residuals remain: endpoints/midpoint agreement is not
full-spin acceptance. No controller result was used in the fit.

Shape prior only: operator identifies touching Cytron
[IR Line Tracking Module](https://sg.cytron.io/p-ir-line-tracking-module)
boards, underside emitter/receiver, AO high on black. Product information
does not supply numerical sigma/power or a measured footprint at8mm.

## Original linear fit (comparison A)

**ASSUMED, straight-strip only.** Frozen measured-profile JSON is unchanged.

| Item | Value / result |
| --- | --- |
| Profile record | [`s3_assumed_line_profiles.json`](../../calibration/s3_assumed_line_profiles.json) |
| Explicit loader | [`s3_assumed_line.py`](../../sim/webots/pil/s3_assumed_line.py): `load_assumed_line_profile('GP26'/'GP27')`, `load_assumed_line_mount(...)` |
| Label | `ASSUMED_SIL_STRAIGHT_ONLY`; fitted to boot-5209 spin recording |
| Fit source | [Spin recording](../archive/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md#evidence): midpoint38–40% each; plain-line sum<=102% |
| Fixed geometry | Line18mm MEASURED(operator2026-11-15); relative sensor spacing14.5mm MEASURED |
| Effective Gaussian sigma | 15.924122728751232mm; solve coverage at7.25mm offset=0.39 |
| ADC / noise | Floor200, line3700, black HIGH; noise0; already post-eight-average |
| Mount body XY | GP26 `(119,0)`mm; GP27 `(119,+14.5)`mm;119forward MEASURED(operator2026-11-15), lateral datum assumed |
| Height | Approximately8mm MEASURED(operator2026-11-15); axle-relative Z=`8-wheel_radius` |
| Midpoint output | Raw1565 each; normalized390 each; pair sum780 |
| Across-strip check | Offsets-100..100mm in1mm steps: maximum normalized pair sum0.78<=1.02 |
| Tests | [`test_s3_assumed_line.py`](../../sim/webots/tests/test_s3_assumed_line.py):6pass; red import and red sigma2 fit recorded first |

## Use / limits

- Pair target0: place line centre at body y=+7.25mm initially.
- Feed raw `[GP27,GP26,0]` to C line-position pair mode; C normalizes it.
- Public calibration API: feed separate labelled synthetic floor200/line3700
  training rows, with floor in the majority. A narrow-strip sweep cannot
  reach3700 with this footprint; do not relabel its maximum as pure ink.
- Use a long straight strip and continuous floor. F1's conservative guard
  extends about90.08mm from each sensor; no end-cap fallback.
- Fit is an effective response match, not a measured beam/height survey.
  Transverse isotropy remains assumed;119mm forward and approximately8mm
  height are operator-measured. Sigma stays15.92412273mm: it is spatial,
  so reference-height coverage remains39% each / pair maximum78% at8mm.
  GP28 height/profile remain unchanged;80mm forward/77mm left, no axis swap.
- Paper levels are schema placeholders only; paper/barcode use is unsupported.
- `load_profile('GP26'/'GP27')` still fails UNMEASURED. No thresholds, controller,
  strip width or frozen profile changes.
