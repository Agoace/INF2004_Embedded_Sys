# SIL line profiles — boot-5209 fit

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
