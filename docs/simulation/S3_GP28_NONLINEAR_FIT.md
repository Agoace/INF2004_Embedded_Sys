# GP28 nonlinear fit — offline only

**EXPLORATORY, not physical calibration.** New opt-in record:
`calibration/s3_gp28_nonlinear_profiles.json`. Frozen GP28 JSON and normal
loader are unchanged. This uses the same monotonic power-response family
as line B, not its8mm-height constants. GP28 reference remains5.5mm.

## Result

Fit paper ADC=`white + gain*coverage**power`; fixed Code39F,n5.8/w17.5mm,
body273.5mm. Train430mm capture; hold out the large-forward capture.
Only one translation is registered on holdout; response, geometry and
encoder scale stay fixed. All source hashes, exclusions, metrics and
search bounds are in the JSON. Bounds match the earlier replay protocol.

| Pose variant / along-bar sigma assumption | Cross sigma mm / power | Train / holdout RMSE ADC | Holdout edge / width p95 mm | Result |
| --- | --- | --- | --- | --- |
| Sensor-X /1mm |2.155 /5.875 |151.17 /126.66 |1.883 /1.886 |FAIL /FAIL |
| Sensor-X /4mm |2.150 /5.900 |151.17 /126.64 |1.880 /1.875 |FAIL /FAIL |
| Sensor-X /8mm |2.145 /5.925 |151.16 /126.70 |1.877 /1.864 |FAIL /FAIL |
| Mean-wheel /1..8mm |1.565..1.570 /13.125..13.250 |119.62..119.64 /173.54..174.13 |3.682..3.715 /1.224..1.230 |PASS /FAIL |

Sensor-X white164,gain1338.53..1339.16. Mean-wheel gain1324.3 hits its
lower search bound; its training PASS is not holdout acceptance. Existing
normalized-RMSE/edge/width gates and three recorded failures remain intact.
Photo widths are uncertain priors, not fit variables. Separate fixed-response
sensitivity at n5.2/w19(body275) and n6.4/w16(body272), with translation only,
gives sensor-X holdout RMSE143.34/188.96ADC; both FAIL. No width was selected
from those checks. Small sheets are not assigned large-sheet dimensions.

## Rotation and identifiability

Operator confirms same Cytron module, GP28 board rotated90degrees relative
to touching GP26/27 boards. Board U is body-left at zero yaw (line-board
reference U=body-forward is an explicit mounting-axis convention, not an
underside emitter/receiver survey). Apply `board_axis_u(yaw)` to offline
sampling. Projected blur onto the world-X bar normal is
`sqrt((sigmaAlong*sin(yaw))^2 + (sigmaCross*cos(yaw))^2)`.
No-slip encoder yaw ranges only−2.14..+1.58degrees in train and−0.54..+3.08
in holdout. Crossings therefore do not identify along-bar width;1/4/8mm are
predeclared sensitivity assumptions, not three measured footprints.
Finite bar ends, optical-axis tilt and a height law remain unmeasured.
A surveyed transverse/endcap sweep is needed to identify anisotropy.

## Data limits and use

Primary data: `tests/fixtures/barcode_430_large_fwd.csv` (2245ADC) and
`barcode_large_f_forward.csv` (3080ADC), no flagged/saturated/unbracketed
rows. Fit ROI0..430mm excludes the older capture's later off-mat dark region.
Motor gaps reach40/50ms: constant-twist/no-slip interpolation is assumed,
not measured sensor motion. No distance corrections or decoder changes.
SHADOW gen3 white167/high2673/max3178 and3422 MQTT raw_bc observations are
moving mat data, not identified paper-bar plateaus. They cannot constrain
this paper response without sheet/pose correspondence. Unmatched small
captures and those mat readings are excluded, not silently fitted as ink.

```sh
python3 -S -m sim.webots.pil.s3_gp28_nonlinear > build/gp28-fit.json
```

Explicit offline API: `load_gp28_profile(sigma_along_mm=4)` from
`pil.s3_gp28_nonlinear`; optional variant=`sensor_x` or `mean_wheel`.
Omitting the along-bar assumption fails. Mat levels are unsupported schema
placeholders; noise0 is for deterministic mean replay, not measured quietness.
F1 fast anisotropic sampling remains invalid and needs separate approval.
