# SIL straight line — 2026-11-15

## What

| Item | Implementation |
|---|---|
| Code | Unchanged `app_program/subsystem3/line_position.c` + `line_steer.c`; simulation-owned C ABI, shared library/ctypes; compiler/source/library hashes retained |
| Clock | Webots synchronous fast mode;1ms physics/line position; steering every10ms simulation; no wall-time deadline |
| World | `sim/webots/courses/sil/straight_assumed.json`: six315mm tiles,18mm line,1.89m strip; axle start(196,150.25)mm/yaw0 aligns pair midpoint to y157.5mm |
| Sensor | SW4 [boot5209 fitted profiles](S3_ASSUMED_LINE_PROFILES.md), noise0; LEFT=GP27, CENTRE=GP26; barcode not right-line |
| Calibration | Public pair-mode/calib APIs;16 synthetic uniform-floor200 rows +4 uniform-ink3700 rows; NOT a thin-strip calibration or private-state injection |
| Controller | ASSUMED base20,min8,max40%,P100,D100,trim0,far gain0; stop latched on line loss; not locked robot tuning |
| Plant | All forward rows5..100% in [DM1](../archive/s2-motion/DUTY_MAP.md), boot5d51 table; per-wheel linear interpolation ONLY,0→velocity0,other duties rejected/logged |
| Actuation | speed_mm_s/ASSUMED32.5mm radius; velocity servo cap ASSUMED0.12Nm; reverse/braking unmodelled;80–100% DM1 rows have wheelie/unsettled caveat |

Fast optical query uses actual XY/yaw projected to fixed9mm clearance; tilt/bounce
response UNMODELLED, upright cosine<0.999 rejects. Calibration applies native
normalisation once to post-average12bit ADC; no extra ADC averaging. Truth error
is diagnostic only, never a steering input. Input/output/pose tuples buffer until
stepping ends; line-loss/error/end requests wheel velocity0.

## Result

| Check | Observed |
|---|---|
| Two fresh10s runs |10000 samples each; clean exit/log; all8 checks true |
| Line seen / false junction |10000 /0 |
| Forward / wheel-angle travel |1.288997m /39.662984rad each |
| Pair error |max2.28e−13mm (ideal aligned symmetry, NOT physical accuracy) |
| Calibration |OK,bad_mask0,LINE_HIGH,endpoints200/3700 each |
| Determinism | sensor/command/pose CSV BYTE-identical across both runs; independent C-input replay true |

Evidence: `build/webots/sil-line-sealed-{a-20261115T114822Z,b-20261115T114827Z}/`.
CSV SHA256 `cf4b6d8cffaf39fd6bb1a812c5559d17fd39e338b19b9612a06b72b3eb42c41d`.
Red-first ABI/map tests; original C line-position/steering tests pass. Out-of-range
mapping and corrupted C replay fail tests. Full suite306 tests OK/3 existing
expected failures; frozen158RX/58TX replay unchanged.

## Run / limits

```sh
DISPLAY=:0 XAUTHORITY="$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)" \
 python3 sim/webots/tools/sil_line_smoke.py --output build/webots/sil-NEW
```

Same90s timeout/headless fast flags/owned-PGID cleanup as prior smokes. No hardware,
serial, robot source edits, RTOS/PID/vehicle/search/obstacle modules. Aligned straight
plumbing only: disturbance recovery, turns, curves, calibrated physics and final
settling are NOT tested. End issues0 then quits without an extra physics step.
The first launcher missed its controller override and timed out in stage0_preview;
failed evidence `build/webots/sil-line-fit-20261115T114007Z/` remains unchanged.
