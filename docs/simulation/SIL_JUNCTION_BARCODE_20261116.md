# Observe-only junction and barcode simulation

## Scope

This experiment does not reopen Gate1. Gate1 remains **TRIAL INCOMPLETE / NOT
GATE1 PASS**. No device, firmware flash, or hardware scheduling test is used.

The course has a straight18mm line, Code39 `*A*` to its LEFT, then a cross
junction. A maps to LEFT in the unchanged reader. This experiment does **not**
permit that action: B2b remains SHADOW, valid0, without OFFSET_VERIFIED.
`BARCODE_LIVE_ENABLE=0` and `BARCODE_MEASURED_CLEAR_MM=0` remain explicit.

## Native code and simulation replacement

| Path | Implementation |
| --- | --- |
| Steering/motor | Existing P7 speed command, unchanged motor_ctl and signed DM1/80ms effective lag |
| Vehicle | Unchanged vehicle_sm, jl_module (line_trace+junction_action), SEARCH |
| Barcode | Unchanged barcode_calibration, barcode_window, barcode_live_producer, junction_live |
| Acquisition | Simulation-owned1ms GP28 ADC and10ms native motor snapshots |
| Publication | Simulation-owned coherent in-process control/record boards; jlive is the only control writer |
| Calibration | One actual aligned motor/encoder/optics spin; public calibration APIs; sampled rows replayed after independent pose reset |

The replacement is in
[barcode_adapter.c](../../sim/webots/controllers/sil_line/barcode_adapter.c).
It replaces SDK task/board acquisition, not robot algorithms. The motor tick
precedes vehicle commands. The shell step precedes ja_cross_progress and
jlive route/tick/read/ja_offer, as in vehicle_task.c1167–1206. No command or
valid flag is injected. Pending LINE events survive the1ms/10ms boundary.
The adapter retains128 native window-result events; overflow makes the run
incomplete. Native input replay is a separate C replay, **not repeated physics**.

## Geometry and optical limits

See [geometry provenance](SIL_BARCODE_GEOMETRY.md). A4 landscape is297x210mm
MEASURED; print centring and zero paper thickness are ASSUMED. Bar height is
160+/-10mm PHOTO_ESTIMATED. Paper starts atx970.5mm and is25mm left of the
line. Its end tox1417.5mm junction centre is150mm ASSUMED. Actual paper quiet
zones are11.75mm; surrounding MAT is not extra paper. No paper-only domain,
white pad, or admission seam is used.

The GP28 along-sigma1/4/8mm family is ASSUMED. Cross sigma is approximately
2.15mm; board U is body-left. MAT proxies178/2321 come from unlocated MQTT
clusters, not pure-material calibration; MAT response power1 and noise0 are
ASSUMED. Physical GP28 edge/holdout gates remain **FAIL**.

Line channels retain authoritative sample_sensor. GP28 uses explicitly named
boundary-aware fine_sample at tolerance1e-7, not a fixed-grid fallback. SW4's
independent straight-A4 check has8341 positions/family: q17 max184–188 and
p9599–104ADC; q257 still max10.55–12.78/p956.84–7.06ADC. No tested fixed grid
passes max8/p952ADC. Fine1e-7/1e-8 spots differ by<4.43e-7ADC; independent
analytic/referee difference is<1.82e-7ADC. Dense128 spots have9.201ADC error
and are **not** a certified reference. These numerical checks do not establish
physical model accuracy or cover an arbitrary turning trace.

Main separately approved the exact uniform-MAT certificate in
[sil_uniform_mat.py](../../sim/webots/pil/sil_uniform_mat.py). One original
axis-aligned opaque white face must contain every corner of the full rotated
4-sigma **box**, not an ellipse. All competing faces and layers are checked.
A relevant material boundary, unknown, nonrectangle, nonexactz10 plane,
height mismatch, or tilt uses fine_sample instead. Only faces strictly hidden
by more than SceneSnapshot.EPS_MM may be ignored. No white-union corner
approximation is used. Two near-plane/high-layer regression cases prevent
false178 certificates. Certified and full calls are counted separately.

## Run protocol

The approved budget is one aligned spin and three independent physics runs,
each at most1200s wall time. Original protocol: GP28 starts300mm before paper
atx670.5mm (axle590.5mm, y150.25mm), with a5.3s simulation target. Original
pre-run estimates were spin500–1000s; sigma1/4 each500–700s; sigma8
850–1100s. The longer lead is cheap only where full MAT support is certified.

The approved continuation for sigma4/8 uses100mm before paper (GP28x870.5mm,
axle790.5mm). Stop at the FIRST native junction confirmation, with5300ms as
a maximum, not a required duration. This criterion is declared BEFORE those
runs. It must never be applied retroactively to another attempt. The replay
checks that the first positive native counter is the final recorded row.
The runner now requires an explicit `--sigma`; it never launches another case.
Sigma4's updated pre-run wall estimate is650–850s, based on the observed
original-segment wall time. Re-estimate sigma8 before its launch.

Every run must show wheel speeds within5% of146mm/s before the first paper
support intersection and at first ink. Speed is a post-run observation and
never changes the controller. Different starts/end criteria prevent a claim
of identical whole-run comparisons across the original and short protocols.

```sh
DISPLAY=:0 XAUTHORITY="$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)" \
python3 sim/webots/tools/sil_spin_calibration.py --barcode --full-scene \
  --timeout-seconds 1200 --output build/webots/NEW-SPIN
DISPLAY=:0 XAUTHORITY="$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)" \
python3 sim/webots/tools/sil_barcode_smoke.py --output build/webots/NEW-TRIAL \
  --sigma 4 --lead-mm 100 \
  --spin-calibration build/webots/NEW-SPIN/spin-result/calibration.json
```

Results must report letters, attempts/reasons, native quiet/start proofs,
junction detection, actual action versus expected LEFT, actual admission
reason, final state, calibration reports, wheel speeds, and call counts.
No Webots result is claimed by this wiring record.
