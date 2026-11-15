# Back-to-back S-curve SIL — 2026-11-15

| Result | Value |
|---|---:|
| Completes S-curve | Yes |
| Maximum tracking error, sensor-pair midpoint |11.343 mm|
| LOST / SEARCH / FOUND events |0 /0 /0|
| Completion time |7.401 s simulation|
| Recorded duration / wall time |10 s /829.32 s|
| Aligned calibration spin, simulation / wall |2.260 s /162.29 s|

## Geometry and unchanged control

Straight → left90° → right90° back to back → straight, with315 mm white-tile margins. Both radii157.5 mm are **nominal, vendor tile sheet; not ruler-surveyed**. Cytron tiles are315×315×10 mm with18 mm lines and corner-centred quarter circles; see the [published track specification and vendor link](../archive/simulation/SIMULATION.md#published-track-specification). Layout/margins are assumed; seams are not modelled. Generator and truth metrics: `pil/sil_s_curve.py`.

The authoritative full-scene sampler uses profile B and default17-point-per-axis quadrature, without fallback or decimation. ADC/physics1 ms; native LINE+SEARCH shell, P7 steering and motor control10 ms; signed DM1 and fitted80 ms lag unchanged. No tuning, GP28 or additional vehicle modules.

An aligned motor-driven spin first collects1740 calibration rows through public APIs; endpoints L172..2488 / C158..2614. The fresh S-curve world replays these rows and checks profile/sampler hashes. Error is the119 mm-forward sensor-pair midpoint's distance to the analytical straight/arc centreline, diagnostic only. Peak error occurs3827 ms. Completion requires visiting both arc segments, passing315 mm beyond the second arc exit, axle past that exit and heading within15° of east.

## Evidence and checks

- Spin: `build/webots/sil-s-curve-full-spin-20261115T140054Z/`.
- S-curve: `build/webots/sil-s-curve-full-20261115T140344Z/`;10,000 samples, clean exit/log, shell fault0, independent recorded-input C replay.
- Sample SHA256: `24ce912a71e5e95f06a5da6452195842923cfda6fcbdf4082b551e6901dcd339`.
-331 tests OK,3 existing expected failures; unchanged native LINE/SEARCH, shell and motor tests pass; frozen158 RX/58 TX match. Docs/whitespace checks pass. Both owned groups are gone; Webots wrapper unchanged.

Exactly two approved Webots runs,1200 s timeout each; no repeated-physics claim. Existing [single-curve limits](SIL_CURVE_20261115.md) apply. No hardware or physical acceptance; **NOT GATE1 PASS**. Entry point: `tools/sil_curve_smoke.py --s-curve`, with a completed full-scene spin calibration file. Further extended runs require approval.
