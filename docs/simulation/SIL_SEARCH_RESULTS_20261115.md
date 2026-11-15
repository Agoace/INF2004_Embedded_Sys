# SIL SEARCH results — 2026-11-15

Same21 starts,10 s each. Unchanged P7 LINE/SEARCH shell and motor controller; no tuning. Each trial starts a fresh world and uses public calibration replay of the aligned motor-driven [spin samples](SIL_SPIN_CALIBRATION_20261115.md).

| Profile and calibration | Settled | Found by SEARCH | Had LOST event | Completed/replayed |
|---|---:|---:|---:|---:|
| A, old synthetic200/3700 diagnostic |13|0|8|21/21|
| A, actual simulated spin |1|0|0|21/21|
| **B, actual simulated spin** |**21**|**6**|**6**|**21/21**|

LOST counts runs with an event, not terminal failures: all6 B losses are reacquired and settle. Terminal settling remains<3 mm through the last≥500 ms. A and B use the same actual-spin protocol, world, starts and controller settings. Their optical response differs; neither comparison proves physical recovery.

## Calibration and cause

- B aligned spin:2260 ms;1740 sampled rows; public endpoints **L172..2487, C158..2611**, matching boot5209 extrema. Source fit: [SW4 profile B](S3_ASSUMED_LINE_PROFILES.md),1634 real spin rows, not controller results. Full-trace maximum residual still exceeds1000 ADC; this is not physical-model acceptance.
- A actual spin endpoints200..1698 each. Its broad footprint gives approximately91% midpoint after proper calibration, contradicting the measured38–40%;183154 false-junction samples across21 runs, versus0 for B. The unchanged LINE module then takes its junction straight behavior. A's no-loss count is not better recovery.
- With old synthetic200/3700 ends, all8 A SEARCH sweeps cross both sensor centres over the line. Peak ADC1698 gives norm only284–408 after adaptation, below FOUND500. This explains their NOT_FOUND outcome; the sweep did not miss geometrically. Per-run truth-only diagnostics remain in the retained A evidence.

| Offset mm | Yaw ° | Settled from ms | SEARCH found | LOST |
|---:|---:|---:|---:|---:|
|0|0|1|0|0|
|0|5|208|0|0|
|0|-5|197|0|0|
|5|0|104|0|0|
|5|5|1232|1|1|
|5|-5|116|0|0|
|-5|0|99|0|0|
|-5|5|123|0|0|
|-5|-5|579|1|1|
|10|0|180|0|0|
|10|5|1145|1|1|
|10|-5|1|0|0|
|-10|0|172|0|0|
|-10|5|1|0|0|
|-10|-5|672|1|1|
|15|0|333|0|0|
|15|5|1193|1|1|
|15|-5|89|0|0|
|-15|0|302|0|0|
|-15|5|86|0|0|
|-15|-5|709|1|1|

## Evidence and checks

All paths are under `build/webots/`:

- B spin: `sil-spin-B-20261115T130815Z/`.
- B grid: `sil-search-B-20261115T130824Z/`.
- B repeat: `sil-search-B-repeat-20261115T131151Z/`; all21 CSV SHA256 values match the first run;42/42 complete C replays. Repeatability is limited to the current environment.
- A actual-spin grid: `sil-search-A-calibrated-20261115T131023Z/`; spin: `sil-spin-A-20261115T130348Z/`.
- Old synthetic A diagnostic: `sil-search-wide-20261115T125447Z/search-cause-diagnostic.json`. Earlier narrow-world and dead-band rejection records are retained separately, not overwritten or counted as successes.
-325 tests OK,3 existing expected failures; unchanged native LINE/SEARCH, shell and motor tests pass; frozen158 RX/58 TX match. Docs/whitespace checks pass; all owned groups are gone; Webots wrapper SHA unchanged.

World white margins are explicit315 mm blank tiles,945×3150 mm overall; seams are not modelled. No sampler override, hardware or serial. Only LINE/SEARCH are registered. **TRIAL INCOMPLETE / NOT GATE1 PASS**.
