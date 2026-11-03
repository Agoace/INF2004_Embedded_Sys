# v1.5 box-size sweep

## Scope and reproduction

Run the real, unchanged `detour_test` v1.5 firmware (source `b9dcb6f`).
Use the host model from `c77e829`, without model or parameter changes.
Only scenario name, box dimensions, and box centre x differ from
`box_straight`. All other fields, including assumption notes, are unchanged.
Width means world y; depth means world x. Every box has front face x =
740 mm, centre y = 0, and heading 0°. The main box spans x = 740–1220 mm,
y = −120–120 mm. Each scenario ends at 40 s.

Generate the six new scenarios and run all seven cases:

```sh
python3 sim/box_sweep.py --run
```

The baseline uses the existing `box_straight.json`; the script does not
rewrite it. Omit `--run` to generate only. Results are saved in ignored
`sim/output/bigbox/SCENARIO/`: `trace.json`, `run.html`, and `console.txt`.
The main scenario is [`box_wide_deep.json`](../sim/scenarios/box_wide_deep.json).

## Results

All STOP readings are the printed ultrasonic distance, not the box-to-body
clearance. TURN_AWAY headings are taken at the first STRAIGHT hook: module
heading / true world heading. Straights count entries into STRAIGHT, not
control updates. Firmware end is the absolute sim time of the captured
`[run] ... stopped by` summary. Simulator end is the separate scenario
deadline; the firmware can already be stopped.

| Width × depth (mm) | Scenario | STOP (mm) | Scan side | TURN_AWAY end (module / world) | Straights | Contact | Final result | Firmware end (s) | Simulator end (s) |
| --- | --- | ---: | --- | --- | ---: | --- | --- | ---: | ---: |
| 120 × 120 | `box_straight` | 224 | LEFT | +51° / +51.213694° | 3 | No | FAILED: TIME (detour time limit) | 34.810300 | 40.000000 |
| 120 × 240 | `box_w120_d240` | 224 | LEFT | +51° / +51.213694° | 3 | No | FAILED: TURN_LIMIT (obstacle not seen / not lost) | 34.790300 | 40.000000 |
| 120 × 480 | `box_w120_d480` | 224 | LEFT | +51° / +51.213694° | 2 | No | FAILED: TURN_LIMIT (obstacle not seen / not lost) | 26.800300 | 40.000000 |
| 240 × 120 | `box_w240_d120` | 224 | NONE | Not entered | 0 | No | FAILED: NO_SIDE (no free side) | 9.810200 | 40.000000 |
| 240 × 240 | `box_w240_d240` | 224 | NONE | Not entered | 0 | No | FAILED: NO_SIDE (no free side) | 9.810200 | 40.000000 |
| **240 × 480** | **`box_wide_deep`** | **224** | **NONE** | **Not entered** | **0** | **No** | **FAILED: NO_SIDE (no free side)** | **9.810200** | **40.000000** |
| 360 × 480 | `box_w360_d480` | 224 | NONE | Not entered | 0 | No | FAILED: NO_SIDE (no free side) | 9.810200 | 40.000000 |

No trace has a contact event. Contact time, robot corner/edge, and box face
are therefore not applicable in every row. The milestone 2 first-contact
latch is unchanged. No case stalls or reaches DONE, so line rejoin x and
heading at DONE are not applicable. Every STOP occurs at 7.681700 s.

## Main case: what happens

The robot follows the line and stops at the same 224 mm reading as the
baseline. The scan returns distances at all 13 angles. It reports free
steps R 7 / L 7 and selects NONE. Detour initialization immediately gives
FAILED / NO_SIDE. The robot does not enter TURN_AWAY, STRAIGHT, or
TURN_TOWARD. The firmware ends the run at 9.810200 s, without contact.
There is no travel along the long side of the box in this case.

### Full captured console

The timestamps below are absolute simulated time. The last `Complete`
line is the launcher status, not a firmware print.

```text
2.000000s Obstacle detour test. GP21: calibrate, then run 30 s (press again to stop). GP20: calibrate only.
2.100600s [spin] Spin calibration at 20 % duty, 633 counts (a quarter wheel turn).
2.100600s [spin] step 1: left
2.558900s [spin] -1 done in 458 ms; from start L -663  R 633
2.759300s [spin] step 2: back through the centre to the right
3.630400s [spin] +1 done in 871 ms; from start L 633  R -640
3.830800s [calib] seen range L 202-1645  C 185-2615  R 191-3424
3.830800s [calib] OK. Line reads HIGHER than the floor.
3.830800s [calib]   L: floor  202  line 1645
3.830800s [calib]   C: floor  185  line 2615
3.830800s [calib]   R: floor  191  line 3424
3.830800s [spin] step 3: back to the centre, stop on the line
4.280300s [spin] -1 done in 450 ms, stopped on the line; from start L 59  R -103
4.280300s [spin] centre peak 951, now 951 per mille
5.280300s [run] start, run 1
7.681700s [obst] STOP at 2401 ms, 224 mm
9.806500s [scan] 221 217 215 214 214 214 214 214 214 214 215 218 222
9.806500s [scan] nearest 214 mm, free step R 7 L 7 -> detour NONE
9.810100s [detour] FAILED at 4530 ms: pose (0, 0) heading +0
9.810100s [detour] failed: no free side
9.810200s [run] 4530 ms, stopped by detour failed (no free side)
9.810200s [run] obstacle stops 1, detours done 0, searches 0
9.810200s [run] wheels L 5748  R 5752 counts
9.810200s [run] detour v1.5: turn steps 5 deg (24.05 counts/deg), back off 40 deg, look 150 ms, past nearest 350 mm, seen < 400 mm; turn 20%, straight 20%
Complete: scenario_end at 40000000 us
```

### Full hooked detour timeline

These are both module calls in the main trace. Only the first row is a
step transition; the second confirms the same failed state. Module x/y
remain (0, 0). World x/y locate the rear axle. The module heading is not a
world heading. The later console label does not replace the earlier hook
time.

| Step / reason | Hook | Time (s) | Module heading | True world heading | True world x (mm) | True world y (mm) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| FAILED / NO_SIDE | `dt_init` | 9.806500 | 0° | +0.443031899° | 460.664136315 | 7.830945962 |
| FAILED / NO_SIDE (unchanged) | `dt_update` | 9.810100 | 0° | +0.443031899° | 460.664136315 | 7.830945962 |

### Viewer captures

Captured from `sim/output/bigbox/box_wide_deep/run.html` through a file URL
in offline Chromium 149.0.7827.55, with zero page/console errors and zero
network requests. Files are ignored output, not repository assets:

| Capture | Time (s) | Path |
| --- | ---: | --- |
| Only detour step change: FAILED | 9.806500 | `sim/output/bigbox/01_failed.png` |
| Firmware run end | 9.810200 | `sim/output/bigbox/firmware_run_end.png` |
| Scenario end | 40.000000 | `sim/output/bigbox/scenario_end.png` |
| Fit-all overview of the whole path at scenario end | 40.000000 | `sim/output/bigbox/overview_end_fit_all.png` |

## Checks

- All seven scenarios pass the existing schema and finish their 40 s trace.
- A field-by-field check permits only name, box size, and box centre x to
  differ from the baseline. The front face is 740 mm in all seven cases.
- The main repeat trace is byte-identical. The HTML and console also match.
- Firmware source/header hashes, simulator model, target, viewer, build
  settings, and baseline scenario are unchanged.
- CTest and the documentation checker pass. The post-commit trace hash and
  structured rows are in ignored `sim/output/bigbox/verification.json`;
  browser details are in `sim/output/bigbox/browser_report.json`.
