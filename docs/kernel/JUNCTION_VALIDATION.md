# B2a validation and junction trials

## Status

The 2026-11-10 forced-corner redesign replaces `eb18004` blind travel and
encoder-target turns. The worker did not push or flash. The new C-route fix has hardware status **Not run**. Historical operator
evidence on `eb18004` and `7648081` is separated
below; it is not a PASS for the new images. See the
[design](../architecture/JUNCTION_ACTION.md) for ownership and thresholds.

## Operator-supplied evidence — 2026-11-10

Tested revision: `eb18004`, hotspot/MQTT session
`build/mtk_logs/robot-mqtt-20261110-140557.csv` in the main checkout.
Main_Worker supplied these results; the worker did not inspect that CSV.

- J1 NONE: **PASS** for crossing `+`, zero kind7 and zero line losses.
  This establishes normal LINE crossing, not pending-command barcode rejection.
- J2 old C: six attempts. The detector, pending source and phase10 worked
  when the cross was reached. Runs1–3 stopped with reason7 due to the
  operator's object ahead; exclude them from bridge-accuracy judgement.
- Runs4/6 failed reason4 APPROACH_LIMIT: no stable line by190 mm. Both
  normalized channels fell near zero. The blind160 mm bridge at about96 mm/s
  drifted off the18 mm line. A4-degree error over160 mm gives about11 mm
  lateral displacement; this explains the risk, not a measured heading trace.
- Run5 lost the line before the cross; natural SEARCH failed, and pending
  was correctly cancelled with phase9. The new policy cancels pending at the
  natural non-LINE handoff itself, rather than waiting for final STOP.

Old J2 did **not** pass. Do not tune locked LINE to compensate for blind
travel. That travel,132 mm approach, settle and nominal90/180-degree endpoint
have been removed.

### Operator-supplied `7648081` J2 C x3

Session: `robot-mqtt-20261110-140557.csv`, boot1318100973, supplied by
Main_Worker. The worker did not open the log. Detector/pending/phase10
confirmed3/3. Run3 reached phase7 DONE in about1.1 s, then normal LINE used
two short LINE -> SEARCH -> LINE recoveries: operator PASS. Runs1/2 stopped
phase8 reason6 after about0.9/0.7 s. The operator reported an angled approach
and brief line loss after the cross. Thus J2 was not3/3 successful.

The operator approved a C-only correction: permit the normal SEARCH route
during C; keep the detector disarmed across handoffs; complete at160 mm in
LINE or SEARCH. Remove C-local line-level/time failure gates so ordinary
SEARCH failure/stop stays authoritative. A/B/D remain unchanged for J3–J5
trials on `7648081`. New C hardware results are not supplied yet.

## Software evidence

Checks below apply to the C-route fix based on `7648081`, with public example
settings in an isolated copy; Main must rebuild device-configured images.

| Check | Result |
| --- | --- |
| Strict host C/Python suite |184/184 passed |
| Clang ASan/UBSan/float-cast-overflow |184/184 passed |
| Recorder / dashboard Python tests |70 /187 passed, included above |
| Firmware | Nine warning-free profiles |
| Strict ARM application compilation |675/675 (75 units x9 profiles) |
| Retained SDK hardware link proof |9/9 passed; never-called probe, never flash |
| ELF / legacy sources | No atomic helpers or forbidden legacy drive/servo/spin/scan sources |
| Baseline | Matching public `168d705`: data904+BSS189768=190672 B |
| Documentation/template | Links checked; eight deviations against `c3b0954` |

Five profiles: `SMP=0 WIFI=cyw43 NET=lwip MQTT=1`, corrected IMU,
`JUNCTION_TEST_CMD=NONE/A/B/C/D`. Additional NONE profiles: single-core
radio-off robot, radio-off TEST, radio-off BENCH and dual-core MQTT robot.
Profile invalidation rejects stale objects. Invalid Make options are tested.
Strict flags: `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror
-fstack-usage`, SDK headers treated as system headers. SDK proof precedes each
profile change. No GPIO IRQ/template source or subsystem3/5 file changes.

```sh
# Isolated public-example tree, from build_make/:
for cmd in NONE A B C D; do
    make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 JUNCTION_TEST_CMD=$cmd \
        -j4 --output-sync=target
done
```

NONE is `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`; the other files append
`_jtestA` through `_jtestD` before `.uf2`. Keep boot FAKE command and forced-
corner label. Use NONE for H3 IMU source comparisons. Do not distribute any
credential-bearing image or log.

### Fixtures and limits

- NONE comparison: original and wrapped LINE,1000 calibrated samples with
  junctions and real LP loss/return; exact drives, outcomes and line state.
- Detector: clear baseline, already-on-cross START, threshold ramp, exact700,
  short ink spans, weak channel, stop/reverse and stale observations.
- C: real LINE steering outputs, not equal wheel commands;160 mm detector
  suppression and side-arm observations cannot re-fire. Loss returns the
  original LINE_LOST route while keeping the consumed C job/origin. Real
  shell fixtures cover SEARCH -> LINE, completion in SEARCH without taking
  over its drive, and normal SEARCH failure with LS_FAIL_NOT_FOUND (not a
  junction fault). Final C cancellation cannot change that shell cause.
- Bar clearance:33 mm plus two fresh observations, duplicate suppression and
  a renewed black observation resetting the exit count;70 mm/1.5 s bounds.
- A/B: forward-arm/stale FOUND rejected until fresh white20 ms; exact300 does
  not qualify as white;499 does not qualify as found;500 does. D skips first,
  does not count its continuing black level again, needs new white and second
  line>=120 degrees. Early second, missing line and angle/time limits fail.
- SEARCH uses131 mm/s and small virtual counts; no reverse sweep, nominal
  endpoint, settle or taper. Per-wheel stall, wrong sign, translation,
  stale input, obstacle, whole-job cap and unsigned time/encoder wrap tested.
  A stalled wheel wins over a simultaneous fresh line appearance.
- Real vehicle glue executes A/B/C/D, checks source busy/ACK and angle marks.
  Four additional GP21 fixtures stop each command while active (C after claim,
  A/B/D during SEARCH), verify cancelled result and zero motion. Busy/stale
  START board reads fail without waiting. Natural LINE loss cancels unconsumed
  pending immediately without changing the original route.
- C-packed angle events round-trip signed centidegrees through Python; first/
  accepted dashboard labels, recorder CSV-column coverage, sign/window/tag
  rejection and legacy event-v1
  kind7 acceptance tested. No summary, RAM event or KR1 enlargement.

These are synthetic checks, not proof of optical bar clearance, barcode
rejection, wheel slip, actual branch choice or final physical heading.

## RAM and stack

Matching public-config single-core MQTT, GNU Arm14.3, SDK2.3.0:

| Image | Text B | Data B | BSS B | Static RAM B |
| --- | ---: | ---: | ---: | ---: |
| `168d705` |403988 |904 |189768 |190672 |
| Old B2a `eb18004` NONE |410244 |904 |190048 |190952 |
| `7648081` NONE |408816 |904 |190052 |190956 |
| C-route fix NONE |409012 |904 |190052 |190956 |
| C-route fix A |409076 |904 |190052 |190956 |
| C-route fix B |409072 |904 |190052 |190956 |
| C-route fix C |409076 |904 |190052 |190956 |
| C-route fix D |409072 |904 |190052 |190956 |

C-route fix adds **zero RAM** versus `7648081`: **284 B** versus `168d705`,
**4 B** versus `eb18004`, within
the approved320 B estimate. LINE adapter184 B; result52 B; source record24 B,
board28 B; source sequence/ACK/busy12 B; observed phase/angle marks2 B plus
alignment. Telemetry message116 B, INFO108 B, MBF4096 B and task stacks stay
unchanged. No allocation or task is added.

| Compiler frame | `168d705` | Old B2a | Redesign MQTT |
| --- | ---: | ---: | ---: |
| Vehicle entry |880 |NONE1000/fake992 |992 all five |
| Telemetry entry |360 |360 |360 |
| Junction observer |— |144 |152 |
| Pure action step |— |128 |136 |
| C progress observer |— |— |24 |
| LINE wrapper step |— |40 |72 |
| USB formatter |— |96 |104 |
| IMU sample |1000 |1000 |1000 |

Frames are **not** whole-stack bounds. Vehicle/telemetry stacks4096 B,
IMU2048 B. Older hardware IMU480 B free (`29d57de`) and vehicle2592 B free
(`d51d27b`) are not new-image acceptance. Require named GP20 marks with
**at least256 B free** in every affected task, including IMU. Collect them
after repeated/longest runs. Timing peaks, missed samples, radio loss and
response latency still need new hardware evidence.

## J0–J6 hardware procedure

Only the operator authorizes flash/power. Main rebuilds configured images.
Keep physical GP21 ESTOP ready. Lift wheels and check direction/stop before
floor trials. Use the18 mm `+` on two tiles. Start at least200 mm before it;
mark outgoing arm headings and record video/protractor results. Do not use a
wall collision or hard wheel restraint as a stall test.

| Check | Required evidence | Redesign status |
| --- | --- | --- |
| J0 | Verify pair spacing/height, normalized floor/line/bar contrast and pending-command barcode-only rejection. Capture33 mm/two-fresh exit evidence, white20 ms and >=500 return. Lifted A/B/D direction and ESTOP checks. No axle/pivot ruler target is required. |Not run |
| J1 | NONE plain-line/`+` baseline with zero kind7 and zero losses on the new image. Old `eb18004` cross passed; repeat after redesign. |Not run |
| J2 | Repeat C with angled approaches and brief loss: normal LINE -> SEARCH -> LINE, detector suppressed160 mm across handoffs, DONE in LINE or SEARCH. No private SEARCH or blind bridge. Verify the straight outgoing course is retained after recovery; DONE during SEARCH alone is not PASS. Record normal SEARCH failure/stop if recovery fails. `7648081` results above are not acceptance of this fix. |Not run |
| J3 | A x3: white rejects forward stem; first LEFT arm found; normal LINE swings onto that arm. Measure final heading on the arm and record first/accepted SEARCH angles. |Not run |
| J4 | B x3: same checks on RIGHT arm. |Not run |
| J5 | D x3: first LEFT arm skipped; re-white; second incoming stem>=120 degrees accepted; LINE runs back. Record both angles and final heading. |Not run |
| J6 | Missing branch/line, early-second line, safety window/timeouts, cancel/ESTOP, speed variation, barcode+cross. No automatic retry, pending cancellation on natural non-LINE handoff, named stacks>=256 B free and no timing/sample faults. |Not run |

Final heading tolerance remains **at most10 degrees** after LINE settles onto
the intended arm. Report actual heading and whether LINE resumed on every
repeat. SEARCH angle at first/accepted line is evidence, NOT the final90/180-
degree arm heading. The old15 mm axle-pivot-offset gate no longer applies:
there is no axle-centred pivot target. If path/turn location is measured,
report it as observation, not a pass against the removed target. DONE alone
cannot establish physical acceptance. Do not retune locked LINE gains.

Preserve revision, image/boot label, bootID/runID, source sequence, phase/angle
kind7 events, USB marks/result, measured heading, LINE resume, stop reason,
loss/missed/overrun counters and named stacks. Keep operator reports separate
from direct inspection. Do not open/publish private logs. Leave each new gate
Not run until its evidence is supplied.
