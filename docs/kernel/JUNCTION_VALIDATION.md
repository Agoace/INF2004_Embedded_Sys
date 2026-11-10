# B2a validation and junction trials

## Status

**B2a CLOSED**, as reported by Main_Worker after the `50426ae` A regression.
All3 runs reached DONE. The worker did not push or flash. Evidence below is
specific to its tested revision; closure does not invent missing heading,
stack or recovery-path measurements. B2b is not authorized here: its proposed
40-byte record with junction-encounter generation awaits operator approval
and the contract from Main_Worker. See the
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
  was correctly cancelled with phase9. The intermediate policy cancelled
  pending at natural LINE loss; `50426ae` now preserves successful natural
  SEARCH recovery instead.

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
trials on `7648081`. The later `75a95f7` results below supersede that earlier C status.

### Operator acceptance — `7648081` J3–J5

Main_Worker supplied these 2026-11-10 MQTT/USB observations. The worker did
not inspect the logs. PASS below means operator acceptance, not invented
measurements of final heading error.

| Check | Supplied evidence | Operator status |
| --- | --- | --- |
| J3 A,3 attempts | Two claimed/DONE turns: first=accepted42.6/43.0 degrees; operator saw the left arm followed. Run2 later failed normal SEARCH at the arm/mat end. Run3 lost line before the cross and cancelled pending under the old policy. This is not three claimed powered turns. |PASS |
| J4 B,3 attempts | DONE at first=accepted -50.6/-47.6/-48.4 degrees; operator saw the right arm followed. |PASS |
| J5 D,3 attempts | Operator reported all3 worked. MQTT connected late, so no kind7 capture. Last USB result: phase7 OK, first46.27 degrees, accepted177.08 degrees, elapsed1580 ms. |PASS |

The last D stack report supplied vehicle1648 B used /4096 B total (2448 B
free) and idle272 B free. Both exceed256 B on that tested image. Other named
stack values were not supplied here; this is not new-image whole-stack proof.
Final physical heading errors were not supplied for A/B/D: these encoder
angles are line-contact observations, not final90/180-degree heading errors.

### Operator acceptance — `75a95f7` J2 C x3

Boot3219594035, 2026-11-10, operator-supplied MQTT/USB and observations:

- Run1: claim -> loss -> normal SEARCH without a junction failure. Operator
  ESTOP during SEARCH; cancellation phase8/reason1, as designed.
- Run2: claim -> SEARCH -> LINE -> DONE.
- Run3: line loss just before the cross -> pending cancelled phase9 under
  the old policy; the cross was traversed as NONE.
- Operator accepts **J2 PASS**. This is not a claim that all3 attempts
  reached DONE, nor acceptance of the newer pending-recovery policy.

The operator then approved pending survival through successful natural
LINE -> SEARCH -> LINE. Cancel on SEARCH failure/stop, SCAN, DETOUR, any
other route or STOP. No travel/time expiry is added before the B2b contract.
This specifically addresses the pre-cross cancellations in A run3/C run3.
The later `50426ae` A regression below supplies new-image acceptance;
specific pre-claim recovery/cancellation traces were not supplied for it.

### Closure regression — `50426ae` A x3

Main_Worker reported3/3 DONE on hardware, with line-found encoder angles
**45.1 /43.6 /40.8 degrees**. B2a is **CLOSED** by Main_Worker. No log was
opened by this worker. Final physical heading errors, new stack values and
which pre-claim SEARCH paths occurred were not supplied; do not infer them
from these contact angles or the three DONE results.

Main then merged CR1 motion revision `51be7a0` into `mtk3-vehicle`, producing
`235af44`, and reported: host191/191, documentation OK, template8, configured
NONE/A/BENCH builds with zero warnings, MQTT static RAM187092 B. Main reported
that `mtk3-staging` was advanced to `235af44` and both branches were pushed.
These are Main-supplied integration results, not this worker's local checks.
The software/resource tables below retain the earlier `50426ae` public-build
measurements; do not substitute them for CR1-merged image measurements.

## Software evidence

Checks below apply to `50426ae`, the pending-recovery fix based on `75a95f7`, with public example
settings in an isolated copy; Main must rebuild device-configured images.

| Check | Result |
| --- | --- |
| Strict host C/Python suite |186/186 passed |
| Clang ASan/UBSan/float-cast-overflow |186/186 passed |
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
  START board reads fail without waiting. A new real-task fixture recovers
  before the cross, verifies ACK remains unconsumed, then claims/completes C.
  A second real-task fixture loses the line before the cross, exhausts normal
  SEARCH and records an unclaimed cancellation without starting a new job.
  Pure A/B/C/D fixtures retain pending through SEARCH, ignore black samples
  while SEARCH drives, require fresh clear LINE after return, then claim.
  SCAN/DETOUR/other-route and failure/STOP cancellation remain covered.
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
| `75a95f7` NONE |409012 |904 |190052 |190956 |
| Pending fix NONE |409080 |904 |190052 |190956 |
| Pending fix A |409144 |904 |190052 |190956 |
| Pending fix B |409140 |904 |190052 |190956 |
| Pending fix C |409144 |904 |190052 |190956 |
| Pending fix D |409140 |904 |190052 |190956 |

Pending fix adds **zero RAM** versus `75a95f7` / `7648081`: **284 B** versus `168d705`,
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
| Pending route guard |— |— |8 |
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
| J2 | C normal LINE -> SEARCH -> LINE with160 mm detector suppression across handoffs. Verify straight course after recovery; DONE during SEARCH is not physical PASS. Retest pre-claim recovery under the new pending policy. |Operator PASS on `75a95f7`; new pending policy Not run |
| J3 | Earlier two DONE turns plus one cancellation on `7648081`; regression3/3 DONE at45.1/43.6/40.8 degrees on `50426ae`. Final heading error and specific recovery paths were not supplied. |Operator PASS; `50426ae` regression accepted; B2a CLOSED |
| J4 | Three RIGHT-arm turns accepted; final heading errors not supplied. |Operator PASS on `7648081` |
| J5 | Three operator-confirmed returns, last USB first/accepted angles and vehicle/idle stack values supplied; final heading error not supplied. |Operator PASS on `7648081` |
| J6 | Complete remaining missing-line/window/expiry-free pending recovery, blocked-route cancellation, barcode and named-stack/timing checks. Pending survives normal SEARCH only; STOP/failure/other routes cancel it. |Partial: C SEARCH ESTOP on `75a95f7`, vehicle/idle margins on `7648081`; other gates Not run |

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
