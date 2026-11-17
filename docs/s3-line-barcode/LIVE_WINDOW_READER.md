# Size-tolerant course window reader — B2b

## Status

`barcode_window.{h,c}` is a pure, single-owner streaming reader. It does not
read hardware, create tasks, publish commands or call vehicle actions.
The ROBOT feed and slot/control policy are now connected by the
[SHADOW producer](LIVE_PRODUCER.md). [The canonical boards](LIVE_SOURCE_CONTRACT.md)
are initialized by S3 before tasks, with an invalid SHADOW record.
Vehicle alone publishes control. No live action is enabled.

This step supplies the reader and tests for later integration, not an on-robot
live-read acceptance result. The current size extension covers a bounded
4.0--7.0 mm fitted narrow, not arbitrary small print. Existing
[TEST barcode](BARCODE.md) WIDTH/PEAK code and quiet rules are unchanged.

## Inputs and bounded work

One task owns all reader calls. These methods are not a cross-task queue API:

1. `barcode_window_begin()` copies valid current-boot calibration and config.
2. `barcode_window_adc()` queues timestamped GP28 ADC values and loss flags.
3. `barcode_window_motor()` retains original timestamped encoder pairs.
4. `barcode_window_step()` processes at most 32 queued samples per call.
5. `barcode_window_candidate()` exposes FIRST BAR time for context admission.
6. An accepted result remains immutable until explicit `rearm()` or `begin()`.

Default source bounds: 12414 counts/m, 1500 us ADC gap for the 1 kHz feed,
30000 us motor gap. GP28 is modelled at 80 mm forward and 77 mm left of the
drive axle, with a 111 mm track. The axis uses the GP28 point's differential-
drive position projected onto its direction at the first motor observation,
instead of mean wheel travel. This compensates a forward zigzag when the
barcode is perpendicular to that initial direction; it cannot infer an
unknown print angle. The offsets are physical measurements to recheck on the
robot. The first home calibration reported gaps near 1024 us.
A missing 1 ms ADC sample is rejected even without an explicit loss flag.
The future feeder must ALSO propagate acquisition missed/dropped counters;
timestamp checks are not a substitute for those counters.

There are 64 ADC slots, eight motor points and 29 run widths. No full-pass
sample capture, allocation or unbounded scan exists. Excess queue/history,
invalid raw/flags, nonpositive ADC time, excessive gaps, reverse mean wheel
progress, both wheels stationary, or a GP28-axis retreat of more than 2.5 mm
below the furthest point abandon the candidate. A smaller cumulative retreat
(a hard left steering correction) holds the stored position at the furthest
point, and ADC rows in a held interval are skipped until progress resumes.
The 2.5 mm bound is ASSUMED; see the
[FC1/FC2 analysis](SCHOOL_FAILED_START_ANALYSIS.md#approved-changes-and-host-plan).
Exact repeated motor snapshots are
idempotent; changed counts at the same timestamp are not. Per-wheel modular
counter differences handle wrap before the mean is calculated. Large counter
jumps are rejected. Stationary samples cannot earn spatial white evidence.

Distance is interpolated only BETWEEN genuine motor observations, on the
GP28-axis projection. No future
bracket means wait, not extrapolate. Older-than-history samples fail closed.
Result counts are ORIGINAL upper motor brackets, not invented interpolated
counter pairs. ADC edge/start/eligible times and the source bracket times are
separate evidence. The final-edge bracket is conservative for later arming.

## Frozen ADC thresholds and leading white

The [white-offset correction](LIVE_CALIBRATION.md) supplies calibration.
Fallback without spin white is usable only if NEW leading white qualifies;
air/flat-dark cannot self-normalize into white. Fatal calibration faults and
aborted/empty calibration are not accepted.

Before the first edge, absolute white candidates are 120--220 ADC counts.
The reader tracks their mean with a 1/16 update and requires spatial leading
white. It derives `rise=min(white+150,350)` and
`fall=min(white+75,rise-75)` BEFORE admitting the first bar, then freezes both
through all 29 elements and short white confirmation. Mat black never sets
the threshold scale. A short bounded transition from white to the first bar
is allowed; it is not counted as credible leading white.

Leading white must cover 40 mm both before admission and after fitting the
start star. Any above-rise black that is not admitted clears the white
qualification, even if the black run is short. Transition-band handling
below rise stays bounded as before. Mid-window adaptation is forbidden.
See the [FC1/FC2 analysis](SCHOOL_FAILED_START_ANALYSIS.md).

## The course window, not a trailing quiet-zone repair

Accept exactly:

```text
start star (9) + narrow gap (1) + data (9) + narrow gap (1) + stop star (9)
= 29 elements = 15 bars + 14 spaces
```

Both print orientations are accepted while the robot drives FORWARD.
There is no TIME-only fallback. Each group ranks three wide elements, fits
bar/space edge bias without a requested letter, and uses the full ordinary
Code 39 symbol table. Only a single uppercase A--Z is eligible for mapping.
Digits/punctuation are recognized but rejected for course actuation.

Current size gates, to verify in SHADOW hardware. The start star supplies the
scale; a second fixed absolute wide band does not constrain it:

| Check | Bound |
| --- | --- |
| Corrected narrow | 4.0--7.0 mm (ASSUMED scope, not measured optical resolution) |
| Corrected wide | Fitted from the three wide elements; ratio/consistency checks below |
| Wide/narrow ratio | 1.6--3.4 |
| Minimum class separation | 1.4 |
| Each element against its class mean | 0.65--1.5 |
| Data/stop scale against start | Within 20% |
| Corrected inter-character gaps | 0.65--1.5 start narrow |
| Total corrected body extent | Within two narrows of `20n + 9w` |
| Leading white | 40 mm before and after fitting; unadmitted black clears qualification |
| Pre-star white-to-ink transition allowance | 7 mm maximum candidate narrow (unchanged) |
| Candidate time/body limits | 5 s / 360 mm (unchanged absolute watchdogs) |
| Run limit before start-star fit | 35 mm absolute ceiling |
| Run limit after start-star fit | Smaller of 35 mm and `1.5w + abs(bias)` |

The body formula is about 273.5 mm at n=5.8 and w=17.5. It validates one
candidate; it is NOT a 285 mm sensor blackout or a delay based on map geometry.
With a wide/narrow ratio of three, approximate 274/210/141 mm bodies give
n=5.83/4.47/3.00 mm. The first two are in scope; 141 mm remains out of scope.
The 210 mm home-sheet ruler measurement is approximate, not a measured fit.
The cross-axis GP28 sigma estimate of about 2.15 mm cannot establish the
along-travel optical resolution. The 4 mm floor is ASSUMED. Do not lower it
without straight, full-rate GP28 and encoder captures.

The start, data and stop fits retain the geometry and bar/space bias correction
from `52244f2`. No sensor offsets, forward-axis projection, ADC hysteresis,
source-gap checks, command mapping or one-narrow trailing proof change.

After the fifteenth bar ends, require continuous credible white for one
narrow width (the larger fitted start/stop narrow). Ink before that proof
rejects the window. White proof has a bounded 2.5n opportunity, not arbitrary
look-ahead. Once accepted, later junction ink, ADC faults or motor reversals
cannot change that result. No 58 mm trailing quiet is required here.
Missing/merged/extra runs are never inserted, deleted or repaired. A failed
candidate requires fresh leading white, not sliding resynchronization inside
that damaged barcode. Without a checksum, damage that becomes another fully
legal waveform cannot be distinguished from that legal letter.

## One mapping table

Build-time `BW_MAP_VALUES` has 26 entries. Operator-approved defaults are
A=LEFT, B=RIGHT, C=STRAIGHT, D=UTURN; every other letter is NONE.
Default mapping policy ID is 1 and confirmation is 1 for this approved table.
An invalid/unconfirmed table makes all action observations NONE. There is no
special F path. Changing F to an action is a table edit, not decoder training.

Helpers return table entries, confirmation and policy ID. Vehicle boot logging
will print this same table once when integration lands; no second hardcoded
copy is permitted. A mapped value remains data here, never an executed action.
SHADOW and the unverified 150 mm guard cannot enable a real action.

## Size-extension acceptance gates

| Gate | Result and limit |
| --- | --- |
| RED before implementation | Original reader aborts on the 210 mm synthetic acceptance assertion (exit 134). |
| GCC host | 536/536 pass, including size cases. |
| Clang ASan/UBSan host | 536/536 pass. |
| Size corpus | All 26 letters, both print directions, rectangular 274/210 mm bodies decode; 141 mm rejects. ASSUMED optics. |
| Bounds and negatives | 3.9/7.1 mm narrow reject; 4.05/6.95 mm pass. Bad ratio/gap/scale, missing/merged bars, short white, long run, line-only and synthetic junction ink reject. |
| Original optical fixtures | Existing large forward F fixtures still pass; backward drive still rejects. No measured 210 mm optical fixture yet. |
| Hash76 against `25f70a7` | 38 warning-free profiles: 48 image hashes differ as expected; 28 match. Only `barcode_window.o` changes in every object comparison. |
| Host-only optical SIL | PASS: 180 ASSUMED stripe cases, 64 correct accepts, no wrong letters/negative decodes; all 141 mm cases reject. No Webots process or hardware test. |
| Home straight 210 mm SHADOW test | NOT RUN; wait for review. Record body/quiet dimensions and full-rate GP28/encoder data. |

The host-only SIL uses infinite transverse stripes on white paper, a linear
ASSUMED ADC response and exact no-slip motion, not the finite-paper or fitted
nonlinear GP28 model. The straight 274/210 mm corpus passes at ASSUMED
along-sigma 0/0.5/1 mm and rejects at 2.15/4 mm. High-blur/yaw rejections
remain in the result. This does not demonstrate physical 210 mm readability
or a safe minimum narrow width. The simulation branch stores the full record
as `docs/simulation/S3_SIZE_READER_SIL.md`.

Hash76 uses identical export/build paths for baseline and candidate. For each
of UP, split and forced-core0 control placement, the UF2 and ELF change for
`usb`, `wifi`, `lwip`, `mqtt`, and `mqtt-jA` through `mqtt-jD` (24 profiles).
These are exactly the images that retain `barcode_window_step`, checked with
`arm-none-eabi-nm`. For each placement, `test`, `bench`, `mqtt-test` and
`mqtt-bench` match, as do `capture-f0/f1` (14 profiles). Thus all other 28
UF2/ELF hashes match. The default robot image changes intentionally because
its live reader uses the new size policy. No other object hash changes.
Local evidence is under `build/size-reader/` and `/tmp/sw10-size-h76{,.base,.cand,.base-log}`;
these compiler/profile results are not hardware acceptance.

### School attempts: context, not full-rate optical replay

MEASURED timestamps below come from the final GP20 dump of each `b0c3a9a`
run. BASE run start is 19.627 s uptime; RAM run start is 81.189 s. KR rows
are nearest retained rows, not reconstructed edge brackets.

| Run/attempt | Start uptime s | Run s | KR row | Failure |
| --- | --- | --- | --- | --- |
| BASE 1 | 23.671988 | 4.045 | 199 | BW_START |
| BASE 2 | 25.201988 | 5.575 | 272 | BP_CONTROL |
| BASE 3 | 29.699989 | 10.073 | 493 | BP_DISABLED |
| BASE 4 | 68.693986 | 49.067 | 2413 | BW_MOTION |
| BASE 5 | 69.966989 | 50.340 | 2473 | BW_WIDTH |
| RAM 1 | 85.458045 | 4.269 | 203 | BW_START |
| RAM 2 | 86.772046 | 5.583 | 261 | BW_WIDTH |
| RAM 3 | 91.279045 | 10.090 | 457 | BP_DISABLED |
| RAM 4 | 130.125045 | 48.936 | 2293 | BW_START |
| RAM 5 | 131.499045 | 50.310 | 2357 | BW_WIDTH |

ASSUMED: the first two candidates belong to the one barcode after the first
turn. A candidate is not a separate physical barcode. Later candidates are
not proven negatives from ADC alone. MQTT samples GP28 at about 5 Hz, too
slow to count its 15 bars or replay the live 1 kHz edge widths. The first
cluster shows two sampled dark runs in BASE and three in RAM. Near candidate
one, measured yaw is -21.2/+25.3 degrees/s and requested left/right speeds are
181/111 and 102/190 mm/s respectively. Yaw reaches about +/-54.2 degrees/s
within the cluster. Size rejection alone is not a demonstrated cause.

These ten attempts constrain the context review, not a measured false-positive
rate for the new policy. Do not interpolate 5 Hz ADC into an asserted full-rate
regression. Wider size admission can admit more false starts; complete stars,
class separation, gaps, scale agreement, single-letter framing, source health
and fresh leading white remain mandatory. Keep SHADOW until hardware review.
Local source logs: `build/kram_robot_b0c3a9a/robot_{BASE,RAM}.final.log` and
`mqtt_{BASE,RAM}.csv` in the review workspace. They are not public fixtures.

## Historical evidence and limits

- At step3: strict host and Clang ASan/UBSan297/297, including35 window cases.
- All 43 data symbols in both print orders, across default/custom/invalid/
  unconfirmed tables; all 26 uppercase letters decode in forward motion.
- Every missing bar and merged space, added edge, malformed gap, multi-letter,
  small print, too-short leading/trailing white, single missed ADC, source
  faults/gaps/reversal, wrap, queue bounds, immutable result and rearm tested.
- Paper, flat dark/air and line-only negatives reject.
- Two ORIGINAL forward large captures decode F/NONE. The original backward
  cable capture rejects because the ROBOT drove backward (negative encoder
  progression), not because the PRINT was reversed.
- Synthetic compression of fixture timestamps tests delivery timing only;
  optical response at 146 mm/s is NOT proven. Historical retained fixtures
  explicitly use a host-only 3000 us ADC / 50000 us motor gap profile, not
  weaker live defaults. BC_RUN rows never provide classification or targets.
- Five warning-free firmware profiles: normal, capture, BENCH, MQTT and fake-A.
  Strict ARM compilation covers 15 units; SDK link probes normal and fake-A.
- ARM context is 988 B, result 64 B; context assertion is at most 2048 B.
  One forced-retention LINK-ONLY probe adds exactly 988 B RAM and 5264 B text
  including the probe wrapper. No frozen TEST workspace is pulled in.
- At step3, the context was not allocated. Invalid-record startup added48 B
  RAM /152 B text in a matched-profile relink against7177edd. The subsequent
  [producer record](LIVE_PRODUCER.md) budgets the actual connected state.
- Compiler frames: model 176 B, begin 128 B, rearm 88 B, step 128 B.
  These are individual frames, not complete call-chain/runtime stack proof.
  CPU target remains below about 2% average and 1 ms task slices; NOT measured.

Artifacts and link-only probes are under `build/b2b-validation/`.
Never flash `step3-RETAINED-NOT-FOR-FLASH.elf`. Public-example Wi-Fi builds
are validation artifacts, not configured operator images. No push/flash was
performed. Hardware live reading, same-stripe offset bound, measured stack/CPU
and vehicle admission remain integration gates. Producer heartbeat/revision/
slot policy is covered in the subsequent producer record.
