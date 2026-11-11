# Large course window reader — B2b step 3

## Status

`barcode_window.{h,c}` is a pure, single-owner streaming reader. It does not
read hardware, create tasks, publish commands or call vehicle actions.
The ROBOT feed and slot/control policy are now connected by the
[SHADOW producer](LIVE_PRODUCER.md). [The canonical boards](LIVE_SOURCE_CONTRACT.md)
are initialized by S3 before tasks, with an invalid SHADOW record.
Vehicle alone publishes control. No live action is enabled.

This step supplies the reader and tests for later integration, not an on-robot
live-read acceptance result. Small-print work remains deferred. Existing
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
30000 us motor gap. The first home calibration reported gaps near 1024 us.
A missing 1 ms ADC sample is rejected even without an explicit loss flag.
The future feeder must ALSO propagate acquisition missed/dropped counters;
timestamp checks are not a substitute for those counters.

There are 64 ADC slots, eight motor points and 29 run widths. No full-pass
sample capture, allocation or unbounded scan exists. Excess queue/history,
invalid raw/flags, nonpositive ADC time, excessive gaps or negative MEAN
encoder progress abandon the candidate. Exact repeated motor snapshots are
idempotent; changed counts at the same timestamp are not. Per-wheel modular
counter differences handle wrap before the mean is calculated. Large counter
jumps are rejected. Stationary samples cannot earn spatial white evidence.

Distance is interpolated only BETWEEN genuine motor observations. No future
bracket means wait, not extrapolate. Older-than-history samples fail closed.
Result counts are ORIGINAL upper motor brackets, not invented interpolated
counter pairs. ADC edge/start/eligible times and the source bracket times are
separate evidence. The final-edge bracket is conservative for later arming.

## Frozen thresholds and leading white

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

Leading white must cover ten fitted narrow widths at the start star. The
pre-search gate is only provisional; matching the first star cannot waive
that final leading-white test. Mid-window adaptation is forbidden.

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

Initial large-print gates, to verify in shadow hardware:

| Check | Bound |
| --- | --- |
| Corrected narrow | 4.6--7.0 mm |
| Corrected wide | 13.5--21.5 mm |
| Wide/narrow ratio | 1.6--3.4 |
| Minimum class separation | 1.4 |
| Each element against its class mean | 0.65--1.5 |
| Data/stop scale against start | Within 20% |
| Corrected inter-character gaps | 0.65--1.5 start narrow |
| Total corrected body extent | Within two narrows of `20n + 9w` |
| Candidate time/body/run limits | 5 s / 360 mm / 35 mm |

The body formula is about 273.5 mm at n=5.8 and w=17.5. It validates one
candidate; it is NOT a 285 mm sensor blackout or a delay based on map geometry.

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

## Evidence and limits

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
