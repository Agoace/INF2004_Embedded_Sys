# Subsystem 3 kernel line acquisition

| ID | Procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S3-U01 | TEST=1, wheels lifted. Move a black/white card under GP27 L and GP26 C during the 1 s window. | Actual TIMER window count within 1%; both raw means 0..4095 and nonzero spread; no missed conversions/alarms | PASS 2026-11-06: 1009 samples in 1 s, 0 missed (no card moved; spread from the floor only). [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |

Alarm 0 / TIMER IRQ0 runs on core 0 at priority 2, below GPIO IRQ13 at 1.
Each 1 ms sample is the integer mean of eight ADC conversions per channel,
L then C; right is zero, matching the bare-metal two-sensor path. A 128-slot
queue transfers timestamped samples to task context. TEST deliberately
leaves the queue undrained during measurement; queue drops in that window
do not mean acquisition missed samples. A robot run must have no new
queue drops or missed alarms.

The app releases ADC reset and selects PLL_USB=48 MHz / 1 for CLK_ADC.
The template ADC device `adca` stays unopened. No template device,
configuration or kernel source is changed. GP29/ADC3 is never selected.

## Task-split image checks

| ID | Procedure and acceptance | Status |
| --- | --- | --- |
| S3-U02 | TEST: line task processes timestamped samples, no queue drops; snapshot age <=10000 us; report task elapsed max/sample, total and per-mille window cost; report three-channel ISR max/total/per-mille and GP28 ADC2 range | PASS 2026-11-06: age <= 993 us, task 7–13 per-mille, ISR 69–75 per-mille, ISR max 114 us. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| S3-U03 | Bench only, do not drive: GP28 analog barcode input, VBAT jumper open; show bars at intended height and record raw range, narrow-bar visibility and ISR cost | Not run |

S3 now averages eight ADC conversions on each of L/ADC1, C/ADC0 and
barcode/ADC2 every 1 ms (24 total). In the normal robot, barcode remains
raw observation only. The exclusive B1 profile below adds TEST decoding;
no Ziven reference source is changed. Pure line logic receives L/C/zero,
never ADC2. See the [control architecture](../../docs/archive/architecture/CONTROL_ARCHITECTURE.md)
for the GP28 wiring and bar timing constraints. ISR timing excludes wrapper
entry/exit and its final statistics stores; task timing is elapsed work,
including any higher-priority preemption, an upper bound on CPU cost.

## B1 barcode software validation

Tested source: B1 `70caef0`, based on staging `b569adc`, on `mtk3-line`.
These software results do not certify the later B1b powered profile.

| Check | Result |
| --- | --- |
| Full strict host suite | PASS: 136/136, including six barcode checks |
| Standard table | PASS: 43 data symbols x 2 polarities x 2 directions |
| Framing | PASS: `*A*`, `*Z*`, F, strings, both width methods |
| Command safety | PASS: only single A/B/C/D map; F/Z/strings have NONE |
| Independent methods | PASS: varying time/encoder widths, missing motion |
| Rejection | PASS: low contrast, loss, glitch, crop, limits, ambiguity |
| Arithmetic | PASS: timestamp wrap, negative motion, null inputs |
| Capture | PASS: 4096-row freeze, no overwrite, gaps, motion overflow |
| Actual TEST observer | PASS: three ranges, GP20 arm/freeze, frozen dump |
| Actual line-task hook | PASS: 18 copied raw samples, normal tests retained |
| Exclusive runner | PASS: priority 20; skips S2 and all ordinary tests |
| Priority failure | PASS: no barcode or other test runs on failure |
| Clang ASan/UBSan | PASS: pure decoder and capture tests |
| ARM strict checks | PASS: 54 app compile invocations with warnings as errors |
| Make profiles | PASS: nine profiles below, no warnings |
| Invalid profiles | PASS: reject non-TEST, BENCH, IMU_STREAM, SMP, radio/net |
| Static capture image RAM | data 152 + BSS 159812 = 159964 bytes |
| ARM decoder workspace + report | 1728 + 64 = 1792; assert <=2048 |
| Host workspace + report | 1728 + 88 = 1816 bytes |
| Capture arrays | 38912 bytes |
| Normal radio-off ELF | No barcode/capture/workspace symbols linked |
| Documentation / template / whitespace | PASS; template count remains 8 |

Host warnings: `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`.
ARM checks use the profile's own compile command plus those warning flags.
The nine Make profiles are radio-off normal, TEST, BENCH; MQTT normal,
TEST, DIAG; default SMP/radio; radio-off IMU_STREAM; radio-off barcode.
Only the tracked Wi-Fi example was used temporarily for build settings;
no real credentials were copied, and the temporary header was removed.
GCC ASan could not link its missing local runtime; Clang sanitizers passed.
No firmware was flashed by this worker.

Independent F fixtures use 29/87 ms and 58/175 ms element widths at an
assumed 100 mm/s. Their ideal frame widths are 136.3/273.5 mm, with white
margins added; these are not sensor traces or exact measured print totals.
The operator's photo and ruler confirm F, not Z: two gaps differ from Z.
Expected physical result is **F, DECODED, NO command**. Course samples are
standard A and Z; they do not change the operator print's expected letter.

## B1 hand-push procedure and operator follow-up

The procedure below is historical, for `70caef0` only. Do not apply its
buttons to the current B1b image: GP21 now drives in that exclusive profile.
Use the [current B1b contract](../../docs/s3-line-barcode/BARCODE.md).

| ID | Check | Status |
| --- | --- | --- |
| S3-B1-01 | Stationary contrast ranges | PASS; corrected placement |
| S3-B1-02 | 2x F, straight hand push | FAIL; capture-window procedure |
| S3-B1-03 | Reverse order and course-size F | FAIL; capture-window procedure |
| S3-B1-04 | Loss, timestamps, dump counts and timing | Partial evidence below |
| S3-B1-05 | Normal LINE regression | Not run |

1. **S3-B1-01:** At installed height, GP20 records FLOOR, WHITE_PAPER and
   BLACK_BAR for 1 s each. Save raw min/max and sample counts. Judge actual
   contrast and narrow-bar visibility; floor-only spread is not proof.
2. **S3-B1-02:** HAND PUSH STRAIGHT over 2x F first: about 285 mm total,
   narrow 5.8 mm, wide 17.5 mm. Arm with GP20, finish within 4.1 s, freeze
   or await full. Retain both method results and complete raw/motion/run
   dump. Expect F/DECODED/NONE; no guessed letter.
3. **S3-B1-03:** Repeat 2x F with reverse print order. Then repeat both
   directions on the about 141 mm F copy: narrow 2.9 mm, wide 8.7 mm.
   Keep sensor height fixed and record quiet-zone margins.
4. **S3-B1-04:** Inspect missed/dropped deltas, ADC gaps/full, motion
   loss/overflow, observer misses, hook cost, decoder step/total cost and
   line/ISR statistics. Compare source timestamps and dump counts. No
   timing PASS until measured.
5. **S3-B1-05:** On a fresh normal image, repeat normal LINE acquisition
   and track checks. Host checks do not establish physical performance.

Before a pass: verify the open GP28 VBAT jumper, mount clearance, USB
console, CRASH_REPORT=0 and the `_test_barcode_capture` filename. This
profile skips the automatic S2 motion tests and requests stop. Zero duty
is coast. Do not press GP21; it is not a barcode control. GP7 is GY-511
INT1 now and is never configured by B1. Do not change wiring or mounting.

Save all three contrast ranges before the first pass. Start and end over
white margins; quiet zones need ten narrow widths, and a nearby line may
truncate one. After freeze, wait for both results and `[BC_DUMP] END`.
Verify BEGIN/END counts and every ADC/motion row; USB loss invalidates the
saved evidence. Save it before GP20 rearms and replaces the capture.

Hand-push encoder distance is unreliable because back-driven gearmotor
wheels slip. Time widths are therefore required for this first contrast
and geometry check, but changing push speed can distort them too. Keep
both results and reject disagreement rather than calling it a command.
No powered trial, weaving correction, copy vote or robot routing is part
of B1. These checks were Not run at the initial software validation.

### Operator evidence on B1 `70caef0`

Supplied by Main after nine hand-push captures; private logs were not read
or copied by this worker. The operator confirmed good static contrast:

| Surface | Corrected `raw_bc` range |
| --- | --- |
| White paper | 151..170 |
| Floor | 155..172 |
| Black bar | 1334..1660 |

Black is HIGH; floor reads like white. The **first** contrast round had
incorrect placement: the supposed white read about 1500. Do not use that
round as white evidence. S3-B1-01 PASS covers the corrected contrast only,
not a narrow-bar resolution or decoding claim.

All nine captures failed to decode: CONTRAST when no bars were observed,
or NO_FRAME/MOTION. Bars arrived only during the last 0.2..0.5 s of the
4.1 s window because the push started too late. Encoder travel was 0..70 mm,
consistent with hand-push slip. S3-B1-02/03 are **procedure FAIL**, not a
verdict on the decoder. Print/direction coverage is not separately proven.

Observed: decode_step_max_us up to 1010; isr_max_us 113; missed/dropped 0;
adc_full 1 on every capture. These are measured values for that B1 image,
not timing or powered-decoding acceptance for B1b. Stack checks and normal
LINE regression remain Not run.

## B1b software validation — `afcdea2` exclusive driven profile

This source change descends from B1 `70caef0`. It does not include a powered
hardware PASS. The final operator plan uses **one sheet per run**, swapped
between runs: default 600 mm forward/backward, target clamp <=620 mm. The
earlier two-sheet / 850 mm proposal is not implemented.

| Check | Software result |
| --- | --- |
| Full strict host suite | PASS: 154/154 |
| Original standard table | PASS: all 43 data symbols, polarities/directions |
| Packed records / frame reports | PASS: both methods, wrap, positions/loss |
| Multiple host frames | PASS: independent F/Z frames; ambiguous summary |
| Late frame | PASS: frame after 5 s, beyond the old hand-push window |
| Drive bounds | PASS: forward/backward, distance, timeout, direction faults |
| Compile-time clamp | PASS: requested 850 maps to 620; default stays 600 |
| Longer slow pass | PASS: 600/25 settings derive 27 s deadline, 5/60 ms |
| Actual TEST observer | PASS: 15 normal/fault/button/stop-retry paths |
| GPIO adapter | PASS: only GP20/GP21 inputs; GP7 never configured |
| Line hook / exclusive runner | PASS: raw copy and ordinary-path checks |
| Clang ASan/UBSan | PASS: decoder, packed, capture, drive, observer paths |
| Firmware profiles | PASS: nine warning-free profiles, as listed for B1 |
| Strict ARM app compile | PASS: 63 invocations, warnings as errors |
| SDK hardware link proof | PASS: nine profiles; probe ELFs never flashed |
| SDK profile mismatch | PASS: incorrect BARCODE_CAPTURE flag is rejected |
| Static image RAM | data 152 + BSS 152204 = 152356 bytes |
| Capture arrays | ADC 24576 + motion 6144 = 30720 bytes |
| ARM decoder workspace/report | 1760 + 64 = 1824 bytes, assert <=2048 |
| Normal radio-off ELF | No barcode/capture/drive workspace symbols |
| Docs / template / columns / whitespace | PASS; template deviations remain 8 |

The observer tests cover a full forward/backward synthetic F crossing;
GP21 early stop; delayed stop API confirmation; command, motor, stale or
missing board, sensor, acquisition and wake faults; timeout; capacity and
held buttons, and refusal to start from a non-stopped snapshot. No dump
is allowed while the simulated drive is active or
stop is unconfirmed. GP21 before range completion and GP20 while driving
cannot start another job. Forward/backward frame positions are independent
of the whole-capture summary. Multiple frames are a host robustness test,
not a request to place two sheets in the physical run.

Capture is now 6144 four-byte raw/flags + microsecond-delta records, with
default 2 ms decimation (12.288 s), plus 512 source-stamped motion records
at default 30 ms. Intervals derive from deadline plus 2 s margin, including
slower settings. No extra ADC consumer/ISR setup or normal LINE actuation
was added. At capacity, appends stop without overwrite, but freeze waits
for the checked stop. Raw gaps are checked even on skipped records.

The SDK checker was extended to include BARCODE_CAPTURE in its profile
identity. Build settings used only the tracked Wi-Fi example temporarily;
no real settings were copied or retained. No image or link probe was
flashed by this worker. Clang sanitizer builds for the TEST print double
use the normal warning set; GCC host/ARM strict checks include -Wformat=2.

Initial task stack is 1 KiB. Owner boards/state were moved to static RAM;
compiler function frames are owner 280, decoder step 296, character 112
bytes. This avoids the former 624-byte owner frame, but does not constitute
complete call-graph or runtime high-water proof. No template stack change
was made. Powered timing/stack acceptance is still Not run.

## B1b powered checks — revision-scoped

Use the [current contract](../../docs/s3-line-barcode/BARCODE.md) for the
full button, settings, loss and output rules. Do not use B1 hand-push
button instructions on this image.

```sh
make -C build_make SMP=0 WIFI=none NET=none TEST=1 BARCODE_CAPTURE=1 -j4
```

| ID | Physical check | Status |
| --- | --- | --- |
| S3-B1b-01 | VIN, lifted wheels | 0cd2c03 PASS, both directions |
| S3-B1b-02 | One 2x F sheet, floor | 0cd2c03 F both; back cable caveat |
| S3-B1b-03 | One 141 mm F sheet | 0cd2c03 FAIL: NO_FRAME both ways |
| S3-B1b-04 | Stop/coast and inhibition | 433 mm observed; faults pending |
| S3-B1b-05 | Cost, loss, USB and stack | Loss/dump PASS; stack pending |

### Supplied first lifted trial — `afcdea2`, 2026-11-09 about 18:05

Facts supplied by Main; the private hardware log was not read.
VIN was on and wheels lifted. Contrast PASS: FLOOR 152–170,
WHITE_PAPER 158–174, BLACK_BAR 1206–1492. GP21 accepted FORWARD,
then within about 30 ms reported STOP UNCONFIRMED, STOP CONFIRMED /
FAULT / travel_mm 0, and inhibited 1. Wheels never turned. Dump contained
15 ADC and three motor rows. Missed/dropped, gaps, full, capture drops,
motor misses/overflow and observer misses were all zero.

This is a health-guard failure before powered movement, not a decoder
verdict or a stop-distance PASS. The old image did not report the condition
that set FAULT. Root cause remains unknown. A sequence-copy collision was
incorrectly fatal and is one candidate. Line normally publishes every
1 ms; motion every 10 ms. The watchdog flag is armed before commands and
is not cleared by motion_set_speed; no startup grace is justified by that
code. The first STOP UNCONFIRMED notice alone does not prove API failure.

### Snapshot-health and fault-diagnostic correction

The TEST owner now caches only successful coherent board copies. A busy
E_TMOUT read retries on the next 10 ms tick. Fault after three consecutive
failed reads or source age above 30000 us, whichever applies first. Other
read errors, invalid/raw-out-of-range line data, motor faults and false
watchdog flag remain immediate faults. Source stamps are never fabricated.
Stop confirmation still requires two distinct fresh zero snapshots.

Every FAULT transition and START REFUSED prints BC_FAULT with cause mask,
line rc/age/valid/raw/failure count, motor rc/age/failure count/faults,
watchdog flag, actual wake rc and queued wakes. Stop is requested before
printing. See the [mask table](../../docs/s3-line-barcode/BARCODE.md).

| Correction check | Result |
| --- | --- |
| Full strict host suite | PASS: 168/168 |
| Actual owner paths | PASS: 29, including 14 new health cases |
| Busy copies | PASS: one/two busy reads, both boards, busy at start |
| Line freshness | PASS: 10 ms accepted; >30 ms stops/refuses |
| Bad line state | PASS: invalid, raw above 4095, hard read errors |
| Watchdog false | PASS: refuses start or stops at first active tick |
| Failure diagnostics | PASS: expected cause bits and values present |
| Clang ASan/UBSan | PASS: all 29 owner paths |
| Firmware / SDK proofs | PASS: nine warning-free profiles |
| Strict ARM application compiles | PASS: 63 |
| Static capture image RAM | data 152 + BSS 152468 = 152620 bytes |
| Owner / diagnostic compiler frames | 304 / 56 bytes |
| Decoder workspace/report and arrays | Unchanged: 1824 and 30720 bytes |
| Docs / whitespace / template ledger | PASS; template count 8 |

No flash or powered retest was performed by this worker. Replug/reboot to
clear inhibition, repeat contrast and lifted-wheel forward/backward/GP21
checks before floor work. Save BC_FAULT if it stops. Actual physical
stop/coast, decoding, runtime stack/timing and normal LINE regression
remain unaccepted.

1. Verify `_test_barcode_capture` and the B1b DRIVEN banner. VIN is required.
   Open GP28 VBAT jumper, leave GP7/IMU wiring unchanged, clear the mechanism
   and lift wheels. Keep VIN removal available. Wait for initial confirmed
   stop. Redo GP20 floor/white/black ranges at the actual installed height.
2. Lifted: GP21 starts forward; release it. Check positive equal requests,
   bounded travel/deadline, then zero duty and speed <=10 mm/s in two fresh
   post-stop source snapshots. Wait for the full dump. A new GP21 press
   starts backward. Also check a fresh press while moving requests stop.
   No lifted run proves optical geometry or physical floor distance.
3. Reboot to restore FORWARD. Place one 2x F sheet, bars across travel; align
   the front-left sensor before its white margin. Leave quiet zones and
   coast room within the available run space. Do not use the old 85 cm
   two-sheet layout. Current target must not exceed 450 mm; default is
   430 mm, about 4.3 s at 100 mm/s before acceleration/stop allowance.
   The 2x F quiet margins each need >=58 mm, within that 430 mm path.
4. After fresh contrast ranges, GP21 starts capture and the forward pass.
   Freeze/decode/dump must wait for confirmed stop. Save TIME and DISTANCE,
   each `[BC_FRAME]` entry/exit position, final left/right travel, all raw
   ADC/motion/run rows and counters. Expected: F, DECODED, command NONE.
5. Save `[BC_DUMP] END`, verify row counts, and wait for READY before GP21
   backward. Full output can take tens of seconds. One notice explains a
   press ignored during the dump; it never queues a start.
   If stopped early, reposition before another full-distance run. Swap to
   the 141 mm F sheet only while idle and repeat; one sheet per run.
6. Any fault/deadline/capacity stops and inhibits restart. Stop API success
   alone is not enough: retries continue until fresh zero-duty/low-speed
   snapshots confirm it. No dump/rearm while pending. If wheels still move,
   remove VIN. Record coast/stop distance; >450 mm reports OVERRUN. Encoder
   slip/coast mean the target clamp is not an independent physical ruler.
7. Record new copy/decoder/line/ISR cost, missed/dropped/gap/overflow and
   observer counters, USB integrity and stack high-water. Do not carry B1's
   1010/113 us observations into this powered image as a PASS.

### Supplied 45040c7 lifted forward PASS

Main supplied contrast PASS: FLOOR 151–169, WHITE_PAPER 151–168 and
BLACK_BAR 1335–1576. With VIN on and wheels lifted, forward finished at
DISTANCE, travel 603 mm (left/right 603/603), without FAULT. The complete
dump had 3030 ADC and 179 motor rows, then next BACKWARD / inhibited 0.
This passes the FORWARD part of S3-B1b-01 under the old 600 mm target.
Backward has not run. It is not proof of floor stop distance or optics.

The dump took about 40 s. GP21 was silently ignored during output, which
made the operator think backward had failed. That was a UI problem, not
a reported command refusal or fault. Current output announces estimated
row count/time, warns once on an arriving dump-time press, then prints
READY with the next direction. Held/ignored presses cannot queue a start.
Full raw output remains the default; no faster batching is enabled.

### Supplied first floor 2x F capture and optical correction

The old image drove forward 602 mm without a fault. Contrast was 1839;
only 15 runs survived the old thresholds, so both methods gave NO_FRAME.
Main approved its extracted ADC/MOTOR/RUN CSV as public-safe fixture data;
that extraction, not the private console log, was read and copied into
`tests/fixtures/barcode_large_f_forward.csv`. It contains 3080 ADC and
180 motor rows. The 15 old BC_RUN rows are not decoder training input.
SHA256:
`a10e86e72b88944aeb310e1ce2d7a7a5e6c9f3fe22764e6c4d607d2ff667de0f`

Supplied analysis: full black about 1500, white about 150; 5.8 mm narrow
bars peak only 600–900 and appear 2–3 mm wide near the old middle level.
White spaces grow while bars shrink by about 3 mm per edge pair. Lower
levels recover the full star/F/star sequence. Dark floor starts after
about 450 mm; it also raises the learned full-range maximum.

The corrected detector uses a 10% rise / 6% fall band above learned min,
with minimum margins 32/16 ADC counts to exceed measured white jitter.
For this trace (150..1989), rise is 333.9 and fall 260.34; a retained peak
must be at least 334. That is below the measured 2x peaks and below a
linear half-width estimate for the smaller print. Synthetic 141 mm cases
pass at peak 350, and reject peak 320 under the same full contrast.
Physical 141 mm peaks/valleys remain unmeasured; no missing bar is guessed.

Each character still selects exactly the three widest of nine. Separate
narrow bar/space means estimate additive blur bias. The corrected widths
must retain strict ratio/separation/spread bounds; no requested letter
changes a mask. Character gaps use the space reference, quiet zones the
physical narrow estimate. Full-contrast polarity tests remain; partial-
contrast tuning is for the measured black-HIGH sensor.

Host replay of the WHOLE real trace now gives F / DECODED / NONE from
both TIME and DISTANCE, forward and reversed (34 measured runs). The
forward frame spans about 73–348 mm. The 2220-ADC prefix ending at the
last encoder observation below 430 mm also decodes F by both methods,
without the dark tail. Reversed replay is not a backward hardware test.
New powered decoding
still needs retest; a software replay is not a new physical PASS.

### Short-map operator change and software validation

The operator's map now permits at most 450 mm travel. Current default is
430 mm at 100 mm/s, compile-time clamp 450; the lifted ~3 mm overrun leaves
margin at this default. The old 600/620 settings above are historical.
Default deadline is 7.3 s, retention ADC 2 ms / motion 20 ms, sized for
deadline plus 2 s. At 25 mm/s: deadline 20.2 s, retention 4/50 ms.
Motion decoder gap allowance is period +30 ms, accounting for two allowed
busy polls and source/poll phase; live-board age still must be <=30 ms.
The raw dump remains full. Its estimate uses the observed ~40 s / 3209
rows; host drain/tick rounding can make actual time differ. Faster output
is proposed, not enabled: opt-in summaries or measured, back-pressure-aware
batches. Blindly removing yields risks the 4096-byte USB TX ring dropping
bytes before lower-priority USB can drain it.

| Current software check | Result |
| --- | --- |
| Full strict host suite | PASS: 173/173 |
| Real capture, both methods and reversed order | PASS: F / NONE |
| Synthetic small-print partial peaks / color bias | PASS: 43 x 2 directions |
| Below-minimum peak | PASS: rejects 320 under 150..1989 span |
| Existing table / rejects / fault diagnostics | PASS, retained |
| Dump presses / held through READY | PASS: no queued start, one warning |
| Distance / deadline / 450.1 mm overrun | PASS |
| Two busy polls with source/poll phase | PASS: valid 50 ms retained gap |
| Clang ASan/UBSan | PASS: decoder units and 32 owner paths |
| Warning-free firmware / SDK proofs | PASS: nine profiles |
| Strict ARM compiles | PASS: 63 |
| Static RAM / decoder workspace+report | 152620 / 1824 bytes |
| Owner / decoder-step / character compiler frames | 320 / 304 / 144 bytes |
| Docs / columns / template ledger | PASS; template count 8 |

No new firmware was flashed by this worker. Repeat lifted forward/backward
at 430 mm, then one properly placed 2x F sheet. Wait for READY, not just
STOP CONFIRMED or the start of the dump. Backward hardware, corrected
powered decode, 141 mm optics, physical 450 mm map margin, runtime stack/
timing and normal LINE regression remain pending.

## B1c — supplied 0cd2c03 hardware and peak-path candidate

This section supersedes the preceding pending statuses for the old image,
not its revision-scoped evidence. Main supplied all hardware facts below;
this worker read only the four approved extracted CSVs, not private logs.

### 0cd2c03: six powered passes at 430 mm / 100 mm/s

Contrast: FLOOR 152..170, WHITE 151..169, BLACK 1576..1716. All six passes
reported STOP CONFIRMED / DISTANCE at observed encoder travel 433 mm;
wheel endpoints were within 437/429 mm. BC_STATS loss/fault counters were
all zero. READY/ignored notices worked; complete dumps took 31–33 s.
These are encoder observations, not an independent physical ruler test.

| Pass | Supplied result |
| --- | --- |
| Lifted forward / backward | NO_FRAME both; air 1170..1290, expected |
| Large F forward | F / OK both WIDTH methods, separation 2699/2719 |
| Large F backward | F / OK both, dirs 2; drove over own USB cable |
| Course-size F forward / backward | NO_FRAME both WIDTH methods; 13 runs |

The cable-contaminated backward trace is useful stress data, not a clean
backward floor acceptance. Repeat with the cable clear. All 15 small-print
bars are present as local peaks; the nine narrow peaks are only about
200–234 raw with prominence 29–70, while six wide bars are about 1140–1323.
The earlier linear half-width response prediction was wrong. Lowering the
WIDTH threshold alone does not solve its bar-width distortion.

**Recorded mechanical constraint:** the sensor is at its lowest possible
5.5 mm height and already touches the floor on the 10 mm hump. The proposed
lower-height test is ruled out. B1c must use this height; no mount, GPIO,
subsystem 4, line steering or vehicle-routing change is included.

### Approved real fixtures and analysis

Copies under `tests/fixtures/` are byte-for-byte approved extractions;
old BC_RUN rows are not used for classification. SHA256 per file:

- `barcode_430_large_fwd.csv`
  `0e8c9c04d7c17d618610db8164cb8866ee9e56fe9fda61d5e93b3c0086acccd5`
- `barcode_430_large_back_cable.csv`
  `56a8eba819cc6229b6e4b17b0611b8bc6fc7ceab05b791cb7c2e32ad570df634`
- `barcode_430_small_fwd.csv`
  `4df870325f12efb9aa9a0550d703ced1bbe46775f085f3768ed3fcb54e76f9f6`
- `barcode_430_small_back.csv`
  `79f465c82c98adef42b58362fe6c9eadcde6305fde92d1e962bb7e6128cdcb5f`

Offline 1 mm mean-ADC analysis used source-stamped wheel interpolation at
12414 counts/m. Quiet SD used bins 5..25 and 410..430 relative to capture
start. Peak prominence used a +/-4 mm maximum and +/-6 mm minimum.

| Trace | Quiet SD | Narrow raw / prominence | Wide raw / prominence |
| --- | --- | --- | --- |
| Large forward | 1.6–2.2 | 401–1105 / 238–944 | 1376–1793 / 1210–1551 |
| Large back/cable | 1.3–2.4 | 396–904 / 229–743 | 1354–1805 / 466–1586 |
| Small forward | 1.6–2.1 | 200–234 / 35–70 | 1203–1295 / 1036–1127 |
| Small backward | 1.7–1.8 | 199–213 / 29–52 | 1138–1323 / 969–1149 |

Across 792 quiet bins (12 mm barcode guard), none of 86 local maxima reached
20 prominence; largest was 10.9. This does not establish a field error rate.
Naive local maxima give 16 peaks on each large trace. The extra pair is a
single wide lobe: intervening valleys ~1432/915, unlike real white valleys
~156–179. White-return merging preserves the small neighbours 5–6 mm apart.
The implemented noise estimate is conservative max(SD, 1.4826 MAD), so it
can differ from the offline SD table. Merged-lobe prominence also includes
white flanks to avoid rejecting flat wide tops.

### B1c candidate software evidence — not a powered PASS

WIDTH is unchanged except standard-table accessors. PEAK runs sequentially
after checked stop/freeze, on independent TIME and DISTANCE axes. Both
framing stars anchor amplitude bands; physical space widths come from
centre gaps and a bounded ordinary Code 39 model, not shrunken analog bar
widths. Whole-capture conflicts within/across paths produce AMBIGUOUS.
No selected payload controls a vehicle action. See the barcode contract for
baseline gates, geometric bounds, capacity and one-bin quiet uncertainty.

| Software check | Result |
| --- | --- |
| Full strict host suite | PASS: 175/175 |
| Four real captures, both axes, raw/packed, reversed | PASS: F / NONE |
| Large WIDTH unchanged / small WIDTH unchanged | PASS: F / NO_FRAME |
| Whole quiet prefixes/suffixes on both paths | PASS: no frame |
| All 43 symbols, two sizes and both directions | PASS: synthetic |
| Noise, split/flat lobes, drift, 25/200 mm/s, speed change | PASS: synthetic |
| 18 mm line, paper/floor, air/all-black | PASS: synthetic rejection |
| Missing/extra/weak/merged bars, bad geometry/stars/quiet | PASS: rejection |
| 16 symbols / 90 bars, overlength/capacity, clock/count wrap | PASS |
| ADC loss, absent/stale/reversed motion, independent TIME | PASS |
| Competing real-shaped payloads / cross-path conflict | PASS: AMBIGUOUS |
| Clang ASan/UBSan | PASS: both new units and 32 owner paths |
| Warning-free firmware / SDK profiles | PASS: nine |
| Strict ARM compile invocations | PASS: 72 |
| Primary barcode storage / whole image static RAM | 38600 / 158676 bytes |
| Barcode object static sections before GC/padding | 39474 bytes |
| Owner / PEAK driver compiler frames | 328 / 64 bytes |
| PEAK step / star / model compiler frames | 176 / 80 / 96 bytes |
| WIDTH step / character compiler frames | 304 / 144 bytes |

Compiler frames are not runtime stack/IRQ proof. A noinline PEAK driver
keeps new locals out of the WIDTH call chain on the 1 KiB TEST task. Primary
storage has an assert <=40000 bytes; small owner/capture counters are extra.
No kernel stack or template setting was changed. Normal images exclude
all barcode workspace symbols. Full raw dump and its pacing remain intact.

Retest using the exact B1c image: verify B1c WIDTH+PEAK startup, perform GP20
ranges, then lifted forward/backward. Clear the USB cable before floor
work. Use one large F then one course-size F sheet, both directions. Save
all WIDTH, PEAK, SELECTED, PEAK_META/COST, stats and complete raw output;
wait for READY. Obtain the requested real line-only and paper-only passes.
B1c powered decode, real line/paper negatives, clean backward floor,
runtime timing/high-water, physical map-margin and normal LINE checks are
**Not run**. No worker push or flash was made.
