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

## B1b software validation — exclusive driven profile

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

## B1b powered procedure — Not run

Use the [current contract](../../docs/s3-line-barcode/BARCODE.md) for the
full button, settings, loss and output rules. Do not use B1 hand-push
button instructions on this image.

```sh
make -C build_make SMP=0 WIFI=none NET=none TEST=1 BARCODE_CAPTURE=1 -j4
```

| ID | Physical check | Status |
| --- | --- | --- |
| S3-B1b-01 | VIN, lifted wheels: both directions and GP21 stop | Not run |
| S3-B1b-02 | One 2x F sheet, floor forward/backward 600 mm passes | Not run |
| S3-B1b-03 | Swap to one 141 mm F sheet, both directions | Not run |
| S3-B1b-04 | Stop/coast distance and fault-inhibited restart | Not run |
| S3-B1b-05 | Cost, loss, USB integrity and runtime stack | Not run |

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
   two-sheet layout. Requested target must not exceed 620 mm; default is
   600 mm, about 6 s at 100 mm/s before acceleration/stop allowance.
4. After fresh contrast ranges, GP21 starts capture and the forward pass.
   Freeze/decode/dump must wait for confirmed stop. Save TIME and DISTANCE,
   each `[BC_FRAME]` entry/exit position, final left/right travel, all raw
   ADC/motion/run rows and counters. Expected: F, DECODED, command NONE.
5. Save `[BC_DUMP] END` and verify row counts before pressing GP21 again for
   backward. No automatic restart or queued press during a dump is allowed.
   If stopped early, reposition before another full-distance run. Swap to
   the 141 mm F sheet only while idle and repeat; one sheet per run.
6. Any fault/deadline/capacity stops and inhibits restart. Stop API success
   alone is not enough: retries continue until fresh zero-duty/low-speed
   snapshots confirm it. No dump/rearm while pending. If wheels still move,
   remove VIN. Record coast/stop distance; >620 mm reports OVERRUN. Encoder
   slip/coast mean the target clamp is not an independent physical ruler.
7. Record new copy/decoder/line/ISR cost, missed/dropped/gap/overflow and
   observer counters, USB integrity and stack high-water. Do not carry B1's
   1010/113 us observations into this powered image as a PASS.
