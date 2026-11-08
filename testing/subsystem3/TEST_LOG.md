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

Validation base: staging `b569adc`, on `mtk3-line`. This section describes
this B1 source change, not a physical run of the earlier line image.

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

## B1 physical procedure — Not run

Use the [B1 contract](../../docs/s3-line-barcode/BARCODE.md) for the full
settings, output fields, quiet-zone limits and no-command rule.

```sh
make -C build_make SMP=0 WIFI=none NET=none TEST=1 BARCODE_CAPTURE=1 -j4
```

| ID | Check | Status |
| --- | --- | --- |
| S3-B1-01 | Stationary contrast ranges | Not run |
| S3-B1-02 | 2x F, straight hand push | Not run |
| S3-B1-03 | Reverse order and course-size F | Not run |
| S3-B1-04 | Loss, timestamps, dump counts and timing | Not run |
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
of B1. Contrast, physical decoding, stack use and timing remain Not run.
