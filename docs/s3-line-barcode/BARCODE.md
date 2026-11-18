# Analog barcode B1c: exclusive driven TEST

## Scope and evidence

B1c keeps B1b WIDTH decoding and adds a separate post-stop PEAK path.
It runs an operator-requested straight pass over **one barcode sheet**.
GP21 starts 430 mm forward at 100 mm/s; after confirmed stop and a complete
dump, a new press starts 430 mm backward. Swap the 2x F and 141 mm F sheets
between runs, while stopped. Do not place both sheets in one run. The
requested distance is clamped at compile time to **at most 450 mm**.
The operator shortened the map: 430 mm leaves 20 mm margin for coast
(about 3 mm observed at 100 mm/s). The old 600/620 settings are superseded.
The first floor capture also reached unrelated dark floor after ~450 mm.

This is not barcode-controlled navigation. A decoded command is data only;
no barcode changes the Vehicle Controller route. Normal LINE behaviour is
unchanged. On `0cd2c03`, six lifted/large/small passes stopped at observed
433 mm encoder travel, no loss/fault; UI notices worked, dump 31–33 s.
Large F decoded with WIDTH both ways; backward crossed the USB cable and
is a stress case, not a clean floor acceptance. Small F failed WIDTH both
ways (13 runs). B1c PEAK passes host replay; powered B1c is **Not run**.
The earlier 603/602 mm observations belong to the old 600 mm setting.

**Mount constraint, recorded by the operator:** 5.5 mm is the lowest
possible sensor height. The sensor already touches the floor when the
robot climbs a 10 mm hump. Do not lower it. The earlier proposed height
comparison is ruled out; B1c must work at this mount. Floor contact and
hump clearance remain mechanical risks, not decoder calibration steps.

On B1 `70caef0`, the operator proved static contrast: white paper 151..170,
floor 155..172, black bar 1334..1660. Black is HIGH; floor looks like white.
The first contrast round was misplaced (the supposed white read about
1500); use the corrected ranges, not that round. Nine hand-push captures
failed the procedure: bars entered only in the last 0.2..0.5 s of the
4.1 s window, or did not enter it. None decoded. Encoder travel was only
0..70 mm because back-driven wheels slip. This is **not a decoder verdict**.
See [S3 TEST_LOG](../../testing/subsystem3/TEST_LOG.md) for evidence scope.

The [requirements summary](../project-requirements/README.md) points to local
course originals; do not publish their pages. `origin/ziven-barcode-algo`
is reference only. The pure decoder uses the standard Code 39 table.

## Input and ownership

| Item | Contract |
| --- | --- |
| GP28 / ADC2 | Existing 1 kHz, eight conversions; VBAT jumper stays open |
| `raw_bc` | Third ADC value, never a third line-control input |
| GP7 | GY-511 INT1 wire; barcode code never configures it |
| GP20 | Record FLOOR / WHITE_PAPER / BLACK_BAR; redo when idle |
| GP21 | Fresh press starts a pass; while moving, fresh press stops |
| Line task | Priority 11; O(1) capture copy after mutex release, outside ISR |
| TEST runner | Priority 20; bounded drive observation every 10 ms |
| Decoder / dump | Incremental TEST work only after confirmed stop/freeze |
| ADC ownership | No new setup or second `drv_line_take()` consumer |

At about 5.5 mm or more above the floor, optical blur can cover several
narrow bars. Slower travel does not remove that blur. Keep the fixed
left-front sensor clear of ramps. Equal wheel requests are not heading
control; measure drift and keep the bars across the sensor's travel path.
No sensor-offset/turn correction or normal steering change is implemented.

## Build and settings

```sh
make -C build_make SMP=0 WIFI=none NET=none TEST=1 BARCODE_CAPTURE=1 -j4
```

Select `mtk3pico_smp0_usb_cdc_test_barcode_capture.uf2` and verify the
**B1c WIDTH+PEAK TEST ONLY** banner. Use USB console and CRASH_REPORT=0 on the
connected robot. BENCH, IMU_STREAM, SMP, Wi-Fi, network and MQTT combinations
are rejected. Ordinary subsystem tests and the destructive reset test are
skipped. This profile alone has the GP21 barcode-pass client.

Settings live in
[`barcode_settings.h`](../../app_program/subsystem3/barcode_settings.h):

| Setting | Default / bound |
| --- | --- |
| `BARCODE_PASS_DISTANCE_MM` | 430 mm; actual target clamps to <=450 mm |
| Minimum requested distance | 50 mm; smaller/negative settings fail build |
| `BARCODE_PASS_SPEED_MM_S` | 100 mm/s; permitted 25..200 mm/s |
| Encoder geometry | 12414 counts/m |
| Motion deadline | Ceiling(distance / speed in ms) + 3000; 7300 ms default |
| ADC retention | Every 2 raw samples by default; actual u16 time deltas |
| Motion retention | Every 20 ms by default, using real source timestamps |

Retention intervals are derived from deadline plus 2 s storage margin.
At 430 mm / 25 mm/s, deadline is 20.2 s, ADC retention 4 ms, motion 50 ms;
capacity still covers the longer pass. This is decimation, not averaging
or min/max compression. Raw 1 kHz gaps are checked even on skipped records.
A skipped brief signal is not reconstructed or guessed. Decoder motion-gap
allowance is retention period +30 ms (50 ms by default): two allowed busy
polls plus source/poll phase can defer a retained observation that far.
This does not relax the independent 30 ms live-board age bound.

## Driven procedure

**VIN is required. Lift both wheels for the first GP21 check.** Clear the
mechanism, keep access to VIN removal, and do not treat zero duty as a
physical brake. Hardware stop distance, drift and stack/timing checks are
not yet accepted. A target clamp is not a guarantee against wheel slip or
coast; verify actual travel before using the available floor space.

1. Flash the exact profile, start terminal recording, and wait for initial
   `STOP CONFIRMED`. A held GP21 at boot does not start motion.
2. With the robot stationary, place FLOOR under the sensor and press GP20.
   Record the **next** one-second range. Repeat for WHITE_PAPER and BLACK_BAR
   at the prompts. Save minimum, maximum and sample count. GP21 is disabled
   until all three ranges complete. Keep USB and its terminal connected
   during ranges: these results are not queued for later replay. Ranges do
   not automatically certify narrow-bar visibility.
3. Lifted check: with VIN present and wheels free, press/release GP21.
   Check equal positive wheel requests, the 430 mm target and confirmed
   zero duty/low measured speed. Fetch the dump as in step 6. Press/release
   again for
   negative requests. Test a GP21 early stop. Do not call this a barcode
   or floor-distance PASS. Reboot before floor work to restore FORWARD.
4. Floor check: place **one** F sheet, bars across travel; use the 2x copy
   first. Align the left-front sensor, not an assumed axle/body offset,
   before the white margin. The entire printed frame and white quiet
   zones must fit within the 430 mm run (hard target limit 450 mm). A 2x F
   needs at least 58 mm white/floor quiet margin on EACH end. Position it
   accordingly, for example about 70 mm leading margin; do not rely on
   coasting to complete a quiet zone. Keep extra physical space for coast.
5. Press/release GP21. Capture arms at drive start. The task requests equal
   100/100 mm/s through motion APIs; there is no automatic line correction.
   Another fresh GP21 press requests stop. GP20 has no effect while moving.
6. At distance/deadline/fault, the task repeatedly requests stop. Capture
   stays active until stop is confirmed. Then it freezes and keeps the
   data in RAM. USB can be unplugged before the pass, but **keep VIN on**.
   After the robot stops, reconnect USB and open the capture terminal.
   The task waits for CDC DTR for one continuous second before it decodes
   and prints the existing report/dump. No host means an indefinite silent
   wait, no new pass and no READY. GP21 presses cannot queue another pass.
   See [USB capture wait](USB_CAPTURE_WAIT.md) for terminal requirements.
7. Save `[BC_DUMP] END` and verify all row counts. Full dump stays enabled:
   it can take tens of seconds (about 40 s on the first 600 mm lifted pass).
   Before output, `[BC] DUMP n rows ~X s: GP21 ignored until READY` gives an
   estimate, not a timeout. A new press during the dump prints one ignored
   notice per dump; it never queues a start. Wait for
   `[BC] READY: GP21 = BACKWARD` (then FORWARD alternately), release GP21
   and press again. Fault inhibition prints NOT READY instead.
   A press held through a dump must be released before another can start.
8. After both directions, swap to the 141 mm F sheet while stopped and
   repeat. Keep one sheet per run. After an early stop, reposition safely
   before another full-distance pass; do not assume the car is at an end.

Motion/sensor/command faults, observer wake faults, wrong encoder direction,
wheel travel difference >100 mm, capture capacity and timeout stop the job
and inhibit further starts until reboot. A stop-API failure also inhibits
restart, even if the fallback succeeds. Remove VIN if wheels keep moving.
No dump or rearm is permitted while stop remains unconfirmed.

Stop confirmation requires successful `motion_stop()` (or `motion_estop()`
fallback), then two distinct fresh post-request motion snapshots with zero
duty and both measured speeds within +/-10 mm/s. These are encoder-based
observations, not independent physical brake verification. Retries continue
if snapshots/API confirmation are unavailable. Final wheel travel includes
observed coast; above 450 mm sets OVERRUN and inhibits another pass.

## Snapshot health and fault diagnostics

Line publishes after each queued 1 ms ADC sample; motion publishes after
its 10 ms control tick. A sequence-guarded read can return E_TMOUT if a
higher-priority writer interrupts the copy. One busy read is not a sensor
fault. The TEST owner retains the last successful coherent board and
retries on its next 10 ms tick. It never uses a partially copied board.

For BOTH boards, three consecutive failed reads or source age >30000 us
stops the pass. A successful read resets that board's failure count. An
error other than E_TMOUT, invalid line data, raw ADC above 4095, motor fault
or false watchdog flag stops immediately. Without any successful board,
start is refused. Cached source timestamps are not advanced or restamped.
This replaces the former one-read / 5000 us line check only in this TEST.
Stop confirmation still needs distinct fresh post-request zero snapshots.

The watchdog is armed before speed commands: `motion_set_speed()` refuses
an unarmed controller, and does not clear the flag. No artificial arming
grace is added. A coherent false watchdog flag remains a real fault.

After the stop API request, every FAULT transition and every START REFUSED
prints `[BC_FAULT]`. The row has mask, line read rc/age/valid/raw/failure
count, motor read rc/age/failure count/faults/watchdog flag, actual sleep
wake rc and queued wakes. Ages are source ages; UINT32_MAX means no good
board yet. A mask can contain multiple causes. API return values use signed
decimal; zero is E_OK. Preserve the full row for any failed retest.

| Mask bit (hex) | Cause |
| --- | --- |
| 0001 / 0002 | Sleep/cyclic error / queued owner wakes |
| 0004 / 0008 | Motor read failure / motor source too old |
| 0010 / 0020 | Line read failure / line source too old |
| 0040 / 0080 | Line not valid / ADC above 4095 |
| 0100 / 0200 | Motor fault flags / watchdog not armed |
| 0400 / 0800 | Acquisition missed/dropped delta / capture ADC gap |
| 1000 / 2000 | Motion buffer overflow / moving at start |
| 4000 / 8000 | Invalid settings / restart already inhibited |

The corrected image prints `health_age_limit_us 30000 read_fail_limit 3;
BC_FAULT masks enabled` at startup. Verify this before the lifted retest.

The first lifted trial of `afcdea2` passed contrast but stopped at FAULT
before the wheels turned. That image did not print a cause mask; its root
cause is not established. Busy-copy rejection is a candidate, not a proven
diagnosis. Lifted forward later passed on 45040c7, and both directions
passed on 0cd2c03; this does not prove B1c runtime timing or stack use.

## Capture, RAM and diagnostics

| Storage | Current B1c bytes / limit |
| --- | --- |
| ADC records | 6144 x 4 = 24576; about 12.288 s at default 2 ms |
| Motion records | 512 x 12 = 6144; about 10.24 s at default 20 ms |
| Capture arrays | 30720 = 30 KiB, exclusive profile only |
| Decoder workspace + report | 1760 + 64 = 1824 ARM bytes; assert <=2048 |
| PEAK workspace / report | 5876 / 112 ARM bytes |
| Selected result / two parameter blocks | 28 / 40 ARM bytes |
| Primary arrays, workspaces, reports, parameters | 38600; assert <=40000 |
| Barcode object static sections before GC/padding | 39474, below 40 KiB |
| Whole image static RAM | data 152 + BSS 158524 = 158676 bytes |

These are compiler/linker results, not runtime free-heap/stack evidence.
Owner boards and small counters are extra; barcode storage stays below
40 KiB. The TEST initial task has only a 1 KiB stack. Its compiler frame is
328 bytes. WIDTH step/character frames remain 304/144. A separate noinline
PEAK driver uses 64; PEAK step/star/model use 176/80/96. Keeping PEAK-only
locals out of the WIDTH call chain avoids inflating its task stack. These
are not complete SDK/interrupt call-graph or measured high-water proofs.
No kernel stack setting was changed. Normal images allocate none of these
capture arrays/workspaces. Runtime stack/timing remains a hardware check.

The four-byte ADC record has u16 raw/flags (bits 0..11 ADC, bit 15 loss,
12..14 reserved zero) and a u16 microsecond delta from the prior retained
sample. First delta is zero; one u32 origin reconstructs wrap-safe time.
Delta overflow marks loss; no time is silently invented. Raw gaps >1.5 ms
mark loss even on decimated ticks. Capacity stops appends without overwrite
and requests a checked stop; freezing remains the drive owner's decision.

- `[BC_FRAME]`: one row per complete frame **per width method**, string,
  command/quality/direction, entry/exit timestamps and encoder position in
  micrometres relative to capture start. INT32_MIN position means absent.
  The same physical F normally appears once in TIME and once in DISTANCE.
  Frames can be in search order; sort wrap-safe (timestamp - origin_us)
  offsets for travel order.
- `[BC_RESULT]`: whole-capture summary. Differing frame strings make this
  summary ambiguous; individual frame reports remain separate. No vote or
  barcode-controlled driving is implemented.
- `[BC_STATS]` / `[BC_LINE]`: missed/dropped ADC deltas, raw gaps, full,
  capture drops, motion misses/overflow, observer misses, copy maximum,
  ISR and line cost. `[BC_META]` adds decoder step/total elapsed cost.
- `BC_ADC`, `BC_MOTOR`, `BC_RUN`: retained raw values/time deltas, coherent
  source-stamped counts and run widths in time/distance. Verify BEGIN/END
  counts; paced USB output does not repair transport loss.

`capture_dropped` counts rejected retention slots, not deliberately skipped
1 kHz samples. OVERRUN compares encoder counts before integer-mm rounding
and latches an observed crossing of the 450 mm limit, including stop retries.
Raw loss rejects both decode methods before frame emission. Missing,
gapped, non-monotonic or overflowing motion rejects DISTANCE; TIME remains
separate. USB/frame printing occurs only after stop and freeze. Measured
elapsed cost includes preemption; frame-output steps also include output.

### Dump speed proposal — not enabled

Keep the full dump for optical tests. A requested 1 ms task delay can round
to a kernel tick, which helps explain the observed long dump. Removing
per-row yields blindly is unsafe for evidence: the USB TX ring is 4096
bytes and drops output when full; USB priority 21 is below this TEST owner.
A future opt-in summary-only setting (full remains default), or bounded
batches with TX occupancy/drop checks and measured host draining, can be
faster. Neither change is enabled here; row-count verification stays required.

## Decoder rules and fixtures

Sources: [`barcode.h`](../../app_program/subsystem3/barcode.h),
[`barcode.c`](../../app_program/subsystem3/barcode.c),
[`barcode_drive.c`](../../app_program/subsystem3/barcode_drive.c).
The drive bounds are independent of the pure decoder and of subsystem 4.

The decoder learns min/max over the frozen profile and requires contrast
>=64 ADC counts. For the measured black-HIGH sensor, rising threshold is
min + max(32 counts, 10% of span); falling threshold is min + max(16 counts,
6% of span). The former 40%/60% band missed partial-contrast narrow bars.
The 32-count minimum rising margin exceeds the supplied stationary white
spread; hysteresis avoids repeated switching near one level. This is an
optical signal rule, not a requested-letter search or confidence score.

For the first real capture min 150/max 1989, WIDTH needs retained peak
>=334, with falling level 260.34. Actual 2x narrow peaks 600–900 exceed it.
The earlier linear half-width prediction of ~375–525 was disproved by the
0cd2c03 physical small-print tests: narrow peaks were only about 200–234,
wide bars full scale. Synthetic peak-350 PASS / peak-320 rejection cases
remain model regressions, not physical forecasts. B1c does not lower or
otherwise change WIDTH levels; it uses a separate PEAK interpretation.
At lower full contrast WIDTH still needs at least a 32-count rise.
Full-contrast polarity tests remain; partial-contrast tuning is for the
observed black-HIGH input. No missing bar is invented.

Ordinary Code 39 has 43 data symbols plus framing `*`, nine alternating
elements with exactly three wide, ranked as the three widest of nine.
Compute separate narrow bar/space means. Their half-difference estimates
edge bias: add it to measured bars and subtract it from measured spaces.
Require absolute bias <=80% of the common physical narrow estimate, then
require corrected wide/narrow ratio 1.6..3.4, class separation >=1.4 and
bounded within-class spread. This removes the observed bar shrink / space
growth without changing the selected mask to force a letter. Inter-character
gaps use the narrow-space reference; white quiet zones use ten physical
narrow widths on both ends. Up to 16 data symbols
and 192 alternating runs fit. No extended ASCII or checksum is implemented.

Only exactly one A/B/C/D maps to LEFT/RIGHT/STRAIGHT/UTURN. Other valid
strings, including F and Z, decode with NONE. Quality is REJECTED or DECODED;
separation is the corrected minimum-wide / maximum-narrow ratio, per mille,
not a
confidence probability. Rejection reasons
cover loss, contrast, widths/pattern, quiet zones, truncation, capacity,
ambiguity and motion. No noise repair guesses a letter. Each incremental
step processes <=32 records or one possible frame, then the TEST task yields.

| Fixture | Known value / expected result |
| --- | --- |
| Course samples | Standard `*A*` and `*Z*`, not the operator's F sheet |
| Operator sheet | Photo plus ruler confirms standard `*F*`; command NONE |
| 2x F, first floor test | About 285 mm; narrow 5.8 mm, wide 17.5 mm |
| Course-size F | About 141 mm; narrow 2.9 mm, wide 8.7 mm |
| `*` / F | `nwnnwnwnn` / `nnwnwwnnn` |
| Z / A | `nwwnwnnnn` / `wnnnnwnnw` |
| Track reference | 20 mm barcode-to-line gap, 18 mm line width |

Expected physical result: **F, DECODED, NO command**, independently for each
method. A differing or unreadable result is not a navigation instruction.
At 100 mm/s, narrow bars last about 29/58 ms, giving about 14/29 retained
samples. Most host fixtures are synthetic. The approved public-safe
[real trace](../../tests/fixtures/barcode_large_f_forward.csv)
is a real 3080-ADC/180-motor trace; its 15 old BC_RUN rows are not used to
classify the new result. Both width methods decode F/NONE from the whole
capture and its reversed replay, including the unrelated dark tail. Its
<=430 mm prefix also passes both methods; the frame spans about 73–348 mm.
Reversed replay is not a backward hardware PASS. Current 0cd2c03 evidence
and B1c limitations are recorded below; runtime stack, new powered PEAK,
line-only/paper-only physical negatives and normal LINE checks remain open.
[LINE](LINE.md) and [BUILD](../kernel/BUILD.md) retain normal rules.

## B1c PEAK contract

[`barcode_peak.h`](../../app_program/subsystem3/barcode_peak.h) and
[`barcode_peak.c`](../../app_program/subsystem3/barcode_peak.c) implement
an additional black-HIGH interpretation. They do not change WIDTH, LINE,
stop logic, travel bounds, pin assignments or subsystem 4. No PEAK work
runs during motion or in the ADC ISR. Frozen inputs remain immutable.
WIDTH runs/reports remain available for the unchanged full raw dump.

### Baseline, lobes and resolution

Each native axis forms mean-ADC bins. DISTANCE interpolates valid bracketed
source-stamped encoder points into 1 mm bins. TIME uses timestamps alone:
10 ms bins at 100 mm/s, scaled by configured speed, never encoder-derived
centres relabelled TIME. Missing/gapped/reversed motion rejects DISTANCE
without changing TIME. Raw loss or invalid ADC rejects both.

Bins 5..25 must be a quiet white lead-in consistent with the GP20 floor/
paper union, within 32 counts. Configured white range must span <=128.
Dark/air cannot become the white reference. Noise estimate is the larger
of sample SD and 1.4826 times median absolute deviation on those same bins.
A fixed 16-step Newton square root avoids libc errno RAM. Noise SD >10
rejects the path. Gate is max(6 times noise SD, 20 ADC counts); it is never
reduced to recover a letter.

White-return band is baseline + gate. A connected above-band lobe is one
bar candidate even when its broad top contains several local maxima.
Use its excess-signal centroid, highest amplitude and prominence against
local +/-6-bin minimum, also including its white flanks. No global minimum
centre distance erases small neighbours 5–6 mm apart. Low-prominence lobes
are not silently deleted to fix a frame: a frame containing one rejects.
There are at most 2048 bins and 96 lobes per axis; overflow rejects, with
no overwrite. Capacity includes the 90 bars in the maximum 16-symbol frame.

### Framing and geometric spaces

Try both print orientations. Candidate start AND stop stars anchor the
amplitude bands from their known narrow/wide bars, not a global two-means
split or the requested letter. Their shared gap must exceed both the gate
and 12 noise SD; every later bar needs amplitude margin >=max(6 SD,
half the gate) and enough prominence. No missing bar, gap, polarity or
payload is invented.

For each character, combine its five amplitude-classified bar bits with
four geometric space bits. Never rank ADC counts together with millimetres.
For adjacent centres, space = centre gap minus half each PHYSICAL bar
width; observed threshold widths are deliberately not used. Fit narrow
and wide scale against the unchanged 43-symbol Code 39 table. Exactly
three of nine elements must be wide. Multiple admissible characters reject.

Unconstrained width ratio must be 1.5..3.6; constrained fit uses 1.6..3.4,
allowing only the measured-centre uncertainty, not arbitrary distortion.
Each space residual must be <=0.35 narrow, spread 0.65..1.5 of its class,
and minimum-wide/maximum-narrow separation >=1.4. Report the measured
separation, not the acceptance threshold. Local scale changes between
characters must remain within 0.65..1.5; this allows TIME speed variation
without using motion to repair TIME. Inter-character gap stays 0.5..1.8
narrow. Quiet zones remain ten fitted narrow widths, with at most ONE
native-bin boundary uncertainty (1 mm, or 10 ms at 100 mm/s). Nine-narrow
quiet-zone synthetic input rejects. No checksum or extended ASCII is added.

Each step handles <=32 ADC/motion records or bins, one reference-star
candidate, or one character's finite alphabet. The task yields after each
step. Workspaces and parameter blocks are static/caller-owned, not large
automatic task arrays. Frame coordinates are capture-relative centres,
not threshold-edge positions; timestamps and signed encoder positions
remain source-bracketed. INT32_MIN position means absent.

### Reports, choice and limits

- Existing `[BC_FRAME]`, TIME/DISTANCE summaries and BC_RUN stay WIDTH.
- `[BC_PEAK_FRAME]`: method, payload, print direction, centre timestamps
  and encoder positions. F still maps to NONE.
- `[BC_PEAK_META]`: lobe count, baseline, noise SD, gate, minimum accepted
  prominence, amplitude-band margin, maximum geometric residual.
  ADC-valued fields use milli-counts; geometry is per mille of fitted narrow.
- `[BC_RESULT] PEAK_TIME/PEAK_DISTANCE` and SELECTED report all outcomes.
  `[BC_SELECT]` names the chosen source or CONFLICT/NONE.
- `[BC_PEAK_COST]`: elapsed step maximum and total. New analysis can add
  stopped time before the dump estimate; GP21 remains ignored until READY.

Quality selection first requires full framing, noise and geometry checks.
Keep WIDTH as the preferred source when accepted results agree. A WIDTH
rejection can permit a valid PEAK result. Any conflicting accepted payload,
or ambiguity within ANY path, makes SELECTED AMBIGUOUS. A larger amplitude
or separation never overrides conflict. Missing distance stays missing;
agreement is a consistency check on one sensor, not independent proof.
No selected value can execute a vehicle command.

### B1c evidence and remaining physical tests

Only these Main-approved extracted ADC/MOTOR/RUN CSVs were copied; private
console logs were not read. Old BC_RUN output is never decoder input:

- [Large forward](../../tests/fixtures/barcode_430_large_fwd.csv)
- [Large back/cable](../../tests/fixtures/barcode_430_large_back_cable.csv)
- [Small forward](../../tests/fixtures/barcode_430_small_fwd.csv)
- [Small backward](../../tests/fixtures/barcode_430_small_back.csv)

Host replay gives F/NONE on both PEAK axes, raw and packed, forward and
reversed: 16 combinations. All have exactly 15 lobes; merging removes the
extra broad-bar maxima on both large traces. WIDTH stays F on large and
NO_FRAME on small. Whole quiet prefixes/suffixes, excluding a 120 ms guard
around the outer centres, give no frame on either path; valid baselines
are required, not a short-input rejection shortcut. Synthetic tests cover
all 43 symbols at 2.9/5.8 mm, both directions, flat/split tops, noise,
25/200 mm/s, baseline drift and nonuniform speed. Limits, missing/extra/
merged/weak bars, malformed geometry/stars/quiet zones, bad ADC/motion,
clock/count wrap, competing payloads and cross-path conflicts reject.
Moving a bar by about 2n can make a different LEGAL symbol (F becomes 6 in
one test); that must decode 6, not be repaired to F.

Initial offline quiet SD was about 1.3–2.4 counts in 1 mm means. In 792
quiet bins, all 86 local maxima had prominence <20 (largest 10.9). Small
bar prominences were about 29–70; this is useful but limited margin, not a
field false-positive probability. Raw ADC SD is a different statistic.
A plain floor/paper edge or impulse need not have Gaussian noise.

A synthetic 18 mm flat/split-top line forms one lobe and gives no frame.
A real line is optically indistinguishable from a wide bar by amplitude
alone. Grazing, weaving, broken lines or a cable can form several lobes;
full framing/geometry/quiet checks are essential. A nearby line can defeat
the required quiet zone and must not be ignored. Physical line-only and
paper-only negatives are **Not run** and requested from the operator with
the B1c image. The cable trace is retained as a stress case, not a substitute
for a clean backward floor pass.

Software checks: 175/175 host, Clang ASan/UBSan on both new units and 32
owner paths, nine warning-free firmware/SDK profiles and 72 strict ARM
compiles. B1c powered decode, runtime stack/timing, clean backward floor,
physical line/paper negatives and normal LINE regression remain open.
The fixed 5.5 mm mount and its hump contact cannot be solved by this decoder.
