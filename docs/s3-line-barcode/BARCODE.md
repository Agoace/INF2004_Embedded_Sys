# Analog barcode B1b: exclusive driven TEST

## Scope and evidence

B1b runs an operator-requested straight pass over **one barcode sheet**.
GP21 starts 600 mm forward at 100 mm/s; after confirmed stop and a complete
dump, a new press starts 600 mm backward. Swap the 2x F and 141 mm F sheets
between runs, while stopped. Do not place both sheets in one run. The
requested distance is clamped at compile time to **at most 620 mm**.

This is not barcode-controlled navigation. A decoded command is data only;
no barcode changes the Vehicle Controller route. Normal LINE behaviour is
unchanged. B1b powered hardware acceptance is **Not run**.

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
**B1b DRIVEN TEST ONLY** banner. Use USB console and CRASH_REPORT=0 on the
connected robot. BENCH, IMU_STREAM, SMP, Wi-Fi, network and MQTT combinations
are rejected. Ordinary subsystem tests and the destructive reset test are
skipped. This profile alone has the GP21 barcode-pass client.

Settings live in
[`barcode_settings.h`](../../app_program/subsystem3/barcode_settings.h):

| Setting | Default / bound |
| --- | --- |
| `BARCODE_PASS_DISTANCE_MM` | 600 mm; actual target clamps to <=620 mm |
| Minimum requested distance | 50 mm; smaller/negative settings fail build |
| `BARCODE_PASS_SPEED_MM_S` | 100 mm/s; permitted 25..200 mm/s |
| Encoder geometry | 12414 counts/m |
| Motion deadline | Ceiling(distance / speed in ms) + 3000 ms; 9000 default |
| ADC retention | Every 2 raw samples by default; actual u16 time deltas |
| Motion retention | Every 30 ms by default, using real source timestamps |

Retention intervals are derived from deadline plus 2 s storage margin.
At 600 mm / 25 mm/s, the deadline is 27 s, ADC retention 5 ms, motion 60 ms;
capacity still covers the longer pass. This is decimation, not averaging
or min/max compression. Raw 1 kHz gaps are checked even on skipped records.
A skipped brief signal is not reconstructed or guessed.

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
   until all three ranges complete. Ranges do not automatically certify
   narrow-bar visibility.
3. Lifted check: with VIN present and wheels free, press/release GP21.
   Check equal positive wheel requests, the 600 mm bound and confirmed
   zero duty/low measured speed. Save the dump. Press/release again for
   negative requests. Test a GP21 early stop. Do not call this a barcode
   or floor-distance PASS. Reboot before floor work to restore FORWARD.
4. Floor check: place **one** F sheet, bars across travel; use the 2x copy
   first. Align the left-front sensor, not an assumed axle/body offset,
   before the white margin. The entire printed frame and white quiet
   zones must fit within the one <=620 mm run. Keep extra space for coast.
5. Press/release GP21. Capture arms at drive start. The task requests equal
   100/100 mm/s through motion APIs; there is no automatic line correction.
   Another fresh GP21 press requests stop. GP20 has no effect while moving.
6. At distance/deadline/fault, the task repeatedly requests stop. Capture
   stays active until stop is confirmed. Only then does it freeze, decode
   TIME and DISTANCE independently, and print the complete dump.
7. Save `[BC_DUMP] END` and verify all row counts. No pass restarts by itself.
   A new GP21 press after the dump starts BACKWARD, then FORWARD alternately.
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
observed coast; above 620 mm sets OVERRUN and inhibits another pass.

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
diagnosis. This health-policy fix needs a new lifted-wheels retest.

## Capture, RAM and diagnostics

| Storage | Current B1b bytes / limit |
| --- | --- |
| ADC records | 6144 x 4 = 24576; about 12.288 s at default 2 ms |
| Motion records | 512 x 12 = 6144; about 15.36 s at default 30 ms |
| Capture arrays | 30720 = 30 KiB, exclusive profile only |
| Decoder workspace + report | 1760 + 64 = 1824 ARM bytes; assert <=2048 |
| Arrays + decoder/report | 32544 bytes; owner boards/counters are extra |
| Whole image static RAM | data 152 + BSS 152468 = 152620 bytes |

RAM is from this B1b software-validation image descended from `70caef0`,
not runtime free heap or stack evidence. Arrays, decoder and owner state
remain below about 40 KiB. Owner boards/state are static because the TEST
initial task has only a 1 KiB stack. Compiler function frames are 304 bytes
for the owner, 296 for decoder step and 112 for character classification;
these are not a complete call-graph or measured high-water proof. No
kernel stack setting was changed. Normal images allocate none of these
capture arrays/workspaces. Runtime stack use remains a hardware check.

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
and latches an observed crossing of the 620 mm limit, including stop retries.
Raw loss rejects both decode methods before frame emission. Missing,
gapped, non-monotonic or overflowing motion rejects DISTANCE; TIME remains
separate. USB/frame printing occurs only after stop and freeze. Measured
elapsed cost includes preemption; frame-output steps also include output.

## Decoder rules and fixtures

Sources: [`barcode.h`](../../app_program/subsystem3/barcode.h),
[`barcode.c`](../../app_program/subsystem3/barcode.c),
[`barcode_drive.c`](../../app_program/subsystem3/barcode_drive.c).
The drive bounds are independent of the pure decoder and of subsystem 4.

The decoder learns min/max over the frozen profile, requires default
contrast >=64 ADC counts, and uses 40%/60% hysteresis. Both polarities and
read directions are tested. This handles the observed black-HIGH polarity
without a fixed sensor threshold. Optical blur and clipping can still fail.

Ordinary Code 39 has 43 data symbols plus framing `*`, nine alternating
elements with exactly three wide. Require wide/narrow mean ratio 1.6..3.4,
class separation >=1.4, bounded class spread, narrow character gaps and
white quiet zones >=10 narrow widths on both ends. Up to 16 data symbols
and 192 alternating runs fit. No extended ASCII or checksum is implemented.

Only exactly one A/B/C/D maps to LEFT/RIGHT/STRAIGHT/UTURN. Other valid
strings, including F and Z, decode with NONE. Quality is REJECTED or DECODED;
separation is the minimum-wide / maximum-narrow ratio in per mille, not a
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
samples. Host fixtures are ideal widths, not sensor traces. Quiet margins,
physical drift, footprint, stop distance and powered decoding remain to be
measured. [LINE](LINE.md) and [BUILD](../kernel/BUILD.md) retain normal rules.
