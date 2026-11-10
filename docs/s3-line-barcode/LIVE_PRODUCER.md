# B2b SHADOW producer — step 4b

## Scope

ROBOT LINE now feeds every acquired GP28 sample to ONE static
`barcode_live_producer_t`, which contains one 988 B window context. No heap,
second acquisition consumer, ISR work or vehicle/action call is added.
TEST/capture and BENCH feed functions are no-ops; their framing is unchanged.

Files: `barcode_live_producer.{h,c}` hold pure policy and tests;
`barcode_live_source.{h,c}` own the coherent boards, task-context feed and
USB/DUMP formatter. `line_service.c` copies the calibration report and missed/
dropped counters under its existing mutex. ALL decoder, control/motor reads
and diagnostic publication occur AFTER unlocking. No sample-path printing.

Every record is SHADOW with valid=0, including mapped A--D and mode2 input.
OFFSET_VERIFIED is never set. The 150 mm placeholder cannot enable an action.
The public ABI stays [40 B record / 20 B control](LIVE_SOURCE_CONTRACT.md).
Sub_Worker5 owns the single stopped USB/DUMP hook next to `[JUNCTION]`, and
vehicle admission, terminal ACKs and boot printing of the SAME mapping table.

## Threshold evidence

Keep the [step-3 leading-white policy](LIVE_WINDOW_READER.md): qualified paper
white is 120--220, mandatory leading extent is ten fitted narrows, and its
mean +150/+75 gives thresholds, with rise capped at350. Freeze at FIRST BAR;
never adapt to mat black or later ink. The current-boot spin report is the
reference, not a replacement paper-white scale. Learned or fallback report
must be usable; no new spin-white evidence is invented for NO_WHITE fallback.

Each frame logs spin W, leading W and signed difference. A spin W of zero
means no learned spin white; the numerical difference is then not a measured
change of white. Spin-derived thresholds / 320,240 fallback remain boot
reference evidence. The window needs its own absolute leading white proof.

## Control and original time evidence

- First coherent control read is a baseline, NOT fresh permission.
- Need a changed coherent publication tag. Its earliest possible publication
  time is the previous successful read, not the current read time.
- After more than50 ms without a successful observation, establish a new
  baseline and require NEW progress. A scheduling pause cannot certify old
  control as fresh. Eligible control fails closed above50 ms.
- Mode0 disables reading, even when it shares a terminal ACK. Modes1/2 can
  collect SHADOW observations; neither enables a live record in this commit.
- Bind epoch/revision/cut at FIRST BAR; first_ms must be strictly after cut.
  Changed epoch/revision/cut or calibration generation abandons the window.
  This also discards leading white retained across a missed SEARCH cycle.
- Propagate acquisition missed/dropped changes. A 1500 us ADC gap rejects a
  silently missing1 ms sample. Fault, clipping, stale motor >30 ms or an
  unavailable motor copy abandons the candidate, without repair.
- Motor observations come only from `motion_snapshot_try_read`, with original
  timestamp and coherent counts. No motion mutex or counter interpolation
  is used for the record references. Window interpolation stays bracketed.

The source processes at most32 queued samples per call, one at a time so
FIRST BAR context is bound before later runs. The pending slot is rechecked
against coherent current control before publication, plus50 ms control/record
age bounds. If that final check fails, it was NOT offered: no ACK is required
for an unseen record. The source discards it and retains diagnostic evidence.
The vehicle must independently reject wrong-context or aged copies, including
any source/control publication interleaving. No cross-board transaction exists.

Microsecond and millisecond clocks wrap independently. `barcode_live_ms_at`
uses the current paired clock phase to convert ORIGINAL source observations;
it never divides a wrapped microsecond stamp by1000 as a global clock. Tests
cover nonzero phase at both wraps. No retry restamps read_ms or encoder pairs.

## One occupied slot and boot serial

A complete uppercase window publishes once. Its original packet is immutable
until terminal ACK; ADC samples received while occupied are not queued for
later replay. A matching fresh ACK retires it. Mode0 keeps reading disabled;
only future eligible samples can build a NEW ten-narrow leading white proof.

Serial advances for the entire BOOT, nonzero modulo32 bits, skipping zero.
STOP, epoch/route change, ACK, calibration and window reset NEVER reset it.
`source_init` is idempotent before tasks and never resets either writer's
state. A final pre-publication rejection can leave a serial gap, never reuse.
Old-epoch occupied slots remain until a terminal ACK; vehicle rejects/ACKs
such slots, never admits them. An old ACK X cannot retire the new slot X+1.

Partial optical/context failures produce diagnostic rows only (offered0),
not fake complete packets requiring ACK. Full geometry A--Z windows produce
an offered1 packet, even if its mapping is NONE. Non-letter Code39 payloads
retain their symbol in rejection diagnostics but never become course tokens.
Reason is keyed diagnostic evidence, not an extra field in the40 B ABI.
Window reasons0--10 are WAIT, OK, CALIBRATION, LOSS, MOTION, START, WIDTH,
GAP, EXTENT, TAIL, NOT_LETTER. Policy100--107 are CONTROL, DISABLED,
CONTEXT, CUT, CALIBRATION, RECORD_AGE, MOTOR_AGE, CLOCK.

## Bounded USB/DUMP evidence

`barcode_live_source_dump()` is one stopped consumer; it takes no LINE mutex
and reads bounded coherent rows. A concurrent/interrupted or overwritten row
is skipped, not printed as a valid copy. No DTR-retained TEST dump is changed.

| Tag | Evidence |
| --- | --- |
| BC_LIVE | SHADOW, current epoch, sample count, total frame/attempt counts |
| BC_CPU | Total source us / samples, max source slice, max eligible control age and source publication age |
| BC_CPU_ELIGIBLE | Source us / samples and max while control mode1/2; STOP samples do not dilute this view |
| BC_FRAME | Original epoch/serial/letter/command, reason, offered flag, control/publication ages, white means/difference, frozen thresholds |
| BC_REF | Original first/final ADC edges, final-edge motor upper bracket, eligible ADC time, original read time/counts |
| BC_EDGES / BC_EDGE | Total/retained/lost raw GP28 edges AFTER window, ADC time, original upper motor time/counts, polarity/raw |

Keep the latest8 frame/attempt rows and24 edge rows across runs, keyed by
original epoch/serial. Counts are lifetime; age maxima reset at epoch change.
Control-age maximum covers eligible modes or active partial windows, not
an intentionally stopped mode0 board left unpublished for minutes.
CPU snapshots cover completed samples, including summary-publication cost in
the next snapshot. Slice us is wall time, so preemption can only increase this
cost estimate. Offered rows include pre-publication age; a final-rejected row
retains its preparation-age evidence and offered0, not a claim of delivery. Existing LINE cpu_us remains its mutex path; add source
cost, or use heartbeat cpu_load for the total. All CPU figures await hardware.

Post-window edge evidence is UNCLASSIFIED; later barcode ink is not a proven
junction. ADC loss/clipping, unreadable control/motor or motor faults stop that
trace and increment its loss count. Original upper brackets need at most30 ms;
no interpolation/extrapolation supplies these diagnostic encoder pairs.
These rows are NOT an offset measurement or permission. Compare GP28 with
the independent line-pair observation on the SAME transverse18 mm stripe;
unknown barcode-to-cross spacing cannot establish the sensor offset.

## CR1 memory and compiled stack

Matched fake-A relink against `366ec9a`, same current objects/profile:

| Image | Static RAM change | Text change |
| --- | --- | --- |
| Feed, before external DUMP hook | +3264 B | +8736 B |
| Feed + forced-retained DUMP hook | +3448 B | +10176 B |

The full-hook RAM delta consists of producer1208, frame boards800, edge
boards864, summary116, dump scratch184, input/motor/pending164, calibration
copy56, newly retained control board24, scalars/alignment32 bytes. The whole
source-storage assertion is at most4096 B, a ceiling, NOT an allocation target.
There is no full-pass capture or frozen TEST decoder workspace pulled in.
TEST/capture and BENCH static sizes stay unchanged from366ec9a.

### Required state versus removable evidence

CR1 follow-up: ONE future build switch, `BARCODE_LIVE_DIAGNOSTICS`, must
select all evidence storage together. Default ON for SHADOW and offset work.
Do not switch it off before acceptance. It is not implemented in this commit.

| Required addition versus366ec9a | Bytes |
| --- | ---: |
| Producer without evidence fields, including window988/context20/slot40 | 1092 |
| Input descriptor / coherent motor scratch | 68 / 64 |
| LINE-owned calibration copy | 56 |
| Newly retained control board | 24 |
| Required total | 1304 |

| Diagnostic-only addition | Bytes |
| --- | ---: |
| Producer event96, counters20, event flag1; net after alignment | 116 |
| Eight coherent frame/attempt boards | 800 |
| Twenty-four coherent post-window edge boards | 864 |
| Mutable summary56 + coherent summary board60 | 116 |
| DUMP scratch: frame96 + edge32 + summary56 | 184 |
| Pending post-window edge | 32 |
| Post-window/frame-ring scalar state and alignment | 32 |
| Removable total | 2144 |

The sums1304+2144 equal the measured3448 B full-hook delta. Existing record
board/ready storage from366ec9a stays required and is NOT counted twice.
The producer's diagnostic event currently doubles as a packet-construction
scratch. When adding the ONE switch, build the packet directly in the required
slot, copy it into the event only when diagnostics are ON, and keep age checks
in required scalar logic. Remove all rows, tracing, summary counters/scratch
and event publication together; retain heartbeat/age admission, acquisition
loss checks, epoch/revision/cut binding, ACK/boot serial and all decoder guards.
Required calibration and motor scratch must not be hidden behind that switch.

Individual compiler frames: LINE88 B, feed80 B, policy72 B, window step128 B,
model176 B. Longest application decoder chain sums to544 B. Library math,
RTOS context/interrupt paths require additional space; this is NOT a measured
high-water result. The existing LINE stack is2048 B. No stack size or firmware
stack-floor enforcement is added. First SHADOW hardware run must show at
least256 B free in GP20 `[stack]` output. CPU targets remain about2% average
and below1 ms per source slice; use BC_CPU/BC_CPU_ELIGIBLE and heartbeat
cpu_load. A failure is an acceptance blocker, not grounds to weaken guards.

## Validation and next gate

Strict and Clang ASan/UBSan host suites:329/329. Tests cover first-read and
paused freshness, stale/no progress, every control field, wrap/phase, ADC loss,
motor faults/staleness, calibration generation, both print orders, mode2 still
valid0, terminal ACK, BUSY no replay, old-epoch ACK and old ACK vs new slot,
boot-serial zero skip, real board publication and final-context race, stopped
DUMP, spin/leading delta and two post-window edges with original brackets.
LINE integration asserts that motor reads and printing stay outside its mutex.

Firmware: normal, TEST/capture, BENCH, MQTT and fake-A; strict ARM20 units,
SDK normal/fake-A link probes. All five firmware builds have zero warnings.
Clang uses the existing missing-format-attribute exception for host tm_printf
stubs: the RTOS unsigned-byte format signature cannot take printf's standard
char-format attribute. No new compiler exception was added.
Artifacts: `build/b2b-validation/step4b-*`.
Link-only DUMP-retention ELF is NOT FOR FLASH. Wi-Fi example settings were
used only for link validation and removed. No push/flash was performed.

After Sub_Worker5 merges: verify SHADOW hardware with the LARGE print at the
fixed5.5 mm mount and normal146 mm/s LINE. Keep full source/vehicle logs,
stack/CPU evidence and same-stripe offset observations. Live action remains
blocked until geometry, mapping, temporal safety and integration are approved.
