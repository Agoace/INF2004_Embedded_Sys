# B2b STEP 4a — Vehicle live-source adapter

## Scope

The default NONE image reads S3 source2 in SHADOW. It never offers a SHADOW
record to the action consumer. The integrated STEP 4b producer also forces
all records to valid0, including when control mode2 is supplied.
A decoded letter is therefore evidence, not a motor command.

B2a fake A–D images remain exclusive source1 builds. The fixed24 B consumer,
locked LINE/SEARCH, subsystem5, module numbers, summaryv3 and KR1 do not change.
IMU remains report-only. B2a closure does not approve live barcode actions.

## Entry points and ownership

- `app_program/vehicle/junction_live.{h,c}`: admission, encounter observations,
  control value, terminal ACK and reason counters. No task, heap or wait.
- `junction_source.{h,c}`: compile-time fake/live selection and owner calls.
- `vehicle_task.c`: updates routes and observations before each live read.
  Fake START/read timing stays on the B2a path. Only vehicle glue commands motion.
- `junction_print.c` and subsystem1 telemetry: bounded value messages and USB.
- S3 owns initialization, record publication, decoder, window and table.
  Vehicle never initializes, resets or writes the S3 record board.

Use the canonical [source contract](../s3-line-barcode/LIVE_SOURCE_CONTRACT.md),
[window reader](../s3-line-barcode/LIVE_WINDOW_READER.md) and
[producer record](../s3-line-barcode/LIVE_PRODUCER.md). Vehicle prints the
26-entry table once at boot through S3 getters. It has no second mapping table.

## Fixed boards and validation

The canonical record is40 B; control is20 B. Their sequence/word boards use
44+24=68 B. Control contains epoch, eligibility revision, revision time,
terminal ACK sequence and mode. Vehicle is its sole writer.

A live read copies the whole record once, then samples the receipt clock.
No cast of shared storage to the24 B consumer is permitted. Successful
admission copies the first24 B fields individually without a new read time,
new counts or a new sequence. Failed output remains unchanged.

Before action permission, require source2/version1/reserved0, current epoch,
nonzero new serial, current revision, read time after the cut, letter A–Z,
valid direction1/2, action1–4, valid1 and exactly the seven required flags.
SHADOW, NONE, malformed, old-run and rejected observations never reach offer.

| Limit | Rule |
| --- | --- |
| Delivery age | At most50 ms, checked once after the coherent read. |
| LINE observation age | At most10 ms at admission; otherwise terminal refusal. |
| S3 control age | At most50 ms; S3 must first observe publication progress. |
| Clear guard | Explicit measured bound for action permission; no assumed offset. |
| Pending lifetime | No time/course-gap expiry. Routes and encounters invalidate it. |

Every RUN tick publishes control, even when its words do not change. Mode0
means disabled/non-LINE/BUSY; mode1 means SHADOW; mode2 means verified live
permission. A held clear-guard record publishes BUSY immediately. After a
successful offer, publish BUSY before or in the same coherent value as ACK.

## Encounter and route ordering

The revision is an invalidation token, not a count of physical junctions.
Advance it on confirmed encounters, SEARCH entry and return, and eligibility
transitions. Keep the latest original first-edge/route cut with modular time
comparison. Re-enabling eligibility also advances the cut. S3 captures this
context at the first GP28 bar and must not relabel queued old samples.

The independent observer uses fresh calibrated forward LINE samples, both
channels at least700 plus the junction flag,6 mm clear and6 mm confirmation.
Spin, reverse, stationary and duplicate observations cannot add distance.
The first credible edge is already too late for a new offer. Strong black
also blocks offers when no clear baseline is available.

A source2 pending command not armed before that edge is cancelled. An armed
command belongs to that encounter. Source1 behavior is unchanged. An already
admitted pending command is not rechecked against the source revision:
successful natural SEARCH retains it; failed SEARCH, SCAN, DETOUR and STOP
cancel it through the B2a policy.

## One slot, terminal ACK and serial lifetime

ACK retires ownership of a producer slot. It does not report motor execution.
Retire SHADOW/NONE/rejected slots once. ACK old-epoch terminal slots too, but
never admit them. This lets a retained slot clear after STOP/START.

The producer serial is nonzero and lasts until MCU reset. It skips zero on
wrap and is not reset by STOP, routes, epoch changes, ACK, calibration or
idempotent initialization. Prepublication refusals can leave gaps. Comparisons
assume fewer than half the32-bit range between relevant serials.

An ACK for copied slot X cannot retire a newly published X+1. A delayed motor
result cannot regress a newer source ACK. There is no replacement, queue or
retry of barcodes encountered while busy. Route cancellation is terminal.

## Build permission

Defaults are `JUNCTION_TEST_CMD=NONE`, `BARCODE_LIVE_ENABLE=0` and
`BARCODE_MEASURED_CLEAR_MM=0`. Fake A–D select source1 exclusively. A live
opt-in requires enable1, NONE and an explicit positive bound no greater than
65535 mm. Make and C checks reject invalid combinations; profile changes
invalidate objects. Fake suffixes stay `_jtestA`–`_jtestD`; opt-in adds `_bclive`.

The150 mm SHADOW label is unverified. It is not a measured default or action
permission. Host-only permitted tests use a synthetic1 mm bound; they do not
build or authorize live-action firmware. No live-action image was requested.

## Diagnostics and remaining gates

USB reports `[BC_ADMIT]`, `[BC_EVIDENCE]` and `[BC_CONTROL]`, including original
record fields, pair first-edge counts/stamp, revision/cut and maximum delivery
age. SHADOW geometry uses original frame-end and pair-edge evidence, not an
assumed sensor offset. The clear-count guard belongs to the permitted path.

Sixteen saturating16-bit per-run counters cover NONE, SHADOW, accepted,
invalid, old run, late, route, crossed, busy, guard, unarmed, stop, edge,
confirmed, LINE-age refusal and read failure. Repeated seen slots do not add
refusals. `[JUNCTION_COUNTS]` is sent after STOP and repeated at stopped DUMP;
queue failure retries before GP20 DUMP. The saturation value is65535.

Telemetry calls `barcode_live_source_dump(void)` at run end and GP20 DUMP,
after queued junction results and outside the LINE mutex. The empty weak
fallback is overridden by S3's strong definition. It is not called at a
mid-run DONE. Message116 B, INFO108 B and the4096 B buffer stay unchanged.

See [validation](../kernel/JUNCTION_VALIDATION.md). Physical SHADOW flow,
measured offset/clearance, named-task stack margins of at least256 B,
CPU/timing, rejection cases and heading acceptance remain operator gates.
No worker flash or push, private log inspection or live-action approval is
implied by software tests. IMU H3 corrected/filtered circuit evidence remains
separate and outstanding.
