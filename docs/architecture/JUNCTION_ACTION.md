# B2a — Forced corner at the next junction

## Scope and approval

The operator approved this redesign on 2026-11-10 through Main_Worker.
It replaces the blind bridge and axle-centred encoder turn in `eb18004`.
B2a uses a fake START command, not a live barcode decoder. B2b agreement with
the barcode owner is still pending. No barcode source file is changed.

| Command | Action |
| --- | --- |
| NONE | Run unchanged LINE. No detector or action. |
| A | Follow LINE past the bar; SEARCH LEFT; accept the first new line. |
| B | Follow LINE past the bar; SEARCH RIGHT; accept the first new line. |
| C | Follow LINE throughout; suppress only the junction detector for160 mm. |
| D | SEARCH LEFT; skip the first new line; accept the second incoming stem. |

A command is latched once at START, not on LINE restart. Confirmation consumes
it even if the job fails. Failure never retries it. STOP cancels unfinished
work. A natural non-LINE handoff cancels an unconsumed pending command at once.
A/B/D private SEARCH stays inside the LINE wrapper; it does not cancel itself.

## Ownership and entry points

- `app_program/vehicle/junction_action.{h,c}`: pure detector and job.
- `junction_line.{h,c}`: adapter around unchanged `lt_*` and `ls_*`.
- `junction_source.{h,c}`: fake START producer and bounded source board.
- `junction_print.c`: telemetry-owned saved completion and GP20 DUMP.
- `vehicle_task.c`: coherent counts, fresh line input and live events.
- `tests/test_junction.c`: detector, job, safety and LINE comparison fixtures.
- `tests/test_vehicle_task.c`: real task glue with service/motion doubles.

There is no new module ID. KR1 module5 still means SCAN reverse. LINE,
SEARCH, SCAN and DETOUR files/settings are unchanged. Only vehicle glue
commands motion; no job waits, tasks, allocation, PWM, GPIO or IRQ changes.
Before claim, normal LINE outcomes and routes win. During a claimed job,
line loss during LINE travel, obstacle, sensor fault or deadline causes an
explicit stop, not a detour or blind fallback. Existing ESTOP and motion
faults remain authoritative. The LINE-only IMU diagnostic epoch is excluded
only during private SEARCH; normal LINE travel in C/bar clearance remains
LINE. IMU still cannot command navigation or motors.

## Draft B2b source contract

`ja_command_t` remains a24-byte value, not a wire packet:

| Offset | Member | Meaning |
| --- | --- | --- |
| 0 | `uint32_t epoch` | Current odd run epoch. |
| 4 | `uint32_t sequence` | Nonzero producer sequence; skip zero at wrap. |
| 8 | `uint32_t read_ms` | Coherent observation time. |
| 12 | `int32_t read_counts[2]` | Coherent LEFT/RIGHT encoder counts. |
| 20 | `uint8_t command` | A/B/C/D =1/2/3/4. |
| 21 | `uint8_t valid` | Exactly1. |
| 22 | `uint8_t source` | Fake1; future live barcode2. |
| 23 | `uint8_t reserved` | Zero. |

The consumer performs one bounded read per tick. A failed read leaves its
output unchanged; it never waits. B2a's sole writer is vehicle START, using a
bounded motion-board read at most10 ms old. A busy/stale START read fails
without waiting or starting a run. A live backend must replace that producer,
not write concurrently. `junction_source_start()` is not a multi-producer API.

Keep both counts: average short modular per-wheel differences, not absolute
wrapped counters. Read time/counts are evidence, not permission to execute an
already occupied cross. Each new command still needs a clear forward baseline.
Invalid commands, wrong runs, consumed/old sequences and replacement during
pending/active work are rejected. Sequence gaps must be less than half the
32-bit range. STOP/new START cancels the old job.

The consumer exposes busy and consumed-sequence ACK. Set busy BEFORE ACK;
a claimed action remains busy after consumption. A live producer must check
busy again after reading ACK and reject an unacknowledged occupied slot. It
must have one writer and a defined START/STOP handshake, reject/report lost
replacement, and not republish the same decoded barcode repeatedly. Source
age, delayed publication and concurrency still require B2b agreement. The
consumer's natural non-LINE cancellation policy is now fixed by this redesign;
it does not supply the missing live-source handshake or summary integration.

## Junction detection

Detection is unchanged: pending command, active LINE, calibrated fresh samples,
forward encoder motion, existing pair junction bit AND both channels>=700,
continuously for6 mm (about75 mean counts). A new command needs6 mm clear
forward baseline before a new black edge. A threshold ramp to the pair's
1250-sum junction flag does not erase an already clear approach. Noise,
nonqualifying, stationary, reverse or duplicate samples cannot accumulate a
candidate. Save the first qualified edge; confirmation latency is not added.

The measured pair spacing is14.5 mm, height about9 mm. Inferred21 mm optical
spacing is not a physical measurement. Barcode white gap20 mm gives only
5.5 mm ideal point-sensor margin; yaw, footprint and drift defeat that model.
The141 mm barcode length is not an exclusion rule. A wide bar can pass N=6.
A pending-command barcode-only hardware pass is mandatory; synthetic fixtures
are not proof of physical rejection.

## Phases and bounds

```text
pending -> confirmed C -> normal LINE for160 mm -> done
        -> confirmed A/B/D -> LINE past bar -> private SEARCH
          A/B: white -> first new line -> LINE + done
          D:   white -> skip first -> white -> second -> LINE + done
fault -> explicit zero command -> existing shell FAULT/STOP
```

Bar clearance requires33 mm from the saved edge AND two fresh consecutive
nonjunction/not-both-black observations.33 rounds up18 mm stroke plus14.5 mm
spacing as an initial clearance allowance for skew/optical exit. The spacing
is lateral, not a measured longitudinal offset; this sum is a starting safety
allowance, not an exact projection for arbitrary yaw. Verify the actual exit
on hardware. Approximate123 mm sensor radius then leaves the axle roughly
100 mm behind centre, with side-arm contact roughly30–40 degrees. Neither
that geometry nor the contact estimate is used as a turn endpoint.

Continue the original LINE controller on every bar-clear/C tick; never force
equal wheel speeds. C completes only after160 mm from first edge, fresh line
level evidence and uninterrupted LINE operation. Only its detector is disarmed.
No extra junction hold is added: J1 showed the existing LINE can cross `+`.
Pair mode does not use the old three-sensor junction-hold implementation.

Private SEARCH uses unchanged `ls_*` at `SEARCH_CFG.speed_mm_s`=131 mm/s.
Reject all old/global FOUND events. Require both normalized channels<300 for
20 ms of fresh observations, then a later fresh channel>=500. These are local
qualified FOUND events for `ls_*`; do not edit global line state. A/B accept
the first such line. D records/skips it, requires a new white interval, then
accepts the second only at>=120 degrees. An earlier apparent second line
fails rather than being mistaken for the incoming stem. Resume LINE at once;
no settle, taper, centring pass, axle approach or requested-angle endpoint.
DONE marks line handoff, NOT final physical arm alignment.

| Guard | Bound |
| --- | --- |
| Bar exit |70 mm maximum from edge;1.5 s after confirmation |
| C exit |160 mm minimum;3 s after confirmation; no line loss |
| A/B SEARCH | Stop at150 degrees or4 s without a qualifying line |
| D SEARCH | Second line>=120; stop at270 degrees or6 s |
| Private helper extent |160/280 degrees, beyond the hard limit; never reverse sweep |
| Whole A/B/C job |6 s after confirmation |
| Whole D job |8 s after confirmation |
| Spin wheel stall |500 ms without progress in commanded direction, per wheel |
| Spin direction/translation | Wrong direction beyond3 degrees; mean translation>15 mm |
| Sample age | Pure guard50 ms; vehicle10 ms check remains stricter |

Whole-job caps can preempt leg budgets. All time differences are unsigned;
counts use short signed modular deltas and64-bit intermediate arithmetic.
The unchanged scale24.05 differential counts/degree is used ONLY for safety
windows and measured reports, not as a turn target. LEFT angle is positive;
RIGHT negative. Pass small relative virtual counts to `ls_*`, never absolute
wrapped counts. Its extent lies beyond our stop window so its ordinary
opposite-direction sweep is unreachable. Safety bounds are not accuracy proof.

## Reports and failures

Kind7 remains live-only event v2,20 bytes, from/to LINE=1. Phase packets keep
word low byte command1..4, high byte phase; detail reason0..12:

| Phase | Current meaning |
| --- | --- |
|1 | pending |
|2 | LINE to bar clearance |
|3/5 | Legacy settle phases; decoded but never emitted now |
|4 | SEARCH first new line |
|6 | D skipped first; SEARCH second new line |
|7 | done / LINE handoff |
|8 | failed |
|9 | unused/cancelled before confirmation, including natural non-LINE handoff |
|10 | C normal LINE with detector suppressed |

Two new angle variants preserve the same event size: detail=`0x80|command`
for first line, `0x40|command` for accepted line. Word is signed16-bit
centidegrees. A/B emit both; D emits first at skip and accepted at second;
C emits neither. Angle events precede their phase event and do not enter
RAM logs/summary. Queue loss can remove any live event. Keep USB and revision
identity too. Old `eb18004` phase2 meant blind approach,4 encoder turn,
6 reacquisition,10 blind bridge; retain tested revision when reading history.
Legacy v1 reserved kind7 remains readable. No version/summary change.

Reasons:0 OK,1 cancel,2 stale,3 stall,4 travel/exit limit,5 search window,
6 missing/early line,7 obstacle,8 sonar fault,9 reverse,10 wheel imbalance,
11 whole-job limit,12 legacy settle only. Failure uses `0xb20+reason` and the
existing shell fault path. No source identity is added to live packets.

The52-byte result includes source, run/sequence, phase/reason, edge/search/
final counts, first/accepted angles and elapsed time. Marks bits0/1/2 validate
the count groups; bits3/4 validate first/accepted angles (zero can be valid).
Telemetry prints it once and repeats on GP20 DUMP. BEGIN clears the saved
result. Nonblocking queue failure can lose the saved result. Message116 B,
INFO108 B, KR1 and summaryv3 stay unchanged. B2b owns later barcode summary work.

See [validation and J0–J6](../kernel/JUNCTION_VALIDATION.md).
