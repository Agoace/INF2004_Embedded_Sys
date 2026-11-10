# B2a — One command at the next junction

## Scope and approval

The operator approved this design through Main_Worker. B2a uses a fake
build-time command. It does not decode a barcode. B2b source agreement is
pending with the barcode owner; do not treat the draft below as that agreement.

| Command | Action |
| --- | --- |
| NONE | No command. Run the unchanged LINE module. |
| A | Move the axle to the cross centre; turn LEFT 90 degrees. |
| B | Move the axle to the cross centre; turn RIGHT 90 degrees. |
| C | Drive straight across; ignore side arms during the bridge. |
| D | Move the axle to the cross centre; turn LEFT 180 degrees. |

A command is latched once at START, not on each LINE restart. It is consumed
when the junction is confirmed. Failure never retries the consumed command.
STOP cancels an unfinished command. START starts a new run and command.

## Ownership and entry points

- `app_program/vehicle/junction_action.{h,c}` is the pure detector/action job.
- `junction_line.{h,c}` wraps the original `lt_*` LINE module.
- `junction_source.{h,c}` is the B2a fake producer and bounded source board.
- `junction_print.c` prints a saved result in the telemetry task.
- `vehicle_task.c` supplies coherent motion counts and fresh line data.
- `tests/test_junction.c` tests the pure job and the no-command comparison.
- `tests/test_vehicle_task.c` runs A/B/C/D through the real vehicle glue.

No global module number is added. The shell remains in LINE during an action;
KR1 module 5 still means SCAN reverse. The adapter delegates unchanged LINE
start/step calls when no action owns the wheels. LINE position, trace, steering,
SEARCH, SCAN and DETOUR source/settings are not changed. The original LINE
obstacle/lost-line outcomes win before a junction is claimed. Detection is
suspended outside LINE; an unconsumed command can remain pending across those
routes. B2a does **not** establish the correct command policy across a detour
that passes an intersection. B2b must settle that policy before such a trial.

Only the vehicle glue calls motion APIs. The action returns wheel-speed values
and never accesses PWM, GPIO or interrupts. There is no new task, wait or
allocation. Existing motion faults, ESTOP, deadline and sensor/service checks
remain authoritative. A claimed action stops on obstacle or sonar failure;
it cannot start a detour in the middle of a turn.

The LINE-only IMU diagnostic epoch is zero while the action owns the wheels.
This excludes intentional junction spins from the LINE-only motion maxima;
it changes no IMU detector or motor decision.

## Draft B2b source contract

The action consumer does one bounded board read each vehicle tick. A failed
read leaves its output unchanged. It never waits for the producer. B2a's sole
writer is the vehicle task at START. Its coherent read counts/time come from
a bounded motion-board read, at most10 ms old; a busy/stale read aborts START
without waiting. A live backend must replace that producer,
not write concurrently with it. `junction_source_start()` is a fake-backend
entry point, **not** a safe multi-producer live publication API.

`ja_command_t` is a 24-byte value, not a wire packet:

| Offset | C member | Meaning |
| --- | --- | --- |
| 0 | `uint32_t epoch` | Current odd run epoch. Other runs are rejected. |
| 4 | `uint32_t sequence` | Nonzero producer sequence; wrap skips zero. |
| 8 | `uint32_t read_ms` | Time of the coherent observation at the read. |
| 12 | `int32_t read_counts[2]` | LEFT/RIGHT counts from that same observation. |
| 20 | `uint8_t command` | 1 A, 2 B, 3 C, 4 D; 0 NONE is not a pending job. |
| 21 | `uint8_t valid` | Exactly 1 for a command. |
| 22 | `uint8_t source` | 1 fake; 2 reserved for the later live barcode source. |
| 23 | `uint8_t reserved` | Must be zero. |

Both counts are kept instead of an absolute mean: one wheel can wrap while the
other does not. Compute short travel from modular per-wheel differences, then
average. Use a single coherent motion observation; do not invent travel from
nominal speed. `read_ms` and counts are source evidence, not permission to
execute a junction that was already partly observed when the command arrived.
The consumer needs a new clear forward baseline, then a new black edge.

The consumer rejects an invalid command, other epoch, consumed/old sequence,
replacement while a command is pending, or a command during an active action.
It exposes `junction_source_acknowledged()` and `junction_source_busy()`.
The consumer sets busy before publishing a consumed-sequence acknowledgement;
an active action is still busy after its source record is consumed. A live
producer must check busy again after reading ACK, without waiting, and must
also reject an unacknowledged slot. The real producer must honour
both slot occupancy and action-busy state, report a rejected/lost replacement,
and have one writer with a defined START/STOP handshake. It must not overwrite
an unacknowledged earlier command or repeatedly publish a decoded barcode.
A sequence gap must remain less than half the 32-bit range. Source age, delayed
publication and the detour policy still need B2b agreement. No live decoder,
producer concurrency protocol or barcode file is implemented here.

## Junction detection

The detector is armed only by a pending command in LINE. It requires valid,
calibrated, fresh input and forward encoder travel. A stationary, reverse or
duplicate observation cannot accumulate distance. A new command or LINE entry
needs 6 mm of clear forward line before it can recognise a new cross.

A candidate requires the existing pair junction bit **and** both normalized
channels at least 700/1000 continuously for **6 mm**, about 75 mean counts.
Any nonqualifying observation resets the candidate. The first qualifying
observation records the edge count; confirmation does not move that origin.
At the retained 146 mm/s LINE setting, 6 mm is about 41 ms. It rejects isolated
flashes but fits within an 18 mm stroke, with a nominal 12 mm margin. Sampling,
optical threshold position and steering still require physical checks.

Measured line-sensor spacing is 14.5 mm, at about 9 mm above the floor. An
inferred optical spacing of about 21 mm is not a replacement measurement.
The barcode's 20 mm white edge gap exceeds the measured point separation by
5.5 mm. Ideal point sensors therefore cannot sit on barcode ink and main-line
ink across that gap at the same time. Wide optical spots, yaw, drift onto the
barcode or a wide bar can still violate that simple model. The 141 mm barcode
length is **not** a safe time/travel exclusion rule. N alone does not reject a
wide black bar. A barcode-only pass with a pending command is mandatory; the
host noise fixtures are not that hardware proof.

## Action phases and bounds

Geometry defaults are parameters in `junction_action.h`, used by both boot
labels and `JUNCTION_CFG`. Axle-to-front-sensor distance is **123 mm**, derived
from uncertain earlier offsets; the operator estimated about 120 mm. Verify
with a ruler in J0. With an 18 mm cross, the axle target is 123+9 = **132 mm**
from the first qualified edge, about 1639 counts. Neither the 6 mm confirmation
nor its latency is added again. Optical edge position and braking can shift
the pivot; the formula is not a measured calibration.

```text
pending -> new cross -> approach -> settle -> 90/180 turn -> settle
                                                      -> reacquire -> LINE
                   C -> straight bridge -------------------------> LINE
Any action fault -> zero command -> existing shell FAULT/STOP
```

| Leg | Initial setting | Failure bound |
| --- | --- | --- |
| Approach | Equal 100 mm/s; 62 mm/s in last 20 mm | 3 s; target+30 mm; reverse >2 mm; wheel difference >12 mm |
| C bridge | Equal 100 mm/s; suppress junctions for 160 mm from edge | 3 s; exit+30 mm; fresh stable outgoing line required |
| Settle | Zero speed; both measured speeds at most 20 mm/s for 100 ms | 500 ms |
| Turn | Opposite 80 mm/s; 62 mm/s in last 15 degrees | 4 s for 90; 6 s for 180; target+20 degrees; mean translation >15 mm |
| Reacquire | Fresh centred, nonjunction LINE evidence for 30 ms; otherwise bounded pure SEARCH | 3 s; hard nominal +/-15 degrees; mean translation >15 mm |
| Moving wheel | Progress in its commanded direction | 500 ms without progress, per wheel |
| Whole job | Tick-driven, including settle/search | 10 s for A/B/C; 12 s for D |

The whole-job cap can end a job before all individual worst-case leg budgets
expire. Stale input also stops a job; vehicle's existing 10 ms line-age check
is stricter than the pure job's 50 ms guard. Bounds are starting values, not
hardware acceptance. C's 160 mm exit is 132 mm to axle centre plus 28 mm; a
new command still needs a clear baseline. Closely spaced crosses need separate
course validation; do not shorten suppression without evidence.

The differential scale is **24.05 (right-left) counts/degree**, not per wheel.
90 degrees needs about 2165 differential counts, 180 needs 4329. An ideal
in-place spin moves each wheel about 1082 or 2165 counts. LEFT is positive.
The 111 mm track and 12414 counts/m are unchanged. Slip remains unmeasured.

Old LINE_FOUND events during the spin cannot finish the turn. The nominal
encoder angle must be reached and settled first. Reacquisition uses fresh
nonjunction LINE level evidence, not only an event edge: existing SEEN/CENTRED
plus both normalized readings at least300. The extra level check rejects weak
floor that can retain CENTRED during the LOST grace period. If lost, the unchanged
pure `ls_*` helper sweeps around the **requested** heading with virtual bounded
counts. This avoids its legacy signed subtraction on absolute wrapped counts.
The commanded extent is +/-12 degrees, inside a hard +/-15 degree cap; 3 degrees
is reserved for reversal/coast. A failed reacquisition stops, not a wider SEARCH
or blind LINE resume. A centred weak pair after LOST may need the bounded sweep
to produce the normal 50% FOUND evidence before it can be accepted.

## Reports and failures

Event kind **7** is live-only, existing event v2 size 20 bytes. From/to are
both LINE=1. Word low byte is command 1..4; high byte is phase:

| Phase | Name |
| --- | --- |
| 1 | pending |
| 2 | confirmed junction / approach |
| 3 | approach settle |
| 4 | turn |
| 5 | turn settle |
| 6 | reacquire |
| 7 | done |
| 8 | failed |
| 9 | unused at STOP |
| 10 | C straight bridge |

Detail is reason: 0 OK, 1 cancelled, 2 stale, 3 stall, 4 approach limit,
5 turn limit, 6 no outgoing line, 7 obstacle, 8 sonar fault, 9 reverse,
10 wheel imbalance, 11 whole-job limit, 12 settle limit. A job failure returns
`VSM_OUT_FAILED` with payload `0xb20 + reason`. The existing shell stop reason
and fault detail carry it; no summary version or byte is added. The dashboard
names the action phase. Legacy event-v1 reserved kind7 records remain readable.
The wire event has no fake/live source field: preserve the boot/source label.

A 48-byte result is copied non-blocking to the telemetry task, which prints
one `[JUNCTION]` line after completion/cancellation and repeats it on GP20 DUMP.
It contains source, command, epoch/sequence, phase/reason, edge/centre/final
counts, final turn in centidegrees and job time. Centre counts for C are the
observed passage of the axle target, not a stopped pivot. `marks` bits 0/1/2
identify valid edge/centre/final counts; a zero count can itself be valid. This does not grow
the telemetry message size, KR1 row, INFO structure or summary. Live event and
USB queue loss remain non-critical; a full queue can lose a result. BEGIN
clears the telemetry-owned saved result. B2b owns the later summary integration.

See [validation and J0–J6 procedure](../kernel/JUNCTION_VALIDATION.md) for
build commands, software evidence, resource measurements and physical gates.
