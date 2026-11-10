# B2b local source contract, version 1

## Status and ownership

This commit supplies the canonical header and boards for parallel vehicle
work. It does NOT feed the live reader, publish decoded frames, initialize a
new task or enable actions. The initial record has valid=0 and SHADOW set.

- Header/source: `app_program/subsystem3/barcode_live_source.{h,c}`.
- Record writer/reset owner: S3 producer only.
- Control writer: vehicle task only. S3 init does not publish control.
- No wire format or existing `ja_command_t` change. Copy accepted fields;
  do not cast a shared board to an action token.
- Fake JTESTA--D remains a mutually exclusive backend, not another live writer.

## Final sizes

Record is 40 B, its sequence board is 44 B. Control is 20 B, its board is
24 B. Combined board storage is 68 B, without artificial padding. Earlier
28/32/40 B control drafts are withdrawn.

| Record byte offset | Field |
| --- | --- |
| 0 | uint32 epoch |
| 4 | uint32 sequence, nonzero per producer epoch |
| 8 | uint32 read_ms |
| 12, 16 | int32 read_counts[2] |
| 20 | uint8 command: NONE/LEFT/RIGHT/STRAIGHT/UTURN = 0/1/2/3/4 |
| 21 | uint8 valid |
| 22 | uint8 source = 2 |
| 23 | uint8 reserved = 0 |
| 24, 28 | int32 frame_end_counts[2] |
| 32 | uint32 eligibility_revision, captured at FIRST BAR START |
| 36 | uint8 original A--Z letter |
| 37 | uint8 print direction: 1 normal, 2 reverse; motion stays forward |
| 38 | uint8 flags |
| 39 | uint8 version = 1 |

Record flags, in hexadecimal: 01 DISTANCE_FORWARD, 02 WINDOW29,
04 LEADING_WHITE, 08 TAIL_ONE_N, 10 LOSS_FREE, 20 MAP_CONFIRMED,
40 OFFSET_VERIFIED, 80 SHADOW. A live action needs low seven bits set,
SHADOW clear, valid=1, supported metadata and nonzero mapped command.
Shadow, unverified-offset and unmapped observations always have valid=0.

`read_ms/counts` describe one ORIGINAL coherent motor observation at
eligibility, not a publication retry time. `frame_end_counts` are the first
original motor bracket at/after the final ADC black-to-white edge. This is a
conservative reference for later not-before gating, not interpolated encoder
truth. Exact ADC edge time and motor-bracket time stay in keyed source
statistics. Convert source microseconds to the common millisecond clock with
current us/ms observations; naive `us/1000` fails at microsecond timer wrap.

## Final control: five uint32 words

| Field | Purpose / failure case without it |
| --- | --- |
| epoch | Reject old-run record or context |
| eligibility_revision | Reject confirmed-cross or route-spanning candidates |
| revision_ms | Reject backlogged first edge after a missed route cycle |
| ack_sequence | Retire one occupied slot without overwrite/reoffer |
| mode | Disabled/BUSY, shadow, or verified live permission |

Modes: 0 disabled/non-LINE/BUSY; 1 shadow; 2 permitted live ONLY after a
measured and approved sensor-offset bound. The 150 mm placeholder remains
shadow-only. Publish control every RUN vehicle tick even if unchanged.

The single revision advances for confirmed physical junctions and route
invalidations, including SEARCH entry AND return, SCAN/DETOUR and STOP.
`revision_ms` retains the wrap-safe latest original first-edge/route cut time.
Producer snapshots one coherent context at FIRST BAR START, rejects
start_ms <= revision_ms, and rejects changed epoch/revision through publish.
Never relabel a completed old frame with a new revision.

Vehicle processes its independent encounter observation BEFORE offering a
new record. The source2-only first-credible-edge guard cancels a token not
armed before that edge; an already-armed token remains assigned to that cross.
First-edge encoder evidence stays in vehicle diagnostics for the later
same-stripe offset check. Accepted pending may survive approved short SEARCH;
partial barcode windows may not. No course-gap/pending-distance expiry exists.

ACK is terminal slot retirement, not proof that a command executed.
For accepted pending, mode0/BUSY must precede or be coherent with the ACK.
Shadow/NONE/rejected records are also retired and never reoffered. S3 must
recheck mode after ACK; no queued replay of a card seen while BUSY.

## Coherent APIs, not a freshness certificate

- `barcode_live_source_init()`: S3 startup, before tasks; idempotent.
  Publishes only the invalid initial record, never vehicle control.
- `barcode_live_source_try_read(record*)`: bounded, no consumption.
- `barcode_live_control_publish(control*)`: vehicle sole writer.
- `barcode_live_control_try_read(control*, publication_tag*)`: bounded;
  optional tag belongs to the SAME accepted coherent control copy.

Errors: E_PAR for null/invalid mode, E_OBJ before init, E_TMOUT for an
unpublished/interrupted copy, E_OK on success. Failed reads leave both output
and optional tag unchanged. Init does not reset an existing control/record.
No production API lets the vehicle write the record.

Record delivery age and control-staleness limits are BOTH 50 ms, fail closed.
The raw getters prove coherence only. The later producer must first observe
publication progress before trusting eligible control. After a scheduling
pause it must qualify progress again, rather than treat an old first read as
fresh. A local wall-clock progress watchdog and fresh ADC/motor evidence are
required; log maximum ages per run in SHADOW. The vehicle separately checks
record age, epoch/revision and first-edge timing before `ja_offer`.

## Validation scope

Header assertions fix all critical offsets and the 40/20 B layouts. Host tests
cover unready/null/unpublished reads, unchanged failed outputs/tags, coherent
BUSY+ACK, heartbeat tag progress, invalid mode, interleaved record/control
publication and idempotent init. Record injection exists only in the host
build; production exports no record setter.

The full strict host suite passes 262/262 at this ABI stage. Source lifecycle,
freshness watchdog, revision admission and slot-retirement behavior are later
producer/adapter tests, NOT implemented by the mechanical getters. No hardware
or live-action acceptance is claimed.
