# Barcode live command path (producer gate)

Status: host-tested producer gate, OFF by default. No live-action robot image
is approved. A live image needs these three items first:

- SW9 adds V1 (pending expiry).
- SW9 adds V3 (SEARCH policy).
- The operator measures the clearance (`BARCODE_MEASURED_CLEAR_MM`).

## Scope

The vehicle consumer already existed: [JUNCTION_LIVE_ADAPTER.md](../architecture/JUNCTION_LIVE_ADAPTER.md)
admits a record and [JUNCTION_ACTION.md](../architecture/JUNCTION_ACTION.md)
executes it. The producer published only SHADOW/valid=0 records. This change
lets the producer publish a valid command, but only in a live opt-in build.

## When a record is valid

A record gets `valid=1` and `flags=0x7f` (OFFSET_VERIFIED set, SHADOW clear)
only when every gate passes.

| Gate | Rule | Where |
| --- | --- | --- |
| Build | `BARCODE_LIVE_ENABLE=1` | `barcode_live_producer.c`; compiled out when 0 |
| Clearance | `BARCODE_MEASURED_CLEAR_MM` > 0 (measured) | make and C errors (existing) |
| Fake source | `JUNCTION_TEST_CMD=NONE` | make and C errors (existing) |
| Map | `BARCODE_MAP` is not empty | make error (new) |
| Map at runtime | `barcode_window_mapping_id() >= 2`; the built-in A–D table (id 1) never acts | producer |
| Control | mode 2 (PERMITTED) in the context bound at the FIRST BAR, and mode 2 in the publishing sample | producer |
| Reader | accepted, the five reader flags, and MAP_CONFIRMED | producer (unchanged) |
| Command | mapped command 1..4 (NONE stays SHADOW) | producer |

All other records stay SHADOW with `valid=0`, which is the old behaviour.

The source publication recheck is unchanged: same epoch, revision and cut,
and mode not 0. The jlive control uses mode 2 in a live build and mode 1 in
other builds, so a mode change from 2 to 1 between the bind and the publish
cannot occur.

## MQTT and the dashboard

The `robot/barcode` v2 SHADOW bit (4) is 0 only when a terminal BW_OK record
had `valid=1`, so it was offered for action. Failed attempts and NONE letters
keep SHADOW. The dashboard shows a record with SHADOW 0 as LIVE, and shows
`command` as 0..4 = NONE/LEFT/RIGHT/STRAIGHT/UTURN. The vehicle result is on
`robot/event` kind 7 (JUNCTION).

## One decode gives one command

1. One accepted window gives one record with a boot-lifetime sequence. The
   record uses one slot. There is no queue and no retry.
2. The slot stays occupied until the vehicle ACKs that exact sequence.
3. jlive admits each sequence once and retires it once.
4. `ja_offer` rejects a second command, and any sequence that is not newer
   than the last consumed one.
5. While a command is held or pending, control mode is 0, so the reader is
   disabled. The same pass cannot give a second record.
6. The next pass over the barcode gives a new sequence, so it is a new
   command.

The host test `test_barcode_live_e2e_*` proves this through the real source,
producer, jlive and ja code:

- One decode gives one record and one offer.
- A duplicate offer is refused.
- The reader is disabled while the command is pending.
- The command is consumed at the first junction, not at the second.
- The default build (LIVE=0) and the built-in map give no offer.

## Epoch and run binding (unchanged)

- The record carries the epoch and revision from the first bar.
- jlive retires a record from another epoch as OLD_RUN.
- `ja_offer` checks the epoch.
- STOP publishes epoch 0.
- The revision changes at confirmed junctions, at SEARCH entry and return,
  and at permission changes. A decode that started before one of these
  cannot steer.

## Course information (OPERATOR, 2026-11-17, not measured)

- Barcode 2 is between two passes of a 4-way crossing. On the figure-8 this
  is probably the same crossing at both ends of one loop.
- The distance is not known. It is long enough that the robot straightens
  before the barcode.
- The operator believes there is no other black between barcode 2 and the
  next crossing, but is not fully sure.
- The crossing angle is about 90 degrees (not fully sure).
- Consequence: D_cross can be as long as one loop of the course8 map. The
  loops are about 3.4 m (L1) and 5.2 m (L2), from the course map, not
  measured. The clearance is easy to meet. But a pending command must last
  across curves and SEARCH events, so V1 and V3 are required.

## Vehicle work for SW9 (before any live image)

| Item | Need |
| --- | --- |
| V1 expiry | A pending live command has no limit today. Add a distance-based expiry from the offer counts. Use the measured D_cross plus a margin. The hard upper bound is the loop length. Add a new FAILED reason (EXPIRED), with wire documentation and dashboard support. |
| V2 window | Optional: consume only when the junction edge is within [D_min, D_max] of the frame end. |
| V3 SEARCH | A pending command survives a natural SEARCH today. Define the policy. Proposal: cancel when the heading change is more than about 10 degrees. |
| V4 turn | Check that the LEFT spin finds the correct branch at this crossing (about 90 degrees, OPERATOR). |
| V5 detector | Confirm that this crossing gives a confirmed junction. The detector needs both channels >= 700 permille and the junction flag. |
| V6 first run | Slow speed, kill switch ready, `BARCODE_MAP="F=LEFT"` only. |

## Clearance measurement (next robot session)

`BARCODE_MEASURED_CLEAR_MM` is the travel from the barcode frame end before
the vehicle may offer the command. Frame end is where GP28 passes the
trailing edge of the last bar. The command must be offered and armed (6 mm
of white) before the first black of the crossing. If it is not, jlive retires
it as CROSSED or UNARMED. This is safe, but the robot does not turn.

Measure D_cross with the current SHADOW image:

1. Start the run before barcode 2. Use the normal run speed.
2. Let the robot decode the barcode.
3. Stop the robot immediately after the line pair crosses the next 4-way
   crossing, and before any other junction or barcode.
4. Use GP20 DUMP. Record `[BC_EVIDENCE] frame_end L,R` and
   `[BC_CONTROL] edge L,R`. Make sure that `[BC_ADMIT] seq` is that decode.
5. Calculate D = ((edgeL - endL) + (edgeR - endR)) / 2 x 1000 / 12414 mm.
6. Do steps 1 to 5 three times. Add each D and the speed to
   [MEASUREMENTS.md](../architecture/MEASUREMENTS.md).

`scripts/barcode_session_report.py` does this calculation. It also checks
that no other dark pair occurs between the frame end and the edge
([BARCODE_SESSION_REPORT.md](BARCODE_SESSION_REPORT.md)). Existing logs give
a DERIVED D_cross of 251–254 mm only.

Set the clearance to D_min - 40 mm, and to no less than 10 mm. The 40 mm
margin is ASSUMED: it covers the 6 mm arm distance, about 6 mm of read
latency, the ticks, and slip. If D_min is less than 50 mm, a live command at
this barcode is not possible. The V1 expiry uses D_max plus a margin.

## Validation

- Red runs before the change:
  - Producer live variants failed (custom `mode2`/`mode2_ack`, none `mode2_ack`).
  - The live feed variant failed.
  - The e2e custom variant failed (abort).
  - The make test failed on the missing BARCODE_MAP error.
- After the change: GCC and Clang ASan/UBSan, 631/631 host tests.
- The firmware matrix and RAM proof results are in the commit report.
