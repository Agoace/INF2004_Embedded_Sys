# Barcode live command path (producer gate)

Status: host-tested producer and V1/V3 vehicle policy, OFF by default.
Main approved the V1/V3 policy on 2026-11-18. A live image still needs the
approved placement checks on the actual demo course. Host tests and a
successful firmware build do not prove robot acceptance.

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
| Clearance | `BARCODE_MEASURED_CLEAR_MM` > 0; existing name, runtime distance guard (see placement rule) | make and C errors (existing) |
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
- Historical concern: D_cross can be as long as one loop of the course8
  map (about 3.4 m L1 and 5.2 m L2, map values, not measured).
  The approved straight placement rule below supersedes the proposal to
  retain commands across curves or SEARCH. V1/V3 bound pending commands.

## Vehicle work for SW9 (before any live image)

| Item | Need |
| --- | --- |
| V1 expiry | Implemented: 400 mm mean wheel travel from decode `read_counts`, live source only. UNUSED reason 13 EXPIRED; see the approved policy below. Dashboard labels are a separate follow-up. |
| V2 window | Optional: consume only when the junction edge is within [D_min, D_max] of the frame end. |
| V3 SEARCH | Implemented: 200 ms continuous SEARCH before consumption cancels the live command, UNUSED reason 14 SEARCH. SCAN/DETOUR keep CANCELLED. No heading-change rule. |
| V4 turn | Check that the LEFT spin finds the correct branch at this crossing (about 90 degrees, OPERATOR). |
| V5 detector | Confirm that this crossing gives a confirmed junction. The detector needs both channels >= 700 permille and the junction flag. |
| V6 first run | Slow speed, kill switch ready, `BARCODE_MAP="F=LEFT"` only. |

## Approved V1/V3 vehicle policy

Approved by Main on 2026-11-18. This policy applies only to source 2 live
commands in `JA_PENDING`, with `BARCODE_LIVE_ENABLE=1`.

- **V1:** expire when the mean signed wheel travel from
  `command.read_counts` reaches `BLV_PENDING_MAX_MM = 400` mm. This is a
  design value, not a measured distance. The decode follows the frame end,
  so this is not a strict 400 mm bound from `frame_end`. The origin does
  not restart at arming or during a reverse. Reverse net travel does not
  reach the positive bound. Count differences use wrap-safe subtraction
  and 64-bit addition. No time limit is added.
- The one constant is in
  [`policy.h`](../../app_program/barcode_live_vehicle/policy.h). A static
  assertion requires it to exceed `BARCODE_MEASURED_CLEAR_MM + 160`.
  The nominal recommendation is about 200 mm physical placement (at least
  160 mm). After the 49 mm line-pair lead and 6 mm confirmation, normal
  consumption is about 117–157 mm from the frame end. The 400 mm design
  limit leaves margin without retaining a command for a full loop.
- **V3:** cancel after 200 ms of continuous SEARCH, not on entry.
  `BLV_SEARCH_CANCEL_MS` is 200. Main approved this after LIVE1. A brief
  SEARCH returns to LINE without dropping the command. The timer starts
  on the first SEARCH tick and resets on return to LINE; it uses wrap-safe
  elapsed time. V1 expiry still wins during SEARCH. A new run resets it. SCAN/DETOUR keep their existing cancellation.
  No heading-change rule is added: the approved approach must be straight;
  wheel-difference heading during line corrections is not a reliable
  extra course-location check.
- A drop calls `ja_finish`: command invalid, sequence consumed (retired),
  final counts recorded, phase **UNUSED (9)**, claimed false. The new
  reasons are **13 EXPIRED** and **14 SEARCH**. UNUSED means no turn was
  attempted; it is not FAILED. SCAN/DETOUR retain reason CANCELLED (1).
  These are command drops, not emergency stops. Ordinary navigation
  continues. Active/consumed actions and fake source 1 are unchanged.
- The check runs before each normal module step. If a pending command
  reaches the bound on a tick that could confirm a junction, expiry wins.
  No expiry is applied after consumption.

Implementation: [`policy.c`](../../app_program/barcode_live_vehicle/policy.c)
wraps `ja_pending_route` through live-only forced includes on
`vehicle_task.o` and `junction_print.o`. The new folder is excluded from
all default builds. Shared action/Vehicle headers and C files are unchanged.
Kind 7 keeps its layout; see [wire contract](../s1-telemetry/MQTT_WIRE.md).

Live build (example secrets only for the recorded build):

```sh
make -C build_make clean SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
make -C build_make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
```

Output: `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_bclive_kernel_ram_bmap2_bcfail2.uf2`.
The build has no IMU capture. RAM_PROOF: 77/77 listed functions in RAM,
19852 B. ELF text 451744 B, BSS 218964 B; link-time free RAM 30356 B
(including static stacks), from ELF RAM section addresses and 270336 B.
These are build results, not measured run-time stack margins.
The 200 ms update adds 16 B of state, with text 451952 B and free 30340 B.

## Live LEFT/RIGHT turn correction

LIVE1 and LIVE2 did not turn, despite DONE. Operator observation: a twitch
then straight travel. Recorded first/accepted angles were 1.58, 39.04 and
2.45 degrees. DONE is not physical branch acceptance. Sources:
`school_stg_60e1af7/robot_LIVE1.final.log` and
`school_stg_6c40c33/robot_LIVE2.log` under `mtk3-core-review/build/`.

The old spin began about 35 mm from the edge. The axle was still about
96 mm before the arm centre (122 mm sensor lead + half an assumed 18 mm
stroke - 35). A sensor orbit of radius 122 meets that arm around 38 degrees,
not 90. The 73 mm GP28 offset is for the barcode, not the line pair.
`new_line` also allowed straight-line recontact after 20 ms with both
sensors below 30%, followed by either at least 50%. A gap/noise can give
that at 1–3 degrees. Neither mechanism proves a turn.

Approved live-only correction:

- LEFT/RIGHT own straight advance at 120 mm/s. Target 131 mm from saved
  edge (122 mm ruler lead + 9 mm half stroke), max 170 mm, timeout 2 s.
  Require two fresh nonjunction/not-both-black samples at the target.
  These are design values, not a measured crossing centre.
- During advance, stale/invalid data, sonar stop/fault, any backward wheel
  step, 500 ms without progress on either wheel, distance/time limits or
  total 6 s limit stop the action. Brief optical loss does not cancel it.
  The live LINE wrapper bypasses `lt_step` during this owned advance.
- LEFT/RIGHT accept only signed turn progress at least 60 degrees.
  Earlier contacts stay diagnostic, never DONE; another fresh 20 ms white
  departure is required after an ignored contact. Keep the first contact,
  and report the accepted contact separately. Existing 150 degree/4 s
  spin, reverse, stall and translation limits remain.
- STRAIGHT and UTURN retain their previous logic. This does not establish
  UTURN acceptance or fix the fake build.

Implementation: [`action.c`](../../app_program/barcode_live_vehicle/action.c)
is a copy of `vehicle/junction_action.c` at `43e1963`, linked instead of the
original only for LIVE=1. [`line.c`](../../app_program/barcode_live_vehicle/line.c)
provides the advance wrapper. Duplication is intentional to preserve all
default ELF/UF2 bytes. **Follow-up: merge into general ja fix.** The defect
also affects fake commands; reopen physical B2a turn acceptance.

Tests are synthetic, not physical acceptance: red first; both compilers.
They check centre travel, false contacts, 60-degree gate, departure, right
turn, stale/sonar/reverse/stall/distance/time failure and wrapper ownership.
Live example-secret profile above: text 452684 B, BSS 218980 B, free 30340 B;
RAM_PROOF 77/77, 19852 B. Main must observe a real branch turn before
acceptance; no autonomous full-course run is approved by host tests.

## Clearance measurement (next robot session)

### Operator decision (2026-11-18)

**OPERATOR:** Do not run dedicated D_cross measurement runs. Do not use the
school-track DERIVED 251–254 mm value to set the demo guard. The demo course
is different, and the team can choose the barcode position on the day.
The earlier stop-run procedure and D_min minus 40 mm rule are superseded.
The school results remain historical evidence only
([session report](BARCODE_SESSION_REPORT.md)).

### Placement rule (operator option C, 2026-11-18)

Choose the barcode location to meet a design rule, instead of calculating
the guard from a run. The existing build name `BARCODE_MEASURED_CLEAR_MM`
does not prove that its value is measured. Document its origin as a design
value, not a measured distance.

**OPERATOR (2026-11-18), option C:** G = 100 mm, no extra 100 mm
margin, and the last black bar at least 160 mm before the near edge of
the intended junction. Use a straight approach with no other black feature
between the barcode and the junction (the continuous tracking line remains).
The <= 150 mm/s first-run speed remains an ASSUMED operating recommendation,
not a measured timing guarantee.

- Guard **G = 100 mm**, counted from the barcode frame end in wheel travel.
  This is a design choice, not a required reader dimension and not a
  school-track estimate.
- There is **no extra 100 mm margin**. The geometry budget is
  100 mm guard + 49 mm line-pair lead + 6 mm arming = 155 mm,
  rounded up to **160 mm**.
- Put the trailing edge of the LAST BLACK BAR at least **160 mm**
  before the nearest edge of the intended junction's transverse black
  feature. Measure along the straight direction of travel, not from the
  paper edge. Use a ruler on the day; no dedicated D_cross run is needed.
- Keep the line continuous and clean. There must be no other dark pair,
  junction, branch, sharp curve or black patch between that bar and the
  intended junction. Keep the barcode approach straight too.
- Record the actual placement distance. It also needs an approved upper
  bound for pending expiry (V1); a minimum gap alone is not an expiry rule.

Geometry and placement budget:

| Item | Value and purpose |
| --- | --- |
| GP28 position | 73 mm forward of the axle and 73 mm left (existing reader offsets). Align the barcode under GP28, not under the line pair. The lateral 73 mm is NOT added to the straight travel guard. |
| Line pair position | About 122 mm forward of the axle (existing ruler record). It meets the junction about 49 mm before GP28 would. Thus physical last-bar-to-junction distance is NOT the wheel distance from frame end to line-pair edge. |
| Placement budget | At 160 mm physical spacing, the line pair meets black after about 111 mm of wheel travel from frame end. G = 100 mm leaves about 11 mm: 6 mm arming and only 5 mm nominal rounding allowance. |
| Pre-junction clear | ja needs at least 6 mm of fresh forward travel on the ordinary line, without a junction flag, to arm. This is not 6 mm of empty mat: the tracking line must remain visible. |
| Timing and uncertainty | No separate delivery/slip margin is included. Assume the record is ready by G and the 6 mm arming interval completes before black. Only 5 mm nominal rounding allowance remains for scheduling, placement error, slip and geometry error combined. This is NOT a verified timing or safety bound. Late delivery can prevent the turn. |
| Junction confirmation | Both norms >= 700 permille, junction flag, then 6 mm confirmation. That confirmation is AFTER the leading edge; do not use it to reduce the pre-edge spacing. |
| Post-junction bar clear | LEFT/RIGHT/UTURN use at least 33 mm from the junction edge, black exit and two fresh exit samples before SEARCH. Keep that exit area and the intended branch unobstructed. This is after consumption, not an extra pre-barcode guard. |

The reader also needs its existing 40 mm leading quiet zone on the GP28
scan path. Do not cover or dim the tracking line with the barcode sheet.

### What the operator checks on the day

1. Mark the intended junction and command. Check that no earlier feature
   could be detected as a junction. Inspect both the line-pair path and the
   GP28 path, including the barcode paper and tape.
2. Align the last black bar square to the straight approach. Measure and
   record at least 160 mm from that bar to the near edge of the junction.
   Check GP28's lateral alignment and the leading quiet zone.
3. Check that the real junction can satisfy the detector: both channels
   >= 700 permille, the junction flag, and enough width for 6 mm of
   confirmation. A dark-pair row alone does not prove this.
4. Check room for the post-edge 33 mm bar-clear phase and the intended
   branch acquisition. Use the approved speed, explicit map and pending
   expiry/SEARCH policy. Keep KILL ready for the first live attempt.
5. Record placement and guard as design/placement values, not a
   MEASURED school D_cross. Use normal-run dumps to inspect behaviour;
   the operator decision does not require special stop runs.

### Runtime and failure limits

The producer validity gates do not use G. jlive holds the token until mean
wheel travel from `frame_end` reaches G (and neither wheel is behind its
frame-end count). It then offers the command to ja. ja holds it for the
next confirmed junction, subject to the applicable expiry/SEARCH policy.

- Too little gap: a junction seen while jlive still holds the token retires
  it as CROSSED. If ja has not acquired its 6 mm clear interval, it does not
  consume at that feature. No commanded turn there is the usual outcome,
  but the robot can continue normal LINE; this is NOT an emergency stop.
- Invalid epoch/revision, route change, stale data or other failed admission
  checks reject/retire the token. Do not assume it survives SEARCH.
- A missed junction can leave a command pending for a later junction
  within the V1 travel limit. Expiry bounds that risk but cannot remove it.
- V1 limits net travel from the decode, not accumulated path length. It
  does not identify the intended junction or impose a time bound.
- An unintended feature AFTER arming that satisfies the detector CAN consume
  the command and turn at the wrong place. The code cannot identify the
  intended junction from placement alone. This rule violation is NOT
  guaranteed to fail safe; inspection and KILL remain necessary.

No reader threshold, geometry constant or producer build gate is changed.
V1/V3 are implemented; the actual-course placement checks and operator
approval remain live-image prerequisites.

### Variable-name proposal (no code change)

Recommend renaming `BARCODE_MEASURED_CLEAR_MM` to
`BARCODE_CLEAR_GUARD_MM` in a separate reviewed change. The present value
is a **100 mm design guard**, not a measured D_cross. Until that change,
use the existing name with value 100; do not use the proposed name in builds.

Proposed make/C error text: "Live actions need BARCODE_CLEAR_GUARD_MM > 0
(design guard in mm from barcode frame_end; check the placement rule)."
The rename must also update tests, build/profile labels, C references and
docs together. Preserve the nonzero live-build gate and the runtime
behaviour; do not silently reinterpret a guard as a measured distance.
This proposal does not change any variable, error message or firmware.

## Validation

- Red runs before the change:
  - Producer live variants failed (custom `mode2`/`mode2_ack`, none `mode2_ack`).
  - The live feed variant failed.
  - The e2e custom variant failed (abort).
  - The make test failed on the missing BARCODE_MAP error.
- Producer gate: GCC and Clang ASan/UBSan, 631/631 host tests.
- V1/V3: red first (expiry, SEARCH, pure policy and missing make seams),
  then GCC and Clang, 653/653 tests each. Tests cover exact expiry boundary,
  count wrap, reverse, fake/idle passthrough, SEARCH, SCAN/DETOUR, duplicate
  retirement, and end-to-end expiry/SEARCH with no later junction claim.
  Normal live consumption remains covered through the policy wrapper.
- The 200 ms update: red first; tests cover 199/200 ms, resetting the
  timer, clock wrap and expiry during SEARCH. No wire change.
  The make test checks that live-only sources and seams are absent by default.
- Default-build proof against staging `a93b1de`: HASH76 MATCH (38 UF2 and
  38 complete ELF files, including debug information), zero build warnings.
  No shared Vehicle/action C or header file changed.
- The firmware matrix and RAM proof results are in the commit report.
