# CR1: minimum-bit data representation

## Requirement

Change Requirement 1 (Week 8, `Change Requirement 1.docx`): application
data uses only the bits that its valid range needs. Scope: motion and
encoders, line and barcode, obstacle, hump, vehicle states, task/task and
ISR/task shared data, inter-task messages, MQTT telemetry and commands.

| Rule | Detail |
| --- | --- |
| Packing | C bit-fields or explicit masks/shifts |
| Faults | Each independent fault is one bit in a shared fault mask |
| MQTT | Compact binary, not individual text values |
| Shared data | Packed shared data stays race-safe |
| Week 13 deliverables | Representation table, RAM before/after, message size before/after, correctness/performance evidence |

Design rule from the Tribunal: size a field from the range that can change
robot behaviour, not from the full hardware range. Example: the sonar can
measure metres, but avoidance uses only 0–400 mm. The
[requirement details](#requirement-details) keep §11 and §33–38. The full
Week 8 analysis (per-buddy decisions, ultrasonic timing, servo and width geometry,
Spatial Optimizer, vehicle states) is in the
[architecture history](../archive/architecture/HISTORY.md#files).

## Requirement details

Kept from the Week 8 Tribunal notes (§11, §33–38). Rules and numbers are
verbatim; the lists are trimmed. Later records can supersede a value; the
current implementation is in the sections below.

### IMU hump snapshot (Week 8 §11)

| Word | Fields (bits) | Total |
| --- | --- | ---: |
| 1 | pitch 8, heading 8, turn rate 8, hump state 2, collision 1, valid 1, spare 4 | 32 |
| 2 | height 8, peak 8, age/sequence 16 | 32 |

So this exact design is **64 bits = 8 bytes**.

Decision: use **8 B** for the two-word IMU snapshot. The earlier 6 B
claim is superseded; no 6 B layout is specified.

The reserved MQTT frame adds a 4 B version/kind header to this 8 B snapshot,
for 12 B total. This is a format reservation only, not an implemented IMU
publisher. See [the IMU wire layout](../s1-telemetry/MQTT_WIRE.md#imu).

Heading quantisation: 8 bits over 360° is `360 / 256 = 1.40625° per LSB`.
At 90°/s over 13 ms, `90 × 0.013 = 1.17°`, smaller than one LSB. Decision:

- retain higher internal precision;
- publish quantised heading if desired;
- do not derive fine turn rate only from the quantised 8-bit heading;
- BYPASS / LINE_REACQUIRE should not depend only on published heading;
- encoder odometry remains important.

Minimum-bit storage does not mean minimum-bit internal computation.

### Vehicle Controller state (Week 8 §33)

Flattened state set: INIT, CALIBRATE, IDLE, LINE_FOLLOW, LINE_SEARCH,
EXECUTE_NAV, HUMP_TRAVERSE, HUMP_OBST_VALIDATE, OBSTACLE_STOP, BYPASS,
LINE_REACQUIRE, MISSION_COMPLETE, FAULT. Thirteen states require
**4 bits**.

### Mode flags, small enums and fault mask (Week 8 §34–36)

| Item | Values | Bits |
| --- | --- | ---: |
| Mode flags | RUN, LINE_EN, SCAN_REQ, SLOW (the RTOS object itself may still be wider) | 4 logical |
| Pending navigation | none, left, straight, right; if more actions are added, recalculate | 2 |
| Barcode/action | Four basic commands fit in 2 bits; with recovery/none/reverse/reserved semantics the design uses 3 | 3 |
| Fault mask | One bit per independent fault: motor stall, encoder, IMU, ultrasonic no-echo/sensor, Wi-Fi loss, collision (if a fault), other subsystem faults | Depends on the final fault list |

### Controller events and message rule (Week 8 §37–38)

| Source | Events discussed |
| --- | --- |
| Motion | MOTION_DONE, STALL, self-test/encoder status |
| Line/barcode | BARCODE, JUNCTION, LINE_LOST, LINE_FOUND, sensor status |
| Obstacle | OBST_NEAR, OBST_CLEAR, OBST_STOP, PROFILE, SENSOR_FAULT, echo self-test |
| IMU | HUMP_START, HUMP_END, COLLISION, SENSOR_FAULT, self-test/calibration status |
| Telemetry | START, STOP, ESTOP, RESET |

A flat list of about 20+ unique events would likely need around
**5 bits**, but the final event enumeration is not fixed. Alternative:
subsystem/source ID, event ID within subsystem, type-specific compact
payload.

Do not make every message contain every possible field. Prefer
**event/command type + type-specific payload**: `OBST_CLEAR` may need no
payload; `OBST_NEAR` may need bounded distance; `PROFILE` carries compact
profile; `SENSOR_FAULT` carries compact fault subtype. Motion can use
**3-bit command type + command-specific parameter**.

## Status and scope

Baseline: tag `cr1-baseline`, commit `168d705`. A1 implements compact
motion observation storage; B1 removes the inactive bench stack/slot.
Their software checks pass. Module order:
motion, sonar/obstacle, IMU, telemetry/MQTT and shared faults, then
line/barcode and vehicle. Obtain approval before each
module change. Merge current staging into the helper branch first.

The local course original was read. It is not part of this record or Git.
The [motion module plan](#motion-module-plan) records the approved
A1 substep. Controller packing and derived-copy removal are not approved.
A1/B1 hardware gate: **PASS 2026-11-10**, with the pre-existing idle-stack
margin exception below. The later idle-stack increase is not yet retested
on hardware. Main's review/merge and further CR1 work are still pending.

## Module round 2026-11-18

Baseline: `7350da0` (demo kernel `demo-kernel-v1` plus the detour
lost-result fix), demo build line, public example settings. The operator
approved category A for each module below. Category B is not approved.
Integration branch: `mtk3-cr1`. Each module has its own record:

| Module | Record | Category A | Static RAM (alone) |
| --- | --- | --- | --- |
| Sonar/obstacle | [CR1_SONAR_A.md](CR1_SONAR_A.md) | sonar board 44 to 40 B, history 704 to 640 B, obstacle board 36 to 28 B | 241228 to 241148 B |
| Vehicle | [CR1_VEHICLE_A1.md](CR1_VEHICLE_A1.md) | vehicle board 44 to 40 B | 241228 to 241224 B |
| Barcode (private evidence) | [CR1_BARCODE_A.md](CR1_BARCODE_A.md) | fail-capture edges 1536 to 1392 B per slot, barcode frame 104 to 92 B; capacity and precision unchanged | 241228 to 240892 B |
| Telemetry T110 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md#t110) | MQTT `robot/state` RUN deadline 200 to 110 ms (operator requirement, option A; expected gap about 120 ms with the 20 ms Wi-Fi poll) | no change |
| Fault mask F1 + sonar bit 5 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md), [CR1_SONAR_F.md](CR1_SONAR_F.md) | shared single-writer fault mask interface; sonar owner word (bit 5); a cost, not a saving | +8 B |
| Telemetry A (events, index, scan, info, health, sonar copy) | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) | run events 512 x 12 to 9 B, log index 24 to 20 B, scan 576 to 528 B, info 108 to 104 B, health board 120 to 112 B, sonar copy 40 to 32 B | -1608 B |
| Fault mask F (vehicle bits 4/6, STOP capture, combiner, USB output) | [CR1_VEHICLE_F.md](CR1_VEHICLE_F.md), [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) | one writer per owner word; Health is the only combiner; run mask freezes at STOP; USB dump shows now/run masks; no MQTT change; a cost | +104 B total |

All three merged (`db13b39`): static RAM 241228 to 240808 B (-420 B),
subsystem3 36501 to 36145 B, subsystem5 1013 to 945 B, vehicle 2516 to
2504 B. RAM code 21004 to 21212 B (+208 B, barcode pack code in the LINE
task). 684 host tests, 0 warnings, RAM_PROOF 79/79. Queue and MQTT sizes
do not change. Target timing and whole-stack checks are not run.
The first merge (`ea5fae6`, sonar/obstacle and vehicle only) gave 241144 B.

After T110, F1/F1b and the sonar owner word (`a3a7578`): static RAM
240816 B, subsystem5 953 B, 686 host tests, 0 warnings, RAM_PROOF 79/79
(21212 B). All robot MQTT payloads are binary (audit in
[CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md)).

Final CR1 round (`mtk3-cr1` after merge of `bea07b5`): static RAM
241228 to 239304 B (-1924 B net: -2028 B packing, +104 B fault mask).
subsystem1 80402 to 78882 B, subsystem3 36501 to 36145 B, subsystem5
1013 to 953 B, vehicle 2516 to 2512 B. ELF text 468260 B. 698 host
tests, Clang ASan/UBSan pass, 0 warnings, RAM_PROOF 79/79 (21212 B).
MQTT message sizes do not change. Robot timing at 110 ms is not
measured yet. The older baseline and motion evidence
below keep their original scope.

## Shared-access design rule

Dual-core operation is not planned now. A later move, first possibly IMU
to core 1, must remain possible. For each packed task/task or task/ISR
field:

- Assign exactly one writer.
- Publish one whole aligned 32-bit word, or use the existing sequence/word
  observation board in `app_program/platform/app_board.h`.
- Readers copy the whole word or board and unpack the local copy.
- Do not use read-modify-write on shared packed words under `DI`/`EI`.
  Those operations mask only the local core's interrupts.
- If two writers need different bits, give each writer a separate word,
  or use the existing ISR-to-task queue/board path.

The RP2040 has no data cache. Keep the board's current atomic accesses and
barriers for both-core visibility. Each module table must name the writer
and access method. Do not pack a task flag into an ISR-owned word.

## Reproduce the baseline

Use a separate checkout at the baseline. Copy only the tracked public
example headers to the ignored private-header paths. Never replace actual
settings in an operator checkout. No network or device was used here.

```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h
cp app_program/subsystem1/robot_mqtt_secrets.h.example \
   app_program/subsystem1/robot_mqtt_secrets.h
make -C build_make -j4 --output-sync=target \
    SMP=0 WIFI=cyw43 NET=lwip MQTT=1
python3 scripts/ram_report.py \
    build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.map
python3 scripts/message_sizes.py
python3 tests/test_ram_report.py
```

Compiler: ARM GNU 14.3.rel1, GCC 14.3.1, Cortex-M0+ Thumb ABI. The size
probe compiles an object and reads array symbol lengths with ARM `nm`;
it does not run firmware or use host pointer sizes. It includes public
interface headers only. Temporary probe files are removed after use.
Generated maps, ELF files and reports stay outside Git.

## Static RAM before

| Measurement | Bytes |
| --- | ---: |
| ELF text | 403988 |
| ELF data | 904 |
| ELF bss, including RAM vector | 189768 |
| Total static RAM | **190672** |
| Main RAM not statically reserved | 71472 |

The remainder is not free RAM: runtime kernel buffers and stacks also use
it. No queue, stack or log duration has been reduced.

The map parser counts linked input sections in the RAM address range.
It handles wrapped rows, `.noinit`, library objects and the RAM vector.
Discarded sections are excluded. Group totals do not include padding.

| Group | Bytes |
| --- | ---: |
| subsystem1 | 79562 |
| lwIP | 57359 |
| platform | 24377 |
| kernel | 11828 |
| Other support | 4397 |
| radio SDK | 2483 |
| subsystem4 | 2672 |
| vehicle | 1971 |
| subsystem3 | 1927 |
| USB SDK | 1907 |
| subsystem5 | 913 |
| C library | 828 |
| subsystem2 | 289 |
| SDK hardware subset | 4 |
| Input total | 190517 |
| Padding | 155 |
| **Static total** | **190672** |

Do not compare these groups directly with an `nm` filter restricted to
`b/B/d/D`: kernel `.noinit` symbols can have other symbol letters. The
link map is the authority for module attribution. `size -A` separates the
192 B RAM `vector` from `.bss` (189576 B); ordinary `size` includes both in
its bss column. Neither section may be omitted or counted twice.

| Largest linked allocation | Bytes | Source owner |
| --- | ---: | --- |
| rows | 72016 | S1 telemetry |
| PBUF pool | 36771 | lwIP memp |
| heap | 16403 | lwIP mem |
| events | 6144 | S1 telemetry |
| task `.noinit` | 4140 | kernel task |
| tx_buffer | 4096 | USB console, not S1 |
| bench_stack | 4096 | platform app_task |
| telemetry_stack | 4096 | platform app_task |
| vehicle_stack | 4096 | platform app_task |
| motion_stack | 3072 | platform app_task |
| cyw43_state | 2448 | radio SDK |
| health/IMU/sonar/line stack, each | 2048 | platform app_task |
| deviceio `.noinit` | 1884 | kernel deviceio |
| line sample queue | 1536 | S3 drv_line |

`bench_stack` is reserved by an unconditional slot in
`app_program/platform/app_task.c`. The real `motion_bench_start()` exists
only with `!APP_TEST && APP_BENCH`; S2 calls it only with `APP_BENCH`.
The robot image therefore reserves but does not start this stack. Keep it
unchanged for the motion representation comparison; removal is a separate
RAM change, not a minimum-bit saving.

## Board and queue values before

Sizes are ARM `sizeof`, not logical bit totals. Source keys refer to
headers below `app_program/`. Each observation board adds one 4 B sequence
word and rounds the payload to 4 B. Motion and health also add a 4 B time.
The script prints the complete source path and expression for each row.

| Source key | Header |
| --- | --- |
| M | `subsystem2/motion.h` |
| C | `subsystem2/motor_ctl.h` |
| L | `subsystem3/line_service.h` |
| D | `subsystem3/drv_line.h` |
| I | `subsystem4/imu_service.h` |
| H | `subsystem4/imu_hump.h` |
| G | `subsystem4/imu_hump_diag.h` |
| S | `subsystem5/sonar_service.h` |
| T | `subsystem1/telemetry.h` |
| N | `subsystem1/robot_network.h` |
| V | `vehicle/vehicle_board.h` |
| W | `subsystem1/mqtt_wire.h` |

| Type or record | Bytes | Source |
| --- | ---: | --- |
| motion_status_t | 64 | M |
| mc_config_t | 58 | C |
| mc_wheel_t | 36 | C |
| mc_t | 152 | C |
| mc_output_t | 6 | C |
| drv_line_sample_t | 12 | D |
| line_snapshot_t | 164 | L |
| imu_snapshot_t | 504 | I |
| imu_hump_report_t | 48 | H |
| imu_hump_summary_t | 6 | H |
| imu_hump_diag_report_t | 72 | G |
| sonar_request_t | 20 | S |
| sonar_snapshot_t | 44 | S |
| sonar_history_entry_t | 48 | S |
| telemetry_event_t | 12 | T |
| telemetry_info_t | 108 | T |
| telemetry_message_t | 116 | T |
| telemetry_log_index_t | 20 | T |
| telemetry_stats_t | 16 | T |
| RAM log row | 16 | T |
| robot_command_t | 8 | N |
| vehicle_snapshot_t | 40 | V |
| rm_state_t | 76 | W |
| rm_sonar_t | 40 | W |
| rm_obstacle_t | 24 | W |
| rm_detour_t | 36 | W |
| rm_event_t | 20 | W |
| rm_summary_t | 52 | W |
| rm_heartbeat_t | 112 | W |
| rm_command_t | 24 | W |
| rm_log_t | 36 | W |
| rm_log_batch_t | 36 | W |

Private board payloads: motion 68 B, health 116 B. Including sequence and
word padding, the boards reserve motion 72, line 168, IMU 508, hump 52,
diagnostic 76, run-hump 12, vehicle 44, obstacle 28, detour 40, summary 56,
log-index 24 and health 120 B. Sonar's custom board reserves 48 B; its
history has 16 x 44 B payloads and separate guards/count. This is not 16
copies of `sonar_history_entry_t`.

Queue messages are telemetry 116 B, sonar request 20 B, network event
20 B and command/ESTOP 8 B. Kernel buffer headers are separate (4 B per
message in the network queue allocation). Motion uses direct calls under
an inherited mutex; there is no current motion-command message buffer.
Line's ISR queue stores 12 B samples. These source inventories describe
the baseline only; extend the manifest when new interfaces are merged.

## MQTT application payloads before

All sizes come from the `RM_*_SIZE` constants in
`app_program/subsystem1/mqtt_wire.h`, evaluated by the ARM size probe.
MQTT topic strings, MQTT/TCP/IP framing and broker overhead are excluded.
The baseline is already binary; do not claim a JSON-to-binary saving.

| Topic or record | Bytes |
| --- | ---: |
| robot/state v5 | 76 |
| robot/sonar v2 | 36 |
| robot/obstacle v2 | 24 |
| robot/detour v2 | 36 |
| robot/status v2 | 6 |
| robot/event v2 | 20 |
| robot/heartbeat v4 | 108 |
| robot/summary v3 | 52 |
| command v1 | 22 |
| log control v2 | 36 |
| batched log header | 36 |
| one log row/event record | 16 |
| batch with 12 records | 228 |

No implemented IMU topic exists in this baseline. A reserved wire layout
is not a transmitted message. The 16 B event wire record is distinct from
the 12 B internal telemetry event and the 20 B live MQTT event.

## Separate savings categories

Keep category A and category B in separate commits. Step 0 is measurement
only. A1 has the measured result below; B results use a separate row.
Measure each package against its immediate parent and the frozen baseline;
otherwise later merges can hide the source of a RAM change.

### A: CR1 minimum-bit savings

The module table below records A only. For the RAM log, field packing is
A; reducing capacity or recording rate is B. Never count the same bytes
in both categories.

### B: Other RAM savings

Each package needs a separate plan, approval, test and commit. Keep the
following order. Do not edit lecturer `lwipopts.h` or `sys_arch.h`.

| Package | Candidate | Saving | Status |
| --- | --- | --- | --- |
| B1 | Omit non-BENCH stack/slot | 4112 B MQTT | Software passed |
| B2 safety | Increase idle stack to 768 B | -256 B (cost) | Approved |
| B2 remaining | Right-size other measured stacks | Not measured | Deferred |
| B3 | Change run-log capacity/rate | Not approved | Deferred |
| B4 | Keep immutable tables in flash | Not measured | Deferred |
| B5 | Remove duplicate stored copies | Not measured | Deferred |

B1 is approved and implemented in a separate commit after A1. Both
`bench_stack` and its `slots[]` entry are under `#if APP_BENCH`. Other
slot names and task-start lookup are unchanged. BENCH keeps the same
4096 B allocation and name. Non-BENCH
`app_task_start(..., "motion_bench")` returns `E_PAR`, not another stack.
See the B1 measurements below.

B2 proposed margin: round each allocation up to 8 B after adding
`max(256 B, ceil(worst_observed_use / 4))` to the worst measured use.
This gives at least 256 B and at least 25 percent extra stack. Require
named-task, worst-case target measurements covering DUMP, reconnect,
fault/ESTOP, scan/detour, hump and, later, barcode. A cumulative high-water
from an incomplete run or a compiler local frame is insufficient. Do not
reduce a stack until the operator approves that task's coverage and size.

Removing stored derived fields is recorded in B5, not as minimum-bit
packing. The motion plan separates that saving from packing the retained
board fields. B1 and the specific idle-stack safety increase are approved;
other B2 work and B3..B5 remain deferred.

## B1: omit the inactive bench stack and slot

Baseline for this package: A1 (`08319f1`), not `cr1-baseline`. The only
firmware change is conditional stack and slot allocation in
`app_program/platform/app_task.c`. No remaining stack is resized.

| MQTT measurement | A1 before | B1 after | Delta |
| --- | ---: | ---: | ---: |
| text | 404780 | 404764 | -16 |
| data | 904 | 888 | -16 |
| bss | 189760 | 185664 | -4096 |
| **Static RAM** | **190664** | **186552** | **-4112** |
| Platform linked inputs | 24377 | 20261 | -4116 |
| bench_stack | 4096 | 0 | -4096 |
| S2 linked inputs | 281 | 281 | 0 |
| Motion board | 64 | 64 | 0 |
| All messages and MQTT payloads | A1 sizes | Unchanged | 0 |

The deleted slot is 20 B. Removing stack plus slot saves 4116 input bytes;
link alignment increases by 4 B (155 -> 159), so net MQTT static saving is
4112 B. Do not claim 4116 B as the linked MQTT total, or combine the A1
8 B saving with B1. Net baseline-to-current saving is 4120 B: A1 8 plus
B1 4112. Main RAM not statically reserved is now 75592 B, before dynamic
kernel allocations.

| B1 profile | text | data | bss | Static delta vs A1 |
| --- | ---: | ---: | ---: | ---: |
| Default | 340164 | 736 | 127644 | -4120 |
| Robot | 93292 | 672 | 121108 | -4112 |
| BENCH | 42370 | 148 | 95452 | 0 |
| MQTT | 404764 | 888 | 185664 | -4112 |
| TEST | 85556 | 672 | 120228 | -4112 |
| MQTT-TEST | 392900 | 888 | 184388 | -4112 |
| IMU_STREAM | 83780 | 672 | 128060 | -4112 |
| MQTT filtered | 404760 | 888 | 185664 | -4112 |
| MQTT DIAG | 405896 | 888 | 185776 | -4112 |

All nine firmware profiles build with zero warnings. The map confirms
4096 B `bench_stack` only in BENCH and no such allocation in the other
eight profiles. BENCH text/data/bss exactly match A1. ARM `sizeof` output
for every listed message and board exactly matches A1.

Strict host suite: 176/176 pass. Clang ASan/UBSan with leak checks and
halt-on-UB: 176/176 pass. Four app-task variants (normal, BENCH, MQTT,
TEST) check slot lookup, duplicate start, stack size/alignment, high-water,
missing BENCH/IMU/health names as appropriate, and all retained slots.
The BENCH test checks its unchanged 4096 B stack. All nine profiles pass
strict ARM warnings for app_task, motion and motion_pack. No template or
private setting changed. At software validation, hardware was Not run;
the later operator A1/B1 gate result is recorded above. All-load coverage
beyond that result is still untested; the helper did not flash an image.

## B2 safety correction: idle-meter stack margin

Operator approved on 2026-11-10, after the A1/B1 hardware gate. Increase
only `IDLE_STACK_WORDS` from 64 to 96: 512 -> 768 B. This is a RAM cost,
not a packing saving or a general stack reduction. MQTT-only allocation
is unchanged in scope; no other stack size or slot changes.

Hardware at `d2d71a0` used 296/512 B, leaving 216 B, below the 256 B free
policy. Staging `168d705` had 240 B free the same day: the policy shortfall
predates CR1. Interrupt frames use the stack of the running task, and the
lowest-priority idle meter runs about 55 percent of the time. A larger
reserve is required even though the idle-meter loop itself is small.
At the observed use, 768 B would leave 472 B. This is a calculation, not
a measured high-water result for the enlarged stack.

| MQTT measurement | B1 before | Idle correction | Delta |
| --- | ---: | ---: | ---: |
| text | 404764 | 404764 | 0 |
| data | 888 | 888 | 0 |
| bss | 185664 | 185920 | +256 |
| **Static RAM** | **186552** | **186808** | **+256** |
| Platform linked inputs | 20261 | 20517 | +256 |
| Idle-meter stack | 512 | 768 | +256 |

ARM public-example MQTT build and the map parser confirm the +256 B
change. Net baseline-to-current static saving is now 3864 B:
A1 8 B + B1 4112 B - idle safety reserve 256 B. All messages and motion
representations are unchanged. Strict host suite: 176/176 pass, including
the MQTT slot test updated to require 768 B. Clang ASan/UBSan also passes
176/176 with leak checks and halt-on-UB. No hardware was flashed by the
helper. Target high-water for the enlarged stack remains **Not run**.

## A1/B1 hardware result: 2026-11-10

Operator-supplied result: **PASS**, with the idle-stack margin exception
and correction above. Tested code `mtk3-cr1 d2d71a0`: configured hotspot
MQTT robot plus BENCH. MQTT lifted checks and short LINE were OK. Task
gap was 9970..10025 us, lost wakes/overruns/lost telemetry all zero,
line samples 62030 with zero missed; fault 4 was the deliberate ESTOP.
Motion stack use was 488/3072 B, free 2584 B: +48 B against the earlier
440 B, matching the compiler-local frame increase. IMU free was 552 B.
All named stacks except idle_meter retained at least 256 B free.

BENCH rest/forward150/reverse150/forward300 had correct speed signs:
about +150/-150/+325 mm/s. GP21 stop latched fault `0x04`;
`motion_bench` was present at 568/4096 B. Tick maximum was 241 us,
gap 9980..10013 us, late/missed wakes zero and USB dropped zero.

See [the hardware gate record](../../testing/subsystem2/TEST_LOG.md) for
all named stack values, operator procedure and the local USB/MQTT log
names. These results do not extend to untested detour, barcode, future A2
or the enlarged idle stack. Main reviews/merges this motion round; no
sonar planning or other CR1 advance is authorized yet.

## A1: compact motion observation board

Approved scope: observation storage only. The controller, API mutex,
ISR counts, task stacks, MQTT formats and derived fields are unchanged.
Faults are four **independent bits**: all 16 combinations are representable,
not four enum alternatives. Motion is the sole packed-board writer.
Atomic word publication and sequence validation stay in `app_board`;
readers unpack only a successful local copy. No shared packed RMW is added.

| MQTT robot measurement | Before | A1 after | Delta |
| --- | ---: | ---: | ---: |
| text | 403988 | 404780 | +792 |
| data | 904 | 904 | 0 |
| bss | 189768 | 189760 | -8 |
| **Static RAM** | **190672** | **190664** | **-8** |
| S2 linked inputs | 289 | 281 | -8 |
| Observation board including sequence | 72 | 64 | -8 |
| Decoded public motion_status_t | 64 | 64 | 0 |
| Stored payload excluding sequence | 68 | 60 | -8 |
| All queue and MQTT payload sizes | Baseline above | Unchanged | 0 |

The ARM size probe now reports `motion_packed_t` (60 B) and its 64 B
board instead of reconstructing the retired native wrapper. The map
parser reports static 190664 B and S2 281 B. Other groups are unchanged.
This is a storage saving, not a change to output precision or log rate.

The 15-word payload stores 458 logical bits; 22 unused bits are zero.
Words 0..11 hold counts L/R, travel L/R, heading, ticks, work maximum,
gap minimum/maximum, late/missed wakes and timestamp (32 bits each).
Word 12 holds two signed 16-bit speeds. Word 13 holds two 11-bit biased
duties and a 7-bit angle code. Word 14 holds a separate 7-bit pulse code,
one settled bit, four independent fault bits and one watchdog bit. The
[source header](../../app_program/subsystem2/motion_pack.h) gives offsets;
the [motion table](#approved-a1-substep-motion-observation-board) gives ranges and owners.
No B5 duplicate removal is counted in this A1 result.

Software evidence:

- Strict host suite: 173/173 pass.
- Clang ASan/UBSan suite: 173/173 pass, leak checks and halt-on-UB enabled.
  System GCC sanitizer runtimes are absent; Clang provides the working
  sanitizer build. The ordinary GCC strict suite also passes.
- Exhaustive round trips cover both duty signs, all 73 angles and pulse
  codes independently, all 16 fault combinations and all four Boolean
  combinations, and all signed-16 speeds. Every 32-bit field has zero,
  signed endpoint or unsigned endpoint fixtures. Invalid ranges, unused
  codes/bits and null pointers preserve outputs.
- Compact-board tests inject a complete new publication after each of
  the 15 payload-word loads. Torn copies are rejected, cached output and
  time stay unchanged, and the next bounded read returns the new value.
  Generic board tests retain odd/zero sequence and sequence-wrap checks.
- Nine firmware profiles build with zero warnings: default, robot, BENCH,
  MQTT, TEST, MQTT-TEST, IMU_STREAM, MQTT filtered source and MQTT DIAG.
  Each uses public example settings and `make -j4 --output-sync=target`.
- Motion and codec pass strict ARM warnings in all nine profiles:
  `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`.
- MQTT ARM local frames: motion task 168 -> 216 B; snapshot reader
  104 -> 96 B; new pack 72 B and unpack 160 B. These are local compiler
  frames, not whole-stack bounds. No static stack was reduced. Measure
  both motion and caller stack high-water on the target before acceptance.

| A1 profile | text | data | bss |
| --- | ---: | ---: | ---: |
| Default | 340188 | 760 | 131740 |
| Robot | 93316 | 688 | 125204 |
| BENCH | 42370 | 148 | 95452 |
| MQTT | 404780 | 904 | 189760 |
| TEST | 85580 | 688 | 124324 |
| MQTT-TEST | 392916 | 904 | 188484 |
| IMU_STREAM | 83804 | 688 | 132156 |
| MQTT filtered | 404776 | 904 | 189760 |
| MQTT DIAG | 405912 | 904 | 189872 |

The helper did not flash an image. The later operator hardware result is
recorded above and is limited to its stated runs. Host checks and smaller
RAM alone are not evidence of acceptable target timing.

## A2 planned: shared 16-bit modular encoder values

Idea approved; **do not start yet**. Wait for A1/B1 completion and the
barcode (`mtk3-line`) and B2a (`mtk3-vehicle`) merges. A2 touches their
files. The motion owner keeps private full 32-bit totals. Only the shared
counts become 16-bit wrapping values. This is not implemented by A1.

Safety bound: 1000 mm/s, or 12414 counts/s. Measured floor 100-percent
speeds were about 770..807 mm/s and still rising, with wheelies; the
controller uses an 80-percent duty cap and a 460 mm/s command cap. See
[the archived duty map](../archive/s2-motion/HISTORY.md#files).

The critical bound is distance, not elapsed time: each wheel's difference
from a retained origin must stay below 32768 counts, about 2.64 m. Audit
and document each maximum span: detour, reverse scan, search spin, vehicle
home moves and line-trace junction `straight_from`. Change subtraction
users only after coordination. Subtract per-wheel counts before averaging
or scaling. In particular, do not average wrapping totals then subtract.

Require one host wrap test for each user: IMU, run_stats, line_trace,
line_search, servo_scan, detour/obstacle_detour, vehicle home moves and
odometry. Locked line tracing (`line-tracing-2sensor-v1`) needs proof of
identical host behavior and a hardware run. Cosmetic USB/MQTT/dashboard
counts can show wrapped values or run distance; choose that presentation
separately. A MQTT wire change requires a separate decision.

## After and acceptance (category A)

| Module | RAM after | Message after | Software | Hardware/timing |
| --- | --- | --- | --- | --- |
| Motion A1 | 190664 B total | Board 64 B | Passed | PASS, idle note |
| Sonar/obstacle | 241148 B alone; boards 40/640/28 B | Unchanged | Passed | Not run |
| IMU | Pending | Pending | Not started | Not run |
| Telemetry/faults | Pending | Pending | Not started | Not run |
| Line/barcode | Barcode evidence: 240892 B alone; line path not changed | Unchanged | Passed (barcode) | Not run |
| Vehicle | 241224 B alone; board 40 B | Unchanged | Passed | Not run |

Parser host checks cover wrapped sections, RAM vector, `.noinit`, archive
objects, discarded sections, zero-size sections, flash exclusion, padding
and module classification. The complete linked baseline reconciles with
ARM `size`. These checks are not hardware performance evidence.

## M9 Wi-Fi stack accounting

This is a later migration measurement, not a change to the CR1 baseline
or a category-A saving. Against `d91c4c3`, matched public-example SMP1 MQTT
builds move the Wi-Fi stack from the kernel heap to the static registry.
Actual capacity stays 8320 B (8192 B request plus the former 128 B kernel
allowance); radio-only capacity stays 6272 B.

Static RAM rises **192228 -> 200564 B (+8336 B)**: an 8320 B stack, a 24 B
slot and removal of 8 B of sampled-SP state. The dynamic Wi-Fi stack payload
request falls **8320 -> 0 B**. The initial kernel pool also shrinks with
BSS growth. These are two sides of a storage move, not two independent
savings. The payload accounting difference is +16 B; allocator bookkeeping,
fragmentation and runtime heap-free space are not measured here.

The [M9 record](../kernel/KERNEL.md#wi-fi-stack-m9) gives the matched ELF
figures, tests, template boundaries and device procedure. Persistent paint
now supplies an ordinary `cyw43` stack row and Health minimum. The actual
256 B Wi-Fi hardware margin gate remains **Not run**. No stack is reduced,
no new task is added, and no merged staging B2b RAM or timing acceptance
is inferred from this helper build.

## Motion module plan

A1 observation packing is approved and implemented. Baseline:
`cr1-baseline` (`168d705`). Software checks pass. The A1/B1 hardware gate
passed on 2026-11-10 with the pre-existing idle-stack margin note; its
approved stack increase is not yet retested on hardware.
Merge current `mtk3-staging` before the module round. Approval and hardware
acceptance are separate gates. Do not move to sonar before motion closes.

The old Tribunal values are not the current contract: the controller now
runs every 10 ms, uses permille duty, accepts wheel speed rather than a
six-command enum, and has a servo API wider than the scan's +/-30 degrees.
Do not apply the historical 9-bit speed or 8-bit percentage duty directly.

### Before: static allocation

These are linked map input sizes, excluding alignment and platform stacks.

| Motion symbol or group | Bytes |
| --- | ---: |
| mc | 152 |
| public observation board | 72 |
| sampled_counts | 8 |
| raw_count | 8 |
| Seven timing/counter words | 28 |
| Four kernel object IDs | 16 |
| last_servo hardware cache | 2 |
| previous_ab | 2 |
| watchdog_armed | 1 |
| **S2 total** | **289** |

The platform also reserves a 3072 B motion stack. Do not reduce it.
At baseline, the inactive 4096 B bench stack belongs to platform, not S2.
The separate approved B1 commit omits it and its slot in non-BENCH builds;
it is not counted in A1. `mc_config_t` is 58 B: its static constant is in
flash, but `mc_t` also holds a RAM copy. Each `mc_wheel_t` is 36 B. Hardware output
`mc_output_t` is a 6 B transient. Public `motion_status_t` is 64 B; the
board adds time (4 B) and sequence (4 B), for 72 B.

There is no current motion message queue. Calls use `mtx_motion` with
priority inheritance. Readers use either that mutex or a one-attempt
atomic-word board. MQTT state is 76 B; it must not change in this round.


The [shared-access design rule](#shared-access-design-rule) applies.

### Approved A1 substep: motion observation board

Keep existing public API signatures and decoded temporary structures.
Replace only the stored observation payload with explicit masks/shifts.
Only `motion_task`, at the end of a completed tick, writes this payload.
The immutable robot configuration supplies derived values. Do not change
PID arithmetic, PWM resolution, control period, fault behavior or MQTT.

| Stored field | Valid range | Bits | Writer; access method |
| --- | --- | ---: | --- |
| Counts, each | Signed 32-bit API count | 32 | Motion; whole board |
| Speed mm/s, each | -32768..32767 | 16 | Motion; whole board |
| Duty permille, each | -800..800 | 11 | Motion; whole board |
| Travel mm, each | Signed 32-bit API value | 32 | Motion; whole board |
| Heading cdeg | Signed 32-bit API value | 32 | Motion; whole board |
| Servo pulse code | 73 angle-derived pulses | 7 | Motion; whole board |
| Servo angle | -36..36 whole degrees | 7 | Motion; whole board |
| Servo settled | 0/1 | 1 | Motion; whole board |
| Motion fault conditions | Four independent bits | 4 | Motion; whole board |
| Watchdog armed | 0/1 | 1 | Motion; whole board |
| Tick count | Wrapping u32 | 32 | Motion; whole board |
| Tick work maximum us | u32 | 32 | Motion; whole board |
| Gap minimum/maximum us, each | u32 | 32 | Motion; whole board |
| Late/missed wakes, each | Wrapping u32 | 32 | Motion; whole board |
| Publication time us | Wrapping u32 | 32 | Motion; whole board |

Duty has 1601 possible values, so 11 bits, not 8. Servo limits come from
`400 * 10 / 111 = 36`, not the sonar scan range. Encode signed fields with
explicit offset or two's-complement masks; document unused codes and
reject them in codec tests. Do not use C bit-fields or packed unaligned
struct accesses. Set all unused storage bits to zero.

The current measured-speed function clamps to the full signed-16 range.
The 460 mm/s command limit does not prove the measured-speed bound: the
wheel can coast, slip, reverse, or receive an encoder fault. Preserve the
full existing range until a narrower application contract is approved.
Likewise, wrapping counts/time/counters retain 32-bit modulo semantics;
there is no accepted maximum run time/read outage that proves less is safe.
These are explicit unresolved range questions, not a claim that motion
has completed all minimum-bit work.

Keep travel, heading and servo pulse in the first category A commit.
The pulse has 73 reachable values, one per angle; store its table/code
index in 7 bits, without changing any pulse. Keep its code separate from
the angle code in A, even though they are redundant. A separate B5 package
can later remove these derived fields:

- Travel derives from the same count pair and immutable origin/config.
- Heading derives from count difference with the existing 64-bit formula.
- Servo pulse derives from angle with the existing rounded 111/10 mapping.

Motion initialization currently starts from zero sampled counts and never
resets counts. Freeze and test this invariant before removing origins from
any stored representation. Reconstruct with the exact existing rounding
and clamping rules, not a new formula. Keep all derivation outside the
publication critical path where possible; measure reader cost too.

Category A retains all fields: 458 logical bits (58 B, 60 B aligned),
plus the 4 B sequence. Measured board storage: **72 -> 64 B**, saving
**8 B**. Measured S2 input allocation: **289 -> 281 B**; MQTT image RAM:
**190672 -> 190664 B**. Public decoded status stays 64 B. MQTT and queue
sizes stay unchanged in this substep.

A later B5 commit could remove the derived values: 355 bits (45 B, 48 B
aligned), plus sequence, for **64 -> 52 B**, another **12 B**. Do not
combine these changes or count the 12 B as minimum-bit packing. The
B5 figures remain proposals, not measured after results.

### Persistent controller and ISR inventory

The first substep does not complete CR1 for the stored controller. Review
these fields before closing the motion module. Wider local arithmetic and
kernel object IDs are not packed application storage. The general pure
controller accepts other configurations; do not silently restrict its
public contract to the robot constants.

| Persistent field | Robot range/meaning | Bits or action |
| --- | --- | --- |
| target_q8, setpoint_q8 | +/-14618 | 15 each |
| integral_q8 | +/-102400 | 18 |
| last_count, origin | Signed 32-bit counts | 32 each, or derive origin |
| speed_cpt | Current clamp: signed 16-bit | 16; measured bound open |
| duty | +/-800 permille | 11 |
| kick_travel | 0..32777 at completion | 16, or store remaining |
| kick_ms | 0..260 in 10 ms steps | 5 tick units |
| still_ms | 0..500 in 10 ms steps | 6 tick units |
| kicking | 0/1 | 1 |
| command_ms, last_tick_ms | Wrapping ms | 32 each |
| servo_settled_ms | Deadline in wrapping ms | 32 |
| servo_deg | -36..36 | 7 |
| servo_us | Derived from servo_deg | B5 removal, separate commit |
| faults | Four conditions | 4 |
| has_command | 0/1 | 1 |
| cfg | Immutable robot settings | B5 duplicate-removal candidate |
| raw_count per wheel | Signed 32-bit API counter | Keep modulo width |
| previous_ab per wheel | 0..3 | 2 each |
| sampled_counts per wheel | Signed 32-bit | 32 each |
| watchdog_armed | 0/1 | 1, separate owner word |
| last_servo | Cache of derived hardware pulse | Prefer angle code |

`kick_travel` can reach 9 + abs(-32768) before the completion check, not
just the 10-count threshold. Timer bounds must include the final increment
before timeout, not just the configured limit. The tick-unit proposal
applies only to the fixed 10 ms robot configuration. Target bound:
`460 * 12414 * 10 * 256 / 1000000`, truncated as today. Do not round it.

Writer/access review:

| Field group | Current writers | Required change before packing |
| --- | --- | --- |
| raw_count, previous_ab | Encoder IRQ on core 0 | IRQ-only whole words |
| sampled_counts, timing | Motion task | Motion-only whole words/board |
| Controller state | Motion and API caller tasks | Single owner required |
| watchdog_armed | Startup task | Own whole word; no IRQ bit sharing |
| last_servo | Existing hardware write path | Resolve owner first |
| TEST probe/window flags | Runner and motion | Separate per-writer words |

Existing `DI`/`EI` count copies are a core-0 hardware contract, not a
cross-core lock. Do not combine those counters with task-owned packed
flags. Fix signed-overflow/wrap behavior with unsigned modulo operations
and explicit signed conversion tests before claiming wrap correctness.
Do not narrow counters without approved bounds on count rate and read gap.

For controller packing, choose one design after the board substep:

1. Make motion the only persistent-state writer. API callers send bounded
   immutable command requests; motion processes them and acknowledges.
   STOP/ESTOP must retain synchronous zero-PWM completion and error policy.
   Calls must not wait for the next 10 ms tick. Define bounded queue and
   caller lifetime rules before implementation. Multiple API caller tasks
   must not share a request word that they modify independently.
2. Retain the current API and do not pack its multi-writer persistent
   state yet. This is a documented partial CR1 result, not full acceptance.

A robot-specific compact controller could use about 442 logical bits
(56 B aligned), instead of the 152 B `mc_t`, if the table bounds hold,
origin is retained, and duplicated configuration/pulse are removed. That
suggests **96 B combined gross saving**, but it mixes A packing with B5
duplicate removal. Separate and remeasure these packages before approval.
It also excludes new request/ack storage, code state and alignment; it may
not save RAM after queues are added. Do not count it as a measured saving
or implement it without another approval. Retain the generic core as the
reference calculation if a robot-specific stored-state adapter is used.
Do not add this larger ownership redesign to the first board change.

Shared system fault allocation belongs to the later S1 round. Preserve
all four motion conditions now; no separate task may edit bits in the
motion-owned published fault word. S1 will combine per-owner words into
the shared fault mask as its sole writer.

### Future A2: wrapping shared counts

The operator approved the idea, not an immediate source change. Start only
after A1/B1 and the barcode/B2a merges. Keep private totals 32-bit in the
motion owner and publish only a 16-bit modular count. Use 1000 mm/s or
12414 counts/s as the safety bound; measured 100-percent floor speeds
were about 770..807 mm/s and still rising. The current robot command cap
is 460 mm/s with 80-percent maximum duty.

For each stored origin, prove a per-wheel span below 32768 counts
(about 2.64 m). This is a distance constraint, not a timer constraint.
Audit detour, scan reverse, search spin, vehicle home moves and junction
`straight_from`. Compute per-wheel modular differences first; then average
or scale. Rework line_trace's average-total-then-subtract calculation only
with coordination and locked-line host equivalence plus a hardware run.

Add one wrap test per user: IMU, run_stats, line_trace, line_search,
servo_scan, detour/obstacle_detour, vehicle_task home moves and odometry.
USB DUMP/MQTT/dashboard count displays are cosmetic; propose wrapped
values or run distance separately. MQTT wire changes need a new decision.
See [A2 planned](#a2-planned-shared-16-bit-modular-encoder-values).
No A2 or other-module source change is part of A1/B1.

### Tests and acceptance

Before and after each approved motion substep:

1. Run strict motor controller and motion task tests, then all host tests.
2. Exhaustively round-trip duty -800..800, angle -36..36 and all 16 fault
   combinations. Test signed speed extremes, count/time wrap, reserved
   codes, unused-bit zeros and null/busy/unready unchanged-output rules.
3. Compare decoded board output with the baseline controller for forward,
   reverse, stop, ESTOP, clear, stale command, kick, stall and servo moves.
   Include odd signed rounding and asymmetric wheel counts.
4. Inject publication/reader interleavings. Readers must get a coherent
   copy or unavailable, never torn values and never a retry loop. Check
   that only motion publishes. No shared packed RMW or libatomic helper.
5. Build the MQTT image with public fixtures. Run both size scripts;
   compare board, module, total RAM and message sizes mechanically. Build
   TEST and BENCH too. Check compiler stack frames, not only static RAM.
6. Run documentation/template checks (count stays eight), line-length and
   diff checks. No template config, other module source or secrets change.
7. Operator hardware gate: lifted-wheel forward/reverse/stop/ESTOP/servo,
   then approved floor run. Record motion gap/work, late/missed wakes,
   watchdog faults, all named stack margins and loaded MQTT/USB behavior.
   Compare with same-profile baseline under the same load. No new missed
   deadlines, stale/fault behavior, or loss of required motion precision.
   Agree a numerical work-time regression limit before the run.

No helper flash or push is authorized. The A1/B1 hardware gate passed on
2026-11-10 with the idle-stack note in the evidence record. The enlarged
idle stack is not yet retested on hardware. Main reviews and merges this
round; do not advance CR1 until instructed. Persistent controller changes
and A2 remain future work, not part of this gate result.
