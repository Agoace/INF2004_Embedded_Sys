# CR1 evidence

## Status and scope

Baseline: tag `cr1-baseline`, commit `168d705`. A1 implements compact
motion observation storage; B1 removes the inactive bench stack/slot.
Their software checks pass. Module order:
motion, sonar/obstacle, IMU, telemetry/MQTT and shared faults, then
line/barcode and vehicle. Obtain approval before each
module change. Merge current staging into the helper branch first.

The local course original was read. It is not part of this record or Git.
The [motion plan](../s2-motion/CR1_MOTION_PLAN.md) records the approved
A1 substep. Controller packing and derived-copy removal are not approved.
Hardware correctness and timing for changed representations: **Not run**.

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
| B2 | Right-size measured task stacks | Not measured | Deferred |
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
board fields. B1 only is approved; B2..B5 remain deferred.

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
private setting changed. Hardware and all-load stack/timing checks remain
**Not run**; no image was flashed.

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
the [motion table](../s2-motion/CR1_MOTION_PLAN.md) gives ranges and owners.
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

No image was flashed. Hardware work/gap timing, loaded MQTT/USB behavior,
watchdog behavior and named stack high-water remain **Not run**. Host
checks and smaller RAM are not evidence of acceptable target timing.

## A2 planned: shared 16-bit modular encoder values

Idea approved; **do not start yet**. Wait for A1/B1 completion and the
barcode (`mtk3-line`) and B2a (`mtk3-vehicle`) merges. A2 touches their
files. The motion owner keeps private full 32-bit totals. Only the shared
counts become 16-bit wrapping values. This is not implemented by A1.

Safety bound: 1000 mm/s, or 12414 counts/s. Measured floor 100-percent
speeds were about 770..807 mm/s and still rising, with wheelies; the
controller uses an 80-percent duty cap and a 460 mm/s command cap. See
[the archived duty map](../archive/s2-motion/DUTY_MAP.md).

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
| Motion A1 | 190664 B total | Board 64 B | Passed | Not run |
| Sonar/obstacle | Pending | Pending | Not started | Not run |
| IMU | Pending | Pending | Not started | Not run |
| Telemetry/faults | Pending | Pending | Not started | Not run |
| Line/barcode | Pending | Pending | Not started | Not run |
| Vehicle | Pending | Pending | Not started | Not run |

Parser host checks cover wrapped sections, RAM vector, `.noinit`, archive
objects, discarded sections, zero-size sections, flash exclusion, padding
and module classification. The complete linked baseline reconciles with
ARM `size`. These checks are not hardware performance evidence.
