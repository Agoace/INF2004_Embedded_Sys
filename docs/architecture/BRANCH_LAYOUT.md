# Branch layout

The current layout is the [mtk3 layout](#mtk3-layout). Merge module
branches only into `mtk3-staging`, with `--no-ff`. Do not rebase. Copy
needed code from other people's branches by hand. Do not merge those
branches.

## Active branches

| Branch | Use |
| --- | --- |
| `mtk3` | Base: course template and platform code |
| `mtk3-motion`, `mtk3-vehicle`, `mtk3-line`, `mtk3-search`, `mtk3-scan`, `mtk3-detour` | Module branches |
| `mtk3-telemetry` | S1 nonblocking MBF transport, RAM log and dump owner; starts from `mtk3` |
| `mtk3-staging` | Integration; the robot program |
| `main` | Original course-template history. Do not change it |
| `ziven-barcode-algo` | Barcode code (Ziven), not ported yet. Do not merge |
| `yanshan` | MQTT reference code (Buddy 1), for the later telemetry work. Do not merge |
| `jaison-imu` | IMU reference code, for subsystem 4. Do not merge |

## Archived branches

The superseded branches were deleted on 2026-11-06. Before the delete,
every commit of each branch was reachable from a kept branch or one of
the archive tags below. To see an old branch, use its tag, for example
`git switch --detach archive/yx-detour`.

| Tag | Commit | Also contains the history of |
| --- | --- | --- |
| `archive/yx-staging` | f15ae89 | `yx-heartbeat`, `yx-motor-controller`, `yx-vehicle-controller`, `yx-mod-line`, `yx-mod-search`, `yx-mod-scan`, `yx-mod-detour` (the old bare-metal module layout) |
| `archive/yx-detour` | 3ef9e4b | `yx-line-steer`; the bare-metal detour runs R1 to R4 |
| `archive/exp-detour-tuning` | 7c7c64d | `exp/backoff-0`; simulator back-off tests |
| `archive/yx-line-abstraction` | 249eac5 | `motor-line-manager` (Buddy 2 motion and line manager) |
| `archive/joseph` | ad7527d | Joseph's motion control and speed tests |
| `line-tracing-2sensor-v1` | e4ba9ec | Locked 2-sensor line tracing; `yx-line_tracing` is in `mtk3` |

`yx`, `sim`, `vehicle-controller`, `ziven-barcode-reader-2` and
`ziven-barcode-interface` were already in `mtk3` or `ziven-barcode-algo`.

Earlier records name these old branches. Read such a name as the archive
tag above.

## Tests and integration

Small independent tests are in `host-tests/`, one C file for each module.
Run them from the repository root with the commands below. The existing
`tests/test_line_trace.c` also needs SEARCH; it belongs on staging only.
`tests/test_obstacle_modules.c` needs all four modules and also belongs
on staging only. The LINE branch keeps `tests/test_line_steer.c`.

```sh
FLAGS='-std=c11 -Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror -Isrc'
cc $FLAGS host-tests/test_line.c app_program/subsystem3/line_trace.c app_program/subsystem3/line_steer.c app_program/subsystem5/obstacle.c app_program/subsystem3/line_position.c -o /tmp/test_line
cc $FLAGS host-tests/test_search.c app_program/subsystem3/line_search.c app_program/subsystem3/line_position.c -o /tmp/test_search
cc $FLAGS host-tests/test_scan.c app_program/subsystem5/servo_scan.c app_program/subsystem5/obstacle_profile.c -o /tmp/test_scan
cc $FLAGS host-tests/test_detour.c app_program/subsystem5/detour_run.c app_program/subsystem5/obstacle_detour.c app_program/subsystem3/line_position.c -o /tmp/test_detour
```

Run only the command for the module present on a module branch. Run all
four commands and executables on staging. Heartbeat, motor controller and
vehicle interface keep their existing tests in `tests/`.

Only staging adds the firmware glue (`vc_line_test` and its CMake target),
cross-module tests, simulator updates, log receiver changes, and run
records. It also holds the RTOS plan. Module source files match the
snapshot; documentation records this branch layout.

## mtk3 layout

The course template is merged from `template/main` at `c3b0954`. The base
branch `mtk3` starts at `yx` (`942e158`). Module snapshots come from
`yx-staging` at `f15ae89` (now `archive/yx-staging`). Only include paths
change in the module files. 
```text
yx + template/main -> mtk3
  +-- mtk3-telemetry ---------------+
  +-- mtk3-motion ------------------+
  +-- mtk3-vehicle -----------------+--> mtk3-staging
        +-- mtk3-line --------------+
        +-- mtk3-search ------------+
        +-- mtk3-scan --------------+
        +-- mtk3-detour ------------+
```

| Area | Files and owner |
| --- | --- |
| Subsystem 1 | `telemetry.[ch]`: priority-30 MBF consumer, RAM log/dump; future Wi-Fi publication boundary, no MQTT now; GP19 free |
| Subsystem 2 | Motion: `motor_ctl.[ch]`; producer demo removed |
| Subsystem 3 | `line_service.[ch]`: per-sample task and snapshot/calibration owner; barcode raw only. Pure modules: `line_position.[ch]`, `line_steer.[ch]`, `line_trace.[ch]`, `line_search.[ch]`; consumer demo removed |
| Subsystem 4 | IMU: no robot module yet; monitor demo removed |
| Subsystem 5 | Ultrasonic and obstacle: pure `ultrasonic.h`, `obstacle.[ch]`, `obstacle_profile.[ch]`, pure `obstacle_scan.h`, `servo_scan.[ch]`, `detour_run.[ch]`, `obstacle_detour.[ch]` |
| `app_program/vehicle/` | Shared types in `vehicle_controller.h`; interface `vehicle_sm.[ch]` and `vehicle_inputs.h` |
| `src/` | Bare-metal reference programs and SDK hardware layers |
| `tests/` | Independent host tests; kernel glue uses host doubles, no kernel/SDK runtime |
| `host-tests/` | One independent host test per new module, present on its branch |

**Pure modules stay separate from hardware and kernel code.** Every C file below
`app_program/` is compiled by the template. Pure modules do not include
kernel headers or SDK headers. Platform and hardware adapters use SDK-only
`*_hw.c` files; their kernel-side files never include SDK headers. References may include pure application
headers; application code must never include files from `src/`. Standard
C headers remain allowed. Module branches merge only into `mtk3-staging`.
Other people's code is copied by hand.

`line_sensors.c` mixes ADC reads and logic, so it and its hardware header
stay in `src/`. `ultrasonic.c`, `obstacle_scan.c`, `*_pico.c`, heartbeat,
bench programs, and `vc_line_test.c` also stay as bare-metal references.
The ultrasonic and obstacle-scan headers are split: pure word types and
helpers move; hardware declarations stay in `src/` wrappers. The legacy
`vehicle_controller.h` is pure and moves in full. Its old C state machine
stays in `src/`. Barcode hardware and its original helpers remain reference
code; no barcode runtime port is done by this layout change.

SCAN and DETOUR share the same `servo_scan.h` SIDE payload interface.
Cross-module tests belong only to staging. Motor Pico/bench code and robot
settings are now in `app_program/subsystem2/motor_ctl_robot.h`. The kernel
motor hardware port uses SDK-only `drv_motor_hw.c`; `src/` keeps the bare-metal reference.
The template demo is removed. The motion branch connects the motor
controller to a kernel task. Staging now connects the four pure modules
through the vehicle task and hardware adapters recorded below.
No robot hardware or timing behaviour is validated by a successful build.

### Host tests

```sh
cmake -S tests -B /tmp/mtk3-host
cmake --build /tmp/mtk3-host
ctest --test-dir /tmp/mtk3-host --output-on-failure
```

CMake selects the tests whose sources exist on the current branch. It uses
C11, `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`. Module branches
keep their independent tests; staging also runs the cross-module tests.

### Integration status

`mtk3-staging` keeps the `f15ae89` integration records, simulator changes,
log format and bare-metal references. These records describe the earlier
bare-metal runs, not tests of the template robot runtime. The existing
RTOS plan is retained as a design input. Motion step 1 implements only the
motor task, not the full vehicle plan. The kernel tick remains 10 ms.
The app-owned SDK hardware fragment adds headers and selected sources;
it changes no kernel source or configuration.

## Kernel platform bring-up

`mtk3` adds `app_program/platform/`: one core-0 GPIO IRQ dispatcher and
hardware timer access. IRQ 13 uses priority 1; USB IRQ 5 retains reset
priority 0. Kernel code and SDK register code stay in separate translation
units. The robot platform uses SDK register headers, including when the
radio is off. Build it with `PICO_SDK_PATH` set and the USB console enabled.

## Kernel motion step 1

`mtk3-motion` merges `mtk3` before adding the motor task; no branch is
rebased. `mtk3-staging` merges both with real merge commits. The platform
base removes the LED/producer/consumer/monitor demo; all empty subsystems
keep only init/start/test stubs. Motion owns S2 and connects its init/start
functions to `motion_init()` and `motion_start()`. The floor bench starts
only with BENCH=1 in normal builds; the default selects the robot task.
TEST builds run lifted-wheel subsystem checks with neither button task.

Robot settings move from `src/` to S2. Pure modules still have no kernel
or SDK includes. New kernel-side adapters include no SDK headers; new
`*_hw.c` adapters include no kernel headers. GPIO IRQ 13 has one core-0
owner and priority 1. See [Motor controller](../archive/s2-motion/MOTOR_CONTROLLER.md#μt-kernel-layer)
and TEAM.md. No on-target test is marked as passed by a build.

## Kernel robot port

`mtk3-line` merges the base before adding paired ADC hardware and the core-0
1 ms TIMER alarm-0 queue. `mtk3-scan` merges the base before adding the
ultrasonic GPIO/TIMER adapter. Both merge into the candidate with `--no-ff`.
No history is rewritten. Pure module settings and source stay unchanged.

The staging-only integration is `app_program/vehicle/vehicle_task.c` plus
the reference-extracted settings and fixed RAM log/decoder. A normal build
runs the robot; `BENCH=1` selects only the S2 bench, and `TEST=1` selects
subsystem checks with neither button task. Selection changes the image
name and stale-object profile. See [Kernel robot program](../kernel/KERNEL.md)
for rates, ADC ownership, known phase/watchdog differences and R5 status.

## STEP 7 task split

`mtk3` owns the application-only static-stack helper. `mtk3-line` owns
per-sample line processing and its snapshot. `mtk3-scan` owns asynchronous
sonar requests/results. New `mtk3-telemetry`, created from `mtk3`, owns S1
RAM logging and dump transport. Staging alone rewires the robot consumer.
Use fresh worktrees, commit in the owning branch, then merge `--no-ff`
into the candidate; never rebase or merge another student's reference
branch. [Kernel robot](../kernel/KERNEL.md) has the task/object and plan-vs-port
tables. No MQTT or barcode algorithm is added in this step.

## STEP 8 MQTT and shared template changes

`mtk3` owns shared early TIMER startup, direct Wi-Fi priority20, named
application hook, lifetime/window poll timing and the bounded publication
helper. Template code is not frozen: [TEMPLATE_CHANGES.md](TEMPLATE_CHANGES.md)
is the complete allowed file ledger against `c3b0954`. Shared fixes use
small separate commits and `--no-ff` merges; no branch is rebased.

`mtk3-motion` and `mtk3-line` own transport-independent status/sample boards.
`mtk3-scan` owns the bounded 16-result history and non-consuming latest read.
`mtk3-telemetry` owns MQTT, health31, queues, app-only settings, codec and
laptop client. Staging owns vehicle board/events/command policy, cross-board
adapters and integration/timing acceptance. Protocol and test-only
`mtk3-*` topic branches merge into their owner with `--no-ff`; they are
not additional runtime modules. Only `mtk3-staging-next` combines all work.

[MQTT design](../s1-telemetry/TELEMETRY.md) and [runbook](../s1-telemetry/OPERATION.md) describe the
non-critical network policy. New-image hardware rows remain **Not run**;
software validation never grants push or flash permission.
