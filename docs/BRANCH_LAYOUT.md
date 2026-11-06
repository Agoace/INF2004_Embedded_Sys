# Branch layout

Use one branch for each module. Merge module branches only into `yx-staging`.
Copy needed code from other people's branches by hand. Do not merge those
branches. Do not merge `yx-line-abstraction`.

```text
yx (942e158)
  +-- yx-heartbeat ----------+
  +-- yx-motor-controller ---+
  +-- yx-vehicle-controller -+--> yx-staging
        +-- yx-mod-line -----+
        +-- yx-mod-search ---+
        +-- yx-mod-scan -----+
        +-- yx-mod-detour ---+
```

The diagram shows ownership, not a rewrite of existing history. The four
`yx-mod-*` branches start at `yx-vehicle-controller`. Existing branches
keep their history: heartbeat already contains an early motor core; motor
and vehicle branches inherit heartbeat. The vehicle branch also inherits
the motor controller. These dependencies stay in place.

The source snapshot is `yx-detour` at `3ef9e4b`. Keep that branch unchanged.
The shared detectors and `obstacle_detour` are already on `yx` and do not
change. `motor_ctl_pico` uses heartbeat, so its inherited heartbeat stays.
Both SCAN and DETOUR carry the same `servo_scan.h`: its SIDE payload is a
SCAN-to-DETOUR interface. A later change can move the payload helpers to
the interface branch. Do not change them in this snapshot.

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
branch `mtk3` starts at `yx` (`942e158`). Keep all existing branches unchanged.
Module snapshots come from `yx-staging` at `f15ae89`. Only include paths
change in the module files. Do not push these branches before operator review.

```text
yx + template/main -> mtk3
  +-- mtk3-motion ------------------+
  +-- mtk3-vehicle -----------------+--> mtk3-staging
        +-- mtk3-line --------------+
        +-- mtk3-search ------------+
        +-- mtk3-scan --------------+
        +-- mtk3-detour ------------+
```

| Area | Files and owner |
| --- | --- |
| Subsystem 1 | Wi-Fi and telemetry; template example LED task remains active on GP19 |
| Subsystem 2 | Motion: `motor_ctl.[ch]`; template producer remains active |
| Subsystem 3 | Line and barcode: `line_position.[ch]`, `line_steer.[ch]`, `line_trace.[ch]`, `line_search.[ch]`; template consumer remains active |
| Subsystem 4 | IMU: no robot module yet; template monitor remains active |
| Subsystem 5 | Ultrasonic and obstacle: pure `ultrasonic.h`, `obstacle.[ch]`, `obstacle_profile.[ch]`, pure `obstacle_scan.h`, `servo_scan.[ch]`, `detour_run.[ch]`, `obstacle_detour.[ch]` |
| `app_program/vehicle/` | Shared types in `vehicle_controller.h`; interface `vehicle_sm.[ch]` and `vehicle_inputs.h` |
| `src/` | Bare-metal reference programs and SDK hardware layers |
| `tests/` | Host tests and independent CMake build; no kernel or SDK |
| `host-tests/` | One independent host test per new module, present on its branch |

**Logic in `app_program/`; SDK code outside.** Every C file below
`app_program/` is compiled by the template. Pure modules do not include
kernel headers or SDK headers. References may include pure application
headers; application code must never include files from `src/`. Standard
C headers remain allowed. Module branches merge only into `mtk3-staging`.
Other people's code is copied by hand. Never merge `yx-line-abstraction`.

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
settings stay in `src/`; the template has no motor hardware port yet.
Robot functions are compiled, not connected to the example task workflow.
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
