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
cc $FLAGS host-tests/test_line.c src/line_trace.c src/line_steer.c src/obstacle.c src/line_position.c -o /tmp/test_line
cc $FLAGS host-tests/test_search.c src/line_search.c src/line_position.c -o /tmp/test_search
cc $FLAGS host-tests/test_scan.c src/servo_scan.c src/obstacle_profile.c -o /tmp/test_scan
cc $FLAGS host-tests/test_detour.c src/detour_run.c src/obstacle_detour.c src/line_position.c -o /tmp/test_detour
```

Run only the command for the module present on a module branch. Run all
four commands and executables on staging. Heartbeat, motor controller and
vehicle interface keep their existing tests in `tests/`.

Only staging adds the firmware glue (`vc_line_test` and its CMake target),
cross-module tests, simulator updates, log receiver changes, and run
records. It also holds the RTOS plan. Module source files match the
snapshot; documentation records this branch layout.
