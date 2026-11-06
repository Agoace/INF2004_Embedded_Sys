# Course template layout checks

This is the layout-only record at `2117122`. Later motion bring-up removes
the demo and connects the motor task. See [Motor controller](MOTOR_CONTROLLER.md#μt-kernel-layer) for the current layer and its not-run hardware tests.

## Scope

The source snapshot is `yx-staging` at `f15ae89`. The course template is
`template/main` at `c3b0954`. This change moves pure robot logic into the
course folders. It does not connect robot modules to the example tasks.
The examples still run, including the GP19 LED. No firmware was flashed.

## Results

| Check | Result |
| --- | --- |
| Each of the six module branches: strict C11 host tests | Pass |
| Staging: CMake host tests, including four independent module tests | 20 of 20 pass |
| Template `make -j8` | Pass; 0 warnings |
| Template `make SMP=0 -j8` | Pass; 0 warnings |
| Standard headers with both template profiles: `stdint.h`, `stdbool.h`, `stddef.h`, `string.h` | Pass; 0 warnings; no kernel headers included |
| Pure module files against the snapshot, with quoted include paths ignored | Identical; the two base header splits are checked separately |
| Application includes that point into `src/` | None |
| Simulator configure, build and CTest | 6 of 6 pass |
| Bare-metal Pico W CMake configure and full build | Pass; dummy `wifi_config.h.example` used |
| Documentation links and anchors | Pass |
| Tracked `wifi_config.h` or `wifi_secrets.h` | None |

Host tests use `-std=c11 -Wall -Wextra -Wconversion -Wshadow -Wformat=2
-Werror`. The template uses its own compiler flags. Both template profiles
compile all robot C files in `app_program/`. The linker removes unused
functions, because the example tasks do not call the robot modules yet.
Thus a successful image build is not a robot runtime test.

Toolchain: Arm GNU 14.3.Rel1, GCC 14.3.1. Pico SDK: 2.3.0. The SDK and
kernel configuration were not changed. No real credentials were used.

## Header split and reference code

- `ultrasonic.h`: result types and helpers are in subsystem 5. Driver
  declarations, echo callback and debug data stay in the `src/` wrapper.
- `obstacle_scan.h`: profile types and helpers are in subsystem 5. Servo
  and sensor driver declarations stay in the `src/` wrapper. That wrapper
  also includes the ultrasonic hardware declarations.
- `vehicle_controller.h`: pure types and the legacy interface move to
  `app_program/vehicle/`. The old C state machine stays in `src/`.
- `line_sensors.[ch]` stay in `src/`: the C file calls SDK ADC functions.
- Heartbeat, `*_pico.c`, servo/drive hardware, bare-metal benches and
  `vc_line_test.c` stay in `src/`. They are references, not template tasks.
- Simulator devices and probes include the hardware wrappers, while pure
  application modules include only the moved pure headers.

The simulator wrapper fix is also committed on `mtk3` after the module
branches were created. Staging merges that base fix without rewriting the
six single-commit module branches.

## Repeat the checks

```sh
cmake -S tests -B /tmp/mtk3-host
cmake --build /tmp/mtk3-host
ctest --test-dir /tmp/mtk3-host --output-on-failure
(cd build_make && make -j8 && make SMP=0 -j8)
cmake -S sim -B /tmp/mtk3-sim -DSIM_FIRMWARE=detour_test
cmake --build /tmp/mtk3-sim
ctest --test-dir /tmp/mtk3-sim --output-on-failure
python3 scripts/check-docs.py
```

Keep build output outside the repository or in the ignored build paths.
Remove local credentials and temporary build output after the check.
