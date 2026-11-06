# Agent Navigation Map — INF2004 Embedded Systems

Use this map before editing an unfamiliar part of the repository.

## Map chain

| Scope | Map or rule |
| --- | --- |
| Current repository | This file: `docs/AGENT_MAP.md` |
| Parent workspace | [School Repos map](https://github.com/Agoace/School-Repos/blob/main/docs/AGENT_MAP.md) |

The parent-workspace link resolves only when this repository is checked out as a School Repos submodule. It does not resolve in a standalone clone.

## Documentation locations

| Content | Location |
| --- | --- |
| Documentation index | [docs/README.md](README.md) |
| Repository instructions | [AGENTS.md](../AGENTS.md) |
| Public orientation and setup | [README.md](../README.md) |
| Development setup | [docs/DEVELOPMENT.md](DEVELOPMENT.md) |
| Calibration record | [docs/CALIBRATION_RESULTS.md](CALIBRATION_RESULTS.md) |
| Motor rotation drift record | [docs/MOTOR_ROTATION_DRIFT.md](MOTOR_ROTATION_DRIFT.md) |
| Interrupt and polling conventions | [docs/INTERRUPT_CONVENTIONS.md](INTERRUPT_CONVENTIONS.md) |
| USB power safety | [docs/USB_POWER_SAFETY.md](USB_POWER_SAFETY.md) |
| Wi-Fi telemetry dashboard architecture | [docs/WIFI_TELEMETRY_DASHBOARD.md](WIFI_TELEMETRY_DASHBOARD.md) |
| Week 8 Change Requirement 1 notes | [docs/week8_change_requirement_1_notes.md](week8_change_requirement_1_notes.md) |
| Ultrasonic driver record | [docs/ULTRASONIC_DRIVER.md](ULTRASONIC_DRIVER.md) |
| Forward obstacle watch record | [docs/OBSTACLE_MONITOR.md](OBSTACLE_MONITOR.md) |
| Servo obstacle scan record | [docs/OBSTACLE_SCAN.md](OBSTACLE_SCAN.md) |
| Obstacle bypass record | [docs/OBSTACLE_BYPASS.md](OBSTACLE_BYPASS.md) |
| 2D firmware simulation | [docs/SIMULATION.md](SIMULATION.md) — design and milestone status; [sim/README.md](../sim/README.md) — host build, schema, trace, and tests |
| Echo material and recorded-scan fit | [docs/SIM_ECHO_FIT.md](SIM_ECHO_FIT.md) — assumed specular settings, native-probe tables, and fit limits |
| v1.5 box-size test record | [docs/SIM_BOX_SWEEP.md](SIM_BOX_SWEEP.md) — seven fixed-front box cases, captured console, hooks, and screenshots |
| Obstacle detour handoff | [docs/OBSTACLE_DETOUR_HANDOFF.md](OBSTACLE_DETOUR_HANDOFF.md) |
| Detour v1.6 test plan | [docs/DETOUR_V1_6_TEST_PLAN.md](DETOUR_V1_6_TEST_PLAN.md) |
| ADR: motor controller owns the actuators | [docs/ADR_MOTOR_CONTROLLER.md](ADR_MOTOR_CONTROLLER.md) |
| Vehicle Controller record | [docs/VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md) |
| Two-sensor line tracing (locked design) | [docs/TWO_SENSOR_LINE_TRACING.md](TWO_SENSOR_LINE_TRACING.md) |
| Normalised line position record | [docs/LINE_POSITION.md](LINE_POSITION.md) |
| Line steering record | [docs/LINE_STEERING.md](LINE_STEERING.md) |
| Steering Wi-Fi log | [docs/STEER_WIFI_LOG.md](STEER_WIFI_LOG.md) |
| Week 6 design-review diagrams | [`design-review/`](../design-review/) — Mermaid sources and rendered images for the §5 system statechart and §4 task interaction diagram; render with `npx -y @mermaid-js/mermaid-cli -i <file>.mmd -o <file>.png -b white --size 3000` |
| Parent checkout ledger | [School Repos checkout ledger](https://github.com/Agoace/School-Repos/blob/main/docs/README.md#coursework-checkout-ledger) |

## Implementation locations

| Area | Start here |
| --- | --- |
| General GPIO probe | [`src/pin_probe.c`](../src/pin_probe.c) |
| Robo Pico diagnostics | [`src/board_diagnostics.c`](../src/board_diagnostics.c) |
| Servo test | [`src/servo_test.c`](../src/servo_test.c) |
| Positional-servo calibration | [`src/servo_calibration.c`](../src/servo_calibration.c) |
| Motor encoder calibration | [`src/motor_encoder_calibration.c`](../src/motor_encoder_calibration.c) |
| Motor duty calibration | [`src/motor_duty_calibration.c`](../src/motor_duty_calibration.c) |
| Motor speed calibration | [`src/motor_speed_calibration.c`](../src/motor_speed_calibration.c) |
| Motor rotation drift | [`src/motor_rotation_drift.c`](../src/motor_rotation_drift.c) |
| Telemetry transport | [`src/telemetry.c`](../src/telemetry.c), [`src/telemetry.h`](../src/telemetry.h) |
| Wi-Fi telemetry probe | [`src/wifi_telemetry_probe.c`](../src/wifi_telemetry_probe.c) |
| Ultrasonic driver | [`src/ultrasonic.c`](../src/ultrasonic.c), [`app_program/subsystem5/ultrasonic.h`](../app_program/subsystem5/ultrasonic.h) |
| Ultrasonic bench test | [`src/ultrasonic_test.c`](../src/ultrasonic_test.c) |
| Forward obstacle watch | [`app_program/subsystem5/obstacle.c`](../app_program/subsystem5/obstacle.c), [`app_program/subsystem5/obstacle.h`](../app_program/subsystem5/obstacle.h) |
| Obstacle watch bench test | [`src/obstacle_test.c`](../src/obstacle_test.c) |
| Servo obstacle scan | [`src/obstacle_scan.c`](../src/obstacle_scan.c), [`app_program/subsystem5/obstacle_scan.h`](../app_program/subsystem5/obstacle_scan.h) |
| Obstacle profile analysis | [`app_program/subsystem5/obstacle_profile.c`](../app_program/subsystem5/obstacle_profile.c), [`app_program/subsystem5/obstacle_profile.h`](../app_program/subsystem5/obstacle_profile.h) |
| Obstacle bypass | [`src/obstacle_bypass.c`](../src/obstacle_bypass.c), [`src/obstacle_bypass.h`](../src/obstacle_bypass.h); bench test [`src/bypass_test.c`](../src/bypass_test.c) |
| Obstacle detour (straights and turns on the spot) | [`src/obstacle_detour.c`](../src/obstacle_detour.c), [`src/obstacle_detour.h`](../src/obstacle_detour.h); track test [`src/detour_test.c`](../src/detour_test.c); host test [`tests/test_obstacle_detour.c`](../tests/test_obstacle_detour.c) |
| Normalised line position | [`app_program/subsystem3/line_position.c`](../app_program/subsystem3/line_position.c), [`app_program/subsystem3/line_position.h`](../app_program/subsystem3/line_position.h) |
| Line position bench test | [`src/line_test.c`](../src/line_test.c) |
| Line steering | [`src/line_steer.c`](../src/line_steer.c), [`src/line_steer.h`](../src/line_steer.h); bench test [`src/steer_test.c`](../src/steer_test.c) |
| Steering Wi-Fi log | [`src/wifi_log.c`](../src/wifi_log.c), packet format [`src/steer_log.c`](../src/steer_log.c); receiver [`scripts/steer_log_receiver.py`](../scripts/steer_log_receiver.py) |
| Drive, IR reads, spin calibration (bench parts) | [`src/drive.c`](../src/drive.c), [`src/line_sensors.c`](../src/line_sensors.c), [`src/line_spin.c`](../src/line_spin.c) |
| Vehicle Controller | [`src/vehicle_controller.c`](../src/vehicle_controller.c), [`app_program/vehicle/vehicle_controller.h`](../app_program/vehicle/vehicle_controller.h) |
| Servo driver (clamped) | [`src/servo.c`](../src/servo.c), [`src/servo.h`](../src/servo.h) |
| Servo scan bench test | [`src/scan_test.c`](../src/scan_test.c) |
| Host unit tests | [`tests/`](../tests/) — run without hardware; see [OBSTACLE_MONITOR.md](OBSTACLE_MONITOR.md#host-unit-test), [OBSTACLE_SCAN.md](OBSTACLE_SCAN.md#host-unit-test), [OBSTACLE_BYPASS.md](OBSTACLE_BYPASS.md#host-unit-test), [VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#host-unit-test), [LINE_POSITION.md](LINE_POSITION.md#host-unit-test), [LINE_STEERING.md](LINE_STEERING.md#host-unit-test), and [STEER_WIFI_LOG.md](STEER_WIFI_LOG.md#host-tests) |
| IR line detection and barcode reader | [`src/robocar_ir_sensors.c`](../src/robocar_ir_sensors.c), [`src/line_detector.c`](../src/line_detector.c), [`src/barcode_reader.c`](../src/barcode_reader.c) |
| Telemetry receiver | [`scripts/telemetry_receiver.py`](../scripts/telemetry_receiver.py) |
| Telemetry dashboard | [`scripts/telemetry_dashboard.py`](../scripts/telemetry_dashboard.py) |
| Telemetry JSON schema | [`TELEMETRY_SCHEMA.md`](TELEMETRY_SCHEMA.md) |
| Eventual control architecture | [`CONTROL_ARCHITECTURE.md`](CONTROL_ARCHITECTURE.md) |
| Telemetry port restriction | [`scripts/restrict-telemetry-port.sh`](../scripts/restrict-telemetry-port.sh) |
| Separate host simulator | [`sim/CMakeLists.txt`](../sim/CMakeLists.txt), [`sim/targets/detour_test.cmake`](../sim/targets/detour_test.cmake) — host-only source selection; reads firmware without changing it |
| Simulator clock, scenario, and trace | [`sim/core/runtime.c`](../sim/core/runtime.c), [`sim/core/scenario.py`](../sim/core/scenario.py), [`sim/core/protocol.py`](../sim/core/protocol.py), [`sim/core/runner.py`](../sim/core/runner.py) |
| Simulator motion, contact safety, and IR | [`sim/core/world.c`](../sim/core/world.c), [`sim/include/sim_world.h`](../sim/include/sim_world.h) — calibrated curves, coast, fractional counts, tile sensing, and conservative contact stop, and ideal/specular sector echoes |
| Simulator devices and SDK headers | [`sim/devices/placeholders.c`](../sim/devices/placeholders.c), [`sim/devices/ultrasonic.c`](../sim/devices/ultrasonic.c), [`sim/include/sim_devices.h`](../sim/include/sim_devices.h), [`sim/include/`](../sim/include/) — motion/IR/servo, timed echo/fault events, local sink, and minimal host SDK surface |
| Simulator call hooks | [`sim/targets/detour_hooks.c`](../sim/targets/detour_hooks.c), [`sim/targets/detour_decode.py`](../sim/targets/detour_decode.py) — exact external-call observations, separate inferred phases, and hook/printf step checks; no firmware edits |
| Offline trace viewer | [`sim/viewer/render.py`](../sim/viewer/render.py), [`sim/viewer/template.html`](../sim/viewer/template.html), [`sim/viewer/viewer.js`](../sim/viewer/viewer.js), [`sim/viewer/viewer.css`](../sim/viewer/viewer.css) — self-contained saved-data HTML, true/firmware poses, playback and camera controls |
| Simulator scenarios | [`sim/scenarios/box_straight.json`](../sim/scenarios/box_straight.json), [`sim/scenarios/no_box_straight.json`](../sim/scenarios/no_box_straight.json), [`sim/scenarios/unloaded_reference.json`](../sim/scenarios/unloaded_reference.json); two line sensors: [`sim/scenarios/two_sensor_track.json`](../sim/scenarios/two_sensor_track.json), [`sim/scenarios/two_sensor_straight.json`](../sim/scenarios/two_sensor_straight.json) |
| Optional local environment editor | [`sim/viewer/serve.py`](../sim/viewer/serve.py), [`sim/viewer/editor.html`](../sim/viewer/editor.html), [`sim/viewer/editor.js`](../sim/viewer/editor.js), [`sim/viewer/editor.css`](../sim/viewer/editor.css) — loopback-only environment edits, validated batch runs, save and session history; echo materials are editable, robot model/firmware are not |
| Fixed-front box-size sweep | [`sim/box_sweep.py`](../sim/box_sweep.py), [`sim/scenarios/box_wide_deep.json`](../sim/scenarios/box_wide_deep.json) — generate six size-only variants under `sim/scenarios/`, retain the baseline, and optionally run all seven cases |
| Simulator tests | [`sim/tests/test_milestone1.py`](../sim/tests/test_milestone1.py), [`sim/tests/probe.c`](../sim/tests/probe.c), [`sim/tests/test_milestone2.py`](../sim/tests/test_milestone2.py), [`sim/tests/model_probe.c`](../sim/tests/model_probe.c), [`sim/tests/test_milestone3.py`](../sim/tests/test_milestone3.py), [`sim/tests/test_specular.py`](../sim/tests/test_specular.py), [`sim/tests/ultrasonic_probe.c`](../sim/tests/ultrasonic_probe.c), [`sim/tests/test_milestone4.py`](../sim/tests/test_milestone4.py), [`sim/tests/viewer_browser.cjs`](../sim/tests/viewer_browser.cjs), [`sim/tests/test_server.py`](../sim/tests/test_server.py), [`sim/tests/editor_browser.cjs`](../sim/tests/editor_browser.cjs) |
| CMake build | [`CMakeLists.txt`](../CMakeLists.txt) |
| Documentation checker | [scripts/check-docs.py](../scripts/check-docs.py) |
| Hook installer | [scripts/install-hooks.sh](../scripts/install-hooks.sh) |
| Pre-commit gate | [.githooks/pre-commit](../.githooks/pre-commit) |
| Other coursework | Repository root and clearly named task directories |

## Repository boundaries

| Boundary | Rule |
| --- | --- |
| Current repository | This independent Git repository owns all files below its root. It has no nested repositories or submodules. |
| Parent superproject checkout | The School Repos superproject tracks this repository as a gitlink. Commit and push here first; then update the gitlink in the parent repository. |

- [Branch layout](BRANCH_LAYOUT.md)

## Course template layout

See [Branch layout](BRANCH_LAYOUT.md#mtk3-layout). Pure logic is in `app_program/`; bare-metal hardware code stays in `src/`; kernel platform register access uses isolated `*_hw.c` files. The template kernel and support code are in `kernel/`, `include/`, `lib/`, `device/`, `config/`, `etc/`, and `tools/`. Template build: [`build_make/makefile`](../build_make/makefile). Team ownership: [`TEAM.md`](../TEAM.md). Host test build: [`tests/CMakeLists.txt`](../tests/CMakeLists.txt).

## Kernel platform

- [`app_program/platform/drv_gpio_irq.c`](../app_program/platform/drv_gpio_irq.c) — kernel-owned GPIO dispatcher, core 0, IRQ 13 at priority 1.
- [`app_program/platform/drv_time.c`](../app_program/platform/drv_time.c) — microsecond and millisecond hardware clock.
- Corresponding `*_hw.c` files include SDK headers only; kernel-side files include no SDK headers.
- The template demo tasks and their hardware tests are removed. GP19 is free.

- [Motor controller](MOTOR_CONTROLLER.md) — subsystem 2.

## Motion kernel task

- [`app_program/subsystem2/motion.c`](../app_program/subsystem2/motion.c) and [`motion.h`](../app_program/subsystem2/motion.h) — controller task and inherited-mutex API.
- [`drv_motor_hw.c`](../app_program/subsystem2/drv_motor_hw.c) — SDK-only PWM, encoder inputs and buttons.
- [`motion_bench.c`](../app_program/subsystem2/motion_bench.c) — RAM log and paced USB dump; not started in TEST builds.
- [`motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h) — unchanged robot settings, moved from `src/`.
- [`testing/subsystem2/TEST_LOG.md`](../testing/subsystem2/TEST_LOG.md) — U01–U04 and B4; all not run.
- [`tests/test_motion_task.c`](../tests/test_motion_task.c), [`tests/test_gpio_irq.c`](../tests/test_gpio_irq.c), and [`tests/mtk_stubs/`](../tests/mtk_stubs/) — strict host tests with kernel/hardware doubles.

## Application SDK build hook

- [`build_make/app_sdk_hw.mk`](../build_make/app_sdk_hw.mk) — explicit SDK hardware header/source subset; no SDK startup/runtime.
- [`build_make/mtkernel_3/app_program/subdir.mk`](../build_make/mtkernel_3/app_program/subdir.mk) — application hook that includes the fragment.
- [Motor controller — SDK subset](MOTOR_CONTROLLER.md#sdk-hardware-subset) — exhaustive paths, sources and APIs that must not be used.
