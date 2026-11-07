# Agent Navigation Map — INF2004 Embedded Systems

Use this map before editing an unfamiliar part of the repository.

## Map chain

| Scope | Map or rule |
| --- | --- |
| Current repository | This file: `docs/AGENT_MAP.md` |
| Parent workspace | [School Repos map](https://github.com/Agoace/School-Repos/blob/main/docs/AGENT_MAP.md) |

The parent-workspace link opens the School Repos map on GitHub, so it also works in a standalone worktree.

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
| Live MQTT dashboard (laptop) | [docs/MQTT_DASHBOARD.md](MQTT_DASHBOARD.md) |
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
| ADR: one hardware heartbeat | [docs/ADR_HEARTBEAT.md](ADR_HEARTBEAT.md) |
| ADR: Vehicle Controller runs one module at a time | [docs/ADR_VEHICLE_CONTROLLER.md](ADR_VEHICLE_CONTROLLER.md) |
| Vehicle state machine record | [docs/VEHICLE_STATE_MACHINE.md](VEHICLE_STATE_MACHINE.md) — shell [`app_program/vehicle/vehicle_sm.c`](../app_program/vehicle/vehicle_sm.c), test [`tests/test_vehicle_sm.c`](../tests/test_vehicle_sm.c) |
| Vehicle Controller line run | [docs/VC_LINE_RUN.md](VC_LINE_RUN.md) — modules [`app_program/subsystem3/line_trace.c`](../app_program/subsystem3/line_trace.c), [`app_program/subsystem3/line_search.c`](../app_program/subsystem3/line_search.c); target [`src/vc_line_test.c`](../src/vc_line_test.c) |
| Vehicle Controller detour run | [docs/VC_DETOUR_RUN.md](VC_DETOUR_RUN.md) — modules [`app_program/subsystem5/servo_scan.c`](../app_program/subsystem5/servo_scan.c), [`app_program/subsystem5/detour_run.c`](../app_program/subsystem5/detour_run.c); test [`tests/test_obstacle_modules.c`](../tests/test_obstacle_modules.c) |
| Line steering in mm/s | [docs/LINE_STEER_MM.md](LINE_STEER_MM.md) — `steer_mm_*` in [`app_program/subsystem3/line_steer.c`](../app_program/subsystem3/line_steer.c) |
| Duty map record | [docs/DUTY_MAP.md](DUTY_MAP.md) — bench [`src/duty_map_bench.c`](../src/duty_map_bench.c) |
| Heartbeat record | [docs/HEARTBEAT.md](HEARTBEAT.md) — core [`src/heartbeat.c`](../src/heartbeat.c), Pico layer [`src/heartbeat_pico.c`](../src/heartbeat_pico.c), bench [`src/heartbeat_test.c`](../src/heartbeat_test.c), test [`tests/test_heartbeat.c`](../tests/test_heartbeat.c) |
| Motor controller record | [docs/MOTOR_CONTROLLER.md](MOTOR_CONTROLLER.md) — core [`app_program/subsystem2/motor_ctl.c`](../app_program/subsystem2/motor_ctl.c), Pico layer [`src/motor_ctl_pico.c`](../src/motor_ctl_pico.c), floor bench [`src/motor_ctl_bench.c`](../src/motor_ctl_bench.c), settings [`app_program/subsystem2/motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h), test [`tests/test_motor_ctl.c`](../tests/test_motor_ctl.c) |
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
| Obstacle detour (straights and turns on the spot) | [`app_program/subsystem5/obstacle_detour.c`](../app_program/subsystem5/obstacle_detour.c), [`app_program/subsystem5/obstacle_detour.h`](../app_program/subsystem5/obstacle_detour.h); track test [`src/detour_test.c`](../src/detour_test.c); host test [`tests/test_obstacle_detour.c`](../tests/test_obstacle_detour.c) |
| Normalised line position | [`app_program/subsystem3/line_position.c`](../app_program/subsystem3/line_position.c), [`app_program/subsystem3/line_position.h`](../app_program/subsystem3/line_position.h) |
| Line position bench test | [`src/line_test.c`](../src/line_test.c) |
| Line steering | [`src/line_steer.c`](../app_program/subsystem3/line_steer.c), [`src/line_steer.h`](../app_program/subsystem3/line_steer.h); bench test [`src/steer_test.c`](../src/steer_test.c) |
| Steering Wi-Fi log | [`src/wifi_log.c`](../src/wifi_log.c), packet format [`src/steer_log.c`](../src/steer_log.c); receiver [`scripts/steer_log_receiver.py`](../scripts/steer_log_receiver.py) |
| Drive, IR reads, spin calibration (bench parts) | [`src/drive.c`](../src/drive.c), [`src/line_sensors.c`](../src/line_sensors.c), [`src/line_spin.c`](../src/line_spin.c) |
| Vehicle Controller | [`src/vehicle_controller.c`](../src/vehicle_controller.c), [`app_program/vehicle/vehicle_controller.h`](../app_program/vehicle/vehicle_controller.h) |
| Servo driver (clamped) | [`src/servo.c`](../src/servo.c), [`src/servo.h`](../src/servo.h) |
| Servo scan bench test | [`src/scan_test.c`](../src/scan_test.c) |
| Host unit tests | [`tests/`](../tests/) — run without hardware; see [OBSTACLE_MONITOR.md](OBSTACLE_MONITOR.md#host-unit-test), [OBSTACLE_SCAN.md](OBSTACLE_SCAN.md#host-unit-test), [OBSTACLE_BYPASS.md](OBSTACLE_BYPASS.md#host-unit-test), [VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#host-unit-test), [LINE_POSITION.md](LINE_POSITION.md#host-unit-test), [LINE_STEERING.md](LINE_STEERING.md#host-unit-test), [MOTOR_CONTROLLER.md](MOTOR_CONTROLLER.md#host-unit-test), [HEARTBEAT.md](HEARTBEAT.md#host-unit-test), [VEHICLE_STATE_MACHINE.md](VEHICLE_STATE_MACHINE.md#host-unit-test), and [STEER_WIFI_LOG.md](STEER_WIFI_LOG.md#host-tests) |
| IR line detection and barcode reader | [`src/robocar_ir_sensors.c`](../src/robocar_ir_sensors.c), [`src/line_detector.c`](../src/line_detector.c), [`src/barcode_reader.c`](../src/barcode_reader.c) |
| Motor bench report | [`scripts/motor_bench_report.py`](../scripts/motor_bench_report.py) — reads a `motor_ctl_bench` text log |
| Telemetry receiver | [`scripts/telemetry_receiver.py`](../scripts/telemetry_receiver.py) |
| Telemetry dashboard | [`scripts/telemetry_dashboard.py`](../scripts/telemetry_dashboard.py) |
| Telemetry JSON schema | [`TELEMETRY_SCHEMA.md`](TELEMETRY_SCHEMA.md) |
| Eventual control architecture | [`CONTROL_ARCHITECTURE.md`](CONTROL_ARCHITECTURE.md) |
| Barcode reading ideas (not built) | [`BARCODE_READING_IDEAS.md`](BARCODE_READING_IDEAS.md) |
| Telemetry port restriction | [`scripts/restrict-telemetry-port.sh`](../scripts/restrict-telemetry-port.sh) |
| Separate host simulator | [`sim/CMakeLists.txt`](../sim/CMakeLists.txt), [`sim/targets/detour_test.cmake`](../sim/targets/detour_test.cmake) — host-only source selection; reads firmware without changing it |
| Simulator clock, scenario, and trace | [`sim/core/runtime.c`](../sim/core/runtime.c), [`sim/core/scenario.py`](../sim/core/scenario.py), [`sim/core/protocol.py`](../sim/core/protocol.py), [`sim/core/runner.py`](../sim/core/runner.py) |
| Simulator motion, contact safety, and IR | [`sim/core/world.c`](../sim/core/world.c), [`sim/include/sim_world.h`](../sim/include/sim_world.h) — calibrated curves, coast, fractional counts, tile sensing, and conservative contact stop, and ideal/specular sector echoes |
| Simulator devices and SDK headers | [`sim/devices/placeholders.c`](../sim/devices/placeholders.c), [`sim/devices/ultrasonic.c`](../sim/devices/ultrasonic.c), [`sim/include/sim_devices.h`](../sim/include/sim_devices.h), [`sim/include/`](../sim/include/) — motion/IR/servo, timed echo/fault events, local sink, and minimal host SDK surface |
| Simulator call hooks | [`sim/targets/detour_hooks.c`](../sim/targets/detour_hooks.c), [`sim/targets/detour_decode.py`](../sim/targets/detour_decode.py) — exact external-call observations, separate inferred phases, and hook/printf step checks; no firmware edits |
| Offline trace viewer | [`sim/viewer/render.py`](../sim/viewer/render.py), [`sim/viewer/template.html`](../sim/viewer/template.html), [`sim/viewer/viewer.js`](../sim/viewer/viewer.js), [`sim/viewer/viewer.css`](../sim/viewer/viewer.css) — self-contained saved-data HTML, true/firmware poses, playback and camera controls |
| Simulator scenarios | [`sim/scenarios/box_straight.json`](../sim/scenarios/box_straight.json), [`sim/scenarios/no_box_straight.json`](../sim/scenarios/no_box_straight.json), [`sim/scenarios/unloaded_reference.json`](../sim/scenarios/unloaded_reference.json); narrow-beam real box (run R2 fit): [`sim/scenarios/box_real_160x250.json`](../sim/scenarios/box_real_160x250.json); two line sensors: [`sim/scenarios/two_sensor_track.json`](../sim/scenarios/two_sensor_track.json), [`sim/scenarios/two_sensor_straight.json`](../sim/scenarios/two_sensor_straight.json) |
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

## Branch layout

See [Branch layout](BRANCH_LAYOUT.md) for the active branches, merge rules and the [archive tags](BRANCH_LAYOUT.md#archived-branches) of deleted branches.

Standalone module tests: [`host-tests/`](../host-tests/), one C test per module.

- [Robo Pico RTOS multicore plan](robo_pico_rtos_multicore_plan.md) — proposed task and core split.

## Course template layout

See [Branch layout](BRANCH_LAYOUT.md#mtk3-layout). Pure logic is in `app_program/`; bare-metal hardware code stays in `src/`; kernel platform register access uses isolated `*_hw.c` files. The template kernel and support code are in `kernel/`, `include/`, `lib/`, `device/`, `config/`, `etc/`, and `tools/`. Template build: [`build_make/makefile`](../build_make/makefile). Team ownership: [`TEAM.md`](../TEAM.md). Host test build: [`tests/CMakeLists.txt`](../tests/CMakeLists.txt).

The `mtk3` base contains shared pure helpers. `mtk3-motion` and
`mtk3-vehicle` start there. `mtk3-telemetry` also starts from `mtk3`.
LINE, SEARCH, SCAN and DETOUR start from
`mtk3-vehicle`; only `mtk3-staging` combines them. Independent module tests
are in [`host-tests/`](../host-tests/); run all host tests with
[`tests/CMakeLists.txt`](../tests/CMakeLists.txt). The template examples are removed. The motion branch provides the first
kernel task. Staging now connects LINE/SEARCH/SCAN/DETOUR through the
vehicle task described below.

- [Course template layout checks](MTK3_LAYOUT_CHECKS.md)

## Kernel platform

- [`app_program/platform/drv_gpio_irq.c`](../app_program/platform/drv_gpio_irq.c) — kernel-owned GPIO dispatcher, core 0, IRQ 13 at priority 1.
- [`app_program/platform/drv_time.c`](../app_program/platform/drv_time.c) — cold-power tick initialization, microsecond/millisecond clock and TIMER-to-kernel rate check.
- [`tests/test_drv_time.c`](../tests/test_drv_time.c), with [`tests/mtk_stubs/tk/tkernel.h`](../tests/mtk_stubs/tk/tkernel.h) — host doubles for clock ratios and errors.
- [`testing/subsystem1/TEST_LOG.md`](../testing/subsystem1/TEST_LOG.md) — shared platform S0-U01 cold-power clock test, run by the S1 TEST hook.
- Corresponding `*_hw.c` files include SDK headers only; kernel-side files include no SDK headers.
- The template demo tasks and their hardware tests are removed. GP19 is free.

## Motion kernel task

- [`app_program/subsystem2/motion.c`](../app_program/subsystem2/motion.c) and [`motion.h`](../app_program/subsystem2/motion.h) — controller task and inherited-mutex API; TEST-only independent timing window and buffered lifetime probes.
- [`drv_motor_hw.c`](../app_program/subsystem2/drv_motor_hw.c) — SDK-only PWM, encoder inputs and buttons.
- [`motion_bench.c`](../app_program/subsystem2/motion_bench.c) — RAM log and paced USB dump; not started in TEST builds.
- [`motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h) — unchanged robot settings, moved from `src/`.
- [`testing/subsystem2/TEST_LOG.md`](../testing/subsystem2/TEST_LOG.md) — U01–U04 and B4; operator post-flash passes, lifted/floor distinction and cold-power TIMER fault.
- [`tests/test_motion_task.c`](../tests/test_motion_task.c), [`tests/test_gpio_irq.c`](../tests/test_gpio_irq.c), and [`tests/mtk_stubs/`](../tests/mtk_stubs/) — strict host tests with kernel/hardware doubles.
- [`tests/test_test_runner.c`](../tests/test_test_runner.c) and [`test_timing_test_hooks.c`](../tests/test_timing_test_hooks.c) — TEST priority/failure policy, isolated S2 thresholds, deferred PROBE A output and retained lifetime evidence.

## Application SDK build hook

- [`build_make/app_sdk_hw.mk`](../build_make/app_sdk_hw.mk) — explicit SDK hardware header/source subset; no SDK startup/runtime.
- [`build_make/mtkernel_3/app_program/subdir.mk`](../build_make/mtkernel_3/app_program/subdir.mk) — application hook that includes the fragment.
- [Motor controller — SDK subset](MOTOR_CONTROLLER.md#sdk-hardware-subset) — exhaustive paths, sources and APIs that must not be used.

- [`tests/probes/sdk_hw_link_probe.c`](../tests/probes/sdk_hw_link_probe.c) and [`scripts/check-sdk-hw-link.py`](../scripts/check-sdk-hw-link.py) — retained SDK hardware link proof, never run or flashed.

- [μT-Kernel motion bring-up checks](MTK3_MOTION_CHECKS.md) — candidate revisions, SDK link proof, sizes, IRQ priority and hardware checks still not run.

## Kernel line acquisition

- [`app_program/subsystem3/drv_line.c`](../app_program/subsystem3/drv_line.c) — core-0 1 ms TIMER alarm 0 queue and task wake; SDK-only `drv_line_hw.c` averages eight reads per L/C/barcode channel; ISR timing counters.
- [`tests/test_drv_line.c`](../tests/test_drv_line.c) — scheduling, queue, drop and error doubles.
- [`testing/subsystem3/TEST_LOG.md`](../testing/subsystem3/TEST_LOG.md) — S3-U01 rate and range test.

## Kernel ultrasonic acquisition

- [`app_program/subsystem5/drv_ultrasonic.c`](../app_program/subsystem5/drv_ultrasonic.c) — core-0 echo state machine with GPIO13 edge dispatch and TIMER timestamps; SDK-only `drv_ultrasonic_hw.c` drives GP3.
- [`tests/test_drv_ultrasonic.c`](../tests/test_drv_ultrasonic.c) — edge, timeout, distance and fault doubles.
- [`testing/subsystem5/TEST_LOG.md`](../testing/subsystem5/TEST_LOG.md) — S5-U01 fixed-object checks.

## Kernel robot integration

- [`app_program/vehicle/vehicle_task.c`](../app_program/vehicle/vehicle_task.c) — priority-13 core-0 navigation: snapshots, spin behaviour, modules, buttons, motion/service/telemetry messages; deadline counter, no sensor polling or dump loop.
- [`app_program/vehicle/vehicle_robot_config.h`](../app_program/vehicle/vehicle_robot_config.h) — reference-extracted locked settings.
- [`app_program/vehicle/vehicle_log.c`](../app_program/vehicle/vehicle_log.c) and [`vehicle_log.h`](../app_program/vehicle/vehicle_log.h) — pure fixed 16-byte KR1 packing.
- [`scripts/robot_log_decode.py`](../scripts/robot_log_decode.py) — rows/events CSV decoder.
- [`tests/test_vehicle_task.c`](../tests/test_vehicle_task.c), [`test_vehicle_log.c`](../tests/test_vehicle_log.c) and [`test_robot_contract.py`](../tests/test_robot_contract.py) — glue, packing/decoder and reference-settings checks.
- [Kernel robot design](KERNEL_ROBOT.md) — five-task/object and plan-vs-port tables; freshness, deadlines, KR1 bits, software validation and pending hardware checks.
- [R5 procedure](../testing/integration/KERNEL_ROBOT.md) — pending track check, compare with bare-metal R4.

## Kernel hardware watchdog

- [`app_program/platform/drv_watchdog.c`](../app_program/platform/drv_watchdog.c) and [`drv_watchdog_hw.c`](../app_program/platform/drv_watchdog_hw.c) — kernel startup/feed guard and SDK-only hardware/reset snapshot.
- [`tests/test_watchdog.c`](../tests/test_watchdog.c), [`test_watchdog_hw.c`](../tests/test_watchdog_hw.c) and [`watchdog_stubs/hardware/watchdog.h`](../tests/watchdog_stubs/hardware/watchdog.h) — driver/register doubles.
- [Kernel hardware watchdog](KERNEL_WATCHDOG.md) — ownership, SDK subset, reset classification and two-boot S2-U05 procedure.

## Application task stacks

- [`app_program/platform/app_task.c`](../app_program/platform/app_task.c) — core-0 static user stacks and fill-pattern high-water report.
- [Application task stacks](APPLICATION_TASKS.md) — sizes, ownership and measurement limits.

- [`app_program/subsystem3/line_service.c`](../app_program/subsystem3/line_service.c) — priority-11 per-sample line task, inherited-mutex snapshot and calibration owner; `raw_bc` is sampled only.
- [`app_program/subsystem5/sonar_service.c`](../app_program/subsystem5/sonar_service.c) — priority-12 request/echo service: MBF requests, inherited-mutex result, result event flag and 10 ms deadline cyclic.
## Kernel telemetry

- [`app_program/subsystem1/telemetry.c`](../app_program/subsystem1/telemetry.c) — priority-30 RAM log/dump owner; value messages via `mbf_telemetry`, `TMO_POL` producer sends.
- [Kernel telemetry](KERNEL_TELEMETRY.md) — RAM ownership, local loss policy and current MQTT boundary.

## Approved template deviations

- [Template change ledger](TEMPLATE_CHANGES.md) — shared startup and Wi-Fi changes; template source is not frozen.
- [`scripts/check-template-changes.py`](../scripts/check-template-changes.py) — verify the complete recorded deviation file list against `c3b0954`.

## STEP 8 MQTT navigation

- [Timing investigation](MOTION_TIMING_INVESTIGATION.md) and [`motion_diag.h`](../app_program/subsystem2/motion_diag.h) / [`motion_diag_capture.h`](../app_program/subsystem2/motion_diag_capture.h) — opt-in `DIAG=1` fixed timing snapshots, GP20-only output and [`host check`](../tests/test_motion_diag.c); no scheduler fix. [Hardware results](MTK3_STEP8_HARDWARE_RESULTS.md) remain tied to the tested earlier image.

- [`boot_id.c`](../app_program/platform/boot_id.c), [`boot_id_hw.c`](../app_program/platform/boot_id_hw.c), and [`boot_entropy.c`](../app_program/platform/boot_entropy.c) — bounded ROSC per-boot observation label and pure bit extraction; [`test_boot_id.c`](../tests/test_boot_id.c). Binary two-boot fixtures: [`mqtt_boot_fixtures.h`](../tests/mqtt_boot_fixtures.h).

- [Follow-up handoff](MTK3_FOLLOWUP_HANDOFF.md) — final owner tips, commit sequence, helper cleanup/recovery and hardware limits.

- [`obstacle_report.c`](../app_program/vehicle/obstacle_report.c) — pure completed-scan report; [`test_obstacle_report.c`](../tests/test_obstacle_report.c).
- [`odometry.c`](../app_program/vehicle/odometry.c) and [`run_stats.c`](../app_program/vehicle/run_stats.c) — encoder geometry, read-only run/detour summaries; [`test_odometry.c`](../tests/test_odometry.c), [`test_run_stats.c`](../tests/test_run_stats.c).
- [`idle_meter.c`](../app_program/platform/idle_meter.c) and [`cpu_load.c`](../app_program/platform/cpu_load.c) — calibrated lowest-priority CPU meter and pure arithmetic; [`test_cpu_load.c`](../tests/test_cpu_load.c), [CPU load record](CPU_LOAD.md).

- [Design](KERNEL_MQTT.md), [wire layout](MQTT_WIRE.md), [operator runbook](MQTT_RUNBOOK.md).
- [`app_program/subsystem1/robot_mqtt.c`](../app_program/subsystem1/robot_mqtt.c) — strong Wi-Fi-task hook, retry/subscription/publish and the sink for `log_stream`.
- [`log_stream.c`](../app_program/subsystem1/log_stream.c) — live RAM-log stream (QoS 0, jump-to-newest, data age) and batched QoS 1 DUMP refill with ACK cursor; no lwIP dependency.
- [`robot_network.h`](../app_program/subsystem1/robot_network.h), [`network_queue.c`](../app_program/subsystem1/network_queue.c) — bounded live events and ordinary/ESTOP command queues, losses and provider contract.
- [`robot_health.c`](../app_program/subsystem1/robot_health.c) — priority31 cyclic producer, app-stack scan and bounded health board.
- [`mqtt_wire.c`](../app_program/subsystem1/mqtt_wire.c), [`scripts/robot_wire.py`](../scripts/robot_wire.py), [`scripts/mqtt_robot.py`](../scripts/mqtt_robot.py) — explicit binary codec, pure Python decoder with log merge/gap store (no Paho), and Paho2 VERSION2 receiver/CSV/commands.
- [`robot_mqtt_secrets.h.example`](../app_program/subsystem1/robot_mqtt_secrets.h.example) — app-owned private-header example; do not edit the template config example.
- [`vehicle_board.h`](../app_program/vehicle/vehicle_board.h), [`vehicle_mqtt.c`](../app_program/vehicle/vehicle_mqtt.c), [`vehicle_network_policy.c`](../app_program/vehicle/vehicle_network_policy.c) — vehicle read board, bounded cross-board adapters and idle/command policy.
- [`scripts/mqtt_dashboard.py`](../scripts/mqtt_dashboard.py), [`dashboard_model.py`](../scripts/dashboard_model.py), [`dashboard_page.py`](../scripts/dashboard_page.py) — laptop-only live dashboard ([runbook](MQTT_DASHBOARD.md)): HTTP/SSE server with CSRF/Origin/Host checks, pure panel and button-gating model, inline page; it uses `robot_wire.py` and never sends the token to the browser.
- [`tests/mqtt_protocol.cmake`](../tests/mqtt_protocol.cmake), [`dashboard_checks.cmake`](../tests/dashboard_checks.cmake), [`network_checks.cmake`](../tests/network_checks.cmake), [`mqtt_client_checks.cmake`](../tests/mqtt_client_checks.cmake) — C/Python packing, board/queue/log and mocked transport regression tests.
- [`tests/test_sonar_history.c`](../tests/test_sonar_history.c) — non-consuming latest/history reads, overflow/miss and sequence wrap.

- [`app_program/platform/app_board.h`](../app_program/platform/app_board.h) — single-writer atomic word publication and one-attempt core-0 reads; unavailable never spins.
- [`tests/test_template_tick.c`](../tests/test_template_tick.c) and [`tests/test_template_wifi.c`](../tests/test_template_wifi.c) — startup register/idempotence/bounded-failure and Wi-Fi priority/lifetime/window/ABI doubles; [`tests/template_checks.cmake`](../tests/template_checks.cmake).

- Motion public `motion_snapshot_try_read` is a transport-independent last-tick board, not the inherited control mutex. Tests in [`tests/test_motion_task.c`](../tests/test_motion_task.c) cover null/unready/no-first-tick and coherent counts/time.


- Line public `line_snapshot_try_read` is a transport-independent bounded board, never an event consumer or mutex waiter. [`tests/test_line_service.c`](../tests/test_line_service.c) checks unready/null, coherent sample identity and non-consumption.

- [`tests/vehicle_network_checks.cmake`](../tests/vehicle_network_checks.cmake) — adapter age/wrap/busy, exhaustive command policy, and real vehicle task non-critical queue/DUMP/ESTOP tests.
- [STEP 8 validation](MTK3_STEP8_VALIDATION.md) — clean build/strict/link/host results, ELF bindings, memory limits and pending physical acceptance.
- [`tests/test_log_stream.c`](../tests/test_log_stream.c) — live batching/jump/generation/failure and DUMP window/ACK/rewind/abort/throughput tests.
