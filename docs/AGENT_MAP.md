# Agent Navigation Map — INF2004 Embedded Systems

Use this map before editing an unfamiliar part of the repository.

## Map chain

| Scope | Map or rule |
| --- | --- |
| Current repository | This file: `docs/AGENT_MAP.md` |
| Parent workspace | [School Repos map](https://github.com/Agoace/School-Repos/blob/main/docs/AGENT_MAP.md) |

The parent-workspace link opens the School Repos map on GitHub, so it also works in a standalone worktree.

## Documentation locations

Only `README.md` and this map stay at the `docs/` root. The index lists each
record once. Use these topic directories before choosing a source entry below:

| Directory | Scope |
| --- | --- |
| [release/](release/) | DEMO_FREEZE.md: frozen demo kernel (tag, build line, checks, robot evidence, known limits). |
| [project-requirements/](project-requirements/) | Only our summary is tracked; course originals remain local. |
| [architecture/](architecture/) | PROJECT_STATUS (areas, branches: main, mtk3-staging, mtk3-testing), ARCHITECTURE (system and three ADRs), MEASUREMENTS, CR1, junction action/adapter, template ledger. |
| [kernel/](kernel/) | KERNEL.md (tasks, timing, RAM, locks), BUILD.md (build, flash, bench, SWD), JUNCTION_VALIDATION.md. |
| [s1-telemetry/](s1-telemetry/) | TELEMETRY.md (design and operation: broker, recorder, dashboard) and MQTT_WIRE.md (bytes). |
| [s2-motion/](s2-motion/) | MOTION.md (controller, pins, calibration, duty map, results). |
| [s3-line-barcode/](s3-line-barcode/) | LINE/SEARCH, TEST barcode B1c, B2b SHADOW live reader and results, and analog barcode ideas; originals in archive/s3-line-barcode/. |
| [s4-imu/](s4-imu/) | Robot/TEST IMU, calibration/filter/height formulas, software evidence and retained raw-stream history; IMU_SAFETY_PORT.md tracks the manual UP safety port. |
| [s5-obstacle/](s5-obstacle/) | OBSTACLE.md: pipeline, coarse/fine geometry and sonar service; COARSE_FINE_VALIDATION.md: software proof, scoped robot floor result and remaining limits; history in archive/s5-obstacle/HISTORY.md. |
| [simulation/](simulation/) | Current host simulator summary, explicit model assumptions and revision-scoped echo/sweep results; figure-8 demo course. |
| [archive/](archive/) | `HISTORY.md` pointers per area (`git show 569672a:<path>`), records that code still cites, and line/barcode/sim originals. |

The detailed entries below retain source and test navigation. Archive links do
not imply current hardware acceptance. Follow the root README for current
robot orientation; [kernel/BUILD.md](kernel/BUILD.md) is the current Make guide.
The [kernel history](archive/kernel/HISTORY.md#files) keeps the older CMake bench examples.

| Content | Location |
| --- | --- |
| Documentation index | [docs/README.md](README.md) |
| Demo freeze (2026-11-18) | [docs/release/DEMO_FREEZE.md](release/DEMO_FREEZE.md) — tag `demo-kernel-v1`, demo build line, checks and limits |
| Project status (read first) | [docs/architecture/PROJECT_STATUS.md](architecture/PROJECT_STATUS.md) — state, evidence and open items per area; branch layout and merge rules |
| Measured and assumed values | [docs/architecture/MEASUREMENTS.md](architecture/MEASUREMENTS.md) — every physical/timing number with date, label and source; check before asking the operator |
| Course requirements | [Our summary](project-requirements/README.md); original PDFs/Word files are local-only and ignored. |
| Repository instructions | [AGENTS.md](../AGENTS.md) |
| Public orientation and setup | [README.md](../README.md) |
| Development setup | [docs/kernel/BUILD.md](kernel/BUILD.md) |
| SWD debug probe | [docs/kernel/BUILD.md](kernel/BUILD.md#swd-debug-probe) — wiring, safety, memory reads, GDB on both cores |
| Interrupt priorities | [docs/kernel/KERNEL.md](kernel/KERNEL.md#interrupt-priorities-f-nvic) — F-NVIC: measured IPR, all external IRQs at 0 |
| Calibration record | [docs/s2-motion/MOTION.md](s2-motion/MOTION.md#hardware-wiring-and-calibration) |
| Motor rotation drift record | [docs/s2-motion/MOTION.md](s2-motion/MOTION.md#encoder-ratio-and-drift-method) |
| Current architecture | [ARCHITECTURE.md](architecture/ARCHITECTURE.md) — tasks/cores, interrupt/polling rules and shell routes; [`vehicle_sm.c`](../app_program/vehicle/vehicle_sm.c), [`test_vehicle_sm.c`](../tests/test_vehicle_sm.c). |
| Obsolete USB power safety (archive only) | [docs/archive/s2-motion/USB_POWER_SAFETY.md](archive/s2-motion/USB_POWER_SAFETY.md) |
| Archived UDP bench dashboard architecture | [telemetry history](archive/s1-telemetry/HISTORY.md#files) |
| Live MQTT dashboard (laptop) | [docs/s1-telemetry/TELEMETRY.md](s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard) |
| Week 8 Change Requirement 1 notes | [architecture history](archive/architecture/HISTORY.md#files); summary in [CR1.md](architecture/CR1.md#requirement) |
| IMU safety manual port | [IMU_SAFETY_PORT.md](s4-imu/IMU_SAFETY_PORT.md) — pure [`imu_safety.c`](../app_program/subsystem4/imu_safety.c), [`imu_safety.h`](../app_program/subsystem4/imu_safety.h), synthetic [`test_imu_safety.c`](../tests/test_imu_safety.c), pure Vehicle gate [`imu_gate.c`](../app_program/vehicle/imu_gate.c)/[`imu_gate.h`](../app_program/vehicle/imu_gate.h), receipt/recovery [`test_imu_gate.c`](../tests/test_imu_gate.c), and bounded service boards in [`imu_service.c`](../app_program/subsystem4/imu_service.c). |
| IMU safety synthetic bench | [IMU_SAFETY_BENCH.md](s4-imu/IMU_SAFETY_BENCH.md) — opt-in UP profiles `TEST=1 IMU_SAFETY_BENCH=1` (TEST runner, BENCH2) and `IMU_SAFETY_BENCH=robot_synthetic` (full robot app, BENCH3): [`imu_safety_bench/`](../app_program/imu_safety_bench) (TEST and ROBOT replay, per-phase and per-section stats, forced-include seams, TEST reporter `bench.c`, ROBOT reporter `robot.c`), [`imu_safety_bench.mk`](../build_make/imu_safety_bench.mk), [`imu_safety_bench.list`](../tools/kernel_ram/imu_safety_bench.list); tests [`test_imu_replay.c`](../tests/test_imu_replay.c), [`test_imu_replay_service.c`](../tests/test_imu_replay_service.c) (TEST and ROBOT builds), [`test_imu_safety_bench_make.py`](../tests/test_imu_safety_bench_make.py). |
| IMU safety latch capture | [IMU_SAFETY_CAPTURE.md](s4-imu/IMU_SAFETY_CAPTURE.md) — opt-in ROBOT profile `IMU_SAFETY_CAPTURE=1` (combines with `BARCODE_FAIL_CAPTURE=1`): [`imu_safety_capture/`](../app_program/imu_safety_capture) (pure ring `capture.c`, forced-include seams on `imu_service.o`, `telemetry.o` and `vehicle_task.o`, `hook.c` wrappers and GP20 printer), [`imu_safety_capture.mk`](../build_make/imu_safety_capture.mk); tests [`test_imu_safety_capture.c`](../tests/test_imu_safety_capture.c), [`test_imu_safety_capture_make.py`](../tests/test_imu_safety_capture_make.py). |
| Current kernel ultrasonic driver/service | [OBSTACLE.md](s5-obstacle/OBSTACLE.md#ultrasonic-driver-and-service) — bounded request/echo contract, packed result and sensor limits. |
| Current obstacle pipeline | [OBSTACLE.md](s5-obstacle/OBSTACLE.md) — five brief stages; coarse/fine scan, 13-bin decision completion, estimated geometry, monitor/profile, reattempt and line recovery. |
| Coarse/fine proof and scoped robot floor result | [COARSE_FINE_VALIDATION.md](s5-obstacle/COARSE_FINE_VALIDATION.md) — [`test_scan_stages.c`](../tests/test_scan_stages.c), [`obstacle_report.c`](../app_program/vehicle/obstacle_report.c), [`test_obstacle_report.c`](../tests/test_obstacle_report.c); stopped `[S5_SCAN]` lines in [`telemetry.c`](../app_program/subsystem1/telemetry.c), wire v3 and legacy decoder [`robot_wire.py`](../scripts/robot_wire.py). |
| Obstacle bypass record | [docs/archive/OBSTACLE_BYPASS.md](archive/OBSTACLE_BYPASS.md) |
| Host simulation, echo fit and box sweep | [SIMULATION.md](simulation/SIMULATION.md) — current model/interfaces; historical v1.5 evidence is not the checkout's v1.8 target. [sim/README.md](../sim/README.md) gives build/schema/trace commands. Full originals stay in archive/simulation/. |
| Obstacle detour handoff | [docs/archive/OBSTACLE_DETOUR_HANDOFF.md](archive/OBSTACLE_DETOUR_HANDOFF.md) |
| Detour v1.6 test plan | [obstacle history](archive/s5-obstacle/HISTORY.md#files) |
| ADR: motor controller owns the actuators | [docs/architecture/ARCHITECTURE.md](architecture/ARCHITECTURE.md#adr-the-motor-controller-is-the-only-owner-of-the-actuators) |
| ADR: one hardware heartbeat | [docs/architecture/ARCHITECTURE.md](architecture/ARCHITECTURE.md#adr-one-hardware-heartbeat-short-work-in-the-interrupt-decisions-in-the-main-loop) |
| ADR: Vehicle Controller runs one module at a time | [docs/architecture/ARCHITECTURE.md](architecture/ARCHITECTURE.md#adr-the-vehicle-controller-runs-one-module-at-a-time-and-maps-outcomes-to-the-next-module) |
| Current kernel LINE/SEARCH | [LINE.md](s3-line-barcode/LINE.md) — pair detection, retained P7 mm/s settings, search and evidence; modules [`app_program/subsystem3/line_trace.c`](../app_program/subsystem3/line_trace.c), [`app_program/subsystem3/line_search.c`](../app_program/subsystem3/line_search.c); target [`src/vc_line_test.c`](../src/vc_line_test.c) |
| SCAN/DETOUR modules and scoped physical results | [OBSTACLE.md](s5-obstacle/OBSTACLE.md) — modules [`app_program/subsystem5/servo_scan.c`](../app_program/subsystem5/servo_scan.c), [`app_program/subsystem5/detour_run.c`](../app_program/subsystem5/detour_run.c); test [`tests/test_obstacle_modules.c`](../tests/test_obstacle_modules.c) |
| Line steering in mm/s | [docs/s3-line-barcode/LINE.md](s3-line-barcode/LINE.md) — `steer_mm_*` in [`app_program/subsystem3/line_steer.c`](../app_program/subsystem3/line_steer.c) |
| Duty map record | [docs/s2-motion/MOTION.md](s2-motion/MOTION.md) — bench [`src/duty_map_bench.c`](../src/duty_map_bench.c) |
| Heartbeat record | [docs/s2-motion/MOTION.md](s2-motion/MOTION.md) — core [`src/heartbeat.c`](../src/heartbeat.c), Pico layer [`src/heartbeat_pico.c`](../src/heartbeat_pico.c), bench [`src/heartbeat_test.c`](../src/heartbeat_test.c), test [`tests/test_heartbeat.c`](../tests/test_heartbeat.c) |
| Motor controller record | [docs/s2-motion/MOTION.md](s2-motion/MOTION.md) — core [`app_program/subsystem2/motor_ctl.c`](../app_program/subsystem2/motor_ctl.c), Pico layer [`src/motor_ctl_pico.c`](../src/motor_ctl_pico.c), floor bench [`src/motor_ctl_bench.c`](../src/motor_ctl_bench.c), settings [`app_program/subsystem2/motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h), test [`tests/test_motor_ctl.c`](../tests/test_motor_ctl.c) |
| Superseded preliminary Vehicle Controller record | [docs/archive/VEHICLE_CONTROLLER.md](archive/VEHICLE_CONTROLLER.md) |
| Steering Wi-Fi log | [docs/archive/STEER_WIFI_LOG.md](archive/STEER_WIFI_LOG.md) |
| Week 6 design-review diagrams | [`design-review/`](../design-review) — Mermaid sources and rendered images for the §5 system statechart and §4 task interaction diagram; render with `npx -y @mermaid-js/mermaid-cli -i <file>.mmd -o <file>.png -b white --size 3000` |
| Parent checkout ledger | [School Repos checkout ledger](https://github.com/Agoace/School-Repos/blob/main/docs/README.md#coursework-checkout-ledger) |

## Source groups: what each does

One line per directory or file group. Checked against the tree at the
commit that changed this section (header comments and file lists).

| Group | What it does |
| --- | --- |
| [`app_program/app_main.c`](../app_program/app_main.c), [`app.h`](../app_program/app.h), [`test_report.h`](../app_program/test_report.h) | Application entry from `usermain`: starts subsystems and the Vehicle task; shared build switches; TEST report helpers. |
| [`app_program/platform/`](../app_program/platform) | Kernel-facing services: 1 µs time, GPIO IRQ dispatcher (core 0), watchdog, boards (bounded publication), task registry with static stacks, startup handoff, boot ID/entropy, CPU/idle meters, P1 run timing and lock stats, UP SMP shim. |
| [`app_program/subsystem1/`](../app_program/subsystem1) | S1 telemetry: sole Wi-Fi/MQTT owner, binary wire codec (`mqtt_wire`), network queue, RUN/IDLE rates, RAM log stream and DUMP, Health task. |
| [`app_program/subsystem2/`](../app_program/subsystem2) | S2 motion: motion task (only PWM writer), pure motor controller and robot settings, encoder/PWM hardware, packed 60 B status board, opt-in timing diagnostics, BENCH bench. |
| [`app_program/subsystem3/`](../app_program/subsystem3) | S3 line and barcode: 1 ms ADC driver and line service, line position/steer/trace/search modules, TEST barcode (WIDTH, PEAK, capture, drive), B2b SHADOW live window reader, producer and source boards. |
| [`app_program/subsystem4/`](../app_program/subsystem4) | S4 IMU (GY-511 accelerometer, no gyro): I2C probe/decode, FIFO, roll and height core, report-only hump detector and diagnostics, IMU service task and TEST capture. |
| [`app_program/subsystem5/`](../app_program/subsystem5) | S5 obstacle: ultrasonic driver and sonar request service, forward watch, servo scan, profile analysis, detour modules. |
| [`app_program/vehicle/`](../app_program/vehicle) | Vehicle Controller: shell state machine, vehicle task, detector inputs, robot settings, junction action/live adapter/source, odometry, run/detour stats, obstacle report, vehicle log and MQTT glue. |
| [`app_program/bench_kernel/`](../app_program/bench_kernel) | `BENCH_KERNEL=1` spare-Pico bench: GPIO generator, motor/input stand-ins, load and idle meters, statistics, report and per-counter top lines. |
| [`src/`](../src) | Earlier bare-metal targets (SDK, no kernel): calibration and probe programs, bench tests (servo, ultrasonic, line, steer, scan, detour, bypass, duty map, heartbeat, motor), bare-metal drivers, Wi-Fi/UDP logs. The simulator compiles some of them. |
| [`tests/`](../tests) | Host unit test suite (CMake + ctest, strict warnings); one test per module plus fixtures. |
| [`host-tests/`](../host-tests) | Four standalone module tests (detour, line, scan, search). |
| [`build_make/`](../build_make) | Course-template Make build: `makefile`, board files, `pico_rp2040.mk`, `app_sdk_hw.mk` SDK subset hook, per-directory source lists under `mtkernel_3/`. |
| [`kernel/`](../kernel), [`lib/`](../lib), [`device/`](../device), [`include/`](../include), [`config/`](../config), [`etc/`](../etc) | μT-Kernel 3.0 course template and its libraries (`libtk`, `libtm` USB console, `libwifi` CYW43, `liblwip` MQTT, `libbsp`). Change only through [TEMPLATE_CHANGES.md](architecture/TEMPLATE_CHANGES.md) (8 deviations). |
| [`scripts/`](../scripts) | Checks (`check-docs.py`, `check-template-changes.py`, `check-sdk-hw-link.py`), hook installer, RAM and message-size reports, MQTT recorder/dashboard (`mqtt_robot.py`, `mqtt_dashboard.py`, `robot_wire.py`, `dashboard_*`), KR1 log decoder, motor bench report, old UDP receivers and dashboard, port restriction. |
| [`tools/`](../tools) | `barcode_map/` (letter map to `BW_MAP_VALUES`), `kernel_ram/` (RAM placement lists and `ram_proof.py`), `elf2uf2/`, `verify_phase11_usb.py`. |
| [`sim/`](../sim) | Separate host simulator of bare-metal sources: core clock/world/scenario/runner, devices, target hooks, scenarios JSON, offline viewer and local editor, box sweep, tests. |
| [`sim/webots/`](../sim/webots) | Figure-8 demo course: tile map, course JSON, map-to-JSON tool and its tests. The Webots generator itself is on `mtk3-hil` (SW8). |
| [`testing/`](../testing) | Operator test logs per subsystem, integration (`KERNEL_ROBOT.md`) and system; `logs/` placeholders. |
| [`design-review/`](../design-review) | Week 6 Mermaid sources and images: system statechart, task interaction. |
| [`videos/`](../videos) | README for hardware test and demo videos. |
| [`.githooks/`](../.githooks) | Shared pre-commit hook: runs `check-docs.py` when Markdown, docs or hook files are staged. |

## CR1 measurement and planning

- [CR1.md](architecture/CR1.md) records the `168d705`
  baseline, ARM message sizes and the single-writer publication rule.
- [Motion module plan](architecture/CR1.md#motion-module-plan) records approved A1 packing;
  controller ownership changes and derived-copy removal remain separate.
- [`motion_pack.h`](../app_program/subsystem2/motion_pack.h) and
  [`motion_pack.c`](../app_program/subsystem2/motion_pack.c) define the
  60 B motion payload, published in a 64 B sequence/word board. Its sole
  writer is motion; decoded public APIs are unchanged.
- [Vehicle A1](architecture/CR1_VEHICLE_A1.md) records the observation-only
  board change (44 to 40 B), unchanged public API and wire, and software
  evidence. [`vehicle_pack.h`](../app_program/vehicle/vehicle_pack.h) and
  [`vehicle_pack.c`](../app_program/vehicle/vehicle_pack.c) define its nine-word
  payload; [`test_vehicle_pack.c`](../tests/test_vehicle_pack.c) checks
  ranges, invalid codes and interrupted publication. Vehicle is sole writer.
- [Sonar/obstacle A](architecture/CR1_SONAR_A.md) records the sonar board,
  history and obstacle board change. [`sonar_pack.h`](../app_program/subsystem5/sonar_pack.h)
  and [`obstacle_pack.h`](../app_program/vehicle/obstacle_pack.h) define the
  layouts; [`test_sonar_pack.c`](../tests/test_sonar_pack.c) and
  [`test_obstacle_pack.c`](../tests/test_obstacle_pack.c) check them.
- [Telemetry A, faults and T110](architecture/CR1_TELEMETRY_A.md) records
  the approved S1 storage work, diagnostic owner masks and 110 ms RUN state
  deadline. [`test_telemetry_mode.c`](../tests/test_telemetry_mode.c) checks
  eligibility, unchanged modes and the 20 ms polling model.
  [`fault_mask.h`](../app_program/platform/fault_mask.h) defines the diagnostic
  single-writer owner board and Health-only combiner interface;
  [`test_fault_mask.c`](../tests/test_fault_mask.c) checks ownership, interrupted
  reads, reserved bits and run-epoch resets. Publishers are separate changes.
  [`telemetry_pack.h`](../app_program/subsystem1/telemetry_pack.h) defines
  private retained RAM codecs, not MQTT formats;
  [`test_telemetry_pack.c`](../tests/test_telemetry_pack.c) checks field ranges,
  byte layouts, spare bits and unchanged outputs on invalid input.
  [`network_pack.h`](../app_program/subsystem1/network_pack.h) defines private
  Health and pending-sonar codecs; [`test_telemetry_optional_pack.c`](../tests/test_telemetry_optional_pack.c)
  checks them and the INFO/scan codecs. [`value_bits.h`](../app_program/platform/value_bits.h)
  provides local byte operations, not shared atomics.
- [Barcode A](architecture/CR1_BARCODE_A.md) records the private barcode
  evidence change. [`barcode_evidence_pack.h`](../app_program/subsystem3/barcode_evidence_pack.h)
  defines the layouts; [`test_barcode_evidence_pack.c`](../tests/test_barcode_evidence_pack.c)
  checks them.
- [`test_motion_pack.c`](../tests/test_motion_pack.c) checks ranges,
  reserved codes and interrupted compact-board reads.
- [`ram_report.py`](../scripts/ram_report.py) reads linked RAM sections;
  [`test_ram_report.py`](../tests/test_ram_report.py) tests its parser.
- [`message_sizes.py`](../scripts/message_sizes.py) compiles an ARM size
  probe for public boards, queue values and MQTT payload constants.
  Update its manifest when an interface is added.

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
| Earlier bench telemetry transport | [`src/telemetry.c`](../src/telemetry.c), [`src/telemetry.h`](../src/telemetry.h) |
| Earlier UDP Wi-Fi telemetry probe | [`src/wifi_telemetry_probe.c`](../src/wifi_telemetry_probe.c) |
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
| Line steering | [`app_program/subsystem3/line_steer.c`](../app_program/subsystem3/line_steer.c), [`app_program/subsystem3/line_steer.h`](../app_program/subsystem3/line_steer.h); bench test [`src/steer_test.c`](../src/steer_test.c) |
| Steering Wi-Fi log | [`src/wifi_log.c`](../src/wifi_log.c), packet format [`src/steer_log.c`](../src/steer_log.c); receiver [`scripts/steer_log_receiver.py`](../scripts/steer_log_receiver.py) |
| Drive, IR reads, spin calibration (bench parts) | [`src/drive.c`](../src/drive.c), [`src/line_sensors.c`](../src/line_sensors.c), [`src/line_spin.c`](../src/line_spin.c) |
| Earlier bare-metal Vehicle Controller | [`src/vehicle_controller.c`](../src/vehicle_controller.c), [`app_program/vehicle/vehicle_controller.h`](../app_program/vehicle/vehicle_controller.h) |
| Servo driver (clamped) | [`src/servo.c`](../src/servo.c), [`src/servo.h`](../src/servo.h) |
| Servo scan bench test | [`src/scan_test.c`](../src/scan_test.c) |
| Host unit tests | [`tests/`](../tests) — run without hardware; see [obstacle history](archive/s5-obstacle/HISTORY.md#files), [OBSTACLE_BYPASS.md](archive/OBSTACLE_BYPASS.md#host-unit-test), [VEHICLE_CONTROLLER.md](archive/VEHICLE_CONTROLLER.md#host-unit-test), [LINE_POSITION.md](archive/s3-line-barcode/LINE_POSITION.md#host-unit-test), [LINE_STEERING.md](archive/s3-line-barcode/LINE_STEERING.md#host-unit-test), [motion history](archive/s2-motion/HISTORY.md#files), [architecture history](archive/architecture/HISTORY.md#files), and [STEER_WIFI_LOG.md](archive/STEER_WIFI_LOG.md#host-tests) |
| Legacy IR line-data utilities | [`src/line_detector.c`](../src/line_detector.c), [`src/ir_line_data.c`](../src/ir_line_data.c); host test [`tests/test_ir_line_data.c`](../tests/test_ir_line_data.c) |
| Motor bench report | [`scripts/motor_bench_report.py`](../scripts/motor_bench_report.py) — reads a `motor_ctl_bench` text log |
| Earlier bench telemetry receiver | [`scripts/telemetry_receiver.py`](../scripts/telemetry_receiver.py) |
| Earlier bench telemetry dashboard | [`scripts/telemetry_dashboard.py`](../scripts/telemetry_dashboard.py) |
| Telemetry JSON schema (old) | [telemetry history](archive/s1-telemetry/HISTORY.md#files) |
| Archived control proposal, not current ownership | [`CONTROL_ARCHITECTURE.md` in architecture history](archive/architecture/HISTORY.md#files) |
| Barcode B1c | [BARCODE.md](s3-line-barcode/BARCODE.md); TEST only. |
| Telemetry port restriction | [`scripts/restrict-telemetry-port.sh`](../scripts/restrict-telemetry-port.sh) |
| Separate host simulator | [`sim/CMakeLists.txt`](../sim/CMakeLists.txt), [`sim/targets/detour_test.cmake`](../sim/targets/detour_test.cmake) — host-only source selection; reads firmware without changing it |
| Simulator clock, scenario, and trace | [`sim/core/runtime.c`](../sim/core/runtime.c), [`sim/core/scenario.py`](../sim/core/scenario.py), [`sim/core/protocol.py`](../sim/core/protocol.py), [`sim/core/runner.py`](../sim/core/runner.py) |
| Simulator motion, contact safety, and IR | [`sim/core/world.c`](../sim/core/world.c), [`sim/include/sim_world.h`](../sim/include/sim_world.h) — calibrated curves, coast, fractional counts, tile sensing, and conservative contact stop, and ideal/specular sector echoes |
| Simulator devices and SDK headers | [`sim/devices/placeholders.c`](../sim/devices/placeholders.c), [`sim/devices/ultrasonic.c`](../sim/devices/ultrasonic.c), [`sim/include/sim_devices.h`](../sim/include/sim_devices.h), [`sim/include/`](../sim/include) — motion/IR/servo, timed echo/fault events, local sink, and minimal host SDK surface |
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

See [Branch layout](architecture/PROJECT_STATUS.md#branch-layout-and-integration-rules) for the active branches, merge rules and the [archive tags](architecture/PROJECT_STATUS.md#archived-branches) of deleted branches.

The [on-chip kernel bench section](kernel/BUILD.md#kernel-bench-image-bench_kernel) covers
[`bench_stats.c`](../app_program/bench_kernel/bench_stats.c), its
[public read API](../app_program/bench_kernel/bench_stats.h),
[core0 publication API](../app_program/bench_kernel/bench_stats_publish.h) and
[host test](../tests/test_bench_kernel_stats.c), plus the
[`[bench]` report](../app_program/bench_kernel/bench_report.c) and its
[host test](../tests/test_bench_report.c). The bench directory is excluded
from ordinary source selection. The
[step-0 procedure](kernel/BUILD.md#kernel-bench-image-bench_kernel) covers the bench-only
[`bench_generator.c`](../app_program/bench_kernel/bench_generator.c),
[`bench_runtime.c`](../app_program/bench_kernel/bench_runtime.c),
[`bench_timer_hw.c`](../app_program/bench_kernel/bench_timer_hw.c),
[`bench_motor_hw.c`](../app_program/bench_kernel/bench_motor_hw.c) and
[`bench_inputs_hw.c`](../app_program/bench_kernel/bench_inputs_hw.c).
The report reads the stopped native motion board for per-wheel comparison.
The [virtual GPIO state](../app_program/bench_kernel/bench_gpio.c) and
[hardware-provider seam](../app_program/bench_kernel/bench_gpio_hw.c) have a
[host test](../tests/test_bench_kernel_gpio.c); no native dispatcher is replaced.
The [encoder integration test](../tests/test_bench_kernel_encoder.c) connects
that real dispatcher to the real motion callback/controller with host scheduling.

Standalone module tests: [`host-tests/`](../host-tests), one C test per module.

- [Archived Robo Pico RTOS multicore plan](archive/architecture/HISTORY.md#files) — full historical proposal, not the current task/core contract.

## Course template layout

See [Branch layout](architecture/PROJECT_STATUS.md#mtk3-layout). Pure logic is in `app_program/`; bare-metal hardware code stays in `src/`; kernel platform register access uses isolated `*_hw.c` files. The template kernel and support code are in `kernel/`, `include/`, `lib/`, `device/`, `config/`, `etc/`, and `tools/` (except [`tools/kernel_ram/`](../tools/kernel_ram/), the opt-in [KERNEL_RAM](kernel/KERNEL.md#ram-placement-kernel_ram) lists and placement proof, and [`tools/barcode_map/`](../tools/barcode_map/), the `BARCODE_MAP` converter in [BUILD](kernel/BUILD.md)). Template build: [`build_make/makefile`](../build_make/makefile). Team ownership: [`TEAM.md`](../TEAM.md). Host test build: [`tests/CMakeLists.txt`](../tests/CMakeLists.txt).

The `mtk3` base contains shared pure helpers. `mtk3-motion` and
`mtk3-vehicle` start there. `mtk3-telemetry` also starts from `mtk3`.
LINE, SEARCH, SCAN and DETOUR start from
`mtk3-vehicle`; only `mtk3-staging` combines them. Independent module tests
are in [`host-tests/`](../host-tests); run all host tests with
[`tests/CMakeLists.txt`](../tests/CMakeLists.txt). The template examples are removed. The motion branch provides the first
kernel task. Staging now connects LINE/SEARCH/SCAN/DETOUR through the
vehicle task described below.

- [Course template layout checks (history)](archive/kernel/HISTORY.md#files)

## Kernel platform

- [`app_startup.h`](../app_program/platform/app_startup.h) and
  [`app_startup.c`](../app_program/platform/app_startup.c) — one-way boot
  handoff after all init/start attempts; the MQTT hook acquires before
  application access. [M8 contract](kernel/KERNEL.md#startup-handoff-m8)
  lists covered readers and unchanged runtime synchronization.
  [`test_app_startup.c`](../tests/test_app_startup.c),
  [`test_test_runner.c`](../tests/test_test_runner.c) and
  [`test_robot_mqtt.c`](../tests/test_robot_mqtt.c) check publication,
  lifecycle ordering and refusal of early application work.

- [`app_program/platform/drv_gpio_irq.c`](../app_program/platform/drv_gpio_irq.c) — kernel-owned GPIO dispatcher, core 0, IRQ 13 at requested priority 1 (0 on hardware, [F-NVIC](kernel/KERNEL.md#interrupt-priorities-f-nvic)).
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
- [`tests/test_motion_task.c`](../tests/test_motion_task.c), [`tests/test_gpio_irq.c`](../tests/test_gpio_irq.c), and [`tests/mtk_stubs/`](../tests/mtk_stubs) — strict host tests with kernel/hardware doubles.
- [`tests/test_test_runner.c`](../tests/test_test_runner.c) and [`test_timing_test_hooks.c`](../tests/test_timing_test_hooks.c) — TEST priority/failure policy, isolated S2 thresholds, deferred PROBE A output and retained lifetime evidence.

## Application SDK build hook

- [`build_make/app_sdk_hw.mk`](../build_make/app_sdk_hw.mk) — explicit SDK hardware header/source subset; no SDK startup/runtime.
- [`build_make/mtkernel_3/app_program/subdir.mk`](../build_make/mtkernel_3/app_program/subdir.mk) — application hook that includes the fragment.
- [Motor controller — SDK subset](s2-motion/MOTION.md#sdk-hardware-subset) — exhaustive paths, sources and APIs that must not be used.

- [`tests/probes/sdk_hw_link_probe.c`](../tests/probes/sdk_hw_link_probe.c) and [`scripts/check-sdk-hw-link.py`](../scripts/check-sdk-hw-link.py) — retained SDK hardware link proof, never run or flashed.

- [μT-Kernel motion bring-up checks (history)](archive/kernel/HISTORY.md#files) — removed; candidate revisions, SDK proof, sizes and then-open hardware checks.

## Kernel line acquisition

- [`app_program/subsystem3/drv_line.c`](../app_program/subsystem3/drv_line.c) — core-0 1 ms TIMER alarm 0 queue and task wake; SDK-only `drv_line_hw.c` averages eight reads per L/C/barcode channel; ISR timing counters.
- [`tests/test_drv_line.c`](../tests/test_drv_line.c) — scheduling, queue, drop and error doubles.
- [`testing/subsystem3/TEST_LOG.md`](../testing/subsystem3/TEST_LOG.md) — S3-U01 rate and range test.

- [GP28 spin evidence](s3-line-barcode/LIVE_CALIBRATION.md):
  [`barcode_calibration.h`](../app_program/subsystem3/barcode_calibration.h),
  [`barcode_calibration.c`](../app_program/subsystem3/barcode_calibration.c),
  the [`line_service.c`](../app_program/subsystem3/line_service.c) calibration
  board and [`test_barcode_calibration.c`](../tests/test_barcode_calibration.c)
  observe GP28 during the existing spin, with fallback 320/240.
  No live decoder or actions are added.

- [Live source contract](s3-line-barcode/LIVE_SOURCE_CONTRACT.md):
  [`barcode_live_source.h`](../app_program/subsystem3/barcode_live_source.h)
  and [`barcode_live_source.c`](../app_program/subsystem3/barcode_live_source.c)
  define the 40 B record and 20 B control, coherent getters and board ownership.
  [SHADOW producer](s3-line-barcode/LIVE_PRODUCER.md),
  [`barcode_live_producer.h`](../app_program/subsystem3/barcode_live_producer.h)
  and [`barcode_live_producer.c`](../app_program/subsystem3/barcode_live_producer.c)
  bind the LINE feed, context/ACK policy and stopped DUMP; live actions only in
  the opt-in build ([live command path](s3-line-barcode/BARCODE_LIVE_COMMAND.md),
  e2e test [`test_barcode_live_e2e.c`](../tests/test_barcode_live_e2e.c)).
  Live-only vehicle V1/V3 policy:
  [`policy.h`](../app_program/barcode_live_vehicle/policy.h) and
  [`policy.c`](../app_program/barcode_live_vehicle/policy.c),
  [`hook.c`](../app_program/barcode_live_vehicle/hook.c) owns the timer,
  [`action.c`](../app_program/barcode_live_vehicle/action.c) is the live-only
  centred-turn action copy, and
  [`line.c`](../app_program/barcode_live_vehicle/line.c) owns centre advance,
  [`live.c`](../app_program/barcode_live_vehicle/live.c) is the live adapter
  copy with SEARCH-only decode grace, and
  [`grace.h`](../app_program/barcode_live_vehicle/grace.h) /
  [`grace.c`](../app_program/barcode_live_vehicle/grace.c) provide the
  coherent Vehicle-to-LINE grace board,
  [`vehicle_impl.h`](../app_program/barcode_live_vehicle/vehicle_impl.h),
  [`print_impl.h`](../app_program/barcode_live_vehicle/print_impl.h), and
  [`make seams`](../build_make/barcode_live_vehicle.mk).
  The live-only [`control cache`](../app_program/barcode_live_vehicle/control.c)
  and [`interface`](../app_program/barcode_live_vehicle/control.h) use a
  [`producer seam`](../app_program/barcode_live_vehicle/producer_impl.h) to
  tolerate bounded control-board collisions. The shared source publication
  recheck remains uncached. Tests:
  [`test_barcode_control_cache.c`](../tests/test_barcode_control_cache.c). These expire a
  pending live command at 1000 mm from decode and cancel it after 200 ms of SEARCH;
  default builds omit this folder. Tests:
  [`test_blv_policy.c`](../tests/test_blv_policy.c) and
  [`test_barcode_live_vehicle_make.py`](../tests/test_barcode_live_vehicle_make.py).
  [SHADOW hardware results](s3-line-barcode/SHADOW_HARDWARE_RESULTS.md)
  record the first course run, GP28 geometry and open findings.
  [Live counts snapshot](s3-line-barcode/LIVE_COUNTS_SNAPSHOT.md) specifies
  `barcode_live_counts_try_read`, the read-only epoch terminal counts in
  `barcode_live_source.{h,c}`; absence, cancellation and coherent-read tests
  include [`test_barcode_live_counts_absent.c`](../tests/test_barcode_live_counts_absent.c).
  [Terminal-attempt telemetry](s3-line-barcode/BARCODE_ATTEMPT_TELEMETRY.md)
  links the LINE FIFO [`barcode_attempt.c`](../app_program/subsystem3/barcode_attempt.c)
  and [`barcode_attempt.h`](../app_program/subsystem3/barcode_attempt.h),
  Wi-Fi `robot/barcode` QoS1 service, v2 codec/CSV and early reason/frame dump.
  FIFO tests: [`test_barcode_attempt.c`](../tests/test_barcode_attempt.c);
  codec tests: [`test_barcode_wire.c`](../tests/test_barcode_wire.c) and
  [`test_barcode_v2.py`](../tests/test_barcode_v2.py).
  [School failed-start analysis](s3-line-barcode/SCHOOL_FAILED_START_ANALYSIS.md)
  records FC1/FC2 evidence and timing limits. The host validator is
  [`scripts/barcode_fail_analyze.py`](../scripts/barcode_fail_analyze.py), with
  [`test_barcode_fail_analyze.py`](../tests/test_barcode_fail_analyze.py).
  [Session report](s3-line-barcode/BARCODE_SESSION_REPORT.md):
  [`scripts/barcode_session_report.py`](../scripts/barcode_session_report.py)
  prints per-attempt results and D_cross from a GP20 log, with
  [`test_barcode_session_report.py`](../tests/test_barcode_session_report.py).
  [`test_barcode_quiet_real.c`](../tests/test_barcode_quiet_real.c) checks the
  three retained vectors and complete FC2 ADC trace. The older
  [`test_barcode_peak_real.c`](../tests/test_barcode_peak_real.c) also feeds
  its measured quiet distance bins through the live reader.
  [`test_barcode_motion_twitch.c`](../tests/test_barcode_motion_twitch.c)
  checks the 2.5 mm retreat hold, reverse, stall and FC1 KR twitch rows.
  [Opt-in failed-attempt capture](s3-line-barcode/FAILED_ATTEMPT_CAPTURE.md)
  uses [`barcode_fail_capture.h`](../app_program/subsystem3/barcode_fail_capture.h)
  and [`barcode_fail_capture.inc`](../app_program/subsystem3/barcode_fail_capture.inc),
  included only by the enabled source provider. Window/producer hooks record
  real failed inputs without a default-build object. Tests:
  [`test_barcode_fail_capture.c`](../tests/test_barcode_fail_capture.c),
  [`test_barcode_fail_make.py`](../tests/test_barcode_fail_make.py) and the
  capture-enabled [`feed fixture`](../tests/test_barcode_live_feed.c).

- [Size-tolerant course window reader](s3-line-barcode/LIVE_WINDOW_READER.md):
  [`barcode_window.h`](../app_program/subsystem3/barcode_window.h) and
  [`barcode_window.c`](../app_program/subsystem3/barcode_window.c) provide
  the pure forward-distance window and one mapping table; synthetic tests in
  [`test_barcode_window.c`](../tests/test_barcode_window.c), original-fixture
  tests in [`test_barcode_window_real.c`](../tests/test_barcode_window_real.c).
  [Real window replay](s3-line-barcode/REAL_WINDOW_REPLAY.md) compares old/new
  readers on all five retained ADC/motor fixtures.
  SHADOW feed tests: [`test_barcode_live_feed.c`](../tests/test_barcode_live_feed.c)
  and [`test_barcode_live_producer.c`](../tests/test_barcode_live_producer.c).
  Vehicle merge, stack/CPU and offset hardware acceptance remain pending.

## TEST-only barcode B1c

- [`barcode.c`](../app_program/subsystem3/barcode.c) and
  [`barcode.h`](../app_program/subsystem3/barcode.h) — pure incremental
  Code 39, both width methods/directions, command values only.
- [`barcode_peak.c`](../app_program/subsystem3/barcode_peak.c) and
  [`barcode_peak.h`](../app_program/subsystem3/barcode_peak.h) — separate
  bounded PEAK TIME/DISTANCE path, white-return lobes, framing-star bands,
  geometric spaces and conflict rejection. WIDTH remains unchanged.
- [`barcode_capture.c`](../app_program/subsystem3/barcode_capture.c) —
  packed/decimated ADC and motion capture; O(1) line-task copy and ranges.
- [`barcode_test.c`](../app_program/subsystem3/barcode_test.c) and
  [`barcode_hw.c`](../app_program/subsystem3/barcode_hw.c) — priority-20
  TEST observer; GP20 ranges, GP21 pass; checked stop/freeze then wait for
  terminal DTR + 1 s before report/dump, bounded coherent-board retries and
  detailed fault masks. [USB wait](s3-line-barcode/USB_CAPTURE_WAIT.md).
- [`barcode_drive.c`](../app_program/subsystem3/barcode_drive.c) and
  [`barcode_drive.h`](../app_program/subsystem3/barcode_drive.h) — pure
  forward/backward distance/time bounds and fault/start inhibition.
- [`barcode_settings.h`](../app_program/subsystem3/barcode_settings.h) —
  430 mm / 100 mm/s defaults; short-map target clamps to <=450 mm.
- [`tests/barcode_checks.cmake`](../tests/barcode_checks.cmake) — strict
  decoder, packed frames, capture, drive/limit, GPIO and observer tests;
  [`tests/barcode_stubs/`](../tests/barcode_stubs/) supplies core-0 doubles.
- [Real-capture regression](../tests/test_barcode_real.c) uses the
  [public-safe CSV](../tests/fixtures/barcode_large_f_forward.csv); both
  methods decode F from forward and reversed replay.
- [Blur cases](../tests/test_barcode_blur.c) test partial narrow peaks and
  bar/space bias for all 43 symbols; these are not measured small-print data.
- [PEAK units](../tests/test_barcode_peak.c) cover all 43 symbols,
  directions/sizes, noise, synthetic 18 mm line and failure/conflict guards.
  [Four real captures](../tests/test_barcode_peak_real.c) under
  [fixtures](../tests/fixtures/) cover large/small F, both native axes,
  raw/packed and reversed replay; all quiet crops must reject.
- [B1c contract](s3-line-barcode/BARCODE.md) and
  [S3 procedures/results](../testing/subsystem3/TEST_LOG.md) — B1 contrast
  passed; hand-push procedure failed. Lifted forward passed on 45040c7;
  0cd2c03 passed lifted/large F in both directions at 433 mm observed,
  but small F failed WIDTH. B1c PEAK has host evidence, not powered proof.
  The mount cannot go below 5.5 mm; it touches a 10 mm hump.
  Normal routing does not use the decoder.

## IMU task and hump report

- [GY-511 IMU record](s4-imu/IMU.md) — step 3 report-only hump detector (thresholds, gates, outputs, synthetic fixtures, H1 STREAM hardware PASS on 1480b37; diagnostic C hardware on 29d57de, uncorrected LINE height accepted, corrected over-read under investigation (9/9 humps, 0 false in 5 flat); full-circuit no-false-hump H3 is priority and Not run; accuracy follow-up deferred); step 2d 400 Hz FIFO batch mean (cited datasheet/AN3308 facts, bus deadline, alias fixtures, retired INT1 check, TEST raw capture) feeding the step 2c roll-only RIGHT-wheel primary; legacy support diagnostics, bounded GP21 TEST drive; step 2c hardware NOT a height PASS; step 2d floor PASS for the 10 mm RIGHT-wheel hump only.
- [`imu_core.c`](../app_program/subsystem4/imu_core.c), [`imu_core.h`](../app_program/subsystem4/imu_core.h) — pure guarded calibration, compensated pitch/roll, support-quality peaks and secondary slope integral; no driving decisions or packed fields.
- [`imu_hump.c`](../app_program/subsystem4/imu_hump.c), [`imu_hump.h`](../app_program/subsystem4/imu_hump.h) — pure step 3 hump detector: 100 ms mean, ON/OFF travel, curve/speed gates, run epochs, stale-safe run summary; report only.
- [`imu_hump_diag.c`](../app_program/subsystem4/imu_hump_diag.c), [`imu_hump_diag.h`](../app_program/subsystem4/imu_hump_diag.h) — pure diagnostic C: paired height windows, winning peak context and LINE-only motion maxima; service owns its separate bounded state/board. [`imu_hump_diag_print.c`](../app_program/subsystem4/imu_hump_diag_print.c) prints one USB line after STOP and at GP20 DUMP; no wire change.
- [`imu_support.c`](../app_program/subsystem4/imu_support.c), [`imu_settings.h`](../app_program/subsystem4/imu_settings.h) — pure alternative support hypotheses; WB80/T111, forward60/height130, refresh1degree, consistency2mm settings.
- [`imu_drive.c`](../app_program/subsystem4/imu_drive.c) — pure bounded GP21 job and per-drive Welford stats; TEST reporter owns stop-pending/retries, no height-dependent driving.
- [`imu_roll.c`](../app_program/subsystem4/imu_roll.c), [`imu_roll.h`](../app_program/subsystem4/imu_roll.h) — roll-only height, mean10/median20 maxima, wide diagnostic pitch ratio; fixtures compare15/20/31 windows and explicit crest/noise tradeoffs.
- [`imu_service.c`](../app_program/subsystem4/imu_service.c), [`imu_service.h`](../app_program/subsystem4/imu_service.h) — robot/TEST task14 and own 10 ms cyclic; bounded motion-count read, calibration-epoch guard, sole snapshot and hump board writer, acknowledged MARK, EMPTY-batch skip and IMU_STREAM capture handoff; BENCH excluded.
- [`vehicle_task.c`](../app_program/vehicle/vehicle_task.c) — atomic calibration epoch covers the unchanged line spin/pause/abort/home routine, also without MQTT; atomic run epoch, no-wait hump copy, live kind 9, run-end hump summary and USB line.
- [`imu_mark.c`](../app_program/subsystem4/imu_mark.c) — pure GP20 debounce for TEST IMU_STREAM.
- [`imu_hw.c`](../app_program/subsystem4/imu_hw.c) — robot/TEST SDK register layer, bounded I2C0 on GP0/GP1, shared 2 ms read-cycle deadline, bursts up to 48 B, UART-conflict guard; no SDK runtime ownership.
- [`imu_probe.c`](../app_program/subsystem4/imu_probe.c) — identity/configuration/readback, single-pass FIFO status/burst read, backlog/overrun reset, counted faults with paced retries.
- [`imu_fifo.c`](../app_program/subsystem4/imu_fifo.c) — pure FIFO_SRC decode, raw word decode and HR batch mean.
- [`imu_capture.c`](../app_program/subsystem4/imu_capture.c) — pure fixed 800-sample/256-batch TEST raw capture storage; storage and handoff exist only in IMU_STREAM.
- [`imu_decode.c`](../app_program/subsystem4/imu_decode.c) — pure units, identity mounting table and static pitch/roll.
- [`subsystem4_test.c`](../app_program/subsystem4/subsystem4_test.c) — observer only, 20 ms cyclic; 30 s/10 Hz sample lines normally, continuous/50 Hz with IMU_STREAM; `[S4_FIFO]` counters, raw capture dump after a confirmed GP21 stop, `[S4_HUMP]` per drive; not a bus owner.
- [`tests/imu_checks.cmake`](../tests/imu_checks.cmake) — strict core/task/board/stream/host benchmark, hump detector and corrected/filtered core fixtures, FIFO/alias/capture fixtures and protocol/register-double checks; [`tests/imu_stubs/`](../tests/imu_stubs) supplies SDK doubles.
- [`tests/test_imu_source_make.py`](../tests/test_imu_source_make.py) — isolated Make checks for `IMU_HUMP_SOURCE=1/2`, default/invalid values, `_humpfilt` image suffix and source-aware object invalidation. [H3 builds](kernel/BUILD.md#h3-source-comparison-images) compare both images; boot/result USB lines identify the source, default corrected.
- [`tests/test_imu_hump_diag.c`](../tests/test_imu_hump_diag.c) — peak latch, paired windows/gate history, winners/rejections, run lifecycle, LINE maxima, stale-safe output and USB units; linked in `tests/CMakeLists.txt`.
- [RAM budget](kernel/KERNEL.md#static-ram-growth-mqtt-robot-same-public-example-settings) — measured 5888 B MQTT delta including 2048 B stack (+772 B diagnostic C); IMU diagnostic C hardware high-water 1568/2048 B, free 480 B; sample frame 1000 B. 7888 B raw capture in IMU_STREAM only.
- [`testing/subsystem4/TEST_LOG.md`](../testing/subsystem4/TEST_LOG.md) — historical S4-U01/U03 evidence, partial step 2/2b hardware, step 2c hardware (NOT a height PASS), step 2d floor PASS for the 10 mm hump only and step 3 H1 STREAM hardware PASS on 1480b37, H0/H2-short plus diagnostic C hardware on 29d57de (uncorrected LINE height accepted; corrected over-read under investigation), named robot stacks and shared sensor scrape condition; H3 full-circuit no-false-hump acceptance is priority, H3..H9 Not run; absent-device/cold-power/MQTT-TEST gates still pending.

## Kernel ultrasonic acquisition

- [`app_program/subsystem5/drv_ultrasonic.c`](../app_program/subsystem5/drv_ultrasonic.c) — core-0 echo state machine with GPIO13 edge dispatch and TIMER timestamps; SDK-only `drv_ultrasonic_hw.c` drives GP3.
- [`tests/test_drv_ultrasonic.c`](../tests/test_drv_ultrasonic.c) — edge, timeout, distance and fault doubles.
- [`testing/subsystem5/TEST_LOG.md`](../testing/subsystem5/TEST_LOG.md) — S5-U01 fixed-object checks.

## Kernel robot integration

- [Junction action design](architecture/JUNCTION_ACTION.md) — fixed B2a fake START consumer and LINE/SEARCH wrapper; B2a closed at50426ae. [Live adapter](architecture/JUNCTION_LIVE_ADAPTER.md) — B2b STEP4a SHADOW admission and terminal ACK; no module number or summary change.
- [`junction_action.c`](../app_program/vehicle/junction_action.c) and [`junction_action.h`](../app_program/vehicle/junction_action.h) — pure forced-corner job (A/B/D white-gated SEARCH; C160 mm marker; unconsumed pending survives normal SEARCH only); [`junction_line.c`](../app_program/vehicle/junction_line.c) and [`junction_line.h`](../app_program/vehicle/junction_line.h) delegate travel to locked LINE and wrap unchanged SEARCH for a forced corner.
- [`junction_source.c`](../app_program/vehicle/junction_source.c) and [`junction_source.h`](../app_program/vehicle/junction_source.h) — exclusive fake source1 or live source2 backend. [`junction_live.c`](../app_program/vehicle/junction_live.c) and [`junction_live.h`](../app_program/vehicle/junction_live.h) own admission, independent encounter state, control and terminal ACK. [`junction_print.c`](../app_program/vehicle/junction_print.c) formats bounded USB evidence and counters in telemetry.
- [`test_junction_live.c`](../tests/test_junction_live.c) checks SHADOW, host-only permission, age/cut/route guards, serial wrap and old-epoch slot races. [`test_barcode_dump_stub.c`](../tests/test_barcode_dump_stub.c) checks strong stopped-dump override with the telemetry fixture.
- [`test_junction.c`](../tests/test_junction.c) and [`test_junction_make.py`](../tests/test_junction_make.py) — pure action/no-command comparison and Make option/profile tests. [Validation and J0–J6](kernel/JUNCTION_VALIDATION.md) records software checks, resource delta, operator-supplied J1/J2–J5 acceptance with capture limits, and new pending-policy hardware gates.

- [`app_program/vehicle/vehicle_task.c`](../app_program/vehicle/vehicle_task.c) — priority-13 core-0 navigation: snapshots, spin behaviour, modules, buttons, motion/service/telemetry messages; deadline counter, no sensor polling or dump loop.
- [`app_program/vehicle/vehicle_robot_config.h`](../app_program/vehicle/vehicle_robot_config.h) — reference-extracted locked settings.
- [`line_p_near.h`](../app_program/vehicle/line_p_near.h) — opt-in vehicle
  P-gain/base-speed adapter; [base-speed trial](s3-line-barcode/LINE.md#opt-in-base-speed-trial),
  [native base test](../tests/test_line_base.c) and
  [base Make tests](../tests/test_line_base_make.py); [trial settings](s3-line-barcode/LINE.md#opt-in-proportional-gain-trial),
  [configuration/steering test](../tests/test_line_p_near.c) and
  [Make option test](../tests/test_line_p_near_make.py). Default sources and
  flags are unchanged.
- [`app_program/vehicle/vehicle_log.c`](../app_program/vehicle/vehicle_log.c) and [`vehicle_log.h`](../app_program/vehicle/vehicle_log.h) — pure fixed 16-byte KR1 packing.
- [`scripts/robot_log_decode.py`](../scripts/robot_log_decode.py) — rows/events CSV decoder; KR1 module 5 means SCAN reverse in the GP20 log; `--hump` CSV from run-end, DUMP and `[S4_HUMP]` lines.
- [`tests/test_vehicle_task.c`](../tests/test_vehicle_task.c), [`test_vehicle_log.c`](../tests/test_vehicle_log.c) and [`test_robot_contract.py`](../tests/test_robot_contract.py) — glue, packing/decoder and reference-settings checks.
- [Kernel robot design](kernel/KERNEL.md) — current tasks, priorities, stacks, freshness, watchdog, USB21, CPU load and measured limits.
- [RAM budget](kernel/KERNEL.md#ram-budget) — static RAM by group, dynamic estimate, RAM-shortage steps and CR1 baseline order.
- [R5 procedure and integration results](../testing/integration/KERNEL_ROBOT.md) — track procedures and operator evidence; compare the stated revisions with bare-metal R4.

## Kernel hardware watchdog

- [`app_program/platform/drv_watchdog.c`](../app_program/platform/drv_watchdog.c) and [`drv_watchdog_hw.c`](../app_program/platform/drv_watchdog_hw.c) — kernel startup/feed guard and SDK-only hardware/reset snapshot.
- [`tests/test_watchdog.c`](../tests/test_watchdog.c), [`test_watchdog_hw.c`](../tests/test_watchdog_hw.c) and [`watchdog_stubs/hardware/watchdog.h`](../tests/watchdog_stubs/hardware/watchdog.h) — driver/register doubles.
- [Kernel hardware watchdog](kernel/KERNEL.md) — motion-only feeding, startup, reset classification and S2-U05 procedure.

## UP SMP utility shim

- [`smp_single_core.c`](../app_program/platform/smp_single_core.c) supplies the
  public lock/atomic API only in UP. The instruction-only
  [architecture seam](../app_program/platform/smp_single_core_arch.h) permits
  [real-source tests](../tests/test_smp_single_core.c), wired by
  [`shim_checks.cmake`](../tests/shim_checks.cmake) in both host test builds.
  [M7 contract and proof](kernel/KERNEL.md#lock-rules) records red/green results,
  ARM barriers, provider checks, RAM and hardware limits.

## Application task stacks

- [`app_program/platform/app_task.c`](../app_program/platform/app_task.c)
  owns static stacks and high-water reports. The bench stack and slot exist
  only with `APP_BENCH`. The MQTT idle-meter stack is now 768 B after
  the approved margin correction; other task stack sizes are unchanged.
  [`test_app_task.c`](../tests/test_app_task.c) checks normal, BENCH, MQTT
  and TEST slot layouts; [B1 evidence](architecture/CR1.md)
  separates this saving from minimum-bit packing.
  M9 registers Wi-Fi too: [watermark contract and RAM evidence](kernel/KERNEL.md#wi-fi-stack-m9).
  [`test_template_wifi.c`](../tests/test_template_wifi.c) links the real
  initializer and registry; [`wifi_stack_health_checks.c`](../tests/wifi_stack_health_checks.c)
  checks the real Health minimum at the full slot layout.
  [`test_wifi_stack_contention.c`](../tests/test_wifi_stack_contention.c)
  forces the real registry's diagnostic lock busy in UP/split/control.
  The two existing template ledger rows change; no new deviation path is added.
- [Application task stacks](kernel/KERNEL.md) — current static sizes, ownership and high-water limits.
- [Core placement and lock audit](kernel/KERNEL.md#core-placement-and-locks) records reviews B1/B2.
  The isolated [blocker test build](../tests/core_migration/CMakeLists.txt)
  compiles [production lock sections](../tests/core_migration/extract_lock_sections.py)
  for [interrupt-mask/preemption checks](../tests/core_migration/test_b1_lock_sections.c).
  It also extracts the [task lifecycle](../tests/core_migration/extract_lifecycle.py)
  for [exit/cleanup checks](../tests/core_migration/test_b2_lifecycle.c), without
  the full host suite's SMP stubs.

- [`line_service.c`](../app_program/subsystem3/line_service.c) — priority-11
  line task, inherited-mutex snapshot and calibration owner; `raw_bc` is
  sampled, with an O(1) copy only in BARCODE_CAPTURE.
- [`app_program/subsystem5/sonar_service.c`](../app_program/subsystem5/sonar_service.c) — priority-12 request/echo service: MBF requests, inherited-mutex result, result event flag and 10 ms deadline cyclic.
## Kernel telemetry

- [Core/staging merge (history)](archive/kernel/HISTORY.md#files) — combined core-placement and B2b SHADOW contract, complete parent test union, profile/ABI/RAM evidence and unchanged hardware gates.

- [`tests/test_sdk_profiles.py`](../tests/test_sdk_profiles.py) — M6 complete-log SDK proof gate, exact placement/barcode profile schemas, permission and stale-object rejection without invoking a compiler on failed gates.

- [`tests/test_storage_probe_race.c`](../tests/test_storage_probe_race.c) and [`test_telemetry_probe.c`](../tests/test_telemetry_probe.c) — M5 fast-consumer/noise schedules and exact-token disposition through the real FIFO row handler. Probe code is TEST-only.

- [`tests/review_checks.cmake`](../tests/review_checks.cmake) and [`test_log_stream_selftest.c`](../tests/test_log_stream_selftest.c) — M4 live-instance TEST-runner guard; isolated lifecycle coverage is in [`test_log_stream.c`](../tests/test_log_stream.c).

- [`tests/admission_checks.cmake`](../tests/admission_checks.cmake), [`test_telemetry_busy.c`](../tests/test_telemetry_busy.c) and [`test_vehicle_dump_start.c`](../tests/test_vehicle_dump_start.c) — M3 real telemetry admission and real Vehicle GP20 plus pending START/CALIBRATE cases, in UP, split and forced-core0 host configurations.
- [`tests/test_network_mask.c`](../tests/test_network_mask.c) — M2 deterministic two-writer availability-mask updates; host-only store/RMW instrumentation in SMP0 and SMP1 configurations.

- [`app_program/subsystem1/telemetry.c`](../app_program/subsystem1/telemetry.c) — priority-30 RAM log/dump owner; value messages via `mbf_telemetry`, `TMO_POL` producer sends.
- [Kernel telemetry](s1-telemetry/TELEMETRY.md) — boards/queues, RAM ownership, local loss policy and current MQTT boundary.

## Approved template deviations

- [Template change ledger](architecture/TEMPLATE_CHANGES.md) — shared startup and Wi-Fi changes; template source is not frozen.
- [`scripts/check-template-changes.py`](../scripts/check-template-changes.py) — verify the complete recorded deviation file list against `c3b0954`; the count must remain eight.
- Known reference exceptions: original lecturer-template comments in [`sys_arch.h`](../lib/liblwip/sysdepend/pico_rp2040/include/arch/sys_arch.h) and [`lwipopts.h`](../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h) cite the missing `RELEASE.md`. Leave both files untouched. These are the only two permitted old-path grep hits. The current rule is [single-task Wi-Fi/lwIP ownership](s1-telemetry/TELEMETRY.md#execution-and-hook).

## STEP 8 MQTT navigation

- [USB console priority](kernel/KERNEL.md) — template USB task 21, below driving and Wi-Fi; order check [`test_task_priorities.py`](../tests/test_task_priorities.py).
- [Timing investigation (history)](archive/s2-motion/HISTORY.md#files) and [`motion_diag.h`](../app_program/subsystem2/motion_diag.h) / [`motion_diag_capture.h`](../app_program/subsystem2/motion_diag_capture.h) — opt-in `DIAG=1` fixed timing snapshots, GP20-only output and [`host check`](../tests/test_motion_diag.c); no scheduler fix. [Hardware results](archive/s1-telemetry/HISTORY.md#files) remain tied to the tested earlier image.

- [`boot_id.c`](../app_program/platform/boot_id.c), [`boot_id_hw.c`](../app_program/platform/boot_id_hw.c), and [`boot_entropy.c`](../app_program/platform/boot_entropy.c) — bounded ROSC per-boot observation label and pure bit extraction; [`test_boot_id.c`](../tests/test_boot_id.c). Binary two-boot fixtures: [`mqtt_boot_fixtures.h`](../tests/mqtt_boot_fixtures.h).

- [Follow-up handoff (history)](archive/s1-telemetry/HISTORY.md#files) — final owner tips, commit sequence, helper cleanup/recovery and hardware limits.

- [`obstacle_report.c`](../app_program/vehicle/obstacle_report.c) — pure completed-scan report; [`test_obstacle_report.c`](../tests/test_obstacle_report.c).
- [Barcode run-summary counters](s3-line-barcode/BARCODE_RUN_SUMMARY.md)
  describe the vehicle task's bounded same-epoch SHADOW count reads and
  existing summary fields. No barcode observation changes navigation.
- [`odometry.c`](../app_program/vehicle/odometry.c) and [`run_stats.c`](../app_program/vehicle/run_stats.c) — encoder geometry, read-only run/detour summaries; [`test_odometry.c`](../tests/test_odometry.c), [`test_run_stats.c`](../tests/test_run_stats.c).
- [`idle_meter.c`](../app_program/platform/idle_meter.c) and [`cpu_load.c`](../app_program/platform/cpu_load.c) — calibrated lowest-priority CPU meter and pure arithmetic, including the per-run peak; [`test_cpu_load.c`](../tests/test_cpu_load.c), [CPU load](kernel/KERNEL.md#cpu-load).
- [`run_timing.c`](../app_program/platform/run_timing.c) and [`kernel_lock_stats.c`](../app_program/platform/kernel_lock_stats.c) — P1 report-only per-run latches, `[kcall]` call costs and big-kernel-lock figures; [`test_run_timing.c`](../tests/test_run_timing.c), host double [`kernel_lock_double.c`](../tests/smp_stubs/kernel_lock_double.c), [run timing report](kernel/KERNEL.md#run-timing-report-p1).

- [Design](s1-telemetry/TELEMETRY.md), [wire layout](s1-telemetry/MQTT_WIRE.md), [operator runbook](s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard).
- [`app_program/subsystem1/robot_mqtt.c`](../app_program/subsystem1/robot_mqtt.c) — strong Wi-Fi-task hook, retry/subscription/publish and the sink for `log_stream`.
- [`log_stream.c`](../app_program/subsystem1/log_stream.c) — live RAM-log stream (QoS 0, jump-to-newest, data age) and batched QoS 1 DUMP refill with ACK cursor; live pause in IDLE; no lwIP dependency.
- [`telemetry_mode.c`](../app_program/subsystem1/telemetry_mode.c) — pure RUN/IDLE publication-mode rule (2 s grace, TEST/BENCH always RUN) and the per-mode periods.
- [`robot_network.h`](../app_program/subsystem1/robot_network.h), [`network_queue.c`](../app_program/subsystem1/network_queue.c) — bounded live events and ordinary/ESTOP command queues, losses and provider contract.
- [`robot_health.c`](../app_program/subsystem1/robot_health.c) — priority31 cyclic producer, app-stack scan and bounded health board.
- [`mqtt_wire.c`](../app_program/subsystem1/mqtt_wire.c), [`scripts/robot_wire.py`](../scripts/robot_wire.py), [`scripts/mqtt_robot.py`](../scripts/mqtt_robot.py) — explicit binary codec, pure Python decoder with log merge/gap store (no Paho), and Paho2 VERSION2 receiver/CSV/commands.
- [`robot_mqtt_secrets.h.example`](../app_program/subsystem1/robot_mqtt_secrets.h.example) — app-owned private-header example; do not edit the template config example.
- [`vehicle_board.h`](../app_program/vehicle/vehicle_board.h), [`vehicle_mqtt.c`](../app_program/vehicle/vehicle_mqtt.c), [`vehicle_network_policy.c`](../app_program/vehicle/vehicle_network_policy.c) — vehicle read board, bounded cross-board adapters and idle/command policy.
- [`scripts/mqtt_dashboard.py`](../scripts/mqtt_dashboard.py), [`dashboard_model.py`](../scripts/dashboard_model.py), [`dashboard_page.py`](../scripts/dashboard_page.py) — laptop-only live dashboard ([runbook](s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard)): HTTP/SSE server with CSRF/Origin/Host checks, pure panel and button-gating model, inline page; it uses `robot_wire.py` and never sends the token to the browser. [`scripts/dashboard_replay.py`](../scripts/dashboard_replay.py) makes offline public-fixture pages and screenshots for the responsive layout.
- [`tests/mqtt_protocol.cmake`](../tests/mqtt_protocol.cmake), [`dashboard_checks.cmake`](../tests/dashboard_checks.cmake), [`network_checks.cmake`](../tests/network_checks.cmake), [`mqtt_client_checks.cmake`](../tests/mqtt_client_checks.cmake) — C/Python packing, board/queue/log and mocked transport regression tests.
- [`tests/test_sonar_history.c`](../tests/test_sonar_history.c) — non-consuming latest/history reads, overflow/miss and sequence wrap.

- [`app_program/platform/app_board.h`](../app_program/platform/app_board.h) — single-writer atomic word publication and one-attempt cross-core reads; unavailable never spins.
- [`tests/test_template_tick.c`](../tests/test_template_tick.c) and [`tests/test_template_wifi.c`](../tests/test_template_wifi.c) — startup register/idempotence/bounded-failure and Wi-Fi priority/lifetime/window/ABI doubles; [`tests/template_checks.cmake`](../tests/template_checks.cmake).

- Motion public `motion_snapshot_try_read` is a transport-independent last-tick board, not the inherited control mutex. Tests in [`tests/test_motion_task.c`](../tests/test_motion_task.c) cover null/unready/no-first-tick and coherent counts/time.


- Line public `line_snapshot_try_read` is a transport-independent bounded board, never an event consumer or mutex waiter. [`tests/test_line_service.c`](../tests/test_line_service.c) checks unready/null, coherent sample identity and non-consumption.

- [`tests/vehicle_network_checks.cmake`](../tests/vehicle_network_checks.cmake) — adapter age/wrap/busy, exhaustive command policy, and real vehicle task non-critical queue/DUMP/ESTOP tests.
- [STEP 8 consolidated historical record (history)](archive/s1-telemetry/HISTORY.md#files) — software build/strict/link/host results, hardware evidence and reboot follow-up in separate source sections; never extend a PASS to a later revision.
- [`tests/test_log_stream.c`](../tests/test_log_stream.c) — live batching/jump/generation/failure and DUMP window/ACK/rewind/abort/throughput and live-pause tests.
- [`tests/test_telemetry_mode.c`](../tests/test_telemetry_mode.c) — publication-mode transitions: run cycle, ESTOP, busy and absent vehicle board, time wrap.
