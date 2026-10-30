# Agent Navigation Map — INF2004 Embedded Systems

Use this map before editing an unfamiliar part of the repository.

## Map chain

| Scope | Map or rule |
| --- | --- |
| Current repository | This file: `docs/AGENT_MAP.md` |
| Parent workspace | [School Repos map](../../../docs/AGENT_MAP.md) |

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
| Vehicle Controller record | [docs/VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md) |
| Normalised line position record | [docs/LINE_POSITION.md](LINE_POSITION.md) |
| Line steering record | [docs/LINE_STEERING.md](LINE_STEERING.md) |
| Steering Wi-Fi log | [docs/STEER_WIFI_LOG.md](STEER_WIFI_LOG.md) |
| Week 6 design-review diagrams | [`design-review/`](../design-review/) — Mermaid sources and rendered images for the §5 system statechart and §4 task interaction diagram; render with `npx -y @mermaid-js/mermaid-cli -i <file>.mmd -o <file>.png -b white --size 3000` |
| Parent checkout ledger | [School Repos checkout ledger](../../../docs/README.md#coursework-checkout-ledger) |

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
| Ultrasonic driver | [`src/ultrasonic.c`](../src/ultrasonic.c), [`src/ultrasonic.h`](../src/ultrasonic.h) |
| Ultrasonic bench test | [`src/ultrasonic_test.c`](../src/ultrasonic_test.c) |
| Forward obstacle watch | [`src/obstacle.c`](../src/obstacle.c), [`src/obstacle.h`](../src/obstacle.h) |
| Obstacle watch bench test | [`src/obstacle_test.c`](../src/obstacle_test.c) |
| Servo obstacle scan | [`src/obstacle_scan.c`](../src/obstacle_scan.c), [`src/obstacle_scan.h`](../src/obstacle_scan.h) |
| Obstacle profile analysis | [`src/obstacle_profile.c`](../src/obstacle_profile.c), [`src/obstacle_profile.h`](../src/obstacle_profile.h) |
| Obstacle bypass | [`src/obstacle_bypass.c`](../src/obstacle_bypass.c), [`src/obstacle_bypass.h`](../src/obstacle_bypass.h); bench test [`src/bypass_test.c`](../src/bypass_test.c) |
| Obstacle detour (straights and turns on the spot) | [`src/obstacle_detour.c`](../src/obstacle_detour.c), [`src/obstacle_detour.h`](../src/obstacle_detour.h); track test [`src/detour_test.c`](../src/detour_test.c); host test [`tests/test_obstacle_detour.c`](../tests/test_obstacle_detour.c) |
| Normalised line position | [`src/line_position.c`](../src/line_position.c), [`src/line_position.h`](../src/line_position.h) |
| Line position bench test | [`src/line_test.c`](../src/line_test.c) |
| Line steering | [`src/line_steer.c`](../src/line_steer.c), [`src/line_steer.h`](../src/line_steer.h); bench test [`src/steer_test.c`](../src/steer_test.c) |
| Steering Wi-Fi log | [`src/wifi_log.c`](../src/wifi_log.c), packet format [`src/steer_log.c`](../src/steer_log.c); receiver [`scripts/steer_log_receiver.py`](../scripts/steer_log_receiver.py) |
| Drive, IR reads, spin calibration (bench parts) | [`src/drive.c`](../src/drive.c), [`src/line_sensors.c`](../src/line_sensors.c), [`src/line_spin.c`](../src/line_spin.c) |
| Vehicle Controller | [`src/vehicle_controller.c`](../src/vehicle_controller.c), [`src/vehicle_controller.h`](../src/vehicle_controller.h) |
| Servo driver (clamped) | [`src/servo.c`](../src/servo.c), [`src/servo.h`](../src/servo.h) |
| Servo scan bench test | [`src/scan_test.c`](../src/scan_test.c) |
| Host unit tests | [`tests/`](../tests/) — run without hardware; see [OBSTACLE_MONITOR.md](OBSTACLE_MONITOR.md#host-unit-test), [OBSTACLE_SCAN.md](OBSTACLE_SCAN.md#host-unit-test), [OBSTACLE_BYPASS.md](OBSTACLE_BYPASS.md#host-unit-test), [VEHICLE_CONTROLLER.md](VEHICLE_CONTROLLER.md#host-unit-test), [LINE_POSITION.md](LINE_POSITION.md#host-unit-test), [LINE_STEERING.md](LINE_STEERING.md#host-unit-test), and [STEER_WIFI_LOG.md](STEER_WIFI_LOG.md#host-tests) |
| IR line detection and barcode reader | [`src/robocar_ir_sensors.c`](../src/robocar_ir_sensors.c), [`src/line_detector.c`](../src/line_detector.c), [`src/barcode_reader.c`](../src/barcode_reader.c) |
| Telemetry receiver | [`scripts/telemetry_receiver.py`](../scripts/telemetry_receiver.py) |
| Telemetry dashboard | [`scripts/telemetry_dashboard.py`](../scripts/telemetry_dashboard.py) |
| Telemetry JSON schema | [`TELEMETRY_SCHEMA.md`](TELEMETRY_SCHEMA.md) |
| Eventual control architecture | [`CONTROL_ARCHITECTURE.md`](CONTROL_ARCHITECTURE.md) |
| Telemetry port restriction | [`scripts/restrict-telemetry-port.sh`](../scripts/restrict-telemetry-port.sh) |
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
