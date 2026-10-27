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
| Motion control record | [docs/MOTION_CONTROL.md](MOTION_CONTROL.md) |
| Line-following Manager checkpoint | [docs/LINE_MANAGER.md](LINE_MANAGER.md) |
| USB power safety | [docs/USB_POWER_SAFETY.md](USB_POWER_SAFETY.md) |
| Wi-Fi telemetry dashboard architecture | [docs/WIFI_TELEMETRY_DASHBOARD.md](WIFI_TELEMETRY_DASHBOARD.md) |
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
| Shared team header (packed fields, fault bits, encoder delta) | [`src/common/robot_shared.h`](../src/common/robot_shared.h) |
| Motion-control library | [`src/motion/`](../src/motion/motion_control.h) |
| Motion bench target | [`src/motion_bench.c`](../src/motion_bench.c) |
| Motion and line integration target | [`src/robocar_line_follow.c`](../src/robocar_line_follow.c) |
| Manager (vehicle state) core | [`src/manager/manager.c`](../src/manager/manager.c), [`src/manager/manager.h`](../src/manager/manager.h) |
| Manager shared types | [`src/manager/manager_types.h`](../src/manager/manager_types.h) |
| Integrated line-follow target (Manager + Line + Motion) | [`src/robocar_line_manager.c`](../src/robocar_line_manager.c) |
| μT-Kernel Line/Manager adapter | [`src/manager/manager_task_utkernel.c`](../src/manager/manager_task_utkernel.c) |
| Manager host unit tests | [`tests/manager/test_manager.c`](../tests/manager/test_manager.c) |
| Motion host unit tests | [`tests/motion/test_motion.c`](../tests/motion/test_motion.c) |
| Telemetry transport | [`src/telemetry.c`](../src/telemetry.c), [`src/telemetry.h`](../src/telemetry.h) |
| Wi-Fi telemetry probe | [`src/wifi_telemetry_probe.c`](../src/wifi_telemetry_probe.c) |
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
