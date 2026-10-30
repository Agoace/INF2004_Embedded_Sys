# INF2004 Embedded Systems

This repository contains Raspberry Pi Pico W coursework for the INF2004 Embedded Systems module. It includes CMake firmware targets for hardware diagnostics and calibration on a Cytron Robo Pico board.

## Repository layout

- [`src/pin_probe.c`](src/pin_probe.c) — general Pico W GPIO connection probe.
- [`src/board_diagnostics.c`](src/board_diagnostics.c) — Robo Pico sensor and encoder diagnostics.
- [`src/servo_test.c`](src/servo_test.c) — short GP15 servo movement test.
- [`src/servo_calibration.c`](src/servo_calibration.c) — button-controlled positional-servo calibration.
- [`src/motor_encoder_calibration.c`](src/motor_encoder_calibration.c) — wheel encoder calibration and repeatability test.
- [`src/motor_duty_calibration.c`](src/motor_duty_calibration.c) — minimum free-running motor duty test.
- [`src/motor_speed_calibration.c`](src/motor_speed_calibration.c) — unloaded motor RPM versus PWM calibration.
- [`src/ultrasonic.c`](src/ultrasonic.c) — non-blocking HC-SR04+ driver with a packed 16-bit result.
- [`src/ultrasonic_test.c`](src/ultrasonic_test.c) — ultrasonic range and accuracy bench test.
- [`src/obstacle.c`](src/obstacle.c) — forward obstacle watch: NEAR, STOP, and CLEAR events from ultrasonic results.
- [`src/obstacle_test.c`](src/obstacle_test.c) — obstacle watch bench test; [`tests/test_obstacle.c`](tests/test_obstacle.c) is its host unit test.
- [`src/line_position.c`](src/line_position.c) — normalised line position from learned per-sensor values; [`src/line_test.c`](src/line_test.c) is its bench test.
- [`src/line_steer.c`](src/line_steer.c) — line position to left/right wheel duty; [`src/steer_test.c`](src/steer_test.c) is its bench test. [`src/drive.c`](src/drive.c), [`src/line_sensors.c`](src/line_sensors.c), and [`src/line_spin.c`](src/line_spin.c) are shared bench parts. [`src/wifi_log.c`](src/wifi_log.c) sends the `steer_test` log over Wi-Fi to [`scripts/steer_log_receiver.py`](scripts/steer_log_receiver.py) ([steering Wi-Fi log](docs/STEER_WIFI_LOG.md)).
- [`src/vehicle_controller.c`](src/vehicle_controller.c) — Vehicle Controller state machine (step 1: line following); [`tests/test_vehicle_controller.c`](tests/test_vehicle_controller.c) is its host unit test.
- [`src/obstacle_scan.c`](src/obstacle_scan.c) — 13-point ±30° servo scan; [`src/scan_test.c`](src/scan_test.c) is its bench test.
- [`src/obstacle_bypass.c`](src/obstacle_bypass.c) — obstacle bypass by edge following; [`src/bypass_test.c`](src/bypass_test.c) is its bench test ([obstacle bypass](docs/OBSTACLE_BYPASS.md)).
- [`src/obstacle_detour.c`](src/obstacle_detour.c) — obstacle detour with straight drives and turns on the spot only (version 1: each straight ends 190 mm after the nearest reading); [`src/detour_test.c`](src/detour_test.c) is its track test.
- [`src/robocar_ir_sensors.c`](src/robocar_ir_sensors.c) — line detection and Code 39 barcode reader firmware.
- [`CMakeLists.txt`](CMakeLists.txt) — firmware build definitions.
- [`pico_sdk_import.cmake`](pico_sdk_import.cmake) — Pico SDK locator supplied by the SDK.
- [`docs/`](docs/README.md) — durable documentation and calibration results.
- [`scripts/`](scripts/) — repository maintenance scripts.
- [`.githooks/`](.githooks/) — repository-owned Git hooks.

Future coursework belongs in clearly named task directories.

## Setup

Follow the [Pico W development guide](docs/DEVELOPMENT.md) to install the toolchain and Pico SDK. Then install the repository-owned Git hooks:

```sh
bash scripts/install-hooks.sh
```

The installer sets this repository's local `core.hooksPath` to `.githooks`. It stops without changing the setting if another custom hook path is active.

## Build

```sh
cmake -S . -B build -DPICO_BOARD=pico_w
cmake --build build
```

CMake creates separate UF2 files in `build/` for each diagnostic and calibration target.

## Use

Read the [agent navigation map](docs/AGENT_MAP.md) before changing an unfamiliar area. See the [hardware and calibration results](docs/CALIBRATION_RESULTS.md) for the confirmed wiring and measured values.

Check documentation links and heading anchors with:

```sh
python3 scripts/check-docs.py
```

The pre-commit hook runs this check only when relevant files are staged. Use `git commit --no-verify` only when you intentionally need to bypass the hook.
