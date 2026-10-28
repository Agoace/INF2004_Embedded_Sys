# INF2004 Embedded Systems

This repository contains Raspberry Pi Pico W coursework for the INF2004 Embedded Systems module. It includes CMake firmware targets for hardware diagnostics and calibration on a Cytron Robo Pico board.

## Repository layout

- [`IMU-Telemetry/`](IMU-Telemetry/README.md) - standalone GY-511 IMU producer and USB diagnostics.

- [`src/pin_probe.c`](src/pin_probe.c) — general Pico W GPIO connection probe.
- [`src/board_diagnostics.c`](src/board_diagnostics.c) — Robo Pico sensor and encoder diagnostics.
- [`src/servo_test.c`](src/servo_test.c) — short GP15 servo movement test.
- [`src/servo_calibration.c`](src/servo_calibration.c) — button-controlled positional-servo calibration.
- [`src/motor_encoder_calibration.c`](src/motor_encoder_calibration.c) — wheel encoder calibration and repeatability test.
- [`src/motor_duty_calibration.c`](src/motor_duty_calibration.c) — minimum free-running motor duty test.
- [`src/motor_speed_calibration.c`](src/motor_speed_calibration.c) — unloaded motor RPM versus PWM calibration.
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
