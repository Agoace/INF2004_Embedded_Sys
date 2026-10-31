# Documentation — INF2004 Embedded Systems

This index lists durable documentation for the INF2004 Embedded Systems repository.

## Navigation

For repository structure and implementation entry points, see the [Agent Navigation Map](AGENT_MAP.md).

## Repository entry points

- [README](../README.md) — public orientation, setup, build, and usage.
- [AGENTS](../AGENTS.md) — agent working instructions.

## Records

- [Agent Navigation Map](AGENT_MAP.md) — repository paths, entry points, and boundaries.
- [Pico W development environment](DEVELOPMENT.md) — tool installation, build, flash, and serial instructions.
- [Hardware and calibration results](CALIBRATION_RESULTS.md) — confirmed wiring, sensor results, servo limits, and motor calibration.
- [Motor rotation drift record](MOTOR_ROTATION_DRIFT.md) — 50-revolution counts-per-revolution measurements and jog alignment results.
- [Interrupt and polling conventions](INTERRUPT_CONVENTIONS.md) — rules for event-driven, time-driven, and no-event work.
- [Raspberry Pi 400 USB power safety](USB_POWER_SAFETY.md) — 1.2 A shared limit, motor-test procedure, bounded exception, and over-current recovery.
- [Wi-Fi telemetry dashboard architecture](WIFI_TELEMETRY_DASHBOARD.md) — Pico-to-Pi telemetry, localhost dashboard, and private tailnet access.
- [Telemetry JSON schema](TELEMETRY_SCHEMA.md) — versioned instrument packet contract for firmware, receiver, and future dashboard.
- [Eventual control architecture](CONTROL_ARCHITECTURE.md) — planned line-following workload, timing constraints, and proposed core split.
- [Week 8 Change Requirement 1 notes](week8_change_requirement_1_notes.md) — minimum-bit data representation decisions, calculations, and open measurements.
- [Ultrasonic driver and bench test](ULTRASONIC_DRIVER.md) — non-blocking HC-SR04+ driver, packed 16-bit result, and range test procedure.
- [Forward obstacle watch](OBSTACLE_MONITOR.md) — ultrasonic thresholds, NEAR/STOP/CLEAR events, packed event word, and tests.
- [Normalised line position](LINE_POSITION.md) — per-sensor learned floor and line values, line position −100…+100, LINE_LOST/LINE_FOUND events.
- [Line steering](LINE_STEERING.md) — line position to left/right wheel duty (proportional, optional D), steer word, `steer_test` bench procedure.
- [Steering Wi-Fi log](STEER_WIFI_LOG.md) — `steer_test` text and 50 ms run log over UDP broadcast (port 4243), packet format, receiver use.
- [Vehicle Controller](VEHICLE_CONTROLLER.md) — controller state machine built step by step; step 1 is line following.
- [Obstacle detour handoff](OBSTACLE_DETOUR_HANDOFF.md) — straight-and-turn obstacle detour (successor of the bypass), state at 2026-10-30: design v1.4, track runs D1–D18, open problems, next steps.
- [Detour v1.6 test plan](DETOUR_V1_6_TEST_PLAN.md) — physical test plan: build, bench checks, track runs, pass/fail, measurements still owed to the simulator.
- [Obstacle bypass](OBSTACLE_BYPASS.md) — edge-following bypass ("Blind Bicycle Algorithm"), shelved 2026-10-30 at version 4.1: design, track runs 1–18, open problems, `bypass_test` bench procedure.
- [Servo obstacle scan](OBSTACLE_SCAN.md) — mounted servo limits, 13-point ±30° scan (5° steps), PROFILE event (shorter way around, no width), and tests.
- [School Repos checkout ledger](../../../docs/README.md#coursework-checkout-ledger) — parent workspace state. This link resolves only in a School Repos submodule checkout.
