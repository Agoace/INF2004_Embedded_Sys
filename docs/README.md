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
- [ADR: motor controller owns the actuators](ADR_MOTOR_CONTROLLER.md) — accepted 2026-11-04: one owner of motor PWM, servo PWM and encoders; values in, measurements out; no intent; hardware quirks inside.
- [ADR: one hardware heartbeat](ADR_HEARTBEAT.md) — accepted 2026-11-04: 1 ms timer interrupt owned by a separate `heartbeat` platform module; ADC sample, `mc_tick()` and watchdog in the interrupt; decisions in the main loop.
- [ADR: Vehicle Controller runs one module at a time](ADR_VEHICLE_CONTROLLER.md) — accepted 2026-11-04: shared detectors, running module interprets them and returns an outcome, controller maps outcomes to the next module and is the only motor controller caller.
- [Vehicle state machine](VEHICLE_STATE_MACHINE.md) — the Vehicle Controller shell from the ADR: route table, one module at a time, handoff in the same tick, safety stop, outcome word, host test.
- [Vehicle Controller line run](VC_LINE_RUN.md) — line tracing and line search modules in the shell, wheel speeds through the motor controller; target `vc_line_test`; runs P5–P9, stopped at the P7 settings.
- [Vehicle Controller detour run](VC_DETOUR_RUN.md) — scan and detour modules in the shell (step c), the ping request; same target; not yet run on the track.
- [Line steering in mm/s](LINE_STEER_MM.md) — the locked line steering moved to wheel speeds for the motor controller: same rule, no trim, settings mapped one to one; not yet run on the track.
- [Duty map](DUTY_MAP.md) — loaded floor map from open-loop duty to wheel speed, for moving duty-based settings to the motor controller; bench target `duty_map_bench`.
- [Heartbeat](HEARTBEAT.md) — the 1 ms time base from the ADR: hooks in the interrupt, flags for the main loop, watchdog, health counters, host test, bench target `heartbeat_test`.
- [Motor controller](MOTOR_CONTROLLER.md) — the controller from the ADR: interface, speed loop (from Motion), robot settings, host test, step status.
- [Eventual control architecture](CONTROL_ARCHITECTURE.md) — planned line-following workload, timing constraints, and proposed core split.
- [Week 8 Change Requirement 1 notes](week8_change_requirement_1_notes.md) — minimum-bit data representation decisions, calculations, and open measurements.
- [Ultrasonic driver and bench test](ULTRASONIC_DRIVER.md) — non-blocking HC-SR04+ driver, packed 16-bit result, and range test procedure.
- [Forward obstacle watch](OBSTACLE_MONITOR.md) — ultrasonic thresholds, NEAR/STOP/CLEAR events, packed event word, and tests.
- [Two-sensor line tracing (locked)](TWO_SENSOR_LINE_TRACING.md) — locked 2026-11-03: wiring, position rules, steering settings, evidence, and limits.
- [Normalised line position](LINE_POSITION.md) — per-sensor learned floor and line values, line position −100…+100, LINE_LOST/LINE_FOUND events.
- [Line steering](LINE_STEERING.md) — line position to left/right wheel duty (proportional, optional D), steer word, `steer_test` bench procedure.
- [Steering Wi-Fi log](STEER_WIFI_LOG.md) — `steer_test` text and 50 ms run log over UDP broadcast (port 4243), packet format, receiver use.
- [Vehicle Controller](VEHICLE_CONTROLLER.md) — controller state machine built step by step; step 1 is line following.
- [2D firmware simulation](SIMULATION.md) — design and milestone status; separate host build, deterministic clock, hardware replacements, offline viewer, optional loopback environment editor, limits, and build plan. [Host build and run instructions](../sim/README.md) — offline viewer, local editor, timed echoes/full detour, schema, trace, and tests.
- [Echo material model and fit limits](SIM_ECHO_FIT.md) — deterministic specular incidence, assumed defaults, and recorded-versus-simulated scan tables.
- [v1.5 box-size sweep](SIM_BOX_SWEEP.md) — seven fixed-front box cases, results, full main-case console and hooks, and viewer captures; no firmware or model changes.
- [Obstacle detour handoff](OBSTACLE_DETOUR_HANDOFF.md) — straight-and-turn obstacle detour (successor of the bypass), state at 2026-10-30: design v1.4, track runs D1–D18, open problems, next steps.
- [Detour v1.6 test plan](DETOUR_V1_6_TEST_PLAN.md) — physical test plan: build, bench checks, track runs, pass/fail, measurements still owed to the simulator.
- [Obstacle bypass](OBSTACLE_BYPASS.md) — edge-following bypass ("Blind Bicycle Algorithm"), shelved 2026-10-30 at version 4.1: design, track runs 1–18, open problems, `bypass_test` bench procedure.
- [Servo obstacle scan](OBSTACLE_SCAN.md) — mounted servo limits, 13-point ±30° scan (5° steps), PROFILE event (shorter way around, no width), and tests.
- [School Repos checkout ledger](https://github.com/Agoace/School-Repos/blob/main/docs/README.md#coursework-checkout-ledger) — parent workspace state. This link opens the parent record on GitHub.

## Branch layout

See [Branch layout](BRANCH_LAYOUT.md) for module ownership and merge rules.

- [Robo Pico RTOS multicore plan](robo_pico_rtos_multicore_plan.md) — proposed task and core split.

- [Course template layout checks](MTK3_LAYOUT_CHECKS.md)
