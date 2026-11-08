# Documentation — INF2004 Embedded Systems

Each record below has one entry. Use the agent map for source entry points.
Archive records retain their original revision, evidence and limits; they are
not current acceptance reports. Local course originals are not public records.

## Navigation

- [README.md](README.md) — this grouped documentation index.
- [AGENT_MAP.md](AGENT_MAP.md) — source locations, repository boundaries and map chain.
- [Repository README](../README.md) — public setup, current build and usage.
- [Repository instructions](../AGENTS.md) — working rules and validation commands.

## Project requirements

- [project-requirements/README.md](project-requirements/README.md) — our own course summary; original course files stay local and ignored.

## Architecture

- [ADR_HEARTBEAT.md](architecture/ADR_HEARTBEAT.md) — one time-base owner; original bare-metal interrupt contract.
- [ADR_MOTOR_CONTROLLER.md](architecture/ADR_MOTOR_CONTROLLER.md) — one actuator owner and value-based interfaces.
- [ADR_VEHICLE_CONTROLLER.md](architecture/ADR_VEHICLE_CONTROLLER.md) — one running module, shared detectors and outcome routing.
- [BRANCH_LAYOUT.md](architecture/BRANCH_LAYOUT.md) — module/integration branches, merge rules, template layout and archive tags.
- [CONTROL_ARCHITECTURE.md](architecture/CONTROL_ARCHITECTURE.md) — earlier workload and core-split proposal; not the current scheduler contract.
- [INTERRUPT_CONVENTIONS.md](architecture/INTERRUPT_CONVENTIONS.md) — event, time and polling ownership rules.
- [robo_pico_rtos_multicore_plan.md](architecture/robo_pico_rtos_multicore_plan.md) — task/core design proposal and port comparison.
- [TEMPLATE_CHANGES.md](architecture/TEMPLATE_CHANGES.md) — eight approved template deviations and their constraints.
- [VEHICLE_STATE_MACHINE.md](architecture/VEHICLE_STATE_MACHINE.md) — shared shell, route table, outcomes and safety stop.
- [week8_change_requirement_1_notes.md](architecture/week8_change_requirement_1_notes.md) — our cross-subsystem CR1 interpretation, widths and open measurements.

## Kernel

- [APPLICATION_TASKS.md](kernel/APPLICATION_TASKS.md) — static task stacks, ownership and high-water limits.
- [CPU_LOAD.md](kernel/CPU_LOAD.md) — calibrated idle meter and measurement limits.
- [DEVELOPMENT.md](kernel/DEVELOPMENT.md) — toolchain, flash and serial setup; older CMake bench examples are not the current robot build.
- [KERNEL_ROBOT.md](kernel/KERNEL_ROBOT.md) — task/object design, sensor freshness, deadlines and revision-scoped validation.
- [KERNEL_WATCHDOG.md](kernel/KERNEL_WATCHDOG.md) — motion-only feeding, reset classification and two-boot procedure.
- [MTK3_LAYOUT_CHECKS.md](kernel/MTK3_LAYOUT_CHECKS.md) — course-template layout and early platform checks.
- [MTK3_MOTION_CHECKS.md](kernel/MTK3_MOTION_CHECKS.md) — motion port, SDK link proof, image sizes and original hardware gates.
- [MTK3_USB_PRIORITY.md](kernel/MTK3_USB_PRIORITY.md) — measured USB dispatch issue and priority 21 decision.

## S1 — Telemetry

- [KERNEL_MQTT.md](s1-telemetry/KERNEL_MQTT.md) — sole Wi-Fi/lwIP owner, pull boards, queues and timing limits.
- [KERNEL_TELEMETRY.md](s1-telemetry/KERNEL_TELEMETRY.md) — nonblocking message transport, RAM log owner and local loss policy.
- [MQTT_DASHBOARD.md](s1-telemetry/MQTT_DASHBOARD.md) — responsive live dashboard, offline replay, panels, command gating and web security.
- [MQTT_RUNBOOK.md](s1-telemetry/MQTT_RUNBOOK.md) — broker/firewall scope, private settings, CSV and operator commands.
- [MQTT_WIRE.md](s1-telemetry/MQTT_WIRE.md) — topic versions, boot identity, packed bytes, masks and decoder contract.

## S2 — Motion

- [CALIBRATION_RESULTS.md](s2-motion/CALIBRATION_RESULTS.md) — confirmed wiring, geometry, sensors, servo limits and motor calibration.
- [DUTY_MAP.md](s2-motion/DUTY_MAP.md) — loaded floor mapping from duty to wheel speed.
- [HEARTBEAT.md](s2-motion/HEARTBEAT.md) — original bare-metal time-base implementation and bench tests.
- [MOTOR_CONTROLLER.md](s2-motion/MOTOR_CONTROLLER.md) — controller interface, robot settings, kernel layer and approved SDK subset.
- [MOTOR_ROTATION_DRIFT.md](s2-motion/MOTOR_ROTATION_DRIFT.md) — 50-revolution encoder measurements and alignment evidence.
- [USB_POWER_SAFETY.md](s2-motion/USB_POWER_SAFETY.md) — shared USB current limit, motor-test procedure and bounded exception.

## S3 — Line and barcode

- [BARCODE_READING_IDEAS.md](s3-line-barcode/BARCODE_READING_IDEAS.md) — unimplemented decoding ideas and sensor/geometry limits.
- [LINE_POSITION.md](s3-line-barcode/LINE_POSITION.md) — learned floor/line values and packed line events.
- [LINE_STEERING.md](s3-line-barcode/LINE_STEERING.md) — line position to wheel duty, optional D term and bench procedure.
- [LINE_STEER_MM.md](s3-line-barcode/LINE_STEER_MM.md) — locked steering translated to wheel speeds; see the line-run evidence for physical scope.
- [TWO_SENSOR_LINE_TRACING.md](s3-line-barcode/TWO_SENSOR_LINE_TRACING.md) — locked two-sensor design, wiring, settings and evidence.
- [VC_LINE_RUN.md](s3-line-barcode/VC_LINE_RUN.md) — LINE/SEARCH module integration and recorded P5–P9 runs.

## S4 — IMU

- [IMU.md](s4-imu/IMU.md) — merged bring-up, scale and validation records; positive Z offset explains raw magnitude, six-face axes confirmed and TEST stream passed; calibration remains pending.

## S5 — Obstacle

- [OBSTACLE_MONITOR.md](s5-obstacle/OBSTACLE_MONITOR.md) — forward range thresholds, packed events and tests.
- [OBSTACLE_SCAN.md](s5-obstacle/OBSTACLE_SCAN.md) — servo limits, scan profile, bounded reverse/reattempts and tests.
- [ULTRASONIC_DRIVER.md](s5-obstacle/ULTRASONIC_DRIVER.md) — nonblocking HC-SR04+ driver, packed range and bench procedure.
- [VC_DETOUR_RUN.md](s5-obstacle/VC_DETOUR_RUN.md) — SCAN/DETOUR integration and completed physical detour evidence; retains run-specific limits.

## Simulation

- [SIMULATION.md](simulation/SIMULATION.md) — separate host simulation, deterministic clock, devices, traces, viewer and limits.
- [SIM_ECHO_FIT.md](simulation/SIM_ECHO_FIT.md) — assumed echo materials and recorded-versus-simulated fit limits.
- [SIM_BOX_SWEEP.md](simulation/SIM_BOX_SWEEP.md) — seven fixed-front box cases, console evidence, hooks and viewer captures.

## Archive

- [DETOUR_V1_6_TEST_PLAN.md](archive/DETOUR_V1_6_TEST_PLAN.md) — earlier detour physical-test plan and simulator measurements still owed at that revision.
- [MOTION_TIMING_INVESTIGATION.md](archive/MOTION_TIMING_INVESTIGATION.md) — timing confound and opt-in diagnostic investigation, with original revision limits.
- [MTK3_FOLLOWUP_HANDOFF.md](archive/MTK3_FOLLOWUP_HANDOFF.md) — historical STEP 8 owner tips, cleanup/recovery and hardware limits.
- [MTK3_STEP8_RECORD.md](archive/MTK3_STEP8_RECORD.md) — merged software validation, hardware evidence and boot-session follow-up; each source keeps its own tested revision.
- [OBSTACLE_BYPASS.md](archive/OBSTACLE_BYPASS.md) — shelved edge-following bypass v4.1, runs and unresolved issues.
- [OBSTACLE_DETOUR_HANDOFF.md](archive/OBSTACLE_DETOUR_HANDOFF.md) — earlier detour design, D1–D18 evidence and then-open problems.
- [STEER_WIFI_LOG.md](archive/STEER_WIFI_LOG.md) — earlier steering UDP log format and receiver.
- [TELEMETRY_SCHEMA.md](archive/TELEMETRY_SCHEMA.md) — older instrument JSON contract; not the current MQTT wire format.
- [VEHICLE_CONTROLLER.md](archive/VEHICLE_CONTROLLER.md) — preliminary controller design, superseded by the shared vehicle shell.
- [WIFI_TELEMETRY_DASHBOARD.md](archive/WIFI_TELEMETRY_DASHBOARD.md) — earlier UDP bench dashboard, replaced by the MQTT dashboard.

## Original-template reference exceptions

Two untouched lecturer-template comments cite a missing `RELEASE.md` record:
[`sys_arch.h`](../lib/liblwip/sysdepend/pico_rp2040/include/arch/sys_arch.h) and
[`lwipopts.h`](../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h).
These are the two known old-path grep exceptions. Do not edit them as part of
a documentation move or add a ninth template deviation. The current ownership
rule is the Execution and hook section of the S1 MQTT design listed above.
