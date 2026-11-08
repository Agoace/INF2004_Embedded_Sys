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
- [ARCHITECTURE.md](architecture/ARCHITECTURE.md) — current subsystems, tasks/cores, interrupt/polling rules, shell routes and safety limits.
- [BRANCH_LAYOUT.md](architecture/BRANCH_LAYOUT.md) — current integration rules, source boundaries and archived branch tags.
- [TEMPLATE_CHANGES.md](architecture/TEMPLATE_CHANGES.md) — eight approved template deviations and their constraints.
- [week8_change_requirement_1_notes.md](architecture/week8_change_requirement_1_notes.md) — original CR1 notes; wording and numbers unchanged, rewrite deferred.

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

- [LINE.md](s3-line-barcode/LINE.md) — current kernel pair-mode algorithm, P7 mm/s settings, search and revision-scoped results.
- [BARCODE.md](s3-line-barcode/BARCODE.md) — analog GP28 input and course geometry; decoder ideas only, not robot implementation.

## S4 — IMU

- [IMU.md](s4-imu/IMU.md) — merged bring-up, scale and validation records; positive Z offset explains raw magnitude, six-face axes confirmed and TEST stream passed; calibration remains pending.

## S5 — Obstacle

- [OBSTACLE.md](s5-obstacle/OBSTACLE.md) — five brief stages, implemented 13-point scan, monitor/profile rules, reverse/reattempt and recovery limits.
- [ULTRASONIC.md](s5-obstacle/ULTRASONIC.md) — current kernel trigger/echo service, packed result, timeouts and measured sensor limits.

## Simulation

- [SIMULATION.md](simulation/SIMULATION.md) — implemented host model, interfaces, assumptions, echo-fit and box-sweep summaries; not kernel/track proof.

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

### Architecture history

- [architecture/BRANCH_LAYOUT.md](archive/architecture/BRANCH_LAYOUT.md) — full source record; only links/paths adjusted.
- [architecture/INTERRUPT_CONVENTIONS.md](archive/architecture/INTERRUPT_CONVENTIONS.md) — full source record; only links/paths adjusted.
- [architecture/VEHICLE_STATE_MACHINE.md](archive/architecture/VEHICLE_STATE_MACHINE.md) — full source record; only links/paths adjusted.
- [architecture/CONTROL_ARCHITECTURE.md](archive/architecture/CONTROL_ARCHITECTURE.md) — full source record; only links/paths adjusted.
- [architecture/robo_pico_rtos_multicore_plan.md](archive/architecture/robo_pico_rtos_multicore_plan.md) — full source record; only links/paths adjusted.

### S3 — Line and barcode history

- [s3-line-barcode/LINE_POSITION.md](archive/s3-line-barcode/LINE_POSITION.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/LINE_STEERING.md](archive/s3-line-barcode/LINE_STEERING.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/LINE_STEER_MM.md](archive/s3-line-barcode/LINE_STEER_MM.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/TWO_SENSOR_LINE_TRACING.md](archive/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/VC_LINE_RUN.md](archive/s3-line-barcode/VC_LINE_RUN.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/BARCODE_READING_IDEAS.md](archive/s3-line-barcode/BARCODE_READING_IDEAS.md) — full source record; only links/paths adjusted.

### S5 — Obstacle history

- [s5-obstacle/OBSTACLE_MONITOR.md](archive/s5-obstacle/OBSTACLE_MONITOR.md) — full source record; only links/paths adjusted.
- [s5-obstacle/OBSTACLE_SCAN.md](archive/s5-obstacle/OBSTACLE_SCAN.md) — full source record; only links/paths adjusted.
- [s5-obstacle/VC_DETOUR_RUN.md](archive/s5-obstacle/VC_DETOUR_RUN.md) — full source record; only links/paths adjusted.
- [s5-obstacle/ULTRASONIC_DRIVER.md](archive/s5-obstacle/ULTRASONIC_DRIVER.md) — full source record; only links/paths adjusted.

### Simulation history

- [simulation/SIMULATION.md](archive/simulation/SIMULATION.md) — full source record; only links/paths adjusted.
- [simulation/SIM_ECHO_FIT.md](archive/simulation/SIM_ECHO_FIT.md) — full source record; only links/paths adjusted.
- [simulation/SIM_BOX_SWEEP.md](archive/simulation/SIM_BOX_SWEEP.md) — full source record; only links/paths adjusted.

## Original-template reference exceptions

Two untouched lecturer-template comments cite a missing `RELEASE.md` record:
[`sys_arch.h`](../lib/liblwip/sysdepend/pico_rp2040/include/arch/sys_arch.h) and
[`lwipopts.h`](../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h).
These are the two known old-path grep exceptions. Do not edit them as part of
a documentation move or add a ninth template deviation. The current ownership
rule is the Execution and hook section of the S1 MQTT design listed above.
