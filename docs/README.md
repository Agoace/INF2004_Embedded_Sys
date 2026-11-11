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
- [CR1 evidence](architecture/CR1_EVIDENCE.md) — baseline RAM, board/queue
  and MQTT sizes, measurement tools and shared-access rule.
- [JUNCTION_ACTION.md](architecture/JUNCTION_ACTION.md) — B2a fake START command, LINE/SEARCH corner job, pending retention across natural SEARCH, draft B2b contract and live phase/angle event7.
- [TEMPLATE_CHANGES.md](architecture/TEMPLATE_CHANGES.md) — eight approved template deviations and their constraints.
- [week8_change_requirement_1_notes.md](architecture/week8_change_requirement_1_notes.md) — original CR1 notes; wording and numbers unchanged, rewrite deferred.

## Kernel

- [KERNEL.md](kernel/KERNEL.md) — robot tasks, priorities, stacks, CPU meter, motion-only watchdog, USB21 and measured limits.
- [BUILD.md](kernel/BUILD.md) — current Make profiles, private settings, BOOTSEL/picotool and host checks.
- [JUNCTION_VALIDATION.md](kernel/JUNCTION_VALIDATION.md) — B2a software checks, public-config image builds, measured +280 B RAM, compiler frames and J0–J6 physical procedure; hardware Not run.
- [RAM_BUDGET.md](kernel/RAM_BUDGET.md) — measured step2/2b/2c deltas from b569adc, supplied step2b timing and pending current stack/CPU checks; shortage/CR1 order.
- [CORE_PLACEMENT.md](kernel/CORE_PLACEMENT.md) — Phase 1 core-1 placement seam, ISpinLock rule, DI/EI inventory with per-site verdicts and the handler-versus-task stress test; no device result.

## S1 — Telemetry

- [TELEMETRY.md](s1-telemetry/TELEMETRY.md) — sole Wi-Fi owner, read-only boards, bounded queues, idle/run modes and RAM/live/refill ownership.
- [MQTT_WIRE.md](s1-telemetry/MQTT_WIRE.md) — current binary topics, versions, bytes, masks, boot/session and decoder contract.
- [OPERATION.md](s1-telemetry/OPERATION.md) — broker, recorder, dashboard, Tailscale, security, operation and radio-loss troubleshooting.

## S2 — Motion

- [MOTION.md](s2-motion/MOTION.md) — controller, kernel/bench time-base distinction, settings, duty map and results.
- [HARDWARE.md](s2-motion/HARDWARE.md) — current pins, geometry, encoder ratio, calibration/drift/speed tables and limits.
- [CR1 motion plan](s2-motion/CR1_MOTION_PLAN.md) — baseline inventory,
  approved A1 board packing, ownership limits and approval gates.

## S3 — Line and barcode

- [LINE.md](s3-line-barcode/LINE.md) — current kernel pair-mode algorithm, P7 mm/s settings, search and revision-scoped results.
- [BARCODE.md](s3-line-barcode/BARCODE.md) — analog GP28 input and course geometry; decoder ideas only, not robot implementation.

## S4 — IMU

- [IMU.md](s4-imu/IMU.md) — step 3 report-only hump detector (summary v3, event kind 9, USB/DUMP lines, synthetic fixtures, H1 STREAM hardware PASS on 1480b37; diagnostic C hardware on 29d57de; uncorrected LINE height accepted, corrected over-read under investigation (9/9 humps, 0 false in 5 flat); IMU stack free 480 B; H3 full-circuit no-false-hump acceptance compares source-labelled corrected/filtered images, default corrected, and is priority and Not run; accuracy follow-up deferred); step 2d 400 Hz FIFO batch mean into the unchanged step 2c roll-only RIGHT height, cited datasheet/AN3308 facts, alias fixtures, retired INT1 check and TEST raw capture; step 2c hardware NOT a height PASS; step 2d floor PASS for the 10 mm RIGHT-wheel hump only.

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
### Kernel originals

The full original records retain their prose, numbers and revision limits;
only link/path/anchor repairs follow the move.

- [APPLICATION_TASKS.md](archive/kernel/APPLICATION_TASKS.md) — full historical source, with its measured values and original limits.
- [CPU_LOAD.md](archive/kernel/CPU_LOAD.md) — full historical source, with its measured values and original limits.
- [DEVELOPMENT.md](archive/kernel/DEVELOPMENT.md) — full historical source, with its measured values and original limits.
- [KERNEL_ROBOT.md](archive/kernel/KERNEL_ROBOT.md) — full historical source, with its measured values and original limits.
- [KERNEL_WATCHDOG.md](archive/kernel/KERNEL_WATCHDOG.md) — full historical source, with its measured values and original limits.
- [MTK3_LAYOUT_CHECKS.md](archive/kernel/MTK3_LAYOUT_CHECKS.md) — full historical source, with its measured values and original limits.
- [MTK3_MOTION_CHECKS.md](archive/kernel/MTK3_MOTION_CHECKS.md) — full historical source, with its measured values and original limits.
- [MTK3_USB_PRIORITY.md](archive/kernel/MTK3_USB_PRIORITY.md) — full historical source, with its measured values and original limits.

### S1 originals

- [KERNEL_MQTT.md](archive/s1-telemetry/KERNEL_MQTT.md) — full historical source, with its measured values and original limits.
- [KERNEL_TELEMETRY.md](archive/s1-telemetry/KERNEL_TELEMETRY.md) — full historical source, with its measured values and original limits.
- [MQTT_DASHBOARD.md](archive/s1-telemetry/MQTT_DASHBOARD.md) — full historical source, with its measured values and original limits.
- [MQTT_RUNBOOK.md](archive/s1-telemetry/MQTT_RUNBOOK.md) — full historical source, with its measured values and original limits.

### S2 originals

- [CALIBRATION_RESULTS.md](archive/s2-motion/CALIBRATION_RESULTS.md) — full historical source, with its measured values and original limits.
- [DUTY_MAP.md](archive/s2-motion/DUTY_MAP.md) — full historical source, with its measured values and original limits.
- [HEARTBEAT.md](archive/s2-motion/HEARTBEAT.md) — full historical source, with its measured values and original limits.
- [MOTOR_CONTROLLER.md](archive/s2-motion/MOTOR_CONTROLLER.md) — full historical source, with its measured values and original limits.
- [MOTOR_ROTATION_DRIFT.md](archive/s2-motion/MOTOR_ROTATION_DRIFT.md) — full historical source, with its measured values and original limits.
- [USB_POWER_SAFETY.md](archive/s2-motion/USB_POWER_SAFETY.md) — obsolete by operator decision; retained evidence, not a current procedure.

## Original-template reference exceptions

Two untouched lecturer-template comments cite a missing `RELEASE.md` record:
[`sys_arch.h`](../lib/liblwip/sysdepend/pico_rp2040/include/arch/sys_arch.h) and
[`lwipopts.h`](../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h).
These are the two known old-path grep exceptions. Do not edit them as part of
a documentation move or add a ninth template deviation. The current ownership
rule is the Execution and hook section of the S1 MQTT design listed above.
