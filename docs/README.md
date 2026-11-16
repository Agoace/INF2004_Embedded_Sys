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

- [ARCHITECTURE.md](architecture/ARCHITECTURE.md) — current subsystems, tasks/cores, interrupt/polling rules, shell routes and safety limits; the three ADRs in full: motor controller is the only actuator owner, one hardware heartbeat (bare-metal time base), Vehicle Controller runs one module at a time (four layers, rules, outcome table).
- [PROJECT_STATUS.md](architecture/PROJECT_STATUS.md) — one row per area: state, evidence, open items, demo build line and robot run-log locations; branch table, merge rules (`--no-ff`, no rebase), mtk3 source layout, commit checks, branch state 2026-11-16 and archive tags. Read before proposing work.
- [MEASUREMENTS.md](architecture/MEASUREMENTS.md) — every measured or assumed physical/timing value by area (geometry, mass, motors, encoders, line, GP28, sonar/servo, IMU, timing/RAM), with date, label, source and the values that do not agree. Check it before asking the operator for a number.
- [CR1.md](architecture/CR1.md) — CR1 requirement summary and Week 8 §11 (IMU snapshot 8 B) and §33–38 (vehicle states 4 bits, mode/enum/fault bits, events) details, baseline RAM, board/queue
  and MQTT sizes, measurement tools, shared-access rule, A1/B1/B2 evidence, A2 plan, M9 accounting and the motion module plan (A1 field table, controller/ISR inventory, tests).
- [JUNCTION_ACTION.md](architecture/JUNCTION_ACTION.md) — fixed B2a consumer, fake START command, LINE/SEARCH corner job, pending retention and phase/angle event7.
- [JUNCTION_LIVE_ADAPTER.md](architecture/JUNCTION_LIVE_ADAPTER.md) — B2b STEP4a source2 SHADOW admission, encounter ordering, terminal ACK, limits, counters and action gates.
- [TEMPLATE_CHANGES.md](architecture/TEMPLATE_CHANGES.md) — eight approved template deviations and their constraints.

## Kernel

- [KERNEL.md](kernel/KERNEL.md) — robot tasks, priorities, stacks (Wi-Fi M9, IMU high-water), watchdog, CPU meter, P1/P2 run timing, KERNEL_RAM (rule b, robot pairs), F-NVIC, core placement and lock rules (split parked), M8 startup handoff, RAM budget, USB21 and measured limits.
- [BUILD.md](kernel/BUILD.md) — Make profiles and options, demo line, private settings, BOOTSEL/picotool, BENCH_KERNEL spare-Pico bench (pass rule), SWD probe, host checks.
- [JUNCTION_VALIDATION.md](kernel/JUNCTION_VALIDATION.md) — B2a closure and scoped operator evidence; B2b software/image/resource checks and remaining SHADOW hardware gates.

## S1 — Telemetry

- [TELEMETRY.md](s1-telemetry/TELEMETRY.md) — sole Wi-Fi owner, read-only boards, bounded queues, idle/run modes, RAM/live/refill ownership; operation: broker, private settings, recorder, dashboard (fan, hump, run path), commands, Tailscale, security and troubleshooting.
- [MQTT_WIRE.md](s1-telemetry/MQTT_WIRE.md) — current binary topics, versions, bytes, masks, boot/session and decoder contract.

## S2 — Motion

- [MOTION.md](s2-motion/MOTION.md) — controller interface, settings (feed-forward, P/I, kick, stall, servo), hardware/time boundary, heartbeat, pins, geometry, 50-turn encoder ratio 2535, servo/sonar, unloaded RPM tables, power status, DM1 duty map, B3 results, SDK subset.

## S3 — Line and barcode

- [GP28 spin evidence](s3-line-barcode/LIVE_CALIBRATION.md).
- [Live source contract](s3-line-barcode/LIVE_SOURCE_CONTRACT.md).
- [Large course window reader](s3-line-barcode/LIVE_WINDOW_READER.md).
- [SHADOW live producer](s3-line-barcode/LIVE_PRODUCER.md).
- [SHADOW hardware results](s3-line-barcode/SHADOW_HARDWARE_RESULTS.md) — first full-circuit SHADOW run: system PASS, large A not decoded, findings and the proposal on hold.
- [LINE.md](s3-line-barcode/LINE.md) — current kernel pair-mode algorithm, P7 mm/s settings, search and revision-scoped results.
- [BARCODE.md](s3-line-barcode/BARCODE.md) — pure Code 39 WIDTH and B1c PEAK
  observation; one sheet, 430 mm TEST passes, checked stop, no routing.
- [USB_CAPTURE_WAIT.md](s3-line-barcode/USB_CAPTURE_WAIT.md) — TEST-only
  frozen report retention until the USB terminal asserts DTR for one second.

## S4 — IMU

- [IMU.md](s4-imu/IMU.md) — step 3 report-only hump detector (summary v3, event kind 9, USB/DUMP lines, synthetic fixtures, H1 STREAM hardware PASS on 1480b37; diagnostic C hardware on 29d57de; uncorrected LINE height accepted, corrected over-read under investigation (9/9 humps, 0 false in 5 flat); IMU stack free 480 B; H3 full-circuit no-false-hump acceptance compares source-labelled corrected/filtered images, default corrected, and is priority and Not run; accuracy follow-up deferred); step 2d 400 Hz FIFO batch mean into the unchanged step 2c roll-only RIGHT height, cited datasheet/AN3308 facts, alias fixtures, retired INT1 check and TEST raw capture; step 2c hardware NOT a height PASS; step 2d floor PASS for the 10 mm RIGHT-wheel hump only.

## S5 — Obstacle

- [OBSTACLE.md](s5-obstacle/OBSTACLE.md) — five brief stages, implemented 13-point scan, monitor/profile rules, reverse/reattempt, recovery limits, wide-cone finding; HC-SR04+ trigger/echo service, packed 16-bit result, 60–511 mm range, timeouts, bench/kernel sonar results and limits.

## Simulation

- [SIMULATION.md](simulation/SIMULATION.md) — implemented host model, interfaces, assumptions, echo-fit and box-sweep summaries; not kernel/track proof.

## Archive

- [OBSTACLE_BYPASS.md](archive/OBSTACLE_BYPASS.md) — shelved edge-following bypass v4.1, runs and unresolved issues.
- [OBSTACLE_DETOUR_HANDOFF.md](archive/OBSTACLE_DETOUR_HANDOFF.md) — earlier detour design, D1–D18 evidence and then-open problems.
- [STEER_WIFI_LOG.md](archive/STEER_WIFI_LOG.md) — earlier steering UDP log format and receiver.
- [VEHICLE_CONTROLLER.md](archive/VEHICLE_CONTROLLER.md) — preliminary controller design, superseded by the shared vehicle shell.

### S3 — Line and barcode history

- [s3-line-barcode/LINE_POSITION.md](archive/s3-line-barcode/LINE_POSITION.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/LINE_STEERING.md](archive/s3-line-barcode/LINE_STEERING.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/LINE_STEER_MM.md](archive/s3-line-barcode/LINE_STEER_MM.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/TWO_SENSOR_LINE_TRACING.md](archive/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/VC_LINE_RUN.md](archive/s3-line-barcode/VC_LINE_RUN.md) — full source record; only links/paths adjusted.
- [s3-line-barcode/BARCODE_READING_IDEAS.md](archive/s3-line-barcode/BARCODE_READING_IDEAS.md) — full source record; only links/paths adjusted.

### Simulation history

- [simulation/SIMULATION.md](archive/simulation/SIMULATION.md) — full source record; only links/paths adjusted.
- [simulation/SIM_ECHO_FIT.md](archive/simulation/SIM_ECHO_FIT.md) — full source record; only links/paths adjusted.
- [simulation/SIM_BOX_SWEEP.md](archive/simulation/SIM_BOX_SWEEP.md) — full source record; only links/paths adjusted.
### Kernel history

- [archive/kernel/HISTORY.md](archive/kernel/HISTORY.md) — `git show 569672a:<path>` pointers for the 19 removed kernel records (old robot bring-up, stacks, CPU, watchdog, layout, motion, USB priority, development setup, RAM budget per step, core placement/lock audit, UP shim, Wi-Fi stack, staging merge, bench, SWD).

### Telemetry history

- [archive/s1-telemetry/HISTORY.md](archive/s1-telemetry/HISTORY.md) — `git show 569672a:<path>` pointers for 9 removed telemetry records (OPERATION, first MQTT design and refill simulation, first telemetry owner, old dashboard and runbook, STEP 8 record and follow-up handoff, UDP dashboard, JSON schema).

### Architecture history

- [archive/architecture/HISTORY.md](archive/architecture/HISTORY.md) — `git show 569672a:<path>` pointers for CR1_EVIDENCE (renamed CR1.md), CR1_MOTION_PLAN (merged into CR1.md), the 1738-line Week 8 CR1 Tribunal notes, both BRANCH_LAYOUT records (current one merged into PROJECT_STATUS.md), the three ADRs (merged into ARCHITECTURE.md), and the old interrupt conventions, first vehicle shell, two-core control proposal and RTOS multicore plan.

### Obstacle history

- [archive/s5-obstacle/HISTORY.md](archive/s5-obstacle/HISTORY.md) — `git show 569672a:<path>` pointers for 6 removed obstacle records (ULTRASONIC, first ultrasonic driver with seven bench runs, monitor, scan/profile, R1–R4 and M2-reattempt detour runs, v1.6 detour test plan).

### Motion history

- [archive/s2-motion/HISTORY.md](archive/s2-motion/HISTORY.md) — `git show 569672a:<path>` pointers for 7 removed motion records (HARDWARE, first motor controller with B1–B3, 2026-10-30 calibration, 50-turn drift, DM1 method, bare-metal heartbeat, STEP 8 motion timing investigation).
- [USB_POWER_SAFETY.md](archive/s2-motion/USB_POWER_SAFETY.md) — obsolete by operator decision; retained evidence, not a current procedure.

## Original-template reference exceptions

Two untouched lecturer-template comments cite a missing `RELEASE.md` record:
[`sys_arch.h`](../lib/liblwip/sysdepend/pico_rp2040/include/arch/sys_arch.h) and
[`lwipopts.h`](../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h).
These are the two known old-path grep exceptions. Do not edit them as part of
a documentation move or add a ninth template deviation. The current ownership
rule is the Execution and hook section of the S1 MQTT design listed above.
