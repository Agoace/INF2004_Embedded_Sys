# Documentation — INF2004 Embedded Systems

Each record below has one entry: what it is for, then a **Contains:** line
with its topics, tables and key numbers. Use the agent map for source entry
points.
Archive records retain their original revision, evidence and limits; they are
not current acceptance reports. Local course originals are not public records.

## Navigation

- [README.md](README.md) — this index.
  Contains: one entry per record; the two template-comment exceptions.
- [AGENT_MAP.md](AGENT_MAP.md) — where to start in the source tree.
  Contains: map chain, doc directories, what each source group does, entry files per area, repository boundaries, template build layout, kernel platform/task notes.
- [Repository README](../README.md) — public setup, current build and usage.
- [Repository instructions](../AGENTS.md) — working rules and validation commands.

## Release

- [SUBMISSION_V2.md](release/SUBMISSION_V2.md) — the second submission of 2026-11-19.
  Contains: tag `submission-v2` on `main` = staging `1f46e88`; the staging merges since `submission-v1`; checks (host 722/722, RAM_PROOF 81/81, 0 warnings); robot evidence (images 12, 18, 19); known limits.
- [DEMO_FREEZE.md](release/DEMO_FREEZE.md) — the frozen demo kernel of 2026-11-18.
  Contains: tag `demo-kernel-v1` (`9f15213`), submission branch `main` (`submission-v1`); live-barcode build line and fallback; checks (host 677/677, 76/76 MATCH, RAM_PROOF 79/79, 0 warnings); image SHA prefixes; robot runs LIVE1–4, UTURN1, RIGHT1, GRACE1 (LEFT turns 92–95 deg, U-turn 172.5 deg); known limits (straight barcode placement, 1000 mm expiry, U-turn one pass, no full-feature run on this image).

## Project requirements

- [project-requirements/README.md](project-requirements/README.md) — our own course summary; original course files stay local and ignored.
  Contains: requirements to keep during implementation (data widths from valid range, hump range, evidence types), links to CR1 requirement and implementation records.

## Architecture

- [PROJECT_STATUS.md](architecture/PROJECT_STATUS.md) — read first: the state of each area.
  Contains: demo build line (`KERNEL_RAM=1`, `BARCODE_MAP`); area table (state, evidence, open items: LINE zigzag about ±40..54 deg/s, barcode 0/10 on course, detours 2/2 in 6 runs, KERNEL_RAM 676 vs 931 permille); robot run-log locations 2026-11-10..16; branch table (main = submission, mtk3-staging = integration, mtk3-testing = kernel tests), merge rules (`--no-ff`, no rebase), mtk3 source layout, commit checks, branch clean-up 2026-11-18, archive tags.
- [ARCHITECTURE.md](architecture/ARCHITECTURE.md) — the implemented robot system and its three decisions.
  Contains: subsystem ownership, tasks and cores, interrupt/polling rules, Vehicle Controller shell and route table, safety settings, results; ADRs in full: motor controller is the only actuator owner, one hardware heartbeat (bare-metal 1 ms time base), Vehicle Controller runs one module at a time (four layers, rules, outcome table).
- [MEASUREMENTS.md](architecture/MEASUREMENTS.md) — every physical/timing number, generated from the docs. Check it before asking the operator.
  Contains: one row per value (value, date, label, source line) in 9 areas: geometry, mass, motors and DM1, encoders, line/junction, GP28/barcode, sonar/servo/obstacle, IMU, timing/CPU/RAM/telemetry; values that do not agree, with resolved ones marked (for example counts 2540/2535, sonar offset 58/47 mm); the operator ruler session 2026-11-16 (length 180 mm, track 111-112 mm, wheel width 16 mm).
- [CR1.md](architecture/CR1.md) — Change Requirement 1 (minimum-bit data): requirement, evidence and motion plan.
- [CR1_TELEMETRY_A.md](architecture/CR1_TELEMETRY_A.md) — approved telemetry storage packing, diagnostic fault masks and T110.
  Contains: per-step RAM/build evidence, unchanged binary MQTT formats, 110 ms deadline and 120 ms poll model, existing FIX2/LAST references and pending 110 ms robot gate.
- [CR1_KR1_ROW.md](architecture/CR1_KR1_ROW.md) — approved KR1 RAM row storage 16 → 15 B with time guard; wire and USB text unchanged.
  Contains: field bit audit, 15-byte layout, index-anchored 13-bit time and guard, producer round-trip proof, APP_TEST row changes, RAM before/after, emulator lock-time cost.
- [CR1_BARCODE_A.md](architecture/CR1_BARCODE_A.md) — Private barcode evidence packing: representation, exact bits, RAM split, replay tests and pending robot timing.
  Contains: requirement and Week 13 deliverables; Week 8 §11 (8 B IMU snapshot, 1.40625° heading LSB) and §33–38 (13 states = 4 bits, mode/enum/fault bits, about 5-bit events, message rule); shared-access rule; baseline `168d705` RAM, board/queue and MQTT sizes; B1 (bench stack, 4112 B), B2 idle-stack margin, A1 board 72 → 64 B (static RAM 190672 → 190664 B), A1/B1 hardware PASS 2026-11-10, A2 16-bit counts plan, M9 accounting (+8336 B static); motion field table, controller/ISR inventory, tests.
- [CR1_SONAR_A.md](architecture/CR1_SONAR_A.md) — approved sonar and obstacle board storage only.
  Contains: field ranges, sonar board 44 to 40 B, history 704 to 640 B, obstacle board 36 to 28 B, baseline `7350da0`, static RAM 241228 to 241148 B, unchanged API/messages/wire, 679 host tests, torn-read tests, DERIVED(emulator) pack time, compiler-local frames; category B and private scan/detour packing remain deferred.
- [CR1_SONAR_F.md](architecture/CR1_SONAR_F.md) — sonar owner fault word (bit 5) for the F fault mask.
  Contains: single sonar-task writer, ECHO-only update from the existing fault streak, NO_RETURN not a fault, last value kept while stopped, tests, +8 B static RAM.
- [CR1_VEHICLE_A1.md](architecture/CR1_VEHICLE_A1.md) — approved Vehicle observation storage only.
  Contains: field ranges, single-writer board (44 to 40 B), baseline `7350da0`, static RAM 241228 to 241224 B, unchanged API/messages/wire, 680 host tests, compiler-local frames and hardware limits; private controller packing remains deferred.
- [CR1_VEHICLE_F.md](architecture/CR1_VEHICLE_F.md) — Vehicle owner fault word: IMU latch and other shell fault, single writer, red proof, tests, 8 B board cost and one STOP capture per run.
- [CR1_IMU_A.md](architecture/CR1_IMU_A.md) — approved IMU hump, diag and safety board packing (48/72/36 to 16/36/12 B), exact-output proofs, RAM and emulator timing.
- [CR1_MIN_BIT_PROOF.md](architecture/CR1_MIN_BIT_PROOF.md) — CR1 minimum-bit proof: per-field tables (range, n, minimum bits, bits used, reason) generated by `test_min_bit_proof`, byte totals, report-only gaps and the out-of-scope list.
- [JUNCTION_ACTION.md](architecture/JUNCTION_ACTION.md) — the B2a junction action consumer (fake commands).
  Contains: scope and approval (2026-11-10, closed at `50426ae`), command table NONE/A–D, ownership and entry points, fixed consumer value, junction detection, phases and bounds, reports and failures (event 7).
- [JUNCTION_LIVE_ADAPTER.md](architecture/JUNCTION_LIVE_ADAPTER.md) — the B2b barcode-to-junction adapter, SHADOW only.
  Contains: scope (SHADOW never offered to the consumer, valid 0), entry points, fixed boards and validation, encounter/route ordering, one slot and terminal ACK, build permission, diagnostics and remaining gates.
- [TEMPLATE_CHANGES.md](architecture/TEMPLATE_CHANGES.md) — the only list of allowed course-template changes; `check-template-changes.py` reads it.
  Contains: reference `c3b0954`; 8 deviations (`hw_setting.c`, `devinit.c`, `timer_tick.h`, `cyw43_utk_kernel.c`, `cyw43_utk.c/h`, `usb_console.c`, `mqtt_utk.c`); timing contract; cold-power guarantee; update and check procedure.

## Kernel

- [KERNEL.md](kernel/KERNEL.md) — the μT-Kernel robot: tasks, timing, RAM and locks.
  Contains: task/priority table (motion 10, line 11, sonar 12, vehicle 13, IMU 14, Wi-Fi 20, USB 21, telemetry 30, Health 31, idle 32), kernel objects, settings; application stacks (Wi-Fi M9 8320 B, IMU high-water); watchdog and last reset reason; CPU load; P1 run timing and P2 overrun timeline; KERNEL_RAM placement (rule b, robot P1 pairs); F-NVIC (all external IRQs at 0); split parked 2026-11-15, placement and lock rules, M8 startup handoff; RAM budget by group and growth; USB priority; results and limits.
- [BUILD.md](kernel/BUILD.md) — how to build, flash and read the firmware.
  Contains: tools, Make profiles and options, demo line, private settings, BOOTSEL/picotool flash, USB output, BENCH_KERNEL spare-Pico bench (pass rule), SWD probe (wiring, GDB on both cores), host tests and link proof.
- [JUNCTION_VALIDATION.md](kernel/JUNCTION_VALIDATION.md) — junction hardware evidence and gates.
  Contains: B2a CLOSED (`50426ae`, 3/3 DONE); B2b STEP4a software candidate; operator evidence 2026-11-10; software evidence; RAM and stack; J0–J6 procedure (J3/J4 operator evidence on `7648081`, J6 open).

## S1 — Telemetry

- [TELEMETRY.md](s1-telemetry/TELEMETRY.md) — the Wi-Fi/MQTT owner and how to run the laptop side.
  Contains: execution and hook, boards and queues, events and commands, publication settings (RUN/IDLE), RAM log and DUMP refill, results and limits; operation: broker, private settings, recorder and page, data panels (fan, hump, run path), Tailscale remote view, troubleshooting, tests.
- [MQTT_WIRE.md](s1-telemetry/MQTT_WIRE.md) — the binary wire contract, byte by byte.
  Contains: topic sizes (state 76, sonar 36, event 20, heartbeat 108, log 36 + 16 × n, cmd 22, obstacle 32, detour 36, summary 52, status 6; barcode v2 28 (legacy v1 20), reserved IMU 12); boot identity and sessions; field offsets per topic; publication modes; last will; command token; RAM log; client and host checks.

## S2 — Motion

- [MOTION.md](s2-motion/MOTION.md) — the motor controller, hardware and calibration.
  Contains: interface; settings (feed-forward, P/I, kick 12 %/10 counts/250 ms, stall, servo 1500 µs centre); kernel vs bench time base; heartbeat; pin table; geometry; 50-turn encoder ratio 2535 counts/turn (12414 counts/m); servo and sonar; unloaded RPM at 5 V; power status; DM1 duty map (23 levels, forward/reverse, 8 % running minimum, about 8.5 mm/s per %) and method; B1–B3 results (cap 460 mm/s); SDK subset.

## S3 — Line and barcode

- [LINE.md](s3-line-barcode/LINE.md) — the current two-sensor LINE and SEARCH.
  Contains: acquisition and calibration spin, pair detection, interface, settings (P7: base 146, min 62, max 378 mm/s, gains 85/85/340; locked duty P4 reference), results (P4–P9, kernel S3-U01/U02 1009 samples, M1-S7), limits (straight-after-curve swing).
- [BARCODE.md](s3-line-barcode/BARCODE.md) — TEST-only Code 39 reader (B1c WIDTH and PEAK).
  Contains: scope (one sheet, 430 mm pass at 100 mm/s, at most 450 mm), input and ownership, build and settings, driven procedure, snapshot health, capture/RAM, decoder rules and fixtures, PEAK contract. No routing.
- [USB_CAPTURE_WAIT.md](s3-line-barcode/USB_CAPTURE_WAIT.md) — TEST-only USB report hold.
  Contains: scope (`TEST=1 BARCODE_CAPTURE=1` only), operator procedure (DTR held for one second), implementation, validation.
- [LIVE_CALIBRATION.md](s3-line-barcode/LIVE_CALIBRATION.md) — GP28 evidence from the LINE spin (B2b step 2).
  Contains: status, unchanged spin controls, signal evidence and thresholds, report fields, SHADOW procedure, first home-mat result (`a883455`) and policy correction, software validation.
- [LIVE_SOURCE_CONTRACT.md](s3-line-barcode/LIVE_SOURCE_CONTRACT.md) — the S3-to-vehicle barcode boards.
  Contains: ownership (S3 writes record, vehicle writes control), final sizes (40 B record, 20 B control), five-word control, coherent read APIs, validation scope.
- [LIVE_WINDOW_READER.md](s3-line-barcode/LIVE_WINDOW_READER.md) — pure streaming reader for the course barcode, size-tolerant.
  Contains: inputs and bounded work, frozen thresholds and leading white, 29-element `*X*` window, one letter mapping table, size gates (narrow 4.0..7.0 mm fitted from the start star, wide by ratio 1.6..3.4, run ceiling min(35, 1.5w+|bias|) mm), historical evidence and limits.
- [LIVE_COUNTS_SNAPSHOT.md](s3-line-barcode/LIVE_COUNTS_SNAPSHOT.md) — read-only per-run barcode reader totals.
  Contains: ownership, meaning of frames/attempts/failures, epoch boundary and late-cancellation rule, return values, firmware matrix (+240 B text, +40 B BSS on 24 reader profiles), regression evidence.
- [BLIP_GRACE.md](s3-line-barcode/BLIP_GRACE.md) — live-only grace that keeps the SEEK white and a held LIVE record through a <200 ms SEARCH blip.
  Contains: rule (F1 freeze/thaw, F2 hold/retire), `[BC_SEEK_RESTARTS]` report counter, blip fixtures, no-blip identity, RAM (+208 B), limits; robot test pending.
- [REAL_WINDOW_REPLAY.md](s3-line-barcode/REAL_WINDOW_REPLAY.md) — old vs size-tolerant reader on the five real capture fixtures.
  Contains: result (identical on 15 rows; large F decoded, n/w 5.77/17.6 mm), reconstructed inputs and ASSUMED gap profile, native comparison table, regression lock.
- [SCHOOL_FAILED_START_ANALYSIS.md](s3-line-barcode/SCHOOL_FAILED_START_ANALYSIS.md) — FC1/FC2 capture integrity, quiet and motion diagnosis, source-timing limits, and approved changes.
- [FAILED_ATTEMPT_CAPTURE.md](s3-line-barcode/FAILED_ATTEMPT_CAPTURE.md) — opt-in ROBOT raw failure evidence.
  Contains: first two failed attempts per run, real threshold-crossing edges and 4 ms ADC prefix, bounded storage/ownership, stopped dump tags, combined staging RAM/headroom and KD30/same-tick compatibility, default-off 76 UF2/ELF-load gate.
- [BARCODE_ATTEMPT_TELEMETRY.md](s3-line-barcode/BARCODE_ATTEMPT_TELEMETRY.md) — immutable terminal attempts and early diagnostics.
  Contains: v2/kind1 28-byte MQTT contract, 8-slot FIFO/drop and QoS1 retry behavior, known/unknown widths, epoch reason histogram before KR, terminal timestamps, software gates and hardware limits.
- [BARCODE_RUN_SUMMARY.md](s3-line-barcode/BARCODE_RUN_SUMMARY.md) — barcode ok/failed in the MQTT run summary.
  Contains: meaning, ownership and run boundaries (epoch, busy final read keeps last same-run copy), validation, resource impact (+608 B text, +56 B BSS).
- [LIVE_PRODUCER.md](s3-line-barcode/LIVE_PRODUCER.md) — SHADOW producer that feeds GP28 samples to the reader.
  Contains: scope (one 988 B window context, no heap), threshold evidence, control and time evidence, one slot and boot serial, USB/DUMP output, CR1 memory and stack, next gate.
- [BARCODE_LIVE_COMMAND.md](s3-line-barcode/BARCODE_LIVE_COMMAND.md) — opt-in live command path from the barcode producer.
  Contains: valid=1 gates (build, custom map, mode2, mapped command), MQTT SHADOW bit, one-decode-one-command chain, live-only centred LEFT/RIGHT correction (131 mm advance, 60 degree gate), reopened physical B2a acceptance, V1 expiry (1000 mm from decode) and V3 cancellation after 200 ms SEARCH (UNUSED reasons 13/14), build line and host tests, OPERATOR course answers, vehicle items V1–V6, OPERATOR decision against D_cross runs or school-track transfer, approved option C placement rule (100 mm design guard, >=160 mm physical gap, no extra 100 mm margin), proposed variable rename, geometry budget and failure limits; live-only bounded control-read cache, SEARCH-only decode eligibility grace, and uncached final publication check.
- [BARCODE_SESSION_REPORT.md](s3-line-barcode/BARCODE_SESSION_REPORT.md) — GP20 log tool for per-attempt results and D_cross; dark-mark analysis.
  Contains: tool usage and rules (one dump per run, MEASURED vs DERIVED D_cross), results on FX_1/FX2/B120 (barcode 2 only, frame end to next dark pair, DERIVED 251–254 mm), dark-mark position/width table and operator checks.
- [SHADOW_HARDWARE_RESULTS.md](s3-line-barcode/SHADOW_HARDWARE_RESULTS.md) — first full-circuit SHADOW run.
  Contains: status (merged `b405aef`; table A=LEFT, B=RIGHT, C=STRAIGHT, D=UTURN), results (system PASS, large A not decoded), findings, proposal on hold, evidence.

## S4 — IMU

- [IMU.md](s4-imu/IMU.md) — the IMU hump detector (report only) and its full step history.
  Contains: step 3 report-only hump detector (summary v3, event kind 9, USB/DUMP lines, H1 STREAM PASS on `1480b37`, 9/9 humps and 0 false in 5 flat, IMU stack free 480 B, H3 Not run); step 2 data path, calibration, angle/height formulas; step 2b/2c (historical); step 2d 400 Hz FIFO batch mean, floor PASS for the 10 mm RIGHT-wheel hump only; datasheet/AN3308 audit; step 1 validation and raw-stream evidence.

- [IMU_SAFETY_PORT.md](s4-imu/IMU_SAFETY_PORT.md) — manual safety port and UP validation record.
  Contains: source attribution, pure detector contract, finite/age guards, trigger-count semantics, applied-request acknowledgement and integration steps; E: UP resource table per build profile (demo, capture, bench), ARM stack frames, spare Pico W BENCH1 (absent IMU), kernel services checked vs d3890d0, old-kernel numbers marked not valid; device acceptance not claimed.
- [IMU_SAFETY_BENCH.md](s4-imu/IMU_SAFETY_BENCH.md) — synthetic IMU safety bench runbook (TEST runner and ROBOT synthetic profiles), BENCH2 record and BENCH3 procedure.
- [IMU_SAFETY_CAPTURE.md](s4-imu/IMU_SAFETY_CAPTURE.md) — opt-in ROBOT IMU safety latch capture (`IMU_SAFETY_CAPTURE=1`): build, RAM, USB line format and the lift/overturn operator procedure.
  Contains: make guards and build lines, replay seams, 40 s phase table (OVERTURN, LIFT), spare Pico W procedure, USB lines to watch, pass conditions, limits (wall time, no I2C cost).

## S5 — Obstacle

- [OBSTACLE.md](s5-obstacle/OBSTACLE.md) — obstacle monitor, scan, avoidance and the ultrasonic service.
  Contains: forward monitor thresholds; coarse/fine scan with 13-bin decision completion, estimated width and observed edge-plane spans; mounted servo limits; profile and PROFILE event; avoidance with reverse and reattempt; line recovery; interface; results (R3/R4, M2 reattempt 5/5 free side at 329–435 mm); wide-cone limit; HC-SR04+ pins and timing (rise 2 ms, echo 3 ms, gap 60 ms), packed 16-bit result (60–511 mm), sonar results (front offset 57.9 mm, kernel S5-U01) and limits.

- [COARSE_FINE_VALIDATION.md](s5-obstacle/COARSE_FINE_VALIDATION.md) — scan software proof and scoped robot floor result.
  Contains: 551 GCC and sanitizer checks, 2560 ms host timing vs2230 ms baseline, v3 geometry/legacy decoding, stopped USB scan evidence, +620 B static RAM, and d44bd25 floor result (2026-11-17): 2519/2509 ms scans, clipped/valid geometry, 2/2 detours, task stack high water; separate stationary placements skipped.

## Simulation

- [SIMULATION.md](simulation/SIMULATION.md) — the host simulator: what it models and what it proves.
  Contains: layers, interface and operation, model assumptions, echo-fit summary (narrow box 160 × 250 mm, beam 5°, incidence 16°), box-sweep results (back-off 25°, gaps 22/54/91/91 mm), limits (bare-metal v1.8 sources, not kernel timing or physical proof).
- [COURSE_FIGURE8_20261117.md](simulation/COURSE_FIGURE8_20261117.md) — the final demo line track (figure-8) as a Webots course; tile map from photos, placement parameters for obstacle, ramp and barcode.

## Archive

Each `HISTORY.md` lists removed records. Read one with
`git show 569672a:<path>`.

### History pointers

- [archive/kernel/HISTORY.md](archive/kernel/HISTORY.md) — 19 removed kernel records.
  Contains: old robot bring-up, stacks, CPU, watchdog, layout, motion checks, USB priority, development setup, RAM budget per step, core placement/lock audit, UP shim, Wi-Fi stack, staging merge, bench step 0, SWD.
- [archive/s1-telemetry/HISTORY.md](archive/s1-telemetry/HISTORY.md) — 9 removed telemetry records.
  Contains: OPERATION, first MQTT design and refill simulation, first telemetry owner, old dashboard and runbook, STEP 8 record and follow-up handoff, UDP dashboard, JSON schema.
- [archive/s2-motion/HISTORY.md](archive/s2-motion/HISTORY.md) — 7 removed motion records.
  Contains: HARDWARE, first motor controller (B1–B3), 2026-10-30 calibration, 50-turn drift, DM1 method, bare-metal heartbeat, STEP 8 motion timing investigation.
- [archive/s5-obstacle/HISTORY.md](archive/s5-obstacle/HISTORY.md) — 6 removed obstacle records.
  Contains: ULTRASONIC, first ultrasonic driver (seven bench runs), monitor, scan/profile, R1–R4 and M2-reattempt detour runs, v1.6 detour test plan.
- [archive/architecture/HISTORY.md](archive/architecture/HISTORY.md) — removed and renamed architecture records.
  Contains: CR1_EVIDENCE (now CR1.md), CR1_MOTION_PLAN, Week 8 CR1 Tribunal notes (1738 lines, § index), both BRANCH_LAYOUT records, three ADRs, interrupt conventions, first vehicle shell, two-core control proposal, RTOS multicore plan.

### Kept in the tree (code cites them)

- [OBSTACLE_BYPASS.md](archive/OBSTACLE_BYPASS.md) — shelved edge-following bypass v4.1.
  Contains: assumptions, steps, steering, CR1 command word, failures, `bypass_test` bench, track tests, host test.
- [OBSTACLE_DETOUR_HANDOFF.md](archive/OBSTACLE_DETOUR_HANDOFF.md) — bare-metal detour v1.5–v1.8.
  Contains: v1.8 back-off at most 30°, v1.7 line only on return, v1.6 first run 2026-11-03, v1.5 design, track runs D1–D18, open problems, values to measure.
- [STEER_WIFI_LOG.md](archive/STEER_WIFI_LOG.md) — bare-metal steering UDP log.
  Contains: network (UDP port 4243), robot rules, packet format, use, host tests, track test.
- [VEHICLE_CONTROLLER.md](archive/VEHICLE_CONTROLLER.md) — preliminary controller, superseded by the vehicle shell.
  Contains: steps, step 1 line following, data representation, host test.
- [USB_POWER_SAFETY.md](archive/s2-motion/USB_POWER_SAFETY.md) — obsolete by operator decision; not a current procedure.
  Contains: limit, observed incident, old motor-test procedure, single-motor and spin-calibration exceptions, over-current recovery.

### Line, barcode and simulation originals (on hold until those areas merge)

- [LINE_POSITION.md](archive/s3-line-barcode/LINE_POSITION.md) — normalised line position.
  Contains: thresholds, vehicle events, data representation, host and bench tests, pair mode (2026-11-03).
- [LINE_STEERING.md](archive/s3-line-barcode/LINE_STEERING.md) — duty line steering.
  Contains: trim, limits, D part, far gain (2026-11-03), straight across a junction, calm mode, data representation, tests.
- [LINE_STEER_MM.md](archive/s3-line-barcode/LINE_STEER_MM.md) — steering in mm/s.
  Contains: rule, first track-run settings mapped one to one, host test.
- [TWO_SENSOR_LINE_TRACING.md](archive/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md) — the locked two-sensor duty design (`line-tracing-2sensor-v1`).
  Contains: hardware, method, evidence, limits, later work.
- [VC_LINE_RUN.md](archive/s3-line-barcode/VC_LINE_RUN.md) — Vehicle Controller line runs.
  Contains: layers, modules, spin calibration, log, host test, P5 procedure, runs P5/P6/P7 (2026-11-04), end state.
- [BARCODE_READING_IDEAS.md](archive/s3-line-barcode/BARCODE_READING_IDEAS.md) — early analog barcode ideas.
  Contains: known facts, several bars in view, line-tracing weave, barcode length, ideas, proposed chain.
- [archive/simulation/SIMULATION.md](archive/simulation/SIMULATION.md) — full original simulator record.
  Contains: separation from firmware, SDK surface, clock, world and physics, scenario/trace formats, viewer, editor, layout and commands, risks, operator decisions, narrow beam box, milestone 3, build plan.
- [SIM_ECHO_FIT.md](archive/simulation/SIM_ECHO_FIT.md) — echo-model fit to recorded scans.
  Contains: rule and settings, record constraints, native-probe comparison, checks.
- [SIM_BOX_SWEEP.md](archive/simulation/SIM_BOX_SWEEP.md) — fixed-front box-size sweep.
  Contains: scope and reproduction, results, main case, checks.

## Original-template reference exceptions

Two untouched lecturer-template comments cite a missing `RELEASE.md` record:
[`sys_arch.h`](../lib/liblwip/sysdepend/pico_rp2040/include/arch/sys_arch.h) and
[`lwipopts.h`](../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h).
These are the two known old-path grep exceptions. Do not edit them as part of
a documentation move or add a ninth template deviation. The current ownership
rule is the [Execution and hook](s1-telemetry/TELEMETRY.md#execution-and-hook) section of TELEMETRY.md.
