# Project status

One row per area, then the branch layout and merge rules. Read this page
before you propose work. Update the row in
the same commit as a change of state. Details stay in the linked records.

Baseline state date: 2026-11-16; staging tip at that writing: `a0cd97b`.
2026-11-17: staging integrates barcode v2 (`ad22f8f`), coarse/fine scan
with obstacle v3 (`7fe4054`) and the dashboard panels (`60b41f2`).
The scan row records the scoped `d44bd25` school floor result from that date;
it does not imply general geometry accuracy or acceptance of other changes.
2026-11-18: code freeze. The demo kernel is tag `demo-kernel-v1` (`9f15213`);
see [DEMO_FREEZE.md](../release/DEMO_FREEZE.md).

## Demo image

```sh
make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 [BARCODE_MAP="F=LEFT"]
```

The frozen live-barcode build line is in [DEMO_FREEZE.md](../release/DEMO_FREEZE.md#build).
Check `.ramproof` PASS and the boot line `[BARCODE_TABLE]`. See
[BUILD.md](../kernel/BUILD.md) and [KERNEL.md (KERNEL_RAM)](../kernel/KERNEL.md#ram-placement-kernel_ram).

## Areas

| Area | State | Evidence | Open |
| --- | --- | --- | --- |
| LINE (P7, two sensors) | Completes the course. Zigzags on straights: about +-40..54 deg/s, period about 0.4 s (2026-11-16). | [LINE.md](../s3-line-barcode/LINE.md) | Zigzag reduction (option C): real-log analysis first, then SIL, then tuning. LINE_P_NEAR (lower P) rejected; default P85 kept. LINE_BASE (83..378) override; base 120/100 were rejected on 2026-11-16 runs (D 85, no same-tick), but base 120 with kd 30 + same-tick is the demo default since 2026-11-17. |
| Demo default LINE (base 120, kd 30, same-tick) | Default since 2026-11-17 (`mtk3-demo-default`). B120_1: straight deviation median / p99 5.7 / 60.2 deg/s vs 9.3 / 77.2 at base 146; 0 faults. Overrides: `LINE_BASE`, `LINE_KD`, `MOTION_SAME_TICK=0`. | [LINE.md (settings)](../s3-line-barcode/LINE.md#settings) | Full course check with the default image (curves, junctions, detour, barcode SHADOW). kd 0 not run. |
| SEARCH / reacquire | Works on the course. SIL 6/6 reacquired. | [LINE.md](../s3-line-barcode/LINE.md) | None. |
| Motion controller | Done. Cap 460 mm/s, rise limit 8. | [MOTION.md](../s2-motion/MOTION.md) | Flat-floor motor test only if the zigzag analysis points at the motor. |
| Same-tick retarget (`MOTION_SAME_TICK`) | Default on for robot images since 2026-11-17; `=0` gives the old timing. Loop delay about 10 -> 2 ms. Robot runs 2026-11-16/17: lifted-wheel ESTOP OK, 0 overruns/faults, smoother small corrections. | [MOTION.md (same tick)](../s2-motion/MOTION.md#same-tick-retarget-default-on) | None. |
| Obstacle, scan, detour | Coarse/fine + v3 floor result on `d44bd25` (2026-11-17): same box met twice, 2519/2509 ms scans, clipped/valid geometry, 2/2 detours, no overruns/lost wakes/faults; task stack high water recorded. Prior fixed-scan result: 2/2 in each of 6 runs. | [OBSTACLE.md](../s5-obstacle/OBSTACLE.md), [scan validation](../s5-obstacle/COARSE_FINE_VALIDATION.md) | Separate stationary A2/A3 placements skipped by operator; width accuracy/passability not established; DONE_NO_LINE and STALL_RIGHT remain open. Lost sonar result fix (run LAST 2026-11-18, TURN_LIMIT after a 178° turn with no pings): host test only, not run on the robot. |
| Junction actions (B2a) | Live images (2026-11-18): real LEFT turns at 92–95 deg (LIVE4, GRACE1), RIGHT seen by operator, U-turn accepted at 172.5 deg. LIVE1/LIVE2 DONE events went straight (old spin). Fake-command images still use the old spin. | [DEMO_FREEZE.md](../release/DEMO_FREEZE.md), [JUNCTION_VALIDATION.md](../kernel/JUNCTION_VALIDATION.md) | Merge the live-only centre advance and 60 degree gate into the general junction action. U-turn: one pass only. |
| Barcode reader (B2b) | SHADOW only. Course 2026-11-16: 0 of 10 attempts decoded (robot zigzags). Straight TEST pass: large F decoded both ways. | [SHADOW_HARDWARE_RESULTS.md](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md), [BARCODE.md](../s3-line-barcode/BARCODE.md) | Size-tolerant reader (n 4..7 mm) in staging: identical to the old reader on 5 real captures; not proven on the course. Reliable read needs less zigzag. |
| Barcode failed-attempt evidence | Opt-in ROBOT software candidate, default OFF. Keeps first two failures of the run; edges and 4 ms ADC prefix dump after BC_REF. | [Failed capture](../s3-line-barcode/FAILED_ATTEMPT_CAPTURE.md) | Operator capture/image acceptance not run. Combined staging + KD30/same-tick RAM has30788 B link-time headroom including static stacks; no capacity fallback required. |
| Barcode letter table | Done: `BARCODE_MAP` make switch, boot print. | [BUILD.md](../kernel/BUILD.md), [LIVE_WINDOW_READER.md](../s3-line-barcode/LIVE_WINDOW_READER.md) | None. |
| Barcode LIVE (steers) | Opt-in, default OFF; the frozen demo build turns it on. Pending command expires at 1000 mm or after 200 ms continuous SEARCH (UNUSED 13/14). A decode continues through a natural SEARCH shorter than 200 ms. A failed control read uses the last good copy for up to 50 ms. GRACE1: 0 BP_DISABLED in 13 attempts. | [Live command path](../s3-line-barcode/BARCODE_LIVE_COMMAND.md), [DEMO_FREEZE.md](../release/DEMO_FREEZE.md) | A barcode entered at an angle (after a curve) often fails; place barcodes on straights. Full run with all features on the freeze image not done. |
| IMU humps | Report only. 2 humps per course run (2026-11-16). | [IMU.md](../s4-imu/IMU.md) | Extra IMU requirement: another team member works on staging. H3 Not run. `hump_imu_fault=1` once (2026-11-16 BASE run). |
| Telemetry, MQTT, dashboard | Run-summary counts implemented. Barcode terminal v2 (FIFO/QoS1, early reason/frame dump) and the v2/v3 dashboard panels are in staging (2026-11-17); host checks pass. | [TELEMETRY.md](../s1-telemetry/TELEMETRY.md), [Barcode run summary](../s3-line-barcode/BARCODE_RUN_SUMMARY.md), [Terminal attempts](../s3-line-barcode/BARCODE_ATTEMPT_TELEMETRY.md) | Operator UI check pending: attempts, queue loss, same-run counts and the obstacle v3 panel on the robot. |
| Kernel UP | Robot image. Overruns 0, lost wakes 0. | [KERNEL.md](../kernel/KERNEL.md) | F-NVIC and idle-stack fixes on HOLD. |
| KERNEL_RAM v2 | Robot PASS (rule b): peak load 676 vs 931 permille. | [KERNEL.md (KERNEL_RAM)](../kernel/KERNEL.md#ram-placement-kernel_ram) | Soft-float helpers stay in flash. |
| Dual-core split | PARKED. Demo uses UP. | [KERNEL.md (split parked)](../kernel/KERNEL.md#split-placement-parked-2026-11-15) | None until resumed. |
| CR1 change requirement | Motion round done. Round 2026-11-18 on `mtk3-cr1`: sonar/obstacle A and vehicle A1 merged, static RAM 241228 to 241144 B, API/wire unchanged, software checks pass. | [CR1.md](CR1.md#module-round-2026-11-18) | Barcode A, telemetry/faults and the 110 ms MQTT state period (operator requirement) in progress. Robot timing/stack run not done. IMU not started. |
| SIL (Webots) | LINE, SEARCH, curves, barcode A and F observe-only on `mtk3-hil`. | [SIMULATION.md](../simulation/SIMULATION.md) | Does not reproduce the real zigzag. Records live on `mtk3-hil`. |

## Robot run logs

Logs are local and not in Git. Paths are relative to the repository root of
the worktree named in each row.

| Date | Where | Image | Location |
| --- | --- | --- | --- |
| 2026-11-10 | unknown | B2a/B2b images | `build/mtk_logs/{s5_b2a_j5,sys_s1a,sys_s1b}_20261110_usb.txt` |
| 2026-11-11 | school | B2b SHADOW | `build/mtk_logs/b2b_shadow_F_20261111_usb.txt`, `robot-mqtt-20261111-163412.csv` |
| 2026-11-13 | school | `451e562` force0/split, P1 | `build/school_20261113/` (see `SESSION_NOTES.md`) |
| 2026-11-16 | school | `e06667d`, `2413c4a`, `b0c3a9a` BASE/RAM | `mtk3-core-review/build/kram_robot_<sha>/` |
| 2026-11-16 | home | `a0cd97b`, `7a6cb1c` | `mtk3-core-review/build/home_{straight,size}_<sha>/` |

The four 2026-11-10/11 USB files form one growing log: each file starts
with the full previous file. Count each KR dump once. The last
`=== connect` section of a robot log holds the GP20 dump.

## Branch layout and integration rules

This repository is an independent Git repository. School Repos records its
revision as a submodule. There are no nested repositories here. Work and
commit inside this repository before the parent updates its recorded revision.

### Branches

Rule from 2026-11-18: `mtk3-staging` combines all modules, `mtk3-testing`
holds the kernel tests, and `main` is the submission branch.

| Branch | Purpose |
| --- | --- |
| `main` | Submission branch. Receives `mtk3-staging` at a freeze (`--no-ff`). Tag `submission-v1`. |
| `mtk3-staging` | Integration branch: merge all module branches here. Demo kernel tag `demo-kernel-v1`. |
| `mtk3-testing` | Kernel test branch: SIL/HIL/PIL simulation (Webots, course8 SIL), bench kernel procedures. Merge staging into it before new test work. Do not merge it into staging without operator approval. |
| `mtk3` | Course-template/platform base |
| `mtk3-motion`, `mtk3-vehicle` | Motion and shared vehicle interface |
| `mtk3-line` | Line tracing, line search and the barcode reader |
| `mtk3-search`, `mtk3-scan`, `mtk3-detour` | Navigation modules |
| `mtk3-telemetry`, `mtk3-dashboard` | S1 transport, RAM log, boards, network and laptop dashboard |
| `mtk3-imu` | S4 IMU module |
| `ziven-barcode-algo`, `yanshan`, `jaison-imu`, `mtk3-imu-safety` | Team reference branches; port approved ideas, do not merge them |

The bench kernel switches (`BENCH_KERNEL`, `IMU_SAFETY_BENCH`) stay in
staging, default off, because the freeze keeps shared files unchanged.

Each module has its own development branch. Do module work and module-level
tests on that branch. Merge it into `mtk3-staging`, where the complete
system is integrated and tested. Use a temporary helper branch only for
work that changes many modules at once, for example a documentation
rewrite. Temporary helper branches are not new integration branches.
Create a fresh worktree from the assigned staging revision. Keep the main checkout and
another worker's branch untouched. Main performs hardware runs and pushes
unless it assigns those operations explicitly.

### Merge rules

1. Keep pure interfaces and logic independent of hardware and the kernel.
2. Test a module in its own worktree. Commit there, not in another branch.
3. Merge module/helper work into staging with `--no-ff`. Do not rebase.
   Before new work on a module branch, merge the current staging tip into
   that branch first (`--no-ff`, or a fast-forward when the branch has no
   own commits). This keeps the documentation layout and shared code
   current. On 2026-11-08 all module branches were fast-forwarded to the
   documentation rewrite (`c2b2154`).
4. If parallel changes must be combined, the assigned second merger merges
   the other tip with `--no-ff`, then repairs links and runs checks again.
5. Commit and push the child repository before updating its parent gitlink.
   A local helper's no-push instruction still applies.
6. Do not merge a reference branch. Keep author credit when porting ideas.

### MTK3 layout

| Path | Responsibility |
| --- | --- |
| `app_program/platform/` | Kernel-facing clocks, GPIO dispatcher, watchdog, observation boards and task support |
| `app_program/subsystem1/` | Telemetry and non-critical MQTT |
| `app_program/subsystem2/` | Motion task and pure motor control |
| `app_program/subsystem3/` | Line acquisition/service, pure steering and search |
| `app_program/subsystem4/` | TEST-only IMU probe/decode/register layers |
| `app_program/subsystem5/` | Sonar service and pure scan/detour logic |
| `app_program/vehicle/` | Shell, shared inputs, locked settings and robot glue |
| `src/` | Earlier bare-metal targets and portable code retained for benches/simulation |
| `build_make/` | Course-template firmware build and explicit SDK subset hook |
| `tests/`, `host-tests/` | Host tests; use `tests/CMakeLists.txt` for the full suite |
| `testing/` | Revision-scoped operator procedures and results |
| `sim/` | Separate host simulator, not kernel timing proof |
| `docs/` | Current topic records; full superseded texts under `archive/<topic>/` |

The [current architecture](ARCHITECTURE.md) states task/core ownership.
The root [README](../../README.md) gives the current firmware build commands.
Only the [template ledger](TEMPLATE_CHANGES.md) can authorize a template
change; its recorded deviation count remains eight.

### Checks before commit

Use strict host warnings. Build host tests with `cmake -S tests -B build/host`
and `make -C build/host -j4 --output-sync=target`; run
`ctest --test-dir build/host --output-on-failure`.
Run `python3 scripts/check-docs.py`,
`python3 scripts/check-template-changes.py` and `git diff --check` before
committing. A source/layout change must update the agent map in the same
commit. Keep private Wi-Fi/MQTT settings and generated output out of Git.

### Branch clean-up 2026-11-18

All helper branches were deleted after the freeze. Their commits are in
`mtk3-staging` (and so in `main`), or in `mtk3-testing`, or behind an
archive tag below. Nothing was rebased or force-pushed.

### Archived branches

Earlier branch histories remain reachable through these archive tags. The
historical layout record retains the complete commands and module results.

| Tag | Commit | Scope |
| --- | --- | --- |
| `archive/yx-staging` | f15ae89 | Original bare-metal module integration |
| `archive/yx-detour` | 3ef9e4b | Bare-metal line steering and detour R1–R4 |
| `archive/exp-detour-tuning` | 7c7c64d | Simulator/back-off experiments |
| `archive/yx-line-abstraction` | 249eac5 | Earlier motor/line manager |
| `archive/joseph` | ad7527d | Motion/speed reference history |
| `line-tracing-2sensor-v1` | e4ba9ec | Locked two-sensor duty design |
| `archive/mtk3-hil` | bf09e39 | SIL/HIL line before `mtk3-testing` (also in `mtk3-testing`) |
| `archive/mtk3-course8-sil` | 52051a5 | Course8 SIL demo-default cases (also in `mtk3-testing`) |
| `archive/mtk3-hil-sw4` | 48396c8 | Size-reader SIL gate record |
| `archive/mtk3-hil-sw5` | 3326d91 | Gate 1 trial C2/C3, not merged (Gate 1 closed) |
| `archive/mtk3-line-gap` | 24e2051 | `LINE_GAP=1` opt-in, rejected 2026-11-16 |
| `archive/mtk3-kd-st-trial` | 474cc70 | Throwaway KD + same-tick trial |
| `demo-kernel-v1` | 9f15213 | Frozen demo kernel 2026-11-18 |

### Branch limits

Branch names in old results describe their tested revision, not the active
checkout. A merge does not establish hardware acceptance. Use
[the integration log](../../testing/integration/TEST_LOG.md) and
[track results](../../testing/integration/KERNEL_ROBOT.md) for physical scope.
The original branch/layout record (`BRANCH_LAYOUT.md`, both versions) is in
the [architecture history](../archive/architecture/HISTORY.md#files).
