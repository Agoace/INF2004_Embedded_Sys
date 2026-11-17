# Project status

One row per area, then the branch layout and merge rules. Read this page
before you propose work. Update the row in
the same commit as a change of state. Details stay in the linked records.

Baseline state date: 2026-11-16; staging tip at that writing: `a0cd97b`.
Scan candidate updated 2026-11-17 on `mtk3-scan-int`, based on staging
`b9a47b5`. This branch does not record a new hardware acceptance.

## Demo image

```sh
make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 [BARCODE_MAP="F=LEFT"]
```

Check `.ramproof` PASS and the boot line `[BARCODE_TABLE]`. See
[BUILD.md](../kernel/BUILD.md) and [KERNEL.md (KERNEL_RAM)](../kernel/KERNEL.md#ram-placement-kernel_ram).

## Areas

| Area | State | Evidence | Open |
| --- | --- | --- | --- |
| LINE (P7, two sensors) | Completes the course. Zigzags on straights: about +-40..54 deg/s, period about 0.4 s (2026-11-16). | [LINE.md](../s3-line-barcode/LINE.md) | Zigzag reduction (option C): real-log analysis first, then SIL, then tuning. |
| SEARCH / reacquire | Works on the course. SIL 6/6 reacquired. | [LINE.md](../s3-line-barcode/LINE.md) | None. |
| Motion controller | Done. Cap 460 mm/s, rise limit 8. | [MOTION.md](../s2-motion/MOTION.md) | Flat-floor motor test only if the zigzag analysis points at the motor. |
| Obstacle, scan, detour | Prior fixed-scan detours 2/2 in each of 6 course runs (2026-11-16). Coarse/fine + v3 geometry software candidate on `mtk3-scan-int`: 551 GCC/sanitizer tests, UP RAM proof PASS. | [OBSTACLE.md](../s5-obstacle/OBSTACLE.md), [scan validation](../s5-obstacle/COARSE_FINE_VALIDATION.md) | Stationary geometry/timing/stack check and two-detour regression not run; DONE_NO_LINE and STALL_RIGHT remain open. |
| Junction actions (B2a) | CLOSED. J1..J5 PASS on the floor with fake commands. | [JUNCTION_VALIDATION.md](../kernel/JUNCTION_VALIDATION.md) | Pending command has no timeout. J2 new pending policy Not run. |
| Barcode reader (B2b) | SHADOW only. Course 2026-11-16: 0 of 10 attempts decoded (robot zigzags). Straight TEST pass: large F decoded both ways. | [SHADOW_HARDWARE_RESULTS.md](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md), [BARCODE.md](../s3-line-barcode/BARCODE.md) | Size-tolerant reader (n 4..7 mm) in staging: identical to the old reader on 5 real captures; not proven on the course. Reliable read needs less zigzag. |
| Barcode letter table | Done: `BARCODE_MAP` make switch, boot print. | [BUILD.md](../kernel/BUILD.md), [LIVE_WINDOW_READER.md](../s3-line-barcode/LIVE_WINDOW_READER.md) | None. |
| Barcode LIVE (steers) | Not enabled. Three gates: build flag with measured clearance, producer valid flag, table. | [JUNCTION_LIVE_ADAPTER.md](JUNCTION_LIVE_ADAPTER.md) | Blocked by reliable read. |
| IMU humps | Report only. 2 humps per course run (2026-11-16). | [IMU.md](../s4-imu/IMU.md) | Extra IMU requirement: another team member works on staging. H3 Not run. `hump_imu_fault=1` once (2026-11-16 BASE run). |
| Telemetry, MQTT, dashboard | Run-summary barcode ok/failed implemented; host checks pass. | [TELEMETRY.md](../s1-telemetry/TELEMETRY.md), [Barcode run summary](../s3-line-barcode/BARCODE_RUN_SUMMARY.md) | Operator check of same-run barcode counts on the dashboard. |
| Kernel UP | Robot image. Overruns 0, lost wakes 0. | [KERNEL.md](../kernel/KERNEL.md) | F-NVIC and idle-stack fixes on HOLD. |
| KERNEL_RAM v2 | Robot PASS (rule b): peak load 676 vs 931 permille. | [KERNEL.md (KERNEL_RAM)](../kernel/KERNEL.md#ram-placement-kernel_ram) | Soft-float helpers stay in flash. |
| Dual-core split | PARKED. Demo uses UP. | [KERNEL.md (split parked)](../kernel/KERNEL.md#split-placement-parked-2026-11-15) | None until resumed. |
| CR1 change requirement | Motion round done. | [CR1.md](CR1.md) | Week 13: 110 ms telemetry, before/after tables. Sonar round waits for operator. |
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

| Branch | Purpose |
| --- | --- |
| `mtk3-staging` | Integrated robot program; the only module-combination branch |
| `mtk3` | Course-template/platform base |
| `mtk3-motion`, `mtk3-vehicle` | Motion and shared vehicle interface lineage |
| `mtk3-line` | Line tracing and line search; the barcode reader is an optional extension of this module |
| `mtk3-search`, `mtk3-scan`, `mtk3-detour` | Pure navigation module lineage |
| `mtk3-scan-int` | Approved helper for coarse/fine scan vehicle diagnostics, obstacle wire v3 and legacy-compatible decoder; Main merges into staging |
| `mtk3-telemetry`, `mtk3-dashboard` | S1 transport, RAM log, boards, network and laptop dashboard |
| `mtk3-imu` | S4 IMU module development |
| `mtk3-cr1`, `mtk3-b2b` | Helper branches: CR1 data representation; B2b barcode-to-junction (S3 reader + vehicle adapter). Merged into staging; kept as records |
| `main` | Original course-template history; do not change |
| `ziven-barcode-algo`, `yanshan`, `jaison-imu` | Reference branches; port approved ideas, do not merge them |

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

### Branch state 2026-11-16

Compared with `origin/mtk3-staging` `a0cd97b`. Update this table when a
branch changes role.

| Branch | Role | State |
| --- | --- | --- |
| `mtk3-staging` | Integration | Tip. Demo image source. |
| `mtk3-motion`, `-line`, `-search`, `-scan`, `-detour`, `-vehicle`, `-telemetry`, `-dashboard`, `-imu` | Module branches | In staging. Merge staging in before new work. |
| `mtk3-core-live` | Kernel timing, KERNEL_RAM | In staging. Active (SW7). |
| `mtk3-b2b` | Barcode reader and B2b helper | Ahead of staging: size-tolerant reader, not hardware-proven. |
| `mtk3-hil` | SIL (Webots) integration | Ahead of staging. Not for staging until the operator decides. |
| `mtk3-hil-sw4`, `-sw5-trial*`, `-sw6` | SIL worker branches | In `mtk3-hil`. |
| `mtk3-hil-sw5` | Gate 1 trial C1..C3 | Not merged. Record only (Gate 1 closed). |
| `mtk3-cr1`, `mtk3-detour-tri`, `mtk3-core_migration`, `mtk3-core-m8`, `mtk3-core-m9`, `mtk3-core-review-sw5*`, `mtk3-core-staging-merge`, `mtk3-bench-sw6b`, `mtk3` | Finished helper branches | In staging. Record only. |
| `mtk3-bench-sw6` | Bench record | In staging. Never merge again. |
| `main` | Original history | Diverged. Do not touch. |
| `ziven-barcode-algo`, `yanshan`, `jaison-imu` | Reference | Never merge. |

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

### Branch limits

Branch names in old results describe their tested revision, not the active
checkout. A merge does not establish hardware acceptance. Use
[the integration log](../../testing/integration/TEST_LOG.md) and
[track results](../../testing/integration/KERNEL_ROBOT.md) for physical scope.
The original branch/layout record (`BRANCH_LAYOUT.md`, both versions) is in
the [architecture history](../archive/architecture/HISTORY.md#files).
