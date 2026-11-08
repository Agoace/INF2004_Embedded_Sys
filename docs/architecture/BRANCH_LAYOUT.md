# Branch layout and integration rules

## What it is

This repository is an independent Git repository. School Repos records its
revision as a submodule. There are no nested repositories here. Work and
commit inside this repository before the parent updates its recorded revision.

## How it works

| Branch | Purpose |
| --- | --- |
| `mtk3-staging` | Integrated robot program; the only module-combination branch |
| `mtk3` | Course-template/platform base |
| `mtk3-motion`, `mtk3-vehicle` | Motion and shared vehicle interface lineage |
| `mtk3-line` | Line tracing and line search; the barcode reader is an optional extension of this module |
| `mtk3-search`, `mtk3-scan`, `mtk3-detour` | Pure navigation module lineage |
| `mtk3-telemetry`, `mtk3-dashboard` | S1 transport, RAM log, boards, network and laptop dashboard |
| `mtk3-imu` | S4 IMU module development |
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

## Interface and merge rules

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

## MTK3 layout

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

## Settings and checks

Use strict host warnings. Build host tests with `cmake -S tests -B build/host`
and `make -C build/host -j4 --output-sync=target`; run
`ctest --test-dir build/host --output-on-failure`.
Run `python3 scripts/check-docs.py`,
`python3 scripts/check-template-changes.py` and `git diff --check` before
committing. A source/layout change must update the agent map in the same
commit. Keep private Wi-Fi/MQTT settings and generated output out of Git.

## Results and archived branches

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

## Limits and links

Branch names in old results describe their tested revision, not the active
checkout. A merge does not establish hardware acceptance. Use
[the integration log](../../testing/integration/TEST_LOG.md) and
[track results](../../testing/integration/KERNEL_ROBOT.md) for physical scope.
The [original branch/layout record](../archive/architecture/BRANCH_LAYOUT.md)
retains the detailed earlier branch and source-set history.
