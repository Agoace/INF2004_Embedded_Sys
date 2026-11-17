# Kernel history

The kernel records below were removed from the tree on 2026-11-16 when
the kernel documents were merged into [KERNEL.md](../../kernel/KERNEL.md)
and [BUILD.md](../../kernel/BUILD.md). The current rules and numbers are
in those two files. Each removed file is still in Git, unchanged.

Read one file:

```sh
git show 569672a:docs/kernel/RAM_BUDGET.md
```

Commit `569672a` is on `origin/mtk3-staging`. Use the path from the table.
Relative links inside an old file can point to files that have moved.

## Files

| Path at `569672a` | What it holds | Now in |
| --- | --- | --- |
| `docs/kernel/KERNEL_RAM.md` | KERNEL_RAM switch, lists, proof, bench result, rule (b), robot P1 pairs 1-3 | [KERNEL.md](../../kernel/KERNEL.md#ram-placement-kernel_ram) (full text kept) |
| `docs/kernel/RAM_BUDGET.md` | Static RAM by group at `cr1-pre-imu`; per-step deltas for IMU 2/2b/2c/2d/3/diagnostic C, B2a, B2b and H3 with symbol tables, compiler frames and IMU CPU wall times; RAM-shortage steps; CR1 order | [KERNEL.md](../../kernel/KERNEL.md#ram-budget) (summary) |
| `docs/kernel/CORE_PLACEMENT.md` | Split migration phases 1-5, DI/EI inventory, lock audit per holder with line numbers and byte bounds (RT-0069/0071, B1, B2), lock stress test, performance gate RT-0085, reviews M2-M6, M8 startup handoff, M10 | [KERNEL.md](../../kernel/KERNEL.md#core-placement-and-locks) (rules, lock table, M8) |
| `docs/kernel/UP_SMP_SHIM.md` | M7: 16 UP lock/atomic functions, red 16/19 to green 19/19, ARM checks, +96 B text | [KERNEL.md](../../kernel/KERNEL.md#lock-rules) (summary) |
| `docs/kernel/WIFI_STACK_WATERMARK.md` | M9: static painted `cyw43` stack, sizes 6272/8320 B, RAM +8336 B static vs -8320 B pool, tests, hardware gate | [KERNEL.md](../../kernel/KERNEL.md#wi-fi-stack-m9) |
| `docs/kernel/CORE_STAGING_MERGE.md` | Merge of staging `c193fcd` into migration `8f224b2`: 517 tests, 38 profiles, ABI sizes, static RAM table (204468 B) | [KERNEL.md](../../kernel/KERNEL.md#static-ram-growth-mqtt-robot-same-public-example-settings) (one row) |
| `docs/kernel/NVIC_PRIORITY_FINDING.md` | F-NVIC: `EnableInt` byte store, IPR readback, effect, fix on hold | [KERNEL.md](../../kernel/KERNEL.md#interrupt-priorities-f-nvic) (full content) |
| `docs/kernel/SPLIT_PARKED_20261115.md` | Split parked: UP vs split bench and robot load, cause, step-0 UF2 hashes, next steps | [KERNEL.md](../../kernel/KERNEL.md#split-placement-parked-2026-11-15) |
| `docs/kernel/BENCH_KERNEL_TEST.md` | On-chip bench scope, statistics publication, report fields, virtual GPIO seam, ownership | [BUILD.md](../../kernel/BUILD.md#kernel-bench-image-bench_kernel) |
| `docs/kernel/BENCH_KERNEL_STEP0.md` | Step-0 build/run, report lines, pass rule, limits | [BUILD.md](../../kernel/BUILD.md#kernel-bench-image-bench_kernel) |
| `docs/kernel/SWD_DEBUG_PROBE.md` | SWD wiring, safety, OpenOCD start, memory reads, GDB, load | [BUILD.md](../../kernel/BUILD.md#swd-debug-probe) |
| `docs/archive/kernel/KERNEL_ROBOT.md` | STEP 7 task split, Fix 1 and PROBE A, ADC timing, STEP 7 sizes and results | History only |
| `docs/archive/kernel/APPLICATION_TASKS.md` | First static application stack registry | History only |
| `docs/archive/kernel/CPU_LOAD.md` | First idle-meter method and 2026-11-07 results | History only |
| `docs/archive/kernel/KERNEL_WATCHDOG.md` | First motion-only watchdog contract and S2-U05 | History only |
| `docs/archive/kernel/MTK3_LAYOUT_CHECKS.md` | Course template layout checks at `2117122` | History only |
| `docs/archive/kernel/MTK3_MOTION_CHECKS.md` | Step-1 motion bring-up, SDK proof, cold-power TIMER defect | History only |
| `docs/archive/kernel/MTK3_USB_PRIORITY.md` | USB task 3 -> 21 (STEP 8 option A) and its 2026-11-07 hardware result | History only; result in [KERNEL.md](../../kernel/KERNEL.md#results-and-limits) |
| `docs/archive/kernel/DEVELOPMENT.md` | Old CMake Pico W environment, GPIO probe, UDP benches, motor calibration targets | History only |

`docs/kernel/JUNCTION_VALIDATION.md` is not in this list. It stays until
the junction documents are merged.
