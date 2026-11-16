# KERNEL_RAM: kernel and ISR hot path in RAM (opt-in, UP only)

Demo image (operator decision): `make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1
KERNEL_RAM=1`; `<image>.ramproof` must say `RAM_PROOF PASS listed 75 in_ram 75`.

| Item | Record |
| --- | --- |
| Switch | `KERNEL_RAM=0` (default) or `1`. `1` needs `SMP=0`. The default adds no rule: ordinary images stay byte-identical (hash76). Image suffix `_kernel_ram`. |
| How | Before the link, `objcopy --rename-section` moves each listed section to `.data.ramfunc.*`. The unchanged template linker script copies `.data.*` to RAM. No source or template edit. |
| Lists | [`common.list`](../../tools/kernel_ram/common.list): dispatcher, interrupt entry, timer, `disint`/`ClearInt`, `tk_wup_tsk`, `tk_slp_tsk`, `tk_can_wup`, `tk_loc_mtx`, `tk_unl_mtx` and helpers, line and encoder ISRs, `drv_time_us`. [`robot.list`](../../tools/kernel_ram/robot.list): real line/GPIO/motor providers, sonar echo ISR, USB ISR and its transfer helpers; v2 adds the line-task per-sample path (line sample, live barcode, GP28 window geometry, line position). [`bench.list`](../../tools/kernel_ram/bench.list): bench providers (`BENCH_KERNEL=1`). |
| Left in flash | `rt_call`/`rt_max` (P1), `memcpy`/`memset`, `panic`, `rp2040_chip_version`, Wi-Fi/lwIP/MQTT (priority-20 poll task), soft-float, integer divide and `cosf`/`sinf` (toolchain archives; the rule renames only project objects). |
| Proof | [`ram_proof.py`](../../tools/kernel_ram/ram_proof.py) runs after the link and fails the build if a listed symbol is not in RAM. `<image>.ramproof` lists the remaining flash callees and indirect calls. |
| Bench result (af2e787) | Line-path kernel calls 28.0 -> 14.3 us per ms, `[load]` 292 -> 243 permille, BC_CPU -34 %. Repeats (e06667d, `[bench_top]`): every cost top-20 is lower; waiting tails rose (one `line_wake` event per run, `read_line` overlap with one line sample), all < 0.3 ms. |
| Acceptance (operator, rule b) | Robot P1 school run, BASE then RAM, same course; swap order on a repeat. PASS = overruns 0, lost_wakes 0, `load_run` peak lower, step and wake max <= BASE, KR1 reset not watchdog. Waiting tails (`line_wake`, `read_line`, response) pass if < 0.5 ms. Basis: 6 bench runs (3 per image, 2 Picos), all COUNT_MATCH, overruns 0, `[load]` 302-304 vs 238-240 permille; logs `~/Projects/mtk3-core-review/build/kernel_ram_e06667d/step0_{B1.final,B2,B3,R1,R2,R3}.log`. |

## Robot P1 pairs

Logs: `~/Projects/mtk3-core-review/build/kram_robot_<commit>/` (Main's
checkout). All runs: overruns 0, lost_wakes 0, faults 0, KR1 reset power/RUN.

| Pair (commit, order) | RAM list | load_run peak BASE / RAM | wake max BASE / RAM (us) | line_wake max BASE / RAM (us) | Rule (b) |
| --- | --- | --- | --- | --- | --- |
| 1 (e06667d, BASE first) | v1 | 923 / 811 | 1419 / 1458 | 394 / 539 | FAIL: wake, line_wake |
| 2 (2413c4a, RAM first) | v1 | 947 / 798 | 1328 / 1459 | 356 / 455 | FAIL: wake (+9.9 %) |
| 3 (b0c3a9a, BASE first) | v2 | 931 / 676 | 2043 / 1189 | 600 / 136 | PASS |

v1 cause: wake max = one worst line sample + ~0.4 ms; the barcode path, still
in flash, was slower on the v1 image (933-1016 vs 770-775 us), probably flash
layout. v2 moves the line-task per-sample path to RAM: line sample max 1329 ->
854 us, barcode max 1229 -> 801 us, every step max lower.
