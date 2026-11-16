# KERNEL_RAM: kernel and ISR hot path in RAM (opt-in, UP only)

| Item | Record |
| --- | --- |
| Switch | `KERNEL_RAM=0` (default) or `1`. `1` needs `SMP=0`. The default adds no rule: ordinary images stay byte-identical (hash76). Image suffix `_kernel_ram`. |
| How | Before the link, `objcopy --rename-section` moves each listed section to `.data.ramfunc.*`. The unchanged template linker script copies `.data.*` to RAM. No source or template edit. |
| Lists | [`common.list`](../../tools/kernel_ram/common.list): dispatcher, interrupt entry, timer, `disint`/`ClearInt`, `tk_wup_tsk`, `tk_slp_tsk`, `tk_can_wup`, `tk_loc_mtx`, `tk_unl_mtx` and helpers, line and encoder ISRs, `drv_time_us`. [`robot.list`](../../tools/kernel_ram/robot.list): real line/GPIO/motor providers, sonar echo ISR, USB ISR and its transfer helpers. [`bench.list`](../../tools/kernel_ram/bench.list): bench providers (`BENCH_KERNEL=1`). |
| Left in flash | `rt_call`/`rt_max` (P1), `memcpy`/`memset`, `panic`, `rp2040_chip_version`, Wi-Fi/lwIP/MQTT (priority-20 poll task). |
| Proof | [`ram_proof.py`](../../tools/kernel_ram/ram_proof.py) runs after the link and fails the build if a listed symbol is not in RAM. `<image>.ramproof` lists the remaining flash callees and indirect calls. |
| Bench result (af2e787) | Line-path kernel calls 28.0 -> 14.3 us per ms, `[load]` 292 -> 243 permille, BC_CPU -34 %. Some waiting maxima rose (one run each); repeats with `[bench_top]` decide. |
| Acceptance | Robot P1 school run, BASE vs KERNEL_RAM: overruns 0, lost_wakes 0, `load_run` peak lower, every P1 max <= 1.25 x BASE or inside the bench spread, KR1 reset not watchdog. Then the operator picks the demo default. |
