# Subsystem 1 and shared platform test log

Subsystem 1 remains a stub. Its TEST hook runs the shared timer check in
the initial task; it creates no new task.

| ID | Check and procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S0-U01 | TEST image, both wheels lifted: remove all power, then apply USB or VIN power. Compare TIMER time with kernel time over a nominal 1 s window. Repeat after cold power-on for every kernel image. | Prints TIMER us and elapsed kernel ms; ratio differs from 1000 us/ms by at most 1%; kernel errors fail the check | PASS 2026-11-06, warm and cold (see below) |

`subsystem1_test.c` calls `drv_time_check_window(1000, 1, ...)` and prints
one PASS/FAIL line before the S2 tests. The actual kernel window can be
about 1010 ms because `tk_dly_tsk()` adds one kernel tick. Compare measured
time, not the requested delay. `drv_time_check()` separately checks a
short window with a 2% limit before motion is enabled.

The original TIMER source defect was found after a cold VIN power-on on
2026-11-06. Post-flash tests did not expose it. Do not run the floor bench
until S0-U01 passes after a cold power-on. Lift both wheels before power:
the TEST image's later S2-U02 test sends a real short speed command.

## Hardware run records: 2026-11-06

Image: `mtk3` 1806ed0 / `mtk3-motion` f0b4194, `make SMP=0 WIFI=none TEST=1`.
For the cold run, a local debug copy added only a wait for the USB terminal
(up to 15 s) and two prints of the WATCHDOG_TICK register at the start of
`usermain()`. That copy was not committed. Without the wait, the TEST output
of the first seconds is lost before the host opens the port.

| Start | WATCHDOG_TICK before `drv_time_init()` | After | S0-U01 | S2-U01 task gap |
| --- | --- | --- | --- | --- |
| Warm, after a picotool flash | `0x0000260c` (CYCLES 12, set by boot ROM) | `0x0000560c` | PASS: TIMER 1009986 us, kernel 1010 ms | 9770..10317 us |
| Cold, USB plugged in after all power removed | `0x000eae00` (CYCLES 0: the defect) | `0x0006660c` (CYCLES 12) | PASS: TIMER 1009991 us, kernel 1010 ms | 9458..10548 us |

S2-U02 also passed in both runs. Log of the cold run:
`build/mtk_logs/s0_cold_dbg_20261106-165924.txt` (ignored, not committed).

The committed bench image was then started cold on VIN for floor run B4
([subsystem 2 log](../subsystem2/TEST_LOG.md)). Its TIMER-measured task gap
was 9952..10046 us against the 10 ms kernel cycle, so the fix also holds in
the normal image after a cold VIN power-on.

## Task-split telemetry checks

| ID | Procedure and acceptance | Status |
| --- | --- | --- |
| S1-U02 | TEST: bounded producer burst at priority 1 fills MBF; sends return without waiting, lost count increases; after allowing consumer to run, accepted rows are retained | PASS 2026-11-06 (445084c and 3f889a1): accepted 34, lost 66, no blocking. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| S1-U03 | GP20 dump after a full robot session: valid KR1/VE CSV, telemetry loss 0, dump `tm_usb_dropped` delta 0, control deadlines remain within limits, per-task stack free margin >=256 bytes | PASS 2026-11-06 (M1-S7, M2-S7): telemetry loss 0, USB dump delta 0, overruns 0, stack free >= 1672 B. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
