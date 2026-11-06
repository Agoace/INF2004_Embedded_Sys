# Subsystem 1 and shared platform test log

Subsystem 1 remains a stub. Its TEST hook runs the shared timer check in
the initial task; it creates no new task.

| ID | Check and procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S0-U01 | TEST image, both wheels lifted: remove all power, then apply USB or VIN power. Compare TIMER time with kernel time over a nominal 1 s window. Repeat after cold power-on for every kernel image. | Prints TIMER us and elapsed kernel ms; ratio differs from 1000 us/ms by at most 1%; kernel errors fail the check | Not run |

`subsystem1_test.c` calls `drv_time_check_window(1000, 1, ...)` and prints
one PASS/FAIL line before the S2 tests. The actual kernel window can be
about 1010 ms because `tk_dly_tsk()` adds one kernel tick. Compare measured
time, not the requested delay. `drv_time_check()` separately checks a
short window with a 2% limit before motion is enabled.

The original TIMER source defect was found after a cold VIN power-on on
2026-11-06. Post-flash tests did not expose it. Do not run the floor bench
until S0-U01 passes after a cold power-on. Lift both wheels before power:
the TEST image's later S2-U02 test sends a real short speed command.
