# Integration test log

The template demo interfaces IF-01 through IF-05 are removed.
Earlier M1/M2 and watchdog M1 passes are recorded in
[KERNEL_ROBOT.md](KERNEL_ROBOT.md). New task-split checks remain Not run.
Interfaces are recorded in TEAM.md.

| ID | Check | Status |
| --- | --- | --- |
| R5 | [Kernel robot track check](KERNEL_ROBOT.md): line, two detours, SEARCH recovery, no contact/faults | Not run |
| S7-U01 | [Task-split acceptance](KERNEL_ROBOT.md#step-7-task-split-image-new-acceptance): cold TEST, timing/cost/stacks, queue saturation, last watchdog reset | PASS 2026-11-06 with 3f889a1. [Results](KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| M1-S7 / M2-S7 | Repeat two-tile line and obstacle checks on the five-task robot | PASS (basic) 2026-11-06. [Results](KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
