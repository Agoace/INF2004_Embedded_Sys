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

## STEP 8 acceptance — Not run on the new image

| ID | Check | Result |
| --- | --- | --- |
| S0-U01 | Cold power robot/TEST/BENCH and MQTT: early TIMER1us, unchanged 2% rate gate; no Wi-Fi boot-order assumption | Not run |
| M1 + MQTT | Independent lifted-wheel motor and stack/timing acceptance with link/broker on, off and reconnecting | Not run |
| M2 + MQTT | Independent local navigation/GP21/watchdog acceptance with live telemetry and dump; network never becomes a fault | Not run |
| S1-U04/U05/U06 | Network connect/rates/loss/CSV/commands, separately from M1/M2 | Not run |

Software [validation](../../docs/MTK3_STEP8_VALIDATION.md) is not hardware
acceptance. No push or flash was performed. The operator authorizes these
checks and records revision, profile, power source and duration.
