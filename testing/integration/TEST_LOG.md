# Integration test log

The old template demo interfaces are removed; current IF-01 through IF-05
are defined in [TEAM.md](../../TEAM.md). Earlier M1/M2, watchdog and
2026-11-06 task-split passes are recorded in [KERNEL_ROBOT.md](KERNEL_ROBOT.md).
Later full-course evidence is in
[the demo record](../../docs/release/DEMO_KERNEL_V2.md).

| ID | Check | Status |
| --- | --- | --- |
| R5 | [Kernel robot track check](KERNEL_ROBOT.md): line, two detours, SEARCH recovery, no contact/faults | Later course runs recorded, including image 30 run 6 (2026-11-20), 2/2 DONE_LINE. Open: complete R5 checklist. |
| S7-U01 | [Task-split acceptance](KERNEL_ROBOT.md#step-7-task-split-image-new-acceptance): cold TEST, timing/cost/stacks, queue saturation, last watchdog reset | PASS 2026-11-06 with 3f889a1. [Results](KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| M1-S7 / M2-S7 | Repeat two-tile line and obstacle checks on the five-task robot | PASS (basic) 2026-11-06. [Results](KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |

## STEP 8 earlier-image acceptance — operator results 2026-11-07

See [the hardware record](../../docs/archive/s1-telemetry/HISTORY.md#files) for exact
measurements and limits. These results do not validate later boot/session,
diagnostic or SCAN-retry changes. Later hardware evidence is recorded in
the release record above. Open: remaining STEP 8 gates on the later images.

| ID | Check | Result |
| --- | --- | --- |
| S0-U01 | Cold power robot/TEST/BENCH and MQTT: early TIMER1us, unchanged 2% rate gate; no Wi-Fi boot-order assumption | PASS cold-power TIMER error 0 on tested STEP 8 image; not a cold-power matrix of every later profile |
| M1 + MQTT | Independent lifted-wheel motor and stack/timing acceptance with link/broker on, off and reconnecting | Basic PASS: gap 9916..10087 us, unchanged behavior, live 216/216 rows exact; later ESTOP lifetime 11038 us needs diagnosis |
| M2 + MQTT | Independent local navigation/GP21/watchdog acceptance with live telemetry and dump; network never becomes a fault | Basic PASS at 16:13: LEFT, 22041 ms, 995 mm, lateral 318 mm, returned to line, losses 0; one-point clearance margin; earlier all-blocked scans correctly failed |
| S1-U04/U05/U06 | Network connect/rates/loss/CSV/commands, separately from M1/M2 | Tested reconnect/LWT, live log/DUMP and command cases PASS; full checklist coverage not claimed |

Software [validation](../../docs/archive/s1-telemetry/HISTORY.md#files) is not hardware
acceptance. No push or flash was performed by the software worker in this follow-up. The operator authorizes these
checks and records revision, profile, power source and duration.
