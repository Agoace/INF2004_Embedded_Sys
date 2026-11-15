# Native-motor SIL recovery grid — 2026-11-15

Same21 starts and10 s duration as the [old duty baseline](SIL_RECOVERY_20261115.md). New path: [actual P7 speed steering and native motor controller](SIL_MOTOR_FIDELITY_20261115.md). No controller tuning.

| Result | Old duty baseline | Native P7 + fitted motor |
|---|---:|---:|
| Completed / independently replayed |21/21|21/21|
| Terminal settled (<3 mm for final≥500 ms) |1/21|13/21|
| Initial poses outside sensor reach |2|2|

This is **not an isolated lag comparison**: steering changed from the old simulation-only duty settings to the robot's actual P7 speed settings; approximate sensor height also changed9→8 mm. It does not establish real robot recovery.

| Axle offset mm | Yaw ° | Settled from ms | Final pair error mm | Lost events |
|---:|---:|---:|---:|---:|
|0|0|1|0.89|0|
|0|5|1586|0.95|0|
|0|-5|1454|0.83|0|
|5|0|527|-0.58|0|
|5|5|—|-18.34|1|
|5|-5|1612|0.95|0|
|-5|0|308|0.94|0|
|-5|5|1791|-0.70|0|
|-5|-5|—|18.20|1|
|10|0|2615|0.91|0|
|10|5|—|-19.65|1|
|10|-5|2028|0.89|0|
|-10|0|2357|-0.95|0|
|-10|5|2053|-0.57|0|
|-10|-5|—|19.40|1|
|15|0|—|-19.59|1|
|15|5|—|30.48|1|
|15|-5|2386|0.95|0|
|-15|0|—|19.17|1|
|-15|5|2418|-0.94|0|
|-15|-5|—|-28.55|1|

## Evidence

- First grid: `build/webots/sil-recovery-motor-final-20261115T122522Z/`.
- Repeat: `build/webots/sil-recovery-motor-repeat-20261115T124126Z/`.
- All21 sample CSV SHA256 values match across runs;42/42 clean experiments and C replays. All owned process groups gone. This demonstrates repeatability only in the current environment.
- Eight losses latch stop; no search or recovery state machine. `SIL_RECOVERY_EXPERIMENT_COMPLETE` means recording completed, not that recovery succeeded.
- Full suite313 OK/3 expected failures; unchanged native motor tests pass; frozen158 RX/58 TX MATCH; docs and whitespace checks pass. No hardware access. **NOT GATE1 PASS**.
