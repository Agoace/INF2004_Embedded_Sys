# Application task stacks

`app_start_task()` delegates to `platform/app_task.c`. It creates only
registered core-0 application tasks, using `TA_USERBUF`, static 8-byte
aligned storage, and a 0xa5 fill pattern before `tk_cre_tsk()`. Failed create
or start returns the error; failed start deletes the dormant task. No
kernel/template source is changed. Startup runs on core 0 before the
application tasks can run, so no allocation lock is needed.

| Task | Stack bytes |
| --- | ---: |
| motion_task | 2048 |
| line_task | 2048 |
| sonar_task | 2048 |
| vehicle_task | 4096 |
| telemetry_task | 4096 |
| motion_bench | 4096 |

All six regions reserve 18432 bytes of static RAM even when a profile does
not start all six tasks. These are initial conservative budgets, not
measured hardware needs. Sensor tasks do no formatting. Vehicle has extra
margin for nested module calls; telemetry and bench have extra margin for
`tm_printf()` (about 1 KB) and the stack-report copy. The earlier motion
budget of 2048 bytes is retained. Future tasks must register a region.

`app_task_stacks()` scans upward from the low end of each downward-growing
stack. It records the largest observed use and never reduces that record.
`app_task_report()` prints size, high_water and free bytes for each started
application task. Less than 256 untouched bytes reports LOW MARGIN. The
kernel's initial, USB, radio and idle stacks are outside this helper; no
claim is made that this measures them. A fill-pattern scan is an estimate,
not an overflow detector or a proof of worst-case stack depth.

TEST output and the robot GP20 dump call the report. Hardware high-water
measurements are **Not run** for the task-split image. Before accepting the
budgets, run calibration, LINE/SEARCH/SCAN/DETOUR, saturation and a full USB
dump, then record each high-water mark with at least 256 bytes remaining.
Increase a budget if needed; do not infer runtime margin from link size.
Host doubles test alignment/fill, create/start failure, core affinity,
duplicate starts, monotonic high-water and report output.
