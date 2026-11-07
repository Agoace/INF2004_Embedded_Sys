# Application task stacks

`app_start_task()` delegates to `platform/app_task.c`. It creates only
registered core-0 application tasks, using `TA_USERBUF`, static 8-byte
aligned storage, and a 0xa5 fill pattern before `tk_cre_tsk()`. Failed create
or start returns the error; failed start deletes the dormant task. No
kernel source is needed for this helper; approved shared changes are in
[the template ledger](TEMPLATE_CHANGES.md). Startup runs on core 0 before the
application tasks can run, so no allocation lock is needed.

| Task | Stack bytes |
| --- | ---: |
| motion_task | 3072 |
| line_task | 2048 |
| sonar_task | 2048 |
| vehicle_task | 4096 |
| telemetry_task | 4096 |
| motion_bench | 4096 |
| health_task, MQTT only | 2048 |
| idle_meter, MQTT only, priority 32 | 512 |

All six regions reserve 19456 bytes of static RAM even when a profile does
not start all six tasks. MQTT adds the health region, for 21504 bytes total;
the NORMAL MQTT robot uses 17408 bytes of these regions (bench is inactive).
These are initial conservative budgets, not
measured hardware needs. Sensor tasks do no formatting. Vehicle has extra
margin for nested module calls; telemetry and bench have extra margin for
`tm_printf()` (about 1 KB) and the stack-report copy. The motion region
has 3072 total bytes, 896 more than the earlier 2048 + 128 kernel-allocated
stack. TA_USERBUF sizes include the system/context budget; the kernel does
not add its default 128 bytes to a supplied region. Future tasks must
register a region.

`app_task_stacks()` scans upward from the low end of each downward-growing
stack. It records the largest observed use and never reduces that record.
`app_task_report()` prints size, high_water and free bytes for each started
application task. Less than 256 untouched bytes reports LOW MARGIN. The
kernel's initial, USB, radio and idle stacks are outside this helper; no
claim is made that this measures them. A fill-pattern scan is an estimate,
not an overflow detector or a proof of worst-case stack depth.

TEST output and the robot GP20 dump call the report. Hardware high-water
measurements passed for STEP 7, but remain **Not run** for the changed STEP 8
boards/network image. Health31 scans once per second outside driving tasks
and publishes the minimum free value; it does not measure template stacks. Before accepting the
budgets, run calibration, LINE/SEARCH/SCAN/DETOUR, saturation and a full USB
dump, then record each high-water mark with at least 256 bytes remaining.
Increase a budget if needed; do not infer runtime margin from link size.
Host doubles test alignment/fill, create/start failure, core affinity,
duplicate starts, monotonic high-water and report output.
