# Kernel robot program

## Scope and safety

STEP 7 separates acquisition, navigation and telemetry into core-0 tasks.
Pure LINE/SEARCH/SCAN/DETOUR modules, routes, motor tuning and
[`vehicle_robot_config.h`](../app_program/vehicle/vehicle_robot_config.h)
are unchanged. The configuration contract compares the reference block
in [`src/vc_line_test.c`](../src/vc_line_test.c) exactly (NULL becomes 0).
Earlier watchdog-image S2-U05 and M1 passes do not validate this new image.
All new task-split hardware checks are **Not run**. No flash or push was
performed by this change. Build success does not approve floor driving.

Lift both wheels before TEST power. S2-U02 briefly drives real motors;
S2-U05 runs LAST and intentionally resets, then reports the previous reset
on the next boot without repeating it. Read the
[watchdog procedure](KERNEL_WATCHDOG.md#s2-u05-two-boot-test-procedure).
Normal GP21 starts spin calibration, waits 1 s, then drives for at most
90 s. GP21 during calibration/run latches ESTOP. GP20 requests a saved RAM
dump only while idle. Do not reset before saving. Use VIN without USB for
track work; see [USB power safety](USB_POWER_SAFETY.md).

## Selection and ownership

```sh
cd build_make
make SMP=0 WIFI=none -j4            # robot
make SMP=0 WIFI=none BENCH=1 -j4    # motion bench only
make SMP=0 WIFI=none TEST=1 -j4     # subsystem tests, wheels lifted
make -j4                           # robot, default SMP/radio profile
```

The following application tasks are pinned to physical core 0 (`TP_PRC1`)
in both profiles; the SMP build does not move application work to core 1.
Lower priority numbers run first. USB/radio template tasks remain above
application tasks. No template file is changed.

| Task/context | Priority | Release | Owns / kernel objects | Stack bytes |
| --- | ---: | --- | --- | ---: |
| motion_task | 10 | cyc_motion, 10 ms | Motor/servo PWM, encoders, stale stop; inherited mtx_motion; sole watchdog feed | 3072 |
| line_task, S3 | 11 | Every 1 ms ADC ISR wake | Raw queue consumer, lp/filter/calibration data; inherited snapshot mutex | 2048 |
| sonar_task, S5 | 12 | Request or echo wake; 10 ms deadline cyclic | Trigger/echo; request MBF 256 bytes, result mutex and event flag | 2048 |
| vehicle_task | 13 | Motion after-tick notification, 10 ms | Spin behaviour, buttons, one active module, service requests, motion commands | 4096 |
| telemetry_task, S1 | 30 | Receive mbf_telemetry, event-driven | RAM rows/events and paced dump; MBF 4096 bytes; producer-loss counters use DI/EI | 4096 |
| motion_bench, exclusive BENCH image | 11 | Motion after-tick notification | Earlier bench buttons/log; never runs with vehicle | 4096 |
| ADC TIMER alarm0 / IRQ0 | IRQ level 2 | Absolute 1 ms targets | Eight conversions/channel, timestamped 128-slot queue, loss/ISR timing counters | Interrupt stack |
| Encoder/echo GPIO13 | IRQ level 1 | Edges | Shared dispatcher, counts and rise/fall timestamps; echo service wake | Interrupt stack |

Robot totals: **5 application tasks, 3 mutexes, 2 cyclic handlers, 2 message
buffers and 1 result event flag**. TEST starts the four service/motion tasks,
not vehicle or bench. BENCH starts only motion/bench and leaves ADC/sonar
and telemetry idle. The one motion notification slot remains exclusive to
vehicle or bench. All task functions loop forever or use `tk_ext_tsk()`;
none returns. Existing kernel limits suffice; no template limits change.

USB IRQ5 stays at priority 0; SysTick/PendSV use priority 3. Only core 0
enables application ADC/GPIO interrupts. The installed HLL IRQ wrapper
enters task-independent context; `tk_wup_tsk(explicit_id)` is supported
there. ISR wakes only release task work, never run lp/navigation or format.
Periodic work does not use `tk_dly_tsk()`. USB dump pacing and TEST waits
are not periodic releases.

## Line observations and calibration

ADC alarm targets advance by 1000 us; late periods count as missed, not
replayed. An already-passed target forces the IRQ to avoid a TIMER-wrap
stall. The 128-slot queue has 127 usable entries. DI/EI protects one queued
sample or statistics copy; no cross-core synchronization is claimed.
The ISR now averages eight conversions each on L/ADC1, C/ADC0 and barcode/
ADC2 in that order, about 48 us of conversion time before software overhead.
Actual duration must be measured. The GP28 wiring/bar constraints are in
[Control architecture](CONTROL_ARCHITECTURE.md); no barcode decoder or
Ziven files change. The template `adca` device stays unopened; ADC3/GP29 is
never selected. CLK_ADC still uses the checked 48 MHz PLL_USB source.

Each ISR wakes line_task, rather than waiting for a 10 ms kernel tick.
The task processes every timestamped sample, not just the newest. This
preserves 1 kHz lp filtering and learning and minimizes snapshot age; cost
is one task release per sample and must be measured. Under an inherited
mutex it publishes `line_snapshot_t`: raw L/C, separate raw_bc, lp-compatible
state, latched last LOST/FOUND event, validity, sample timestamp/sequence,
processed/acquisition/missed/dropped counts and timing counters. Pure lp
calls receive **L/C/zero**, never the barcode value. Vehicle reads a short
copy and is the only event consumer. No lock covers a motion command or
navigation step.

The service owns lp calibration min/max/sums. Vehicle controls calibration
begin/end/abort through short mutex APIs and keeps the physical spin,
quarter-wheel goals, independent wheel stops, 131 mm/s, 6 s limit, 200 ms
pauses, return-home and centre peak/drop thresholds. The snapshot carries
the maximum centre norm since calibration end for equivalent per-sample
peak tracking. Abort retains the previous learned ends. lp learning also
continues outside navigation, including during telemetry dumps.

**Only the LINE snapshot has the 10 ms age limit.** A missing/invalid line
snapshot, age >10000 us or new missed/dropped acquisition stops an active
session. This is a runtime check, not a proven worst-case scheduling bound.
No ultrasonic 10 ms age rule is applied.

S3-U02 prints per-sample maximum and accumulated line processing elapsed
us, plus the window duty fraction in per-mille, and the corresponding ISR
max/total/window fraction. The task interval includes mutex entry, lp work,
copy and statistics; it excludes queue removal, mutex exit, scheduler and
wake overhead. Higher-priority preemption is included. It is instrumented
processing wall cost, **not exact total CPU utilization**. ISR measurements
exclude wrapper entry/exit and final counter stores. TIMER resolution is
1 us; accumulated unsigned counters use window subtraction. Hardware values
remain Not run; do not substitute the 48 us conversion estimate for them.

## Ultrasonic request/result service

`sonar_request()` copies bounded requests into an MBF with TMO_POL.
Requests carry mode, ID, generation, angle, not-before time, interval and
absolute deadline. A second outstanding one-shot in the same generation is
rejected, not overwritten. A full MBF returns an error; vehicle stops its
session rather than waiting. The service alone calls trigger/service/busy.

- WATCH keeps the requested interval and hardware minimum 60 ms. A result
  must be acknowledged before another trigger, preserving the no-ping-over-
  unconsumed-result rule. Its result age limit is publication time plus
  twice the interval and 20 ms, not the line age limit.
- ONCE is used at each SCAN measurement point. The vehicle passes its old
  last-ping/not-before timing and a 100 ms deadline, enough for the 60 ms
  minimum interval and echo timeout. Deadline failure stops the session.
- STOP/new generation cancels logical ownership. A physical old ping may
  finish, but its generation/ID is checked before publishing. Late results
  cannot become a new module's observation. Separate trigger generation/
  ID/angle fields make ping_sent a **real trigger acknowledgement**, never
  queue admission or a relabeled old trigger.
- Result copies use an inherited mutex; an event flag signals new results.
  Vehicle checks generation, ID, angle and deadline before consumption and
  acknowledges WATCH results. The 2 ms rise deadline remains enforced by
  the edge ISR even if the service runs late. Echo/no-rise/no-return/fault
  packing and failure streaks are unchanged. **No echo is unknown, never
  clear.** GPIO edges wake the service; a 10 ms cyclic services deadlines
  when no edge arrives, with no busy wait through an echo interval.

## Vehicle timing and safety

Vehicle priority changes from 12 to 13 so sonar12 can acknowledge/publish
before navigation. Vehicle reads only service snapshots, sends requests
and telemetry, steps the existing bounded module shell and calls motion.
Only button GPIO remains directly accessible here. There is no ADC queue
consumption, ultrasonic polling or RAM array/dump loop in vehicle.

Each iteration records wake-to-finish elapsed time and compares motion tick
counts at dispatch and completion. A new tick before completion or elapsed
>=10 ms increments overruns; an active session stops. `response_max_us` in
the dump is the maximum **wake-to-finish** elapsed interval, not a measured
cyclic-release-to-finish response. Queued/lost motion wakes remain a separate
counter and stop condition. Old releases are not replayed.

Commands still usually apply on the following motor tick because vehicle
runs after motion, unlike the bare-metal phase-9 ordering. The synchronous
latest-value motion API has timestamp expiry and no drive-command backlog;
this change adds no queued drive epoch protocol. GP21 is still polled at
20 ms, then uses immediate motion ESTOP; it is not a new hardware ESTOP IRQ.
The [100 ms hardware watchdog](KERNEL_WATCHDOG.md) remains fed **only by
completed motion ticks**. Vehicle failure is also covered by stale command
expiry; sensor age/generation checks cover service failure. This is not a
new multi-task health-vote watchdog. Debug halt deliberately pauses it.

## Plan versus port

| Plan sections 2–8 | STEP 7 implementation / difference |
| --- | --- |
| Core-0 sensor/control/telemetry tasks | Implemented; priorities 10/11/12/13/30 explicitly approved |
| Line task target 1 ms | Hardware ISR acquires; per-sample wake runs lp task work; cost/age instrumentation added, hardware bound not yet measured |
| Observations under short RTOS locks | Inherited line/sonar/motion mutexes; barcode raw carried separately |
| Asynchronous active sensor IDs/generation/deadline | Request MBF, correlated result snapshot plus event flag; old-generation results rejected |
| Section 7 service-owned servo/settle | **Approved difference:** pure SCAN settle logic and settings unchanged; servo ownership stays with motion. Service only pings; no second settle delay |
| Vehicle steps one module, bounded handoffs | Existing pure shell and routes unchanged |
| Lower-priority telemetry, ~110 ms target | Message-driven priority30 owner drains when ready instead of waiting 110 ms; no MQTT/network calls |
| Drive expiry and safety | Existing latest-value/stale-command/ESTOP path retained; no new drive mailbox/generation protocol |
| Critical-task progress watchdog | Explicit approved motion-only feeding; stale command, line age and sonar deadlines stop bad sessions |
| Timing/stack evidence | Instrumentation and host doubles implemented; ISR/processing/stack/physical deadline evidence remains Not run |

## KR1 rows and VE events

S1 telemetry alone owns the 4501 x16-byte rows (72016 bytes) and 512
x12-byte VE events (6144 bytes). Vehicle packs a local row, then sends a
by-value nonblocking message. Full MBF/storage increments lost_telemetry;
a producer failure/storage-full stops the active session, never blocks
control. A BEGIN clears saved indices, not lifetime loss counts. GP20
sends a DUMP command and diagnostic copy; priority30 owns formatting and
paced USB output. No new run starts while a dump is queued/active. See
[telemetry contract](KERNEL_TELEMETRY.md).

`KR index hex` keeps the exact 16-byte little-endian mask layout:

| Bits | Value |
| --- | --- |
| 0..15 | Elapsed 20 ms units |
| 16..17 / 18..20 | State / module |
| 21..32 / 33..44 | L / C raw means |
| 45..52 | Signed position |
| 53..62 / 63..72 | Asked L/R mm/s |
| 73..82 / 83..92 | Measured L/R mm/s |
| 93..108 | Complete ultrasonic result word |
| 109..115 | Signed servo angle |
| 116..123 | LOST=1, FOUND=2, echo=4, ping=8; outcome in bits4..6; motor fault=128 |
| 124..127 | Saturation flag bit0; other three bits spare |

There is no spare byte for barcode ADC; raw_bc is **not** added to KR1.
`VE time_ms kind from to word` keeps kinds 1 outcome, 2 state/module,
3 motor fault, 4 shell end and 5 session end. Every outcome, including
multi-hop same-tick handoffs, remains a separate VE message. Rows start
at shell start; calibration retains events/status but no navigation rows.

The descriptive KR1 header retains cached reset classification and adds
telemetry losses, vehicle overruns/wake-to-finish maximum, ADC/line timing,
raw_bc and per-application-task stack reports. The footer retains
`tm_usb_dropped` total/dump delta. The decoder ignores descriptive text:

```sh
python3 scripts/robot_log_decode.py capture.txt > rows.csv
python3 scripts/robot_log_decode.py --events capture.txt > events.csv
```

Unknown echo distance stays blank; VALID code0 means TOO_NEAR, not 0 mm.
RAM logs are lost on reset. Check row indices and USB dump delta.

## Stack and hardware acceptance

[Static application stacks](APPLICATION_TASKS.md) reserve 19456 bytes for
all six registered task regions, including the mutually exclusive bench.
The robot uses 15360 of these bytes for its five active stacks. Pattern
high-water reports cover application tasks including motion, not template
initial/USB/radio/idle stacks. Initial budgets include extra vehicle/module
and telemetry/formatting margin; they are not measured final requirements.
Accept only after calibration, full navigation, queue saturation and dump
show at least 256 untouched bytes per active application stack. A pattern
scan is not an overflow detector or proof of worst-case use.

Repeat [M1/M2](../testing/integration/KERNEL_ROBOT.md) on this new image;
repeat S0-U01, S2-U01/U02/U04/U05, S3-U01/U02/U03 and S5-U01/U02 first.
Do not reuse old-image hardware passes. New task CPU cost, ISR duration,
stack high-water, overloaded timing, physical STOP and sensor optics are
**Not run**. Exact measured values must be recorded before floor approval.
