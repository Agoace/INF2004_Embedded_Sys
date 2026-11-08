# Kernel robot program

## Scope and safety

STEP 7 separates acquisition, navigation and telemetry into core-0 tasks.
STEP 8 adds optional, non-critical MQTT telemetry through the existing
Wi-Fi/lwIP task. Its new hardware checks are **Not run**; STEP 7 passes
below do not validate the new startup, boards or network load. See
[MQTT design](../s1-telemetry/KERNEL_MQTT.md), [runbook](../s1-telemetry/MQTT_RUNBOOK.md) and
[template ledger](../architecture/TEMPLATE_CHANGES.md).
Pure LINE/SEARCH/SCAN/DETOUR modules, routes, motor tuning and
[`vehicle_robot_config.h`](../../app_program/vehicle/vehicle_robot_config.h)
are unchanged. The configuration contract compares the reference block
in [`src/vc_line_test.c`](../../src/vc_line_test.c) exactly (NULL becomes 0).
Earlier watchdog-image S2-U05 and M1 passes do not validate this new image.
The task-split hardware checks passed on 2026-11-06: TEST image 3f889a1,
M1-S7 and M2-S7. See the [step 7 results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06). R5 (full track) is not run.

Lift both wheels before TEST power. S2-U02 briefly drives real motors;
S2-U05 runs LAST and intentionally resets, then reports the previous reset
on the next boot without repeating it. Read the
[watchdog procedure](KERNEL_WATCHDOG.md#s2-u05-two-boot-test-procedure).
Normal GP21 starts spin calibration, waits 1 s, then drives for at most
90 s. GP21 during calibration/run latches ESTOP. GP20 requests a saved RAM
dump only while idle. Do not reset before saving. Use VIN without USB for
track work; see [USB power safety](../s2-motion/USB_POWER_SAFETY.md).

## Selection and ownership

```sh
cd build_make
make SMP=0 WIFI=none -j4            # robot
make SMP=0 WIFI=none BENCH=1 -j4    # motion bench only
make SMP=0 WIFI=none TEST=1 -j4     # subsystem tests, wheels lifted
make -j4                           # robot, default SMP/radio profile
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 -j4 # robot with MQTT
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 TEST=1 -j4 # Wi-Fi timing TEST
```

The following application tasks are pinned to physical core 0 (`TP_PRC1`)
in both profiles; the SMP build does not move application work to core 1.
Lower priority numbers run first. Wi-Fi is created directly at 20, below
driving tasks 10–13 and above telemetry 30. The USB console is 21 (template
3): a measured GP20 dump delayed motion dispatch by up to 4.5 ms at 3. The
original template used Wi-Fi 4, below USB 3 but above its qualification task. Driving
must not wait for network processing. Approved shared changes are recorded
in [the template ledger](../architecture/TEMPLATE_CHANGES.md). Vehicle activation requires
an armed motion watchdog and successfully started line, sonar and telemetry
services (including the sonar deadline cyclic). Failed startup is reported
by the existing init/start CHECK path and does not permit a robot session.

| Task/context | Priority | Release | Owns / kernel objects | Stack bytes |
| --- | ---: | --- | --- | ---: |
| Wi-Fi service | 20 | Existing poll task; requested sleep 2 ms | Sole CYW43/lwIP/MQTT owner; named application hook, no wrap | Template 8192 with lwIP |
| motion_task | 10 | cyc_motion, 10 ms | Motor/servo PWM, encoders, stale stop; inherited mtx_motion; sole watchdog feed | 3072 |
| line_task, S3 | 11 | Every 1 ms ADC ISR wake | Raw queue consumer, lp/filter/calibration data; inherited snapshot mutex | 2048 |
| sonar_task, S5 | 12 | Request or echo wake; 10 ms deadline cyclic | Trigger/echo; request MBF 256 bytes, result mutex and event flag | 2048 |
| vehicle_task | 13 | Motion after-tick notification, 10 ms | Spin behaviour, buttons, one active module, service requests, motion commands | 4096 |
| USB service | 21 | USB IRQ wake or 10 ms fallback | Template console; local priority change only | Template |
| telemetry_task, S1 | 30 | Receive mbf_telemetry, event-driven | RAM rows/events and paced dump; MBF 4096 bytes; producer-loss counters use DI/EI | 4096 |
| health_task, MQTT only | 31 | 1 s cyclic wake | Bounded health board and preemptible app-stack scan | 2048 |
| motion_bench, exclusive BENCH image | 11 | Motion after-tick notification | Earlier bench buttons/log; never runs with vehicle | 4096 |
| ADC TIMER alarm0 / IRQ0 | IRQ level 2 | Absolute 1 ms targets | Eight conversions/channel, timestamped 128-slot queue, loss/ISR timing counters | Interrupt stack |
| Encoder/echo GPIO13 | IRQ level 1 | Edges | Shared dispatcher, counts and rise/fall timestamps; echo service wake | Interrupt stack |

Robot totals: **5 application tasks, 3 mutexes, 2 cyclic handlers, 2 message
buffers and 1 result event flag** without MQTT. MQTT adds one health task,
one 1 s cyclic and three MBFs: 16 live events, eight normal commands, one
ESTOP. Thus MQTT robot totals are **6 app tasks, 3 mutexes, 3 cyclics,
5 MBFs and 1 flag** (plus template USB/Wi-Fi/initial tasks). None of these
network objects or their readiness gates driving. TEST starts the four service/motion tasks,
not vehicle or bench. BENCH starts only motion/bench and leaves ADC/sonar
and telemetry idle in the radio-off BENCH profile. MQTT adds health even
in TEST; vehicle command consumption remains NORMAL-robot only. The one
motion notification slot remains exclusive to
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
[Control architecture](../architecture/CONTROL_ARCHITECTURE.md); no barcode decoder or
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
1 us; accumulated unsigned counters use window subtraction. Measured on
2026-11-06: ISR max 114 us, 69–75 per-mille (see the [step 7 results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06)).

## Ultrasonic request/result service

`sonar_request()` copies bounded requests into an MBF with TMO_POL.
Requests carry mode, ID, generation, angle, not-before time, interval and
absolute deadline. A second outstanding one-shot in the same generation is
rejected, not overwritten. A full MBF returns an error; vehicle stops its
session rather than waiting. The service alone calls trigger/service/busy.

- WATCH keeps the requested interval and hardware minimum 60 ms. A result
  must be acknowledged before another trigger, preserving the no-ping-over-
  unconsumed-result rule. Its result age limit is actual trigger time plus
  twice the interval and 20 ms, not the line age limit. A late service cannot
  relabel an old echo as fresh. Vehicle also enforces a WATCH progress timer
  and one-shot delivery deadline when the service produces no result.
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
| Timing/stack evidence | Instrumentation and host doubles implemented; ISR, processing and stack measured on 2026-11-06 (see the [step 7 results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06)); physical stop latency not run |

## KR1 rows and VE events

S1 telemetry alone owns the 4501 x16-byte rows (72016 bytes) and 512
x12-byte VE events (6144 bytes). Vehicle packs a local row, then sends a
by-value nonblocking message. Full MBF/storage increments lost_telemetry;
a producer failure/storage-full stops the active session, never blocks
control. A BEGIN clears saved indices, not lifetime loss counts. GP20
sends a DUMP command and diagnostic copy; priority30 owns formatting and
paced USB output. No new run starts while a dump is queued/active. See
[telemetry contract](../s1-telemetry/KERNEL_TELEMETRY.md).

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

[Static application stacks](APPLICATION_TASKS.md) reserve 19456 bytes without MQTT (21504 with health) for
all six registered task regions, including the mutually exclusive bench.
The robot uses 15360 of these bytes for its five active stacks. Pattern
high-water reports cover application tasks including motion, not template
initial/USB/radio/idle stacks. Initial budgets include extra vehicle/module
and telemetry/formatting margin; they are not measured final requirements.
Accept only after calibration, full navigation, queue saturation and dump
show at least 256 untouched bytes per active application stack. A pattern
scan is not an overflow detector or proof of worst-case use.

Repeat [M1/M2](../../testing/integration/KERNEL_ROBOT.md) on this new image;
repeat S0-U01, S2-U01/U02/U04/U05, S3-U01/U02/U03 and S5-U01/U02 first.
Do not reuse old-image hardware passes. New task CPU cost, ISR duration,
stack high-water, overloaded timing, physical STOP and sensor optics are
**Not run** unless listed in the [step 7 results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06).

## Local STEP 7 software validation

Validated source revision: `e4a8318`, after the `--no-ff` merge of current
staging `17f3717`. Later completion changes are documentation only.
These are local build/test results, not hardware acceptance.

| Profile | text | data | bss | Compiler warnings |
| --- | ---: | ---: | ---: | ---: |
| SMP=0 WIFI=none robot | 62540 | 152 | 118840 | 0 |
| SMP=0 WIFI=none BENCH=1 | 41774 | 148 | 95388 | 0 |
| SMP=0 WIFI=none TEST=1 | 51732 | 152 | 118012 | 0 |
| Default SMP/radio | 308632 | 216 | 125280 | 0 |

Each profile used a clean `make -j4 --output-sync=target`. All 16 touched
application C units passed a second real-header ARM compile in each profile
with `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`: **64/64**.
Runtime-free retained SDK adapter link probes passed **4/4**, including
watchdog/time prerequisites and no Pico runtime linkage. Vehicle object's
undefined symbols contain service APIs, not drv_line/drv_ultrasonic calls.
SDK headers remain confined to adapter `*_hw.c` files.

| Host branch suite | Passed |
| --- | ---: |
| Platform, mtk3 | 15/15 |
| LINE, mtk3-line | 19/19 |
| Sonar, mtk3-scan | 20/20 |
| Telemetry, mtk3-telemetry | 16/16 |
| Staging integration | 43/43 |

Doubles cover timestamped sample processing/LP barcode separation,
calibration abort retention, LINE freshness, generation/ID/angle rejection,
not-before timing, deadline delivery, actual-trigger identity, immediate
STUCK_HIGH, unknown NO_RETURN, old WATCH timestamp retention, result flag
reset, cyclic-start failure, service-start gating, nonblocking telemetry
loss/storage/dump ownership and vehicle deadline/WATCH-progress stops.
The configuration/routing and KR1/VE contract tests also pass.

Compiler stack-frame reports (UP robot) show vehicle_task 728 bytes,
line_task 48, sonar_task 88, telemetry_task 256, app_task_report 144.
These are **individual static frames**, including compiler inlining, not
complete call-chain/interrupt costs or runtime high-water measurements.
They do not replace physical margin checks. The largest static RAM use is
125496 bytes (default data+bss); 136648 bytes remain in the 256 KiB linker
RAM region before dynamic kernel/message-buffer/template-stack use. The
19456-byte application-stack reserve is already included in bss.

At STEP 7, no pure module/configuration, `src/`, `sim/`, Ziven source or
template/kernel/lib/device/include/config file changed. STEP 8 keeps pure
references unchanged but permits listed shared template deviations; see
[the current ledger](../architecture/TEMPLATE_CHANGES.md).
Bare-metal/simulator checks were therefore not repeated. Documentation,
map links/anchors, diff whitespace, local secret checks and tracked-tree
clean checks pass. Logs and retained probe/compiler evidence are under
`/tmp/step7-results/`; generated firmware and other scratch worktrees are
removed at completion. The candidate checkout is retained at `/tmp/mtk3t`.
No push or flash was performed. Hardware timing, stack and M1/M2 results:
see the [step 7 results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06).

## FIX 1 and PROBE A (TEST only)

The operator's `445084c` TEST log reported S2-U01 FAIL on both boots:
501 ticks, lifetime gaps 8991..11024 and 8535..11475 us, late1/missed0,
work maxima 113/198 us. S0, S1-U02, S2-U02, S3-U01/U02 and S5-U02 passed;
S5-U01 had 20/20 valid results near209 mm and S2-U05 passed at99 ms.
Reported stack high-water maxima were motion416/3072, line344/2048,
sonar240/2048 and telemetry416/4096 bytes. Instrumented S3 work was about
1.3% of the elapsed window, max207 us/sample; ISR about7.5%, max114 us.
These are earlier-image observations, not acceptance of this correction.
The ignored source log is `build/mtk_logs/s7_test_20261106-215149.txt` in
the main checkout. The [S2 record](../../testing/subsystem2/TEST_LOG.md#step-7-s2-u01-failure-and-approved-windowprobe-correction)
has the analysis and current status.

The earlier S2 check mixed a tick delta with lifetime gap/counter fields.
New S1 saturation work at priority1 (100 sends without a yield), plus
possible USB3 service afterwards, is the leading explanation. It has not
been confirmed by a phase trace. The actual late wake is not erased or
reclassified as a hardware pass.

Startup remains at initial priority1. Only TEST changes its runner to20
before S0/S1, below motion10/line11/sonar12 but above telemetry30. Failure
to change priority prints ABORT and skips tests. Production priorities,
eight conversions per ADC channel, pure modules and watchdog feeding are
unchanged. No extra task, cyclic handler or kernel object is created.

`motion_timing_window_begin/end` are TEST-only. They retain all lifetime
statistics. Window starts are counted separately; gap extrema/late count
cover only pairs of starts both inside the window. The first cross-start
gap is excluded, so N starts give N-1 complete gaps. Missed wakes use the
lifetime difference across begin/end. S2 keeps the 499..501 tick, <=11000 us
gap and zero-missed limits. Window work maximum retains the existing
start-to-PWM measurement interval. Lifetime values print separately as INFO.

PROBE A captures eight fixed-size snapshots without formatting or output:
before/after S0 print; S1 burst begin/end; after its first yield; after its
print; S2 entry/exit. The TEST runner alone writes/reads the buffers;
capture copies motion fields under its short inherited mutex. Reporting
occurs after the window, from immutable captured values. No per-tick
console trace, USB/template probe, ADC optimization or Option H is added.

Expected format (numbers below are placeholders, not a predicted pass):

```text
[TEST] === test run start (runner priority 20) ===
[TEST] S2-U01 window ticks <N> gaps <N-1> gap <min>..<max> us tick_max_us <work> late <window_count> missed <delta>
[TEST] S2-U01 INFO lifetime ticks <total> gap <min>..<max> us tick_max_us <work> late <total> missed <total>
[TEST] PROBE begin: buffered lifetime snapshots (not window values)
[TEST] PROBE before_S0_print us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE after_S0_print us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE S1_burst_begin us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE S1_burst_end us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE after_S1_yield us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE after_S1_print us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE S2_entry us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE S2_exit us <t> ticks <n> gap <min>..<max> us tick_max_us <work> late <n> missed <n>
[TEST] PROBE end
```

Compare S2 entry/exit lifetime counter deltas and the independent window
result. A prior late1 with zero window late/missed confirms it preceded the
window. All-zero lifetime late counts are also valid if the lower TEST
priority removes the delay. A PROBE capture failure prints `unavailable
error <code>` rather than invented zeros. TIMER us is the raw wrapping
32-bit timestamp. **Hardware: PASS on 2026-11-06** (see the [step 7 results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06)).

### FIX 1 local software checks

| Profile | text | data | bss | Warnings |
| --- | ---: | ---: | ---: | ---: |
| SMP=0 WIFI=none robot | 62540 | 152 | 118840 | 0 |
| SMP=0 WIFI=none TEST=1 | 53520 | 152 | 118276 | 0 |

Both profiles passed clean `make -j4 --output-sync=target` builds. All four
touched application C units passed real-header ARM compilation in both
profiles with `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`
(**8/8**). Retained SDK adapter link proofs passed **2/2**. Host tests passed
**48/48**, including old lifetime failures with a clean window, a 11001 us
window that must fail, historical missed-count isolation, multiple windows,
immutable probe snapshots, capture/report order, TEST demotion/failure and
unchanged production runner policy. No bare-metal/simulator source changed.

The UP/no-radio production loadable ROM binary is byte-identical to the
operator-built `445084c` image, SHA256:
`c3aabaacd8cc706197aa7e22ff8eb520de57aa8fdfd28d15140a5a63f8b53544`.
No TEST timing/probe symbols are present in that production ELF. This
checks production code isolation, not new-image hardware performance.
The TEST image adds 1788 text bytes and 264 bss bytes over `445084c`.
No stack budget, production priority, ADC or watchdog-feed change is made.
Documentation/map, whitespace, secret-pattern/credential-config and clean
checks pass. Logs, compiler frame reports, link maps and validation commands
are retained under `/tmp/step7-fix1-results/`. **No push or flash.**
S2-U01 and PROBE A passed on hardware on 2026-11-06.

## STEP 8 pull boards and non-critical MQTT

Sensor boards are transport-independent. Motion and line add bounded
non-consuming last-tick/sample reads; sonar adds a latest board plus the
last 16 ECHO/DEADLINE/CANCELLED service publications with sequence cursors
and eviction counts. Stale/orphan physical echoes remain discarded.
Vehicle publishes state/module/phase, active/calibrated, commands and
counters. Health31 publishes uptime/reset, losses, overruns, app-stack
minimum and network status. One-attempt atomic-word reads never spin.
Busy/unavailable reads count per board; retained values have fresh flags
cleared and explicit saturated ages. History/events do not depend on a
latest-board read. See [board details](../s1-telemetry/KERNEL_MQTT.md#pull-boards).

Only vehicle pushes bounded rare events. Token-bearing commands queue to
vehicle, never directly to motion. ESTOP has a dedicated queue slot and
flushes older start intentions. CALIBRATE/START are idle-only; START needs
learned calibration; DUMP streams committed RAM data without freezing it.
Absent/invalid/example token permits only shape-valid ESTOP, token unchecked;
a valid configured token must match every command, including ESTOP. GP21
remains the real ESTOP; MQTT ESTOP has no latency guarantee. Network failures
never become motor/vehicle faults, never feed watchdog and never gate boot.
M1/M2 acceptance is independent of MQTT; S1-U04..06 are separate.

Kernel reset startup now establishes the 1 us TIMER source immediately
after clk_ref selection and before vectors/data/BSS/kernel/devices/tasks.
It uses only register/stack state, no SDK headers or uninitialized globals.
The app call remains idempotent verification; the 2% rate gate is unchanged.
The previous initial1-before-Wi-Fi boot-order dependency is gone.

Wi-Fi full-poll wall metrics include preemption, not CPU utilization.
Independent begin/end windows preserve lifetime data and count gaps >10000
us. TEST prints its Wi-Fi window after S2 closes; NORMAL prints a complete
vehicle-session window only after motion stops. The MQTT heartbeat reports
lifetime gap max/samples and actual observed priority. Zero polls or stale
health is not a performance pass. No picotool reset interface is added.

New-image S0-U01 cold-power (including MQTT), S1-U04/U05/U06, S2-U01 with
Wi-Fi20, full-load stack/timing, and independent M1/M2 with MQTT are
**Not run**. Exact [wire layouts](../s1-telemetry/MQTT_WIRE.md) and [broker procedure](../s1-telemetry/MQTT_RUNBOOK.md)
are separate records. Final software results are recorded below.

[STEP 8 software validation](../archive/MTK3_STEP8_VALIDATION.md) supersedes the earlier build sizes for the current image. Hardware rows above remain historical or explicitly Not run.
