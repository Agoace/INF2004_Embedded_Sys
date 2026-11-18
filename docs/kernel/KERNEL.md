# Kernel robot program

## What it is

The Pico W robot uses μT-Kernel tasks on physical core 0. A task is a
scheduled function with its own stack. Lower priority numbers run first.
The normal program acquires sensors, calibrates, runs one navigation module
at a time, and saves a RAM log. Optional MQTT sends observations; loss of
Wi-Fi never stops or starts the robot.

Use [BUILD.md](BUILD.md) for images and flashing, [motion](../s2-motion/MOTION.md)
for control, and [telemetry](../s1-telemetry/TELEMETRY.md) for network ownership.
The [template ledger](../architecture/TEMPLATE_CHANGES.md) lists eight approved
shared-file deviations. This record describes current behavior, not the old
multi-core proposal or acceptance of every possible load.

## How it works

```text
1 ms ADC interrupt -> line11 ----+
echo interrupt -> sonar12 -------+-> vehicle13 -> motion commands
10 ms cyclic -> motion10 -------+      |              |
                    | after-tick wake |              v
                    +-----------------+          next motor tick
                                      |
                                      +-> RAM queue -> telemetry30
10 ms IMU cyclic -> IMU14 -> read-only boards -> TEST20 USB observer
                               hump board -> vehicle13 (run result)
read-only network boards -> Wi-Fi20 -> broker; health31 -> health board
USB21 drains the console; idle_meter32 estimates unused core-0 time
```

`TP_PRC1` means physical core 0. Application work stays there even in an
SMP build (a build with both processor cores enabled). Interrupts release
work; they do not run navigation or format output. Only the initial startup
task runs at priority 1. TEST lowers that task to 20 before tests.

### Tasks and priorities

| Task | Priority | Release / owner | Stack bytes |
| --- | ---: | --- | ---: |
| motion | 10 | 10 ms cyclic; motor/servo PWM, encoder sample, stale stop, watchdog feed | 3072 |
| line | 11 | Each 1 ms ADC acquisition; filter, learned line values and snapshot | 2048 |
| sonar | 12 | Request/edge wake; 10 ms deadline cyclic; trigger and result service | 2048 |
| vehicle | 13 | After each completed motion tick; buttons, calibration, modules and commands | 4096 |
| IMU, robot/TEST | 14 | Own 10 ms cyclic; bounded 400 Hz FIFO batch/count reads, pure core, report-only hump detector, snapshot and hump boards | 2048 |
| Wi-Fi | 20 | Template poll loop; sole CYW43/lwIP/MQTT owner | 8320 with lwIP; 6272 radio-only, registered |
| TEST runner | 20 | Initial task after startup; no normal vehicle/bench task | Initial task stack |
| USB console | 21 | IRQ wake or 10 ms fallback; console drain | 2048 UP / 4096 SMP (template-owned) |
| telemetry | 30 | Message arrival; RAM rows/events and paced USB dump | 4096 |
| health, MQTT only | 31 | 1 s cyclic; counters and application-stack scan | 2048 |
| idle meter, MQTT only | 32 | Always ready, preemptible; CPU estimate | 512 |
| motion bench, BENCH only | 11 | Motion after-tick wake; exclusive alternative to vehicle | 4096 |

The motion notification slot has one consumer: vehicle or bench, never both.
Task functions do not return. They loop or call `tk_ext_tsk()`.
IMU_STREAM's TEST20 observer alone can request the operator GP21 drive
trial (100 mm/s,600 mm/10 s), with stop-pending retries. No IMU14 height or
quality result chooses a drive command. VIN/lifted-first hardware checks
remain pending; see the [procedure](../s4-imu/IMU.md#step-2d-current-procedure).

### Interfaces and kernel objects

| Boundary | Interface and rule |
| --- | --- |
| Motion | `motion_*` calls use a priority-inheritance mutex (a waiting high-priority caller raises the holder's priority); no printing or sleeping while held |
| Line | 128 queue slots, 127 usable; one timestamped ADC sample per interrupt; inherited snapshot mutex; pure line code receives L/C/zero, not barcode ADC |
| Sonar | 256-byte request message buffer; copied requests with generation, ID, angle, not-before time and deadline; result mutex and event flag |
| Telemetry | 4096-byte message buffer, by-value messages, no producer wait; array owner is telemetry30 |
| Network | Bounded read-only boards, event/command queues; never a control mutex or PWM writer |
| IMU | Bounded motion-count copy, own snapshot/hump/diagnostic boards; atomic vehicle calibration, run and LINE epochs; vehicle reads without a wait; diagnostic context prints after STOP/DUMP only; no motion mutex or commands |

Only core 0 enables ADC/GPIO interrupts. The source requests GPIO IRQ13 at
interrupt priority 1 and ADC TIMER alarm0 at 2; USB IRQ5 keeps 0;
SysTick/PendSV use 3. On the hardware IRQ0, IRQ5 and IRQ13 are all at 0,
because `EnableInt` does not write the priority
([F-NVIC](#interrupt-priorities-f-nvic)). These are
**interrupt** priorities, not task priorities. A high-level interrupt wrapper
permits `tk_wup_tsk(explicit_id)` from the interrupt context.

### Settings and timing

| Setting | Current rule |
| --- | --- |
| Motor period | 10 ms; discard/count queued wakes rather than replaying old releases |
| ADC | Eight conversions each for L/ADC1, C/ADC0, barcode/ADC2, in that order; 48 MHz ADC source; GP29 never selected |
| ADC targets | Absolute 1000 µs increments; missed periods count as lost, not catch-up samples |
| Line safety | Missing/invalid snapshot, age >10000 µs, or new missed/dropped acquisition stops an active session |
| Sonar WATCH | At least 60 ms between triggers; acknowledge each result; age limit = trigger + twice the requested interval +20 ms |
| Sonar ONCE | SCAN request deadline 100 ms; check generation, ID, angle and actual trigger identity |
| Vehicle deadline | A new motion tick before completion or wake-to-finish >=10 ms counts an overrun and stops the active session |
| Buttons | GP21 sampled every 20 ms; normal press calibrates then starts; press during calibration/run latches ESTOP. GP20 dumps saved RAM data while idle |
| Driving limit | Normal start delay 1 s; navigation at most 90 s |

`response_max_us` is vehicle **wake-to-finish**, not cyclic-release-to-finish.
Motion commands normally take effect on the following motor tick. The line
10 ms age rule is not a sonar age rule. Rejected or full sonar requests stop
the local session instead of making navigation wait. Old physical echoes
cannot become a new generation's observations.

## Application stacks

`app_start_task()` uses registered, static, 8-byte-aligned regions with
`TA_USERBUF` and a 0xa5 fill pattern. Failed start deletes the dormant task;
failed deletion quarantines the still-owned stack until reboot. Wi-Fi uses
`app_task_start_config()` to preserve its `TA_RNG1` policy and old actual
allocation, including the former kernel allowance. Other app tasks use
`TA_RNG3`. See [Wi-Fi stack (M9)](#wi-fi-stack-m9).
The supplied size already includes system/context space; the kernel does
not add its normal 128-byte allowance.

| Reserve/use | Bytes | Meaning |
| --- | ---: | --- |
| Base BENCH regions | 19456 | Before Wi-Fi/MQTT; bench present, IMU absent |
| Base normal robot regions | 17408 | Before Wi-Fi/MQTT; six stacks, including IMU |
| IMU region, not BENCH | 2048 | Included in normal base; Robot/TEST only |
| MQTT extra health + core-0 idle meter | 2816 | 2048 + 768 |
| SMP + MQTT core-1 idle meter | 768 | Not reserved in UP |
| Radio-only Wi-Fi region | 6272 | Includes the former 128 B kernel allowance |
| lwIP Wi-Fi region | 8320 | With or without MQTT; includes former allowance |
| Normal SMP MQTT robot | 29312 | Base + 2816 + 768 + 8320; ten slots |
| Normal UP MQTT robot | 28544 | No core-1 idle meter; nine slots |
| Low-margin threshold | 256 | Untouched bytes required per active application stack |

The A1/B1 hardware gate on 2026-11-10 measured idle-meter use 296/512 B,
leaving only 216 B. The approved safety correction increases that reserve
to 768 B. The enlarged stack's target high-water is not yet measured.
See [the gate record](../../testing/subsystem2/TEST_LOG.md).

`app_task_stacks()` scans the fill pattern from the low end and keeps the
largest observed use. TEST and GP20 print size, high-water use and remaining
bytes. Health scans once per second. This is an estimate, not an overflow
detector or proof of worst-case depth. Initial, USB and interrupt
stacks are outside this measurement; radio is now included. Exercise
calibration, every navigation
module, queue saturation and dumps before accepting a stack budget.

### Wi-Fi stack (M9)

| Item | Rule |
| --- | --- |
| Owner | `app_task.c` holds one static, painted `cyw43` stack and slot when `TM_WIFI_CYW43`; Wi-Fi-off images hold neither |
| Size | Radio only 6272 B; lwIP (with or without MQTT) 8320 B. Same as the old pool allocation (request + 128 B). A size mismatch is rejected |
| Start | `app_task_start_config()` keeps entry, priority 20, `TA_RNG1`, name and core; duplicate start fails; failed delete quarantines the slot until reboot |
| Report | `[stack] cyw43 core N size 8320 high_water U free F`; `free < 256` prints `LOW MARGIN`. Health includes it in `stack_min_free` |
| RAM | Static +8336 B, pool request -8320 B. This is a move, **not** a saving: the kernel pool starts after BSS and shrinks by the same amount |
| Hardware gate | Not run: association, reconnect, MQTT, DUMP, then `cyw43` free >= 256 B |

IMU stack readings (2048 B region, 256 B policy margin):

| Image | High-water / free (B) |
| --- | --- |
| `dfade14` | 752 used |
| H1 `1480b37` (STREAM) | 1448 / 600 |
| `d51d27b` robot | 1456 / 592 |
| Diagnostic C `29d57de`, after hump runs | 1568 / 480 (224 B over policy) |

In the `d51d27b` session `stack_min_free` 272 B was `idle_meter` (512 B,
used 240), not IMU. The meter is now 768 B.

## Watchdog

Only a **completed motion tick** feeds the 100 ms hardware watchdog, after
PWM writes and mutex release. Motion starts its task, cyclic, then watchdog.
Speed commands are rejected until it is armed. No interrupt, Wi-Fi callback,
vehicle work, USB dump or TEST elapsed sampler feeds it.

`drv_watchdog_start(ms)` requires successful time initialization, accepts
1..8388 ms, and starts once. On RP2040-E1 the SDK loads `ms * 2000`:
100 ms loads 200000 because the counter decrements twice per 1 µs tick.
Debug halt pauses it. A timeout resets the chip; pins return to reset inputs
and startup writes motor duty zero. This does not prove physical stop time.
It does not cover a hang before watchdog startup.

### Last reset reason

| Report | Meaning |
| --- | --- |
| `power/RUN` | No watchdog reason; cannot separate power from RUN-pin reset |
| `watchdog` | TIMER reason and SDK enable magic `0x6ab73121` in scratch4 |
| `other` | Forced/unknown watchdog reset without that magic; not a unique picotool label |

The driver caches the reason before enable changes scratch4. S2-U05 runs
last in TEST: stop motors, allow 1 s for the warning to drain, suppress feeds
after a final completed tick, and retain the last elapsed sample in scratch0/1.
The next boot consumes the marker and reports once; it does not reset again.
A last sample of 90..110 ms and an own watchdog timeout pass. No reset by
500 ms fails. Lift both wheels and do not halt a debugger. Full procedure:
[S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md#s2-u05-two-boot-watchdog-procedure).

## CPU load

The MQTT-only idle meter runs the same `noinline,noclone` loop used during a
4000 µs interrupt-disabled startup calibration. Its calibrated rate is the
zero-load reference. Health samples count/time differences once per second:

`load_permille = 1000 - round(1000 * count_rate / calibration_rate)`

Clamp to 0..1000. Unknown calibration, first sample or zero elapsed time gives
65535. Unsigned differences handle counter/TIMER wrap. One permille is 0.1%.
The meter counts interrupt and task activity on core 0, not both cores or
individual tasks. It replaces WFI sleep, so power use increases. Failure is
diagnostic only and does not gate motion. Target accuracy is not established.

## Run timing report (P1)

P1 adds report-only latches for one vehicle run. The GP20 dump prints them
after the unchanged `[load]` line (`TM_MQTT` builds). Code:
[`run_timing.c`](../../app_program/platform/run_timing.c) and
[`kernel_lock_stats.c`](../../app_program/platform/kernel_lock_stats.c).
They do not change driving, wire formats, task priorities or placement.

Run scope: Vehicle sets an odd epoch at run start and an even epoch after
STOP. Each latch has one writer. A writer clears its own latch when it first
sees a new run. After STOP, writes stop, so the dump shows the run, not the
idle time after it. A latch that its writer did not touch in the run prints 0.

| Line | Content | Writer |
| --- | --- | --- |
| `[load_run]` | peak load per core over the 1 Hz samples of the run; 65535 = unknown | Health |
| `[bkl]` | big-kernel-lock figures from run start to STOP; `unavailable` with `SMP=0` | Vehicle |
| `[isr_max]` | line alarm lateness (entry minus target), ADC read, ISR wake call | line ISR |
| `[line_max]` | one sample, `lp_update`, barcode sample | line_task |
| `[step_max]` | wake latency and the parts of a vehicle step | Vehicle |
| `[kcall]` | `n`, `sum`, `max` per call site; mean = `sum / n` | ISR, line, motion, Vehicle |
| `[first_overrun]` | step, elapsed, motion ticks since release, slowest part, wake latency, uptime | Vehicle |

`[kcall]` sites, all on core 0 in the current placement: `isr_wup`
(alarm `tk_wup_tsk`), `line_wake` (ISR wake return to line_task running),
`line_can`, `line_loc`, `line_unl` (line_task `tk_can_wup`, `tk_loc_mtx`,
`tk_unl_mtx`), `mot_wup` (motion notify `tk_wup_tsk`), `veh_wake` (motion
notify return to Vehicle running), `veh_can`, `veh_status`
(`motion_status` in `wait_tick`), `veh_send` (`telemetry_send`).
Compare the same source built as `SMP=0`, force-core-0 and split.

Limits: all values are wall time from the 1 MHz TIMER. Interrupts and
higher-priority tasks inside an interval are included, so use the mean for
`[kcall]`; a maximum can include preemption. Wake latencies above 100 ms are
not counted. A late step skips `motion_status`; its tick count then comes
from the motion board copy, and `ticks +4294967295` means it was not
available. The `[bkl]` reset at run start can race a core-1 update. Runs
shorter than 1 s can have zero `[load_run]` samples.

Cost, measured at `d8adbcc` against `451e562` (school MQTT profiles):

| Item | Cost |
| --- | --- |
| Static RAM (bss) | +408 B (split, force0, `SMP=0`); +384 B USB-only; +280 B capture |
| Flash (text) | +4456 B split and force0; +4200 B `SMP=0` MQTT; +1672 B USB-only |
| Hot-path code size | `alarm_irq` 304 -> 388 B, `line_task` 444 -> 572 B, `wait_tick` 780 -> 1010 B, `finish_step` 176 -> 332 B, `motion_task` 480 -> 516 B; all in flash (XIP) |
| Stack frames | `rt_report` 120 B on the dump caller; `alarm_irq` +8 B (+20 B `rt_call`); `wait_tick` +8 B; `finish_step` +48 B |
| Time | 2 or 3 extra TIMER reads per measured site |

### Overrun timeline (P2)

P2 adds a ring of the last 128 core-0 events of a run. It shows the order
of events in a bad step; the P1 maxima cannot show this. It is report only.
The GP20 dump prints it after the P1 lines with 20 ms USB yields:

```
[timeline] epoch E frozen F records N end_us T
[tl] -US SITE ARG gGPIO | ... (8 records per line, oldest first)
```

`-US` is microseconds before the newest record. `gGPIO` is the low 16 bits
of a GPIO-IRQ dispatch count (encoder and echo). The difference between two
records is the number of GPIO interrupts between them.

| Site | Event | ARG |
| --- | --- | --- |
| 1 | end of the line alarm ISR | queue depth |
| 2 | line_task running | 0 |
| 3 | line_task about to sleep | samples drained |
| 4 | motion tick start | 0 |
| 5 | motion notify returned | 0 |
| 6 | sonar_task wake / ping sent | 0 / 1 |
| 7 | Vehicle running | 0 |
| 8 | end of a step part | `rt_max_t` part id |
| 9 | `finish_step` | 1 = overrun |

Run scope: a new run clears the ring. The first overrun freezes it
(`frozen 1`), so it keeps the bad step. Without an overrun it holds the
last events before STOP. Writers are core-0 contexts only. Each record is
written in one core-local `DI`/`EI` section (index plus two word stores; no
kernel call). A long gap between records is time in a context with no
site, for example the USB or SIO interrupt or a kernel masked section.

Cost, measured at the P2 commit against `ddc5436`: static RAM +1040 B
(all profiles); flash +832 B (MQTT), +800 B (USB-only), +880 B (TEST).
`rt_tl` (196 B) and `rt_tl_gpio` (32 B) run from RAM, because the line and
GPIO ISRs call them; a flash (XIP) cache miss would change the times they
record. The build renames their `.text.<name>` sections to
`.data.ramfunc.<name>` with `objcopy` (`build_make/pico_rp2040.mk`; a
section attribute on code there makes the assembler warn), and the template
linker script copies `.data.*` to RAM; flash callers reach them through a 16 B long-branch
veneer each. Their interrupt mask uses the inline `get_primask` and
`set_primask`, because `DI` calls `disint()` in flash. The ISRs that call
them still run from flash. Cost of the RAM placement: `.data` +224 to
+232 B, `.text` -176 B. The RAM load segment is now RWX on purpose
(`--no-warn-rwx-segments` in `build_make/app_sdk_hw.mk`).

## RAM placement (KERNEL_RAM)

Demo image (operator decision): `make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1
KERNEL_RAM=1`; `<image>.ramproof` must say `RAM_PROOF PASS listed 75 in_ram 75`.

| Item | Record |
| --- | --- |
| Switch | `KERNEL_RAM=0` (default) or `1`. `1` needs `SMP=0`. The default adds no rule: ordinary images stay byte-identical (hash76). Image suffix `_kernel_ram`. |
| How | Before the link, `objcopy --rename-section` moves each listed section to `.data.ramfunc.*`. The unchanged template linker script copies `.data.*` to RAM. No source or template edit. |
| Lists | [`common.list`](../../tools/kernel_ram/common.list): dispatcher, interrupt entry, timer, `disint`/`ClearInt`, `tk_wup_tsk`, `tk_slp_tsk`, `tk_can_wup`, `tk_loc_mtx`, `tk_unl_mtx` and helpers, line and encoder ISRs, `drv_time_us`. [`robot.list`](../../tools/kernel_ram/robot.list): real line/GPIO/motor providers, sonar echo ISR, USB ISR and its transfer helpers; v2 adds the line-task per-sample path (line sample, live barcode, GP28 window geometry, line position). [`bench.list`](../../tools/kernel_ram/bench.list): bench providers (`BENCH_KERNEL=1`). |
| Left in flash | `rt_call`/`rt_max` (P1), `memcpy`/`memset`, `panic`, `rp2040_chip_version`, Wi-Fi/lwIP/MQTT (priority-20 poll task), soft-float, integer divide and `cosf`/`sinf` (toolchain archives; the rule renames only project objects). |
| Proof | [`ram_proof.py`](../../tools/kernel_ram/ram_proof.py) runs after the link and fails the build if a listed symbol is not in RAM. `<image>.ramproof` lists the remaining flash callees and indirect calls. |
| Size (v2, b0c3a9a) | 18152 B moved; `.data` 19048 B; static RAM 219 of 264 KB. |
| Bench result (af2e787) | Line-path kernel calls 28.0 -> 14.3 us per ms, `[load]` 292 -> 243 permille, BC_CPU -34 %. Repeats (e06667d, `[bench_top]`): every cost top-20 is lower; waiting tails rose (one `line_wake` event per run, `read_line` overlap with one line sample), all < 0.3 ms. |
| Acceptance (operator, rule b) | Robot P1 school run, BASE then RAM, same course; swap order on a repeat. PASS = overruns 0, lost_wakes 0, `load_run` peak lower, step and wake max <= BASE, KR1 reset not watchdog. Waiting tails (`line_wake`, `read_line`, response) pass if < 0.5 ms. Basis: 6 bench runs (3 per image, 2 Picos), all COUNT_MATCH, overruns 0, `[load]` 302-304 vs 238-240 permille; logs `~/Projects/mtk3-core-review/build/kernel_ram_e06667d/step0_{B1.final,B2,B3,R1,R2,R3}.log`. |

### Robot P1 pairs

Logs: `~/Projects/mtk3-core-review/build/kram_robot_<commit>/` (Main's
checkout). All runs: overruns 0, lost_wakes 0, faults 0, KR1 reset power/RUN.

| Pair (commit, order) | RAM list | load_run peak BASE / RAM | wake max BASE / RAM (us) | line_wake max BASE / RAM (us) | Rule (b) |
| --- | --- | --- | --- | --- | --- |
| 1 (e06667d, BASE first) | v1 | 923 / 811 | 1419 / 1458 | 394 / 539 | FAIL: wake, line_wake |
| 2 (2413c4a, RAM first) | v1 | 947 / 798 | 1328 / 1459 | 356 / 455 | FAIL: wake (+9.9 %) |
| 3 (b0c3a9a, BASE first) | v2 | 931 / 676 | 2043 / 1189 | 600 / 136 | PASS |

v1 cause: wake max = one worst line sample + ~0.4 ms; the barcode path, still
in flash, was slower on the v1 image (933-1016 vs 770-775 us), probably flash
layout. v2 moves the line-task per-sample path to RAM: line sample max 1329 ->
854 us, barcode max 1229 -> 801 us, every step max lower.

## Interrupt priorities (F-NVIC)

Finding 2026-11-13 on `451e562`. **Not fixed**; the fix is a template change
and needs its own approval.

`EnableInt_nvic()` in
[`int_armv6m.c`](../../lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c)
writes `NVIC_IPR` with a byte store. On Cortex-M0+ these registers are
word-access only, so the store has no effect. `knl_set_irq_priority()`
(`kernel/sysdepend/cpu/rp2040/smp_rp2040.c`) uses a word read-modify-write
and works. This applies to every UP and SMP image so far.

| Register (SWD read, no halt) | Value | Meaning |
| --- | --- | --- |
| IPR0 0xE000E400 | 0x00000000 | IRQ0 line alarm = 0 (requested 2) |
| IPR1 0xE000E404 | 0x00000000 | IRQ5 USB = 0 |
| IPR3 0xE000E40C | 0xC0000000 | IRQ13 GPIO = 0 (requested 1); IRQ15 SIO IPI = 3 |
| SHPR3 0xE000ED20 | 0xC0C00000 | SysTick = 3, PendSV = 3 |

Effect: line alarm, USB and GPIO do not preempt each other; when more than
one is pending, IRQ0 runs first, then IRQ5, then IRQ13. All three preempt
SysTick, PendSV and the SIO IPI. Every hardware result so far was measured
this way. Fix (on hold): word read-modify-write under the same mask, a row
in [TEMPLATE_CHANGES.md](../architecture/TEMPLATE_CHANGES.md), a host test,
an IPR readback and a timing re-run. With the fix, USB and GPIO preempt the
line alarm, so the line ISR worst case can grow.

## Core placement and locks

The demo runs UP (`SMP=0`), all tasks on core 0. The split (dual-core)
placement builds but is **parked**. The rules below still apply to every
SMP build and to any resumed split work.

### Split placement parked (2026-11-15)

| Item | Record |
| --- | --- |
| Decision | Operator: split PARKED; demo uses the UP image. |
| UP load | Core 0 mean 570-662 permille (30-40 % headroom), run peak 957 (P1). Split core 0 / core 1: 621-699 / 569-626. |
| Deadline | Split misses the vehicle deadline on the robot: first overrun at step 20, 9084 us. |
| Bench step 0 (52748fe) | Both images PASS: 2280000 edges, COUNT_MATCH 1140000 per wheel. UP vs split: ISR max 49 vs 193 us; response max 103 vs 482 us; line sample max 132 vs 193 us; motion gap 9983..10017 vs 9909..10064 us; BC_CPU 30.2 vs 49.7 s. |
| Cause | Generator ISR 5.60 vs 7.43 us per call (+33 %): shared XIP cache and bus contention, plus SMP kernel costs (lock, cross-core reschedule, IPI). Split generator cost about 3.2-3.6 % with IRQ entry, over the 3 % limit. |
| Logs | `~/Projects/mtk3-core-review/build/bench_step0_52748fe/step0_{UP.final,SPLIT}.log`; robot P1 `build/school_20261113/` in the parent coursework checkout. |
| If resumed | 1) Put the core-0 ISR path in RAM ([KERNEL_RAM](#ram-placement-kernel_ram) is UP-only today) and re-run split step 0. 2) Move the generator to RAM. 3) Step 1: real ADC model, motor-driven encoder, GP21 calibrate + 90 s run x 10 per image. |

### Placement rules

- `app_task.c` owns the placement table. `app_task_start()` needs a core-0
  creator, accepts core 1 (`TP_PRC1 + 1`) only with `TK_SUPPORT_SMP` and
  `TK_MAX_CORE > 1`, rejects a mismatch with the table (`E_CTX`) and sets
  `assprc` from the request. In UP both `app_prc_core0()`/`app_prc_core1()`
  return `TP_PRC1`.
- A cyclic handler uses the same core as the task it feeds.
- Moving a task to core 1 is two edits in one commit: its slot entry and its
  call site.
- Split map (parked): core 0 motion 10, line 11, sonar 12, vehicle 13, USB 21,
  health 31, idle 32; core 1 IMU 14, Wi-Fi 20, telemetry 30, idle_meter_1 32.

### Lock rules

1. A lock with two holders on one core that can preempt each other (task and
   handler, or two task priorities) uses `ISpinLock`/`ISpinUnlock`. Plain
   `SpinLock` only when every holder is on a different core or is the same task.
2. A caller that must not wait (Vehicle, ESTOP route) uses `ISpinTryLock`
   (returns `E_TMOUT`) or an atomic.
3. `atomic_*` is a hardware spinlock claim plus an interrupt-off window on
   Cortex-M0+ (no LDREX/STREX). Use it for low-rate state, never per sample on
   the 1 ms line path.
4. Board publication: one writer, odd/even sequence, aligned 32-bit
   `__atomic_*` SEQ_CST words ([`app_board.h`](../../app_program/platform/app_board.h)).
   A failed read is `E_OBJ`/`E_PAR`/`E_TMOUT`, never a zero or stale value.
5. No lock section holds a kernel wait, print, allocation or network call.
   Locks do not nest.

| Lock | File | Holders |
| --- | --- | --- |
| `stats_lock` | `subsystem1/telemetry.c` | RAM-log payload and stats snapshot; readers one `ISpinTryLock`. `busy`/`storage_full`/`lost` are separate atomics; `telemetry_send()` takes no lock |
| `dump_lock` | `subsystem1/log_stream.c` | DUMP admission; Vehicle request is a try-lock |
| `timing_lock` | `libwifi/.../cyw43_utk_kernel.c` | Wi-Fi poll timing window; Vehicle calls are try-locks |
| `metrics_lock` | `subsystem1/robot_mqtt.c` | MQTT counters and Health snapshot; released before other calls |
| `status_lock` | `libwifi/.../cyw43_utk.c` | 52 B radio status snapshot |
| `stacks_lock` | `platform/app_task.c` | stack high-water scan; monotonic, try-lock |
| `claim_lock` | `libwifi/.../cyw43_utk_compat.c` | Wi-Fi task only |
| `stress_lock` | `platform/app_lock_stress.c` | `TEST=1` only |

### Startup handoff (M8)

The radio task starts before `usermain`, so MQTT could read init-only
guards while core 0 still writes them.
[`app_startup.h`](../../app_program/platform/app_startup.h) holds one
aligned word. `usermain` publishes it (release store) after all five
subsystem init and start attempts and `vehicle_task_start()` return.
`cyw43_utk_app_poll()` reads it first (acquire load) and returns early while
it is 0; no MQTT client exists before that. Radio setup, DHCP and lwIP do
not wait. The flag means "startup attempts returned", not "providers are
healthy". It is not a mutex; boards, locks and atomics still apply.
Cost: +48 B text, +8 B RAM.

Other review repairs: M2 network availability mask uses atomic bit
operations (two writers); M3 every non-DUMP message checks the DUMP
reservation and returns `E_OBJ` while it is held.

`network_queue.c` has no lock: all counters are atomic. Same-core `DI()`/`EI()`
stays for the encoder counts, line ring, sonar listener and GPIO table (all
core 0).

UP shim (M7, [`smp_single_core.c`](../../app_program/platform/smp_single_core.c)):
the 16 public lock/atomic functions use the real lock word, keep the
incoming interrupt mask on a failed try, capture atomic results before the
mask is restored and run a real DMB. 19 host cases (16 failed before the
fix). Cost: +96 B UP flash, 0 B RAM.

Open (no device run): the lock stress test, the on-target affinity audit,
cross-core load and lock wait times. Byte sizes and links are not timing
acceptance.

## RAM budget

RP2040: 264 KB RAM = 256 KB main (262144 B) + two 4 KB scratch banks.
Percentages use 262144 B. Static RAM = `data` + `bss` from
`arm-none-eabi-size`. Each number belongs to its image.

### Static RAM by group (`cr1-pre-imu`, 2026-11-08, `SMP=0` MQTT)

| Group | Main symbols | Bytes | Share |
| --- | --- | ---: | ---: |
| RAM log for DUMP | `rows` (4501 x 16 B = 90 s at 20 ms), `events` | 78160 | 29.8 % |
| lwIP memory | `memp_memory_PBUF_POOL_base` 36771, `ram_heap` 16403, pools | 56881 | 21.7 % |
| App task stacks | vehicle 4096, telemetry 4096, bench 4096, motion 3072, line/sonar/health 2048, idle 512, kernel 512 | 22528 | 8.6 % |
| Kernel control blocks | `knl_*` | 11124 | 4.2 % |
| USB console | endpoints, `tx_buffer` 4096, queue | 6989 | 2.7 % |
| CYW43 state | `cyw43_state` | 2460 | 0.9 % |
| Other | small symbols | 6501 | 2.5 % |
| **Total** (`data` 368 + `bss` 184416) | | **184784** | **70.5 %** |

Dynamic from the kernel pool at start (then): Wi-Fi stack 8320 B (now
static, M9), USB stack 2048 + 128 B, telemetry buffer 4096 B, sonar buffer
256 B, network queues < 1024 B; about 15.5 KB. Estimated free then: about
60 KB. The free pool is **not measured**.

### Static RAM growth (MQTT robot, same public example settings)

| Image | Static B | Delta |
| --- | ---: | --- |
| `b569adc` (rebuilt `cr1-pre-imu` base) | 184784 | reference |
| IMU step 2 `dfade14` | 188516 | +3732 (2048 B IMU stack included) |
| IMU step 2b / 2c / 2d / 3 | 188812 / 189212 / 189484 / 189900 | +296 / +400 / +272 / +416 |
| IMU diagnostic C (`d51d27b` base) | 190672 | +772 |
| B2a junction | 190956 | +284 |
| B2b STEP4a/4b SHADOW (`235af44` rebuilt 187092) | 190996 | +3904 vs its base |
| Core migration + staging merge (`SMP=1`) | 204468 | +3904 vs migration; includes M9 move of 8320 B |
| KERNEL_RAM v2 demo (`SMP=0`, b0c3a9a) | about 219 KB of 264 KB | `.data` 19048 B |

The IMU_STREAM image adds a 7888 B raw capture (bss +8176 B); other
images have none.

### If RAM becomes too small

Use the steps in this order; measure before and after each.

| Step | Change | Gain | Cost |
| ---: | --- | ---: | --- |
| 1 | Do not reserve `bench_stack` in the robot image | about 4 KB | Check no robot task uses it |
| 2 | Size stacks from measured high-water (keep 256 B) | about 5-8 KB | Needs all-load stack data |
| 3 | RAM log 90 s -> 60 s or 30 s | 24 or 48 KB | Start of long runs lost |
| 4 | Log row 16 B -> about 10 B (minimum-bit fields; CR1) | about 27 KB | New decoder |
| 5 | Log every 40 ms, not 20 ms | 36 KB | Less detail |
| 6 | Smaller lwIP pools | 10-20 KB | More Wi-Fi loss; template change |
| 7 | Remove RAM log and DUMP | 78 KB | No data without Wi-Fi |

CR1 order (team, 2026-11-08): build IMU and barcode first, tag
`cr1-baseline`, measure, then do the minimum-bit work and measure again.
`cr1-pre-imu` is not the CR1 baseline. See [CR1 evidence](../architecture/CR1.md).

## USB priority

USB runs at 21, below driving10..13, IMU14 and Wi-Fi/TEST20, but above
the telemetry30 dump producer and always-ready meter32. Only its task
priority changed; the 4096-byte output ring (4095 usable), 64-byte chunk,
IRQ priority, counted wakes and TinyUSB ownership remain unchanged.
GP20 yields 20 ms every eight rows; no host wait was added. A slow or absent
host can still lose output; check `tm_usb_dropped` and each dump delta.
The startup radio firmware download can delay enumeration; the accepted
cold-power observation below is not a bound for every host or image.

## Results and limits

Each result belongs to its recorded image. It does not validate every later
image, load, stack path or physical stop distance.

| Observation | Recorded result | Evidence |
| --- | --- | --- |
| STEP 7 ADC work, 2026-11-06 | ISR max 114 µs; 69–75 permille of the window | [Integration results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| S2-U05, 2026-11-06 | Last sample 99 ms; own watchdog reset; no repeat | [S2 log](../../testing/subsystem2/TEST_LOG.md#s2-u05-result-2026-11-06) |
| CPU, 2026-11-07 | Idle 117, traffic 139 permille; calibration 4026729 iterations/s | [S1 log](../../testing/subsystem1/TEST_LOG.md) |
| Idle-mode CPU, 2026-11-07 | 147 permille in the recorded idle run; not an accuracy target | [S1-U08](../../testing/subsystem1/TEST_LOG.md#step-8-mqtt-image--operator-results-2026-11-07) |
| USB old priority 3 | Release interval 9995 µs; dispatch delay 4488 µs; mutex wait 15 µs; 18 dump-phase observations; worst task gap 14467 µs | [History: MTK3_USB_PRIORITY](../archive/kernel/HISTORY.md#files) |
| USB21, image `94015fd` | Cold USB enumeration passed; two GP20 dumps, 206 rows/7 events, drops/delta 0; second DIAG: observations 0, gap 9963..10041 µs | [History: MTK3_USB_PRIORITY](../archive/kernel/HISTORY.md#files) |
| Full MQTT TEST output under USB21 | Not separately run in that USB capture; inspected burst about 1.5 KB fits ring | [History: MTK3_USB_PRIORITY](../archive/kernel/HISTORY.md#files) |

The cold-power 1 µs TIMER source is selected in kernel startup, before
radio/devices/tasks. Application verification is idempotent. The production
clock check permits 2% error; S0-U01 checks 1% over its longer window. A warm
picotool reboot alone cannot validate cold-power timing.

Motion's lifetime 9000..11000 µs gap gate is not widened. DIAG=1 records
fixed timing snapshots without a new queue, allocation or output in motion.
GP20 prints a bounded copy. The per-run gap-window proposal was dropped;
the TEST-only S2 measurement window remains. Physical stop latency, absolute
CPU accuracy and complete all-load stack bounds are not established here.

## Interfaces, tests and full evidence

- Source: [app_main.c](../../app_program/app_main.c),
  [app_task.c](../../app_program/platform/app_task.c),
  [motion.h](../../app_program/subsystem2/motion.h),
  [telemetry.h](../../app_program/subsystem1/telemetry.h).
- [IMU observer](../s4-imu/IMU.md), [measured RAM delta](#static-ram-growth-mqtt-robot-same-public-example-settings)
  and [pending S4 hardware](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-and-step-2b).
- [S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md),
  [S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md),
  [integration TEST_LOG](../../testing/integration/TEST_LOG.md).
- Full original records (old robot bring-up, stacks, CPU, watchdog, layout,
  motion and USB checks, and the removed kernel records: RAM budget per
  IMU step, core placement and lock audit, UP shim, Wi-Fi stack, staging
  merge): [kernel history](../archive/kernel/HISTORY.md).
