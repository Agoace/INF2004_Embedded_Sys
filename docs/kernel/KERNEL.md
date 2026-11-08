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
10 ms IMU cyclic -> IMU14 -> read-only board -> TEST20 USB observer
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
| IMU, robot/TEST | 14 | Own 10 ms cyclic; bounded probe/count reads, pure observe-only core and board | 2048 |
| Wi-Fi | 20 | Template poll loop; sole CYW43/lwIP/MQTT owner | 8192 with lwIP (template-owned) |
| TEST runner | 20 | Initial task after startup; no normal vehicle/bench task | Initial task stack |
| USB console | 21 | IRQ wake or 10 ms fallback; console drain | 2048 UP / 4096 SMP (template-owned) |
| telemetry | 30 | Message arrival; RAM rows/events and paced USB dump | 4096 |
| health, MQTT only | 31 | 1 s cyclic; counters and application-stack scan | 2048 |
| idle meter, MQTT only | 32 | Always ready, preemptible; CPU estimate | 512 |
| motion bench, BENCH only | 11 | Motion after-tick wake; exclusive alternative to vehicle | 4096 |

The motion notification slot has one consumer: vehicle or bench, never both.
Task functions do not return. They loop or call `tk_ext_tsk()`.

### Interfaces and kernel objects

| Boundary | Interface and rule |
| --- | --- |
| Motion | `motion_*` calls use a priority-inheritance mutex (a waiting high-priority caller raises the holder's priority); no printing or sleeping while held |
| Line | 128 queue slots, 127 usable; one timestamped ADC sample per interrupt; inherited snapshot mutex; pure line code receives L/C/zero, not barcode ADC |
| Sonar | 256-byte request message buffer; copied requests with generation, ID, angle, not-before time and deadline; result mutex and event flag |
| Telemetry | 4096-byte message buffer, by-value messages, no producer wait; array owner is telemetry30 |
| Network | Bounded read-only boards, event/command queues; never a control mutex or PWM writer |
| IMU | Bounded motion-count copy and own snapshot board; atomic vehicle calibration epoch, no motion mutex or commands |

Only core 0 enables ADC/GPIO interrupts. GPIO IRQ13 is interrupt priority 1;
ADC TIMER alarm0 uses 2; USB IRQ5 keeps 0; SysTick/PendSV use 3. These are
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
`TA_USERBUF` and a 0xa5 fill pattern. Failed start deletes the dormant task.
The supplied size already includes system/context space; the kernel does
not add its normal 128-byte allowance.

| Reserve/use | Bytes | Meaning |
| --- | ---: | --- |
| Six original regions, including exclusive bench | 19456 | Reserved even when some tasks do not start |
| IMU region, not BENCH | 2048 | Static reserve in robot/TEST, physical high-water pending |
| MQTT extra health + idle meter | 2560 | 2048 +512 |
| All registered regions in MQTT image | 24064 | Includes IMU and the inactive bench region |
| Active normal robot, no MQTT | 17408 | Six task stacks, including IMU |
| Active normal MQTT robot | 19968 | Six tasks + health + idle meter |
| Low-margin threshold | 256 | Untouched bytes required per active application stack |

`app_task_stacks()` scans the fill pattern from the low end and keeps the
largest observed use. TEST and GP20 print size, high-water use and remaining
bytes. Health scans once per second. This is an estimate, not an overflow
detector or proof of worst-case depth. Initial, USB, radio and interrupt
stacks are outside this measurement. Exercise calibration, every navigation
module, queue saturation and dumps before accepting a stack budget.

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
| USB old priority 3 | Release interval 9995 µs; dispatch delay 4488 µs; mutex wait 15 µs; 18 dump-phase observations; worst task gap 14467 µs | [USB record](../archive/kernel/MTK3_USB_PRIORITY.md) |
| USB21, image `94015fd` | Cold USB enumeration passed; two GP20 dumps, 206 rows/7 events, drops/delta 0; second DIAG: observations 0, gap 9963..10041 µs | [USB hardware result](../archive/kernel/MTK3_USB_PRIORITY.md#hardware-result-2026-11-07-pass) |
| Full MQTT TEST output under USB21 | Not separately run in that USB capture; inspected burst about 1.5 KB fits ring | [USB limits](../archive/kernel/MTK3_USB_PRIORITY.md) |

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
- [IMU observer](../s4-imu/IMU.md), [measured RAM delta](RAM_BUDGET.md#imu-step-2-measured-delta)
  and [pending S4 hardware](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-not-run).
- [S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md),
  [S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md),
  [integration TEST_LOG](../../testing/integration/TEST_LOG.md).
- Full original records and all earlier build sizes, frames, timing and
  revision-specific results: [robot](../archive/kernel/KERNEL_ROBOT.md),
  [stacks](../archive/kernel/APPLICATION_TASKS.md),
  [CPU](../archive/kernel/CPU_LOAD.md),
  [watchdog](../archive/kernel/KERNEL_WATCHDOG.md),
  [layout checks](../archive/kernel/MTK3_LAYOUT_CHECKS.md),
  [motion bring-up](../archive/kernel/MTK3_MOTION_CHECKS.md),
  [USB](../archive/kernel/MTK3_USB_PRIORITY.md).
