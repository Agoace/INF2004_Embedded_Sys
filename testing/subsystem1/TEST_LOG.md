# Subsystem 1 and shared platform test log

Subsystem 1 remains a stub. Its TEST hook runs the shared timer check in
the initial task; it creates no new task.

| ID | Check and procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S0-U01 | TEST image, both wheels lifted: remove all power, then apply USB or VIN power. Compare TIMER time with kernel time over a nominal 1 s window. Repeat after cold power-on for every kernel image. | Prints TIMER us and elapsed kernel ms; ratio differs from 1000 us/ms by at most 1%; kernel errors fail the check | PASS 2026-11-06, warm and cold (see below) |

`subsystem1_test.c` calls `drv_time_check_window(1000, 1, ...)` and prints
one PASS/FAIL line before the S2 tests. The actual kernel window can be
about 1010 ms because `tk_dly_tsk()` adds one kernel tick. Compare measured
time, not the requested delay. `drv_time_check()` separately checks a
short window with a 2% limit before motion is enabled.

The original TIMER source defect was found after a cold VIN power-on on
2026-11-06. Post-flash tests did not expose it. Do not run the floor bench
until S0-U01 passes after a cold power-on. Lift both wheels before power:
the TEST image's later S2-U02 test sends a real short speed command.

## Hardware run records: 2026-11-06

Image: `mtk3` 1806ed0 / `mtk3-motion` f0b4194, `make SMP=0 WIFI=none TEST=1`.
For the cold run, a local debug copy added only a wait for the USB terminal
(up to 15 s) and two prints of the WATCHDOG_TICK register at the start of
`usermain()`. That copy was not committed. Without the wait, the TEST output
of the first seconds is lost before the host opens the port.

| Start | WATCHDOG_TICK before `drv_time_init()` | After | S0-U01 | S2-U01 task gap |
| --- | --- | --- | --- | --- |
| Warm, after a picotool flash | `0x0000260c` (CYCLES 12, set by boot ROM) | `0x0000560c` | PASS: TIMER 1009986 us, kernel 1010 ms | 9770..10317 us |
| Cold, USB plugged in after all power removed | `0x000eae00` (CYCLES 0: the defect) | `0x0006660c` (CYCLES 12) | PASS: TIMER 1009991 us, kernel 1010 ms | 9458..10548 us |

S2-U02 also passed in both runs. Log of the cold run:
`build/mtk_logs/s0_cold_dbg_20261106-165924.txt` (ignored, not committed).

The committed bench image was then started cold on VIN for floor run B4
([subsystem 2 log](../subsystem2/TEST_LOG.md)). Its TIMER-measured task gap
was 9952..10046 us against the 10 ms kernel cycle, so the fix also holds in
the normal image after a cold VIN power-on.

## Task-split telemetry checks

| ID | Procedure and acceptance | Status |
| --- | --- | --- |
| S1-U02 | TEST: bounded producer burst at priority 1 fills MBF; sends return without waiting, lost count increases; after allowing consumer to run, accepted rows are retained | PASS 2026-11-06 (445084c and 3f889a1): accepted 34, lost 66, no blocking. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| S1-U03 | GP20 dump after a full robot session: valid KR1/VE CSV, telemetry loss 0, dump `tm_usb_dropped` delta 0, control deadlines remain within limits, per-task stack free margin >=256 bytes | PASS 2026-11-06 (M1-S7, M2-S7): telemetry loss 0, USB dump delta 0, overruns 0, stack free >= 1672 B. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |

## STEP 8 MQTT image — operator results 2026-11-07

STEP 7 outcomes above remain historical. Earlier STEP 8 operator results
are summarized below and in [the hardware record](../../docs/archive/MTK3_STEP8_RECORD.md).
They do not validate later boot/session, diagnostics or SCAN-retry changes.
Follow [the broker runbook](../../docs/s1-telemetry/MQTT_RUNBOOK.md).

| ID | Check | Result |
| --- | --- | --- |
| S1-U04 | MQTT connect, AP loss and broker restart/reconnect; retained OFFLINE will and ONLINE after CONNACK, keepalive 5 s (broker silence threshold about 7.5 s plus delivery/timer delay); no effect on boot, RAM log or driving | PASS for watchdog LWT, broker restart (~2 s reconnect), power cut (~7 s OFFLINE); AP-loss case not separately recorded |
| S1-U05 | Live robot/log stream (QoS 0, 10-row batches about every 100 ms, data age, jump/skip report, reconnect), DUMP QoS 1 refill time and merge by index, Wi-Fi hook max wall time with streaming on; nominal state5Hz/heartbeat1Hz, signed distance/duty and encoder heading/rate, completed obstacle profiles (width unavailable), detour records and retained run summary, CPU field, sonar history/event loss and signed binary CSV; Wi-Fi20 poll/work/gap and stack data during driving/dump | PASS for M1: live 216/216 rows byte-identical to USB, median age 20 ms, DUMP ~1 s; full rate/loss/stack checklist not separately recorded |
| S1-U06 | NORMAL robot: fallback ESTOP-only, configured-token matching all opcodes, idle START/CALIBRATE, duplicate/full queue, dedicated ESTOP, live DUMP/ABORT | PASS for uncalibrated START refusal, CALIBRATE/START/DUMP/ESTOP and wrong-token refusal (`command_lost +2`); other queue/duplicate cases not separately recorded |
| S1-U07 | MQTT TEST, idle board, wheels lifted: print calibrated CPU reference and load; establish expected range and independent target accuracy; verify deadlines and >=256 B stack margin | Target values observed: idle 117, traffic 139 permille; peak 4026729 iterations/s. Independent accuracy/range not established. [Method](../../docs/kernel/KERNEL.md) |
| S1-U08 | MQTT robot image, idle telemetry pause: idle for 10 s on the dashboard and recorder (state about 1 Hz with `telemetry_idle` 1, heartbeat about every 5 s, no live `robot/log` and no `robot/sonar`); the page shows `idle, telemetry paused` and `idle (paused)` and START/CALIBRATE stay enabled; CALIBRATE: state, heartbeat and the live log return at once (log jump reported); a run, then GP21 or MQTT ESTOP: full rates for about 2 s, then idle again; DUMP while idle completes with no live rows; ESTOP while idle is decided. Host evidence: `test_telemetry_mode`, `test_robot_mqtt_idle_*`, `test_log_stream_live_paused`, Python `TelemetryMode*` tests | PASS 2026-11-07, image `5e4e8ad` (boot `1c8f883c`): idle 5.4 min with state 1/s (`telemetry_idle` 1), heartbeat every 5 s, no `robot/log`, no `robot/sonar`, sonar history sequence 0 (no pings); dashboard CALIBRATE gave full rates in the same second; one idle state between calibration and START (2 s grace); run 2.5 s: state 5/s, log 34-47 frames/s, sonar 6-11/s; dashboard MQTT ESTOP stopped the run (summary stop source 2, reason 2); full rates for 2 s, then idle; DUMP while idle complete (125 rows, 8 events); `sonar_history_missed` 0; idle CPU load 147 permille. ESTOP while idle: PASS 2026-11-07 on home Wi-Fi (boot `1639107437`): dashboard ESTOP accepted (command result 0), ESTOP event with motor fault 4, robot still; CALIBRATE cleared the latch; START drove 322 mm and 354 mm in two runs, both stopped by dashboard ESTOP (stop source 2) |

Commands are not consumed by TEST/BENCH because those profiles do not start
vehicle. A broker PUBACK is not vehicle acceptance. GP21 remains the real
ESTOP in NORMAL; lift wheels for TEST. Network failures are not faults.
