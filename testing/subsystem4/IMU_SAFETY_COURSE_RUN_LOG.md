# IMU safety course run log

## What this is

This record covers the first course run on the repaired stop path, and it
answers the questions in RT-0099. The operator drove; the Coder recorded the
MQTT capture only.

The result is **a stop that the safety path classified correctly, with a cause
that the image could not report**. Nothing in this record shows that the stop
was right or wrong. It shows what the wire carried.

## Identity

| Field | Value |
| --- | --- |
| Image | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` |
| Image SHA-256 | `75B0F16A9BFA635DB5ECF91176A39EDCD534200086FDDDB97F31A5046C5FE854` |
| Image size | 860160 bytes |
| Source commit in the image | `48b5f9b` |
| Not in the image | `781c1f5` (the atan2f optimisation) and every later commit |
| Build options | `SMP=0 WIFI=cyw43 NET=lwip MQTT=1 IMU_HUMP_SOURCE=1 JUNCTION_TEST_CMD=NONE` |
| Broker | `192.168.72.229:1883`, robot configuration, no persistence |
| Robot address | `192.168.72.191` |
| Capture file | `testing/logs/course-20261117-160506.csv` |
| Boot IDs in the file | `1397466534` (67 rows) and `30935004` (816 rows) |
| Run ID | 1, on boot `30935004` |

The image was built before the optimisation commit, so this run tests the
reviewed stop-path repair without the optimised detector. Do not combine this
record with a later image.

## What the wire shows

All times are robot uptime on boot `30935004`.

| t | Observation |
| --- | --- |
| 5.3 s | Boot. `state=0` IDLE, `active=0`, `motion_faults=0` |
| 31.2 s | `active=1`: the operator pressed GP21 |
| 34.8 s | `state=1` RUNNING, `module=1` line tracing |
| 41.8 s | Module 1 to 0, then to 2 (SEARCH): line lost |
| 43.7 s | Module to 0, then to 2 again: line lost again |
| 44.1 s | `state=2` FAULT, `motion_faults=4` `MC_FAULT_ESTOP` |
| 44.127 s | Live event kind 3, word 3: `IMU_SAFETY_IMU_UNAVAILABLE` |
| after | IDLE once the latch is cleared; no automatic restart |

Run summary:

```
run = 1
stop_reason = 16   (RM_STOP_IMU_UNAVAILABLE)
shell_fault = 6    (VSM_FAULT_IMU)
shell_fault_detail = 3
duration_ms = 9411
hump_count = 1, hump_peak = 149 (14.9 mm)
```

So the course segment lasted about 9.4 s, not 20 s. No lift and no overturn
reason appears, which matches the operator report.

## Why the image could not report the cause

`imu_safety_evaluate()` returns one value, `IMU_SAFETY_IMU_UNAVAILABLE`, for
three different conditions:

- the published sample is older than `IMU_SAFETY_MAX_AGE_MS` (50 ms);
- the newest sample is not valid or not calibrated;
- no copy was ever published, or every read was busy.

The MQTT image carried none of the IMU timing fields, because `robot/imu` is
reserved and not published, and the run event had no detail byte. The stop is
therefore classified but not explained.

## Load correlation

| t | cpu permille | publish_failed | publish_ok | wifi gap max us | mqtt hook max us |
| --- | ---: | ---: | ---: | ---: | ---: |
| 30.4 s idle | 539 | 0 | 31 | 20646 | 2508 |
| 32.2 s | 549 | 2 | 37 | 20646 | 2508 |
| 35.2 s | 612 | 7 | 63 | 31017 | 9396 |
| 37.2 s | 871 | 26 | 108 | 39998 | 10706 |
| 38.2 s | 900 | 27 | 134 | 39998 | 10706 |
| 41.3 s | 891 | 86 | 195 | 39998 | 19084 |
| 44.1 s | IMU stop | | | | |
| 46.3 s after stop | 600 | 279 | 277 | 39998 | 19084 |

`motion_late` and `motion_missed` stayed 0 for the whole run, and
`stack_min_free` stayed 552 B.

## What this correlation does not show

The earlier claim that Wi-Fi or MQTT starved the IMU task is **withdrawn**. It
does not hold with the measured task priorities:

| Task | Priority |
| --- | ---: |
| motion | 10 |
| line service | 11 |
| sonar service | 12 |
| vehicle | 13 |
| IMU service | 14 |
| Wi-Fi / MQTT | 20 |
| health | 31 |

A lower number is a higher priority. The Wi-Fi and MQTT task at 20 **cannot**
preempt the IMU task at 14. The tasks that can preempt it are motion, line,
sonar and vehicle, and they only do that work while a run is active. The
`mqtt_hook_max_us` and `wifi_poll_gap_max_us` rise are therefore effects of the
same load, not proof that network work caused the stop.

The load rise and the stop are correlated with the run. The cause is not
measured. It can be one of:

- a higher-priority task delaying the IMU task past 50 ms;
- an I2C read failure under preemption, which makes the sample invalid;
- the safety work itself stretching the IMU sample beyond its own deadline.

## Instrumentation added after this run

Commit `526cc67` adds, and the follow-up commit repairs after RT-0101:

- `imu_safety_unavailable_cause()`: separates stale, invalid and absent.
- The live `robot/event` for an IMU stop carries the cause in its detail byte,
  and a refused start carries it too. `docs/s1-telemetry/MQTT_WIRE.md` records
  the values.
- A pose refusal is reported as detail 4, not as an unavailable IMU, and a
  refused start while an unavailable latch is in force keeps the saved cause
  instead of reporting NONE (RT-0101/F2).
- The cause is saved once, at the first latch, so the console and the first
  MQTT event cannot disagree (RT-0101/F3).
- The IMU `compute_*` window now covers the safety step and the safety board
  publish. It is wall elapsed time, including preemption, not CPU time
  (RT-0101/F1).
- The USB console lines print `cause <name>`.

## Still not measured

- The cause of this stop. It needs a run with the instrumentation above.
- `gap_max_us`, `missed_wakes`, `work_max_us`, `compute_max_us` for the IMU
  task: these exist in the IMU board but are not on the wire.
- Named IMU task stack high-water.
- Higher-priority task timing under matched idle and course load.
- The 60 ms confirmed-trigger-to-motor-output-off gate: one-second state
  updates cannot show it.
