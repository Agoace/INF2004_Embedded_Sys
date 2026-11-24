# IMU safety synthetic bench

## Purpose

Current (2026-11-24): these are optional synthetic bench profiles, not the
normal IMU input. The demo uses the physical IMU and the safety stop path.
See [real-robot capture results](IMU_SAFETY_CAPTURE.md#results-staging-581ab85-image-3_imucap-37eba5a4).
The BENCH2/BENCH3 results below remain evidence only for their named images.

Two opt-in images measure the ported IMU safety path on the single-core
(UP) kernel without a physical IMU. In both, a synthetic 100 Hz stream
replaces only the physical probe read. The IMU core, the safety detector
and the boards are the production code. The results are timing and
resource evidence. They are not robot safety acceptance.

| Value | Image | App | Vehicle | Record |
| --- | --- | --- | --- | --- |
| `IMU_SAFETY_BENCH=1` | `..._test_imu_safety_bench_kernel_ram.uf2` | TEST runner | Not started | BENCH2 |
| `IMU_SAFETY_BENCH=robot_synthetic` | `..._imu_synth_robot_kernel_ram.uf2` | Full robot demo app | Runs, owns the gate | BENCH3 |

Flash both images only on the spare Pico W (no sensors, no motors). The
ROBOT synthetic image runs the full robot app. On a robot with motors, a
GP21 press starts the calibration spin. That robot also has a real IMU,
which this image does not read.

## Build

Use a clean build after each profile change. Any other value is a make
error. Both values require `SMP=0`, `WIFI=cyw43`, `NET=lwip`, `MQTT=1`,
`KERNEL_RAM=1`, and exclude `IMU_STREAM`, `BARCODE_CAPTURE`,
`BARCODE_FAIL_CAPTURE` and `BENCH_KERNEL`. Value `1` also requires
`TEST=1 BENCH=0`. Value `robot_synthetic` requires `TEST=0 BENCH=0`.
The default value is `0`. At the original bench integration, the 38 default
profiles stayed byte-identical.

```sh
# BENCH2: TEST runner
make -C build_make clean SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 TEST=1 IMU_SAFETY_BENCH=1
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 TEST=1 IMU_SAFETY_BENCH=1
# BENCH3: full robot app
make -C build_make clean SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 IMU_SAFETY_BENCH=robot_synthetic
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 IMU_SAFETY_BENCH=robot_synthetic
```

Build measurements at the original bench integration:

| Image | Warnings | RAM_PROOF | KERNEL_RAM lists |
| --- | --- | --- | --- |
| TEST | 0 | 62/62, 8592 B | `common.list` + `imu_safety_bench.list` |
| ROBOT synthetic | 0 | 77/77, 19544 B (same as demo) | `common.list` + `robot.list` (unchanged) |

[`imu_safety_bench.list`](../../tools/kernel_ram/imu_safety_bench.list) omits
the barcode entries from [`robot.list`](../../tools/kernel_ram/robot.list) and
the entries in [`same_tick.list`](../../tools/kernel_ram/same_tick.list).
Those paths have no callers in the exclusive TEST image. These separate
bench lists do not change ordinary robot RAM placement.

## Mechanism

- `build_make/imu_safety_bench.mk` adds `-DIMU_SAFETY_BENCH=1`,
  `-DIMU_SAFETY_BENCH_ROBOT=0|1` and two forced includes. `imu_service.o`
  receives `imu_safety_bench/service_impl.h`. `app_main.o` receives
  `main_impl.h` (TEST) or `robot_impl.h` (ROBOT). Each profile links only
  its own reporter (`bench.c` or `robot.c`).
- `service_impl.h` maps `imu_probe_init` and `imu_probe_read` to the
  replay. The physical probe functions are not in either ELF. It also
  wraps three calls with timers: `imu_core_step`, `imu_hump_diag_step`
  and `imu_safety_step`. The wrapped functions do not change.
- The wrapper on `app_board_publish` adds each IMU snapshot to per-phase
  statistics and publishes one statistics board (568 B) per tick.
- TEST: `main_impl.h` selects the exclusive TEST branch. No destructive
  test, watchdog test, motor command or physical IMU read runs. **The
  Vehicle task does not run in TEST images** (`vehicle_task.c` builds the
  task only for `!APP_TEST && !APP_BENCH`). The gate, ESTOP, FAULT and
  event path are not in the TEST image.
- ROBOT: `robot_impl.h` maps `vehicle_task_start` to a hook. The hook
  starts the real Vehicle task, then a reporter task (priority 29, 2048 B
  stack). The reporter only reads boards and prints. It sends no command
  and no clear request. The Vehicle owns the gate and recovery.

## Timed sections

Each section is wall time in the IMU task, nested in `compute_us`. A
higher-priority task or interrupt that preempts a section adds to it.

| Section | Contents |
| --- | --- |
| core | `imu_core_step`, both calls when a calibration replay occurs |
| diag | `imu_hump_diag_step` |
| safety | `imu_safety_step` (the detector) |
| rest | `compute_us` − core − diag − safety: capture tick, board publishes, clear handling, timer reads |

`[IMU_BENCH_SPLIT]` gives mean/max/min per section and per phase.

## Synthetic streams

Both streams give 4 FIFO samples per 10 ms read, ±3 mg jitter, and an
invalid magnetometer. Axes: X right, Y forward, Z up. Tilt is a rotation
about Y through near-1 g poses, 30 ms per step, to 85°. Lift is
Z = 300 mg for 200 ms. Phase labels are the same in both streams.

| Phase | Meaning |
| --- | --- |
| 0 | upright idle |
| 1 | tilt |
| 2 | upright after tilt (includes the 320 ms return) |
| 3 | lift |
| 4 | upright after lift |

TEST stream: a 40 s period, from the first service read. 0–12 s phase 0,
12–20 s phase 1, 20–28 s phase 2, 28–32 s phase 3, 32–40 s phase 4.

ROBOT stream: upright (phase 0) until the Vehicle begins a calibration.
A start is a new odd calibration epoch, or a whole start/end pair between
two reads. 2000 ms after a start, one episode runs. Starts alternate:

| Start | Episode | Detector result |
| --- | --- | --- |
| 1st, 3rd, ... | tilt 3000 ms, return 320 ms, then upright (phase 2) | OVERTURN about 2.3 s after the start |
| 2nd, 4th, ... | lift 200 ms, then upright (phase 4) | LIFT about 2.0 s after the start |

A start during a scheduled or running episode adds no episode. With no
start, the stream stays upright for any time. Host tests
`test_imu_replay`, `test_imu_replay_service` and
`test_imu_replay_service_robot` prove these results with the real
detector and the real service source.

## BENCH3 operator procedure (ROBOT synthetic, spare Pico W)

1. Start the broker and the MQTT capture first.
2. Flash the UF2. Attach USB at any time. The reporter repeats
   `[IMU_BENCH_CONFIG]` every 5 s and `[IMU_BENCH_HINT]` every 5 s and on
   each change, so a late attach loses no instruction.
3. Do not press until `[IMU_BENCH_HINT] ... PRESS_GP21_ONCE_TO_START`.
   This is at uptime 70 s. Before that, the hint is
   `IDLE_WINDOW_DO_NOT_PRESS` (idle load window, Vehicle idle).
4. Press GP21 once (start 1). The Vehicle starts calibration. On the
   spare Pico W, the motion stall check stops the calibration after
   about 0.1–0.3 s (no encoders). At about +2.3 s, OVERTURN latches with
   the Vehicle idle. Expect the ESTOP, FAULT, event kind 10 word 1, then
   `[IMU_SAFETY] stop 1 cause N`.
5. Wait for `PRESS_GP21_ONCE_TO_RECOVER` (about +6.4 s after press 1:
   tilt ends at +5.3 s, then a 1 s clear window). Press GP21 once
   (recover). This press only clears. It does not start.
6. Wait for `PRESS_GP21_ONCE_TO_START`. Press GP21 once (start 2). At
   about +2.0 s, LIFT latches. Expect the same outputs with word 2 and
   `[IMU_SAFETY] stop 2 cause N`.
7. Optional: wait for `PRESS_GP21_ONCE_TO_RECOVER` and press once to show
   a second clear.
8. Wait for one more `[IMU_BENCH_SPLIT]` block (every 10 s). Then press
   GP20 for the dump. Record USB for 20 s more.

If a hint says `WAIT` or `WAIT_CLEAR_WINDOW`, do not press. A press while
latched before the clear window is complete gives a refusal event. That
is correct behaviour, but it is not the planned sequence.

### What MQTT shows

- `state`: Vehicle state 2 (FAULT) after each stop. Back to 0 after each
  recovery. The IDLE cadence sends `state` once per second, so the FAULT
  state arrives up to 1 s after the stop.
- `event`, for each start, in this order: kind 3 word 3 (motion stall
  faults, left and right), kind 5 (run end), then at the stop kind 3
  word 4 (`MC_FAULT_ESTOP`) and kind 10 word 1 or 2 (IMU STOP), detail =
  cause. Each recovery press gives one kind 10 PENDING event
  (`0x8000 | latched << 8 | 8`), then one CLEARED event
  (`0x8000 | latched << 8 | 0`). See
  [MQTT_WIRE.md](../s1-telemetry/MQTT_WIRE.md#kind-10-imu_safety).
  The BENCH3 image (`3809dec`) sent these as kind 3, without CLEARED.
- `summary`: **not published on the spare Pico W.** The summary exists
  only for a run that passed calibration. With no line sensors and no
  encoders, calibration cannot complete. The stop reason text is in the
  USB INFO and DUMP lines.

## BENCH3 USB lines

| Line | Read |
| --- | --- |
| `[IMU_BENCH_START] robot_synthetic reporter 0` | Boot only. Lost if USB attaches late. |
| `[IMU_BENCH_CONFIG] robot_synthetic ...` every 5 s | Profile, uptime, episodes, delay, idle window, reporter stack free. |
| `[IMU_BENCH_HINT] uptime_ms T HINT ...` | Operator instruction, latched trigger, clear_ms, Vehicle state, active, response_max_us, overruns, lost wakes. |
| `[IMU_BENCH] ...` every 10 s | missed_wakes, lost_wakes, gap_max_us of the IMU task. |
| `[IMU_BENCH_PHASE] phase P ...` | Per-phase compute mean/max/min, work mean/max, overruns, trigger bits. |
| `[IMU_BENCH_SPLIT] phase P core a/b/c diag ... safety ... rest ...` | Section mean/max/min in µs. |
| `[IMU_BENCH_CPU] aggregate_permille N` | Aggregate core load from Health. There is no per-task CPU. |
| `[IMU_SAFETY] stop T cause C` | Once per stop, from the Vehicle idle loop. |

Pass for BENCH3: overruns 0, lost wakes 0, one stop per start with the
expected trigger, each recovery press clears, and no task stack below
the 256 B low-stack margin in the dump.

## BENCH3 record (ROBOT synthetic image, `3809dec` + home secrets)

Image `3_IMU_SYNTH` (3dd1ff2f). Spare Pico W, no sensors or motors.
Files: `build/home_imu_3809dec/robot_BENCH3.final.log` and
`mqtt_BENCH3.csv` in the core-review worktree. All times are wall time.
The stream is synthetic and has no I2C cost. Values are at uptime
202805 ms unless stated.

Operator sequence: start at 80.98 s, OVERTURN stop at 83.26 s; recover at
110.56 s; start at 145.00 s, LIFT stop at 147.00 s; recover at 163.99 s;
GP20 dump; `tm_usb_dropped 0`.

| Value | Result |
| --- | --- |
| IMU overruns, missed wakes, lost wakes | 0, 0, 0 |
| IMU gap_max_us | 17687 |
| Vehicle response_max_us, overruns, lost wakes | 1299, 0, 0 |
| MQTT | 1 connection; publish_ok 273, publish_failed 18 |
| Aggregate CPU (permille) | MQTT heartbeats 326–370 (46); USB `[IMU_BENCH_CPU]` 339–348 (20); USB `[load]` 346 |
| Stack free (B) | IMU 512 (high water 1536 of 2048), Vehicle 2464, telemetry 2992, Health 1272, idle_meter 520, motion 2576, line 1576, sonar 1720, cyw43 7008, reporter 896 of 2048 |

Per phase, compute and work in µs (mean / max / min), and trigger bits:

| Phase | Samples | Compute | Work (mean / max) | Trigger bits |
| --- | --- | --- | --- | --- |
| 0 upright idle | 8270 | 1180 / 1701 / 420 | 1510 / 2107 | 1 |
| 1 tilt | 300 | 1353 / 1742 / 1162 | 1758 / 2125 | 3 |
| 2 after tilt | 6101 | 1181 / 1694 / 888 | 1514 / 2088 | 3 |
| 3 lift | 20 | 1299 / 1647 / 1152 | 1645 / 2045 | 5 |
| 4 after lift | 5563 | 1180 / 1690 / 875 | 1512 / 2088 | 5 |

Sections in µs (mean / max / min):

| Phase | core | diag | safety | rest |
| --- | --- | --- | --- | --- |
| 0 | 930 / 1277 / 305 | 50 / 215 / 4 | 118 / 325 / 39 | 80 / 279 / 63 |
| 1 | 1044 / 1203 / 911 | 55 / 206 / 43 | 165 / 348 / 112 | 87 / 256 / 69 |
| 2 | 936 / 1260 / 495 | 51 / 212 / 40 | 113 / 327 / 81 | 80 / 288 / 66 |
| 3 | 971 / 1248 / 865 | 55 / 75 / 46 | 185 / 355 / 133 | 86 / 121 / 71 |
| 4 | 935 / 1199 / 501 | 50 / 206 / 39 | 113 / 323 / 76 | 80 / 278 / 67 |

What BENCH3 proves:

- With Vehicle, barcode, junction and Wi-Fi/MQTT running, the IMU task
  keeps its 10 ms period: no overrun, no missed or lost wake.
- `imu_core_step` is about 79% of the IMU compute time. The detector
  (`imu_safety_step`) is 113–185 µs mean and at most 355 µs.
- The Vehicle gate latches OVERTURN and LIFT, sends the ESTOP, enters
  FAULT, publishes the events, and refuses starts until a GP21 recovery.
  Each recovery press cleared the latch.

What BENCH3 does not prove:

- The stop of an active run. Both stops occurred with the Vehicle idle,
  because the motion stall check ended each calibration in about
  0.1–0.3 s. At BENCH3, the active-run stop path had host tests only.
  Later real-robot push-over stops are recorded in [capture results](IMU_SAFETY_CAPTURE.md#results-staging-581ab85-image-3_imucap-37eba5a4).
- Any I2C time, real mounting or real motion.

Findings in the BENCH3 image, and their status:

- Each recovery press published two identical events (kind 3 word
  `0x8108` or `0x8208`, PENDING, consecutive sequence numbers, same
  uptime). The Vehicle called `imu_recovery(IG_PENDING)` after
  `ig_press`, then again in the start-refusal branch of the same press.
  One producer, two calls. The successful clear had no event; only the
  state change to 0 showed it. Fixed in the kind 10 change: one PENDING
  per press and one CLEARED event.
- Event kind 3 had three word meanings: motion fault bits (1, 2, 4),
  IMU trigger (1, 2, 3) and IMU recovery (bit 15). Words 1, 2 and 3 were
  ambiguous on the wire. The source branch has the same encoding. Fixed
  in the kind 10 change: IMU events moved to kind 10; kind 3 is motion
  fault bits only.
- The KR1 dump line `faults 7` is the latched motion fault bits:
  `MC_FAULT_STALL_LEFT | MC_FAULT_STALL_RIGHT | MC_FAULT_ESTOP`. The
  stall bits come from the calibration with no encoders, the ESTOP bit
  from the IMU stop.

## BENCH2 record (TEST image, `f4ffd0e` + home secrets)

Image `2_IMU_BENCH` (e6bc1c71). Spare Pico W, laptop Wi-Fi power save
off. Files: `build/home_imu_f4ffd0e/robot_BENCH2.final.log` and
`mqtt_BENCH2.csv` in the core-review worktree. The USB logger attached
late, so the `[IMU_BENCH_START]` and `[IMU_BENCH_BEGIN]` lines and the
boot banner are missing.

| Value | Result |
| --- | --- |
| samples | 7070 |
| overruns, missed wakes, lost wakes | 0, 0, 0 |
| gap_max_us | 18532 |
| compute mean / max (µs) | 1124 / 1789 |
| work mean / max (µs) | 1459 / 2124 |
| compute mean per phase 0–4 (µs) | 1065 / 1239 / 1112 / 1102 / 1114 |
| trigger bits per phase 0–4 | 1 / 3 / 3 / 5 / 1 (OVERTURN in 1, LIFT in 3) |
| aggregate CPU (permille) | 272–325; load_run peak 398 |
| stack free (B) | IMU 528 (high water 1520), reporter 3336 |

Corrections to the first version of this runbook:

- The Vehicle does not run in TEST images. `[IMU_SAFETY] stop` and
  `[IMU_BENCH_VEHICLE]` cannot appear. The TEST reporter no longer prints
  a Vehicle line. BENCH2 does not prove the gate, ESTOP, FAULT or event
  path; host tests and BENCH3 cover them.
- `publish_failed` 99 (83 state + 16 heartbeats) in BENCH2 is a bench
  artefact. Without a Vehicle board, telemetry uses the RUN cadence
  (6 messages/s). lwIP holds each QoS 0 publish in one of 4 request slots
  until the TCP ACK. Home-link ACK stalls (delay p90 1.1 s, max 5.9 s)
  then give `ERR_MEM`. CPU is not the cause.

## Limits

- `compute_*`, `work_*` and the sections are wall time. They include
  preemption by higher-priority tasks and interrupts. They are not
  exclusive CPU time. There is no valid UP compute baseline from before
  the port.
- The replay has no I2C cost (`probe.last_us = 0`). The physical probe adds
  a bounded FIFO burst read. Measure that separately with the real IMU.
- The section timers and the 568 B statistics board copy add to the IMU
  task time in these images only.
- Not robot acceptance. The physical IMU, mounting and real lift/tilt
  motion are not tested here.
