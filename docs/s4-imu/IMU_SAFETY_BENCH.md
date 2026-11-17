# IMU safety synthetic bench (TEST only)

## Purpose

This image measures the ported IMU safety path on the single-core (UP)
kernel without a physical IMU. A synthetic 100 Hz stream replaces only the
physical probe read. The IMU core, the safety detector, the boards and the
Vehicle gate are the production code. The result is timing and resource
evidence for that path. It is not robot safety acceptance.

## Build

Use a clean build. The option is a make error unless all of these are true:
`TEST=1`, `BENCH=0`, `SMP=0`, `WIFI=cyw43`, `NET=lwip`, `MQTT=1`,
`KERNEL_RAM=1`, no `IMU_STREAM`, no `BARCODE_CAPTURE`, no `BENCH_KERNEL`.
Robot (non-TEST) images cannot select it.

```sh
make -C build_make clean SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 TEST=1 IMU_SAFETY_BENCH=1
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 TEST=1 IMU_SAFETY_BENCH=1
```

Output: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test_imu_safety_bench_kernel_ram.uf2`.
Expect 0 warnings and `RAM_PROOF PASS listed 62 in_ram 62`. The bench uses
`common.list` plus `tools/kernel_ram/imu_safety_bench.list`. The list is
`robot.list` without 13 barcode rows, and without the 2 `same_tick.list`
rows. In this TEST image, those 15 functions have no callers, so the linker
removes them (none is in the ELF). The ordinary lists do not change.

## Mechanism

- `build_make/imu_safety_bench.mk` adds `-DIMU_SAFETY_BENCH=1` and a forced
  include to two objects only. `imu_service.o` receives
  `imu_safety_bench/service_impl.h`. `app_main.o` receives
  `imu_safety_bench/main_impl.h`. The files in `app_program/imu_safety_bench/`
  compile only in this profile.
- `service_impl.h` maps `imu_probe_init` and `imu_probe_read` to the replay.
  It also wraps `app_board_publish`. After each IMU snapshot publish, the
  wrapper adds that snapshot to per-phase statistics. This is one extra
  bounded board copy in the IMU task for each tick.
- `main_impl.h` selects the exclusive TEST branch (`APP_IMU_STREAM` in
  `app_main.c` only). It runs no destructive test, no watchdog test, no
  motor command and no physical IMU read.
- A reporter task (priority 21, own 4096 B stack filled with 0xa5) runs for
  70 s. Each second it reads the safety board. When a latch has a complete
  clear window, it requests a clear. During the run, only the reporter
  requests clears. `imu_safety_clear_request` assumes one requester.

## Synthetic stream

Period 40 s, 4 FIFO samples per 10 ms read, ±3 mg jitter, magnetometer
invalid. Axes: X right, Y forward, Z up.

| Phase | Time in period | Input | Expected detector result |
| --- | --- | --- | --- |
| 0 | 0–12 s | upright, 1 g | none |
| 1 | 12–20 s | rotate about Y through near-1 g poses, 30 ms per step, to 85° | OVERTURN (atan2f path) |
| 2 | 20–28 s | return to upright in 270 ms, then upright | clear after window |
| 3 | 28–32 s | Z = 300 mg for 200 ms, then upright | LIFT (sqrtf/low-g path) |
| 4 | 32–40 s | upright | clear after window |

In 70 s, expect two OVERTURN and two LIFT latches. Host tests
`test_imu_replay` and `test_imu_replay_service` prove this with the real
detector and the real service source.

## Operator procedure (spare Pico W)

1. Use a Pico W with no sensors or motors. Start the broker first.
2. Flash the UF2. Record USB from boot until `[IMU_BENCH_DONE]`, then for
   20 s more.
3. Do not press GP20 or GP21 during the 70 s window. A GP21 press adds a
   second clear requester and can start a Vehicle recovery.

## USB lines to watch

| Line | Read |
| --- | --- |
| `[IMU_BENCH_START] result 0` | Reporter started. Any other result: stop, report. |
| `[IMU_BENCH_BEGIN] ...` | Start of the 70 s window. |
| `[IMU_BENCH] ... ` every ~5 s | samples, compute mean/max, work mean, `response_wall_max_us` (largest IMU work), overruns, missed_wakes, lost_wakes, gap_max_us. |
| `[IMU_BENCH_CPU] aggregate_permille N` | Aggregate core load from Health. There is no per-task CPU. |
| `[IMU_BENCH_VEHICLE] ... response_max_us ...` | Vehicle response, overruns and lost wakes from its public board. |
| `[IMU_SAFETY] stop 1 cause 0` | Expected once. The Vehicle gate latches on the first OVERTURN. It stays latched because recovery needs a local press. Later latches do not repeat the stop. |
| `[IMU_BENCH_PHASE] phase P ...` | Per-phase compute/work mean/max, overruns and trigger bits (bit 1 OVERTURN, bit 2 LIFT). Phase 1 must include bit 1. Phase 3 must include bit 2. |
| `app_task_report` lines | Stack minimum free for each task. |
| `rt_report` lines | ISR, line and kernel-call maxima for the epoch that the reporter opened. |
| `[IMU_BENCH_STACK] ... min_free N` | Reporter stack high water. |
| `[IMU_BENCH_DONE] completed_70s 1` | End. |

Pass for this bench: overruns 0, lost wakes 0, every phase has samples,
the phase 1 and phase 3 trigger bits are present, and no task stack is
below the 256 B low-stack margin.

## Limits

- `compute_*` and `work_*` are wall time. They include preemption by higher
  priority tasks and interrupts. They are not exclusive CPU time.
- The replay has no I2C cost (`probe.last_us = 0`). The physical probe adds
  a bounded FIFO burst read. Measure that separately with the real IMU.
- One reporter board copy per tick is added to the IMU task.
- Not robot acceptance. The physical IMU, mounting and real lift/tilt
  motion are not tested here.
