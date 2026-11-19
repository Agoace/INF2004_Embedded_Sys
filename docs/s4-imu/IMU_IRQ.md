# IMU INT1 watermark wake

Status: **PARKED. Robot FAIL on image 13:** one missed edge → FIFO backlog → READ_FAILED → gate STOP. The build default is now `IMU_INT1_WAKE=0`. Fix F1 and the idle TEST-build bench check wait for the operator's decision.

## Robot FAIL on image 13

| Item | Value |
|---|---|
| Image | 13: staging + IMU A + F pack + KR1 row + this change, FTH=3, `IMU_INT1_WAKE=1` |
| Boot | `c07b3039`; about 11 s after a flat boot, robot at start |
| Event | robot/event kind 10 IMU safety STOP, trigger 3 = IMU_UNAVAILABLE, cause 5 = READ_FAILED; motion ESTOP latched |
| Image 11 (no INT1) | No such stop |

Mechanism (analysis; the hardware reason for the missed edge is not yet measured):

1. After the first INT1 edge, the task waits for edges. If one rising edge does not come, the fallback waits 12 ms plus up to 10 ms to the next cyclic tick plus task latency.
2. At about 400 Hz, this gives 9 or more samples, which is more than `IMU_FIFO_MAX_BATCH` 8. `collect_fifo` (`imu_probe.c`) resets the FIFO, and the read is not valid.
3. `safety_tick` publishes `read_failed`. The gate (`vehicle/imu_gate.c` `ig_step`) latches IMU_UNAVAILABLE when `active` is true.

The earlier claim "at most one READ_FAILED" was wrong as a safety claim: ONE READ_FAILED is a STOP in a run. The SIL below did not include the safety and gate code, and its INT1 model had no single missed edge and no stuck-HIGH level.

New acceptance: `test_imu_wake_acceptance` requires backlog == 0 in the dead-INT1 and single-missed-edge cases (391/400/409 Hz, ">" and ">="). The parked design fails 6 of 12 cases. The test is registered `WILL_FAIL` until F1.

Fix options: F0 (done) default 0. F1 (proposed): the cyclic wakes the task if GP7 is HIGH, or if the last FIFO read is more than 8 ms old. F2 (drain backlogs) and F3 (gate tolerance) are not recommended.

## Summary

| Item | Value |
|---|---|
| Build switch | `IMU_INT1_WAKE` (0 or 1) in `build_make/pico_rp2040.mk` |
| Default | 0 (parked). 1 is allowed only with `SMP=0 BENCH=0 BENCH_KERNEL=0 IMU_SAFETY_BENCH=0` |
| Invalid request | `IMU_INT1_WAKE=1` with an ineligible profile stops the build |
| Off image | `IMU_INT1_WAKE=0` adds no flag, object, list or name part. `=1` adds `_int1` to the name. |
| Pin | GP7 = LSM303 accel INT1 |
| Registers | CTRL_REG3_A 0x22 = `0x04` (I1_WTM). FIFO_CTRL_REG_A 0x2E = `0x83` (stream, FTH=3). `recover_fifo` writes `0x83` again. |
| Interrupt | Existing core-0 GPIO dispatcher (`app_program/platform/drv_gpio_irq.c`). No SDK callback. No template deviation (check stays at 8). |
| Revert | Revert the one merge of `mtk3-imu-irq`, or build with `IMU_INT1_WAKE=0` |

## How it works

1. The accel FIFO fills at the output data rate (ODR, measured about 391 Hz).
2. When the FIFO count reaches the watermark, INT1 goes HIGH.
3. The GPIO interrupt calls `int1_irq` (in RAM). On a rising edge, it stores the time and calls `tk_wup_tsk(IMU task)`. It does no I2C.
4. The IMU task reads the FIFO with the same code as before. The read drains the FIFO, and INT1 goes LOW.
5. If INT1 is still HIGH after the sample, the task does ONE extra pass. It does this only when `probe.ready` is true.

The sample, average, filter, hump and safety code paths do not change. Only the wake moment changes.

## Fallback when INT1 is dead

The 10 ms cyclic handler stays. It wakes the task only when both of these are older than 12 ms (`IMU_WAKE_FALLBACK_US`):

- the last rising edge on INT1;
- the last FIFO read.

The read condition is necessary. After a late task read, the next edge comes up to 4 samples (about 10.2 ms) after that read. Without the read condition, the SIL showed about 50 false fallbacks per 6 s.

The first fallback sets POLL mode. In POLL mode, every 10 ms tick wakes the task, which is the same as the code before this change. The next rising edge clears POLL mode. POLL mode is also the start state.

## FTH default

FTH=3 is the default. The datasheet says the WTM flag is set when the FIFO content "exceeds" the watermark. The hardware meaning of "exceeds" is not verified.

| WTM meaning | FTH=3 batch | FTH=4 batch |
|---|---|---|
| count > FTH | n=4, about 10.2 ms | n=5, about 12.8 ms: above 12 ms, so false fallbacks |
| count >= FTH | n=3, about 7.7 ms | n=4, about 10.2 ms |

FTH=3 stays below the 12 ms fallback limit in both cases. The R1 histogram (`[S4_INT1] ... n0..8`) shows which meaning the hardware uses. If n=4 is the mode, then "count > FTH" is true and FTH=3 stays. If n=3 is the mode, then FTH=4 is also safe; build with `IMU_WAKE_FTH=4` only after a change review.

## Host evidence (SIL, no hardware; NOT a safety pass)

Source: `tests/test_imu_wake.c`. The SIL models the FIFO, the INT1 level, the edge interrupt, the 10 ms cyclic handler, task latency of 0–6 ms, a 1.3 ms read, and the probe rules (read n≤8; n>8 = backlog reset). Each run is 6 s. The table shows the 391 Hz rows.

| Case | Batch n (mode) | Fallbacks | Backlog resets | Max valid gap | Max observation gap |
|---|---|---|---|---|---|
| FTH 3, ">" , latency ≤1 ms | 4 (100 %) | 0 | 0 | 11.1 ms | 11.1 ms |
| FTH 3, ">=", latency ≤1 ms | 3 (100 %) | 0 | 0 | 8.6 ms | 8.6 ms |
| FTH 3, ">" , latency ≤6 ms | 4–6 | 0 | 0 | 16.1 ms | 16.1 ms |
| INT1 dead at 3 s (R3), ">" | 2–6 | 298 | 1 | 35.2 ms | 21.9 ms |
| INT1 dead at 3 s (R3), ">=" | 2–6 | 298 | 1 | 31.7 ms | 26.1 ms |
| FTH 4, ">" (rejected) | 4 | 499 (every tick) | 0 | 10.0 ms | 10.0 ms |

The assertions pass at 391, 400 and 409 Hz:

- With live INT1: no backlog, n≤8, valid gap ≤20 ms, data age <30 ms, ≤2 fallbacks per run.
- With INT1 dead (R3): the first fallback comes 12–28 ms after the last edge. There is at most 1 backlog reset (1 READ_FAILED observation). The observation gap is ≤30 ms (the clear-gap rule) and the valid gap is <50 ms (the stale rule). After that, 10 ms polling continues. **This is NOT a safety pass:** the one READ_FAILED latches a STOP in a run (image 13).

Other tests:

| Test | Checks |
|---|---|
| `test_imu_wake` (policy part) | rise/fall, POLL start state, 12 ms boundary, µs wrap, one extra pass, histogram |
| `test_imu_probe_int1` | probe init writes and reads back `0x04` and `0x83`; recovery writes `0x83` again |
| `test_imu_wake_service` | GP7 client registration, edge wake, fall ignored, fallback, one extra pass, stuck-HIGH bound, not-ready bound, empty bin 0, lost wake count |
| `test_imu_wake_service_add_failure` | refused GPIO client: 10 ms polling continues |

The tests are registered in `tests/imu_wake_checks.cmake`. The `=0` tests in `tests/imu_checks.cmake` do not change.

## Build evidence

All builds use public example secrets.

| Gate | Result |
|---|---|
| GCC host suite | 705/705 (includes `test_imu_wake_acceptance`, expected FAIL: 6 of 12 cases) |
| Clang ASan/UBSan host suite | 705/705 |
| Branch merged with SW11 `f47e3c0` (temporary worktree) | `git merge-tree` clean; 708/708; demo 0 warnings; RAM_PROOF 80/80 |
| Demo, `IMU_INT1_WAKE=1` | 0 warnings; RAM_PROOF PASS 80/80, 21292 B (staging: 79/79, 21212 B) |
| `IMU_STREAM=1` TEST build | 0 warnings; `int1_irq` linked |
| `template check` / `check-docs` | 8 deviations / OK |

Demo command (in `build_make/`): `make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE`.

### Off image identity

| Image | Staging `290439c` UF2 | This branch UF2 |
|---|---|---|
| Demo, default (`IMU_INT1_WAKE=0`) | `83f9192a…b348337e` | `83f9192a…b348337e` (identical; same file name; RAM_PROOF 79/79, 21212 B) |
| SMP=1 default (forced 0) | `398d88c5…2e0f6e38` | `398d88c5…2e0f6e38` (identical) |
| BENCH=1 (forced 0) | `b7df88d3…e201d487` | `b7df88d3…e201d487` (identical) |

For the demo default build, all 225 objects are byte-identical to staging after `--strip-debug`. The ELF files differ only in debug line data.

### Size (demo, `IMU_INT1_WAKE=1` against staging)

| Item | Value |
|---|---|
| UF2 / ELF sha256 | `9b4471db…16e6f7b5` / `4bf78beb…813ce7da` |
| Flash text | 468260 → 468628 B (+368) |
| bss | 216912 → 216984 B (+72) |
| Static RAM (`ram_report.py`) | 239304 → 239456 B (+152) |
| New RAM | `.bss.wake` 72 B; `.data.ramfunc.int1_irq` 80 B (list `tools/kernel_ram/imu_int1.list`) |
| `int1_irq` calls | `drv_time_us`, `tk_wup_tsk` (both in RAM) |

Stack and CPU time were not measured. CPU time is expected to stay the same, because the I2C busy-wait read (work_us about 2.7 ms) does not change.

## Robot checks (operator, after merge)

The `[S4_INT1]` line (TEST build) prints `rise fall ticks fallback extra lost n0..8 fth`.

| ID | Check | Pass |
|---|---|---|
| R1 | FIFO register readback and n histogram | `[S4_FIFO]` shows fifo_ctrl `83` (the probe init checks the CTRL3 readback); one n bin ≥90 % |
| R2 | Timing | fallback count stays near 0 with INT1 live; `lost` = 0 |
| R3 | Dead INT1 | Host SIL only (above). No robot build. |
| R4 | 3 hump drives | Same hump results as before |
| R5 | Lift/overturn | Stop as before |
| R6 | Demo line run with GP20 dump | No change to line or barcode results |

## Files

- `app_program/subsystem4/imu_wake.h` — pure wake policy and constants.
- `app_program/subsystem4/imu_service.c/.h` — ISR client, fallback check, extra pass, `imu_int1_stats_read`.
- `app_program/subsystem4/imu_probe.c` — register values under the switch.
- `app_program/subsystem4/imu_hw.c/.h` — `imu_hw_int1_high()`.
- `app_program/subsystem4/subsystem4_test.c` — `[S4_INT1]` print.
- `build_make/pico_rp2040.mk`, `tools/kernel_ram/imu_int1.list`.
- `tests/test_imu_wake.c`, `tests/test_imu_wake_service.c`, `tests/test_imu_probe.c`, `tests/imu_wake_checks.cmake`.
