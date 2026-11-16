# Project status

One row per area. Read this page before you propose work. Update the row in
the same commit as a change of state. Details stay in the linked records.

State date: 2026-11-16. Staging tip at writing: `a0cd97b`.

## Demo image

```sh
make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 [BARCODE_MAP="F=LEFT"]
```

Check `.ramproof` PASS and the boot line `[BARCODE_TABLE]`. See
[BUILD.md](../kernel/BUILD.md) and [KERNEL.md (KERNEL_RAM)](../kernel/KERNEL.md#ram-placement-kernel_ram).

## Areas

| Area | State | Evidence | Open |
| --- | --- | --- | --- |
| LINE (P7, two sensors) | Completes the course. Zigzags on straights: about +-40..54 deg/s, period about 0.4 s (2026-11-16). | [LINE.md](../s3-line-barcode/LINE.md) | Zigzag reduction (option C): real-log analysis first, then SIL, then tuning. |
| SEARCH / reacquire | Works on the course. SIL 6/6 reacquired. | [LINE.md](../s3-line-barcode/LINE.md) | None. |
| Motion controller | Done. Cap 460 mm/s, rise limit 8. | [MOTION.md](../s2-motion/MOTION.md) | Flat-floor motor test only if the zigzag analysis points at the motor. |
| Obstacle, scan, detour | Detours 2/2 in each of 6 course runs (2026-11-16). | [OBSTACLE.md](../s5-obstacle/OBSTACLE.md) | DONE_NO_LINE and STALL_RIGHT cases. |
| Junction actions (B2a) | CLOSED. J1..J5 PASS on the floor with fake commands. | [JUNCTION_VALIDATION.md](../kernel/JUNCTION_VALIDATION.md) | Pending command has no timeout. J2 new pending policy Not run. |
| Barcode reader (B2b) | SHADOW only. Course 2026-11-16: 0 of 10 attempts decoded (robot zigzags). Straight TEST pass: large F decoded both ways. | [SHADOW_HARDWARE_RESULTS.md](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md), [BARCODE.md](../s3-line-barcode/BARCODE.md) | Size-tolerant reader (n 4..7 mm) on `mtk3-b2b`, not in staging, not proven on hardware. Reliable read needs less zigzag. |
| Barcode letter table | Done: `BARCODE_MAP` make switch, boot print. | [BUILD.md](../kernel/BUILD.md), [LIVE_WINDOW_READER.md](../s3-line-barcode/LIVE_WINDOW_READER.md) | None. |
| Barcode LIVE (steers) | Not enabled. Three gates: build flag with measured clearance, producer valid flag, table. | [JUNCTION_LIVE_ADAPTER.md](JUNCTION_LIVE_ADAPTER.md) | Blocked by reliable read. |
| IMU humps | Report only. 2 humps per course run (2026-11-16). | [IMU.md](../s4-imu/IMU.md) | Extra IMU requirement: another team member works on staging. H3 Not run. `hump_imu_fault=1` once (2026-11-16 BASE run). |
| Telemetry, MQTT, dashboard | Done. | [TELEMETRY.md](../s1-telemetry/TELEMETRY.md), [TELEMETRY.md operation](../s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard) | Run summary shows barcode ok/failed as "not implemented" (fix in progress). |
| Kernel UP | Robot image. Overruns 0, lost wakes 0. | [KERNEL.md](../kernel/KERNEL.md) | F-NVIC and idle-stack fixes on HOLD. Two assembler warnings for `rt_tl` (fix in progress). |
| KERNEL_RAM v2 | Robot PASS (rule b): peak load 676 vs 931 permille. | [KERNEL.md (KERNEL_RAM)](../kernel/KERNEL.md#ram-placement-kernel_ram) | Soft-float helpers stay in flash. |
| Dual-core split | PARKED. Demo uses UP. | [KERNEL.md (split parked)](../kernel/KERNEL.md#split-placement-parked-2026-11-15) | None until resumed. |
| CR1 change requirement | Motion round done. | [CR1.md](CR1.md) | Week 13: 110 ms telemetry, before/after tables. Sonar round waits for operator. |
| SIL (Webots) | LINE, SEARCH, curves, barcode A and F observe-only on `mtk3-hil`. | [SIMULATION.md](../simulation/SIMULATION.md) | Does not reproduce the real zigzag. Records live on `mtk3-hil`. |

## Robot run logs

Logs are local and not in Git. Paths are relative to the repository root of
the worktree named in each row.

| Date | Where | Image | Location |
| --- | --- | --- | --- |
| 2026-11-10 | unknown | B2a/B2b images | `build/mtk_logs/{s5_b2a_j5,sys_s1a,sys_s1b}_20261110_usb.txt` |
| 2026-11-11 | school | B2b SHADOW | `build/mtk_logs/b2b_shadow_F_20261111_usb.txt`, `robot-mqtt-20261111-163412.csv` |
| 2026-11-13 | school | `451e562` force0/split, P1 | `build/school_20261113/` (see `SESSION_NOTES.md`) |
| 2026-11-16 | school | `e06667d`, `2413c4a`, `b0c3a9a` BASE/RAM | `mtk3-core-review/build/kram_robot_<sha>/` |
| 2026-11-16 | home | `a0cd97b`, `7a6cb1c` | `mtk3-core-review/build/home_{straight,size}_<sha>/` |

The four 2026-11-10/11 USB files form one growing log: each file starts
with the full previous file. Count each KR dump once. The last
`=== connect` section of a robot log holds the GP20 dump.
