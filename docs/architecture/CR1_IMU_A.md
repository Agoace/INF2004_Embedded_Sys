# CR1 IMU A: packed IMU shared boards

Approved scope (operator, 2026-11-19): category A storage of the three small
IMU boards. The IMU task keeps full `float` values. Only the shared copies
are packed. The snapshot board (504 B) and items B1–B7 are not approved and
do not change.

Base: `mtk3-staging` at `b68ce6b`. Work kept at tag `archive/mtk3-cr1-imu`. Code commit
`f47e3c0`.

## Rules

| Rule | How it is kept |
| --- | --- |
| Packing | 32-bit words, masks and shifts. No C bit-fields, no packed structs. No field crosses a word. |
| One writer | The IMU task writes the hump, diag and safety boards. Vehicle writes its `run_diag_board` copy. |
| Same values | Readers get the same summary v3, live HUMP event, gate actions and USB line. Proofs are below. |
| Teammate code | `imu_safety.c/.h`, `vehicle/imu_gate.c/.h`, `imu_safety_bench/*` and `imu_safety_capture/*` do not change. |
| MQTT | No wire change. All `rm_*` sizes are the same before and after. |

## Boards

| Board | Before | After | Words | Source |
| --- | ---: | ---: | ---: | --- |
| hump | 48 B | 16 B | 4 | [imu_hump.h:103](../../app_program/subsystem4/imu_hump.h#L103) |
| diag | 72 B | 36 B | 9 | [imu_hump_diag.h:103](../../app_program/subsystem4/imu_hump_diag.h#L103) |
| safety | 36 B | 12 B | 3 | [imu_safety_board.h:23](../../app_program/subsystem4/imu_safety_board.h#L23) |
| snapshot | 504 B | 504 B | — | not changed (B1) |

### Hump board (16 B)

The writer runs the float decisions of the summary once: the open-event
compare and the 0.1 mm rounding (`imu_hump_tenths`). The board carries the
results. A tie inside 0.1 mm therefore still picks the same side (LEFT flag).
Writer: [imu_hump.c:237](../../app_program/subsystem4/imu_hump.c#L237).
Reader: [imu_hump.c:279](../../app_program/subsystem4/imu_hump.c#L279).

| Field | Bits | Range | Note |
| --- | ---: | --- | --- |
| `timestamp_ms` | 32 | u32 ms | The 50 ms age test is unsigned 32-bit. 16 bits would wrap after 65.5 s. |
| `run_epoch` | 32 | u32 | Equality with the Vehicle epoch. |
| best peak | 16 | 0..65534 (0.1 mm) | `imu_hump_tenths()` output. Geometric range 0..1110 needs 11 bits; 16 bits costs no word. |
| last closed peak | 16 | 0..65534 (0.1 mm) | Live HUMP event word. |
| closed count | 16 | 0..65535 | Summary `hump_count` is u16 on the wire. |
| gated starts | 8 | 0..255 | The reader already clamps to 255. |
| flags | 5 | FAULT, OPEN, CURVE, LONG, LEFT | IMPLEMENTED is always set, so it is not stored. |
| open | 1 | bool | `running && state == ACTIVE`, the only state that a reader tests. |
| valid | 1 | bool | `valid_samples != 0`. |
| last LEFT | 1 | bool | Live event detail. |
| **Total** | **128** | | 4 words, 0 spare |

`smoothed_mm`, `event_travel_mm`, `long_rejects` and `smooth_ready` had no
board reader. The S4 TEST line reads `long_rejects` from the snapshot board.

### Diagnostic board (36 B)

Each value is the integer that the USB line prints
([imu_hump_diag.c:182](../../app_program/subsystem4/imu_hump_diag.c#L182),
round half away from zero). Signed values are two's complement. A value
outside its field saturates. This board is USB only.

| Field | Bits | Print unit | Range |
| --- | ---: | --- | --- |
| `run_epoch`, `timestamp_ms` | 32 + 32 | | u32 |
| `valid`, peak `valid`, `left` | 1 + 1 + 1 | | bool |
| peak `ms` | 20 | ms | 0..17.4 min |
| `travel_mm` | u16 | mm | 0..65.5 m |
| `corrected_mm`, `filtered_mm` | s12 + s12 | 0.1 mm | ±204.7 mm (track 111 mm) |
| `speed_mm_s` | s12 | mm/s | ±2047 (limit 460) |
| `yaw_rad_s` | s15 | mrad/s | ±16.38 rad/s (±11.5 reachable) |
| `yaw_accel_rad_s2` | s19 | mrad/s² | ±262 rad/s² (approved ±230) |
| `lateral_mg` | s13 | mg | ±4095 (approved ±600) |
| window yaw, yaw accel, lateral | u14 + u18 + u12 | as above | magnitudes |
| `gate`, `window_gate` | 4 + 4 | | gate bits |
| `line_samples` | 20 | count | 0..17.4 min at 100 Hz |
| `line_lateral_mg`, `line_yaw_accel` | u12 + u18 | as above | magnitudes |
| **Total** | **288** | | 9 words, 0 spare |

Peak bits and run bits never share a field. The writer keeps the packed peak
bits and packs them again only when the peak changes
([imu_service.c:191](../../app_program/subsystem4/imu_service.c#L191)).

### Safety board (12 B), option S2

| Field | Bits | Range |
| --- | ---: | --- |
| `observation_ms` | 32 | u32; signed age test |
| `applied_request` | 32 | u32 clear token |
| `clear_ms` | 10 | 0..1000 (clamped by the writer) |
| `latched_trigger` | 2 | 0..3 |
| `flags` | 8 | all 8 flags |
| `clear_blockers` | 4 | 4 block bits |
| spare | 8 | always 0 |

The board does not carry `tilt_ddeg`, `g_mg`, `vertical_mg`,
`overturn_samples`, `lift_samples`, `current_trigger` and `stops`. No board
reader uses them. They unpack as 0
([imu_safety_board.h:10](../../app_program/subsystem4/imu_safety_board.h#L10)).
`imu_safety_try_read()` unpacks into the same `imu_safety_report_t`
([imu_service.c:455](../../app_program/subsystem4/imu_service.c#L455)), so
the gate and the bench code do not change.

## Week 8 §11 mapping

| §11 field | In code | This record |
| --- | --- | --- |
| pitch 8 | internal only (snapshot) | not on a small board |
| heading 8 | no IMU heading | not added |
| turn rate 8 | encoder-derived yaw rate | diag board, s15 mrad/s |
| hump state 2 | `imu_hump_state_t` | 1 bit (`open`); readers test ACTIVE only |
| collision 1 | no collision detector | not added |
| valid 1 | hump and safety | 1 bit each |
| height 8, peak 8 | peak heights | 16 bits at 0.1 mm; 8 bits (0.44 mm) would change the summary |
| age/seq 16 | `timestamp_ms` | 32 bits; 16 bits wraps after 65.5 s |

## Proofs (host)

| Test | What it proves |
| --- | --- |
| `test_imu_hump_board` | 410089 cases: summary v3 and live HUMP event from the board equal a verbatim copy of the old code. Ties inside 0.1 mm, NaN, ±inf, saturation, count 65535, every flag, gated 0..65535, the 0.1 mm grid 0..6553.4 mm. |
| `test_imu_hump_diag_board` | 200000 in-range reports: the USB line from the board equals a verbatim copy of the old print. Saturation at every edge. Board finish equals report finish. Cached peak pack equals the full pack. |
| `test_imu_safety_board` | The carried-field list; every carried value round-trips; the 7 others read 0; 400000 random gate steps give the same actions (4223 stops, 2285 clears, 4173 clear requests, 59447 refusals). |
| `test_imu_gate_board` | Every `test_imu_gate.c` vector passes with the gate fed a board copy. |
| `test_imu_service` | The service board equals `imu_hump_pack()` of the same observation; a torn read returns E_TMOUT. |

Red first: with stub pack functions, the 4 new tests fail (abort);
698 others pass (`build/wip/imu-a/red-test.log`, not committed). After the change: 702/702 GCC and 702/702 Clang
ASan/UBSan (`halt_on_error=1`), 0 warnings.

## Build and RAM

Demo build from `git archive` with the public example secrets, 0 warnings,
RAM_PROOF PASS 79/79 (21212 B), before and after.
The `IMU_SAFETY_BENCH=robot_synthetic` and `IMU_SAFETY_CAPTURE=1` images
also build with 0 warnings and RAM_PROOF 77/77.

| Item (`scripts/ram_report.py`) | Before | After |
| --- | ---: | ---: |
| Static RAM | 239304 B | 239124 B (−180) |
| subsystem4 | 2780 B | 2741 B (−39) |
| vehicle | 2512 B | 2372 B (−140) |
| `.text` | 468260 B | 469268 B (+1008) |

The subsystem4 change includes the peak cache (+84 B: 48 B peak copy, 36 B
packed peak words). Vehicle saves `hump_copy` 32 B and three diag copies of
36 B each.

## Timing in the 10 ms IMU task

DERIVED(emulator): the publish code, built with the firmware flags and run
in an instruction emulator with the Cortex-M0+ cycle table at 125 MHz and a
warm flash cache. It was not measured on the robot.

| Publish path | Before | After |
| --- | ---: | ---: |
| hump (pack + board) | 5.61 us | 12.61 us |
| diag, peak unchanged | 7.70 us | 14.51 us |
| diag, peak changed | 7.70 us | 55.46 us |
| safety (pack + board) | 4.30 us | 2.23 us |
| **Each sample (peak unchanged)** | **17.61 us** | **29.35 us (+11.7)** |
| Worst sample (peak changed) | 17.61 us | 70.30 us (+52.7) |

The peak changes only while a hump event rises. Compiler frames
(prologue): `sample()` 1016 → 1048 B; the deepest new call below it is
`imu_hump_diag_pack_run` 88 B + `imu_hump_diag_scaled` 16 B. The measured
IMU stack high-water is 1520–1568 of 2048 B.

## Open gates

These are the original isolated-change gates. Current: the IMU is integrated
in the robot, with hump reporting and lift/overturn safety. Combined CR1
image 12 checked hump, lift stop, timing and stacks (2026-11-19); see
[project status](PROJECT_STATUS.md#areas). Image 30 run 6 later reported a
hump on a full course. Open: isolated before/after compute cost and the
remaining overturn/recovery cases. INT1 wake is parked at tag
`archive/mtk3-imu-irq`; FIFO partial-read option D is integrated.

- Robot: compute/work max of the IMU task before/after
  (`IMU_SAFETY_BENCH=robot_synthetic` images are built; Main flashes).
- Robot: hump run (summary peak/count/flags/gated and live HUMP events) and
  IMU lift and overturn stop with GP21 clear.
- Robot: IMU stack high-water with this image.
