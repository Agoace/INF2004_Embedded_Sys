# Demo kernel v2 2026-11-20

This record identifies the kernel for the demo day. It is code freeze 2 of
2026-11-20 (operator). `main` receives `mtk3-staging` with one `--no-ff`
merge and gets the tags `submission-v3.1` and `demo-kernel-v2`. The
earlier tags (`demo-kernel-v1`, `submission-v1`..`v3`,
`code-freeze-20261120`) stay unchanged.

## Freeze point

| Item | Value |
| --- | --- |
| Tags | `demo-kernel-v2` and `submission-v3.1` on `main` |
| Code | staging merge `972f7cd` (`mtk3-demo-limits` `3f4706c`) on top of [submission v3](SUBMISSION_V3.md) |
| Robot image | 30, `3f4706c`, `BARCODE_MAP="A=LEFT,F=UTURN"`, UF2 SHA-256 prefix `d89bcd6c` |
| Date | 2026-11-20 |

No code changes after this point. Only the letter map changes on the demo
day.

## Changes since submission v3

| Change | Before | Now |
| --- | --- | --- |
| Run time limit (`RUN_LIMIT_MS`) | 90 s | 300 s |
| Full RAM run log (4501 rows, 90 s) | Stopped the run | The run continues. The GP20 dump keeps the first 90 s. MQTT continues. |
| Live barcode command hold (`BLV_PENDING_MAX_MM`) | 1000 mm | 10000 mm |
| Live command in an obstacle SCAN or DETOUR | Cancelled | Kept; the detour travel counts toward the 10 m |

Details: [BARCODE_LIVE_COMMAND.md (demo limits)](../s3-line-barcode/BARCODE_LIVE_COMMAND.md#demo-limits-operator-2026-11-20).
Remove all of them with `git revert -m 1 972f7cd`.

## Demo day: letter map (to be announced)

The letter actions are given on the demo day. Set them in `BARCODE_MAP`,
build, check, and flash. Nothing else changes.

```sh
make -C build_make clean
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
  KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="<LETTER=ACTION,...>" \
  JUNCTION_TEST_CMD=NONE
```

| Step | Check |
| --- | --- |
| 1. Put the private Wi-Fi and MQTT headers in place | See [BUILD.md](../kernel/BUILD.md). Remove them after the build. |
| 2. Set `BARCODE_MAP` | Actions: `LEFT`, `RIGHT`, `STRAIGHT`, `UTURN`. Example: `"A=LEFT,B=RIGHT,C=STRAIGHT,D=UTURN"`. Unlisted letters are read and shown, with no action. |
| 3. Build | The output shows `BARCODE_MAP: <map> -> policy 2`, `RAM_PROOF PASS` and 0 warnings. |
| 4. Flash | `picotool load -v -x <file>.uf2`, then the boot shows `[BARCODE_TABLE]`. |
| 5. Boot | Flat and hands off for at least 2 s (IMU level). |

The action runs at the next junction after the barcode.

## Checks at the freeze

| Check | Result |
| --- | --- |
| Host tests | 735/735 |
| Live build (`A=LEFT,F=UTURN`, example secrets): RAM_PROOF | PASS 84/84, 23906 B in RAM |
| Live build: ELF text / bss | 475180 / 212492 B |
| Fallback build (no live barcode): RAM_PROOF | PASS 80/80, 23018 B |
| Build warnings (gcc and gas) | 0 |
| `check-docs.py` | OK |
| `check-template-changes.py` | PASS, 8 recorded deviations |

## Robot evidence 2026-11-20 (image 30)

Hotspot settings, GP28 at 2.5 mm. Run 6 comes from MQTT only (the board
was powered off before the GP20 dump).

| Run | Result |
| --- | --- |
| Boot `082361e3`, run 6, 122 s, 11.5 m | No stop at 90 s. A then LEFT (93.5 deg); detour DONE_LINE (193 mm obstacle); large F then U-turn (178.5 deg); Z; detour DONE_LINE (214 mm); small F read (n 2.93, w 8.45); hump. Barcodes 4/4, detours 2/2. Stopped by the operator. |
| Boot `082361e3`, runs 1..5 | A 5/5, LEFT 5/5 (93.5..95.2 deg). Run 4 stopped by a left wheel stall (STALL_LEFT) at the start of a slow detour pivot (left asked 39 mm/s, measured 0 for 250 ms). |
| Boots `a255e347`, `af9b19fa` | A then LEFT (93.8 deg). One BW_START: the first star bar was faint (peak 362, measured 1.3 mm); the other 8 star parts were correct. |
| Boot `2f4eca77`, 4 runs | A out of the bend failed 4/4 (BW_WIDTH): GP28 crossed only the bar ends (3 bars, white, 2 bars; peaks 1300..1500). |

## Known limits

- All limits in [SUBMISSION_V3.md](SUBMISSION_V3.md#known-limits) stay.
- A kept command through a detour is host tested only. On the robot each
  command was used before the obstacle. After a detour the robot can
  rejoin the line at an angle; a kept command can then act at a false
  junction.
- A slow detour pivot can latch a wheel stall (STALL_LEFT seen once,
  STALL_RIGHT earlier). Use a charged battery.
- On a bend exit the first star bar can be faint or GP28 can cross only the
  bar ends. The reader cannot recover these cases.
- After 90 s the GP20 dump has no new rows. Use MQTT for the rest of a run.
