# Demo freeze 2026-11-18

Superseded by [demo-kernel-v2](DEMO_KERNEL_V2.md). Tag `submission-v4`
uses the same firmware. The current submission is tag `submission-v4.1`.
Build, limits and results below: freeze of 2026-11-18.

This record identifies the frozen demo kernel. Use it if a later change
fails. Do not change code on the freeze tag. Put new work on a module
branch and merge it into `mtk3-staging` first.

## Freeze point

| Item | Value |
| --- | --- |
| Tag | `demo-kernel-v1` |
| Commit | `9f15213` (`mtk3-staging`) |
| Submission branch | `main`, tag `submission-v1` (same tree as `9f15213` plus documentation) |
| Date | 2026-11-18 |

## Build

Live barcode demo image (barcode F turns LEFT):

```sh
make -C build_make clean
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
  KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
```

Use `BARCODE_MAP="F=RIGHT"` or `BARCODE_MAP="F=UTURN"` for the other
actions. Add more letters with commas, for example `"F=LEFT,K=RIGHT"`.
Fallback image without live barcode actions:

```sh
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1
```

Put the private Wi-Fi and MQTT headers in place before the build and
remove them after the build. See [BUILD.md](../kernel/BUILD.md).

## Checks at the freeze

| Check | Result |
| --- | --- |
| Host tests | 677/677 |
| Default build matrix (76 profiles) vs `56b7f03`: UF2 SHA and ELF PT_LOAD | MATCH |
| Same matrix with `BARCODE_MAP=F=LEFT` | MATCH |
| Live build: RAM_PROOF | PASS 79/79, 21004 B in RAM |
| Live build: text / bss / link-time free RAM | 454516 / 219052 / 29116 B |
| Live build warnings (gcc and gas) | 0 |
| `check-docs.py` | OK |

The robot images of 2026-11-18 contain local network settings. They stay
outside Git. Image SHA-256 prefixes: `9_LIVE_F_LEFT` `1fd43c66`,
`9_LIVE_F_RIGHT` `cdb2096f`, `9_LIVE_F_UTURN` `b4228ad2`.

## Robot evidence 2026-11-18 (school course, hotspot)

| Run | Image | Result |
| --- | --- | --- |
| LIVE1, LIVE2 | `60e1af7`, `6c40c33` | F decoded; DONE at 1.6, 39.0 and 2.5 deg, but the robot went straight. Cause: spin started about 96 mm before the arm centre, and any first line was accepted. |
| LIVE3 | `606ebbe` | 9 of 9 reads failed, 8 with BP_CONTROL: a failed coherent control read cancelled the attempt. Fixed by the live control-read cache. |
| LIVE4 | `3a6973e` | 4 real LEFT turns at 92.6, 93.2, 92.9 and 95.0 deg (operator saw them). 9 of 22 attempts BP_DISABLED, each at a 10-30 ms line loss. |
| UTURN1 | `56b7f03`, F=UTURN | U-turn: first contact 36.7 deg ignored, accepted 172.5 deg. |
| RIGHT1 | `9f15213`, F=RIGHT | Operator saw the RIGHT turn. No dump. |
| GRACE1 | `9f15213`, F=LEFT | 3 runs, 13 attempts, 0 BP_DISABLED. One decode passed through a 20 ms line loss. Barcode 1 decoded on each pass; barcode 2 (after a curve) failed. LEFT turns at 92.0 to 93.5 deg. |

LIVE1 to LIVE4 detail is in [BARCODE_LIVE_COMMAND.md](../s3-line-barcode/BARCODE_LIVE_COMMAND.md).
The values are in [MEASUREMENTS.md](../architecture/MEASUREMENTS.md).

## Known limits

- Put each barcode on a straight, at least 300 mm after a curve, about
  200 mm (minimum 160 mm) before the crossing. Put it left of the line
  under GP28. Keep at least 40 mm of white before it. A barcode that the
  robot enters at an angle often fails (BW_START, BW_WIDTH): the reader
  takes the robot heading at the first bar as the barcode axis.
- A pending command expires after 1000 mm of travel. Continuous line
  search for 200 ms or more also cancels it.
- U-turn uses the earlier spin logic. One robot pass only.
- Obstacle detour, IMU safety and hump reports are in this image. On this
  image only barcode reads and turns were run. Their last robot results
  are from earlier staging images. A full run with all features on this
  image was not done.
- On a weak hotspot, more than half of the MQTT publications can fail.
  The USB dump (GP20) keeps the last run.

## Branches after the freeze

See [PROJECT_STATUS.md](../architecture/PROJECT_STATUS.md#branch-layout-and-integration-rules).
