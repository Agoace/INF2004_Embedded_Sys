# Submission v2 2026-11-19

Superseded by tag `submission-v4`; its firmware is the same as
[demo-kernel-v2](DEMO_KERNEL_V2.md). The build, limits and results below
apply to submission v2, not the current demo.

This record identifies the second submission. `main` receives
`mtk3-staging` `1f46e88` with one `--no-ff` merge and gets the tag
`submission-v2`. The first submission (`submission-v1`) and the frozen
demo kernel (`demo-kernel-v1`) stay unchanged; see
[DEMO_FREEZE.md](DEMO_FREEZE.md).

## Submission point

| Item | Value |
| --- | --- |
| Tag | `submission-v2` on `main` |
| Staging commit | `1f46e88` (`mtk3-staging`) |
| Previous submission | `submission-v1` (`a791f4c`) |
| Date | 2026-11-19 |

## Changes since submission v1

Each change entered staging as one `--no-ff` merge. Use
`git revert -m 1 <merge>` to remove one change.

| Change | Staging merge | Record |
| --- | --- | --- |
| Detour: lost sonar result fix | `951b6a0` | [OBSTACLE.md](../s5-obstacle/OBSTACLE.md) |
| CR1 minimum-bit round 1 (packing, fault mask F, `PM_RUN_STATE_MS` 110) | `5a022a6` | [CR1.md](../architecture/CR1.md) |
| CR1 minimum-bit proof record and test | `56b8403` | [CR1_MIN_BIT_PROOF.md](../architecture/CR1_MIN_BIT_PROOF.md) |
| CR1 IMU boards (hump, diag, safety) | `666c82d` | [CR1.md](../architecture/CR1.md) |
| CR1 fault mask storage | `40ab815` | [CR1.md](../architecture/CR1.md) |
| CR1 KR1 RAM rows 16 to 15 B | `aab3cbb` | [CR1.md](../architecture/CR1.md) |
| Barcode blip grace | `d06ee35` | [BLIP_GRACE.md](../s3-line-barcode/BLIP_GRACE.md) |
| IMU FIFO partial read (option D, 6 per catch-up) | `18d9e41` | [IMU.md](../s4-imu/IMU.md) |
| Barcode leading white 40 to 30 mm | `1f96db1` | [LIVE_WINDOW_READER.md](../s3-line-barcode/LIVE_WINDOW_READER.md#leading-white-30-mm-operator-decision-2026-11-19) |
| Detour minimum turn and S2 free-ray clear heading | `e594b26` | [OBSTACLE.md](../s5-obstacle/OBSTACLE.md) |

## Build

The live barcode build line is the same as in
[DEMO_FREEZE.md](DEMO_FREEZE.md#build). Put the private Wi-Fi and MQTT
headers in place before the build and remove them after the build.

## Checks at submission v2

| Check | Result |
| --- | --- |
| Host tests | 722/722 |
| Live build (`F=LEFT`, example secrets): RAM_PROOF | PASS 81/81, 21370 B in RAM |
| Live build: ELF text / bss | 471348 / 212300 B |
| Live build warnings (gcc and gas) | 0 |
| `check-docs.py` | OK |
| `check-template-changes.py` | PASS, 8 recorded deviations |

## Robot evidence 2026-11-19

The robot image of `1f46e88` with the hotspot settings has the same UF2
SHA-256 as robot image 19 (prefix `4d3fe5a8`). Image 19 contains all the
changes in the table above. Image 12 (`73395bb`) contains the CR1 round.

| Run | Image | Result |
| --- | --- | --- |
| CR1 round | 12 (`73395bb`) | KR1 rows decode, hump report, lift stop, fault run mask holds the IMU bit after an IMU stop, 0 overruns. |
| Leading white 30 mm | 18, 19 | Decoded a barcode that the 40 mm image (16) missed. |
| Detour minimum turn | Test image from the work now at tag `archive/mtk3-detour-minturn` | Normal path seen on the robot (2 DONE_LINE). The S2 path is host only. |
| GP28 at 2.5 mm, large sheet | `1f46e88` | Decoded Q; narrow 6.51 mm, wide 17.38 mm. |

## Known limits

- All limits in [DEMO_FREEZE.md](DEMO_FREEZE.md#known-limits) stay, but the
  leading white before a barcode is now 30 mm (was 40 mm).
- Leading white 30 mm: a false start is possible above 200 mm/s (FC2
  replay). A faint mark about 5 mm before the first bar can start the read
  early and give BW_START.
- The small barcode sheet (narrow bar 2.9 mm) does not decode with this
  reader at a GP28 height of 5.5 mm or 2.5 mm. Use the large sheet.
- After an MQTT ESTOP stop, the run fault mask is 0.
- The S2 detour path and the lost sonar result fix are host tested only.
