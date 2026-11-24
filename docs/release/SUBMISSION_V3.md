# Submission v3 2026-11-20

Superseded by tag `submission-v4.1`. Tag `submission-v4` uses the same
firmware as [demo-kernel-v2](DEMO_KERNEL_V2.md).
Build, limits and results below: submission v3. The demo-day letter map is
recorded in the current demo kernel record.

This record identifies the third submission. `main` receives
`mtk3-staging` with one `--no-ff` merge and gets the tag `submission-v3`.
The earlier tags (`submission-v1`, `submission-v2`, `demo-kernel-v1`) stay
unchanged; see [SUBMISSION_V2.md](SUBMISSION_V2.md) and
[DEMO_FREEZE.md](DEMO_FREEZE.md).

## Submission point

| Item | Value |
| --- | --- |
| Tag | `submission-v3` on `main` |
| Code | `2123e1f` (tag `archive/mtk3-bc-endstar`), merged into staging as `083cde6` |
| Previous submission | `submission-v2` (`e0eff53`) |
| Date | 2026-11-20 |

## Changes since submission v2

One `--no-ff` merge (`083cde6`) brings both barcode branches. Use
`git revert -m 1 083cde6` to remove them.

| Change | Archive tag (former branch) | Record |
| --- | --- | --- |
| Local-midpoint edges for the small sheet at GP28 2.5 mm, faint first mark drop, `N_MIN` 2.5 mm | `archive/mtk3-bc-small` | [Local-midpoint mode](../s3-line-barcode/LIVE_WINDOW_READER.md#local-midpoint-mode-at-gp28-25-mm) |
| End-star split fallback, axis re-anchor in SEEK, wide/narrow limit 3.55, late-start rescue | `archive/mtk3-bc-endstar` | [Bend exit and held letter](../s3-line-barcode/LIVE_WINDOW_READER.md#bend-exit-and-held-letter-2026-11-20) |
| Held letter: keep the letter once parts 1..19 pass (operator 2026-11-20), and publish it when a motor or ADC sample ends the window | `archive/mtk3-bc-endstar` | [Bend exit and held letter](../s3-line-barcode/LIVE_WINDOW_READER.md#bend-exit-and-held-letter-2026-11-20) |

## Demo build: set the letter map on the day

The letter commands for the demo are not known yet. Only `BARCODE_MAP`
changes; the code does not change. Set the map, build, and flash:

```sh
make -C build_make clean
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
  KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="A=LEFT,F=UTURN" \
  JUNCTION_TEST_CMD=NONE
```

| Item | Rule |
| --- | --- |
| Format | `LETTER=ACTION`, comma-separated, for example `"A=LEFT,B=RIGHT,C=STRAIGHT,D=UTURN"` |
| Actions | `LEFT`, `RIGHT`, `STRAIGHT`, `UTURN`, `NONE` |
| Unlisted letters | Read and reported (SHADOW), no action |
| Build check | The build prints `BARCODE_MAP: <map> -> policy 2`. The boot prints `[BARCODE_TABLE]`. |
| When the action runs | At the next junction after the barcode, not at the barcode |

Put the private Wi-Fi and MQTT headers in place before the build and remove
them after the build. See [BUILD.md](../kernel/BUILD.md).

## Checks at submission v3

| Check | Result |
| --- | --- |
| Host tests | 734/734 |
| Live build (`A=LEFT,F=UTURN`, example secrets): RAM_PROOF | PASS 84/84, 23906 B in RAM |
| Live build: ELF text / bss | 475204 / 212492 B |
| Fallback build (no live barcode): RAM_PROOF | PASS 80/80, 23018 B |
| Build warnings (gcc and gas) | 0 |
| `check-docs.py` | OK |
| `check-template-changes.py` | PASS, 8 recorded deviations |

## Robot evidence 2026-11-20

Hotspot settings, GP28 at 2.5 mm. Image 29 has the code of this submission
with `BARCODE_MAP="A=LEFT,F=UTURN"` (UF2 SHA-256 prefix `4e842c78`).

| Run | Image | Result |
| --- | --- | --- |
| Full lap, 90 s, 8.9 m | 29 (`2123e1f`) | A, large F and Z read: 3/3 OK, 0 failed. LEFT at the junction after A (93.7 deg). U-turn after F (177.5 deg). Detour DONE_LINE (obstacle 220 mm). |
| Full lap, 90 s, 8.8 m | 27 (`0831be0`, held letter without the publish fix) | A then LEFT (94.8 deg), Q, detour DONE_LINE, hump. One held read was lost; this gave the publish fix. |
| Bend exit, A sheet | 26 (`4e556c8`) | A 4/5, LEFT after A 3/3, Z 1/1, small F 0/1. |

## Known limits

- All limits in [SUBMISSION_V2.md](SUBMISSION_V2.md#known-limits) stay,
  except the small sheet: it now decodes on a calm straight (image 21,
  3/3), but fails on a hard zigzag.
- A held read has no end-star confirmation. A wrong letter gives a wrong
  turn. `*AB*` reads as A.
- On a bend exit or in a weave, GP28 can leave the bar ends before the
  letter is complete. The reader cannot fix this. Put barcodes on
  straights where possible.
- U-turn after a barcode: one robot pass (image 29).
- The bcfail USB dump can lose lines (up to about 600 on one dump).
