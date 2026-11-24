# Blip grace

## Status

- Approved by the operator on 2026-11-19 (proposal `/tmp/SW11_BLIP_GRACE_PROPOSAL.md`).
- Integrated in the demo; development ref: tag `archive/mtk3-blip-grace`, base `871c806`.
- Code commits `d809e64` and `307a213`.
- The change is live builds only (`BARCODE_LIVE_ENABLE=1`). The shared producer and the SHADOW builds do not change behaviour; they get only the report counter.
- Robot: included in the later LIVE images. Image 30 run 6 read 4/4 barcodes;
  see [DEMO_KERNEL_V2.md](../release/DEMO_KERNEL_V2.md). This is combined-image
  evidence, not a separate on-robot F1/F2 isolation test.

A blip is a LINE → SEARCH → LINE change that is shorter than `BLG_SEARCH_MS` (200 ms). The image-12 log has 41 blips in the KR1 rows (20–100 ms, 20 ms rows) and 45 in the VE events (9–100 ms), in 3 boots. The 4 extra VE blips fall between two 20 ms rows.

## Scope

| Item | Change |
| --- | --- |
| F1: SEEK leading white before the first bar | Kept through a blip. |
| F2: a decoded LIVE record that waits for the clear guard | Kept through a clean blip. |
| Report counter | New: SEEK white restarts per run, by cause. |
| Line steer, detour, run stats, IMU, obstacle watch, sonar | Not in scope. Unchanged. |

## Rule

The two fixes use the existing grace without changes. The grace has the same begin rule and the same 200 ms limit. It ends on any of these conditions: busy, an active action, a stale or invalid sample, not calibrated, a junction, black, a hard route (STOP, SCAN, DETOUR) or 200 ms ([live.c](../../app_program/barcode_live_vehicle/live.c)).

### F1: SEEK white

| Step | Rule | Source |
| --- | --- | --- |
| Hold | While the grace is active and the window is in SEEK, the live control cache keeps the window and freezes it. | [control.c](../../app_program/barcode_live_vehicle/control.c) `seek_hold` |
| Freeze | The window ignores ADC samples from the freeze time. The white run, the white mean and the white count do not change. No pivot sample enters them. | [barcode_window.c](../../app_program/subsystem3/barcode_window.c) `barcode_window_freeze`, `process` |
| Ink in the blip | A frozen sample at or above `rise` ends the white run. A bar that starts inside the blip is never bridged. | `process` (`BW_WHITE_LOST_BLIP`) |
| Thaw | At the end of the grace, the frozen travel is removed from the white run (`white_start` moves forward by it). White from before the blip still counts. | `process` |
| After the thaw | If the first sample after the thaw is not white, the white run ends. | `process` |
| Motion | The pose and the axis-retreat check (2.5 mm) run as before. A pivot that moves the sensor back is a BW_MOTION loss, as today. | `barcode_window_motor` (unchanged) |
| Safety | No read or offer while the control mode is DISABLED: a completed result stays in quarantine, as for the bound grace. | `control.c` (unchanged) |

### F2: held LIVE record

| Step | Rule | Source |
| --- | --- | --- |
| Begin | The grace can begin while a record waits for the clear guard (`waiting`). | `live.c` `jlive_route` `begin` |
| Hold | During the blip, `jlive_read` returns false and the record stays held. The mode stays DISABLED. | `live.c` `jlive_read` |
| Resume | On a clean return to LINE, the record waits again, and the 100 mm guard runs as before. | `live.c` |
| Exit | Any grace exit retires the record with `JLIVE_ROUTE`, as before. This includes a junction or black sample during the blip. | `live.c` `cut` |

## Report counter

`[BC_SEEK_RESTARTS] epoch E disabled D black B other O` is printed in the stopped (GP20) dump, after `[BC_REASON_COUNTS]`. It is the count for the last nonzero run epoch and survives STOP, like the reason counts. It is a report only; no decision reads it.

| Field | Meaning |
| --- | --- |
| disabled | A white run ended by a route or disable (including a blip with no grace, or ink inside the blip). |
| black | Unadmitted ink (mat line or a short dark mark) ended a white run. |
| other | A loss, motion, a cancel for another reason, or a grey transition longer than 7 mm ended it. |

## Tests

| Test | Proves |
| --- | --- |
| `test_barcode_producer_blip_blip20/40/180` | A 20, 40 or 180 ms blip that ends at 500 ms keeps the white: 1 record. Only 7 mm of white follow the blip before the bar at 548 ms, so the read needs the kept run (valid for `QUIET_MIN_MM` 40 and 30). `leading_mm` = 80 mm minus the frozen travel (±3 mm). |
| `test_barcode_producer_blip_blip220` | A 220 ms blip ends the grace: 0 records, 1 disabled restart. |
| `test_barcode_producer_blip_blip40pivot` | A pivot that moves the sensor back gives BW_MOTION: 0 records. |
| `test_barcode_producer_blip_blip40grey` | Non-white samples inside the blip do not enter the white: 1 record. |
| `test_barcode_producer_blip_blip40bar` | A bar that starts inside the blip ends the run: 0 records. |
| `test_barcode_producer_blip_shared_*` | The shared producer (no grace) never keeps the white. |
| `test_jlive_grace` (F2 part) | A held F record survives 20/40/180 ms with a pivot and is then offered; 220 ms, a junction, black, a stale sample, busy and SCAN retire it. |

Red first: with the old `control.c` and `live.c`, 6 of 720 tests fail (`build/wip/blip/red-test.log`).

No-blip identity: all barcode, junction, jlive, blv, line service and capture tests (307 tests, including the real capture fixtures in `tests/fixtures/`) print the same output as base `871c806`. The only differences are the new `[BC_SEEK_RESTARTS]` line and two lines that the base archive cannot print (Make probes do not run from an archive). The check found one bug: a bar inside the blip was bridged. Commit `307a213` fixes it.

## Evidence

Date 2026-11-19.

| Check | Result |
| --- | --- |
| Host tests | 720/720 GCC, 720/720 Clang ASan/UBSan (`halt_on_error=1`), 0 warnings |
| Demo build (`git archive`, public example secrets) | 0 warnings, RAM_PROOF PASS 81/81 (21370 B in RAM); before: 79/79 (21212 B) |
| Static RAM (`scripts/ram_report.py`) | 234596 → 234804 B (+208: subsystem3 +136, other +88, alignment −16) |
| `.text` | 470092 → 470244 B (+152) |
| MQTT and board sizes (`scripts/message_sizes.py`) | Unchanged |
| Minimum-bit proof | Unaffected: no packed record changed |

The new RAM rows are `barcode_window_freeze` and `barcode_window_thaw` (`tools/kernel_ram/barcode_live_control.list`). They run in the per-sample line task path.

## Limits

- The 41 blips in the image-12 log were replayed from KR1 rows (20 ms). The replay is not a firmware run.
- The counter can show whether SEEK restarts came from blips. It cannot show whether F1 was the cause of 0 attempts in boot `10d9de00`.
- A blip at the junction: F2 retires the record only when a junction or black sample is seen during the grace. If the sensor misses the junction during the blip, the record can still meet the next junction.
