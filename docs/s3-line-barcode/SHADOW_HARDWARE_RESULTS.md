# B2b SHADOW hardware results

## What it is

This record gives the first hardware results for the B2b live barcode
reader in SHADOW mode. In SHADOW mode, the robot reads GP28 and logs the
decoded letter. It does not use the letter to turn.

## Status

| Item | Status |
| --- | --- |
| Integration | Merged into `mtk3-staging` from helper branch `mtk3-b2b` (`b405aef`) |
| Default image | SHADOW; letter table A=LEFT, B=RIGHT, C=STRAIGHT, D=UTURN, others NONE |
| Large-print decode on the course | FAIL (not decoded) |
| Live actions | Not enabled; they need an enable switch and a measured clear bound |
| Work state | FROZEN by the operator on 2026-11-11 for the dual-core migration |

## Results

### GP28 spin calibration (home mat, `a883455`)

| Run | GP28 range | Black | Mode | Rise/fall |
| --- | --- | --- | --- | --- |
| 1 | 152..1820 | 1521 | LEARNED | 303/249 |
| 2 | 155..184 | none | FALLBACK | 320/240 |
| 3 (clean) | 152..3178 | 2673 | LEARNED | 417/317 |

The mat line absorbs much more IR than printed ink (about 2700 against
600..900 for large narrow bars). Thus thresholds now use white plus a
fixed offset (`5c34854`): rise = white + 150, fall = white + 75, rise cap
350. The reader freezes the leading paper white at the first bar.

### Full circuit, SHADOW image `b405aef` (school, 2026-11-11)

One 90 s run, 9.9 m, LINE base 146 mm/s. One large "A" was on lap 2, on
the straight between the two loops, left of the line.

| Item | Result |
| --- | --- |
| LINE/SEARCH | Full run; 45 short line losses, all recovered |
| Junctions | 7 confirmed; all NONE; no action |
| Obstacle | SCAN, DETOUR complete, back to LINE |
| Humps | 12.7 mm and 13.4 mm |
| Timing | 0 motion late/missed, 0 vehicle overruns; KR1 gap 9931..10066 us |
| Stacks | All named stacks >= 496 B free; `line_task` 1384 B free |
| Heartbeat CPU | Up to 917 permille in the run (previous maximum 680) |
| Barcode | 3 attempts, 0 frames |

| Attempt | Time after START | Reason | Cause (best evidence) |
| --- | --- | --- | --- |
| 1 | 56.0 s | 6 WIDTH | Line or junction ink, not the A (opposite heading) |
| 2 | 69.6 s | 5 START | A, first star rejected |
| 3 | 70.8 s | 5 START | A, about 166 mm after attempt 2 |

## Findings (not fixed)

1. **Diagnostic defect.** `[BC_LIVE]` and `[BC_CPU]` reset when the epoch
   changes at STOP. Thus the stopped dump shows the time after STOP, not the
   run. Per-run statistics must latch before the reset.
2. **Disabled-path cost.** When the reader is disabled, it still reads the
   control board, unpacks the motion board and does software divisions for
   each sample. Measured cost is about 73 us per 1 ms sample (about 7 %
   CPU). This agrees with the idle CPU increase (about 45 % to 54 %). The
   disabled path must exit before this work.
3. **Probable cause of the A failure: yaw.** The reader measures distance at
   the mean of the two wheels. GP28 is far from the centre:

   | Offset | Value | Source |
   | --- | --- | --- |
   | Lateral | 77 mm left of the centre line | Operator ruler |
   | Forward | About 80 mm (+-5 mm) ahead of the wheel axle | Ruler photo |
   | Track | 111 mm | Robot record |

   The forward travel of GP28 is about
   `d_s = 1.194 * dL - 0.194 * dR`. Near attempt 2, the wheel counts were
   L 40 and R 217: the mean is 128.5, but GP28 moved about 6 counts. This
   cause is probable for attempt 2. It is not proven, because failed frames
   keep no element widths.

## Proposal on hold (not approved)

| ID | Change | RAM |
| --- | --- | --- |
| A | Early exit of the disabled path; latch per-run statistics at STOP; phase CPU counters | 0 |
| B | Failed-frame record: gate, 29 widths, frozen thresholds, L/R counts at each edge | 0 (reuse diagnostic pools) |
| C | Optional short raw GP28 snippet (about 0.5 s) | 0 (reuse diagnostic pools) |

After A and B: test the large print on a straight line, once with low yaw
and once with zigzag. If the low-yaw pass decodes and the zigzag pass does
not, use GP28-point distance in the reader.

## Evidence

Logs are local and not in Git (`build/mtk_logs/`):
`b2b_cal_home_20261110_usb.txt`, `b2b_shadow_F_20261111_usb.txt`
(the file name is historical; the card was an A), and
`robot-mqtt-20261111-163412.csv`.
