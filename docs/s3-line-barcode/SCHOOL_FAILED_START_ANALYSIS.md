# School failed starts: FC1 and FC2

## Scope and evidence

The image was staging `024d01a`. No powered result for the changes in this
record is claimed. Main owns integration and the operator owns flashing.

- FC1: `robot_FC_1.final.log`, epoch 1, attempts 1 and 2. One lossy dump.
- FC2: `robot_FC_1d2.final.log`, epoch 3, attempt 5. Two dumps of this
  **different run**. Do not combine FC1 and FC2 to fill missing rows.
- Original logs remain in the operator workspace under
  `build/school_stg_024d01a/`. The two small CSV fixtures in
  [`tests/fixtures`](../../tests/fixtures) retain the three run vectors and
  all 204 FC2 ADC rows. No missing FC1 row was invented.
- **OPERATOR, no measured number:** before the first printed bar there is
  only white paper, then white mat. The physical white approach is long,
  well over 40 mm. This is not a measurement of reader-qualified white.

The host validator is [`scripts/barcode_fail_analyze.py`](../../scripts/barcode_fail_analyze.py).
Its tests are [`test_barcode_fail_analyze.py`](../../tests/test_barcode_fail_analyze.py).
The repeat-union flag is an operator assertion that the buffers are from the
same run. All overlapping rows and frozen headers must agree. Different
run epochs cause an error, even with this flag.

```sh
python3 scripts/barcode_fail_analyze.py /path/to/robot_FC_1.final.log \
  --json build/FC1.json --csv-dir build/FC1
python3 scripts/barcode_fail_analyze.py /path/to/robot_FC_1d2.final.log \
  --merge-repeats --json build/FC2.json --csv-dir build/FC2
```

Exit 0 means all declared capture rows arrived and the dump boundaries were
seen. Exit 2 means missing, conflicting, or invalid evidence. This does not
validate KR row delivery, prove full-rate ADC capture, or undo storage overflow.
The JSON retains malformed-line reports and storage-truncation flags. Do not
connect missing CSV row numbers with an inferred waveform.

## Delivery audit

| Run / dump | Valid edge rows | Valid ADC rows | Result |
| --- | ---: | ---: | --- |
| FC1 attempt 1 | 10/10 | 189/221 | Incomplete |
| FC1 attempt 2 | 10/10 | 173/209 | Incomplete |
| FC2 dump A | 10/10 | 193/204 | Missing rows 88..98 |
| FC2 dump B | 10/10 | 165/204 | Missing 7..30 and 133..147 |
| FC2 A + B | 10/10 | 204/204 | 154 overlapping ADC rows agree; no conflicts |

FC1 has 362 valid ADC rows out of 430. Three tagged lines are malformed;
partly readable text is not accepted as a complete row. Attempt 1 misses
23..48 and 207..212; attempt 2 misses 73..76 and 133..164.
A final `dump delta 0` does not prove the barcode prefix arrived: that
counter interval starts after this prefix. FC2 B also has missing ADC rows
although its final delta is zero.

## Width diagnosis

Widths below are **DERIVED** differences of measured edge coordinates,
printed to 0.01 mm. Fit values are a diagnostic reconstruction from those
rounded widths, not bit-exact replay of the original floating-point state.
Uppercase marks a wide bar; lowercase marks a wide space.

| Run / attempt | Nine-run pattern | Narrow mm | Wide mm | Bias mm | Wide/narrow | Recorded lead mm |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| FC1 / 1 | `WnWnNnNnW` | 5.685 | 17.778 | 3.015 | 3.127 | 46.44 |
| FC1 / 2 | `NwNnWnWnN` | 5.895 | 17.607 | 3.372 | 2.987 | 42.63 |
| FC2 / 5 | `NwNnWnWnN` | 5.738 | 18.215 | 3.515 | 3.174 | 43.07 |

- FC1/1 widths: 15.97, 8.63, 13.49, 9.54, 3.22, 9.48, 2.12, 7.15, 14.83 mm.
  Mask `0x141` is not a star. It must remain rejected.
- FC1/2: 3.11, 20.74, 2.02, 9.20, 14.57, 9.08, 14.14, 9.52, 2.44 mm.
- FC2/5: 2.87, 21.66, 1.83, 9.63, 14.37, 8.33, 15.10, 9.80, 1.97 mm.
- Both latter vectors have star mask `0x094`; all size, ratio, bias,
  separation, and spread checks pass. The old post-fit leading guard alone
  rejects them: 42.63 < 58.95 mm and 43.07 < 57.383 mm.
- Rise/fall are 329/254, 326/251, and 324/249: all have 75-count hysteresis.
  No evidence here requires a contrast, ratio, width, or bias change.

**The recorded leading distance is not continuous white.** FC2 has two
observed black pulses before candidate time 111161457 us:

| First above-rise sample us | First below-fall sample us | Peak raw |
| ---: | ---: | ---: |
| 111004455 | 111024455 | 539 |
| 111083455 | 111104455 | 991 |

These are sampled bounds, not exact crossing timestamps. FC1/2 has the same
two-pulse shape before its star. The SEEK code can retain `white_start`
across an unadmitted black run shorter than 7 mm while it waits for 40 mm.
Therefore 42.63/43.07 mm, or 7.23/7.51 fitted narrow widths, cannot establish
a safe new physical quiet-zone minimum.

A start-star suffix followed by the F prefix explains FC1/1. A closing star
preceded by the narrow F-tail bars explains FC1/2 and FC2/5. This is a strong
**DERIVED interpretation**, not a captured full `*F*` decode. The actual F
table mask is `0x058`; `0x064` is digit 9. No test or firmware may select a
payload to force this interpretation.

## Source readiness and motion

**The old headers do not record source-enable or last unbound-reset time.**
A capture start is first-bar admission time, not source-enable time. ADC
`pre=1` means retained before binding; it can include the first-bar sample.
A small first-edge coordinate does not date source enable: motion loss can
reset the projection origin while the source remains enabled.

- FC1 RUN starts at 18569 ms. First candidate is 21998.424 ms. The retained
  20 ms KR rows show LINE throughout the approach and no confirmed initial
  junction/module change. Sparse KR/MQTT observations do not prove that
  every control context remained enabled between samples.
- FC2 RUN starts at 110449 ms. Candidate is 111161.457 ms, only 712.457 ms
  later; its first edge is at projected 93.38 mm. If this is the closing
  star, the fitted `*F*` start-to-closing-star distance is about 189.63 mm.
  Thus the initial star likely passed before RUN/source start. This is an
  inference, not a measured source-enable timestamp. FC2 live MQTT states
  start at 217022 ms, after the traversal, so they cannot resolve this.
- FC1 has commanded left-zero corrections at 20629, 20749, 20769, 20789,
  21569, and 21949 ms in the two seconds before attempt 1. At 20809 ms,
  measured L/R are 16/249 mm/s: aligned-axis GP28 speed is about -29.2 mm/s
  with 77 mm lateral offset, or -20.7 mm/s with 73 mm. This is a real risk
  for the old non-increasing-projection reset, 1189 ms before admission.
- No measured L/R ratio below 0.16 occurs in the two seconds before FC1/2.
  None occurs in FC2's 712 ms available RUN approach. Commands are not
  measured wheel motion. A 20 ms record can miss a shorter 10 ms event.
  The aligned-axis ratio is not a complete test when the fixed scan axis
  and current heading differ. No exact unbound-reset correlation is proven.

The operator must start well before the barcode. Even physical white longer
than 40 mm cannot help if its observed context was disabled, was reset, or
started after part of the printed code passed. Do not claim that a reader
patch can recover ink that the source did not observe.

## Approved changes and host plan

Main obtained operator approval to test these together, with separate
revertible commits. No threshold, ratio, bias, requested-letter, steering,
or calibration change is included.

1. Clear white qualification on any above-rise black that is not admitted.
   Keep transition-band handling. Use the existing 40 mm minimum both
   before and after fitting, rather than 40 mm before and `10*n` after.
2. Permit at most 2.5 mm cumulative projection drawdown from the furthest
   point. Do not add each backward step as a new allowance. Ignore ADC
   positions already traversed until progress resumes. Keep reverse,
   stationary-wheel stall, motor-gap, and fault loss checks.
   At base 146 mm/s, worst nominal left pivot is L=0/R=292. Exact 30 ms
   projection loss is about 1.94 mm with 80/77 mm geometry, or 1.60 mm with
   73/73 mm. The 2.5 mm bound adds about 0.56/0.90 mm margin. It is an
   **ASSUMED safety bound**, not a measured twitch amplitude.
3. Set the default forward/left offsets to the existing operator ruler
   values, 73/73 mm. Keep the geometry integration and fit/bias code from
   `52244f2` unchanged.
4. If cheap, add default-off capture context times for enable and the last
   unbound reset, so the next powered run can answer the timing question.

RED-first tests cover the three measured vectors, the two valid star stages
without inventing a full letter, clipped quiet, invalid pattern, and clock
wrap. FC2's complete 204-row ADC fixture is replayed with explicitly ASSUMED
linear interpolation and 100/146/220 mm/s motor brackets anchored to the
header. Old code admits a fragment; corrected code does not. This is not a
reconstruction of original motor timing. Motion tests must include a twitch
inside a star and near an edge, sustained retreat, reverse, stall, and
faults. Recheck all five older real fixtures and quiet input.

The historical 792 quiet bins had no local prominence reaching 20; quiet
SD was about 1.3..2.4 counts. This supports background separation, not a
field false-start rate or immunity to real mat/ink fragments. See
[the original analysis](../../testing/subsystem3/TEST_LOG.md#approved-real-fixtures-and-analysis).
Lowering the pre-admission minimum to 4n or 7n is not justified by these
captures. The approved 40 mm policy avoids that extra early-admission risk.

## Validation

Analysis tests: seven unit tests pass. The pre-change native baseline
confirms fragment admission at all three assumed speeds. The new native
quiet test fails on the old post-fit guard before reader changes.
Quiet change A: the new native test is RED on `024d01a`, then GREEN after
black-reset and the single 40 mm policy. GCC passes 572/572 tests. The five
older fixtures retain their expected results. An additional quiet check
feeds 829 actual DISTANCE bins from PEAK-framed quiet crops into the live
reader, in both orientations, with no candidate. These crops use the existing
120 ms frame-centre guard; this count is not the historical 792-bin analysis.
Motion change B: [`test_barcode_motion_twitch.c`](../../tests/test_barcode_motion_twitch.c)
simulates differential-drive motion with the configured GP28 offsets. RED on
change A: a 30 ms L0/R292 twitch inside the first star, the same twitch on
the first bar's trailing edge, and the six FC1 KR measured wheel-speed rows
(ASSUMED mm/s, 20 ms each) all end in BW_MOTION. GREEN after B: all three
read F. A 120 ms twitch (retreat above 2.5 mm), both-wheel reverse, and a
stationary motor step still end in BW_MOTION. GCC passes 580/580 tests.
The recovery motor interval maps its ADC rows from the furthest point; edge
error there is at most the held retreat (ASSUMED linear interpolation).
Firmware matrix and sanitizer validation will be recorded with the remaining
changes; no flash or push is authorized.
