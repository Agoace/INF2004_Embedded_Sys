# Barcode session report tool, D_cross and the dark mark

Status: host tool and analysis of existing logs. No firmware change.

## Tool

```sh
python3 scripts/barcode_session_report.py LOG [--json] [--dark 0.5]
python3 tests/test_barcode_session_report.py
```

`scripts/barcode_session_report.py` reads one USB log. The log can hold
several runs and repeated GP20 dumps. The tool groups dumps by run, using the
KR1 rows/events and the frozen timeline `end_us`. For each run it uses ONE
dump: the smallest dump delta, and then the latest one. It never unions dumps.

Per attempt it prints these values:

- seq and letter
- the result (BW/BP reason)
- the start and end in run milliseconds
- the mean KR wheel speed
- the LINE->SEARCH loss that ends the attempt (VE outcome within 3 ms)
- a DERIVED D_cross

The run start is the timeline `end_us` minus the time of the VE STOP event
(kind 4).

| D_cross form | Rule | Label |
| --- | --- | --- |
| From the stop report | `((edgeL-endL)+(edgeR-endR))/2 x 1000/12414` mm. `end` is `[BC_EVIDENCE] frame_end`; `edge` is `[BC_CONTROL] edge`. | MEASURED, but only when the edge is the NEXT junction |
| From KR rows | Wheel-speed integral from the frame end (`BC_REF bracket_us`) to the first later KR row where both line channels are dark (≥ 50 % of the printed range) | DERIVED; 20 ms rows; not the firmware detector |

The firmware keeps only the LAST record and the LAST confirmed junction. So
the MEASURED form exists for at most one attempt per run. The tool prints
"NOT the next junction" when KR shows a dark pair between the frame end and
the reported edge. The tool also reports the following problems:

- attempts lost from the dump, by comparison with `[BC_REASON_COUNTS]`
- captures without a `BC_REF`
- a `BC_REF` whose `BC_FRAME` line was lost or garbled; its result is `?`

A `BC_REF` with seq > 0 is shown as BW_OK.

The synthetic tests cover these cases:

- the formula
- the "next junction" check
- an edge before the record
- a stop report with no record
- the choice between a lossy dump and a delta-0 dump of the same run
- two different runs
- a garbled FRAME line
- a STOP event that is not the last VE row
- attempts missing from a dump
- the dark-mark width from capture edges

## Results on existing logs

| Run | Attempts in dump | Barcode 2 reads (seq, run ms) | D_cross DERIVED | Stop-report D_cross |
| --- | --- | --- | --- | --- |
| FX_1 (start 44909) | 3 of 4: dump 2 garbled one BC_REF; dump 1 delta 772 | none | — | no record (seq 0) |
| FX2 (start 41669) | 7 of 7 (attempt 3 result `?`, FRAME line lost; MQTT says BP_DISABLED) | 1 at 50100, 2 at 80041 | 253, 254 mm | 1021.8 mm, NOT the next junction |
| B120 (start 48929) | 8 of 8 | 1 at 25391, 2 at 61751 | 251, 254 mm | 2823.9 mm, NOT the next junction |

D_cross from the barcode 2 frame end to the next dark pair is 251–254 mm. This
was seen in four reads at 118–145 mm/s (DERIVED). The existing stop reports
cannot give a MEASURED value, because each run continued past several more
junctions. The [clearance procedure](BARCODE_LIVE_COMMAND.md#clearance-measurement-next-robot-session)
still needs the robot stopped just after the first junction past barcode 2.

Run start, end events and attempt times match the earlier manual analysis.

KR dark-pair clusters: B120 has 9, which equals `[JUNCTION_COUNTS] confirmed
9`. FX2 has 16 against 12 confirmed. B120 shows four dark-pair events per lap,
at lap offsets of about 9.0, 15.9, 22.9 and 29.6 s after the barcode 1 first
bar. Each event lasts 80–120 ms. Barcode 2 sits between the 15.9 s and
22.9 s events. Its first bar is about 320 mm after the earlier event, and its
frame end is about 250 mm before the later one. This agrees with the
OPERATOR statement that barcode 2 is between two crossing passes.

## Dark mark (BW_WIDTH every lap)

| Run | First black (run ms) | After barcode 1 first bar | Wheel travel to next dark pair | Black length | Peak raw | Lead white |
| --- | --- | --- | --- | --- | --- | --- |
| FX_1 | 50584 | 876 mm | 132 mm | no capture | — | — |
| FX2 | 9775 | 857 mm | 151 mm | 54.8 mm wheel / 32.6 mm reader | 1277 | 155 mm |
| B120 | 12371 | 859 mm | 142 mm | 47.2 mm wheel | 1270 | 151 mm |
| B120 | 48643 | 865 mm | 141 mm | no capture | — | — |
| B120 | 85430 | 861 mm | 143 mm | no capture | — | — |

Definitions:

- "Wheel" length is the ADC black duration (raw ≥ rise, then > fall)
  multiplied by the KR wheel speed (DERIVED).
- "Reader" length is the projected GP28 edge distance from the capture edges.
- The two lengths differ because the reader projects GP28 motion, and GP28
  sits 73 mm left of the centre line while the robot zigzags.

Findings:

- **Position:** the mark is the same place every lap. It is 857–876 mm after
  the barcode 1 first bar and 132–151 mm of wheel travel before the next
  dark-pair junction event. The line pair is about 122 mm ahead of the axle,
  and GP28 is 73 mm ahead and 73 mm left. So the mark is about 190 mm before
  the junction feature, measured along the path (DERIVED, assumes a straight
  path).
- **Appearance:**
  - It is one black element, 47–55 mm long along the travel, after at least
    150 mm of clean white.
  - It reads darker than the barcode bars: peak raw about 1270–1277, with
    the thresholds 349/274.
  - The reader rejects it correctly. A single 33–55 mm black element is
    wider than any Code 39 element, so the result is BW_WIDTH.
- **Hypothesis (INFERENCE, not measured):** the mark is a black line that
  joins the course line from the left at a shallow angle. A typical tape
  width of about 18 mm crossed at about 20–25 degrees gives a 40–55 mm
  length along the travel. With GP28 at 73–95 mm left, the meeting point is
  about 190 mm ahead, which matches the junction event that follows.

What the operator should check on the mat. Start about 0.86 m after the
barcode 1 first bar and stop about 0.2 m before the next junction. Look on
the left side, 70–100 mm from the line:

1. A branch or merging line that joins at a shallow angle. If one exists,
   the mark is course geometry and no change is needed. Record the angle.
2. Otherwise, a tape patch, a mat seam, a dark stain, or the box footprint.
   Measure its length along the line, its width, and its distance from the
   line. Add the values to MEASUREMENTS.md.

The mark costs one BW_WIDTH failure per lap. It does not cancel any barcode
read, and it does not reach the line pair.
