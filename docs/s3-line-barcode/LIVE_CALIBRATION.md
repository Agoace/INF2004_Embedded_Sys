# GP28 calibration evidence for the live barcode reader

## Status and scope

B2b step 2 adds observation only. No live window reader, letter table,
barcode command source or sensor-offset estimator is present yet.
The operator reported the first home-mat spin results on `a883455` below.
Live decoding and action acceptance are not claimed.

The approved course uses the large print and one uppercase letter.
The later live reader will use a 29-element `*X*` window and approximately
one narrow width of white after its last bar. This record does not change
[TEST barcode](BARCODE.md) framing, USB wait or its quiet-zone checks.
Small-print changes and the separate noise investigation remain deferred.

## Existing spin, unchanged controls

The existing pre-START [LINE spin](LINE.md#acquisition-and-calibration)
runs at 131 mm/s and 633 counts each side. Its pause, timeout, return-to-line
logic, buttons and GP20 DUMP are unchanged. GP28 is sampled from the same
queued samples already consumed by the line task. There is no second ADC
consumer and no new work in the acquisition interrupt.

- `barcode_calibration.{h,c}` holds bounded integer-only evidence.
- `line_service.c` starts, collects and ends that evidence with the existing
  LINE calibration calls. A failed pair calibration produces an ABORTED
  GP28 report; it does not change the pair's existing return result.
- `line_barcode_calibration_try_read()` reads a separate coherent report.
  It never waits or consumes a LINE event. Failed reads leave output unchanged.
- The report is published at init, begin and end/abort, not every sample.
  `line_snapshot_t`, telemetry wire sizes and routing are unchanged.
- Two `[BC_CAL]` lines are printed after mutex release by the calibration
  caller, not by the acquisition task or interrupt. Output is not retained
  for a terminal that connects later. The report remains in its board.

## Signal evidence and thresholds

All values below are 12-bit ADC counts unless a unit is shown.
They are initial engineering gates; shadow hardware must check them.

| Evidence | Rule |
| --- | --- |
| White candidate | 120 through 220; at least 16 samples |
| High candidate | At least 800, below clipping |
| Credible high duration | At least 10 ms of consecutive high samples |
| Time continuity | Positive source timestamp steps, at most 5 ms |
| ADC validity | Values 0 and 4095 or greater invalidate the result |
| Storage bound | At most 65535 samples; overflow invalidates the result |
| Fallback | Rise 320, fall 240, black is HIGH |

Normal source steps are 1 ms; the 5 ms bound is a calibration fault limit,
not permission for loss in a future barcode window. Timestamp wrap is handled
with unsigned subtraction. The sample cap bounds all sums and durations.
No sample history or heap allocation is used.

The report includes raw min/max even when calibration fails. One isolated
high spike does not establish a credible black crossing. Clipping, timing,
capacity or no data makes the result INVALID. With valid data but insufficient
white evidence, it is FALLBACK with 320/240 and NO_WHITE set. This is NOT
white evidence or permission to read air as white. The later reader must
independently qualify leading white. Abort does not retain old readiness.

LEARNED now means credible WHITE, even if NO_BLACK is set. The high/line
samples remain a sensor/surface observation, not a threshold scale. For
candidate-white mean `W`, the operator-approved policy is:

```text
rise = min(W + 150, 350)
fall = min(W + 75, rise - 75)
```

A mat line near 2700--3178 is not representative of printed-paper ink.
Changing black strength must not change these thresholds.

The operator confirmed fixed fallback rise 320 / fall 240 from white
150--180 and large narrow peaks 600--900 on the floor. Fallback is used when
spin WHITE is not credible, not merely because no black line was crossed.
The later reader must
qualify leading white before a candidate, may track that white before its
first edge, and must freeze thresholds throughout the window. Calibration
alone never authorizes a command.

**Air is not black evidence of floor contact.** Measured air 1170--1290
can exceed both thresholds and overlap black. Flat air/dark is not accepted
as white. Even a high interval bracketed by white does not prove which
surface caused it. Keep the sensor at its fixed 5.5 mm mount above the mat;
verify the physical sweep and the report together.

## Report fields

Example syntax, not a hardware result:

```text
[BC_CAL] gen ... samples ... GP28 min..max mode LEARNED faults 00
[BC_CAL] white mean/count black mean/count hold_us ... gap_us ...
         rise ... fall ...; shadow only
```

The second line is one physical log line. `black` denotes candidate-high
statistics, not a surface classifier. Modes are IDLE, COLLECTING, LEARNED,
FALLBACK, INVALID and ABORTED. Fault bits are:

| Hex bit | Meaning |
| --- | --- |
| 01 | ADC clipping or invalid range |
| 02 | Duplicate, reversed or excessive timestamp gap |
| 04 | Sample cap exceeded |
| 08 | Too few candidate-white samples |
| 10 | No high interval of at least 10 ms |

## Shadow hardware procedure

1. Use a ROBOT build with `JUNCTION_TEST_CMD=NONE`. A fake A/B/C/D build
   can still command an action: this step does not replace that source.
2. Place the robot as for normal LINE calibration. Keep the full sweep on
   the mat. The GP28 front-left sensor should cross the same 18 mm black
   line during the existing sweep. Do not lower the mount.
3. Keep USB connected for the report, or arrange an existing logging path.
   Unplugging USB can reset this hardware even with VIN; old-boot readings
   cannot prove the current calibration. No TEST DTR wait applies here.
4. Use the existing calibration operation. Where the existing CALIBRATE
   command is available, it ends stopped. Normal GP21 calibrate-and-run
   still starts LINE afterward: prepare a safe clear route and ESTOP.
   No new button sequence is introduced by this change.
5. Save both `[BC_CAL]` lines, pair calibration outcome and the observed
   physical crossing. Check current generation, nonzero sample count,
   min/max, white/high counts, high duration, gap and mode. A high maximum
   alone is not proof of crossing the line.
6. Repeat from a fresh boot. Record NO_BLACK when no high interval exists.
   With credible white, LEARNED still uses the white-only policy. Otherwise
   FALLBACK remains 320/240 and NO_WHITE must remain visible. Do not alter
   thresholds to obtain a preferred mode.

No live turn is permitted before the later offset check. The approved
method compares GP28 and line-pair encoder positions at the SAME transverse
18 mm stripe/cross arm. Frame-end-to-junction travel alone mixes unknown
course gap and sensor offset. The provisional 150 mm barcode-clear guard
is unverified and shadow-only; this step does not implement it.

## First home-mat result and policy correction

Main relayed these operator results from the home Cytron mat, USB attached,
using the `a883455` NONE MQTT image. These rows describe the OLD span policy,
not hardware execution of the corrected policy:

| Gen | Samples | GP28 range | White mean/count | High mean/count | Old rise/fall |
| --- | ---: | --- | --- | --- | --- |
| 1 | 2070 | 152..1820 | 168/1885 | 1521/134 | 303/249 |
| 2 | 2070 | 155..184 | 167/2070 | 0/0 | 320/240 |
| 3, clean run | 1980 | 152..3178 | 167/1904 | 2673/62 | 417/317 |

Generations 1/3 reported LEARNED, faults 00; generation 2 reported FALLBACK,
faults 10 under the OLD no-black fallback rule. High holds were 70003/61000 us
for generations 1/3; maximum sample gaps were 1027/1024 us. The operator
confirmed no line crossing in generation 2. Mat paint gave far more IR
contrast than printed ink, so the old high-span scale was rejected.

The approved correction computes 318/243 from generation 1's white and
317/242 from generations 2/3. Generation 2 will now mean LEARNED WHITE with
NO_BLACK still reported. Those corrected values are host expectations;
repeat corrected firmware on hardware before claiming a physical result.

Correction tests add dark-line 1000/2700/3178 invariance and the 350/275 cap.
The strict and Clang ASan/UBSan suites pass 261/261. No live reader or action
source is introduced by this correction.

## Initial software validation (`a883455`)

- Strict host suite: 259/259, including 18 pure calibration scenarios and
  the normal/capture line-owner integration tests.
- Clang ASan/UBSan: 259/259. The existing barcode TEST printf stub needs
  `-Wno-error=missing-format-attribute`; no new suppression is added to code.
- Firmware: normal, TEST capture, BENCH, MQTT and MQTT/fake-A compile and
  link without warnings. Public example Wi-Fi settings are removed afterward.
- Strict ARM recompilation: 10 units across those five profiles, with
  `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`.
- Retained SDK link proofs: normal and MQTT/fake-A. These are link-only
  probes, never hardware images.
- Matched MQTT/fake-A baseline relink: +136 B static RAM and +1308 B text.
  This uses the old line-owner object from `b8df7c9` with the same other
  objects, not a comparison between different SDK/configuration profiles.
- CR1 ARM manifest checks confirm a 56 B report and 60 B board.
- New state is 76 B; its separate board is 60 B. The report is 56 B.
  Compiler frames: calibration add 16 B; line task 64 B (was 48 B);
  calibration end 96 B (was 32 B); logger 40 B. These are individual frames,
  not complete call-chain or runtime stack proofs.
- Runtime CPU, stack headroom and live decoding are NOT RUN. The later
  operator spin observations above do not prove those properties.
  Existing no-command host LINE/action tests remain passing.

Logs, object/stack reports, baseline relink and copied SDK probes are under
`build/b2b-validation/`. WIP backups remain under `build/wip/`.
