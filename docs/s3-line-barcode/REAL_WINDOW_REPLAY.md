# Real straight-capture window replay

## Scope and result

Host-only comparison of old reader `25f70a7` and size-tolerant reader
`4667a83`. **All five fixtures give identical results with both readers**,
including rejection histories, observed processing times and fitted widths.
This is a regression lock, so no new RED-first claim is made. No physical
run, Webots run or new optical calibration was done for this comparison.
Later reader changes and robot evidence are in
[LIVE_WINDOW_READER.md](LIVE_WINDOW_READER.md). The replay below used a
4 mm floor; the reader now uses 2.5 mm.

The first fixture is the public-safe extraction documented under
[Supplied first floor 2x F capture](../../testing/subsystem3/TEST_LOG.md#supplied-first-floor-2x-f-capture-and-optical-correction).
Its older detector threshold retained only15 BC_RUN rows. Those rows are
ignored. The replay feeds the original GP28 ADC values, raw flags, motor
observation timestamps and signed encoder counts, not that detector's runs.
The ordinary Code39 table and reader headers are identical at both revisions.

## Reconstructed inputs and assumptions

The files do not contain the original spin-calibration report. The host test
reconstructs an **ASSUMED fallback report** from the initial uninterrupted
120--220 ADC prefix: rounded arithmetic mean and sample count, with the
reader's capped +150/+75 threshold formula. No requested letter or later
barcode amplitude is used for calibration. The window still has to earn NEW
spatial leading white and sets/fixes its actual thresholds from that white.
Thus the fallback seed is not a claim about the original detector settings.

| Fixture | ADC/motor rows | Prefix count/rounded mean ADC | Max ADC/motor gap us |
| --- | --- | --- | --- |
| `barcode_large_f_forward.csv` | 3080/180 | 414/164 | 2011/49998 |
| `barcode_430_large_fwd.csv` | 2245/188 | 364/165 | 2009/39999 |
| `barcode_430_large_back_cable.csv` | 2240/188 | 504/167 | 2008/39998 |
| `barcode_430_small_fwd.csv` | 2240/185 | 377/167 | 2008/49996 |
| `barcode_430_small_back.csv` | 2250/191 | 1300/163 | 2008/30006 |

The native retained-data profile uses an **ASSUMED host-only**3000 us ADC
and50000 us motor gap allowance, selected before this comparison to match
historical retained data. It does not weaken live defaults. Counts/m remains
12414; GP28 forward80/left77 and track111 mm remain the current configuration,
not a reconstruction of historical wheel/mount geometry. Initial signed
motor values are cast modulo32 bits without deleting negative counts or
changing backward motion into forward motion.

Only a common time origin (earliest ADC or motor row) is subtracted. Native
elapsed times and motor/ADC ordering are preserved, motor first on a tie.
No interpolation outside the reader, count scaling, extra motor rows,
reversal or invented high-rate ADC data is supplied.

## Native retained-data comparison

Times are seconds relative to the common source origin. For BW_OK, time is
the reader's ADC eligibility time. For a rejection, time is the host source
processing time at which the LAST rejection was observed, not a reconstructed
bar edge. A failed result has no accepted n/w: dashes do not mean a zero-width
physical barcode.

| Fixture | Reader | Final reason/time s | Letter | Fitted n/w mm |
| --- | --- | --- | --- | --- |
| `barcode_large_f_forward.csv` | old25f70a7 | BW_OK /3.686283 | F | 5.766/17.551 |
| same | new4667a83 | BW_OK /3.686283 | F | 5.766/17.551 |
| `barcode_430_large_fwd.csv` | old25f70a7 | BW_OK /3.526276 | F | 5.776/17.715 |
| same | new4667a83 | BW_OK /3.526276 | F | 5.776/17.715 |
| `barcode_430_large_back_cable.csv` | old25f70a7 | BW_MOTION /4.510001 | — | — |
| same | new4667a83 | BW_MOTION /4.510001 | — | — |
| `barcode_430_small_fwd.csv` | old25f70a7 | BW_WIDTH /1.670001 | — | — |
| same | new4667a83 | BW_WIDTH /1.670001 | — | — |
| `barcode_430_small_back.csv` | old25f70a7 | BW_MOTION /4.529999 | — | — |
| same | new4667a83 | BW_MOTION /4.529999 | — | — |

The first large fixture has no prior failures. The430 mm large forward
fixture has one initial BW_MOTION at0.020001 s, then earns fresh white and
decodes; it is not edited out. The two backward fixtures first reject for
motion at0.020000/0.020001 s and continue rejecting backward observations.
Small forward has one terminal BW_WIDTH and no valid fitted scale. These
captures therefore do not establish that the new4 mm lower bound can read
physical210 mm print. None supplies a measured straight210 mm acceptance.

### Other profiles

Both readers also match under the existing explicitly SYNTHETIC timestamp
compression (`100/146`), which changes delivery timing, not optical response.
Large-forward eligibility becomes2.524851/2.415257 s with the same fitted
widths; small forward rejects BW_WIDTH at1.143836 s, and backward rejects
BW_MOTION. Compression is not the native comparison above.

With unmodified live1500 us ADC/30000 us motor gap limits, ALL five original
files reject. Their first failure is BW_LOSS at about2.276--2.283 ms because
the retained ADC interval is about2 ms, not the live1 ms feed. Later motor
failures supersede that result. Reporting only the final live BW_MOTION
would hide the first source-gap cause; the test prints both. This is expected
source incompatibility, not a demonstrated physical barcode failure.

## Regression and reproduction

[`test_barcode_window_real.c`](../../tests/test_barcode_window_real.c) now
covers all five files in native retained, synthetic compressed retained,
and strict live profiles. It checks accepted letters, rejection categories,
source bounds, width tolerances and native accepted timing. It prints a CSV
including first/last failure observations and reconstructed calibration.
No fixture or firmware source changed for this replay.

For each revision, export its `barcode_window.c` under `build/`, then compile
this test with that file, `barcode.c`, the subsystem3 include path and
`BC_FIXTURES` set to `tests/fixtures`. The comparison used strict GCC and
retained both CSVs under `build/real-reader-replay/`. Comparing all fields
except the reader label gives no difference in all15 profile rows.
The test is part of the normal host CMake suite as
`test_barcode_window_real`; Clang ASan/UBSan also runs the same regression.
