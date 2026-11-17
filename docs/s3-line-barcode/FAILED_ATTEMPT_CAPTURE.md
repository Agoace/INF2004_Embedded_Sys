# Opt-in ROBOT failed-attempt capture

## Scope

Software candidate from `mtk3-b2b` `ad22f8f`, with staging `d44bd25` merged
before implementation. Staging includes barcode v2, scan integration
`7fe4054` and the matching dashboard. No robot flash or hardware run for
this capture change. The operator must approve the image before use.

The old `BC_EDGES` list starts only after a successful record publication.
It is post-window evidence, not a record of failed start-star edges. Thus
`total 0 retained 0` on the school KD/KDG all-failure runs is expected and
cannot explain `BW_START`. Do not infer the cause from that empty list.

This switch records the inputs of the real ROBOT decoder. It does not use
the TEST-only `BARCODE_CAPTURE` path, change thresholds or inject symbols.
It does not change LINE, motion, source control, record ACKs, command mapping,
MQTT v2, or the run-summary cutoff. The stop-cut summary/count consistency
question is separate work. No barcode location, lap boundary or route shape
is assumed; the first two FAILED bound attempts of the RUN are kept.

## Build

```sh
make -C build_make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
  KERNEL_RAM=1 BARCODE_MAP="F=LEFT" BARCODE_FAIL_CAPTURE=1
```

- `BARCODE_FAIL_CAPTURE=0` is the default. It adds no compiler flag, source
  object, buffer, call or image/profile suffix.
- `BARCODE_FAIL_CAPTURE=1` requires ROBOT (`TEST=0 BENCH=0`). It adds
  `_bcfail2` to the image name and `-bcfail2` to the build profile. Switching
  on/off or changing capacity invalidates old objects.
- `BARCODE_FAIL_SLOTS=2` retains the first two failures per run. The optional
  fallback `BARCODE_FAIL_SLOTS=1` retains only the first failure and uses
  `_bcfail1`. It is not required by the measured headroom below. A future
  combined image must be checked again; capacity is not selected at runtime.

For the proposed operator combination, also integrate `d06a787` and
`c4e39f8`, then add `LINE_KD=30 MOTION_SAME_TICK=1`. Those switches are NOT
merged into this helper branch. A throwaway combination builds successfully.
There is no firmware-C conflict. There are mechanical conflicts in the Make
option, image-suffix and profile-ID insertion points, and documentation rows:
keep all three independent conditions, including each closing `endif`.
Do not choose one side and lose another switch or stale-object guard.

## What is retained

Each of three fixed RAM buffers can hold:

| Data | Capacity | Meaning |
| --- | ---: | --- |
| Threshold-crossing rows |64 x24 B | ADC time/raw/polarity, decoder phase, interpolated GP28-axis distance, original upper motor bracket time and L/R counts |
| ADC rows |512 x8 B |64 rolling pre-start points plus the FIRST448 post-start points, at least4000 us apart |
| Context |172 B | Saved original frame/reference, thresholds, leading white distance, fitted model parameters, failure stage and bounded-array counters |

The three buffers allow one active capture plus two retained failures. A
terminal failure transfers buffer ownership; LINE does not copy or clear a
whole buffer. A successful terminal attempt releases its working buffer and
does not use a failure slot. Later attempts/laps cannot replace the first
two failures. A NEW nonzero run epoch releases the old captures; STOP epoch0
does not. Each retained buffer has5804 B; the array is17412 B. All capture
storage together is17436 B, including source/reader indices. A compile-time
ceiling limits the module to18 KiB.

The pre-start ring covers up to256 ms at a continuous4 ms sample interval.
The post-start prefix covers about1.792 s. It is NOT an unbounded full-code
ADC recording. Earlier white evidence can be outside the prefix; the stored
leading-white distance and frozen thresholds remain available. Gaps reduce
the number of observed samples and do not create replacement points.
Overflow keeps the earliest points/edges and reports the omitted totals.

Edge phase2 means window; phase3 means tail. Edges are sampled at the
actual processed ADC timestamp, normally1 ms
apart, NOT at the decimated trace interval. They are observed threshold
crossings before the current sample's width/start decision; a row does not
assert that the decoder accepted that edge. In particular a width rejection
can occur before the proposed crossing is admitted. The first-bar edge is
recorded when the real decoder binds its window. The failed stage's element
count is saved BEFORE the decoder resets it, so BW_START preserves9 runs.
No independent approximate edge detector is substituted for the real one.

Fault-triggering ADC values that cannot enter the processing queue bypass
decimation. Their flags OR the original flags with bit15 and diagnostic bits:
bit0 time discontinuity, bit1 raw0/4095 clipping, bit2 nonzero input flags,
bit3 queue full. Valid ROBOT input flags are currently zero. A motor/control
cancellation can have no new ADC edge; its terminal reason is still retained.

## Ownership and bounded work

LINE is the only buffer-state writer. DUMP is the sole reader. Before it
reads a frozen buffer, DUMP publishes that buffer's index in an atomic reader
indicator, then checks the buffer state. Before reuse, LINE changes the state
to active, then checks the reader indicator. Either DUMP sees active and
skips, or LINE sees the reader and skips that buffer. Neither waits. A new
run cannot overwrite a buffer while an old-run dump reads it.

Only aligned32-bit atomic loads/stores are used. RP2040 has no native atomic
compare/exchange; this implementation does not require it or `libatomic`.
Record arrays are immutable while read. DUMP releases its indicator after
printing. No large copy, heap allocation, mutex, task wait or publish occurs
in LINE. The source's existing CPU counters include this work, but hardware
cost and stack high-water have NOT been measured.

## Stopped dump format

The bounded capture dump remains before the large KR table. It adds:

```text
[BC_FAIL_STATUS] slots 2 edges 64 samples 512 pre 64 div_us 4000 missed_lifetime M
[BC_REF] ...                         # existing matching reference
[BC_FAIL_CAPTURE] epoch E index I ...
[BC_FAIL_EDGE] epoch E index I row R adc_us U raw V black B phase P mm_cmm X bracket_us T counts L/R
[BC_FAIL_ADC] epoch E index I row R us U raw V flags F pre 0/1
```

`BC_FAIL_CAPTURE` includes original start/terminal times, final source reason,
window reason (zero for a source-only cancellation), element count, rise/fall,
leading distance, fitted n/w/bias/space, `fit_valid`, pre/ADC/edge counts and
`adc_lost`/`edge_lost`. `fit_valid=0` means n/w are unavailable; zero model
fields are unset parameters, not measured physical zero. Distances use
0.01 mm print units, not an accuracy claim. Times and counts keep the existing
low32-bit modular-clock/encoder rules. Print direction is not robot motion.

The key is **epoch + internal source frame index**, not decoded serial:
failure serials are zero. Legacy BC_FRAME/BC_REF do not print that index;
match their adjacent capture by epoch/start_us. The normal8-row history can
wrap while the first two
captures stay frozen. Such captures are still printed after that history,
preceded by `BC_FAIL_REF` with the saved key/start/terminal/reason. A repeat
GP20 dump prints the same retained data. `missed_lifetime` counts a failure
that could not be paired to a working buffer; it does not count deliberate
omission after the two-failure limit. No extra MQTT payload is added.

## Software gates and RAM

All resource builds use public example settings, UP MQTT, KERNEL_RAM and
`BARCODE_MAP=F=LEFT`. GCC14.3.1; matched base/candidate exports share a path.
The combo additionally integrates KD `d06a787` and same-tick `c4e39f8` in a
throwaway tree, with both switches enabled.

| Build | .text flash | .data | .bss | vector | Allocated RAM | Unallocated SRAM |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Combined staging `d44bd25`, capture OFF |422224 |20104 |201052 |192 |221348 |48988 B |
| Combined staging + capture ON,2 failures |425224 |20360 |218484 |192 |239036 |31300 B |
| Above + KD30 + same-tick1 |425168 |20776 |218580 |192 |239548 |30788 B |

Allocated RAM is `vector + .data + .bss`. Unallocated is physical SRAM
270336 B minus that allocation (`__end` to `0x20042000`). All static application
and kernel stack arrays, fixed pools and heap reservation are already in
those sections; stacks are NOT subtracted a second time. This is link-time
headroom, not free bytes inside a dynamic heap or measured stack margin.
Both ON combinations exceed the required8 KiB floor. Retain two failures.

RAM proof:75/75 symbols PASS for staging and capture,77/77 for the same-tick
combination. Capture adds17688 B allocated RAM and3000 B flash `.text` to
the matched combined base. Individual compiler frames: window step128 ->144 B;
LINE96, source96, producer88, telemetry432, DUMP96 and MQTT service312 B stay
unchanged. The new print helper uses160 B; observer40 B; bad-ADC helper32 B.
These are NOT full-call-chain/high-water results. Stack reservations did not
change. Terminal/capture helpers can call flash; no all-RAM timing claim.

Acceptance evidence:

- RED: new host test first failed on missing capture API, before implementation.
- GCC563/563; Clang ASan/UBSan563/563, strict warnings enabled.
- Unit tests: both1/2 retained-slot configurations, first-failure retention,
  stop/new-epoch boundaries, success discard, overflow counters, reader reuse
  protection and raw fault capture across clock wrap.
- Actual source tests: BW_START with9 runs saved before reset, final source
  rejection, same-epoch cancellation, old-epoch exclusion, success no-capture;
  output order after BC_REF, unchanged v2 and existing feed behavior.
- Make tests: ROBOT guard, invalid options, default adds nothing, capacity
  suffixes and stale-object invalidation.
- Capture OFF: **76/76 UF2 literal SHA MATCH** and **76/76 ELF PT_LOAD MATCH**
  vs `d44bd25`:38 original public profiles and38 with `F=LEFT`. This tests
  two38-profile suites, not76 distinct placement/mode combinations. Debug
  line/macro differences are permitted; each segment's VMA, LMA, file/memory
  sizes, flags, alignment and load bytes match. Every build has zero warnings.
- Documentation/link/anchor check PASS;8 recorded template deviations unchanged.
- No calibration JSON, optical assumptions, action policy or control tuning
  changed. No hardware, Webots, flash or push for this work.

Local logs/artifacts are under `build/barcode-failcap/` and
`/tmp/sw10-{bc-failcap-ram,failcap-builtin,failcap-fleft,failcap-combo}`.
