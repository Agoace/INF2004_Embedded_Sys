# Junction and barcode observations — old window implementation

**Gate1 remains TRIAL INCOMPLETE / NOT GATE1 PASS.** These are simulation
observations, not physical barcode or scheduling acceptance. All four approved
Webots runs are finished; all owned process groups are gone.

## Version and protocol

All runs use the **old wheel-midpoint barcode_window**, before52244f2. No result
in this record tests the new GP28-position width calculation. Native source
hashes and library bytes are identical across all three letter runs:
`7a6d0d5c18088b5c9ba4134313b610307c2a167b93909a0368d8ed500ea00412`.

See [wiring and assumptions](SIL_JUNCTION_BARCODE_20261116.md) and
[print geometry](SIL_BARCODE_GEOMETRY.md). GP28 along-sigma1/4/8mm are ASSUMED;
A4 is297x210mm MEASURED. Print centring, zero paper thickness, placement,
MAT response power1 and noise0 remain ASSUMED; bar height160+/-10mm is
PHOTO_ESTIMATED. MAT178/2321 are unlocated MQTT proxies, not material calibration.

One aligned motor-driven Webots spin supplies public LP and GP28 calibration
APIs. Each independent start replays the actual sampled rows. P7, native
motor_ctl, jl_module/junction_action and SEARCH remain unchanged. SHADOW,
valid0 and no OFFSET_VERIFIED remain unchanged. Pose is diagnostic only.

## Observed results

A maps to LEFT in the native table. **No LEFT action was admitted or executed.**

| Along sigma ASSUMED | Letter / native window result | Attempts / producer failures | Quiet / start proof | Junction confirmations | Last native state | Run status |
| --- | --- | --- | --- | --- | --- | --- |
|1mm |A / BW_OK |1 /0 |yes /yes |1 |LINE /RUNNING; JA_IDLE; fault0 |INCOMPLETE: invalid GP28 tail at5120ms |
|4mm |A / BW_OK |1 /0 |yes /yes |1 |LINE /RUNNING; JA_IDLE; fault0 |COMPLETE at3520ms native confirmation |
|8mm |A / BW_OK |1 /0 |yes /yes |1 |LINE /RUNNING; JA_IDLE; fault0 |COMPLETE at3340ms native confirmation |

Every published record has flags191 (0xBF), valid0, OFFSET_VERIFIED clear.
Actual frame admission reason is JLIVE_SHADOW(1); actual junction command is
NONE(0), not LEFT. SEARCH is registered but is not activated in these runs.
"COMPLETE" means the declared simulation segment finished, not Gate1 PASS.

| Along | Window read time | Native leading white | Fitted narrow /wide | Window rise /fall |
| --- | --- | --- | --- | --- |
|1 |4090ms |312.107mm |5.722 /17.659mm |324 /249ADC |
|4 |2720ms |112.147mm |5.756 /17.618mm |323 /248ADC |
|8 |2550ms |87.421mm |5.701 /17.632mm |325 /250ADC |

The unchanged native leading10n gate and start-star test pass. This is **not**
a claim that11.75mm of paper quiet alone meets10n: actual MAT context supplies
the rest. No paper pad or paper-only sampling domain is used.

## Speeds and time budget

Wheel speeds below are10ms means from recorded encoder angles, radius32.5mm.
Both wheels satisfy the declared146mm/s +/-5% band (138.7..153.3mm/s) before
the first paper-support contact and at the first ink. Instantaneous1ms values
are also retained in the audit and satisfy that band at these points.

| Along | First support touches paper L/R | GP28 centre crosses paper edge L/R | First ink centre L/R |
| --- | --- | --- | --- |
|1 |145.88 /146.17 |145.83 /145.99 |146.06 /145.92mm/s |
|4 |151.18 /145.86 |148.52 /147.37 |147.23 /147.54mm/s |
|8 |150.48 /150.98 |149.51 /149.96 |148.42 /149.18mm/s |

| Run | Lead before paper | Recorded simulation | Wall time | Approved cap | GP28 fine /uniform calls |
| --- | --- | --- | --- | --- | --- |
|Aligned spin, all3 hypotheses |not applicable |2260ms |762.94s |1200s |2651 /4129 |
|Along1 |300mm |5119ms; invalid next sample |about871.33s |1200s |2354 /2766 |
|Along4 |100mm |3520ms |743.27s |1200s |2264 /1256 |
|Along8 |75mm |3340ms |1250.62s |1500s, THIS run only |2258 /1082 |

Along1 wall time is a filesystem-time estimate; the other times are measured
by the launcher monotonic clock. Along8 exceeded its1155–1170s estimate and
would have exceeded1200s. Main approved1500s before launch; the wrapper and
all native C/profile inputs stayed unchanged. No further run was made.

## Calibration and numerical limits

LP endpoints are172..2488 and158..2614. GP28 reports are LEARNED(mode2),
1740samples each, faults0, white mean178, rise328/fall253; maxima are
2321/2306/2003ADC and black holds71/78/89ms for1/4/8 respectively. These
are **simulated** calibration results, not new hardware measurements.

Line channels retain sample_sensor. GP28 uses fine_sample1e-7 or the explicit
original-face uniform-MAT proof. On24 selected recorded-pose checks,1e-7 vs
1e-8 gives max2.277e-6/p952.153e-6ADC; all are valid. SW4's independent
straight-A4 analytic/reference spots agree within1.82e-7ADC. The coarse
q17/q257 and dense128 failures remain recorded; no fixed-grid fallback is used.
The physical GP28 edge/holdout gate remains **FAIL**.

Along1's GP28 REFEREE_COVERAGE_ERROR at5120ms is fatal. Its original summary,
5119-row CSV and failure remain unchanged. A separate replay matches all5119
recorded native/plant rows; this does **not** complete or retroactively truncate
the original5300ms request. A was decoded at4090ms and the native junction was
confirmed at4890ms, before the invalid sample. The later runs' shorter starts
and first-confirmation stops were approved **before** their launches. Different
starts/end criteria prevent a strict single-factor or repeated-physics claim.
C input replay is not repeated physics.

The SDK replacement is synchronous, not an RTOS or board-fault model. It
presents a record while the producer occupies its slot; diagnostic READ_FAILURE
counts outside that interval must not be interpreted as hardware board faults.
No claim about hardware scheduling, barcode F, zigzag, or actual LEFT turns is
supported by these runs.

## Validation

349 host tests pass, with3 expected failures. The unchanged native position,
steering, motor, LINE and vehicle-shell tests pass. Frozen replay remains
158RX/58TX MATCH, physical_gate1_pass=false. Documentation check scans131
files with no broken links/anchors; whitespace check is clean.

## Retained evidence

Under `build/webots/` (ignored, retained locally):

- `sil-bc-spin-20261116T072506Z/spin-result/calibration.json`
- `sil-bc-A-20261116T074205Z/along1/` — original INCOMPLETE and separate prefix audit.
- `sil-bc-A4-short-20261116T081033Z/along4/`
- `sil-bc-A8-short-20261116T083318Z/along8/`
- `sil-bc-numerics-20261116/` — SW4 handoff and recorded-pose tolerance checks.
- `sil-bc-delivery-20261116/audit.json` — hashes, original statuses, replay checks,
  speeds, events, calibration, source equality and process cleanup.

Original generated manifests match for all letter projects. The spin manifest
retains its pre-overlay world checksum; the delivery audit exposes that one
mismatch and separately seals the actual spin world/controller. No original
manifest is rewritten. Owned PGIDs628373/697966/900084/905013 are all gone.
