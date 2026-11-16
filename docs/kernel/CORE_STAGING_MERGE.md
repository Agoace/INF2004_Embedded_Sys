# Core migration and staging integration

## Scope

This is the operator-approved no-fast-forward merge of staging `c193fcd`
into core migration `8f224b2`. It combines the B2b SHADOW source with the
reviewed core placement, startup, admission, UP primitive and Wi-Fi stack
changes. It does not approve live actions or claim a new device result.

## Resolution contract

- Keep every normal B2b message kind before the TEST-only row probe, then
  `TELEMETRY_KIND_COUNT`. Normal message numbers are unchanged. Admission
  uses this count, not the pre-B2b upper bound.
- Keep the B2b source/count caches, BEGIN reset, stopped barcode hook and
  strong S3 dump implementation. Normal UP, split and forced-core0 ELFs
  retain `T barcode_live_source_dump`.
- Keep M3 atomic DUMP reservation, failed-send rollback and busy refusal
  for all non-DUMP kinds. Add refusal cases for SOURCE, BARCODE_DUMP and
  COUNTS in all three host placements.
- Keep M5 exact-token TEST probes and the ordinary/probe shared row handler.
- Keep the M6 checker. The Make profile ends in
  `junctionN-forcecoreN-barcode_captureN-bcliveN-bcclearN`. Artifact names
  retain both placement and capture suffixes.
- Keep the complete test suites. Telemetry-only clones inherit their fake
  source setting. Real Vehicle/Telemetry integration links the real S3
  strong dump hook, not a second fixture definition. Production startup
  fixtures also link the canonical barcode table implementation.
- Retain M8 publication after all subsystem init/start calls and Vehicle
  startup return, including B2b initialization. The auto-merged boot table
  print stays before tasks; it uses the canonical S3 table only.

The vehicle, source/adapter, LINE/SEARCH and producer code from staging is
not redesigned. Source2 stays SHADOW, `BARCODE_LIVE_ENABLE=0` and measured
bound 0. Fake A-D remain exclusive source1 builds. IMU remains report-only.
The [staging SHADOW results](../s3-line-barcode/SHADOW_HARDWARE_RESULTS.md)
remain revision-scoped; their decoder, diagnostic and disabled-path issues
are not repaired or reclassified by this merge.

## Host validation

| Suite | Parent cases | Missing after merge |
| --- | ---: | ---: |
| Migration `8f224b2` | 336 | 0 |
| Staging `c193fcd` | 363 | 0 |
| Unique parent union | 508 | 0 |
| Merge, including nine added busy cases | 517 | 0 |

Test names were compared mechanically against both exact parent trees.
GCC passes 517/517. Clang with ASan, UBSan, float-cast-overflow and leak
detection passes 517/517. No global warning option is weakened. Tests use
bounded host doubles and do not establish RP2040 scheduling or timing.

## Firmware validation

All 38 public-example profiles link with zero normal compiler warnings:

- 24 standard profiles: UP, split and forced-core0, each with USB, TEST,
  BENCH, Wi-Fi, lwIP, MQTT, MQTT-TEST and MQTT-BENCH.
- Two existing barcode capture profiles: UP, TEST, radio off, with
  FORCE_CORE0 0 and 1. The capture requirement for SMP0 is not relaxed.
- Twelve exclusive fake profiles: A-D for UP, split and forced-core0 MQTT.

Live permission stays off in every firmware build. Synthetic permission
cases remain host-only. All 38 separate retained SDK probes link, retaining
the required symbols with no unresolved or forbidden runtime references.
They are link-only proof ELFs; do not flash them.

Strict ARM checks pass for all 92 application units in all 38 profiles:
3496 checks with `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`.

ARM ABI checks retain telemetry116 B, INFO108 B, event12 B, KR1 record16 B,
summary52 B, barcode record40 B and control20 B. The message buffer stays
4096 B. The source/count cache sizes remain 84/36 B.

## Matched RAM comparison

The exact parents and merge were built with the same public examples and
toolchain, SMP1, USB, Wi-Fi/lwIP/MQTT, corrected IMU, fake NONE and live off.
Static RAM is linked data plus BSS, not remaining runtime heap.

| Image / revision | Text B | Data B | BSS B | Static B |
| --- | ---: | ---: | ---: | ---: |
| Production migration `8f224b2` | 419756 | 1032 | 199532 | 200564 |
| Production staging `c193fcd` | 432956 | 896 | 194244 | 195140 |
| Production merge | 435364 | 1032 | 203436 | 204468 |
| TEST migration `8f224b2` | 408716 | 1032 | 198084 | 199116 |
| TEST staging `c193fcd` | 402884 | 896 | 189132 | 190028 |
| TEST merge | 410336 | 1032 | 198388 | 199420 |

Production merge delta: +3904 B static / +15608 B text versus migration;
+9328 B static / +2408 B text versus staging. TEST delta: +304 B static /
+1620 B text versus migration; +9392 B static / +7452 B text versus staging.

The staging comparison includes M9's move of the 8320 B Wi-Fi stack from
runtime allocation to the static map. It is not all new runtime demand,
and it is not a RAM saving. See the
[stack accounting](WIFI_STACK_WATERMARK.md#ram-static-reserve-up-dynamic-allocation-down).
Do not subtract the removed allocation a second time or claim heap-free
margin from these section totals. No additional task or stack is introduced
by conflict resolution.

## Evidence and limits

Local software artifacts are under `build/merge-review/`: both exact parent
source/build trees, the public merged tree, full build/probe/strict logs,
ABI and size reports, test-name union and sanitizer logs. A precautionary
patch/status backup is under `build/wip/merge_staging_core/`.

No private log, configured operator image, push or flash is part of this
work. Named-task stack margin, CPU/deadline load, physical stop latency,
placement and decoder acceptance remain hardware gates. Historical B2a,
SHADOW and IMU evidence must not be relabelled as acceptance of this image.
