# CR1 barcode A: private evidence storage

## Authority and scope

OPERATOR approval, relayed by Main on 2026-11-18: change only the private
fail-capture edge arrays and stored barcode frames. This is category A
(representation). Public decoded types, units and APIs do not change.

- Keep two failed slots plus one working slot, 64 edges per slot, 512 ADC
  rows per slot and the 64-row ADC prefix. Keep eight frame boards.
- Keep ADC rows, binary32 values, all timestamps, counts and counter wrap.
- Do not change reader, thresholds, calibration, projection or command policy.
- Do not change the locked line path, queue, `lp_t`, snapshot or their boards.
- Do not change MQTT bytes or the attempt FIFO.
- Category B is NOT approved: no smaller capacities, fixed-point conversion,
  timestamp deltas or removal of duplicate data.
- Do not push, flash or merge into another branch. Main owns integration.
  At this substep the LIVE robot run was pending. Current: CR1 is merged;
  image 12 checked barcode LIVE, timing and stacks on 2026-11-19. See
  [project status](PROJECT_STATUS.md#areas) and [demo-kernel-v2](../release/DEMO_KERNEL_V2.md).
  The tables below retain this substep's baseline and measurements.

Implementation: [barcode_evidence_pack.h](../../app_program/subsystem3/barcode_evidence_pack.h),
[barcode_live_source.c](../../app_program/subsystem3/barcode_live_source.c) and
[barcode_fail_capture.inc](../../app_program/subsystem3/barcode_fail_capture.inc).

## Representation and ownership audit

The bit counts below are logical widths, not automatic RAM savings. Rows marked
"defer" do not change in this substep. No C bit-fields or unaligned packed
structures are used.

| Stored field | Behaviour-valid range | Logical bits | Writer and access |
| --- | --- | --- | --- |
| Driver raw L/C/BC | ADC 0..4095 | 12 each | ISR to existing core-0 queue; defer |
| Acquisition time, counts, statistics, serials | Full 32-bit wrap; no shorter approved span | 32 each | Separate ISR/task-owned words; defer |
| Line norm / position / flags | 0..1000 / -100..100 / nine flag bits | 10 / 8 / 9 | LINE and Vehicle, serialized by inherited mutex; defer |
| Line Q8 values, sums and times | Full generic API bounds not proved | Retain 32 | Same serialized owners; defer |
| Line pair target / peak / side / hint / booleans | -1000..1000 / 0..1000 / 0..3 / 0..3 / 0..1 | 11 / 10 / 2 / 2 / 1 | Same serialized owners; defer |
| GP28 calibration mode / faults / sample counts | 0..5 / five independent faults / 0..65535 | 3 / 5 / 16 each | LINE and calibration APIs, serialized; defer |
| Window ADC / motor and pose data | Raw valid 12 bits; rejected raw and flags can be full 16; times/counts/binary32 exact | Retain current widths | LINE private; defer |
| Window head / queued / motor count / elements / phase / white count | 0..63 / 0..64 / 0..8 / 0..29 / 0..4 / 0..16 | 6 / 7 / 4 / 5 / 3 / 5 | LINE private; defer |
| Window and producer booleans | 0..1 | 1 each | LINE private; defer |
| Stored frame record command / valid / source / direction / flags / version | 0..4 / 0..1 / 0..2 / 0..2 / all eight bits / 0..1 | 3 / 1 / 2 / 2 / 8 / 1 | LINE; coherent frame board or frozen slot |
| Live control mode / context and ACK | 0..2 / full 32-bit identities | 2 / 32 each | Vehicle to its existing board; defer, coordinate with vehicle owner |
| Frame reason / letter / leading white / rise / fall | 19 reasons / 45 character codes / 0..220 / 0..350 / 0..275 | 5 / 6 / 8 / 9 / 9 | LINE; coherent frame board or frozen slot |
| Frame times, counts, ages, spin white / floats | Existing full 32-bit values / binary32 bits | 32 each | Same owner; do not narrow |
| Fail edge raw / black / phase | 0..4095 / 0..1 / WINDOW=2 or TAIL=3 | 12 / 1 / 1 | LINE writes ACTIVE slot; DUMP reads FROZEN slot after hazard advertisement |
| Fail ADC raw / flags / time | Faulted raw 0..65535 / full flags16 / time32 | 16 / 16 / 32 | Same hazard transfer; unchanged 8 B rows |
| Fail prefix head / prefix count / post ADC count / edge count | 0..63 / 0..64 / 0..448 / 0..64 | 6 / 7 / 9 / 7 | LINE private; defer metadata packing |
| Slot state / DUMP hazard | FREE, ACTIVE, FROZEN / none or one of three slots | 2 logical; **32 stored each** | LINE owns state; DUMP owns separate hazard word; retain whole-word atomics |
| Terminal FIFO | Eight rows, full 32-bit indices | Retain 28 B rows | LINE owns payload/head/drop; Wi-Fi owns tail; existing release/acquire transfer |
| TEST packed ADC | Raw12, gap1, delta16 | 29 logical / 32 stored | Existing TEST handoff; unchanged |

**Ownership finding:** `line_service` is not a sole-LINE-writer object. Vehicle
can consume its latched event and call calibration/reset APIs. Startup also
initializes it. Keep the inherited mutex. Any later change to this locked path
needs identical host traces **and a robot run**. Local IRQ masking does not
provide a cross-core queue transfer.

Calibration minimum/maximum and rejected ADC evidence can contain invalid raw
values up to 65535. Do not assume that all diagnostic raw fields are 12-bit.

## Exact stored frame layout

A frame is 22 aligned `uint32_t` words (88 B). Bit zero is the least-significant
bit of word zero. A field can cross words. All 32-bit fields keep every bit;
there is no timestamp delta, float conversion or count truncation.

| Inclusive bit range | Field |
| --- | --- |
| 0..31 / 32..63 / 64..95 | record epoch / sequence / read_ms |
| 96..127 / 128..159 | record read counts L / R, signed bit pattern |
| 160..191 / 192..223 | record frame-end counts L / R, signed bit pattern |
| 224..255 | record eligibility_revision |
| 256..258 / 259 / 260..261 | command / valid / source |
| 262..267 / 268..269 / 270..277 / 278 | letter code / direction / flags / version |
| 279..310 / 311..315 | frame index / reason code |
| 316..347 / 348..379 / 380..411 | start_us / edge_us / frame_us |
| 412..443 / 444..475 | eligible_us / terminal_us |
| 476..507 / 508..539 | control_age_ms / record_age_ms |
| 540..571 / 572..579 | spin_white (full 32 bits) / leading_white |
| 580..611 / 612..643 / 644..675 | narrow_mm / wide_mm / leading_mm, unchanged binary32 bits |
| 676..684 / 685..693 | rise / fall |
| 694..703 | Reserved: write zero, reject nonzero |

Letter code zero means unknown/NUL. Codes 1..44 index, in order,
`0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*`. Codes 45..63 are invalid.
Reason codes 0..10 mean BW reasons 0..10; codes 11..18 mean BP reasons
100..107. Codes 19..31 are invalid. `record.reserved` is reconstructed as zero.
All other invalid field values are rejected, not masked into a valid value.
Pack/unpack failure leaves the caller's output unchanged.

The thresholds above come from the existing qualified-white path (white
120..220, rise cap 350, fall cap 275). Spin-white evidence keeps its full
32-bit API width. Zero remains valid for evidence not yet observed.

The public native frame remains 100 B. Each stored frame board changes from
104 B to 92 B, including its separate 32-bit sequence. LINE packs a local value
before publication. The reader copies the atomic words into local storage,
checks coherence and only then decodes. No shared packed-word update is used.

## Exact stored edge layout

Each edge uses 174 bits. Edge `i` starts at bit `174*i` in an aligned word
array. The array has 64 records, 11136 bits, 348 words, or 1392 B.
There are no unused bits between records.

| Offset from edge start | Field |
| --- | --- |
| 0..31 / 32..63 | ADC time / upper motor time |
| 64..95 / 96..127 | Left / right counts, full 32 bits |
| 128..159 | mm, unchanged binary32 bits |
| 160..171 / 172 / 173 | raw12 / black1 / phase1 |

The phase bit decodes as `2 + bit`: WINDOW or TAIL. Start records are captured
after entry to WINDOW; later edges occur only in WINDOW or TAIL. Rejected ADC
inputs remain in the unchanged ADC evidence array. They are not stored as
edges. A partial array is read only up to its existing `edge_count`.

LINE remains the only slot payload writer. DUMP advertises a hazard before
it inspects a FROZEN slot. The writer marks a candidate ACTIVE before it checks
the hazard. Keep that protocol and its separate aligned atomic words. Do not
wait in LINE or combine state and hazard flags into a shared read/modify/write.

## RAM evidence

MEASURED, Arm GNU 14.3.Rel1, Cortex-M0+, public example secrets. The sole
before baseline is SW7's `7350da0`, at
`~/Projects/cr1-sw7-sonar/build/wip/cr1/BASELINE_7350da0.md`.
No second firmware baseline was built. The after image is from implementation
commit `9575191`; red tests are `4fdc7d2`. The after build uses the same profile:

```sh
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
```

| Linked RAM | Before B | After B | Change B |
| --- | ---: | ---: | ---: |
| All static RAM output sections | 241228 | 240892 | -336 |
| Unreserved RAM (DERIVED from 270336 B) | 29108 | 29444 | +336 |
| S3 stored data | 23625 | 23061 | -564 |
| S3 RAM code | 12876 | 13084 | +208 |
| S3 total | 36501 | 36145 | -356 |
| Alignment / other output bytes | 927 | 947 | +20 |
| Listed kernel RAM functions | 21004 | 21212 | +208 |
| Failed-capture slots | 17448 | 16980 | -468 |
| Eight stored frame boards | 832 | 736 | -96 |

The 564 B data saving is category A only: edge arrays save 432 B, the three
capture frames save 36 B, and the eight frame boards save 96 B. Do not count
the capture frames a second time. RAM code grew: `barcode_live_source_sample`
changes from 2400 B to 2608 B. Other module totals are unchanged. Alignment
and other output bytes grow by 20 B: a new 16 B RAM-to-flash call stub
(`__bcep_frame_pack.part.0_veneer`) plus 4 B of padding. The listed RAM
functions grow by 208 B; including that stub, executable RAM grows by 224 B.
Thus the **net static saving is 336 B**,
not 564 B. Existing capacity is not claimed as a saving.

The after build has zero warnings, including assembler output, and
`RAM_PROOF PASS 79/79`, 21212 B. A new direct flash callee,
`bcep_frame_pack.part.0`, is reported from the RAM source function; it is not
claimed to be in RAM. Do not interpret 79/79 as a proof that every callee is
in RAM or that the LINE deadline is met.

| After image | SHA256 |
| --- | --- |
| ELF | `4cf85057c55bea263349fc3d615337d891642c6e89127b70bae28b08dbeea1ae` |
| UF2 | `4b8416d007b4eb04cc6d8c6205ce0c90653fbd2002e2eca72f4015ae6bca7eea` |

Generated logs, maps and replay CSVs stay under `build/wip/cr1/`, out of Git.
Use `scripts/ram_report.py <map> --module subsystem3 --top 999` for the split.

## Tests and timing limits

- Red-first commit: `4fdc7d2` fails because the evidence codec is absent.
- GCC: **679/679**, the existing 677 plus codec and evidence-replay tests.
- Clang with AddressSanitizer and UndefinedBehaviorSanitizer: **679/679**.
- Codec tests cover all 4096 raw values times both black values and both
  phases; all word-crossing positions and independent neighbouring records;
  all reason and character codes, thresholds, directions and flag bytes;
  full-width integer bit walks and wrap; signed-zero, finite, infinity and
  NaN payload bits; invalid and reserved values with unchanged failure output.
- A publication interrupts each of the 22 frame-word reads. The failed read
  cannot modify the decoded output. Zero, odd and wrapped sequence cases pass.
- Existing fail-capture tests retain slot/hazard/new-epoch/STOP behaviour,
  success-slot reuse, full fault raw/flags and old evidence on a held hazard.
- Real-capture replay runs all five existing captures in three timing profiles.
  Each result also passes a bit-exact frame pack/unpack check. Native and
  codec-checked CSV outputs are byte-identical, SHA256
  `466adf222f6f5d8e5346698442038a44e1dad66bf2c8aa171ad7754ce7ed55ab`.
  Original acceptance, reason, letter, event time, widths, failure counts and
  threshold assertions still pass. Native-live failures remain failures;
  this does not claim that retained logs now decode under live timing.
- Locked line and reader sources are unchanged. No line robot proof is claimed.

**Timing limit:** no RP2040 per-frame or per-edge duration was measured. There
is no permission to flash. The codec adds bounded integer work in LINE: eight
field writes per retained edge (at most 14 word updates), and 31 field writes
per stored frame. Each frame pack scans at most 44 character codes twice.
A retained failure can require both a capture-frame pack and a board-frame
pack. Three fewer words are published per frame board. There is no float
arithmetic, allocation or wait in the codec. These operation counts are
DERIVED, not a processor timing measurement. The existing source CPU counters
include the changed work. Check LINE timing and stack on the post-merge LIVE
robot run, including flash access and USB/DUMP activity.

DERIVED from the linked ARM instruction sequences (saved registers included):

| Function's own stack frame | Before B | After B |
| --- | ---: | ---: |
| `barcode_live_source_sample` | 96 | 184 |
| New `bcep_frame_pack.part.0` child | absent | 160 |
| `bcfc_print` (DUMP, not LINE) | 160 | 272 |
| `barcode_live_source_dump` (DUMP) | 96 | 184 |

The source-to-pack call path therefore uses 344 B before `memcpy` or other
callees. This is not a complete task-stack bound or a measured watermark.
The LINE stack remains 2048 B. No increase in stack allocation is claimed
as a data saving. The later combined image 12 checked stack margins;
compiler-local frames list each function's own stack use.
