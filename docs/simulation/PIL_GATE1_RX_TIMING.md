# Gate 1 — stopped RX timing diagnostic version1

## Scope

This diagnostic locates delay between PC write, Pico UART delimiter reception,
worker scheduling, decode and receive/stage dispatch. It does not change live
UART v1, D1 v1 binary records, scheduling priority, deadlines, queue capacities,
clock qualification, transport acceptance or physical acceptance rules.

Attempt5 underflowed at demand657: epoch1170263, target1827263, stop1827305.
Slots0..657 were staged,0..656 applied. Current slot657 was ready; lookahead658
was not ready. DROPPED6, MISSED1 and REFUSED1 do not identify whether the late
packet waited in the PC/Probe path or the Pico worker. PC wrote SENSOR658 at
590110024; it continued through663. D1 UART errors/late/lease/gap were zero.
Worker wall span925us covered1827135..1828060; RX depth reached4 at1827402.
The old image did not retain rejected packets' RX stamps. Ownership is not
proved from that evidence. No physical Gate1 PASS is implied by this record.

## Bounded capture

[RX log source](../../app_program/sim/pil_gate1_rxlog.c) holds64 pre-fault packet
records,16 post-fault records and16 alarm records. The first latched session
fault freezes pre-fault allocation and the alarm ring. An already allocated
packet can still receive its worker/decode/stage completion stamps after that
fault. New packets use the separate post-fault tail. Extra post-fault packets
increment a diagnostic loss count; they do not alter session counters/faults.
The normal stopped runtime freeze seals every diagnostic field before printing.

The receive IRQ records only delimiter-complete packets, including RX queue-full
packets. Empty delimiters and oversized/unframed byte streams are not packet
records; existing D1 error counters retain their original meaning. A queue-full
record has no worker stamps. No CRC, formatting or USB calls occur in the IRQ.
Packet tags use existing private queue metadata, not live wire fields.

## ASCII envelope and canonical order

Append this printable block AFTER the complete D1 v1 manifest and BEFORE the
unchanged `[PILG1] stopped; reset required; worker stack untouched ...` line.
Print one row per service iteration. The PC capture must continue through that
stack line; neither a fixed USB tail timeout nor the UART drain timeout applies
as a USB completion condition. The block contains no zero bytes. Existing D1
readers can skip printable console text without accepting a new binary kind.

Every value below is an unsigned32-bit word: exactly eight lowercase hex digits,
no prefix, separated by one space. Each line ends `]` then LF.

```text
[PILG1RX1 BEGIN nonce session fault_us reason pre post alarms post_lost]
[PILG1RX1 P serial rx worker decoded enter done seq source meta aux]
... pre+post P rows ...
[PILG1RX1 A demand entry sequence ready reason done]
... alarms A rows ...
[PILG1RX1 END packet_count alarm_count crc32c]
```

`pre` is min(total pre-fault allocated,64), not the number of completed workers.
`post` is allocated post-fault records,0..16. `alarms` is0..16. Print retained
pre-fault P rows oldest first, then post-fault P rows, all in increasing serial
order. Then print A rows oldest first, including the failing alarm when the
fault occurs inside that alarm. There are no P/A rows after END.

`packet_count=pre+post`; `alarm_count=alarms`. The CRC is standard CRC32C over
canonical little-endian words: BEGIN's8 words, each P's10 words in printed
order, then each A's6 words in printed order. END counts/CRC and ASCII characters
are NOT included. This is the same initial/final XOR convention as
`pil_crc32c`. The nonce/session/reason/fault time must match stopped D1.

## Packet words

| Word | Meaning |
| --- | --- |
| serial | Allocation counter, starts0. Retained rows can start above0 after wrap of the64-entry ring. |
| rx | TIMER read after UARTDR delivered the delimiter, before stream completion/IRQ queue accounting. |
| worker | TIMER read with IRQ exclusion after taking the queue-front pointer, before CRC/decode. |
| decoded | TIMER read immediately after `pil_decode`, before acquiring receive/stage IRQ exclusion. |
| enter | TIMER read after queue drop/accounting, before error handling or `pil_gate1_receive`. This brackets dispatch, not the exact slot-ready write. |
| done | TIMER read after error handling/receive dispatch, before lease update. Existing D1 traces still give exact slot availability for accepted SENSORs. |
| seq/source | Decoded header fields, only when decode succeeds. |
| meta | type bits0..7; decode result bits8..15; session reason at dispatch completion bits16..31. |
| aux | length bits0..7; depth at RX bits8..11; worker-start depth bits12..15; post-drop depth bits16..19; RX result bits20..23; flags bits24..31. |

Unknown/unreached timestamps, seq and source are `ffffffff`. Type/result start
as`ff`; reason starts as`ffff`. Decode result is the low8 bits of the frozen C
codec return:00=OK,ff=INVALID,fe=SIZE,fd=CRC. An unset `decoded` stamp identifies
pending decode; result`ff` with a set stamp means INVALID. On decode failure,
type/seq/source stay unknown. AUX depth`f` means not reached; observed depths
are0..8. Length includes the delimiter. RX result1 means queued,2 means queue
full. Flags: bit0 means RX allocated after first fault; bit1 means dispatch
observed an already latched fault on entry or completion. Other flag bits are
reserved zero. An in-progress pre-fault RX can have the post-fault-dispatch bit
set while the post-fault-RX bit remains clear.
Receive/stage brackets also cover control frames and rejected packets; do not
infer successful SENSOR staging merely from the presence of a `done` stamp.

## Alarm words

`demand` is the native demand grid stamp. `entry` is the existing first-C IRQ
entry stamp used by Gate1. `sequence` is next_apply before the alarm. `ready`
bit0 is current ready, bit1 is one-lookahead ready, sampled before the existing
alarm reducer. Other bits are zero. `reason` is the session reason afterward;
`done` is a real TIMER read after alarm/output queue work. The failing alarm's
completion fields can finish after first-fault ring allocation freezes.

## Parser and test requirements

- Verify exact version, field widths/counts, count bounds, row order, matching
  D1 identity/fault and CRC. Reject duplicates, changed repeats, unexpected rows,
  missing footer and truncated blocks. Partial/missing diagnostics are
  INCOMPLETE diagnostic evidence, never a fabricated transport PASS.
- Keep D1 faults and original verdicts. These rows cannot cure missing cohorts,
  lost packets, a real underflow, UART faults or an unqualified clock.
- Interpret TIMER arithmetic modulo32 bits. Do not sort raw TIMER values across
  wrap; serial/row order is canonical. An unfinished record is not a bad CRC.
- Tests cover64-entry wrap, first-fault retention, a decode spanning the fault,
 16-entry post tail, overflow count, sealing, chronological alarms, CRC and
  once-only printing. Real runtime tests require D1 then RX block then stack
  line and retain a rejected post-fault SENSOR. Synthetic fixtures are not HIL.

## Image and verification record

This is the original priority16 diagnostic checkpoint. The subsequent
[priority9 change](PIL_GATE1_PRIORITY.md) preserves this schema and fixtures;
use the current image pin in the [build handoff](PIL_GATE1_BUILD.md).

Original UP diagnostic image:248320B at
`build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2`.
UF2 SHA256: `71cf7cb2b3bd51fbd798a6d6c7a8e3d6543c0d9fd6ca129ac5d5eba1dc248b4a`.
ELF SHA256: `0e2767cb8e7465d11ebbfa25cbf6ea4b6a24f338d9523fbdf087061ed3a4fdbd`.
Retained artifacts: `build/pil-gate1/artifacts/rx-timing/`.
The [build handoff](PIL_GATE1_BUILD.md) retains the exact restricted build flags.

- RED: real runtime test required the missing ASCII block and failed before
  implementation (`build/pil-gate1/rxlog-red.log`).
- GREEN:586/586 strict GCC and586/586 Clang ASan/UBSan host tests. Includes all18
  UP/split/control runtime scenarios plus ring/CRC tests. Logs are
  `build/pil-gate1/rxlog-{gcc,clang}-{build,tests}.log`.
-45 strict ARM units pass:15 SIM/Gate1 units across UP/split/control, including
  the new module. Frames are in `build/pil-gate1/rxlog-strict/`.
-38 ordinary profiles and SDK link proofs pass. All76 literal ELF/UF2 hashes,
  including debug data, match retained baseline451e562 at the same public path:
  `build/pil-proof/base/hashes.txt` equals `gate1-rxlog/hashes.txt` there.
  The Gate1 SDK probe script itself does not recognize the Gate1 profile suffix;
  do not confuse that tool limitation with the38 successful ordinary proofs.
- Full forced UP rebuild gives the same hashes as the incremental build.
  Native application/template C, Gate1 priority16, physical providers, real
  kernel/alarm0/watchdog, live UART and stopped D1 binary format stay unchanged.

| Storage | Before | Diagnostic | Delta |
| --- | ---: | ---: | ---: |
| Runtime g |9112|12744|3632B|
| Painted worker stack |2048|2048|0|
| Native-line timestamp |4|4|0|
| Added writable storage |11164|14796|3632B|
| UP whole-image data / bss |696 /136444|696 /140076|3632B|

The log itself is3624B, the private optional fault observer adds8B. Compile-time
assertions enforce log<4096B and total added storage<=16384B. UP text is123236B.
Reviewed additional IRQ maxima remain below256B: UP196B, SMP/control240B.
UART frame72B is unchanged; native hook grows24->40B. The deepest reviewed
worker chain grows888->896B (service432->440B); the stopped atomic row printer
has an80B caller and160B row frame, with no IRQ formatting. These are C call
chains, not measured target reserve or IRQ latency. Read the new stopped stack
line on the operator run; old-image measurements do not qualify this image.

Actual C-produced fixtures, all under `build/pil-gate1/`:

| Fixture | Purpose | SHA256 |
| --- | --- | --- |
| `rxlog-synthetic.txt` |80P/16A, ring wrap, fault-spanning decode, post-tail loss6|`8701fc51749a4eaf686fa8da84d9ca0c7b7d3d233872026b40261ee6e9530a98`|
| `rxlog-missing-d1.bin` |real runtime D1 plus ASCII/stack, missing lookahead and rejected post-fault SENSOR|`3b11631a8f561df600a8b1e784fedc243a7e5b3333207e9416971d482a110e04`|
| `rxlog-runtime-d1.bin` |real runtime controlled short stop plus ASCII/stack|`e27320790310ba1ff99b446a1d3f20c541595040e75093cfbd3404814909f1b3`|

The PC owner independently checks these fixtures and owns the parser. No target
access, serial access, flashing or push is part of this checkpoint. The new
image is for an authorized diagnostic run; it is not a600s transport PASS.
