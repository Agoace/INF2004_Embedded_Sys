# Gate 1 — TX FIFO startup correction

## Operator attempt 1

Main reported bootstrap timeout: no BOOT within5s. The operator verified the
previous UF2 hash `c7df4c0b...`, rebooted, then opened capture/soak about60s later.
Evidence is in Main's checkout under
`build/webots/home-gate1-20261113T144041Z/`. Coding workers did not access a target.
This attempt is FAIL, not a transport or sensor PASS.

The stopped D1 has session0, nonce`0382b6b1`, reason264=`0x108`/TX queue full:

| Field | Exact meaning |
| --- | --- |
| COUNTS[7]=8 | Eight TX queue mutations: eight BOOT enqueues, no dequeue |
| COUNTS[9]=1 | One TX queue overflow |
| TRACE kind5 | Internal fault event |
| TRACE/CONTEXT flags6 | LINKFAULT2 plus INHIBITED4; not DISARM reason6 |
| CONTEXT coverage1423=`0x58f` | Base`0xf`, internal fault`0x80`, raw queue overflow`0x100`, frozen`0x400`; drained`0x10` absent |
| Fault TIMER1130785 | Stop at about1.13s, before any negotiated session |
| Freeze TIMER1250720 | UART drain timed out before the summary froze |

This is not a boot-without-HELLO timer and not evidence of RX framing/noise.
A closed Probe port cannot apply UART flow control; this image uses no CTS/RTS.

## Cause and correction

Enqueue only enabled PL011 TXIM. An initially empty FIFO does not provide the
required TX threshold transition. With no RX IRQ to run the shared handler,
BOOT packets remained in the software queue until its eighth slot filled.
The previous host double incorrectly generated TX IRQs merely because TXIM was
set. It could not expose this hardware startup condition.

The corrected enqueue holds core0 IRQ exclusion and directly transfers up to32
bytes with the same FIFO consumer used by the IRQ. TXIM continues the transfer
on threshold transitions. Priming also runs when old bytes are still shifting:
a late enqueue must not depend on an already-passed transition. Queue indices,
FIFO-loaded cohort counts and depth extrema use the same serialized consumer.
No wire, epoch, lease, capability, native/template source or hardware setting
changes. There is no new requirement to start the host within N seconds.

## Regression evidence

- The PL011 register/FIFO double leaves IRQ inactive when TXIM is enabled on
  an initially empty FIFO. RED failed before any RX or manually forced TX IRQ.
- GREEN requires the first32 BOOT bytes to be loaded directly. It also checks
  enqueue while an old packet is still shifting with TXIM off, plus20 BOOT
  intervals without HELLO and without TX overflow.
- GCC585/585 and Clang ASan/UBSan585/585. All18 runtime profile/scenario cases
  use this bounded32-byte FIFO/threshold model, not an always-writable sink.
- All38 ordinary builds/SDK proofs pass; all76 ELF/UF2 hashes equal451e562 at
  the retained public path. All42 strict ARM unit compilations pass.
- UP is rebuilt. Static RAM is unchanged11164B added; FIFO consumer frame48B.
  The IRQ wake branch remains240B on SMP, because the consumer returns before
  wake. Its metric-fault branch is196B, below the same256B target. The888B
  reviewed worker maximum is unchanged. Target reserve still needs a new run.
- New runtime synthetic D1:3165B/35 records,11staged/8applied/3cancelled,
  SHA256`50cb2742f70bc9b707b0cdec24e6d144f10297a5a4f2bfcb523f75aba5c189a8`.
  It is not hardware evidence. Logs are `build/pil-gate1/prime-*`; ordinary
  identity evidence is `build/pil-proof/gate1-tx-prime/`.

## USB check and operator action

D1 does not use PL011 or TXIM. `tm_snd_dat` appends a whole record to the USB
ring. The existing USB service task drains it, calls write/flush, and has an
independent10ms polling fallback even when no USB event occurs. Attempt1's
complete D1 and worker-stack line (1060/2048 untouched, on the old faulted
image) also demonstrate that path started correctly.
This correction does not change USB code or its capture requirements.

Use the corrected image/hash in the [build handoff](PIL_GATE1_BUILD.md). The old
image is withdrawn. Only the operator reflashes/resets, which clears the latch
and produces a fresh nonce. Start separate CDC capture before negotiation/ARM,
then run the cap4 soak. No special reset-to-host deadline is imposed. A new
operator run is required; do not reuse attempt1 as a PASS.
