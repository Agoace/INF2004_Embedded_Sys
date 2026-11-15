# Gate 1 transport image — build and operator handoff

## Status and limits

The image is host-tested and ready for an operator transport trial on a spare
RP2040 Pico. Attempt5 armed but failed on missing lookahead658 after657 applied
slots. Attempt6 localized delay to worker scheduling. Attempt7 had no measured
Pico fault but retained long RX-to-worker delays and stopped on a PC phase error.
The current image keeps [RX1](PIL_GATE1_RX_TIMING.md) and
[priority9](PIL_GATE1_PRIORITY.md), and fixes three Gate1
[IRQ dispatch requests](PIL_GATE1_IRQ_WAKE.md). It now adds an early
[raw reset-cause USB boot row](PIL_GATE1_RESET_CAUSE.md) and approved
[retained diagnostic B](PIL_GATE1_BREADCRUMBS.md) after repeated watchdog resets.
The current A/B image adds a [painted app idle stack](PIL_GATE1_IDLE_STACK.md)
after attempt20 captured a checked invalid-MSP HardFault.
It has no physical PASS.
Earlier startup failure/correction is recorded in the [TX startup record](PIL_GATE1_TX_PRIME.md).
This is NOT sensor/physics acceptance.
Only the operator flashes or opens target ports. Coding workers do neither.
Do not flash a plain Stage0 `_sim` image. Do not connect a robot or actuators.

Use the [design/status](PIL_GATE1.md), frozen [UART v1](PIL_PROTOCOL.md) and
separate [D1 contract](PIL_GATE1_DIAGNOSTICS.md). The PC-side owner supplies the
matching home runbook and capture/soak tools. Use explicit cap4 transport opt-in;
the default full-simulator capability requirement must stay0x1f.

## Exact image

From the repository root:

```sh
make -C build_make SIM=1 PIL_GATE1=1 SMP=0 FORCE_CORE0=0 \
  WIFI=none NET=none MQTT=0 CONSOLE=usb_cdc CRASH_REPORT=0 \
  TEST=0 BENCH=0 DIAG=0 IMU_STREAM=0 BARCODE_CAPTURE=0 \
  BARCODE_LIVE_ENABLE=0 JUNCTION_TEST_CMD=NONE -j4 --output-sync=target
sha256sum build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2
```

- File: `build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2` (251392 bytes).
- UF2 SHA256: `6e8ac6c58275edbc8c5206810c71fd80aff3e19f00a4a523bf940ca4edb15258`.
- ELF SHA256: `0b0b8145f9849c3e24276aaa7bc86c9c9c9aba404bf46a1b80b024b465d1d010`.
- Compiler: Arm GNU14.3.1, Cortex-M0+, `-O2 -g3`; Pico SDK2.3.0.
- D1 profile flags: `0x79` (UP, USB-only, no radio/live actions, SIM, Gate1).
- Retained copy: `build/pil-gate1/artifacts/idle-stack/`, with map/size/hashes.
- Capture frozen RESET1 then281-byte BC1, both tied to this boot's nonce.
  A CDC reader must not stop after RESET1; missing B is UNKNOWN.
- Appends versioned ASCII after D1 manifest, before the unchanged stopped stack
  line. Use the matching PC RX diagnostic parser; live UART and D1 v1 stay fixed.

Debug paths affect ELF hashes. Another checkout path can give a different ELF
hash. Record the exact UF2 used for each operator run.

## Wiring and first output

| Connection | Rule |
| --- | --- |
| Pico GP0 TX | Connect to Debug Probe U RX |
| Pico GP1 RX | Connect to Debug Probe U TX |
| GND | Connect common ground |
| Probe power | Do NOT connect |
| Pico USB | Separate USB CDC, not the Probe UART |

UART0 is921600 baud,8N1, binary only. Use the Probe UART's stable by-id path.
Identify the separate Pico CDC port as the operator; do not guess its tty number.
Start a binary-safe Pico CDC capture BEFORE UART negotiation/ARM. Do not run a
second reader on either port. USB terminal newline conversion is not permitted.

After completed native startup, nonzero boot identity and a valid alarm0 grid:

1. USB prints `[PILG1] transport-only cap4 UART0 921600 8N1; ...`.
2. UART emits BOOT at10Hz: session0, first sequence0, no-ACK, cap4, baud921600,
   schema`0x50494c01`. No successful initialization means no valid ready BOOT.
3. Accepted HELLO returns HELLO_ACK with a native-grid epoch at least100ms after
   its generation timestamp. ARM must name that latest ACK.
4. Negotiated OUTPUT runs at1kHz: flags4/INHIBITED and ACK`ffffffff` before ARM;
   flags20/ARM_PENDING and no-ACK after ARM. BOOT stops once pending.
5. SENSOR0 and1 must be ready before epoch. At epoch, the first successful apply
   gives OUTPUT flags1/ARMED, ACK0, then contiguous ACK1+. Missing/late input or
   a timing/lease fault latches inhibition. Sending ARM is not acceptance.

Neutral SENSOR validity is`0xC18`: zero encoder endpoints, released buttons,
invalid ADC/IMU/sonar. SENSOR peer ACK stays no-ACK. OUTPUT is observed by the PC,
never physics-applied. No physical PWM, trigger, encoder, ADC or IMU is emulated.

## Stop and verdict

- Use a planned601s PC run to obtain at least600s contiguous coverage.
- The PC first inhibits locally, stops new source production, then sends valid
  DISARM reason1(user) or3(exit), even if the session is stale.
- Drain the active OUTPUT cohort. Pico retains a final inhibited OUTPUT, drains
  its UART queue/FIFO/shift register (100ms limit), then freezes the summary.
- Pico sends D1 automatically and exactly ONCE on the already-open USB CDC.
  No GP20/request is needed. Keep capture open through MANIFEST and the following
  `[PILG1] stopped; reset required; worker stack untouched .../2048 bytes` line.
  USB backpressure can delay the dump; do not use the100ms UART limit as a USB
  capture deadline. A missing record/manifest is INCOMPLETE, never PASS.
- Require all26 counts,12 all-event extrema, exact contiguous stage/apply/OUTPUT
  cohorts, manifest CRC/identity, controlled tail<=8, zero faults, matching image
  flags and source capture/clock evidence. Availability upper<=3000us, actual
  source age upper<=5000us and jitter upper<=250us remain unchanged. Apply outward
  timer quantization once, separate from clock drift/source brackets.
- A plot, trace sample, mean or short replay cannot establish coverage or PASS.
  Fault, pause or reset requires a new boot nonce/session; never auto-rearm.

## Host/build evidence

The current [app idle A/B checkpoint](PIL_GATE1_IDLE_STACK.md) passes GCC614/614,
Clang ASan/UBSan614/614,48 strict ARM units,38 ordinary builds/SDK proofs and
all76 unchanged literal hashes. Its ELF proof checks retention, the Thumb vector
and18 interpreted RAM-stub paths; no target fault injection is claimed. The following bullets record the original
runtime checkpoint, not additional physical evidence.

- GCC585/585 and Clang ASan/UBSan585/585. The session test includes10000 continuous
  slots and TIMER wrap. HAL and option-guard tests were RED first. Runtime tests
  compile the actual task/session/stream/D1 sources against explicit single-thread
  kernel/MMIO doubles; these do NOT prove hardware IRQ latency or SMP exclusion.
- Runtime cases on UP/split/control: clean drain, independent lease IRQ, UART
  overrun, RX queue full, late progress and missing lookahead. Extra RED tests
  exposed repeated dumps and stale ARM receive-state mutation; both are fixed.
- Original checkpoint runtime short D1 fixture:3165 bytes,35 records/20 traces,
 11staged/8applied/3cancelled and8generated/enqueued/FIFO-loaded OUTPUTs.
  SHA256`69edc2e017d48f1b3bff42776660720ae5f5994e7733967f373b3ad863e89e5d`.
  Independent PC review passes structure/metrics. It remains synthetic8ms and
  INCOMPLETE, not a Pico result.
- All38 ordinary profiles and38 SDK link proofs pass without warnings. All76
  literal ELF/UF2 hashes equal baseline451e562 at the retained public build path,
  including debug data. Final source edits are SIM-only. Template deviations stay8.
- UP/split/forced-core0 Gate1 links pass. SIM0 -> Stage0 -> SIM0 restores native
  ELF/UF2 bytes; restoring Gate1 UP also restores its ELF/UF2 bytes.
- All42 strict ARM compilations pass (14 SIM/Gate1 units x3 placements). Native
  clock/IRQ/kernel/console objects also have frame records with unchanged flags.

## RAM, stack and ownership proof

The table and frame values below are the original runtime checkpoint record.
The RX diagnostic adds3632B, RESET1 adds164B, and retained B adds688B:
the app idle A/B stack adds512B: total added storage16160B,
UP `.data`872B and `.bss`141260B,
service440B, native hook40B. The current UART frame is80B. Reviewed maxima are
204B UP /248B SMP added IRQ chains and896B worker. See the
[app idle evidence](PIL_GATE1_IDLE_STACK.md) for current totals and test logs.
The current stopped line appends the512B app idle paint high-water on the same
line. Missing target watermark data is UNKNOWN, not a proved reserve. No new target reserve is claimed.

| Image | `.data` | `.bss` | Total static RAM |
| --- | ---: | ---: | ---: |
| UP native |696|125324|126020 B|
| UP Stage0 |696|125300|125996 B|
| UP Gate1 |696|136444|137140 B|
| Split/control Gate1 |704|140564|141268 B|

New writable storage is11164B: runtime`g`9112B, painted worker stack2048B,
and last native-line timestamp4B. A compile-time assertion covers all three.
Net UP increase from Stage0 is11144B: disabling sample SER startup removes20B.
The previous Stage0 omission of24B hardware state is not a functional saving.

ARM static frames: service432B, transmit192B, UART IRQ72B, lease IRQ16B,
native hook24B, session alarm72B, fail40B, trace12B, note24B, metric observer20B.
Trace writes owned storage directly; it has no64B automatic array/memcpy chain.

Reviewed additional IRQ chains, including kernel wake paths:

- UP maximum180B: ACK8 + hook24 + alarm72 + note24 + fail40 + trace12.
- SMP/control maximum240B: UART72 + wake16 + release8 + ready16 + reschedule48
  + ready-assign72 + core-mask8. Lock-enter and dispatch-request branches are
  smaller. PRIMASK exclusion is explicit; `knl_dispatch` only pends PendSV.
- These are additional C call chains, not a measured IRQ assertion latency.
  Existing hardware exception/HLL/native frames and the2048B exception stack
  are separate. The largest reviewed worker chain is888B: worker32 + service432
  + decode288 + validate16 + sensor48 + arithmetic72. The arithmetic term is
  ul-div thunk16 + divide48 + clz8 from actual binary prologues. Memcpy24 and
  memset20 are also included in the path review. Target painted reserve is NOT
  measured yet; record it from the stopped USB line.

Kernel owns IRQ20/UART0 and IRQ1/TIMER alarm1. Each UART IRQ transfers at most32 RX
and32 TX bytes; no CRC/decode/encode occurs in IRQ context. Enqueue also primes
up to32 FIFO bytes under core0 IRQ exclusion, without waiting for a TX edge. Service-duration
brackets end before their final metric update; they measure the service body,
not hardware IRQ entry/exit latency. Worker priority9 and TA_USERBUF stack
stay core0. Core0 IRQ exclusion protects session mutations; it
is not a cross-core lock. Whole-word SPSC publication uses ARM DMB.

Only Gate1 devinit.o force-includes the original config plus SER=0; ADC=1/IIC=0
stay unchanged. Only Gate1 drv_line.o aliases its time read; the helper returns
real TIMER and retains the original first-C-entry timestamp for the next ACK.
Native application/template C stays unchanged. Six SIM providers are unique;
UP shim definitions appear only on UP. There is no SDK startup/IRQ/time runtime,
PIO or physical action path. Existing USB SDK-compat names are kernel adapters,
not SDK IRQ runtime providers.

Retained evidence: `build/pil-gate1/cp3-*`, `cycle-*`, `strict/`, `artifacts/`,
`provider-proof.txt`; ordinary matrix in `build/pil-proof/gate1-cp3/`.
