# Gate1 app idle task — approved A/B stack diagnostic

## Why this image exists

Attempt20 on Reset-B imagef0d5e550 reset after about18s. Its reopened BC1
matched failed owner noncecb00b155 and had mask3f: a checked HardFault record,
status3, unreadable frame, SPffffffe0, EXC_RETURNfffffff1, PRIMASK0, IPSR3/core0.
Motion completion-observation preceded the fault by2892us; PIL BEFORE_SLEEP by
142us; the last alarm0 stamp by474us. The positive checked fault record is much
stronger evidence than watchdog reason alone. No fault PC was recoverable.
The feed stamp is not the exact hardware reload time; its age is a lower bound.

Under normal exception entry, f1 names a Handler-mode/MSP return and ffffffe0
is consistent with subtracting32 from zero. Failed stacking/unstacking and
corrupted context prevent claiming the exact previous instruction or SP value.
Task ID0 can mean private idle OR the dispatcher briefly cleared ctxtsk.

Read-only source/disassembly review by SW6 and independent SW7 review found:

- All tasks and interrupts use MSP. HLL enter/leave only increments/decrements
  task-independent nesting; it does not switch to a separate IRQ stack.
- The configured2048B exception reserve is boot/top-of-RAM storage, not the
  running handler stack. Runtime interrupts use the interrupted task's stack.
- The UP private kernel idle stack is only256B. On the f0d5e550 image it is
  [20001c94,20001d94); directly below is idle TCB[20001c28,20001c94).
  Its saved SSP is at20001c40,340B below the stack top. TCB offsets76..107 are
  wait/timer state, initial-stack pointer and mutex list. Idle does not use
  these fields, so limited overflow can be silent.
- PendSV saves36 software bytes and stores SSP BEFORE disabling interrupts.
  An external IRQ in that window can overwrite the saved stack/TCB. A later
  idle restore loads SSP unchecked, then `mov sp,r2` runs in Handler mode.
  This is a concrete corruption path, not proof of the exact event in attempt20.
- The last alarm ran before the last PIL marker. Earlier damage followed by
  PIL sleeping and a later bad idle restore is consistent with that ordering.
  The142us gap alone does not identify a dispatch or a particular ISR.
- The RAM fault stub only READS MSP/PSP; it has no SP write, push or C call.
  Its fixed stores cannot set MSP to zero. The image's register SP loads are
  the dispatcher; other register-add SP epilogues exist in IMU/vehicle task
  code, not the Gate1 IRQ/stub paths. Corruption or bad return control flow
  remains possible; the fault record does not exclude all alternatives.

Map notes apply to the OLD image: worker[20009008,20009808),2048B;
TMP[20003068,20003168),256B; initial SP20040000 and boot exception reservation
[2003f800,20040000). Below TMP are timer state; below idle TCB are the idletsk
pointer and semaphore table. Ordinary task stacks and heap are separate.

## Complete stack budget

These are complete overlap sums from the f0d5e550 machine-code prologues,
existing strict ARM frame records, and SW7's independent CDC-only TinyUSB call
review. They are not merely the earlier204B additional Gate1 IRQ-chain bound.

| Case on private idle | Conservative bytes |
| --- | ---: |
| idle16 + frame36 + HLL8 + native alarm40 + ACK8 + Gate1 hook40 + session alarm72 + OUTPUT48 + memset20 |288|
| PendSV window88 + external frame36 + HLL8 + native alarm40 + remaining Gate1 chain188 |360|
| SysTick outer76 + complete external alarm272 |348|
| PendSV window88 + external frame36 + HLL8 + USB chain168 |300|

The USB chain is dispatch8 + dcd_rp2040_irq64 + dcd_event_handler48 +
tu_fifo_write24 + memcpy24 =168. The endpoint continuation/reset/control,
memset and wake branches are smaller. CDC is the only enabled class; its SOF
callback is NULL, the event hook is empty, and queue interrupt callbacks are
skipped for in-ISR delivery. SysTick's deeper callbacks execute with PRIMASK1;
external IRQs cannot nest on those masked callback frames.

Priority assumption: requested external priorities are written with STRB by
EnableInt_nvic. RP2040 NVIC requires word transfers; SW7 observed all external
priorities0 on the robot through this path. The provisional budget therefore
permits external0 over SysTick/PendSV3, not UART-over-alarm nesting. The initial
SW6 UART-over-alarm scenario was withdrawn after this finding. Only an operator
can confirm the Gate1 NVIC_IPR words; this change does not repair priorities.
Do not reuse this bound after changing NVIC access/priority assignments.

Maximum360 x1.25=450B, rounded up to512B:152B above the conservative maximum,
more than the required25 percent. The new app idle entry is a zero-frame leaf;
the budget conservatively retains the old idle16B frame and alignment allowance.
Worker896 + about344B dispatch/IRQ headroom =1240B, below its2048B reservation.
Other native stacks are unchanged; this is not a new hardware measurement of
all their reserves. A later kernel idle/template repair needs separate approval.

## Approved A/B change

Only SIM=1 PIL_GATE1=1 includes this task. The Gate1 startup paints a private,
aligned512B stack, creates an application task at lowest priority32 and starts
it before the PIL worker/native startup. It loops on NOP without sleep, yield,
WFI, kernel calls or return. It stays READY, so normal running Gate1 dispatch
always has an application task instead of selecting private kernel idle.
Creation/start failure latches existing initialization fault10c; no worker is
started. Initial pre-application kernel startup is outside this guarantee.

All other task priorities, native task load, watchdog feeds/100ms period,
source-age/jitter/lease/fault checks and Reset-B records stay unchanged. This
explicitly approved extra idle task is an A/B diagnostic, not the template fix.
Busy idle does not establish a CPU-reserve measurement or a physical PASS.

The final stopped line now is:

```text
[PILG1] stopped; reset required; worker stack untouched N/2048 bytes; idle stack high-water U/512 bytes\n
```

N/U are decimal. The suffix is on the SAME line before LF, so capture cannot
stop before reading it. U is512 minus untouched paint bytes; it includes initial
context construction/observed overwritten bytes, not a certified worst-case SP.
Missing stopped output means no watermark evidence. RESET1,281-byte BC1, UART,
D1 and RX1 schemas are unchanged. BC1 may show new task IDs; do not assume the
old numeric IDs identify motion/PIL/app idle in this image.

## Tests and resources

- RED: the old image failed app-idle-created/started-before-native startup
  (`build/pil-gate1/idle-red.log`). The old ELF also fails the new idle-symbol
  requirement. Production ready-queue tests leave only the lowest task and
  check it remains selected, rather than an empty application queue.
- Host UP/split/control check priority32,512B, alignment/paint, core0 affinity,
  start and synthetic96B paint high-water. ELF proof checks the actual leaf
  NOP/back-branch bytes: no stack, calls, sleep, WFI or return.
- GCC614/614 and Clang ASan/UBSan614/614;48 strict ARM units. Reset-B fault-stub,
  Thumb vector and NOLOAD proofs stay green. Worker/service/startup frames stay
  32/440/56B. The app idle frame is0B.
- All38 ordinary builds/SDK proofs and76 literal ELF/UF2 hashes MATCH baseline:
  `build/pil-proof/gate1-idle/hashes.txt`. Shared template deviations remain8.
- Added RAM512B exactly. Reset-B remains688B; Gate1 added total16160B, below
  16384B by224B. Whole UP image: text124812, data872, bss141260B. Compile-time
  storage guard includes the idle stack and full800B Reset-B reservation.
- Actual C clean/missing/painted USB fixtures under `build/pil-gate1/` are
  `idle-runtime-usb.bin`5918B, `idle-missing-usb.bin`2709B and
  `idle-painted-usb.bin`5919B. Host fake stacks yield U0/0/96; they do not claim
  target watermarks. Independent PC parser review retains strict identity,
  D1/RX1/stack checks and range-checks the optional suffix.

## Image and operator handoff

`build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2`,251392B.
UF2 SHA256: `6e8ac6c58275edbc8c5206810c71fd80aff3e19f00a4a523bf940ca4edb15258`.
ELF SHA256: `0b0b8145f9849c3e24276aaa7bc86c9c9c9aba404bf46a1b80b024b465d1d010`.
Retained artifacts: `build/pil-gate1/artifacts/idle-stack/`.
Use the [restricted build/operator procedure](PIL_GATE1_BUILD.md), with matching
PC suffix parser/capture. Preserve failed attempt20 and distinguish this new
A/B image. Collect both boot records, complete diagnostics and the idle watermark.
Only the operator flashes, opens ports or reads target NVIC/stack state.
No hardware access, reset/fault injection or push was done by coding workers.
