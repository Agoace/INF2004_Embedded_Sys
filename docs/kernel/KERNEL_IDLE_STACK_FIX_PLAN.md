# Kernel idle stack repair — draft plan, NOT approved for implementation

## Approval boundary

This document is a proposal only. Current approved Gate1 firmware remains the
[app idle A/B image](../simulation/PIL_GATE1_IDLE_STACK.md),72239a9/6e8ac6c5.
No kernel, template, native task or firmware change is authorized by this plan.
Main must obtain separate operator approval before implementation or target work.
SW7 reviewed this plan and confirmed the scope, guard/layout distinction and
68B context alignment reasoning. That technical review is NOT implementation
approval. Coding workers do not open ports, flash, halt a target or attach a debugger.

## Evidence and problem

[Attempt20/21 and the complete stack budget](../simulation/PIL_GATE1_IDLE_STACK.md)
show why the256B UP kernel idle stack is insufficient for the reviewed workload:

- Attempt20 retained a checked Handler-mode HardFault with invalid MSP/frame.
  It did not retain a usable fault PC or the exact corrupting instruction.
- Source and ELF review found a360B complete private-idle/PendSV/alarm path.
  PendSV can be interrupted after saving SSP but before masking interrupts.
  Its nested handler can overwrite the idle TCB directly below the stack,
  including that saved SSP. A later restore can load an invalid MSP.
- Attempt21 used the approved512B application idle substitute. The operator
  observed no reset over38.775476s and a340B app-idle paint high-water, above
  the old256B reservation. The worker used1164/2048B. PC consecutive-late policy
  stopped the trial. This is supporting A/B evidence, not600s acceptance or
  proof that every possible fault is removed.
- SW7 independently reviewed the stack layout and USB/SysTick/PendSV overlaps.
  The robot's always-READY idle_meter task can hide private-idle defects.

Two existing definitions couple unrelated reserves:
`config/config.h` sets CNF_TMP_STACK_SIZE to256UP/1024SMP; `kernel/tkernel/task.c`
sets KNL_IDLE_STACK_SIZE to TMP_STACK_SIZE. The temporary dispatcher stack and
an interruptible Thread-mode idle stack need separate bounds.

## Preferred minimal repair

Subject to approval and the full profile review below:

1. Introduce a separate configured idle-stack size. Candidate defaults are
   UP512B and SMP1024B. Retain current TMP sizes; do not grow both arrays as
   a side effect.512 is a reviewed Gate1 candidate, not a universal size proof.
2. Align every private idle stack object and its isstack/end to8 bytes; require
   size/stride multiples of8. Keep the existing68B startup context: saved SSP
   is then4mod8, software-pop36 reaches an8-aligned hardware frame, and the
   hardware-pop32 reaches an8-aligned C entry. Do NOT force saved SSP to8-byte
   alignment or change assembly context offsets to achieve this.
3. Keep normal idle behavior and task ID0. Do not add a public idle task to
   ordinary builds or change priority, affinity, WFI, watchdog feeding or tasks.
4. Make an8B low-side guard mandatory, in the SAME8-aligned allocation object
   as each stack, for example `{ UW guard[2]; UW stk[N]; }`. Initialize each
   core's guard before idle publication; verify real guard loads in diagnostics.
   Specify guard-check timing and failure handling before implementation approval.
   A canary detects writes; it is NOT a memory barrier and cannot stop overflow.
   Place ALL private idle TCBs above ALL idle stack/guard storage, with ELF/map
   assertions, so normal downward overflow does not first hit a saved idle SSP.
   This does not protect all other memory against arbitrarily large corruption.
   Retain paint and the used API; assert size, alignment, stride and non-overlap.
   Never print or call task APIs from a fault/IRQ solely to inspect paint.
5. Record and justify every shared-template/config deviation. The current
   ledger has8 entries; update affected entries and add any new path as required.

Likely edited paths: `config/config.h`, `include/sys/knldef.h`,
`kernel/tkernel/task.c`, host/kernel tests, and the template-change ledger.
Confirm actual declarations and map effects before editing. Keep public task
control-block layout and assembly context offsets unchanged.

## Explicit non-goals

- Do not combine this repair with the RP2040 NVIC byte-write/priority defect.
  Correct priority writes would permit new nesting combinations and require a
  new stack budget. Current360B reasoning assumes external0 and SysTick/PendSV3;
  only operator word reads can confirm actual target priorities.
- Do not treat moving DI earlier in PendSV as a substitute for adequate stack.
  It can reduce one dangerous overlap but leaves other handler use. A dispatch
  masking/SSP-validation change needs its own UP/SMP locking, latency and context
  proof and separate approval; it is not part of the preferred stack-only patch.
- No watchdog change, fault-check relaxation, task suppression, automatic rearm,
  IRQ staging, UART/D1/RX1 change, scratch write or persistent flash record.
- No claim that all other task stacks are safe. Every interruptible task needs
  room for its own deepest call path plus permitted exception/dispatch overlap.

## Required stack and RAM review

For EACH enabled IRQ and profile, sum frames that can coexist, not unrelated
branch maxima. Include hardware32B frames, possible4B alignment, HLL wrappers,
PendSV36B software context, interrupted task depth and enabled callback chains.
Check PRIMASK and actual priority levels before allowing a nested combination.
Resolve indirect callbacks from that profile's enabled classes/providers.

| Reviewed Gate1 case | Bound |
| --- | ---: |
| private idle plus normal alarm |288B|
| idle plus PendSV-window alarm |360B|
| idle plus SysTick/alarm |348B|
| idle plus PendSV-window USB |300B|
| SMP idle plus PendSV-window alarm, provisional360 +96 |about456B|

SW7's SMP review adds tk_wup_tsk's knl_smp_reschedule_all32B,
knl_ready_queue_assign56B and core_mask8B:96B beyond UP. Its robot split
alarm chain is172B versus76B on UP. Core IPI paths add72/80B and timer-handler
execution has indirect callbacks that MUST be resolved before a final bound.
The provisional456 x1.25=570B fits1024B; this is not a complete SMP proof.
SMP PendSV switches to TMP before the72B knl_smp_dispatch_release chain, so
that chain needs a separate TMP bound rather than being added to idle usage.

Candidate UP margin:360 x1.25=450B, rounded to512B. Recompute after alignment,
configuration or compiler changes. Review all38 ordinary profiles, including
radio/network, ADC, TEST and bench variants; retain SMP1024 unless a larger
minimum follows from its actual chains. Review TMP independently, especially
boot and cross-core release; changing idle size does not prove TMP sufficient.

For the worker,896B isolated review plus roughly344B additional overlap is
1240B, below2048B. Attempt21 paint1164B is consistent with overlap and leaves
884B observed untouched. Other native task headroom still lacks this complete
proof. Obtain existing per-task paint reports in a separately approved HIL run;
measurements support the static review but do not establish worst-case bounds.

UP idle256->512 adds256B plus mandatory8B guard and any alignment/layout padding;
SMP adds8B per core plus any padding if its1024B stack size stays unchanged.
The robot's priority32 idle_meter is always READY, so its normal task selection
is unchanged, and a robot-image pass does NOT exercise private idle. SW7's robot
map has about57KiB free, but verify every final map rather than reuse that value.

Do not silently change the Gate1 budget baseline: current added storage is16160B,
leaving224B under16KiB. If charged to that baseline, keeping both idle versions
and adding256B kernel stack +8B guard exceeds the cap by40B before padding.
Removing the512B app substitute instead gives15912B before other costs. Main
must approve shared-kernel accounting and the validation image; no implicit waiver.

## Implementation and validation gates

### Gate A — freeze evidence and baselines

Keep attempt20/21 captures, exact images, current maps, stack frames and all76
ordinary hashes. Preserve the A/B app-idle image for comparison. Record compiler,
SDK, IRQ assignments and template ledger state. Review the proposed file list
and RAM accounting with Main/SW7 before implementation approval.

### Gate B — RED tests, then the smallest repair

- Prefer deterministic RED from the current ELF: reviewed prologue/call-chain
  bound360 > idle allocation256. SW7's `build/wip/p2/stackdepth.py` is a candidate
  read-only analyzer; approval is needed before committing tooling in scripts/.
  Resolve/report every indirect branch and callback target; pair the result with
  a compile-time minimum-size assertion. This is a static bound, not execution
  proof. A modeled push/canary test can supplement, not replace, ELF review.
- Compiled tests must check configured sizes/8-byte alignment on UP and both
  SMP placements, initial stack-frame bounds, paint scanning and idle task ID0.
- Check scheduler behavior with no public task ready, a wake from IRQ, repeated
  idle/task transitions and the context-save interrupt window. Include timer
  wrap and empty/no-op callbacks. Keep assembly context offsets unchanged.
- Implement only the approved idle-size/alignment change. Run all existing host
  tests plus GCC/Clang sanitizers and strict ARM checks for affected units.
- Review actual ELF symbols, context instructions and all permitted IRQ paths.
  Recheck every memory/stack budget, map, template entry and documentation link.

A kernel repair intentionally changes ordinary ELF/UF2 images. Do NOT promise
76 old hashes MATCH: retain the old ledger, produce76 new hashes, explain each
expected change and obtain approval before accepting the new baseline. Frozen
wire codec/D1 fixtures and unrelated protocol behavior must still match.

### Gate C — separately approved operator A/B validation

The kernel fix must be tested with the app idle substitute absent; otherwise
that READY task hides whether private kernel idle is safe. Removing only that
approved diagnostic substitute is a new explicit validation scope, not license
to suppress native tasks. Use a labeled image with a checked RAM budget.

Also account for the robot's always-READY idle_meter-style substitute: an ordinary
robot pass still cannot validate private idle. Any dedicated image without that
substitute requires separate approval; do not silently disable it.

Report PRIVATE kernel idle paint/canary and all native task stack observations.
Do not relabel the current app-idle suffix as kernel idle without a new explicit
schema/image identity; coordinate any versioned console report with the PC owner.
Keep RESET1/BC1 and original failures. Compare app-idle and repaired-kernel-idle
runs under the same transport workload, then require the full accepted soak and
fault/stop campaigns before retiring the workaround.

Optional causal diagnosis needs separate operator approval: halt at the original
flash HardFault handler after the RAM breadcrumb stub, before reset erases BSS,
and inspect idle TCB/SSP/paint, current/scheduled slots and registers. A watchpoint
on a normally immutable idle-TCB tail word can identify an overflow write. Debug
halts perturb deadlines and are NOT qualification runs. Do not inspect a fresh
post-reset TCB and mistake its reinitialized values for fault-time evidence.

## Ownership and approval checklist

Main owns integration and explicit scope/baseline approval; SW7 reviews shared
kernel/UP/SMP behavior; SW6 reviews Gate1 stack/image effects; the PC owner reviews
console/capture changes. Only the operator performs target trials.

Before implementation, approve: configured sizes and margin, alignment/canary
scope, ordinary-hash baseline changes, Gate1 RAM accounting, removal of the app
substitute for validation, report format, target campaign and acceptance criteria.
Until then this remains a plan only; the shared kernel/template is unchanged.
