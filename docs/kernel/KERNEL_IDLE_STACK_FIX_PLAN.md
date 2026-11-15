# Kernel idle stack repair — draft plan, NOT approved for implementation

## Approval boundary

This document is a proposal only. Current approved Gate1 firmware remains the
[app idle A/B image](../simulation/PIL_GATE1_IDLE_STACK.md),72239a9/6e8ac6c5.
No kernel, template, native task or firmware change is authorized by this plan.
Main must obtain separate operator approval before implementation or target work.
Coding workers do not open ports, flash, halt a target or attach a debugger.

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
2. Align every private idle stack to at least8 bytes; require its size/stride
   to be a multiple of8. Verify the initial context and exception return keep
   the intended alignment. Keep the idle TCB outside the writable stack range.
3. Keep normal idle behavior and task ID0. Do not add a public idle task to
   ordinary builds or change priority, affinity, WFI, watchdog feeding or tasks.
4. Retain paint and the existing idle-stack used API. Add compile/link assertions
   for allocation size, alignment and non-overlap. A separate low-side canary
   and a normal-context diagnostic report are proposed for explicit approval;
   never print or call task APIs from a fault/IRQ solely to inspect paint.
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

UP idle256->512 adds256B plus any alignment/canary padding; SMP may gain padding
only if its1024B size stays unchanged. Do not silently change the Gate1 budget
baseline: current added storage is16160B, leaving224B under16KiB. If the kernel
increase is charged to that baseline, keeping both idle implementations would
exceed it by32B even before padding. Removing the512B app substitute while
adding256B kernel idle is15904B before other costs. Main must approve how the
shared-kernel budget and the validation image are accounted; no implicit waiver.

## Implementation and validation gates

### Gate A — freeze evidence and baselines

Keep attempt20/21 captures, exact images, current maps, stack frames and all76
ordinary hashes. Preserve the A/B app-idle image for comparison. Record compiler,
SDK, IRQ assignments and template ledger state. Review the proposed file list
and RAM accounting with Main/SW7 before implementation approval.

### Gate B — RED tests, then the smallest repair

- A deterministic model/canary test must fail with256B under the reviewed
  dispatch/IRQ depth. Distinguish modeled pushes from executed ARM instructions.
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
