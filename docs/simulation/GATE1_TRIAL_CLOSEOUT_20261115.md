# Gate1 trial closeout — 2026-11-15

## Verdict and decision

**TRIAL INCOMPLETE / NOT GATE 1 PASS.** Physical Gate1 is NOT accepted.
The operator stopped Gate1 timing-rule changes after attempt22. Preserve each
run's original FAIL, policy, raw evidence and diagnostic files. Do not relabel
historical runs under later rules, or call the final relaxed trial standard
Gate1 qualification. Work moves to Webots planning; this is not authorization
for hardware use or implementation of a new timing contract.

## What the campaign established

- Physical Pico/Probe link and frozen-v1 protocol exchanges were exercised:
  BOOT identity, nonce/session checks, CRC/order checks, SYNC clock cones,
  source/output traffic and fail-closed stop paths. Clock qualification was
  checked for actual captures; this is not a future clock/delay bound.
- Instrumentation separated sleep delays from CPU-accounted receive work and
  retained USB/RAW evidence, original failures and reset identities. C4's table
  CRC remained byte-equivalent; its speedup did not remove all host stalls.
- Reset-B found an integrity-checked HardFault record with an invalid frame:
  status3, SPffffffe0, EXC_RETURNfffffff1, IPSR3. No usable PC/LR frame existed.
  This is stronger than the unchecked entry marker alone, but the checks are
  not cryptographic and do not prove the precise source instruction.
- The tested Gate1 fix was a private-stack, always-ready application-idle task,
  not a general repair/proof for every kernel idle path. Firmware72239a9a,
  UF2 SHA256 `6e8ac6c58275edbc8c5206810c71fd80aff3e19f00a4a523bf940ca4edb15258`,
  had no observed reset in attempts21/22; both ended through host policy checks.
  The reported idle stack high-water was340/512 bytes. This observation is not
  proof that no future fault/overflow can occur. Raw task IDs can change; TID0
  can be a transient PendSV null task, not proof of kernel-idle execution.
- Representative accepted source-phase distributions were about p50=50us,
  p99=119us and p99.9=135us. Attempt21 specifically reported47/118.5/134us over
  its38.8s run. These are measured distributions, not maximum-delay bounds.

## What was not established

No600s qualified Gate1 cohort completed. Stock Linux/Python/USB/Probe future
latency remains unbounded by this evidence. Small percentiles do not exclude
rare delays. Attempt22 rejected a source phase1431us (about1.4ms), above P600.
This is an observed source-phase error, not a claimed maximum of all host stalls.

Physical age/deadline, lease, queue and readiness checks were never waived.
Exact D1 upper AND lower age proofs remain required; a permitted late phase,
clean Pico counters or a completed trial would not replace those proofs.
Native availability3ms, pipeline4ms, expiry5ms and J250 remain unchanged.
`physical_gate1_pass` and standard `transport_subset_qualified` remain false.

## Final attempts and short attribution

Evidence directories under `build/webots/` in Main's run worktree:

| Attempt | Directory | Outcome |
| --- | --- | --- |
| 21 | home-gate1-trial-20261115T080329Z | 38.8s/38780 slots; first late38779 phase441us, then38780 phase371.5us failed the then-current consecutive rule. |
| 22 | home-gate1-trial-20261115T081429Z | About7.5s; tolerated6844 phase307.5us and7524 phase254us;7525 phase1431us failed P600. Pico clean stop, no reset reported. |

Attempt21 was not one continuous two-slot stall: a requested50us sleep took
956us wall/19.427us thread CPU, then the already-captured loop had374us
receive wall/375.668us CPU plus logging/TX and normal sleep. Its next slot was
late as well. No nearby GC or same-loop SYNC TX was observed.

Attempt22 seq7525 was predominantly CPU-accounted receive/logging, NOT a1.4ms
sleep overshoot. Loop42867 receive RAW22656731643..22656732880 took1237us wall,
1191.655us thread CPU; all switch/fault deltas were0. Marked JSON serialization
accounted for675us wall inside that span. Select/read were26/24us; following
sleep was111us wall/9.599us CPU. Next-loop capture RAW22656733221..33224 had
phase1431us. No GC within20ms; last actual SYNC TX was about10.9ms earlier.
The span includes decode, validation and serialization: it does not isolate
CRC, a particular Python instruction, frequency effects or all interrupt cost.
The one-capture-per-loop guard stayed active; no catch-up or skipped source.

## T1 rule history — all explicit opt-in trials

See [the detailed T1 record](GATE1_T1_LATE_TOLERANCE_20261115.md).

| Revision | Rule and reason |
| --- | --- |
| Original T1 | P600, at most1 late per rolling1000 slots, never consecutive. Defaults remained P250/clock250/adaptive SYNC100ms. |
| After attempt19 | At most3 per rolling1000; still never consecutive. Attempt19 failed on two late captures121 slots apart, without Pico reset. |
| After attempt21, FINAL | At most2 consecutive; third fails. Rolling quota3 and P600 unchanged. Attempt22 then failed the cap, not the count. |

Every slot still needed no-early start/end under the same qualified B400 cones,
H<=400/L<=1000, exact per-slot D1 upper/lower, staged n/n+1 readiness, native
jitter, leases, queues and fault checks. The4650us conditional apply upper bound
and350us margin are not a delivery guarantee. Never amortize an age violation
across captures. Trial completion labels and exit2 were never physical PASS.

## Route if strict bounds become necessary: design E

Use a separately specified **Pico-owned timeline**: the Pico owns capture/apply
timestamps and deadlines; PC/Webots supplies bounded, explicitly identified
preloaded/model data under a new provenance/lease contract. Late or missing PC
data must have a defined inhibit/expiry policy. This requires its own design,
proofs, tests and operator approval; it is not a renamed Gate1 PASS or permission
to enlarge existing physical ages. Moving timing ownership does not itself
prove Webots/model freshness or bound Linux/USB delivery.

Next simulation work needs a separate approved scope and explicit simulation-
time versus wall-time labels. This closeout authorizes no new code or hardware.
