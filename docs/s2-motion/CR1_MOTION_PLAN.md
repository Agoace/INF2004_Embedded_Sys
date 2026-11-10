# CR1 motion inventory and plan

## Status

**Plan only; no motion source change.** Baseline: `cr1-baseline`
(`168d705`). See [the evidence record](../architecture/CR1_EVIDENCE.md).
Merge current `mtk3-staging` before the module round. Approval and hardware
acceptance are separate gates. Do not move to sonar before motion closes.

The old Tribunal values are not the current contract: the controller now
runs every 10 ms, uses permille duty, accepts wheel speed rather than a
six-command enum, and has a servo API wider than the scan's +/-30 degrees.
Do not apply the historical 9-bit speed or 8-bit percentage duty directly.

## Before: static allocation

These are linked map input sizes, excluding alignment and platform stacks.

| Motion symbol or group | Bytes |
| --- | ---: |
| mc | 152 |
| public observation board | 72 |
| sampled_counts | 8 |
| raw_count | 8 |
| Seven timing/counter words | 28 |
| Four kernel object IDs | 16 |
| last_servo hardware cache | 2 |
| previous_ab | 2 |
| watchdog_armed | 1 |
| **S2 total** | **289** |

The platform also reserves a 3072 B motion stack. Do not reduce it.
The 4096 B inactive bench stack belongs to platform, not S2, and is outside
this change. `mc_config_t` is 58 B: its static constant is in flash, but
`mc_t` also holds a RAM copy. Each `mc_wheel_t` is 36 B. Hardware output
`mc_output_t` is a 6 B transient. Public `motion_status_t` is 64 B; the
board adds time (4 B) and sequence (4 B), for 72 B.

There is no current motion message queue. Calls use `mtx_motion` with
priority inheritance. Readers use either that mutex or a one-attempt
atomic-word board. MQTT state is 76 B; it must not change in this round.

## Shared-access rule

Dual-core is not planned now; a later IMU move to core 1 must remain
possible. Each packed shared word must have exactly one writer. Publish
whole aligned 32-bit words or use the current sequence/word board and its
atomic barriers. Readers copy and unpack locally. Never use shared packed
read-modify-write under `DI`/`EI`; it protects only one core. Give separate
writers separate words, or use the ISR-to-task queue/board path. The
RP2040 has no data cache; preserve the existing board barriers.

## Proposed first substep: motion observation board

Keep existing public API signatures and decoded temporary structures.
Replace only the stored observation payload with explicit masks/shifts.
Only `motion_task`, at the end of a completed tick, writes this payload.
The immutable robot configuration supplies derived values. Do not change
PID arithmetic, PWM resolution, control period, fault behavior or MQTT.

| Stored field | Valid range | Bits | Writer; access method |
| --- | --- | ---: | --- |
| Counts, each | Signed 32-bit API count | 32 | Motion; whole board |
| Speed mm/s, each | -32768..32767 | 16 | Motion; whole board |
| Duty permille, each | -800..800 | 11 | Motion; whole board |
| Travel mm, each | Signed 32-bit API value | 32 | Motion; whole board |
| Heading cdeg | Signed 32-bit API value | 32 | Motion; whole board |
| Servo pulse code | 73 angle-derived pulses | 7 | Motion; whole board |
| Servo angle | -36..36 whole degrees | 7 | Motion; whole board |
| Servo settled | 0/1 | 1 | Motion; whole board |
| Motion fault conditions | Four independent bits | 4 | Motion; whole board |
| Watchdog armed | 0/1 | 1 | Motion; whole board |
| Tick count | Wrapping u32 | 32 | Motion; whole board |
| Tick work maximum us | u32 | 32 | Motion; whole board |
| Gap minimum/maximum us, each | u32 | 32 | Motion; whole board |
| Late/missed wakes, each | Wrapping u32 | 32 | Motion; whole board |
| Publication time us | Wrapping u32 | 32 | Motion; whole board |

Duty has 1601 possible values, so 11 bits, not 8. Servo limits come from
`400 * 10 / 111 = 36`, not the sonar scan range. Encode signed fields with
explicit offset or two's-complement masks; document unused codes and
reject them in codec tests. Do not use C bit-fields or packed unaligned
struct accesses. Set all unused storage bits to zero.

The current measured-speed function clamps to the full signed-16 range.
The 460 mm/s command limit does not prove the measured-speed bound: the
wheel can coast, slip, reverse, or receive an encoder fault. Preserve the
full existing range until a narrower application contract is approved.
Likewise, wrapping counts/time/counters retain 32-bit modulo semantics;
there is no accepted maximum run time/read outage that proves less is safe.
These are explicit unresolved range questions, not a claim that motion
has completed all minimum-bit work.

Keep travel, heading and servo pulse in the first category A commit.
The pulse has 73 reachable values, one per angle; store its table/code
index in 7 bits, without changing any pulse. Keep its code separate from
the angle code in A, even though they are redundant. A separate B5 package
can later remove these derived fields:

- Travel derives from the same count pair and immutable origin/config.
- Heading derives from count difference with the existing 64-bit formula.
- Servo pulse derives from angle with the existing rounded 111/10 mapping.

Motion initialization currently starts from zero sampled counts and never
resets counts. Freeze and test this invariant before removing origins from
any stored representation. Reconstruct with the exact existing rounding
and clamping rules, not a new formula. Keep all derivation outside the
publication critical path where possible; measure reader cost too.

Category A retains all fields: 458 logical bits (58 B, 60 B aligned),
plus the 4 B sequence. Board storage: **72 -> 64 B**, expected saving
**8 B**. S2 input allocation would be **289 -> 281 B** if nothing else
changes. Actual linked total may differ due to alignment. Public decoded
status stays 64 B. MQTT and queue sizes stay unchanged in this substep.

A later B5 commit could remove the derived values: 355 bits (45 B, 48 B
aligned), plus sequence, for **64 -> 52 B**, another **12 B**. Do not
combine these changes or count the 12 B as minimum-bit packing. Both
figures are proposals, not measured after results.

## Persistent controller and ISR inventory

The first substep does not complete CR1 for the stored controller. Review
these fields before closing the motion module. Wider local arithmetic and
kernel object IDs are not packed application storage. The general pure
controller accepts other configurations; do not silently restrict its
public contract to the robot constants.

| Persistent field | Robot range/meaning | Bits or action |
| --- | --- | --- |
| target_q8, setpoint_q8 | +/-14618 | 15 each |
| integral_q8 | +/-102400 | 18 |
| last_count, origin | Signed 32-bit counts | 32 each, or derive origin |
| speed_cpt | Current clamp: signed 16-bit | 16; measured bound open |
| duty | +/-800 permille | 11 |
| kick_travel | 0..32777 at completion | 16, or store remaining |
| kick_ms | 0..260 in 10 ms steps | 5 tick units |
| still_ms | 0..500 in 10 ms steps | 6 tick units |
| kicking | 0/1 | 1 |
| command_ms, last_tick_ms | Wrapping ms | 32 each |
| servo_settled_ms | Deadline in wrapping ms | 32 |
| servo_deg | -36..36 | 7 |
| servo_us | Derived from servo_deg | B5 removal, separate commit |
| faults | Four conditions | 4 |
| has_command | 0/1 | 1 |
| cfg | Immutable robot settings | B5 duplicate-removal candidate |
| raw_count per wheel | Signed 32-bit API counter | Keep modulo width |
| previous_ab per wheel | 0..3 | 2 each |
| sampled_counts per wheel | Signed 32-bit | 32 each |
| watchdog_armed | 0/1 | 1, separate owner word |
| last_servo | Cache of derived hardware pulse | Prefer angle code |

`kick_travel` can reach 9 + abs(-32768) before the completion check, not
just the 10-count threshold. Timer bounds must include the final increment
before timeout, not just the configured limit. The tick-unit proposal
applies only to the fixed 10 ms robot configuration. Target bound:
`460 * 12414 * 10 * 256 / 1000000`, truncated as today. Do not round it.

Writer/access review:

| Field group | Current writers | Required change before packing |
| --- | --- | --- |
| raw_count, previous_ab | Encoder IRQ on core 0 | IRQ-only whole words |
| sampled_counts, timing | Motion task | Motion-only whole words/board |
| Controller state | Motion and API caller tasks | Single owner required |
| watchdog_armed | Startup task | Own whole word; no IRQ bit sharing |
| last_servo | Existing hardware write path | Resolve owner first |
| TEST probe/window flags | Runner and motion | Separate per-writer words |

Existing `DI`/`EI` count copies are a core-0 hardware contract, not a
cross-core lock. Do not combine those counters with task-owned packed
flags. Fix signed-overflow/wrap behavior with unsigned modulo operations
and explicit signed conversion tests before claiming wrap correctness.
Do not narrow counters without approved bounds on count rate and read gap.

For controller packing, choose one design after the board substep:

1. Make motion the only persistent-state writer. API callers send bounded
   immutable command requests; motion processes them and acknowledges.
   STOP/ESTOP must retain synchronous zero-PWM completion and error policy.
   Calls must not wait for the next 10 ms tick. Define bounded queue and
   caller lifetime rules before implementation. Multiple API caller tasks
   must not share a request word that they modify independently.
2. Retain the current API and do not pack its multi-writer persistent
   state yet. This is a documented partial CR1 result, not full acceptance.

A robot-specific compact controller could use about 442 logical bits
(56 B aligned), instead of the 152 B `mc_t`, if the table bounds hold,
origin is retained, and duplicated configuration/pulse are removed. That
suggests **96 B combined gross saving**, but it mixes A packing with B5
duplicate removal. Separate and remeasure these packages before approval.
It also excludes new request/ack storage, code state and alignment; it may
not save RAM after queues are added. Do not count it as a measured saving
or implement it without another approval. Retain the generic core as the
reference calculation if a robot-specific stored-state adapter is used.
Do not add this larger ownership redesign to the first board change.

Shared system fault allocation belongs to the later S1 round. Preserve
all four motion conditions now; no separate task may edit bits in the
motion-owned published fault word. S1 will combine per-owner words into
the shared fault mask as its sole writer.

## Tests and acceptance

Before and after each approved motion substep:

1. Run strict motor controller and motion task tests, then all host tests.
2. Exhaustively round-trip duty -800..800, angle -36..36 and all 16 fault
   combinations. Test signed speed extremes, count/time wrap, reserved
   codes, unused-bit zeros and null/busy/unready unchanged-output rules.
3. Compare decoded board output with the baseline controller for forward,
   reverse, stop, ESTOP, clear, stale command, kick, stall and servo moves.
   Include odd signed rounding and asymmetric wheel counts.
4. Inject publication/reader interleavings. Readers must get a coherent
   copy or unavailable, never torn values and never a retry loop. Check
   that only motion publishes. No shared packed RMW or libatomic helper.
5. Build the MQTT image with public fixtures. Run both size scripts;
   compare board, module, total RAM and message sizes mechanically. Build
   TEST and BENCH too. Check compiler stack frames, not only static RAM.
6. Run documentation/template checks (count stays eight), line-length and
   diff checks. No template config, other module source or secrets change.
7. Operator hardware gate: lifted-wheel forward/reverse/stop/ESTOP/servo,
   then approved floor run. Record motion gap/work, late/missed wakes,
   watchdog faults, all named stack margins and loaded MQTT/USB behavior.
   Compare with same-profile baseline under the same load. No new missed
   deadlines, stale/fault behavior, or loss of required motion precision.
   Agree a numerical work-time regression limit before the run.

No flash or push is authorized. Hardware and final timing acceptance are
**Not run**. Approve the small board substep first; resolve the persistent
state ownership/range questions before declaring motion complete.
