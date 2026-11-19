# CR1 telemetry A, diagnostic faults and T110

## Scope and approval

The operator approved T110 option A, A-EVENT, A-INDEX, A-SCAN, A-HEALTH,
A-SONAR, A-INFO and diagnostic fault bits 0..9 on 2026-11-18. Category B
and MQTT wire changes are not approved. Native public value types, units,
identities, queue capacity and admission safety must stay unchanged.

| Step | State |
| --- | --- |
| T110 | Software checks pass; robot measurement pending |
| F1 shared interface | Software checks pass; no publisher or storage is added by the interface |
| A-EVENT | Software checks pass; robot timing pending |
| A-INDEX | Software checks pass; robot timing pending |
| A-SCAN, A-HEALTH, A-SONAR, A-INFO | Software checks pass; robot timing and stack pending |
| F2a STOP capture and Health combiner | Software checks pass; Vehicle call and final output integration pending |
| F final owners and USB output | Software and demo checks pass; combined robot test pending |

The starting revision is `33d2e4c`, which includes the approved sonar,
Vehicle and barcode A changes. The canonical common baseline remains
`7350da0`; SW7's existing map and size reports are authoritative. Do not
build a second common baseline.

## T110

Change only `PM_RUN_STATE_MS` from 200 to 110. Keep
`state_at = now + period`. This is a **110 ms deadline; expected gap
~120 ms with 20 ms poll; robot measurement pending**. It is not an exact
fixed-rate schedule. No Wi-Fi task or kernel scheduling change is made.

| Setting | Before | After |
| --- | --- | --- |
| RUN state deadline | 200 ms | 110 ms |
| IDLE state | 1000 ms | Unchanged |
| RUN / IDLE heartbeat | 1000 / 5000 ms | Unchanged |
| Live log / KR1 rows / Health | 100 / 20 / 1000 ms | Unchanged |
| USB dump yield | 20 ms per eight records | Unchanged |
| State payload | Binary, version 5, 76 B | Unchanged |
| Model gap with 20 ms polls | 200 ms | 120 ms |
| Model state payload rate | 380 B/s | 633.33 B/s |

`test_telemetry_mode` checks eligibility at 109/110 ms, timer wrap and
polls at 0,20,40... ms. The model sends at 0,120,240... ms. The real
`robot_mqtt` schedule test also checks 109/110 ms. RUN and grace tests
expect 18 publications in a 2000 ms window with fine polling. IDLE and
heartbeat expectations do not change.

Dashboard stale limits remain RUN state 1000 ms, IDLE state 2500 ms,
RUN heartbeat 2500 ms and IDLE heartbeat 12000 ms. Producer age 65535
still means unavailable. No decoder threshold is changed.

## F1 shared interface

`app_program/platform/fault_mask.h` defines one 8 B owner board: aligned
u32 sequence and u32 value, using the existing board atomics. Each board
has one writer. Health alone owns the 16 B combined snapshot and calls
`fault_mask_collect_begin(snapshot, normalized_run_epoch)`, then
`fault_mask_combine` for each source. The F2a adapter below handles the
odd START/even STOP epoch contract and freezes the run view. This interface adds no global storage or firmware calls.

| Bits | Meaning | Writer / getter |
| --- | --- | --- |
| 0..3 | Left stall, right stall, ESTOP, stale command | Existing Motion board; STALE is not latched |
| 4,6 | IMU gate latch; other shell FAULT excluding Motion/IMU | Vehicle / `vehicle_fault_mask_try_read` |
| 5 | Ultrasonic streak at least 5; NO_RETURN is not a fault | Sonar / `sonar_fault_mask_try_read` |
| 7 | Existing RAM-log full condition | Telemetry / `telemetry_fault_mask_try_read` |
| 8,9 | Link down; MQTT not connected | Wi-Fi / `robot_network_fault_mask_try_read` |

Writers call `fault_mask_publish(board, owner_mask, value)`. Their getters
call `fault_mask_try_read(board, owner_mask, out)` and return bool. False
leaves out unchanged. Wrong-owner and spare bits are rejected. A zero
sequence is unpublished, not an observed all-clear. No shared byte writes,
new fault latch or actuator input is introduced.

The snapshot contains now, run, valid and epoch. A failed read retains the
last copied now bits with valid clear; it is not a current all-clear. Run
is the OR of successfully copied values in the current run epoch, not a
count or a guarantee that every short transient was sampled. A new epoch
clears run, not the owners' faults. Consumers must check epoch and valid;
they must not show an old epoch as the new run before Health samples it.
The final F integration must handle that boundary in the stopped reader.

F1 tests exhaust all 1024 legal words and check ownership, unchanged
outputs, odd/changing/zero/wrapped sequences, owner independence, current
clear versus run OR and epoch reset. The red build first failed because
the new interface was absent. Host checks pass 685/685; demo build has
zero warnings and RAM_PROOF 79/79, 21212 B. Static RAM remains 240808 B:
F1 has no allocated objects. New owner boards and Health state must be
counted as costs when the publishers and combiner are integrated.
Evidence: `build/wip/cr1/f1-red.log`, `f1-host.log`, `f1-build.log` and
`f1/ram_report.txt` with copied ELF/map/proof.

F1b removes `stddef.h` from the shared header. Kernel `size_t` conflicts
with that header unless a translation unit first sets `PROHIBIT_DEF_SIZE_T`.
`telemetry.c` now includes the fault interface after the kernel headers,
without that define. Its first demo build failed on the conflicting
`size_t`; after the fix it passes with zero warnings, RAM_PROOF 79/79,
21212 B, and unchanged static RAM 240808 B. Host tests remain 685/685.
Evidence: `build/wip/cr1/f1b-red.log`, `f1b-host.log`, `f1b-build.log` and
`f1b/ram_report.txt`. Null pointer checks use 0; the aligned-word offset
assert uses the GCC/Clang builtin without importing a conflicting type.

## A-EVENT retained RAM records

Only the private 512-entry event array changes. Public `telemetry_event_t`
stays 12 B; queued messages stay 116 B. MQTT event and log event formats,
USB VE fields and USB yield pacing do not change. Telemetry remains the
sole array writer. The existing stats lock and committed-count readers
protect encoding and decoding; no shared byte access is added outside it.

| Stored field | Valid range | Bits |
| --- | --- | ---: |
| ms | Full wrapping u32 | 32 |
| word | Full u16 variant payload | 16 |
| detail | Full u8 variant payload | 8 |
| kind | Existing event kinds 1..10 | 4 |
| from / to | Existing state or module IDs 0..5 | 3 each |
| spare | Always zero | 6 |

The 66 meaningful bits use nine bytes. Explicit shifts operate on bytes,
not unaligned words or C bit-fields. Invalid input is rejected before
storage and increments the existing loss counter; no truncated event is
committed. An invalid private record returns an error without replacing
reader output. Valid production values preserve all fields.

The red codec test first failed with the absent header. Tests cover all
legal kind/from/to/detail combinations, every u16 word, u32 endpoints,
reserved bits, invalid inputs, unchanged outputs, event index 511 and the
513th event's existing full/loss policy. Existing log publication,
contention, USB output and log-stream tests pass. The initial full-array
test incorrectly assumed the loss counter started at zero after its
reused fixture; it now checks the increment from the actual prior value.

A-EVENT results: host 686/686; demo zero warnings, RAM_PROOF 79/79,
21212 B. Private events fall from 6144 to 4608 B, saving 1536 B. S1 falls
from 80402 to 78866 B and total static RAM from 240808 to 239272 B.
Codec code is not in RAM and the RAM proof size is unchanged. Queue
allocations and all MQTT payload sizes remain unchanged. Compiler-local
scratch events are not counted as static savings; hardware stack and
timing still need the combined robot gate.

Evidence: `build/wip/cr1/event-red.log`, `event-host.log`,
`event-build.log`, `event/ram_report.txt` and copied ELF/map/proof.

## A-INDEX committed log-index board

The native log index stays 20 B. Its generation, loss counter and newest
row time keep 32 bits each. Row count 0..4501 uses 13 bits; event count
0..512 uses 10 bits. Nine spare bits are zero. The stored payload is four
aligned u32 words (16 B); the sequence adds 4 B, making a 20 B board
instead of 24 B. Telemetry remains the sole writer. Readers first accept
a coherent sequence/word copy, then decode it locally. Failure leaves
the public output unchanged. The existing stats lock, admission fields
and shared counter atomics do not change.

The red test first failed on the absent packed-index type. Tests cover
every legal row and event count, full-width endpoints, reserved bits,
out-of-range codes, changed/odd sequences, committed-count boundaries,
new generations and full event storage. Host tests pass 686/686. Demo
build: zero warnings, RAM_PROOF 79/79, 21212 B.

S1 module RAM falls 78866 to 78862 B (4 B payload saving). Static RAM
falls 239272 to 239264 B (8 B linked saving): an additional 4 B is linker
alignment, not another packed field. Do not promise that alignment delta
for a different link layout. Evidence: `build/wip/cr1/index-red.log`,
`index-host.log`, `index-build.log`, `index/ram_report.txt` and copied
ELF/map/proof. MQTT formats and queue allocations stay unchanged.

## Four optional A records

These changes affect private retained storage only. Existing native APIs,
MQTT binary layouts, field units, queue allocations, timing deadlines and
admission atomics stay unchanged. No source board owned by another helper
is counted in the following savings.

| Record | Information retained / bits | Writer and access | Before / after B |
| --- | --- | --- | ---: |
| 48 stopped scan records | ID32, duration32, fine mask13, coarse hits3 (0..5), attempt1 (1..2), seven zero bits | Telemetry only; decoded for stopped dump | 576 / 528 |
| Retained INFO | Reason48 bytes, ten full u32 diagnostics, two full i32 counts, five 12-bit ADC fields, four independent Motion fault bits | Telemetry only; native scratch for stopped dump; reason termination unchanged | 108 / 104 |
| Health board | 22 full u32 fields, i32 start status, stack/age16 each, CPU10 (0..1000, code1001=unknown), flags7, reset2, link3 (-3..3), MQTT2, priorities6 each (0..32), unavailable mask5 | Health only; existing sequence/word board; metrics lock and caches unchanged | 120 / 112 |
| S1 pending sonar | Seven full u32 IDs/times, status2, valid1, pending1, two angles6 (-30..30), echo14, two zero bits | Wi-Fi only; bounded provider read to local value, encode retained copy, decode for publish | 40 / 32 |

The Health value uses 809 meaningful bits in a 104 B aligned payload;
23 spare bits are zero. Its u32 capture time and sequence add eight bytes.
A failed or invalid encode never publishes a partial value; a failed read
or decode leaves output unchanged. CPU unknown is reconstructed as 65535,
not clipped to a plausible load. INFO is exactly 832 bits, with no spare
bits. IDs, counters and signed extrema are not narrowed. The sonar echo
is already a 14-bit result; unused bits are not physical range evidence.

`value_bits.h` operates on local or owner-protected bytes. It is not an
atomic publication mechanism. Sequence/word boards still use aligned
32-bit atomics. No C bit-fields or unaligned word access is used.

The red optional-codec build failed on the absent header before the
implementation. Tests cover all scan mask/hit/attempt combinations, every
12-bit ADC value, signed/count endpoints, all sonar angle pairs and
status/flags, every 14-bit echo, all CPU codes and link values, all Health
flag/reset/state/unavailable-mask combinations, invalid codes and every
spare bit. Existing real Health refresh/stack, telemetry dump, retry and
MQTT tests also pass. The native sonar pending copy is rebuilt for each
publish retry; identity and acknowledgement policy do not change.

Results: host 687/687; demo zero warnings, RAM_PROOF 79/79, 21212 B.
S1 falls 78862 to 78794 B (68 B retained data). Static RAM falls 239264
to 239200 B (64 B): linker alignment adds 4 B, so do not claim a 68 B
linked saving. Across A-EVENT, A-INDEX and these four records the retained
data saving is 1608 B; linked static saving is also 1608 B in this layout.
No category B or wire saving is included. Hardware codec CPU, task stack
and control timing remain pending; local scratch is not a static saving.
Evidence: `build/wip/cr1/optional-red.log`, `optional-host.log`,
`optional-build.log`, `optional/ram_report.txt` and copied ELF/map/proof.

## F2a STOP capture and Health consumption

The operator requires run history to freeze at STOP, including the fault
that caused STOP. A 1 Hz Health poll alone can miss that fault if the
owner clears before the next poll. `robot_health_faults_stop(odd_epoch)`
is therefore a Vehicle-only bounded capture API. Vehicle must call it
after its fault mirror publishes the stop state and before `rt_begin`
publishes the even STOP epoch. No change to shell state or stop order is
permitted. The Vehicle call belongs to the separate Vehicle integration.

The capture stores five separate owner words, valid bits, odd epoch and
time in a 36 B sequence/word board. It does not OR the owners together.
Vehicle is its sole writer. Health reads the capture and remains the sole
combiner. It reads Motion's existing board and the sonar/Vehicle/log/network
owner getters. Optional absent providers in isolated TEST/BENCH images
are unknown, never an implicit zero. Final robot binding must be checked
in the ELF after all publishers are merged.

Health stores now/run/valid/epoch in its existing snapshot, with sample
time. It normalizes even STOP epochs to the preceding odd run identity,
resets run only for a new START, and ORs successful active samples plus
the final STOP capture. Post-STOP conditions update now only. At wrapped
STOP zero, it keeps the previous run identity. An epoch change during a
collection invalidates that sample rather than combining two runs.

`robot_health_faults_try_read` returns the latest coherent Health snapshot
and capture time; it does not change `rm_heartbeat_t` or MQTT wire bytes.
Valid describes now source-copy quality. Run means faults successfully
observed in that run plus its STOP capture, not proof every short transient
was seen. A missing or busy source at STOP is unknown, not an all-clear.
Consumers must check the epoch before using the frozen run mask.

The real Health test uses fake owner getters and a fake Vehicle caller.
A stall STOP, IMU-lift STOP and sonar-fault STOP each remain in the frozen
run mask even when their owners clear before Health runs. A Wi-Fi fault
raised after STOP changes now only. The test also checks new START reset,
read failures, unchanged heartbeat units, and stack reporting. Its red
build first failed on the missing capture/getter definitions.

F2a software results: host 688/688 (includes the merged sonar owner test),
zero demo warnings, RAM_PROOF 79/79, 21212 B. Relative to optional A at
239200 B, the merged sonar owner adds 8 B, and F2a adds 68 B data:
36 B STOP capture, 16 B Health-only working views, 16 B in the Health
snapshot (board 112 to 128 B). S1 is 78862 B. Linked static RAM is
239272 B, a +72 B delta: +76 B data and -4 B alignment. This is a cost,
not another A saving. F2a has no Vehicle source edit. Evidence:
`build/wip/cr1/f2a-red.log`, `f2a-host.log`, `f2a-build.log`,
`f2a/ram_report.txt` and copied ELF/map/proof.

## F final owner integration and USB output

Main's integration branch was merged, not cherry-picked. The Vehicle
owner publishes bits 4/6, then makes the STOP capture before changing to
the even epoch. Sonar publishes bit 5 from the existing fault streak.
Motion bits 0..3 come from its existing coherent board, not a duplicate
publisher. Telemetry publishes bit 7 from the existing log-full condition
when it publishes log stats. Wi-Fi publishes bits 8/9 from link status
and the actual MQTT connection state at the end of its existing poll.
No owner clears or latches a new fault. STALE remains non-latched; a
connected MQTT client is not called disconnected merely while waiting
for its subscription. Existing multi-writer admission counters and board
unavailable mask remain unchanged.

| Owner | Owned bits | New board B | Single writer / reader |
| --- | --- | ---: | --- |
| Motion | 0..3 | 0 | Existing Motion board / Health and STOP capture |
| Vehicle | 4,6 | 8 | Vehicle / Health and STOP capture |
| Sonar | 5 | 8 | Sonar / Health and STOP capture |
| Log full | 7 | 8 | Telemetry / Health and STOP capture |
| Network | 8,9 | 8 | Wi-Fi / Health and STOP capture |

Every owner board uses complete aligned u32 atomic words with a sequence.
Failed/busy/unpublished reads are unknown. Health is the only writer of
the combined snapshot and run OR. The final ELF binds all five getters
and the STOP API to real GLOBAL symbols, not weak defaults or test fakes.

A raw phase epoch adds 4 B to the Health snapshot. Immediately after STOP,
a prior active Health sample can have the correct odd run identity but
still lack the final STOP fault. `robot_health_faults_try_read` therefore
rejects a snapshot until its raw phase epoch equals the current START/STOP
phase. It leaves outputs unchanged on rejection. Heartbeat reads are not
blocked by this diagnostic quality rule. The owner words and STOP
capture use the same bounded sequence/word read contract.

Both the immediate stopped report and later GP20 dump use the same
Telemetry dump path. If Health has not sampled the new phase, it prints:

```text
[FAULT_MASK] pending Health sample for phase_epoch ...
```

A later GP20 dump prints hexadecimal `now`, frozen `run`, source `valid`,
odd run `epoch` and `sample_us`. It does not wait for Health and does not
combine from the Telemetry task. Now is a sampled condition, not a proof
of continuous current safety. Run is the OR of observed active conditions
plus the final STOP capture, not an event counter. Unknown or busy STOP
sources may still be absent from run; zero must not be called an all-clear.
No new topic, MQTT version or field is added.

Tests first failed because the new owner getters were absent. Log tests
check unpublished/full/next-BEGIN clear. Network tests check unpublished,
connected, link-down and MQTT-disconnected transitions independently. The
real Health tests check stall, lift and sonar STOP captures after owners
clear, new START reset, post-STOP now-only faults, unknown owners and raw
phase rejection. USB tests check the immediate pending line and the later
GP20 hexadecimal values without changing KR/VE or yield expectations.
The pre-existing injected read hook was moved before the new board-header
include so torn-read coverage remains active, rather than being disabled.

Final standard host tests pass 698/698. Clang AddressSanitizer and
UndefinedBehaviorSanitizer checks also pass 698/698. The GCC sanitizer
configure attempt could not link because this machine lacks its libasan
and libubsan runtime files; no GCC sanitizer result is claimed. Clang's
runtime is available. Demo build has zero warnings and RAM_PROOF 79/79,
21212 B. All wire constants/native message sizes and queue
allocations remain unchanged. The combined robot image and 110 ms hardware
gate belong to Main/operator; no flash or physical run was done here.

| Final F allocation | Added data B relative to optional A |
| --- | ---: |
| Four new owner boards (Vehicle/Sonar/Telemetry/Wi-Fi) | 32 |
| Vehicle-only STOP capture board | 36 |
| Health-only working views | 16 |
| Health snapshot: four fault fields plus raw phase | 20 |
| Total F cost | 104 |

Health board is now 132 B (112 B after A plus 20 B F). S1 is 78882 B:
78794 after A plus 88 B F. Static RAM is 239304 B: 239200 after A plus
104 B F. The net A/F saving from `33d2e4c` is 1504 B (1608 minus 104).
The net change from the common `7350da0` baseline is -1924 B, which
includes the earlier 420 B from other helpers; do not count that twice.
Relative to F2a's 239272 B, the final increase is 32 B: Vehicle owner8,
Telemetry/Wi-Fi owners16, raw phase4 and linker alignment4. Relative to
the merged Vehicle F2a result (239280 B, measured by SW11), this final
commit adds 24 B: 20 B S1 data plus 4 B linker alignment. The existing
before result is reused; it was not rebuilt. Final linked
alignment is 943 B, equal to optional A. Category B and wire savings are
zero. Hardware stack/CPU, PBUF/heap headroom and task timing remain pending.

Evidence: `build/wip/cr1/final-red.log`, `final-host.log`, `final-build.log`,
`final/ram_report.txt`, copied ELF/map/proof and `final/owner-symbols.txt`.
`final/message_sizes.txt` matches every baseline S1 native message and wire
size. Sanitizer evidence: `final-clang-asan-build.log` and
`final-clang-asan.log`; unavailable GCC runtime details are retained in
`final-asan-build.log`.

## Binary MQTT audit

| Topic | Version | Payload bytes |
| --- | --- | ---: |
| state / sonar / obstacle | 5 / 2 / 3 | 76 / 36 / 32 |
| event / status / heartbeat | 2 / 2 / 4 | 20 / 6 / 108 |
| detour / summary / barcode | 2 / 3 / 2 | 36 / 52 / 28 |
| log | 2 | 36 control; batch 36 + 16 per record, maximum 228 |
| cmd, dashboard or CLI to robot | 1 | 22 |
| imu | Reserved | No active publisher; reserved binary layout |

All active robot MQTT payloads are already binary, including status's
retained offline will, log batches and tokened commands. No JSON-to-binary
saving is claimed. The dashboard's browser-to-laptop HTTP/SSE traffic is
JSON; its MQTT command is produced by `pack_command()` as bytes. USB and
CSV evidence are text, not MQTT payloads.

The library demo in `mqtt_utk.c` contains text `hb %u`. In SW7's canonical
ELF its weak hook is discarded and the strong `robot_mqtt` hook is bound.
No `mqtt_utk_start` or `heartbeat_timeout` symbol remains in that ELF.
This dormant demo is not an active robot MQTT publisher.

## RAM and build evidence

| Revision / step | Static RAM B | Change B | Evidence |
| --- | ---: | ---: | --- |
| Common baseline `7350da0` | 241228 | Reference | SW7 canonical map |
| Starting integration `33d2e4c` | 240808 | -420 from common baseline | Existing integrated A results |
| T110 | 240808 | 0 from starting integration | Local after map |
| F1 interface only | 240808 | 0 from T110 | Local after map; no global storage |
| A-EVENT | 239272 | -1536 from F1b | Local after map; events 4608 B |
| A-INDEX | 239264 | -8 from A-EVENT | Index -4 B, linker alignment -4 B |
| Four optional A records | 239200 | -64 from A-INDEX | Data -68 B, linker alignment +4 B |
| F2a plus merged sonar owner | 239272 | +72 from optional A | Sonar +8, S1 +68, alignment -4 B |
| Final F including Vehicle and local owners | 239304 | +104 total F cost from optional A | Final S1 78882 B, Health board 132 B |

T110 leaves S1's module RAM at 80402 B. RAM-log rows stay 72016 B,
events 6144 B and scan rows 576 B at this step. Queue allocations stay
4096 B telemetry, 384 B events, 96 B commands and 12 B ESTOP. Later A
steps must report their own delta; do not count the earlier -420 B again.
New F storage is a cost, not a packing saving.

The T110 demo profile uses Arm GCC 14.3.1, Cortex-M0+ Thumb:

```sh
make -C build_make --output-sync=target -j4 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=100 'BARCODE_MAP=F=LEFT' JUNCTION_TEST_CMD=NONE
```

T110 software results: 684/684 host tests, zero compiler/assembler warnings,
`RAM_PROOF PASS 79/79`, 21212 B. The red period test failed at the old
200 ms value before the change. Three old MQTT RUN-count assertions also
failed after the change; they now check the new deadline and counts.
No expected IDLE or heartbeat rate was relaxed.

Ignored evidence is under `build/wip/cr1/`: `t110-red.log`,
`t110-host.log`, `t110-build.log` and `t110/ram_report.txt` plus the
copied ELF/map/proof. Host Make-profile tests can invalidate build objects;
copy each after ELF/map before another full host run. This is not a source
or firmware change.

## Robot gate: 110 ms runs only

No new 200 ms robot run is permitted. Use existing files under
`~/Projects/robot-logs/20261118_post_freeze/`.

| Existing evidence | Observation / permitted use |
| --- | --- |
| `mqtt_FIX2.csv`, primary network reference | 796 RUN-tagged frames; 788 adjacent RUN pairs; 493 missing sequence values between endpoints (38.49%) |
| Same RUN pairs | Receipt gap median / p95 / p99 / maximum: 215.669 / 1435.722 / 2735.613 / 3586.181 ms |
| FIX2 first recorded session | Over 492325 ms reported uptime: publish_ok +5052, publish_failed +8101; mixed RUN/IDLE and topics |
| `robot_FIX2.log`, final epoch 13 | KR gap 9979..10023 us; lost telemetry, lost wakes and overruns zero; response maximum 2432 us; Telemetry stack 1096 used / 3000 free |
| `robot_LAST.log`, epoch 1 | Secondary stopped reference: gap 9983..10015 us; same three counters zero; response maximum 2480 us; Telemetry stack 1104 used / 2992 free |
| `mqtt_LAST.csv` | 1069 state frames but no active/RUN-tagged frames; not a RUN network reference |

The FIX2 file SHA256 is
`906a49270d1e1d8ac976628d7662a252c52fd5400ffdb79f2b370f83dbf3a005`.
Sequence gaps include enqueue, broker and receiver loss, not only radio
loss. Publish counters include mixed-topic local and asynchronous results;
their ratio is not a QoS0 delivery probability. LAST contains the old lost
sonar result behaviour. Do not attribute course differences only to T110.

After Main builds the combined image, the operator records 110 ms runs
on the same course/hotspot and records firmware, boot/run IDs and battery.
Compare RUN and active windows, state sequence gaps, publish counter deltas,
KR task gaps, lost wakes, overruns and first-overrun. Check Telemetry and
Wi-Fi stack high-water, aggregate CPU, admission lost/busy/full state and
queue losses. Do not substitute board-read busy counters for admission
busy. Per-task CPU and minimum live PBUF/heap headroom need approved
bounded diagnostics; existing logs do not provide those values.

The canonical PBUF allocation is 36771 B for 24 buffers; the lwIP heap
allocation is 16403 B for a 16 KiB arena. Runtime lwIP statistics are off.
These allocations and static unreserved RAM are not measurements of free
network headroom. No 110 ms hardware CPU, stack, headroom or timing pass is
claimed. If the robot gate fails, report it; do not reduce another stream
or increase a pool without approval. No flash or robot run was done here.
