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
| A-INDEX | Pending implementation |
| A-SCAN, A-HEALTH, A-SONAR, A-INFO | Pending implementation |
| F combiner and output | Pending implementation and owner publishers |

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
`fault_mask_collect_begin(snapshot, rt_epoch())`, then `fault_mask_combine`
for each source. This interface adds no global storage or firmware calls.

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
