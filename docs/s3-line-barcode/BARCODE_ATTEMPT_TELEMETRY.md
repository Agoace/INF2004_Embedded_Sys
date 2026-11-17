# Barcode terminal-attempt telemetry

## Status and scope

Software implementation from staging `b9a47b5`, merged into the B2b helper
at `a2a5f39`. No robot flash or hardware test. SHADOW only. Driving, source
control, terminal ACK policy, symbol mapping and decode thresholds are
unchanged. A published observation is not permission to steer.

The [wire contract](../s1-telemetry/MQTT_WIRE.md#barcode-v2) defines
`robot/barcode` v2/kind1,28 bytes. It reports each terminal bound candidate,
including failures, instead of treating the existing run-summary counts as
individual decode events. The legacy20-byte v1 decoder is unchanged.
The dashboard panel is a separate integration: it must handle v2 fields,
not invent v1 confidence votes or an executed result.

## Producer and ownership

1. LINE completes the existing producer observation and final source recheck.
2. It records terminal uptime and immutable metadata: original candidate epoch,
   its start time, outcome, reason, optional fitted start n/w and print order.
3. It attempts one nonblocking enqueue. A full queue drops the NEWEST
   observation and increments the lifetime drop counter; driving and decoder
   state do not depend on enqueue success.
4. Only the Wi-Fi task peeks/ACKs the head, encodes bytes and calls MQTT.

The terminal sequence increments for every finished attempt and skips zero,
including events dropped from the queue. It is separate from the decoder's
record sequence, which is zero on failures. A delayed old-context cancellation
keeps the OLD epoch in its event; it cannot be charged to a new epoch.
A failure's letter and mapped command are zero. A decoded F/NONE is still
an OK observation. All current messages have SHADOW set and represent
issued=NO/executed=NO.

A fitted start star can exist on a later failure. Those widths are marked
valid but are NOT accepted full-frame widths. When no valid start fit exists,
widths are unknown (`65535` on wire, blank in CSV), not physical zero. No
requested letter or measured ADC is fabricated. The0.01 mm wire unit is
quantization, not a claim of measurement accuracy.

## Bounded FIFO and delivery

[`barcode_attempt.c`](../../app_program/subsystem3/barcode_attempt.c) is an
8-slot single-producer/single-consumer RAM FIFO. The producer owns `head`,
the consumer owns `tail`; acquire/release atomic indices transfer payload
ownership. A slot cannot be overwritten until successful ACK advances tail.
The producer cannot wait for a consumer. The consumer cannot spin on a writer.
Full queue drops new data, retaining unacknowledged observations unchanged.

The FIFO has236 bytes of C storage with the ARM ABI (8 padded28-byte values
plus three32-bit indices/counters), guarded by a240-byte compile-time ceiling.
It is allocated only for MQTT ROBOT profiles. TEST, BENCH and non-MQTT builds
use no-storage no-op queue functions. These type counts are not a full-image
RAM measurement; the gate section records the matched-image result.

Wi-Fi sends at most one new barcode request per poll and keeps at most one
barcode request in flight. Successful PUBACK retires the matching FIFO head.
Immediate refusal and asynchronous error retain it and retry after the
existing status retry interval. Disconnect invalidates the callback token;
reconnect retries the retained head. Old callbacks cannot ACK a new request.
The callback contains no LINE access, lock or wait.

QoS1 can deliver duplicates. Consumers can identify an attempt by established
boot session, candidate epoch and terminal sequence. The28-byte payload has
no embedded boot ID. State/status establishes boot identity; a frame received
before boot identity cannot independently resolve that boundary. The recorder
keeps received attempt rows rather than silently discarding diagnostic
retransmissions. No flash persistence or unlimited disconnected history is
promised. The source has8 slots: loss is possible and explicit.

The lifetime queue-drop count is in `[BC_REASON_COUNTS]`; sequence gaps also
show missing events once later events arrive. No heartbeat layout changed.
A stalled MQTT consumer must not change the decoder or vehicle behavior.
Pending attempts drain in IDLE too, unlike rate-limited state observations.

## Early stopped dump

The bounded barcode dump now precedes the thousands of KR rows in a stopped
GP20 dump. Its first line is:

```text
[BC_REASON_COUNTS] epoch E attempts T ok O failed F dropped D BW0..10 ... BP100..107 ...
```

The11 BW bins are reason0 through10; the8 BP bins are100 through107. The
reason histogram is LINE-written and copied once with the atomic board
protocol. It resets only for a NEW nonzero run epoch. STOP epoch0 does not
erase the previous run's diagnostic histogram. An old event arriving after a
new nonzero epoch is not assigned to that new histogram. The existing
[read-only counts snapshot](LIVE_COUNTS_SNAPSHOT.md) contract is unchanged;
it is a separate observed run snapshot, not a drain acknowledgement.

The histogram can include a last-run cancellation processed after Vehicle
sampled its pre-stop summary. Do not claim those two different observation
cutoffs must have identical totals. `dropped` is lifetime queue loss; it is
not an extra reason and is not included in `failed`.

The existing bounded8 `BC_FRAME/BC_REF` history now appears BEFORE KR rows.
`BC_REF` appends `terminal_us` (low32-bit microsecond timer), retaining all
existing fields. The start and terminal times identify different contexts:
a disabled-source failure can occur well after the candidate started.
Use the paired run time and modular clock rules; do not treat terminal time
as an ADC bar edge. The reason histogram is complete for the retained named
run, even if its8-row history has wrapped. The existing edge rows remain
bounded. No full-rate ADC capture is added by this feature.

## Tests and acceptance gates

New C wire and Python v2 tests first failed on missing type/API and unknown
version behavior, before implementation. Local RED logs are under
`build/barcode-v2/`. Targeted tests then cover:

- Exact independently specified28-byte little-endian payload, F/NONE,
  failures, missing widths, malformed flags/letter/reason/size/direction,
  unchanged v1 and unknown-version handling; CSV exposes every v2 field.
- FIFO non-consuming reads, wrong-sequence ACK, full/drop-newest, stable head
  until ACK and repeated ring wrap.
- Actual LINE source publication, successful A and F/NONE, final source
  recheck failure, same-epoch cancellation, late old-epoch event identity,
  new-epoch count attribution and histogram persistence after STOP epoch0.
- Actual Wi-Fi task publication context, QoS1, immediate refusal, asynchronous
  failure, no repeat while in flight, reconnect replay and stale callback.
- Barcode dump invocation before KR rows, and appended terminal time.

| Gate | Result |
| --- | --- |
| Strict GCC host | 554/554 pass |
| Clang ASan/UBSan host | 554/554 pass |
|38-profile/76-image comparison |38/38 zero-warning builds;75 DIFF,1 MATCH (UP BENCH UF2) vs `a2a5f39`; ELF debug/layout differences are not a behavior claim |
|Object audit | All38 raw-object sets match the expected source/header dependency list; stripped USB/MQTT/TEST representatives change only intended files plus the new FIFO |
|Matched KERNEL_RAM proof and image budgets |75/75 RAM symbols pass; RAM-function bytes18424 ->19156; full allocated vector+.data+.bss219432 ->220728 B (+1296 B) |
|Compiler stack frames | LINE task96 unchanged; source88 ->96; producer72 ->88; cancel48 ->64; DUMP96 unchanged; MQTT service312 unchanged; enqueue16 and packer16 B. Hardware high-water NOT RUN |
|Documentation/template checks | Link/anchor check PASS;8 recorded template deviations unchanged |
|Dashboard integration | SW8 owns the v2 panel; integrate it with this decoder |
|Robot MQTT/early dump acceptance | NOT RUN; wait for review |

### Resource comparison detail

The matched software-only resource build uses public example settings,
`SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1`, GCC14.3.1 with
`-fstack-usage`, the same export path and clean builds for each revision.

| ELF section | `a2a5f39` | Candidate | Delta |
| --- | ---: | ---: | ---: |
| .text (flash) |418320 |419728 |+1408 B |
| .data (initialized RAM, including hot functions) |19344 |20104 |+760 B |
| .bss |199896 |200432 |+536 B |
| vector |192 |192 |0 B |

The default `size` totals count read-only RAM code in `text`; its summary is
437928 ->440096 text and200088 ->200624 bss. Those summary fields alone do
NOT measure allocated RAM: use the section table above.

Adding terminal direction evidence stops GCC's old `cancel.isra.0` clone.
The real function is now `cancel`; `tools/kernel_ram/robot.list` names its
actual `.text.cancel` section. An initial proof FAILED on the missing old
clone. The list was corrected, not bypassed; final75/75 PASS includes real
cancellation code in RAM. The proof reports34 distinct flash callees vs32:
new `barcode_attempt_enqueue` and `barcode_live_ms_at`; additional terminal
width math also calls existing soft-float/rounding helpers. No claim that
terminal processing executes entirely in RAM or has measured timing is made.

The raw-object differences across38 profiles are the five modified firmware
C files and transitive users of changed public types. A separate
`objcopy --strip-debug` comparison of clean UP USB, MQTT and TEST builds
confirms that unmodified vehicle/control/queue/health objects are identical.
The stripped changes are `mqtt_wire.o`, `telemetry.o`,
`barcode_live_producer.o`, plus `barcode_live_source.o` when present and
`robot_mqtt.o` in MQTT, and added `barcode_attempt.o`. No control law,
calibration profile, action policy or sensor-model input changed.

The stack numbers are individual compiler frames, not full call chains or
runtime high-water results. LINE and telemetry reservations remain2048 and
4096 B; Wi-Fi remains8320 B. No stack reservation was enlarged. Device timing,
dump completeness, broker duplicate behavior and high-water remain operator
gates, not inferred from a successful host build.

Generated logs, matrices and compiler artifacts belong under
`build/barcode-v2/` and `/tmp/sw10-barcode-v2-*`, not in Git.
