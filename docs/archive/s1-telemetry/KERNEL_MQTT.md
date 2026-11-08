# Kernel MQTT telemetry

## Added observations

State v3 adds signed distance, wheel duty and encoder-derived heading/rate at
unchanged 5 Hz. Completed scan profiles and detours use separate last-record
boards with sequence gaps, not the event queue. The run summary uses QoS 1
and retain, with retry and reconnect publication. The vehicle's pure
`run_stats` observer never returns commands or alters module inputs.

Heartbeat v2 adds total core-0 CPU load from the calibrated lowest-priority
meter; see [method and unmeasured target limits](../kernel/CPU_LOAD.md). A retained
ONLINE/OFFLINE last will uses a 5-second keepalive; recent state is still
required to establish that a robot is online. Barcode/IMU/JUNCTION and scan
stage are reserved contracts, not implemented sensors or navigation changes.
All byte layouts and estimator limits are in [the wire contract](../../s1-telemetry/MQTT_WIRE.md).

## Purpose and safety

STEP 8 adds **non-critical telemetry**, not a network controller. Driving
never depends on a Wi-Fi link, radio initialization, broker, subscription
or network queue. Startup does not wait for them. A link/broker failure
never requests a motor fault or stop. RAM logging continues with its
existing local storage/error policy. M1/M2 acceptance is independent of
MQTT; S1-U04/U05/U06 are separate network checks.

Remote START and ESTOP are conveniences. MQTT ESTOP is best effort, with
no stop-latency guarantee. **GP21 remains the real hardware button ESTOP**
(sampled by vehicle); only completed motion ticks feed the watchdog.
The network cannot drive PWM, clear faults, refresh wheel commands or feed
the watchdog. Pure navigation, tuning, routes and eight-conversion ADC
means are unchanged. GP28 is sampled only, with the VBAT jumper open.

Build and broker steps are in [the runbook](MQTT_RUNBOOK.md). Exact wire
layouts are in [the binary protocol](../../s1-telemetry/MQTT_WIRE.md). Shared platform changes
and the lecturer-update procedure are in [the template ledger](../../architecture/TEMPLATE_CHANGES.md).
All new-image hardware checks remain **Not run** until the operator runs them.

## Execution and hook

```text
ADC/echo/encoder ISR -> motion10 / line11 / sonar12 -> vehicle13
                                 | read-only boards       | rare events
                                 +------------------------+------+
                                                                v
                              cyw43_utk_task20 -> lwIP MQTT -> broker
                              (sole lwIP owner)     ^
                                                    | robot/cmd
                              bounded command queues -> vehicle13

vehicle13 -> local RAM queue -> telemetry30 -> committed RAM log
health31 <- bounded boards + stack scan; 1 s cyclic -> health board
```

The template used Wi-Fi priority 4: below USB 3, above its qualification
task. It now **creates Wi-Fi directly at 20**, below driving tasks 10–13
and above RAM telemetry 30. The USB console is 21, below Wi-Fi (see the
[template ledger](../../architecture/TEMPLATE_CHANGES.md)). There is no priority-change
wrapper and no transient execution at priority 4.

Under `TM_MQTT`, the template poll calls `cyw43_utk_app_poll()` after
`sys_check_timeouts()` when the link is UP. The weak library default calls
the old demo; `robot_mqtt.c` supplies the strong robot implementation.
No `--wrap`, `__real` call or second client task is used. The final map,
symbol table and caller disassembly must show the strong application hook
and no retained `mqtt_utk_start` demo. Only this Wi-Fi task and callbacks
on its stack call lwIP. Sensor/vehicle/health providers make no lwIP calls.

Connection failures retry after 3 s. Keepalive is 30 s. Subscription to
`robot/cmd` retries independently after an error. The client is allocated
once from the existing lwIP arena. No SDK runtime or extra SDK source is
added. `0.0.0.0` is a nonconnecting broker placeholder, not a discovery mode.

Each hook makes at most four publish attempts, at most two sonar records,
and at most one log attempt. In RUN, state is due every 200 ms and the
heartbeat every 1000 ms. In IDLE, state is due every 1000 ms and the heartbeat
every 5000 ms, and the hook publishes no live log batches and no sonar
records. It moves the sonar history cursor to the newest entry instead
(a skip, not counted as missed), so RUN does not publish old idle results. RUN covers vehicle work plus a 2 s grace; TEST and BENCH images
always use RUN. The mode rule is pure
([telemetry_mode.c](../../../app_program/subsystem1/telemetry_mode.c)); the hook
reads it from one bounded vehicle-board read
(`robot_network_vehicle_phase`). That read is not counted in the heartbeat
busy or unavailable counters. See
[Publication modes](../../s1-telemetry/MQTT_WIRE.md#publication-modes). There is no catch-up burst after a missed deadline. MQTT callbacks
stage only a bounded 22-byte command, reject malformed fragments and
count failures. No token is logged. lwIP copies publish bytes into its
output buffer; stack-local packed arrays do not escape.

## Pull boards

| Board | Writer | Reader contract |
| --- | --- | --- |
| Line | line task and serialized calibration API | `line_snapshot_try_read`; raw L/C/barcode, lp, validity, acquisition time/sequence and counters; never consumes events |
| Motion | motion task | `motion_snapshot_try_read`; last-tick status and sample time; no control mutex |
| Sonar | sonar task | `sonar_snapshot_try_read`; latest result without clearing its vehicle event; independent history reader |
| Vehicle | vehicle task | `vehicle_snapshot_try_read`; state/module/phase, active/calibrated, last wheel/servo command, last remote request/status, wake/deadline/handoff counters |
| Health | health task 31 | `robot_health_try_read`; uptime/reset, local/network losses, overruns, app-stack minimum and Wi-Fi/MQTT status |

Publication uses aligned atomic 32-bit words and a version guard. Readers
make **one bounded attempt**. Odd/changing guards return unavailable, never
spin. This avoids deadlock if a high-priority reader preempts a writer.
Wi-Fi at 20 cannot itself preempt writers 10–13, but they can interrupt its
copy; health31 can also be interrupted by its Wi-Fi20 reader. All these
boards and callers belong to core 0; this is not a cross-core API.

Readers commit only a successful scratch copy. On failure, the network
retains prior values, records `E_TMOUT` as genuine busy or `E_OBJ`/`E_NOEXS`
as unavailable (with the v3 heartbeat mask/aggregate), and
clears its fresh flag. Only the main-board read controls its unavailable
mask. Empty secondary obstacle/detour/summary/history cursors are not absent
main boards. Sonar history now returns `E_OBJ` for service-not-ready,
`E_TMOUT` for a writer conflict, and `E_NOEXS` for no next entry, so an idle
history cursor does not inflate busy. No sensor acquisition or driving
caller changes; this API has one observation adapter and host tests.
Ages saturate at 65535 ms. Fresh limits are line
10 ms, motion 20 ms and vehicle 30 ms; line also requires valid data. A
stalled source sequence never refreshes its cache age, even across a
32-bit microsecond wrap. Time is observed **after** preemptible reads so
newer writer timestamps cannot underflow into a false ancient age.
The explicit age/fresh bits, not the retained value alone, describe it.
Health is fresh only after a successful read with age at most 2500 ms.
Age uses producer uptime milliseconds, so a stopped producer cannot look
young at the 71-minute microsecond wrap. Before a first sample, age and
stack minimum are unknown (65535), never zero/free.
Its component lifetime counters can lag if a source read was unavailable.

MC and line public boards are transport-independent and also exist in
radio-off builds. Their added publication cost is inside task work timing.
Their published max-work counters can lag by one tick/sample because the
publication must finish before that work interval can be measured. Existing
mutex status reads still provide the updated counters. Stack minimum covers
registered application stacks only, not template Wi-Fi/USB or IRQ stacks.

The sonar ring retains the last **16 service result publications**:
ECHO/DEADLINE/CANCELLED. It is not a trace of every physical edge. Existing
stale/orphan physical echoes stay discarded; ownership logic is unchanged.
Sequence/cursor distance detects eviction and reports misses. History and
events never depend on a successful latest-board read. A pending failed
sonar publish keeps its original identity/time while retries continue.
The state KR1 echo becomes unknown on a busy/invalid or >100 ms old sonar
snapshot; it never presents a retained old range as fresh. Detailed sonar
packets carry original trigger/result timestamps.

## Push events and commands

Only vehicle pushes live events: module change, outcome, ESTOP/fault, scan
completion, session end and command result. The event MBF holds 16 records;
all sends use `TMO_POL`, and full/unavailable sends increment a loss counter
without cancelling navigation. Event sequence advances before enqueue so
missing events are visible. RAM events keep their existing separate path.

Ordinary commands use an 8-record MBF. ESTOP has its own one-record MBF and
is received first. A full ESTOP slot already contains a stop: additional
stops coalesce and count as loss. Vehicle processes at most one command per
step. ESTOP cancels pending actions and drains old queued commands, so a
queued START cannot immediately undo it. Wi-Fi never calls motion APIs.

Every command has version, opcode, request ID and a 16-byte token field:

| Configuration | ESTOP | DUMP / CALIBRATE / START |
| --- | --- | --- |
| No valid non-example token | Accept a correctly shaped ESTOP; token field still present but unchecked | Reject and count |
| Valid configured token | Require matching token | Require matching token |

Empty, malformed or all-zero example token is unconfigured. There is no
separate enable switch that could accidentally leave a configured ESTOP
unguarded. Configured-token comparison visits every byte. Header and laptop token file
are private and untracked. This is not TLS or replay-proof authentication;
use a trusted, isolated robot network. Never retain commands at the broker.

CALIBRATE and START require idle and no pending action or USB dump. START
also requires learned calibration. CALIBRATE spins and returns idle; START
uses existing calibration, waits the normal 1 s and starts navigation.
GP21 keeps its prior calibrate-then-start behaviour. DUMP can run while
driving. Duplicate non-ESTOP request IDs in the last eight deliveries are
rejected; this RAM-only filter does not survive reboot. Command-result
events give opcode/status, not a request-ID acknowledgment. Broker PUBACK
is not robot acceptance. Policy rejections and all network queue problems
never become vehicle or motor faults.

## Live log stream and DUMP refill

`robot/log` carries two streams from [`log_stream.c`](../../../app_program/subsystem1/log_stream.c).
Both read the RAM log through bounded readers and never wait for the logger.
Wire layouts are in [MQTT_WIRE.md](../../s1-telemetry/MQTT_WIRE.md#ram-log), the dashboard contract.

**Live stream (QoS 0, freshness first).** A cursor follows the newest KR1
rows. It publishes 10 rows when 10 are waiting, or fewer when the oldest has
waited 100 ms: about one 192-byte batch every 100 ms at 100 rows/s. Each batch
holds the robot uptime when it was built and when the newest row was stored,
so a dashboard can show data age. If the cursor lags by more than 100 rows
(about 1 s) or the link was lost, it jumps to the newest 10 rows and reports
the skipped count. It never replays a backlog. A new run generation restarts
at index 0 and reports old rows that were never published. QoS 0 means no
head-of-line blocking: a slow or stalled broker cannot delay newer rows, and
no ACK is awaited. A failed publish retries after 50 ms with the same cursor.
At most one live batch is published per poll. In IDLE publication mode the live stream
is paused; on return to RUN it jumps to the newest 10 rows and reports the
skip. DUMP refill runs in both modes.

The newest-row time comes from the telemetry task. It stores the uptime when it
commits each row (`MQTT` builds only, one 32-bit store). Age is the time since
the row entered the RAM log. It does not include the sensor sample time or
network delay.

**DUMP refill (QoS 1, completeness).** The `DUMP` command freezes the run
generation and the row/event counts. The robot sends BEGIN, then batches of
12 rows and 12 events, then END. Up to three QoS 1 batches are in flight (the
lwIP request table holds four). The refill cursor advances only on PUBACK,
in send order. A failed acknowledgement or a reconnect resends from the
acknowledged cursor, so duplicates are possible: drop them by
(generation, index). A new run generation sends ABORT. At most two DUMP
frames are published per poll.

**How the streams share the link.** Live frames use no QoS 1 request slot, so
a DUMP cannot delay them. Each poll serves live first (one batch), then DUMP
(two frames). The Wi-Fi poll, the lwIP send buffer and the network are the
shared resources. Per poll the hook publishes up to 4 routine frames, 1 live
batch and 2 DUMP frames.

**Throughput.** The host simulation in `test_log_stream_dump_throughput`
gives a full 4501-row, 512-event DUMP in 2.8 s at 20 ms round-trip time,
14.2 s at 100 ms and 28.4 s at 200 ms (three batches per round trip, polls
every 10 ms). The previous design needed at least 251 s. The 20 s target holds
for round trips up to about 140 ms. These are simulations, not hotspot
measurements.

**Frame size.** The lwIP output ring is 256 bytes (`MQTT_OUTPUT_RINGBUF_SIZE`
default; not changed). The largest frame (a 12-record QoS 1 batch) needs
1 + 2 + 2 + 9 + 2 + 224 = 240 of its 255 usable bytes. A live batch of 10 rows
needs 206. No template change was needed.

**Memory.** The streams use 560 bytes of static RAM (frame and record buffers
and cursors), not task stack.

In this SDK's lwIP, disconnect **deletes pending requests without calling
their callbacks**; the connection handler rewinds the DUMP to its acknowledged
cursor and makes the live stream jump. QoS1 can duplicate a frame; use
generation and index.
There is no promise of a complete dump under continued disconnect, reset,
new runs or storage loss. State/sonar/events/heartbeat use QoS0, not retained;
sequence gaps and health counters expose known losses. `publish_ok` means
accepted by lwIP, not delivered to a laptop. `publish_failed` includes known
allocation/publish/subscription/connection errors; a connection drop and
its abandoned outstanding log frame count separately. Packet loss after a
QoS0 acceptance is not exactly measurable by firmware.

## Timing and remaining gates

Kernel hardware startup now selects the 1 us tick immediately after clk_ref
becomes the 12 MHz XOSC, before devices, tasks or radio start. Boot safety no
longer depends on usermain running first. Its idempotent TIMER verification,
the unchanged 2% clock check and motion watchdog policy remain in place.

Full Wi-Fi poll metrics cover ALL polls, including link-down, not only MQTT
hooks. Independent window metrics support S2's 5 s and a complete vehicle
session (calibration/delay/run). They measure wall time with preemption, not
CPU utilization. Gap counts use strictly >10000 us; the requested delay is
2 ms, but the 10 ms kernel tick and `tk_dly_tsk` extra-tick semantics make
the actual cadence about 20 ms (roughly 50 polls/s), as observed on hardware.
This known port property is documented, not changed by this fix. The probe's
own max is also reported,
excluding its final stores. Health heartbeat carries lifetime gap max/sample
count; TEST and post-session USB lines show work/threshold/window details.
A no-poll or stale-health interval is not a performance pass.

Static sizes, strict ARM/SDK proofs and host tests are recorded in
[Kernel robot](../kernel/KERNEL_ROBOT.md). Earlier-image [operator results](../MTK3_STEP8_RECORD.md) include
cold-power TIMER, timing, MQTT/reconnect, commands and basic M1/M2. They do not
validate later boot/session/diagnostic or SCAN-retry changes. New-image
hardware still needs its own acceptance. No push or flash is
part of this implementation.
