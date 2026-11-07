# Kernel MQTT telemetry

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
layouts are in [the binary protocol](MQTT_WIRE.md). Shared platform changes
and the lecturer-update procedure are in [the template ledger](TEMPLATE_CHANGES.md).
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
and above RAM telemetry 30. USB remains 3. There is no priority-change
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
and at most one log attempt. State is due every 200 ms; heartbeat every
1000 ms. There is no catch-up burst after a missed deadline. MQTT callbacks
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
retains prior values, increments that board's busy/unavailable counter and
clears its fresh flag. Ages saturate at 65535 ms. Fresh limits are line
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

## RAM dump and delivery limits

`robot/log` uses one QoS1 frame at a time, no faster than 50 ms. Other topics
have priority. The first successful bounded log-index read fixes generation
and committed row/event highwater. Pending local MBF writes are not flushed
or waited for. Rows continue to append. BEGIN, KR1 rows, VE events and END
are explicit records. If a new session resets generation, emit ABORT instead
of mixing runs. The snapshot stays fixed across failed BEGIN attempts.

Advance a log cursor only after PUBACK. In this SDK's lwIP, disconnect
**deletes pending requests without calling their callbacks**; the connection
handler explicitly clears the pending-log marker and retries the same frame
on reconnect. QoS1 can duplicate a frame; use dump ID/generation/index.
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
2 ms but the kernel tick is 10 ms. The probe's own max is also reported,
excluding its final stores. Health heartbeat carries lifetime gap max/sample
count; TEST and post-session USB lines show work/threshold/window details.
A no-poll or stale-health interval is not a performance pass.

Static sizes, strict ARM/SDK proofs and host tests are recorded in
[Kernel robot](KERNEL_ROBOT.md). Hardware must still establish cold-power
clock rate, motion gaps under Wi-Fi traffic, network loss/decode/reconnect,
command policy, stack margins and independent M1/M2. No push or flash is
part of this implementation.
