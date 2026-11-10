# Telemetry design

## What it is

Telemetry observes the robot. It does not own navigation, motors, the
watchdog or startup permission. The local RAM logger and optional MQTT
publisher are separate owners. MQTT is a message protocol through a broker;
a broker acknowledgement means receipt by the broker, not robot acceptance.

See [OPERATION.md](OPERATION.md) to run the tools,
[MQTT_WIRE.md](MQTT_WIRE.md) for every byte and version, and
[KERNEL.md](../kernel/KERNEL.md) for tasks and safety.

## How it works

```text
motion10 / line11 / sonar12 -> read-only boards --+-> Wi-Fi20 -> broker
vehicle13 -> vehicle board + rare event queue ---+     |
health31 -> health board ------------------------+     +-> commands -> vehicle13
vehicle13 -> copied RAM messages -> telemetry30 -> committed RAM rows/events
                                                   ^
Wi-Fi20 reads committed entries for live batches and network DUMP
```

### Execution and hook

Only the existing Wi-Fi task and callbacks on its stack call CYW43, lwIP
(the network stack) or MQTT. It runs at priority 20, below driving10–13 and
above USB21 and logger30. `cyw43_utk_app_poll()` is the named application
hook after network timeout service. A strong application function replaces
the weak library demo. No wrapper, second client task or SDK runtime is used.
The [SDK link proof](../kernel/BUILD.md#host-tests-and-link-proof) checks
binding and dependencies. Sensor, logger, vehicle and health code never
calls lwIP.

Startup does not wait for a radio, broker or subscription. Failure never
sets a motor fault, requests a stop, clears a latch, refreshes drive commands
or feeds the watchdog. The network cannot write PWM. GP21 is the local
ESTOP button; MQTT ESTOP has no delivery or latency guarantee.

## Interfaces: boards and queues

A board is a short, read-only copy of its owner's latest publication.
Aligned 32-bit words and a version guard let a reader make one attempt.
An interrupted or changing copy fails rather than spinning or blocking the
writer. These interfaces are for core 0, not cross-core synchronization.

| Board | Writer | Read interface / content |
| --- | --- | --- |
| Line | line task / serialized calibration | `line_snapshot_try_read`; L/C/barcode ADC, filter, validity, sample time/sequence and counters; no event consumption |
| Motion | motion task | `motion_snapshot_try_read`; coherent last-tick status without the control mutex |
| Sonar | sonar task | `sonar_snapshot_try_read`; latest result without clearing vehicle notification; separate 16-entry history |
| Vehicle | vehicle task | `vehicle_snapshot_try_read`; state/module/phase, active/calibrated, requests and deadline counters |
| Health | health31 | `robot_health_try_read`; uptime/reset, losses, overruns, application-stack minimum and network state |

| Read result | Network meaning |
| --- | --- |
| Success | Commit scratch copy; clear that board's unavailable bit |
| `E_TMOUT` | Genuine contention; count busy, retain old values with fresh bit clear |
| `E_OBJ` / `E_NOEXS` on a main board | Unavailable; set its bit and increment unavailable aggregate |
| No next secondary record | Not proof that its primary board is absent |

Sonar history returns `E_OBJ` if unready, `E_TMOUT` on writer conflict and
`E_NOEXS` if no next entry exists. Empty history is not busy. Main-board
reads alone control the unavailable mask. Ages saturate at 65535 ms;
missing values are unknown, not zero. Unchanged source sequence does not
refresh an old sample, including at TIMER wrap. Read time is taken after
preemptible copies to avoid false unsigned ages.

| Freshness setting | Limit |
| --- | ---: |
| Line, also requires validity | 10 ms |
| Motion | 20 ms |
| Vehicle | 30 ms |
| Health | 2500 ms |
| KR1 echo in state | Unknown after failed/busy read or age >100 ms |

Health uses producer uptime milliseconds, so a stopped producer cannot look
young after the 71-minute microsecond wrap. Its counters can lag a failed
source read. Stack minimum covers registered application regions only,
not USB/radio/interrupt stacks.

### Events and commands

| Queue | Capacity | Full/unavailable behavior |
| --- | ---: | --- |
| Live vehicle events | 16 records | Nonblocking loss count; never cancels navigation |
| Normal network commands | 8 records | Reject/count; no direct motion call |
| ESTOP | 1 record | Dedicated, consumed first; repeated stops coalesce/count |
| Local RAM messages | 4096 bytes | Producer error/loss; existing local logging safety policy applies |

All sends use `TMO_POL` (return at once). Event sequence advances before
enqueue, so missing events leave a gap. Completed obstacle/detour/summary
boards keep last records instead of filling the rare-event queue.

Vehicle processes at most one command per step. ESTOP cancels pending
intentions and drains old ordinary commands. CALIBRATE/START are idle-only;
START also needs learned calibration. CALIBRATE returns idle; START delays
1 s, then navigates. Network DUMP is allowed while driving. The last eight
non-ESTOP request IDs suppress duplicates in RAM only. Command-decision
events contain opcode/status, not an acknowledgement of a request ID.

A valid configured 16-byte token is required for every command, including
ESTOP. Without a valid nonzero/non-example token, only a correctly shaped
22-byte ESTOP is accepted, with token unchecked. There is no encryption or
replay protection. Token rules and units: [command spec](MQTT_WIRE.md#command).

## Publication settings

| Setting | Current value / meaning |
| --- | --- |
| Connection/subscription retry | 3000 ms; subscription retries independently |
| Keepalive | 5 s; retained binary ONLINE/OFFLINE status with explicit six-byte will |
| Status/summary retry | 500 ms; reconnect republishes the last summary |
| Routine work per poll | Four routine frames; at most two sonar records within that budget |
| Separate bounded work | Status/summary service and log stream: at most one live batch + two DUMP frames |
| Wi-Fi requested sleep | 2 ms; observed cadence about 20 ms because the kernel tick/delay adds time |
| Poll metrics | All polls, including link down; wall time includes higher-priority preemption, not CPU percentage |

| Mode | Select when | State | Heartbeat | Live log / sonar |
| --- | --- | --- | --- | --- |
| RUN | Calibration, start delay, navigation, stop, plus 2 s grace | 200 ms | 1000 ms | Publish |
| IDLE | Idle beyond grace | 1000 ms | 5000 ms | Pause |

Sensors and local logging do not slow down in IDLE. TEST/BENCH have no
vehicle consumer and remain RUN. A busy phase read keeps the previous mode.
IDLE skips sonar history to newest without counting overflow; RUN sends
only later results. Sequence gaps across IDLE are expected. Resuming RUN
makes state/heartbeat due immediately and live log jumps to newest rows.
Events, summaries, status, commands and DUMP still work in both modes.
The mode uses state-v5 bit0 at offset70 and heartbeat-v4 flag6; no extra bytes.
See [publication modes](MQTT_WIRE.md#publication-modes).

## RAM log ownership

Telemetry30 alone owns 4501 ×16-byte KR1 rows (72016 bytes) and 512
×12-byte VE events (6144 bytes). BEGIN resets indices/generation, not
lifetime losses. It captures the producer-loss baseline before delayed
consumption. ROW/EVENT/INFO/DUMP messages copy values, never pointers.
A full local queue/storage counts loss. The vehicle's existing policy stops
an active session on a producer failure or storage-full report. This is a
**local logger failure**, not a network failure.

GP20 requests a local USB dump with copied diagnostics. Busy is reserved
before enqueue; no new session starts while queued/active. The logger
formats rows/events and yields 20 ms every eight records, then prints
`tm_usb_dropped` total and dump delta. After the counts line it prints one
`[robot] hump ...` line from the last finished run (12 B board copy, not a
larger INFO message). No control task formats the saved
arrays. A reset destroys RAM data. KR1 remains 128 bits; there is no spare
barcode byte. [Kernel archive](../archive/kernel/KERNEL_ROBOT.md#kr1-rows-and-ve-events)
keeps the exact local KR/VE text description; the binary topic layout is
[RAM log](MQTT_WIRE.md#ram-log).

### Live stream and DUMP

| Stream | Rule |
| --- | --- |
| Live, QoS0 | Freshness first; send ten waiting rows or a partial batch after 100 ms; one batch per poll |
| Live lag/reconnect | More than 100 rows behind, or reconnect: jump to newest ten and report skipped rows |
| Current production | One local row every 20 ms, at most about 50 rows/s; older 100-row/s examples are synthetic stream loads |
| DUMP, QoS1 | Freeze generation and committed row/event totals; BEGIN, batches of up to twelve, END |
| DUMP progress | Up to three batches in flight; advance only in acknowledged order; reconnect rewinds to acknowledged cursor |
| New generation | Abort the old DUMP instead of mixing runs |

QoS0 means no delivery acknowledgement; QoS1 means at-least-once delivery
while the connection/session succeeds, so duplicates are possible. Live
frames use no QoS1 request slot and are served before refill. Network DUMP
does not set local USB busy or freeze append logging. A reset, new run,
prolonged outage or storage loss can prevent a complete refill.

Merge by boot/session, generation, kind and index. A nonzero random 32-bit
ROSC boot ID is initialized once. Extraction failure disables only MQTT,
not driving. It is not cryptographic or guaranteed unique. Receivers also
use OFFLINE-to-ONLINE and per-source uptime rollback for local sessions;
a broker disconnect alone is not a reboot. [Identity spec](MQTT_WIRE.md#boot-identity-and-sessions).

The v2 maximum batch payload is 228 bytes; its QoS1 frame uses 244 of the
255 usable bytes in the unchanged 256-byte lwIP output ring. Row age is
time since commit to RAM, not sensor age or end-to-end network delay.
`publish_ok` means accepted by lwIP, not delivered to a laptop. A disconnect
can discard pending lwIP callbacks, so the reconnect path explicitly rewinds.

## Results and limits

| Observation | Result | Evidence |
| --- | --- | --- |
| Simulated full 4501-row/512-event refill | 2.8 /14.2 /28.4 s at 20 /100 /200 ms round trip; not hardware | [Original design](../archive/s1-telemetry/KERNEL_MQTT.md#live-log-stream-and-dump-refill) |
| M1 live/refill, 2026-11-07 | 216/216 rows match USB; median reported age 20 ms; DUMP about 1 s | [S1 log](../../testing/subsystem1/TEST_LOG.md) |
| Reconnect/will | Broker restart about 2 s; power-cut OFFLINE about 7 s; AP-loss case not separately recorded | [S1 log](../../testing/subsystem1/TEST_LOG.md) |
| Idle-mode run, `5e4e8ad` | Idle 5.4 min: state1/s, heartbeat5 s, no live/sonar; CALIBRATE restored full rate in the same second; idle DUMP 125 rows/8 events, misses0 | [S1-U08](../../testing/subsystem1/TEST_LOG.md) |
| Idle ESTOP, 2026-11-07 | Accepted, fault4; calibration cleared latch; two starts travelled 322/354 mm, stopped by dashboard ESTOP | [S1-U08](../../testing/subsystem1/TEST_LOG.md) |

No network latency or delivery deadline is a motion guarantee. Full
all-load stack bounds and independent CPU accuracy remain unproved.
[S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md) and
[integration TEST_LOG](../../testing/integration/TEST_LOG.md) separate each
measured revision from pending checks.

Full original design, throughput assumptions and historical results:
[MQTT](../archive/s1-telemetry/KERNEL_MQTT.md),
[RAM owner](../archive/s1-telemetry/KERNEL_TELEMETRY.md),
[STEP 8 evidence](../archive/MTK3_STEP8_RECORD.md).
