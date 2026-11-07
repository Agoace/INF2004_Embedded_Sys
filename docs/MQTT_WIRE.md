# Robot MQTT binary protocol

## Scope

The pure C codec is [mqtt_wire.h](../app_program/subsystem1/mqtt_wire.h) and
[mqtt_wire.c](../app_program/subsystem1/mqtt_wire.c). The receiver and command
client is [mqtt_robot.py](../scripts/mqtt_robot.py). No codec function uses
kernel, radio, or TCP/IP headers. The sonar service has no MQTT dependency.

Each message starts with its version byte: state `3`, heartbeat `2`, and
all other implemented messages `1`. Integers use little-endian byte order.
Signed fields use two's-complement representation. Offsets below are in bytes,
from zero. Wire messages do not use C structure layout or C bit-fields.
Send exactly the specified number of bytes. Reserved bits and bytes are zero.
The Python decoder rejects incorrect lengths, unknown versions, and reserved
bits. Each topic has its own layout; there is no common message-type byte.

| Topic | Bytes | Purpose |
| --- | ---: | --- |
| `robot/state` | 72 | State v3, ages, distance, duty and encoder odometry |
| `robot/sonar` | 32 | One completed sonar history result |
| `robot/event` | 16 | Vehicle event, including command decisions |
| `robot/heartbeat` | 100 | Link state, health and loss counters |
| `robot/log` | 32, or 32 + 16 x count | Live RAM-log rows (QoS 0) and DUMP refill (QoS 1) |
| `robot/cmd` | 22 | Command, QoS 1, not retained; token policy below |
| `robot/obstacle` | 20 | Completed scan profile, QoS 0, not retained |
| `robot/detour` | 32 | Finished detour, QoS 0, not retained |
| `robot/summary` | 44 | End-of-run totals, QoS 1, retained |
| `robot/status` | 2 | Online/offline and last will, QoS 1, retained |
| `robot/barcode` | 20 | Reserved; not implemented |
| `robot/imu` | 12 | Reserved; 4-byte header plus 8-byte snapshot; not implemented |

## State

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Flags, below |
| 2 | u8 | State in bits 0–3; module in bits 4–7 |
| 3 | u8 | Motion faults |
| 4 | u32 | Uptime, ms |
| 8 | u32 | State sequence |
| 12, 14, 16 | u16 each | Line, motion, vehicle age, ms |
| 18 | u16 | Raw barcode ADC sample, low 12 bits only |
| 20–35 | bytes | Existing KR1 row, unchanged |
| 36, 40 | i32 each | Left, right encoder count |
| 44 | u32 | Normalised L in bits 0–9; C in bits 10–19; upper bits zero |
| 48, 52, 56 | i32 each | Left, right, mean travel, signed mm since motion init |
| 60, 62 | i16 each | Left, right signed PWM duty, permille |
| 64 | i32 | Encoder heading, 0.01 degree, positive left, since motion init |
| 68 | i16 | Encoder turn rate, 0.1 degree/s, positive left |
| 70 | u16 | Reserved zero |

State v1 has 48 bytes; v2 has 60 bytes (the three distance fields).
The client can accept all three; firmware sends v3 at the unchanged 5 Hz rate.
Mean distance truncates toward zero. All added values inherit motion age and
freshness. A busy motion read is not a new measurement.

Distance uses the existing 12414 counts/metre calibration. Heading keeps the
controller's centidegree precision: one encoder count is about 0.0416 degree,
so 0.01 degree wire units do not discard encoder resolution. Turn rate uses
`(right_mm_s-left_mm_s)*12414/2405`, rounded, in 0.1 degree/s units. The existing
2405 counts/degree-times-100 constant includes the calibrated 111 mm wheel
track. These are encoder estimates, not IMU measurements; wheel slip is not
corrected. Turn rate inherits the integer wheel-speed resolution.

Flags: bit 0 line valid; 1 line fresh; 2 motion fresh; 3 vehicle fresh;
4 line read busy; 5 motion read busy; 6 vehicle read busy; 7 robot active.
Fresh limits are 10 ms for line, 20 ms for motion, and 30 ms for vehicle.
A retained sample after a busy read must have its fresh bit clear, even if
its age is below the limit. Ages saturate at 65535 for missing or old data.
Raw barcode data does not imply that a barcode decoder is implemented.

KR1 remains the existing 128-bit row. The Python client emits its raw hex and
decoded time, state, module, raw L/C, position, requested and measured wheel
speeds, echo, servo angle, events, and flags. KR1 time has 20 ms units and is
run-relative; it is not the top-level uptime. Top-level state and module are
intentional duplicates. KR1 does not contain the ages, barcode, counts, or
normalised L/C fields added here.

## Sonar

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Status bits 0–1; valid bit 2; pending bit 3; upper bits zero |
| 2, 3 | i8 each | Result angle, last trigger angle, degrees |
| 4 | u32 | History publication sequence |
| 8, 12 | u32 each | Request generation, request ID |
| 16, 20 | u32 each | Actual trigger time, result timestamp, ms |
| 24 | u16 | Existing packed ultrasonic result |
| 26 | u16 | Reserved zero |
| 28 | u32 | Physical ping sequence |

Status: EMPTY 0, ECHO 1, DEADLINE 2, CANCELLED 3. ECHO includes sensor error
results, not only valid distances. The packed echo has distance in bits 0–8,
status in 9–10, and streak in 11–13. Echo status 0 is valid distance; 1 is no
rising edge; 2 is stuck high; 3 is no return. A distance is not valid when status is
nonzero. The record valid flag also includes the service deadline check.

The sonar task appends echo, deadline, and cancellation publications to a
16-entry history ring. Trigger and ACK operations do not append entries.
A separate sequence counter permits loss detection; it wraps modulo 2^32.
A reader starts with cursor zero. Each successful read returns one next
entry. If entries were evicted, the read returns the oldest retained entry
and the number missed. Readers must not lag by 2^31 publications.

The history and current-board readers make one bounded attempt. They never
spin or wait for the lower-priority sonar writer. History returns `E_OBJ`
when busy or unready, `E_TMOUT` when stable with no new entry, `E_PAR` for a
null argument, and `E_OK` for an entry. Busy leaves cursor and output unchanged;
missed is zero on busy or no-new. The current-board try-read returns `E_TMOUT`
when busy and does not consume the vehicle's result notification. History
reads do not depend on a successful current-board read. Atomic aligned
32-bit loads and stores protect payload words and sequence guards. No atomic
read-modify-write operation or `libatomic` helper is required on Cortex-M0+.

## Event

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1, 2, 3 | u8 each | Kind, from, to |
| 4 | u32 | Event sequence |
| 8 | u32 | Uptime, ms, or run time when flag bit 0 is set |
| 12 | u16 | Event word |
| 14 | u8 | Detail |
| 15 | u8 | Bit 0 run-relative time; upper bits zero |

Kinds: 1 outcome, 2 module change, 3 ESTOP/fault, 4 scan complete, 5 run end,
6 command decision; 7 JUNCTION (reserved, not emitted). For kind 6, detail is 0 accepted, 1 not idle,
2 uncalibrated, 3 log busy, 4 duplicate, or 5 backend unavailable. The word
contains opcode in bits 0–7 and detail in bits 8–15. Only validated, queued
commands reach the vehicle and produce these events. Bad tokens, malformed
commands, and full command queues increment `command_lost` without a vehicle
event. Valid ESTOP requests are accepted in every state; repeated requests
can coalesce when the dedicated ESTOP queue is already full.
The vehicle is the sole producer. It assigns a monotonic
sequence before enqueue, so dropped events cause sequence gaps. Live events
use uptime and flags zero. RAM-log event bodies use run-relative time,
flag bit 0 set, and the RAM event index as their sequence.

## Heartbeat

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Flags, below |
| 2 | u8 | Reset reason: 0 power/RUN, 1 watchdog, 2 other |
| 3 | u8 | Reserved zero |
| 4 | u32 | Uptime, ms |
| 8 | u32 | Connect attempts |
| 12, 16, 20 | u32 each | Telemetry, event, command loss counts |
| 24 | u32 | Sonar history entries missed |
| 28 | u32 | Vehicle overruns |
| 32, 36 | u32 each | Motion late, motion missed |
| 40 | u16 | Minimum free application stack bytes |
| 42 | u16 | Reserved zero |
| 44 | u32 | Maximum MQTT wrapper execution time, us |
| 48, 52 | u32 each | Publish success, failure counts |
| 56, 60, 64, 68, 72 | u32 each | Line, sonar, motion, vehicle, health busy-read counts |
| 76 | u32 | Latest sonar history sequence |
| 80 | u16 | Health age, ms; saturates at 65535 |
| 82 | i8 | Wi-Fi link status |
| 83 | u8 | MQTT state: disconnected 0, connecting 1, connected 2, subscribed 3 |
| 84 | u32 | Maximum Wi-Fi full-loop start-to-start interval, us |
| 88 | u32 | Number of interval samples |
| 92 | i32 | Wi-Fi startup status |
| 96, 97 | u8 each | Requested, observed Wi-Fi task priority |
| 98 | u16 | v2 CPU load, permille 0–1000; 65535 unknown (v1 reserved zero) |

Heartbeat v2 remains 100 bytes. The load includes task and interrupt time on
core 0; with SMP=0 this includes Wi-Fi. It is not a per-task load. The
lowest-priority task counts the same loop used in a 4 ms, interrupts-disabled
calibration before application startup. Health samples it each second. Other
work reduces the count; the pure arithmetic converts that reduction to load.
The meter replaces WFI sleep and increases power use. It does not preempt any
other task. Host arithmetic checks bound rounding to 1 permille; target
measurement error and the idle-board expected range are **not established**.
S1-U07 prints the measured load and calibration rate in the TEST image.
No target measurement is claimed.

Flags: bit 0 Wi-Fi up; 1 MQTT connected; 2 subscribed; 3 RAM dump active;
4 restricted commands configured (the wire/CSV name remains `commands_enabled`);
5 health fresh. Upper bits are zero. A retained/busy
health sample must have bit 5 clear. Busy counters count bounded-read failures;
they do not mean that a consumer waited for a lock.

The stack minimum covers registered application stacks only. It does not
measure the template Wi-Fi or USB task stack. `mqtt_hook_max_us` measures only
the MQTT wrapper, not a full Wi-Fi poll. The poll-gap metric uses actual
full-loop start timestamps in the Wi-Fi task, including polls while the link
is down. Its maximum and sample count are lifetime values since boot and do
not reset on a link outage. The interval includes higher-priority work; it is
not CPU duration or CPU load. A separate resettable diagnostic window does
not reset these lifetime counters.

## Obstacle profile

`robot/obstacle` reports the last completed scan. The vehicle task writes a
sequence-numbered board, not the event queue: an unavailable network cannot
fill the event queue with profile data. Only the last profile is retained in
RAM; a gap in scan ID shows missed scans. One bounded read never waits for
the writer. SCAN, side selection and detour control are unchanged.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version 1 |
| 1 | u8 | Flags: closest valid bit 0, too near bit 1, left blocked bit 2, right blocked bit 3 |
| 2 | u8 | Decision: NONE 0, RIGHT 1, LEFT 2 |
| 3 | i8 | Closest angle, degrees; negative right |
| 4 | u32 | Scan ID since boot |
| 8 | u32 | Completion uptime, ms |
| 12 | u16 | Closest distance 0–510 mm; 511 none; 0 too near |
| 14 | u16 | Existing packed PROFILE word |
| 16, 17 | u8 each | Right, left free step, 0–6; 7 blocked |
| 18, 19 | u8 each | Right, left clearance angle, step times 5 degrees; 255 blocked |

Flag bits 4–5 reserve scan stage: 0 not reported (current firmware), 1 coarse,
2 fine, 3 reserved. Bits 6–7 are zero. Equal closest distances prefer the
point nearest straight ahead, then the right point. PROFILE does not measure
obstacle width. Width is **unavailable**, not zero or an estimated size.

## Detour summary

`robot/detour` is one last-record board per completed or aborted detour.
Sequence gaps indicate overwritten records. QoS 0 does not guarantee delivery.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version 1 |
| 1 | u8 | Outcome: DONE_LINE 3, DONE_NO_LINE 4, FAILED 7, ABORTED 8 |
| 2 | u8 | Side: NONE 0, RIGHT 1, LEFT 2 |
| 3 | u8 | Bit 0 lateral estimate valid; other bits zero |
| 4 | u32 | Detour sequence since boot |
| 8, 12, 16 | u32 each | Start, end, duration, ms |
| 20 | u32 | Centre path length driven, mm; reverse travel adds to length |
| 24 | u16 | Maximum absolute lateral offset from start heading, mm |
| 26 | u16 | Outcome payload |
| 28 | u32 | Related obstacle scan ID; 0 unknown |

DONE_NO_LINE payload uses the **line** side enum: UNKNOWN 0, LEFT 1, RIGHT 2.
This is opposite to the obstacle enum. FAILED payload uses `dt_fail_t`: NONE
0, NO_SIDE 1, TURN_LIMIT 2, TIME 3, SENSOR 4, TOO_NEAR 5, NO_LINE 6.
ABORTED means the run ended while DETOUR was active.

The observer integrates signed average wheel-count steps, using encoder
heading relative to the detour start. A Q15 sine table with 1-degree entries
and interpolation supplies the lateral projection. Host checks bound its
absolute sine error below 0.0001. Offset is an encoder estimate of deviation
from the heading at entry, **not a measurement of distance to the physical
line**. It does not compensate for wheel slip or irregular line shape.

## Run summary

`robot/summary` is QoS 1 and retained. The last-record vehicle board does not
block driving. The publisher retries after 500 ms on publish/ACK failure and
publishes the latest completed run again after reconnect. Multiple runs
without network service can overwrite older summaries; run-ID gaps show this.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version 1 |
| 1 | u8 | Flags: barcode/hump implemented bit 0 (zero), speed error valid bit 1, detour aborted bit 2 |
| 2 | u8 | Stop source: unknown 0, GP21 1, MQTT 2, fault 3, limit 4 |
| 3 | u8 | Stop reason, below |
| 4 | u32 | Run ID since boot |
| 8, 12 | u32 each | Start uptime, duration, ms |
| 16 | u32 | Centre path length, mm |
| 20, 22 | u16 each | Line-loss outcomes, obstacle outcomes |
| 24, 26 | u16 each | Detours completed (with/without line), succeeded (with line) |
| 28 | u32 | Total detour time including failed/aborted detours, ms |
| 32 | u16 | Detours started |
| 34 | u16 | Maximum steady-command wheel speed error, mm/s; 65535 unavailable |
| 36 | u8 | OR of motion fault bits seen |
| 37, 38 | u8, u16 | Shell fault and detail, existing `vsm_fault_t` contract |
| 40, 41 | u8 each | Reserved barcode success/failure counts (zero) |
| 42 | u16 | Reserved hump peak (65535 unavailable; units not implemented) |

Stop reasons: unknown 0; GP21 1; MQTT ESTOP 2; 90 s limit 3; shell left
RUNNING 4; vehicle deadline 5; motion wake lost 6; motion status failure 7;
line snapshot failure/stale 8; line samples lost 9; sonar failure 10;
telemetry failure 11; other 12. Sources describe the run-ending condition,
not a unified fault mask. Motion faults preserve existing `MC_FAULT_*` bits.

Speed error compares commanded versus measured wheel speed only after the
command has stayed unchanged for at least 300 ms. This excludes early ramp
samples; it is not an independent motor-accuracy measurement. The observer
never changes commands, detector inputs, SCAN or navigation state.

## Link status and last will

`robot/status` is two bytes: version 1, then ONLINE 1 or OFFLINE 2. Both bytes
are nonzero. CONNECT supplies the OFFLINE will with explicit `will_msg_len=2`,
QoS 1 and retain. This lwIP implementation otherwise uses `strlen` when the
length is zero; the explicit length avoids string/binary ambiguity. After
CONNACK the robot publishes retained ONLINE at QoS 1, retrying failures.

Keepalive is 5 seconds. A broker can detect a silent link at 1.5 times that
interval (about 7.5 seconds after the last control packet, plus broker timer
and network delivery delay). This is not a guaranteed wall-clock deadline.
The lwIP cyclic timer runs every 5 seconds; an idle connection sends at most
one 2-byte MQTT PINGREQ per 5 seconds, plus a 2-byte PINGRESP and TCP/IP/link
overhead. Normal 5 Hz telemetry keeps the connection active, so idle pings
are normally unnecessary. Reconnect replaces retained OFFLINE with ONLINE.
Retained ONLINE alone does not prove a current connection: require recent
state messages too. Broker restart and stale retained data can mean unknown
or silent status. MQTT ESTOP remains best effort; GP21 is the real ESTOP.

## Reserved barcode and IMU formats

These formats reserve topics only. Firmware does not publish them or execute
barcode commands. Clients ignore unknown versions/kinds and display **not
implemented**. They must not infer sensor availability from a reserved topic.

### Barcode

`robot/barcode`, 20 bytes, future QoS 1, not retained:

| Offset | Type | Field |
| --- | --- | --- |
| 0, 1 | u8 each | Version 1, kind 1 (result) |
| 2 | u8 | Flags: detected 0, decoded 1, unreadable 2, command issued 3, executed 4; upper bits zero |
| 3 | u8 | Three-bit action code; 0 reserved for internal line-loss semantics |
| 4, 5 | u8 each | Agreeing votes, total reads |
| 6 | u8 | Direction: unknown 0, forward 1, reverse 2 |
| 7 | u8 | Navigation command (enum not yet defined) |
| 8 | i16 | Executed result, e.g. angle reached; final units not yet defined |
| 10 | u16 | Reserved zero |
| 12, 16 | u32 each | Sequence, uptime ms |

Action and navigation enums remain undecided. Do not treat raw values as
implemented instructions.

### IMU

`robot/imu`, 12-byte frame, future QoS 0, not retained: version 1 at byte 0,
kind at byte 1. Kind 1 reserves bytes 2–3 as zero and contains the **8-byte**
snapshot from [Week 8 notes section 11](week8_change_requirement_1_notes.md#11-buddy-4--imu--hump-data):

| Offset | Type | Field |
| --- | --- | --- |
| 4 | u32 | Pitch bits 0–7, heading 8–15, turn rate 16–23, hump state 24–25, collision 26, valid 27, spare 28–31 |
| 8 | u32 | Height bits 0–7, peak 8–15, age/sequence 16–31 |

Two 32-bit words are 8 bytes, **not 6 bytes**. This snapshot is 8 bytes plus
a 4-byte MQTT header. Suggested pitch is signed degrees and heading is an
unsigned full-turn scale (360/256 degrees per unit). Turn-rate scale, hump
states, height/peak units, and age-versus-sequence remain undecided. Clients
show raw values until those units are approved. Higher internal heading
precision must be kept; do not derive fine turn rate from quantised heading.

Kind 2 is a motion event: byte 2 event, byte 3 zero, sequence u32 at 4, uptime
ms u32 at 8. Events reserve STATIONARY 1, ACCELERATING 2, TURNING 3, CLIMBING
4, DESCENDING 5, IMPACT 6. No such events are currently emitted.

## Command

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Opcode: ESTOP 1, DUMP 2, CALIBRATE 3, START 4 |
| 2 | u32 | Request ID |
| 6–21 | bytes | Raw 16-byte command token |

The decoder always checks version, exact length, and known opcode. Invalid
input leaves output unchanged. Token policy depends on the robot configuration:

| Robot configuration | ESTOP | DUMP, CALIBRATE, START |
| --- | --- | --- |
| Valid 32-hex-digit token with at least one nonzero byte | Matching token required | Matching token required |
| No token, example all-zero token, or invalid token | Accepted with any token field | Rejected and counted |

An unconfigured robot therefore accepts a well-formed 22-byte ESTOP without
checking its token. The 16-byte token field is still required. A configured
robot checks the token for **every** opcode, including ESTOP. Comparison visits
all 16 bytes. There is no separate enable setting. The decoder's
`commands_enabled` parameter and heartbeat flag retain their existing names,
but mean that restricted commands have a valid token configured, not that
unconfigured ESTOP is disabled. The expected-token pointer must remain valid
in both modes.

`rm_command_allowed()` permits ESTOP and DUMP in any state after the token
policy check; CALIBRATE and START require idle. Request-ID duplicate handling
belongs to the integration layer, not the codec. A valid ESTOP remains
accepted even when repeated. The default all-zero example selects ESTOP-only
mode, not unrestricted access to other commands.

A configured token prevents accidental commands on a trusted network. It provides no
encryption and does not prevent capture or replay. Do not expose this service
to an untrusted network. An unconfigured robot permits any network peer to
request ESTOP. A broker publish acknowledgement does not prove that
the robot accepted or executed a command; inspect the command-decision event.
Remote ESTOP is best-effort only. Use GP21 for the physical ESTOP. A network
failure increments diagnostic counters; it does not cause a motor/vehicle
fault or stop. Boot does not wait for a radio, link, or broker.

## RAM log

`robot/log` carries two things. The **live stream** (QoS 0) follows the newest
KR1 rows for a dashboard. A **DUMP** (QoS 1) refills the whole run later.
Both use the same batch frame. The live stream prefers fresh data over
complete data. Completeness comes from DUMP, merged by index.

### Batch frame (kinds ROWS 5 and EVENTS 6)

Length is 32 + 16 x count bytes; count is 1 to 12. Record i has index
`first_index + i`. There is no per-record index byte.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Kind: ROWS 5 (KR1 rows) or EVENTS 6 (event records) |
| 2 | u8 | Count of records, 1 to 12 |
| 3 | u8 | Flags, below |
| 4 | u32 | Stream ID: 0 for the live stream; the DUMP ID for a refill |
| 8 | u32 | RAM run generation |
| 12 | u32 | Index of the first record |
| 16 | u32 | Publish time: robot uptime, ms, when the frame was built |
| 20 | u32 | Newest-row time: robot uptime, ms, when the newest row in the RAM log was logged. Zero for a refill |
| 24 | u32 | Rows in the RAM log when the frame was built |
| 28 | u32 | Skipped rows just before `first_index` (live only; zero in a refill) |
| 32+ | bytes | Count records of 16 bytes: literal KR1 rows, or event records (event flag bit 0 set: run-relative time) |

Flags: bit 0 REFILL (DUMP frame, QoS 1); bit 1 NEW_GENERATION (first live
frame after a new run generation began); bit 2 JUMPED (live cursor jumped to
the newest rows). Other bits are zero.

**Data age.** For a live frame, `publish - newest` is the age in ms of the
newest row in the RAM log at publish time. `log_rows - (first_index + count)`
is how many newer rows exist but are not yet published. A dashboard shows both.
Both times use the robot clock, so no clock sync is needed. They exclude the
network and broker delay and the laptop receive delay. The newest-row time is
the time the telemetry task stored the row, not the time of the sensor sample.
Times are 32-bit ms and wrap after 49.7 days; subtract with wraparound.

**Live stream rules.**
- The Wi-Fi task reads rows from the RAM log with its own cursor. Driving and
  telemetry tasks do not know about it.
- It publishes when 10 rows are waiting, or when at least one row has waited
  100 ms. At 100 rows/s this is about 10 rows every 100 ms.
- If the cursor lags more than 100 rows (about 1 s), or after an MQTT
  reconnect, it jumps to the newest rows. It skips the older rows without
  replay. The next frame has JUMPED set and `skipped_before` equal to the
  number skipped.
- On a new run generation, the cursor restarts at index 0. The first frame has
  NEW_GENERATION set. `skipped_before` is the number of old-generation rows
  observed but never published (a lower bound).
- QoS 0 gives no delivery confirmation. A lost frame leaves an index gap that
  the client must report. A failed publish is retried later with the same
  cursor.
- Each frame is at most 224 bytes of payload (10 rows: 192 bytes).

### Control frames (kinds BEGIN 0, END 3, ABORT 4)

These 32-byte frames are QoS 1. Kinds ROW 1 and EVENT 2 (one record per frame)
are retired and never sent.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Kind: BEGIN 0, END 3, ABORT 4 |
| 2 | u16 | Reserved zero |
| 4 | u32 | Dump ID, equal to command request ID |
| 8 | u32 | RAM run generation |
| 12 | u32 | Frame index (zero) |
| 16–31 | bytes | Body, below |

| Kind | Body |
| --- | --- |
| BEGIN | u32 rows total, events total, loss total, zero |
| END | u32 rows sent, events sent, loss total at begin, zero |
| ABORT | u32 reason, rows sent, events sent, zero; reason 1 means new run generation |

### DUMP

A DUMP command (`DUMP` opcode) starts one refill. The robot freezes the
generation and the row and event counts when the dump opens. Rows appended
later are not part of it, and the live stream shows them. The frames are:
BEGIN (wait for PUBACK); batches of up to 12 rows, then up to 12 events, with
REFILL set and the dump ID as stream ID; END after every batch is acknowledged.
Up to three QoS 1 batches are in flight. The robot advances its refill cursor
only on PUBACK. After an MQTT reconnect it resends the unacknowledged batches
from RAM, so a client must drop duplicates by (generation, kind, index). A new
run generation sends ABORT and ends the dump. One dump runs at a time; a
second request while one is active is refused. A dump needs about 14 s for a
full log at 100 ms round-trip time (about 28 s at 200 ms). Live frames have
priority for each poll, and use no QoS 1 request slot, so a dump does not
delay them. Merge refill records into the live CSV by (generation, index),
and report any index missing after END. A dump never stops driving or logging.

## Client

The client requires Paho MQTT 2.x and uses `CallbackAPIVersion.VERSION2`.
It connects to an existing broker; it does not start a broker.

```sh
python3 scripts/mqtt_robot.py --host BROKER --csv robot.csv
python3 scripts/mqtt_robot.py --host BROKER --command estop
python3 scripts/mqtt_robot.py --host BROKER --command estop --token-file /private/robot-token
python3 scripts/mqtt_robot.py --host BROKER --command dump --token-file /private/robot-token --request-id 123
```

`--token-file` is optional only for ESTOP. Without it, the client sends 16
zero token bytes. This works only on an unconfigured robot. A configured robot
requires its correct token file even for ESTOP. START, CALIBRATE, and DUMP
always require a token file on the client and a valid configured token on the
robot. Command publication is not proof of acceptance.

When supplied, the token file must be a regular file owned by the current user, with mode
`0600`, and exactly 32 hex digits with an optional final newline. Symbolic
links are rejected. Do not put the token in a command-line argument, log, or
repository file. The client never prints token bytes. A command uses a random
32-bit request ID unless one is supplied. It sends with QoS 1 and retain false.
The default command mode exits after the broker acknowledges publication;
run a separate receiver to record command decisions or a RAM dump.

The receiver subscribes to the five telemetry topics after each connection.
It writes one wide CSV table with topic, receive time, device times, sequence,
and decoded fields. Unused fields are empty. Receive time is Unix wall time
on the host, not a measured network latency. A disconnected receiver cannot
recover ordinary live messages; sequence gaps and loss counters show loss.
Malformed messages are discarded with an error on stderr. Decoding and tests
need no broker and do not import Paho.

## Host verification

```sh
cmake -S tests -B /tmp/robot-host-tests
make -C /tmp/robot-host-tests -j4 --output-sync=target
ctest --test-dir /tmp/robot-host-tests --output-on-failure
```

[test_mqtt_wire.c](../tests/test_mqtt_wire.c) checks C masks, byte order,
signed values, command policy, and invalid commands. It emits C-packed samples.
[test_mqtt_robot.py](../tests/test_mqtt_robot.py) decodes those samples in
Python, checks all topics, rejects malformed packets, checks CSV fields,
and checks private token files. No test uses a real credential or broker.
The scan branch also tests odd and changed guards, overflow, sequence wrap,
and history independence from the result notification.
