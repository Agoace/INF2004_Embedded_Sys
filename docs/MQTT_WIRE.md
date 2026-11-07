# Robot MQTT binary protocol

## Scope

The pure C codec is [mqtt_wire.h](../app_program/subsystem1/mqtt_wire.h) and
[mqtt_wire.c](../app_program/subsystem1/mqtt_wire.c). The receiver and command
client is [mqtt_robot.py](../scripts/mqtt_robot.py). No codec function uses
kernel, radio, or TCP/IP headers. The sonar service has no MQTT dependency.

Every message starts with version `1`. Integers use little-endian byte order.
Signed fields use two's-complement representation. Offsets below are in bytes,
from zero. Wire messages do not use C structure layout or C bit-fields.
Send exactly the specified number of bytes. Reserved bits and bytes are zero.
The Python decoder rejects incorrect lengths, unknown versions, and reserved
bits. Each topic has its own layout; there is no common message-type byte.

| Topic | Bytes | Purpose |
| --- | ---: | --- |
| `robot/state` | 48 | Current robot state and sensor ages |
| `robot/sonar` | 32 | One completed sonar history result |
| `robot/event` | 16 | Vehicle event, including command decisions |
| `robot/heartbeat` | 100 | Link state, health and loss counters |
| `robot/log` | 32 | One paced RAM dump frame |
| `robot/cmd` | 22 | Command, QoS 1, not retained; token policy below |

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
6 command decision. For kind 6, detail is 0 accepted, 1 not idle,
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
| 98 | u16 | Reserved zero |

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

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | Version |
| 1 | u8 | Kind: BEGIN 0, ROW 1, EVENT 2, END 3, ABORT 4 |
| 2 | u16 | Reserved zero |
| 4 | u32 | Dump ID, equal to command request ID |
| 8 | u32 | RAM run generation |
| 12 | u32 | Frame index |
| 16–31 | bytes | Body, below |

| Kind | Body |
| --- | --- |
| BEGIN | u32 rows total, events total, loss total, zero |
| ROW | Literal 16-byte KR1 row |
| EVENT | Literal 16-byte event record, with run-relative flag |
| END | u32 rows sent, events sent, loss total at begin, zero |
| ABORT | u32 reason, rows sent, events sent, zero; reason 1 means new run generation |

The producer captures generation and high-water counts when a dump starts.
New rows can still append; the buffer is not frozen. A generation change
aborts the dump rather than mixing runs. The limit is one log frame per 50 ms;
state, event, and sonar traffic have priority. A dump does not exclude driving.
Use dump ID, generation, and index to group frames and detect gaps.

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
