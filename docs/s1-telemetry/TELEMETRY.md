# Telemetry design

## What it is

Telemetry observes the robot. It does not own navigation, motors, the
watchdog or startup permission. The local RAM logger and optional MQTT
publisher are separate owners. MQTT is a message protocol through a broker;
a broker acknowledgement means receipt by the broker, not robot acceptance.

See [Operation](#operation-broker-recorder-and-dashboard) to run the tools,
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
barcode byte. `KERNEL_ROBOT.md` in the [kernel history](../archive/kernel/HISTORY.md#files)
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
| Simulated full 4501-row/512-event refill | 2.8 /14.2 /28.4 s at 20 /100 /200 ms round trip; not hardware | [History: KERNEL_MQTT](../archive/s1-telemetry/HISTORY.md#files) |
| M1 live/refill, 2026-11-07 | 216/216 rows match USB; median reported age 20 ms; DUMP about 1 s | [S1 log](../../testing/subsystem1/TEST_LOG.md) |
| Reconnect/will | Broker restart about 2 s; power-cut OFFLINE about 7 s; AP-loss case not separately recorded | [S1 log](../../testing/subsystem1/TEST_LOG.md) |
| Idle-mode run, `5e4e8ad` | Idle 5.4 min: state1/s, heartbeat5 s, no live/sonar; CALIBRATE restored full rate in the same second; idle DUMP 125 rows/8 events, misses0 | [S1-U08](../../testing/subsystem1/TEST_LOG.md) |
| Idle ESTOP, 2026-11-07 | Accepted, fault4; calibration cleared latch; two starts travelled 322/354 mm, stopped by dashboard ESTOP | [S1-U08](../../testing/subsystem1/TEST_LOG.md) |

No network latency or delivery deadline is a motion guarantee. Full
all-load stack bounds and independent CPU accuracy remain unproved.
[S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md) and
[integration TEST_LOG](../../testing/integration/TEST_LOG.md) separate each
measured revision from pending checks.

Full original design, throughput assumptions, STEP 8 evidence and the old
UDP tools: [telemetry history](../archive/s1-telemetry/HISTORY.md).

## Operation: broker, recorder and dashboard

Mosquitto is the broker on the operator laptop. The recorder saves decoded
robot observations to CSV. The dashboard shows live data and sends ESTOP,
DUMP, CALIBRATE and START. They are separate subscribers; the dashboard
itself does not save a CSV. See [MQTT_WIRE.md](MQTT_WIRE.md) for bytes and token rules.

Only the operator installs services, changes firewall rules, flashes or
runs powered tests. MQTT ESTOP is best effort. Use GP21 locally in the
normal robot; TEST motor steps do not consume remote commands or GP21.
Lift wheels for TEST. A lost network is not a robot stop mechanism.

### Broker

Use a trusted robot-only network. This firmware has no TLS encryption or
broker user/password authentication. Never expose port 1883 to the public
Internet or unrelated interfaces. Example Mosquitto configuration; replace
both network placeholders before use:

```text
listener 1883 <LAPTOP_ROBOT_IP>
allow_anonymous true
persistence false
```

Run the packaged service **or** one foreground process, not both:

```sh
mosquitto -c /path/to/robot-mosquitto.conf
```

If UFW manages the host firewall, the operator can add a scoped rule:

```sh
sudo ufw allow in on <ROBOT_INTERFACE> from <ROBOT_SUBNET> to <LAPTOP_ROBOT_IP> port 1883 proto tcp
```

Do not change default firewall policy or add a global allow rule. Check the
listener and active interface. The firmware broker address must be explicit.
Use that address in the tools below; a broker bound only to its LAN address
may not listen on 127.0.0.1.

### Private settings

| Setting | File / rule |
| --- | --- |
| Wi-Fi | `config/wifi_secrets.h`, ignored, mode 0600 |
| Broker/client/token | `app_program/subsystem1/robot_mqtt_secrets.h`, ignored, mode 0600; shape in adjacent `.example` |
| Broker IP/port | `ROBOT_MQTT_BROKER_IP`, `ROBOT_MQTT_BROKER_PORT`; default `0.0.0.0: 1883` does not connect |
| Client ID | `ROBOT_MQTT_CLIENT_ID`; unique if another robot shares the broker |
| Command token | `ROBOT_MQTT_COMMAND_TOKEN_HEX`; 32 random hex digits, not all zero |
| Laptop token | `~/.config/robot-mqtt.token`; owned regular file, 0600, 32 hex digits and optional newline; no symbolic link |

For first-time setup only, create both token copies without printing them.
This refuses existing files. Run at the repository root, then edit only the
broker/client fields privately:

```sh
umask 077
python3 - <<'PY'
import os
from pathlib import Path
import secrets
header = Path('app_program/subsystem1/robot_mqtt_secrets.h')
example = Path('app_program/subsystem1/robot_mqtt_secrets.h.example')
token_file = Path.home() / '.config/robot-mqtt.token'
token_file.parent.mkdir(parents=True, exist_ok=True)
if header.exists() or token_file.exists():
    raise SystemExit('Private settings already exist; do not overwrite them.')
token = secrets.token_hex(16)
text = example.read_text().replace('00000000000000000000000000000000', token)
for path, value in ((header, text), (token_file, token + '\n')):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        stream.write(value)
PY
```

Do not print, grep values from, commit or place tokens on a command line.
UF2/ELF files contain the settings too; keep them private. Fresh worktree
copies must retain mode 0600. Without a valid configured token, the robot
accepts only a shape-valid ESTOP (token unchecked). With a valid token,
**every** opcode including ESTOP requires a match. A trusted peer that has
the token can control the robot; this does not prevent capture/replay.

Build and flash only with approval: [BUILD.md](../kernel/BUILD.md).

### Start the recorder and page

Python needs `paho-mqtt` 2.x. From the repository root, in separate terminals:

```sh
python3 scripts/mqtt_robot.py --host <BROKER_IP> --csv robot-mqtt.csv
python3 scripts/mqtt_dashboard.py --host <BROKER_IP> --token-file "$HOME/.config/robot-mqtt.token"
```

Open `http://127.0.0.1: 8765/`. The dashboard binds only to loopback. The page
uses inline canvas/HTML and system fonts; there is no CDN or external asset.
A page can draw without Internet access, but live updates need the local
server and robot/broker link.

| Dashboard option | Default / effect |
| --- | --- |
| `--host`, `--port` | Broker, `127.0.0.1: 1883` by default |
| `--http-port` | Loopback HTTP port 8765 |
| `--token-file` | Server-only private token; without it only zero-token ESTOP is offered |
| `--allow-origin URL` | Repeatable exact proxy scheme/host/port; does not disable CSRF or content-type checks |

The recorder validates length/version/reserved bits, signs and units. It
subscribes again after reconnect and never records commands/tokens. The
[wire spec](MQTT_WIRE.md#client) lists CSV/session rules. It accepts old
state-v1..v5 and heartbeat-v1..v4 captures; invalid data does not update a panel.

#### Commands

Use a second terminal if not using page buttons:

```sh
python3 scripts/mqtt_robot.py --host <BROKER_IP> --command calibrate --token-file "$HOME/.config/robot-mqtt.token"
python3 scripts/mqtt_robot.py --host <BROKER_IP> --command start --token-file "$HOME/.config/robot-mqtt.token"
python3 scripts/mqtt_robot.py --host <BROKER_IP> --command dump --token-file "$HOME/.config/robot-mqtt.token"
python3 scripts/mqtt_robot.py --host <BROKER_IP> --command estop --token-file "$HOME/.config/robot-mqtt.token"
```

Commands use QoS1, never retain, and a random uint32 request ID unless
`--request-id` is supplied. Clear old retained `robot/cmd` data before tests.
Broker PUBACK is not robot acceptance: inspect command-decision events and
state. Bad tokens produce no decision event; they increment `command_lost`.
Queues are consumed only in the normal robot, not TEST/BENCH.

| Page action | Rule |
| --- | --- |
| ESTOP | Visible and enabled even with broker loss; delivery can fail. Restarted/stale pages block all commands |
| DUMP | Broker/token ready; not offline/silent, no pending decision or active DUMP; may run while driving |
| START/CAL | DUMP conditions plus fresh, idle, nonbusy vehicle state; two clicks within 4 s confirm |
| START in firmware | Also needs learned calibration; CALIBRATE returns idle |
| Decision wait | Up to 5 s; only ESTOP remains available while waiting |

State age limit is1 s in RUN/old firmware, 2.5 s in idle telemetry. Full page
security still applies to ESTOP. The firmware has the final idle/calibration
check even if a button was briefly enabled from old page data.

### Page and data

| Row | View |
| --- | --- |
| Header | Robot ONLINE, RUN/IDLE/FAULT, active LINE > SCAN > DETOUR > SEARCH; CAL/START/DUMP and large red ESTOP |
| 1 | ULTRASONIC (largest), LINE L/C bars and position, DRIVE requested/measured L/R speed |
| 2 | RUN PATH, LAST RUN (time, distance, detours, line losses, stop reason text) and HIGHEST HUMP |
| 3 | Collapsible HEALTH: CPU, Wi-Fi, timing, counters, full original detail panels |

Below800 px the same cards stack. State panels turn stale after 1 s (2.5 s idle);
health after 2.5 s (12 s idle). These display limits do not replace producer
fresh flags or motor safety checks. The newest state/heartbeat mode selects
RUN/IDLE. Idle state arrives at 1 Hz, heartbeat every 5 s; sonar/live log pause.
Their last data is labelled `idle (paused)`; the fan is held grey and its
age continues to increase. Silent remains a3 s state gap in both modes.

#### IMU and barcode reports

HEALTH includes IMU and Barcode cards. These cards use existing messages;
they do not require a new firmware image.

| Data | Source and display rule |
| --- | --- |
| Hump event run count, highest peak and side | `robot/event` kind 9; heights use 0.1 mm units on wire and mm on the page. The count is the reported run count, not the number of received messages. Duplicate delivery does not increase it. |
| Completed hump result and warnings | `robot/summary` v3; includes count, peak, side, missing IMU data, open hump at stop, long-hump rejection and curve gating. |
| Barcode ok / failed | `robot/summary`; zero is a reported count, not missing data. These are completed-run totals, not individual attempts. |
| Raw GP28 | `robot/state`; its age includes the line-board age. The board-valid row is separate from the raw value. RUN and idle age limits apply. |
| IMU attitude | Current state/status messages do not contain IMU attitude samples. State heading and turn rate are wheel odometry; do not use them as IMU values. Legacy `robot/imu` v1 raw fields still appear if received. |
| SHADOW | Deployment context only. Current messages do not confirm this mode on wire. A decoded letter or summary count does not prove that a command was issued. |

Each summary shows its run ID, boot/session, received age and retained-copy
warning when applicable. An earlier boot's summary stays labelled as history.
A new active run does not turn the previous completed summary into current
run totals. Hump events reset at an observed run start or new boot. A
retained hump event is not assigned to the current run.

Report age means time since receipt, not age of a continuous sensor sample.
The page marks historical cards `since report`; it does not expire completed
results as if they were samples. GP28, legacy barcode reports and legacy IMU
snapshots keep their own source ages. A new summary or state frame cannot
refresh an old IMU snapshot. Legacy IMU snapshot age is shown separately if
a newer motion event arrives.

The per-attempt barcode row and obstacle width/clearance rows are labelled
future slots. This panel change adds no decoder for their proposed message
versions. Existing clearance angles remain degrees. Future millimetre side
spans must not be described as guaranteed body clearance or passability.
A clipped width is only a lower bound.

The host tests use report fixtures reconstructed from decoded fields in
`mqtt_B120_1.csv` (source SHA256
`a2fc22e835c2b109c3cc0dfd73c55625fdb2b44083878ffc564bb425f4ede640`).
This is not a raw-packet recording. The CSV has no retain flag; tests apply
retained delivery to startup history explicitly. The fixtures check two
hump reports, 11.7 and 12.3 mm, a current summary of 0 barcode successes and
7 failures, and an earlier boot's separate 1-success/3-failure summary.
These are telemetry reports, not a new physical height measurement or a
hardware acceptance result. Host model/page tests do not change Gate1's
INCOMPLETE status or the physical GP28 FAIL status.

#### Ultrasonic fan

Forward is up, LEFT positive angle, RIGHT negative. Rings show0..511 mm;
lines at±30° mark the scan sector. The current received servo direction has
a faint **approximate ±20° cone**, measured2026-11-07. An echo below 258 mm
turns the beam red; the stop arc stays at 258 mm. This is a sensor view, not
a box/obstacle model.

- Recent echo dots fade over10 s, bounded to 128 observations. NO_RETURN is a
  distinct square at the range edge, not a measured511 mm echo.
- The last completed scan freezes13 received angle slots (-30..+30°, 5°
  steps). `?` is unknown; `observed x/13` states coverage. The report does
  not contain all 13 ranges. These samples are not an exact reconstruction
  of the firmware's combined two-read profile.
- Green profile rings mean NO_RETURN or range at least nearest+150 mm;
  red means nearer; gold marks the reported nearest point. The firmware's
  LEFT/RIGHT/NONE decision and chosen-side shade remain authoritative.
- Attempt1/3 is initial. Kind8/module5 shows REVERSING; the last profile
  remains visible during retry. Distances beyond511 mm clip at the edge.

LINE bars use0=floor, 1000=line; the -100..+100 slider clips only its drawing.
DRIVE uses blue requested/green measured signed mm/s bars, capped at±500;
text retains full values. Wheel duty is percent from wire permille.

#### Highest hump

HIGHEST HUMP shows the summary v3 hump result of the last run: peak in mm,
wheel side, hump count and warning flags. During a run a newer live kind 9
event shows the last hump so far. A LEFT-wheel hump is labelled `left
wheel (not proven)`. Peak 65535 shows `unavailable (no fresh IMU data)`;
a v1/v2 summary shows `not reported (summary vN firmware)`. A new run or a
new boot clears the live value. The detector and its limits are in
[IMU step 3](../s4-imu/IMU.md#step-3-hump-report).

#### Run path and retained history

The path is a known-inaccurate illustration, **not a requirement or map**.
It integrates only live measured wheel speeds with a111 mm track, from the
first received running row. New run/generation/boot resets it. DUMP,
duplicates and delayed old generations cannot move it. SCAN, DETOUR and
reverse are marked. Missing indexes, repeated times or gaps above 200 ms
break the line, with visible gap counts; unknown travel is not invented.
At most4600 points are kept. Wheel slip and missing early data cause drift;
there is no external position correction.

Log rows/refill deduplicate by boot/session, generation, kind and index.
A new boot clears live readings, chart baselines and pending commands, but
keeps already recorded rows. Prior-boot retained summaries are labelled as
history, never proof of online state. The local session also detects status
OFFLINE-to-ONLINE or per-source uptime rollback; broker disconnect alone is
not a reboot. Full rules: [boot identity](MQTT_WIRE.md#boot-identity-and-sessions).
The in-memory row store grows until the tool exits; restart after a long
session, after saving the recorder CSV.

### Remote view through Tailscale

Keep the server on127.0.0.1. With operator approval, Tailscale **Serve** can
forward a private tailnet port; do not use public Funnel. Example accepted
HTTP arrangement:

```sh
tailscale serve --bg --http=8080 http://127.0.0.1:8765
python3 scripts/mqtt_dashboard.py --host <BROKER_IP> \
  --token-file "$HOME/.config/robot-mqtt.token" \
  --allow-origin http://<laptop>.<tailnet>.ts.net:8080
```

Open that exact FQDN (full hostname). A short hostname is a different Host
and is refused with403. HTTP shows “Not secure” in the browser, although
Tailscale carries transport over WireGuard. For HTTPS, use
`tailscale serve --bg --https=8443 http://127.0.0.1: 8765` and allow the exact
`https://...: 8443` origin instead. Stop it with
`tailscale serve --https=8443 off`. Do not reset unrelated Serve entries.
Tailnet devices that can reach the page can use **all** buttons; restrict
access with Tailscale rules when required.

#### Security and restart

The token stays server-side. The page gets a separate random CSRF value
(a per-process check against commands sent by another site) and a public
server-instance ID. POST requires permitted Host/Origin, JSON content type
and matching `X-CSRF-Token`. CSP (content security policy) permits the page's
inline script/style only with a fresh nonce (a per-page marker).
`--allow-origin` adds one exact origin, not a bypass for those checks.

A changed server instance disables every button, clears half-armed confirms
and reloads once for the new CSRF value. Another mismatch within 5 s leaves
a red restart banner instead of looping. Reloading sends no command.
Every HTTP/non-JSON/network command failure shows a persistent red line
naming the command and GP21 guidance. Live updates do not clear it; a later
accepted command does. Non409 errors advise reload; 409 means a robot-state
refusal, so reload is not advised. A page from before this fix needs one
manual reload.

### Results and troubleshooting

| Observation | Result / action |
| --- | --- |
| Operator browser check, 2026-11-07 | Laptop and remote HTTP8080: stopped server produced persistent named502 DUMP error; restart reloaded both tabs automatically; next DUMP200 completed |
| Dashboard redesign | Operator accepted on `e37c873`; run path kept as known inaccurate, not a requirement |
| Robot behind wall | **17% ping loss** and real `robot silent` banners. Moving the robot to line of sight removed them; not a page-rendering fault |
| Tailscale HTTP8080 | Live/buttons passed; exact FQDN required |
| Hotspot HTTPS8443 | Certificate/SetDNS timed out; retry on normal Wi-Fi, not an established HTTPS pass |
| MQTT reconnect/commands/rates | Revision-scoped results in [S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md); do not infer every queue/traffic test passed |

| Symptom | Check |
| --- | --- |
| `silent` | Status online but no state for>3 s; inspect recorder, ping loss, wall/placement, AP and power before changing UI thresholds |
| OFFLINE | Broker will says robot dropped; nominal broker threshold about 7.5 s after last packet, plus timer/delivery delay; use GP21 |
| Unknown robot | No status received yet; retained summaries are not live proof |
| Bad Host/Origin403 | Exact proxy URL and `--allow-origin`; never disable checks |
| Bad CSRF403 / stale page | Reload for the current server instance; old tokens must remain refused |
| Named502/network failure | Dashboard/proxy unavailable; error remains visible. GP21, then restore service |
| PUBACK but no movement | Check normal versus TEST/BENCH image, command decision, calibration, token and idle gate |
| Missing live rows | QoS0 loss/intentional jump; request DUMP to refill committed rows; reset/new generation can prevent completeness |
| DUMP completeness | Check BEGIN totals, END counts, boot/generation/index gaps; duplicates are normal QoS1 retries |

### Tests and full evidence

```sh
cmake -S tests -B /tmp/robot-host-tests
cmake --build /tmp/robot-host-tests -j4
ctest --test-dir /tmp/robot-host-tests --output-on-failure
python3 scripts/dashboard_replay.py /tmp/robot-host-tests/test_mqtt_wire
```

Replay writes public-fixture `scan-left`, `reattempt`, `idle`, `phone` and
`health` HTML/PNG under `/tmp/dash-shots`. It uses headless Firefox and a
temporary profile; `--html-only` needs no browser. Explicit scene edits are
not hardware captures. Node tests run the actual page script with a fake
page, no broker/credentials; missing Node is reported as a skip.

Source: [server](../../scripts/mqtt_dashboard.py),
[model](../../scripts/dashboard_model.py), [page](../../scripts/dashboard_page.py),
[decoder](../../scripts/robot_wire.py). Test evidence:
[S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md),
[integration TEST_LOG](../../testing/integration/TEST_LOG.md).
Full old panel tables, commands, configuration and measured observations:
[telemetry history](../archive/s1-telemetry/HISTORY.md).
