# Robot MQTT dashboard

## Purpose and scope

The dashboard is the live view of the robot on the laptop. It reads the
robot MQTT topics from the broker and shows them in a web page. It also sends
the four robot commands (ESTOP, DUMP, START, CALIBRATE).

- The broker (Mosquitto) and the dashboard run on the **laptop**.
- The dashboard binds to `127.0.0.1` only. Only that laptop can open it.
- The page works offline. It has no CDN and loads no file from the network.
- The wire contract is [MQTT_WIRE.md](MQTT_WIRE.md). The broker set-up is in
  [MQTT_RUNBOOK.md](MQTT_RUNBOOK.md).
- The older UDP tool `scripts/telemetry_dashboard.py` is a bench tool. See
  [the UDP dashboard note](WIFI_TELEMETRY_DASHBOARD.md). Use this dashboard for
  the MQTT robot.
- GP21 is the real emergency stop. The remote ESTOP button is best effort.
  The network never stops the vehicle by itself.

## Files

| File | Job |
| --- | --- |
| [`scripts/mqtt_dashboard.py`](../scripts/mqtt_dashboard.py) | HTTP server, MQTT link, command rules, command line |
| [`scripts/dashboard_model.py`](../scripts/dashboard_model.py) | Pure model: messages in, panel data and button rules out |
| [`scripts/dashboard_page.py`](../scripts/dashboard_page.py) | The page (HTML, CSS and JavaScript in one string) |
| [`scripts/robot_wire.py`](../scripts/robot_wire.py) | Shared decoder, log merge and gap store, command packer |
| [`tests/test_mqtt_dashboard.py`](../tests/test_mqtt_dashboard.py) | Host tests (no broker, no Paho) |
| [`tests/dashboard_page_harness.js`](../tests/dashboard_page_harness.js) | Runs the served page script in Node with a fake DOM for the command-error and restart tests |

## Run

Use Python 3 and `paho-mqtt` 2.x. Nothing else is needed.

1. Start Mosquitto on the laptop. Use the scoped configuration in the
   [runbook](MQTT_RUNBOOK.md). The robot must reach the broker address.
2. Start the recorder in one terminal. It writes every message to a CSV file:

   ```sh
   python3 scripts/mqtt_robot.py --host 127.0.0.1 --csv robot-mqtt.csv
   ```

3. Start the dashboard in a second terminal:

   ```sh
   python3 scripts/mqtt_dashboard.py --host 127.0.0.1 --token-file "$HOME/.config/robot-mqtt.token"
   ```

4. Open `http://127.0.0.1:8765/` in a browser on the same laptop.

| Option | Meaning |
| --- | --- |
| `--host`, `--port` | MQTT broker. Default `127.0.0.1` and `1883`. |
| `--http-port` | Dashboard port on `127.0.0.1`. Default `8765`. |
| `--token-file` | Private token file (mode `0600`, 32 hex digits). Without it, only ESTOP works, with a zero token. |
| `--allow-origin URL` | Also accept one exact proxy origin, for example a Tailscale Serve URL. Repeatable. See [Remote view through Tailscale](#remote-view-through-tailscale). |

The recorder and the dashboard are two separate subscribers. You can run one
without the other. The dashboard does not write files.

## Remote view through Tailscale

Verified on 2026-11-07 by the operator:

```sh
tailscale serve --bg --http=8080 http://127.0.0.1:8765
```

From `fedora-desk`, the browser opened
`http://fedora-laptop.taild62d21.ts.net:8080/`. Live data and all buttons
worked. The browser marks the page **Not secure** because the page uses
plain HTTP. Tailscale still carries the connection over WireGuard. The short
name `http://fedora-laptop:8080` returns HTTP 403 by design, because the
dashboard checks the exact Host and Origin.

HTTPS on port 8443 did not work on the hotspot. The Let's Encrypt
certificate fetch or `SetDNS` step timed out. Retry HTTPS on normal Wi-Fi.

The dashboard always binds `127.0.0.1`. To use it from another device on
your tailnet, let Tailscale Serve forward one HTTPS port to it. Do not
change the bind address.

1. Start the forward. This keeps other Serve entries unchanged:

   ```sh
   tailscale serve --bg --https=8443 http://127.0.0.1:8765
   ```

2. Start the dashboard with the same public URL as an allowed origin:

   ```sh
   python3 scripts/mqtt_dashboard.py --host <LAPTOP_ROBOT_IP> \
       --token-file "$HOME/.config/robot-mqtt.token" \
       --allow-origin https://<laptop-name>.<tailnet>.ts.net:8443
   ```

3. Open `https://<laptop-name>.<tailnet>.ts.net:8443/` on the other device.

Without `--allow-origin`, the remote page shows data but the server refuses
every button with `bad Host header` or `bad or missing Origin`. The option
accepts only the exact scheme, host and port. The CSRF value and the
content-type check still apply. Every device on the tailnet can reach the
page and use **all** buttons, including START. Use Tailscale access rules
if a device must not reach it. Stop the forward with
`tailscale serve --https=8443 off`.

## Page layout

The top bar shows four status labels and the buttons.

| Label | Meaning |
| --- | --- |
| broker | Green when the dashboard is connected to the broker. |
| robot | The robot link: **ONLINE** (green), **OFFLINE** (red), **silent** or **unknown** (amber). See [Robot link](#robot-link). |
| row gaps | Live log rows that are missing: **lost** (QoS 0 frames that never arrived) plus **skipped** (rows the robot skipped on purpose to catch up). |
| server | Green while the page receives updates from the dashboard server. |

Each panel shows its **data age** at the top right. A panel turns grey and
shows `stale` when its data is older than 1 s. The age includes the age that
the robot reports for that board.

| Panel | Source | What it shows |
| --- | --- | --- |
| Vehicle | `robot/state`, `robot/event` | State, module, running flag, motion faults, vehicle board status, last command and the robot decision. |
| Line sensor | `robot/state` | Position, raw and normalised left and centre values, and a short position trend. |
| Wheels | `robot/state` | Requested and measured wheel speed, PWM duty of each wheel, distance travelled (left, right, average, in mm), encoder counts and servo angle. |
| Odometry (encoder-derived) | `robot/state`, `robot/imu` | Heading dial, turn rate in degrees per second, and distance. Every value comes from the wheel counts. Two IMU rows (raw) show beside them for comparison when the IMU sends data. |
| Ultrasonic | `robot/sonar` | Latest distance (or the sensor error) and a bar chart of the latest result for each angle. |
| Obstacle profile and detours | `robot/obstacle`, `robot/sonar`, `robot/detour` | The result of each completed scan: closest distance and angle, right and left clearance, the decision, scan ID, scan stage and missed scans. The chart shows the latest sonar result for each angle. Green bands mark the clear sectors. A ring marks the closest point. Below the chart, one table row for each of the last 8 detours: side, outcome, duration, distance, maximum sideways offset and the scan that started it. |
| Events | `robot/event` | The last 50 events. Event gaps count missing event sequence numbers. This panel never turns grey. |
| Health | `robot/heartbeat` | Wi-Fi poll gaps, loss and busy counters, CPU load, minimum free stack, reset reason. It turns grey after 2.5 s. |
| Barcode | `robot/state`, `robot/barcode` | Raw GP28 sample. Rows for detected, decoded, unreadable, decoded command, confidence (agreeing reads of all reads), direction, navigation command, result, and issued or executed. They show `not implemented yet` until data arrives on the reserved topic. |
| IMU | `robot/imu` | Pitch, heading, turn rate, hump state, height, peak, collision, validity and the latest motion event. It is a placeholder until data arrives on the reserved topic. Values are raw: the units are not fixed yet. |
| Run summary | `robot/summary` | The totals of the last finished run: stop source and reason, duration, distance, line losses, obstacles, detours, speed error, faults. Barcode and hump fields show `not implemented yet` until the robot sets the implemented flag. This panel never turns grey. |
| RAM log stream | `robot/log` | Live rows, merged DUMP rows, lost and skipped rows, data age, rows behind. |

**Wheel distance** is signed millimetres since the robot booted. Forward is
positive. The average is `(left + right) / 2`, cut toward zero. It comes from
`robot/state` version 2. An old version 1 message has no distance, and the panel
says `not reported`.

**Obstacle angles:** a negative angle is to the right. A positive angle is to the
left. The decision is the side that the robot detours around. The decision
`none` means that both sides are blocked. The robot cannot reverse, so the
detour then fails. The robot does not send an obstacle width. The panel shows
only angular clearance. A gap in the scan ID means that a scan result was not
received. The panel counts these gaps. The age is the time since the scan
finished. This panel never turns grey, because a scan is one point in time.

**PWM duty** is in permille on the wire. The page shows percent. Forward is
positive. It comes from `robot/state` version 3.

**Odometry:** the heading is in 0.01 degree units, left (anticlockwise) is
positive, and zero is the robot at boot. The dial shows it in the range 0 to
360 degrees, with zero straight up. The robot computes heading and turn rate
from the wheel counts, so a slipping wheel gives a wrong value. All three
values (heading, turn rate, distance) come from the motion board and can be
a stale last copy. The panel age is the age of the motion board data.

**Detours:** the robot sends one `robot/detour` message for each finished
detour, with a sequence number. A gap in the sequence means that the panel
missed a detour, and the panel counts these gaps. The side is the obstacle side
(right or left). The outcome texts are `done, line found`, `done, no line`
(with the side to search, left or right), `FAILED` (with the reason) and
`aborted` (the run ended in the detour). The maximum offset shows only when
the robot sets the valid flag.

**Run summary:** the robot publishes `robot/summary` as a retained message at
the end of each run. Summary v2 flags bits 3–7 show the number of SCAN
reverse-and-rescan attempts (0–31; the robot saturates at 31). The
obstacle panel shows each profile attempt. The event list names kind 8 as
a SCAN retry and shows its phase. After a broker reconnect, the panel can show the retained
copy of an earlier run, and the note says so. While a new run is in progress,
the note says that the summary is for the previous run.

**CPU load:** `robot/heartbeat` version 2 reports it in permille. The panel
shows percent. `not available` means that the robot cannot measure it.

The event `junction` is reserved. The decoder accepts it and the event list
names it. The robot does not send it yet.

The barcode and IMU topics are reserved. The robot firmware does not publish
them yet. When a message with version 1 and a known kind arrives, the panel shows its
fields. A message with another version or an unknown kind is ignored and
counted. A known message with the wrong length or a reserved bit set is
discarded as damaged.

A new panel needs only one builder function in `dashboard_model.py`. The page
draws any panel that the server sends.

## Robot link

The robot publishes `robot/status` (version 2: 6 bytes, QoS 1, retained).
The last four bytes contain the boot ID. Old version 1 messages have 2 bytes. The robot also
sets it as its MQTT last will. When the robot drops, the broker publishes
`offline`. The broker needs about 7.5 s for this (1.5 times the 5 s keep-alive,
plus up to 1 s).

| Level | Rule |
| --- | --- |
| ONLINE | `robot/status` says online **and** `robot/state` arrived in the last 3 s. |
| OFFLINE | `robot/status` says offline. A retained offline message shows at once. A red banner shows. |
| silent | `robot/status` says online but `robot/state` stopped for more than 3 s. The link may be dying. A banner shows. |
| unknown | No `robot/status` message yet. This is normal for older firmware. The older button rules apply, with a check that robot data is recent. |

The dashboard forgets the status when its own broker link drops. The broker
sends the retained status again after the reconnect.

## Buttons

| Button | Enabled when | Confirm step |
| --- | --- | --- |
| ESTOP | Always. When the robot is OFFLINE or silent, a hint warns that delivery is unlikely. Use GP21. | No. One click sends it. |
| DUMP | Broker connected, token file set, robot not OFFLINE and not silent, no command waiting for a decision, and no dump already running. DUMP may run while the robot drives. | No. |
| START | Same as DUMP, and the state message is newer than 1 s, the vehicle board is fresh and not busy, and the robot is idle (state IDLE and not running). | Yes. |
| CALIBRATE | Same as START. | Yes. |

- A confirm step is two clicks. The first click arms the button for 4 s and
  shows `CONFIRM`. The second click sends the command.
- A disabled button shows the reason when you hold the pointer over it.
- The server checks every rule again when it receives a command. A page that
  is out of date cannot send a command that the rules refuse.
- START needs a calibrated robot. The dashboard cannot see this. The robot
  refuses START when it is not calibrated, and the vehicle panel shows
  `refused: not calibrated`.
- ESTOP stays enabled when the broker is disconnected. Then the dashboard
  cannot send it, and it shows an error. Use GP21.

### Command not sent

A command that does not reach the broker is never silent. Any HTTP error
reply, an unreadable reply or a network error shows a red line under the
header. The line names the command and the server error, for example:

`ESTOP NOT SENT: bad CSRF token (HTTP 403). Reload the page. Use GP21 to stop the robot.`

The line stays visible. Live updates do not clear it. Only a later command
that the server accepts clears it. A 409 reply is a robot-state refusal
(confirm needed, or the robot is not idle), so its line does not say to reload.

### Dashboard restart

Each server process has a random instance ID. It is **not** secret and is
not the CSRF value. The page receives it at load, and every `/events` and
`/api/state` snapshot carries it as `server_instance`.

After a dashboard restart, an open page still holds the old CSRF value, so
the server refuses every command with 403. Live updates reconnect
automatically, so the page can look normal. When the page sees a different
instance ID, it:

1. Disables every button at once and stops rendering the old view.
2. Shows the red line `DASHBOARD RESTARTED: reloading the page. Commands are
   disabled until it reloads. Use GP21 to stop the robot.`
3. Reloads itself once.

The reload is safe. It sends no command, and it only fetches the page again
to get the new CSRF value. A reload clears any half-armed START or
CALIBRATE confirm, so no confirm can carry over. If the reloaded page sees a
mismatch again within 5 s (for example, a proxy that serves an old page),
it does not reload again. It keeps the buttons disabled and shows
`DASHBOARD RESTARTED: reload the page`. This prevents a reload loop.

### Command result

A command goes to the broker with QoS 1. **A PUBACK from the broker is not
acceptance.** It only means that the broker has the command.

The robot answers with a command-decision event (kind 6). The vehicle panel
shows that decision (`accepted`, `refused: robot not idle`, and so on). The
dashboard matches a decision to the last command by its opcode.

| Vehicle panel text | Meaning |
| --- | --- |
| `sent; waiting for the robot decision` | The broker has the command. No decision yet. |
| `robot decision: accepted` | The robot accepted it. |
| `robot decision: refused: ...` | The robot refused it. The reason follows. |
| `no robot decision received ...` | No decision in 5 s. The robot refused the token, or a message was lost. |

A bad token produces no event on the robot. That case shows the last text.
While a command waits (up to 5 s), DUMP, START and CALIBRATE stay disabled.
ESTOP is never blocked.

## Security

- The token file is read **on the server** at start-up. The token never goes
  to the browser. It is not in the page, in any response, or in any log line.
- The server logs only the method, the path and the status code.
- A command is a `POST` to `/api/command`. The server accepts it only when all
  of these are true:
  - The `Host` header is `127.0.0.1:PORT` or `localhost:PORT`. This blocks
    DNS rebinding.
  - The `Origin` header is the dashboard's own address (`http://127.0.0.1:PORT`
    or `http://localhost:PORT`). A header that is missing or different is refused.
  - The content type is `application/json`. A web form cannot send this
    without a browser pre-check.
  - The `X-CSRF-Token` header matches the random value of this server run. The
    value is new at every start. Only the dashboard page has it.
- A web page on another site cannot read the dashboard page. It cannot send
  START. The page has a content security policy with a fresh nonce on each load.
- Every `GET` also needs a correct `Host` header.
- Request IDs come from the server (random 32-bit values). The browser cannot
  choose one.
- Commands follow [MQTT_WIRE.md](MQTT_WIRE.md): topic `robot/cmd`, QoS 1, not
  retained, 22 bytes with a 16-byte token.
- Anyone who can run programs on the laptop can read the token file or open
  the dashboard. Keep the laptop trusted.

## Data rules

- The dashboard subscribes to `robot/state`, `robot/sonar`, `robot/event`,
  `robot/heartbeat`, `robot/log`, `robot/obstacle`, `robot/status`,
  `robot/detour`, `robot/summary`, `robot/barcode` and `robot/imu`. It
  subscribes to `robot/log`, `robot/status`, `robot/summary` and `robot/barcode`
  with QoS 1. A subscription limits the delivery QoS, and a DUMP refill batch,
  the status message and the summary are QoS 1.
- The decoder accepts `robot/state` versions 1 to 4 (48, 60, 72 and 76 bytes)
  and `robot/heartbeat` versions 1 to 3 (100, 100 and 108 bytes), so old captures
  still decode. The firmware sends only the newest version.
- The recorder (`scripts/mqtt_robot.py`) also subscribes to all these topics.
  Its CSV includes state, CPU load, detours, run summaries, barcode results,
  IMU snapshots and IMU motion events. It ignores unknown reserved versions
  and kinds without writing a row.
- The dashboard subscribes again after every broker reconnect.
- A message with a wrong length, version or reserved bit is discarded. It
  does not change the panels. The count shows in the page data.
- Live log rows and DUMP refill rows merge by boot/session, generation, kind
  and index. A repeated row (QoS 1 retry) is dropped only within the same
  boot/session. See the log rules in
  [MQTT_WIRE.md](MQTT_WIRE.md).
- The row gap counter covers live rows only. A DUMP fills old holes. The
  recorder prints the final gap report when it stops.
- The log store keeps one entry for each row until the dashboard stops. Restart
  the dashboard after a long session.

## Reboots and old captures

Implemented observation messages carry a 32-bit `boot_id`. Commands stay at
version 1 and 22 bytes. Reserved barcode and IMU messages do not change.
The CSV also has `boot_session`: a local label that contains the boot ID and
a session suffix. For old frames without an ID, it says `legacy:N`.

The log store keeps separate duplicate records, gaps and DUMP request IDs
for each boot/session. Two boots can both send generation 2, row 0: both
rows must reach the CSV. A DUMP from one boot cannot fill a gap from another.
Embedded 16-byte event records stay at version 1; they inherit the ID from
the outer log message.

A change of boot invalidates current sensor readings, charts, event and
detour sequence counters, and pending command decisions. The next idle
state must come from the current boot before START can become available.
Delayed live frames from an already superseded boot do not replace current
state. Retained status can arrive before the first live frame of that boot;
the dashboard saves it until a live message identifies that boot.

The run summary is retained as history, with its boot/session label. If it
belongs to an earlier boot, the note says so. An older retained summary does
not replace a summary already received from the current boot. A retained
summary is not proof that the robot is online. The detour table shows only
the current boot/session. Log records and gaps from all sessions remain
available in the recorder report until the process stops.

The 32-bit ID can repeat by accident. The receiver also starts a new local
session after an OFFLINE-to-ONLINE status change, or when uptime goes
backwards within a source. It checks state, heartbeat and live-log publish
times separately. It does not compare times between topics, or use old
summary/refill timestamps as live uptime. A normal 32-bit uptime wrap does
not start a session. A broker disconnect alone does not prove a robot reboot.

Legacy session labels are **inferred**, not unique robot identities. If both
status transitions and uptime rollback are missed, a legacy reboot cannot
be distinguished. A reordered legacy frame can also look like a rollback.
After an accidental repeated ID, an old frame with that same ID is likewise
ambiguous. Use the new formats for reliable separation of distinct boot IDs.

Heartbeat version 3 separates unavailable boards from busy reads. The
health panel shows the last unavailable-board mask and the total number of
unavailable observations separately from the per-board busy counters. A
busy read does not change the unavailable mask. An old heartbeat does not
report board availability; the panel does not invent it.

## Checks

```sh
cmake -S tests -B /tmp/robot-host-tests
make -C /tmp/robot-host-tests -j4 --output-sync=target
ctest --test-dir /tmp/robot-host-tests -R test_mqtt_dashboard --output-on-failure
```

The tests use messages that the C code packs. They check these items:

- Messages become panel data.
- Stale data turns grey at the 1 s limit.
- The row gap counter and boot-scoped gap refill.
- Two boots with identical generation-2 rows both reach the CSV.
- Legacy uptime rollback, uptime wrap, OFFLINE-to-ONLINE boundaries,
  old retained summaries, delayed old live frames and detour resets.
- Unavailable-board counters stay separate from busy-read counters.
- The button rules (idle, fresh, broker, token, waiting, dump active, robot offline or silent).
- The wheel distance and duty, the odometry dial, the obstacle panel and its scan ID gaps, the detour table and its sequence gaps, the run summary, CPU load, the reserved topics and the robot link levels.
- The command bytes, and the confirm and gate checks.
- The token never appears in a page, response, stream or log line.
- Requests with a bad Origin, Host, CSRF value or content type are refused.
- The instance ID is on the page, in `/api/state` and `/events`, differs per
  process, and differs from the CSRF value.
- Page script, run in Node: a 403, 502 or network error shows the
  persistent red line naming the command; a 409 omits the reload advice; a
  later success clears it; an instance change disables buttons and reloads
  once; a repeat within 5 s shows the banner without a reload loop.

No test uses a broker, a real token or a browser. The page script tests
need `node`; they are skipped, and reported as skipped, if `node` is missing.
