# MQTT operation and dashboard

## What it is

Mosquitto is the broker on the operator laptop. The recorder saves decoded
robot observations to CSV. The dashboard shows live data and sends ESTOP,
DUMP, CALIBRATE and START. They are separate subscribers; the dashboard
itself does not save a CSV. Use [TELEMETRY.md](TELEMETRY.md) for ownership
and [MQTT_WIRE.md](MQTT_WIRE.md) for bytes and token rules.

Only the operator installs services, changes firewall rules, flashes or
runs powered tests. MQTT ESTOP is best effort. Use GP21 locally in the
normal robot; TEST motor steps do not consume remote commands or GP21.
Lift wheels for TEST. A lost network is not a robot stop mechanism.

## Broker

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

## Private settings

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

## Start the recorder and page

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

### Commands

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

## Page and data

| Row | View |
| --- | --- |
| Header | Robot ONLINE, RUN/IDLE/FAULT, active LINE > SCAN > DETOUR > SEARCH; CAL/START/DUMP and large red ESTOP |
| 1 | ULTRASONIC (largest), LINE L/C bars and position, DRIVE requested/measured L/R speed |
| 2 | RUN PATH and LAST RUN (time, distance, detours, line losses, stop reason text) |
| 3 | Collapsible HEALTH: CPU, Wi-Fi, timing, counters, full original detail panels |

Below800 px the same cards stack. State panels turn stale after 1 s (2.5 s idle);
health after 2.5 s (12 s idle). These display limits do not replace producer
fresh flags or motor safety checks. The newest state/heartbeat mode selects
RUN/IDLE. Idle state arrives at 1 Hz, heartbeat every 5 s; sonar/live log pause.
Their last data is labelled `idle (paused)`; the fan is held grey and its
age continues to increase. Silent remains a3 s state gap in both modes.

### Ultrasonic fan

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

### Run path and retained history

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

## Remote view through Tailscale

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

### Security and restart

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

## Results and troubleshooting

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

## Tests and full evidence

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
[dashboard archive](../archive/s1-telemetry/MQTT_DASHBOARD.md),
[runbook archive](../archive/s1-telemetry/MQTT_RUNBOOK.md).
