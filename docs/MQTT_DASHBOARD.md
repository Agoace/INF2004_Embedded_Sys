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

The recorder and the dashboard are two separate subscribers. You can run one
without the other. The dashboard does not write files.

## Page layout

The top bar shows four status labels and the buttons.

| Label | Meaning |
| --- | --- |
| broker | Green when the dashboard is connected to the broker. |
| robot | Green when any robot message arrived in the last 3 s. |
| row gaps | Live log rows that are missing: **lost** (QoS 0 frames that never arrived) plus **skipped** (rows the robot skipped on purpose to catch up). |
| server | Green while the page receives updates from the dashboard server. |

Each panel shows its **data age** at the top right. A panel turns grey and
shows `stale` when its data is older than 1 s. The age includes the age that
the robot reports for that board.

| Panel | Source | What it shows |
| --- | --- | --- |
| Vehicle | `robot/state`, `robot/event` | State, module, running flag, motion faults, vehicle board status, last command and the robot decision. |
| Line sensor | `robot/state` | Position, raw and normalised left and centre values, and a short position trend. |
| Wheels | `robot/state` | Requested and measured wheel speed, encoder counts and servo angle. |
| Ultrasonic | `robot/sonar` | Latest distance (or the sensor error) and a bar chart of the latest result for each angle. |
| Events | `robot/event` | The last 50 events. Event gaps count missing event sequence numbers. This panel never turns grey. |
| Health | `robot/heartbeat` | Wi-Fi poll gaps, loss and busy counters, minimum free stack, reset reason. It turns grey after 2.5 s. |
| Barcode | `robot/state` | Raw GP28 sample. The decoded command is not implemented yet. |
| IMU | none | Placeholder. |
| RAM log stream | `robot/log` | Live rows, merged DUMP rows, lost and skipped rows, data age, rows behind. |

A new panel needs only one builder function in `dashboard_model.py`. The page
draws any panel that the server sends.

## Buttons

| Button | Enabled when | Confirm step |
| --- | --- | --- |
| ESTOP | Always. | No. One click sends it. |
| DUMP | Broker connected, token file set, no command waiting for a decision, and no dump already running. DUMP may run while the robot drives. | No. |
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
  `robot/heartbeat` and `robot/log`. It subscribes to `robot/log` with QoS 1.
  A subscription limits the delivery QoS, and a DUMP refill batch is QoS 1.
- The dashboard subscribes again after every broker reconnect.
- A message with a wrong length, version or reserved bit is discarded. It
  does not change the panels. The count shows in the page data.
- Live log rows and DUMP refill rows merge by generation and index. A
  repeated row (QoS 1 retry) is dropped. See the log rules in
  [MQTT_WIRE.md](MQTT_WIRE.md).
- The row gap counter covers live rows only. A DUMP fills old holes. The
  recorder prints the final gap report when it stops.
- The log store keeps one entry for each row until the dashboard stops. Restart
  the dashboard after a long session.

## Checks

```sh
cmake -S tests -B /tmp/robot-host-tests
make -C /tmp/robot-host-tests -j4 --output-sync=target
ctest --test-dir /tmp/robot-host-tests -R test_mqtt_dashboard --output-on-failure
```

The tests use messages that the C code packs. They check these items:

- Messages become panel data.
- Stale data turns grey at the 1 s limit.
- The row gap counter.
- The button rules (idle, fresh, broker, token, waiting, dump active).
- The command bytes, and the confirm and gate checks.
- The token never appears in a page, response, stream or log line.
- Requests with a bad Origin, Host, CSRF value or content type are refused.

No test uses a broker, a real token or a browser.
