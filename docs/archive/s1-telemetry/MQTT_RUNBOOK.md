# Robot MQTT runbook

## Safety and scope

Only the operator installs the broker, changes firewall rules or authorizes
hardware tests. This implementation did not connect, flash or push. Lift
both wheels before TEST power; S2-U02 moves motors and S2-U05 resets once.
MQTT is optional telemetry. GP21 remains the real ESTOP; remote ESTOP is
best effort. M1/M2 must pass independently of network availability.
See [design and limits](KERNEL_MQTT.md), [wire layout](../../s1-telemetry/MQTT_WIRE.md), and
[template changes](../../architecture/TEMPLATE_CHANGES.md).

## 1. Set up a broker on the robot network

The operator installs Mosquitto (for example, the distribution packages
`mosquitto` and `mosquitto-clients`). Do not expose port 1883 on the public
Internet or unrelated interfaces. Use a trusted robot-only network; this
version has neither TLS nor broker user/password authentication.

Use a dedicated broker configuration. Replace both placeholders before use:

```text
listener 1883 <LAPTOP_ROBOT_IP>
allow_anonymous true
persistence false
```

The listener binds only the laptop interface on the robot network. Use the
packaged service configuration OR a foreground process, not two competing
instances. To run the latter:

```sh
mosquitto -c /path/to/robot-mosquitto.conf
```

If UFW is in use, the operator can add a scoped rule after replacing values:

```sh
sudo ufw allow in on <ROBOT_INTERFACE> from <ROBOT_SUBNET> to <LAPTOP_ROBOT_IP> port 1883 proto tcp
```

Do not add a global `allow 1883`, change firewall defaults or open another
interface. Check the active listener and existing firewall policy. The
broker IP is not inferred by the firmware; set it explicitly below.

## 2. Prepare private firmware settings

Keep the existing `config/wifi_secrets.h` private and mode 0600. For a fresh
worktree, copy it from the main checkout with `cp` and `chmod 600`; never
print, grep values from, or commit it. Do not edit the template config
example. Broker and command settings belong to the application instead:

- Example: `app_program/subsystem1/robot_mqtt_secrets.h.example`.
- Private header: `app_program/subsystem1/robot_mqtt_secrets.h` (gitignored).
- `ROBOT_MQTT_BROKER_IP`: replace `0.0.0.0` with the broker's robot-network IP.
- Port: 1883. Choose a unique client ID if multiple robots use one broker.
- `ROBOT_MQTT_COMMAND_TOKEN_HEX`: 32 randomly generated hex digits, not zeros.

For **first-time** token setup, this creates a private header and laptop
file without printing the token. It refuses existing paths. Run from the
repository root. Then edit only the broker/client settings in the private
header with an editor; do not display it in logs.

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

No valid token means ESTOP-only fallback: a correct 22-byte ESTOP packet is
accepted with its token field unchecked; all other commands are refused
and counted. With a valid non-example token, **all** commands including
ESTOP require it. Never put the token on a command line, in CSV, or in a
committed file. Firmware binaries contain private Wi-Fi/token settings;
keep them private too. Do not publish ELF/UF2 artifacts.

## 3. Build, do not flash without approval

```sh
cd build_make
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 -j4 --output-sync=target
# Lifted-wheel TEST profile, only when authorized:
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 TEST=1 -j4 --output-sync=target
```

Normal image: `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
TEST image has `_test` before `.uf2`. Broker placeholder builds successfully
but never attempts a broker connection. Build readiness never waits for a
link or broker. After an authorized flash, repeat cold-power S0-U01 on the
new image, not just a warm USB reconnect.

## 4. Receive and send

Python uses installed **paho-mqtt 2.x**, `CallbackAPIVersion.VERSION2`.
Keep a receiver running to capture results and losses:

```sh
python3 scripts/mqtt_robot.py --host <LAPTOP_ROBOT_IP> --csv robot-mqtt.csv
```

It subscribes to state, sonar, event, heartbeat and log, validates version,
length/reserved fields, and decodes signed values to a flat CSV. It does
not subscribe to `robot/cmd` or record tokens. It subscribes again after a
broker reconnect. See [wire fields](../../s1-telemetry/MQTT_WIRE.md) for units and flags.

Commands apply only to the NORMAL robot. TEST/BENCH deliberately do not
start vehicle and do not consume these queues; broker ACK never proves a
vehicle acted. In TEST, neither MQTT nor GP21 is a stop control: lift wheels.
Use a second terminal for commands; every command has a token field:

```sh
python3 scripts/mqtt_robot.py --host <LAPTOP_ROBOT_IP> --command calibrate --token-file "$HOME/.config/robot-mqtt.token"
python3 scripts/mqtt_robot.py --host <LAPTOP_ROBOT_IP> --command start --token-file "$HOME/.config/robot-mqtt.token"
python3 scripts/mqtt_robot.py --host <LAPTOP_ROBOT_IP> --command dump --token-file "$HOME/.config/robot-mqtt.token"
python3 scripts/mqtt_robot.py --host <LAPTOP_ROBOT_IP> --command estop --token-file "$HOME/.config/robot-mqtt.token"
# Only an unconfigured robot accepts this zero-token ESTOP:
python3 scripts/mqtt_robot.py --host <LAPTOP_ROBOT_IP> --command estop
```

CALIBRATE and START are idle-only; START requires successful calibration.
CALIBRATE returns idle without starting navigation. DUMP is batched and can
run during driving. Commands use QoS1, `retain=False`, and a random uint32
request ID (optional `--request-id`). Broker PUBACK is not robot acceptance;
watch vehicle command-result events/state. Request IDs only suppress the
last eight duplicate non-ESTOP deliveries in RAM, not replay after reboot.
Clear any old retained `robot/cmd` message before tests; never retain a
command. A trusted publisher with the token can control the robot.

## 5. Acceptance and diagnosis

All rows below start **Not run** on the STEP 8 image. Record revision,
private-setting presence (not values), power source, profile and duration.

| Check | Observe |
| --- | --- |
| S0-U01 cold power, normal/TEST/BENCH/MQTT | Correct 1 us source and unchanged 2% TIMER/kernel check; no dependency on Wi-Fi startup order |
| S1-U04 connect/reconnect | Connect/subscription; retained ONLINE after CONNACK and OFFLINE will after AP loss (5 s keepalive; about 7.5 s broker threshold plus timer/delivery delay); broker restart and AP outage; robot/RAM log unaffected; reconnect without accumulating timeout chains |
| S1-U05 rate/loss/decode | Signed distance/duty, encoder heading/rate, obstacle scan-ID gaps (width unavailable), detour results, retained run summary, CPU known/unknown; Live log stream about 100 rows/s in 10-row batches, data age, gaps/jumps after reconnect, DUMP refill merge and Wi-Fi hook wall time with streaming on; nominal 5 Hz state, all retained service sonar results, rare vehicle events, 1 Hz heartbeat; CSV signs/flags; sequence gaps/history misses/event losses under traffic |
| S1-U06 commands | Unconfigured ESTOP-only; configured wrong token refused for every opcode; idle rules; calibration then START; DUMP while driving; queue-full/duplicate policy; GP21 still works |
| S1-U07 CPU meter | MQTT TEST idle board, wheels lifted: save printed reference rate and three load samples, establish expected range, verify independent target timing and stack margin; [method](../kernel/CPU_LOAD.md) |
| S2-U01 Wi-Fi20 | Unchanged 499–501 ticks / <=11000 us gap / zero missed; window and lifetime Wi-Fi wall counters, >10000 us gaps and probe cost |
| M1/M2 with MQTT | Same local motor/navigation acceptance; separate network acceptance, lost broker never stops motion |

TEST prints Wi-Fi timing after its S2 measurement, not during it. A complete
vehicle session prints its independent Wi-Fi window after motion stops.
Lifetime poll-gap maximum and sample count also appear in heartbeat.
`wifi_observed_priority` comes from the running task; 0 means not observed,
not priority 0. `wifi_start_status` reports task creation/start failure;
template USB startup messages report subsequent radio initialization failure. Poll counts of 0
or stale health do not establish performance. Work/probe intervals include
preemption and are not CPU percentages. Full stack margins under sustained
traffic/dump remain a hardware gate; app minimum excludes template stacks.

The live stream publishes rows on `robot/log` without any command. Check
that the receiver CSV grows about 100 rows/s while the vehicle logs, that each
row's data age is small (the robot reports it), and that `skipped_before`
and index gaps appear only after a reconnect or a stall. Live frames are QoS 0:
a laptop that is slow or disconnected misses rows, and a dashboard prefers
that to showing old data. A DUMP refills the CSV by (generation, index).
A full 4501-row/512-event DUMP takes about 14 s at 100 ms round-trip time
(host simulation; not measured on hardware). Live frames continue during it.

For DUMP, verify BEGIN totals, each generation/index, and END counts. QoS1
may duplicate frames; deduplicate by dump ID/generation/kind/index. A new
run yields ABORT; request again for the new generation. Reset or a prolonged
outage can leave a partial CSV. Rows already queued to local RAM but not yet
committed at snapshot time are excluded. Do not interpret `publish_ok` as
end-to-end delivery. Busy/retained samples have fresh flags cleared and
ages saturated; do not graph them as fresh observations.
