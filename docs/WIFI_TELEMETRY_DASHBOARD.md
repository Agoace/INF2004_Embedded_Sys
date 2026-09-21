# Wi-Fi telemetry dashboard architecture

Status: Implemented (read-only dashboard)

Date: 2026-09-20

## Purpose

Provide live Pico W instrument readings and run status without a USB or UART data connection. Calibration results remain on the Pico and are read over USB. The design keeps motor power away from the Raspberry Pi 400 USB ports and makes the dashboard available privately through the tailnet.

## Implementation status

The Pico transport is confirmed working on hardware. The repository now has:

- A reusable raw-lwIP Wi-Fi and UDP transport module in `src/telemetry.c` and
  `src/telemetry.h`.
- The `wifi_telemetry_probe` firmware target.
- Telemetry integration in `motor_rotation_drift`.
- The standard-library host receiver in
  `scripts/telemetry_receiver.py`.
- The standard-library read-only dashboard in
  `scripts/telemetry_dashboard.py`.

The dashboard binds to `127.0.0.1:8080` only. It has no motor controls. GP20
and GP21 remain the only physical controls. Tailscale Serve is the remote
access path; the dashboard does not bind to a LAN address.

The hardware soak test showed sporadic, scattered sequence gaps. It did not
show consecutive bursts. This is normal packet loss behaviour for UDP on a
2.4 GHz network. The motor firmware sends live instrument readings and a small
terminal notice when a run ends. The authoritative calibration result stays in
RAM on the Pico. No measurement depends on a telemetry packet arriving.

The local credential file `src/wifi_config.h` is ignored. The tracked
`src/wifi_config.h.example` contains placeholders and copy instructions. A
compiled `.uf2` contains the Wi-Fi credentials in plain text, so firmware
binaries must not be shared.

The listener still accepts unauthenticated UDP from the LAN. A bench-grade
source restriction is now applied with
`scripts/restrict-telemetry-port.sh`. It creates its own `inet telemetry_guard`
nftables table with an input chain whose policy is accept, so it governs UDP
port 4242 only and cannot affect other ports or services. The rule order is:

1. Accept UDP 4242 from the Pico hardware address.
2. Accept UDP 4242 from loopback.
3. Count and drop other UDP 4242 traffic.

The script matches the hardware address, not the IP address, so a DHCP change
does not change the rule. MAC matching works only on the local network segment
and a MAC can be spoofed by a determined attacker on that segment. This is a
bench control, not strong authentication. This mitigates exposure but does not
close the security item. The receiver still performs no packet authentication
and must treat all content as untrusted.

The script has three subcommands:

- `apply <pico-mac>` deletes and recreates the table after validating the MAC.
- `status` prints the table or reports that port 4242 is unrestricted.
- `remove` deletes the table and makes port 4242 unrestricted again.

**Open item after reboot:** the rules are not persistent. The Pi has rebooted,
so the UDP restriction is currently absent and must be reapplied with
`scripts/restrict-telemetry-port.sh` after checking the Pico MAC address.
Persistence remains open and requires adding the table to
`/etc/nftables.conf` after review. Until then the UDP listener is
unauthenticated and relies on this bench-grade rule when it is active.

## Architecture

```text
Pico W on Robo Pico
5 V VIN power
        |
        | Standard 2.4 GHz Wi-Fi
        | UDP telemetry
        v
Raspberry Pi 400 LAN address, UDP port 4242
        |
        | Validate and store readings, trends, and history
        v
Dashboard on 127.0.0.1:8080
        |
        | Tailscale Serve private HTTPS proxy
        v
Authorised devices on the tailnet
```

The Pico W does not join the tailnet. It communicates with the Raspberry Pi 400 through the standard local Wi-Fi network. The Raspberry Pi processes the data, hosts the dashboard on localhost, and uses Tailscale Serve to make the dashboard available to the tailnet.

## Responsibilities

### Pico W

- Connect to the existing 2.4 GHz Wi-Fi network.
- Send physical instrument readings to the Raspberry Pi 400 LAN address.
- Send each reading with a session identifier and sequence number.
- Convert encoder counts to milli-RPM on the Pico using its 2535 CPR constant.
- Continue the motor test if Wi-Fi or the receiver becomes unavailable.
- Keep every completed calibration result in RAM and USB output.
- Use GP20 for test start and GP21 for emergency stop.
- Never depend on a network command for emergency stopping.

### Raspberry Pi 400

- Listen for Pico telemetry on UDP port 4242 through the LAN interface.
- Reject malformed and out-of-order packets as necessary.
- Store readings for trends, averages, and history without calibration constants.
- Provide live gauges, numeric history summaries, and status notices.
- Bind the web dashboard only to `127.0.0.1:8080`.
- Keep dashboard operation separate from motor control.
- Keep the UDP receiver thread independent from HTTP request threads.

### Tailscale

- Proxy the localhost dashboard to a private tailnet HTTPS address.
- Apply tailnet identity and access rules.
- Take no part in Pico-to-Pi telemetry.
- Use Tailscale Serve, not Tailscale Funnel. Funnel would make the service public.

The intended proxy command is:

```sh
tailscale serve --bg 8080
```

Useful management commands are:

```sh
tailscale serve status
tailscale serve off
```

## Network endpoints

| Component | Endpoint | Access |
| --- | --- | --- |
| Pico telemetry destination | Pi 400 LAN address, UDP port 4242 | Local Wi-Fi only |
| Dashboard application | `127.0.0.1:8080` | Pi 400 only |
| Dashboard proxy | Tailnet HTTPS name | Authorised tailnet devices |

The Pico cannot send telemetry to `localhost`. It must use the Pi 400 LAN address. The UDP listener must therefore accept traffic from the LAN interface. A firewall rule should restrict UDP port 4242 to the local network or to the Pico's reserved DHCP address.

## Telemetry content

The authoritative payload contract is the [telemetry JSON schema](TELEMETRY_SCHEMA.md).
It defines the common envelope, one flat instrument readings object, terminal
notices, integer units, versioning, and reserved fields. Packets are newline
terminated JSON objects.

UDP does not guarantee packet delivery. Sequence numbers reveal missing packets, and repeated instrument readings and terminal notices provide recovery without blocking motor control.

## Dashboard implementation

`scripts/telemetry_dashboard.py` runs the UDP receiver on its own thread and
serves one self-contained page with the Python standard library. The page
polls `/api/state` about once per second. It shows live readings, packet
health, terminal notices, and up to 300 numeric samples per field. It clears
history when the Pico session changes. Missing fields show `not applicable`;
they are never changed to zero. The browser receives JSON and uses DOM
`textContent`, not `innerHTML`, for received values.

Run `python3 scripts/telemetry_dashboard.py --self-test` to test the state
pipeline without opening sockets.

The dashboard is read-only. It provides no remote motor start and no remote
emergency stop. GP20 and GP21 remain the only physical controls.

## Initial dashboard scope

The dashboard shows:

- Pico connection state and time of the last packet.
- Wheel speed in RPM, motor duty percent, direction, and current state.
- Ultrasonic range and IR voltage when a target provides them.
- Terminal outcome and fault status beside the live gauges.
- Trends, averages, and history without converting encoder counts.

Remote motor start and remote emergency stop are not implemented. GP20 and
GP21 remain the only physical controls.

## Power and failure rules

- Power the motors and Pico W from the regulated 5 V VIN supply during a motor test.
- Keep the Pico USB cable disconnected while the motors operate.
- Wi-Fi failure must not stop, delay, or change motor safety logic.
- Network transmission must not block encoder processing or emergency-stop checks.
- If the dashboard is unavailable, the Pico must finish safely and retain the result in RAM.
- If the Pico stops transmitting unexpectedly, the dashboard must show a stale-connection warning.

## Configuration and secrets

Do not commit Wi-Fi credentials. Keep them in an ignored local configuration file or provide them during the local build. The firmware binary will still contain the credentials needed to join the network, so distribute firmware files carefully.
