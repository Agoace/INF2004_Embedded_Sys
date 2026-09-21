# Wi-Fi telemetry dashboard architecture

Status: Partially implemented

Date: 2026-09-20

## Purpose

Provide live Pico W calibration results without a USB or UART data connection. The design keeps motor power away from the Raspberry Pi 400 USB ports and makes the dashboard available privately through the tailnet.

## Implementation status

The Pico transport is confirmed working on hardware. The repository now has:

- A reusable raw-lwIP Wi-Fi and UDP transport module in `src/telemetry.c` and
  `src/telemetry.h`.
- The `wifi_telemetry_probe` firmware target.
- Telemetry integration in `motor_rotation_drift`.
- The standard-library host receiver in
  `scripts/telemetry_receiver.py`.

The read-only dashboard and the Tailscale Serve proxy do not exist yet. This
record describes the planned architecture for those parts.

The hardware soak test showed sporadic, scattered sequence gaps. It did not
show consecutive bursts. This is normal packet loss behaviour for UDP on a
2.4 GHz network. The motor firmware therefore repeats completed summaries
every 2 seconds and completed split series every 10 seconds. The authoritative
result stays in RAM on the Pico. No measurement depends on a telemetry packet
arriving.

The local credential file `src/wifi_config.h` is ignored. The tracked
`src/wifi_config.h.example` contains placeholders and copy instructions. A
compiled `.uf2` contains the Wi-Fi credentials in plain text, so firmware
binaries must not be shared.

The listener accepts unauthenticated UDP from the LAN. This is an open security
item. Restrict UDP port 4242 with a firewall rule to the local network or the
Pico's reserved DHCP address.

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
        | Validate, calculate, and store results
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
- Send calibration telemetry to the Raspberry Pi 400 LAN address.
- Send each measurement with a session identifier and sequence number.
- Continue the motor test if Wi-Fi or the receiver becomes unavailable.
- Keep the completed result in RAM.
- Repeat the completed result so the receiver can recover missed packets.
- Use GP20 for test start and GP21 for emergency stop.
- Never depend on a network command for emergency stopping.

### Raspberry Pi 400

- Listen for Pico telemetry on UDP port 4242 through the LAN interface.
- Reject malformed, duplicate, or out-of-order packets as necessary.
- Calculate comparisons and store calibration sessions.
- Provide live charts, status, and result tables.
- Bind the web dashboard only to `127.0.0.1:8080`.
- Keep dashboard operation separate from motor control.

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

Each packet should contain at least:

- Protocol version.
- Device identifier.
- Calibration session identifier.
- Sequence number.
- Pico monotonic timestamp.
- Test type and current state.
- PWM duty.
- Motor 1 and motor 2 encoder counts.
- Motor 1 and motor 2 RPM.
- RPM difference.
- Startup, stall, emergency-stop, and error states.
- A flag that identifies a completed result.

UDP does not guarantee packet delivery. Sequence numbers reveal missing packets, and repeated completed results provide recovery without blocking motor control.

## Initial dashboard scope

The first dashboard must be read-only. It should show:

- Pico connection state and time of the last packet.
- Current calibration stage and elapsed time.
- PWM duty and startup-kick state.
- Live encoder counts and RPM for both motors.
- Motor speed difference.
- Stall, timeout, and emergency-stop warnings.
- Completed result tables and RPM-versus-duty charts.
- Calibration history with CSV or JSON export.

Remote motor start and remote emergency stop are outside the initial scope. GP20 and GP21 remain the physical controls.

## Power and failure rules

- Power the motors and Pico W from the regulated 5 V VIN supply during a motor test.
- Keep the Pico USB cable disconnected while the motors operate.
- Wi-Fi failure must not stop, delay, or change motor safety logic.
- Network transmission must not block encoder processing or emergency-stop checks.
- If the dashboard is unavailable, the Pico must finish safely and retain the result in RAM.
- If the Pico stops transmitting unexpectedly, the dashboard must show a stale-connection warning.

## Configuration and secrets

Do not commit Wi-Fi credentials. Keep them in an ignored local configuration file or provide them during the local build. The firmware binary will still contain the credentials needed to join the network, so distribute firmware files carefully.
