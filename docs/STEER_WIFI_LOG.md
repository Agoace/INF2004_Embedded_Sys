# Steering Wi-Fi log

Status: built 2026-10-29; host tests pass; not yet tested on the robot.

`steer_test` sends its printf output and its 50 ms run log over Wi-Fi, so a
run on the track can be watched from the laptop without USB. The flash log
([line steering bench test](LINE_STEERING.md#bench-test)) stays as the
backup.

## Why

The track is away from the PC. A run that ends with a power cut saves no
flash log, and USB must stay disconnected while the motors run
([USB power safety](USB_POWER_SAFETY.md)).

## Network

```text
Robot (Pico W) ──Wi-Fi──▶ phone hotspot (2.4 GHz) ◀──Wi-Fi── laptop
   UDP broadcast, port 4243                       scripts/steer_log_receiver.py
```

- The robot sends to the subnet broadcast address. The laptop needs no
  fixed IP address, and its address is not in the firmware.
- The Pico W supports 2.4 GHz WPA2 only. School networks that need a user
  login, or that block traffic between devices, do not work.
- Port 4243 is separate from the JSON telemetry on port 4242
  ([Wi-Fi telemetry dashboard](WIFI_TELEMETRY_DASHBOARD.md)), because the
  packets are binary.

## Rules on the robot

| Rule | Why |
| --- | --- |
| Joins in the background; the robot never waits for Wi-Fi | Wi-Fi must not change driving |
| Joins again 15 s after a lost link | The robot can drive out of range and back |
| A packet without a link is dropped | The flash log is the backup |
| The Pico W LED is on while joined | Check at the track without a PC |
| printf output goes to a 2 KB buffer, sent from the main loop | Text that does not fit is dropped, never waited for |
| Live log: 5 entries every 250 ms | 4 small packets per second |
| Replay: the whole kept log (up to 20 s) is sent again when a run stops, and when Wi-Fi joins with a saved run | The receiver fills in lost live packets |

Without `src/wifi_config.h` the firmware builds without Wi-Fi, with a
compiler warning.

## Packet format

The format is in [`src/steer_log.h`](../src/steer_log.h). Each field has
only the bits it needs (Change Requirement 1).

Header, 5 bytes:

| Bytes | Field |
| --- | --- |
| 0 | Bits 0–3 type (1 LOG, 2 TEXT), bits 4–7 version (1) |
| 1–2 | Boot id (random at each start) |
| 3–4 | Sequence number (finds lost packets) |

LOG packet: header, then:

| Bytes | Field |
| --- | --- |
| 5 | Run number (0: a saved run from before the restart) |
| 6 | Flags: bit 0 replay, bit 1 last replay packet |
| 7–8 | Index of the first entry (time = index × 50 ms) |
| 9 | Number of entries, at most 20 |
| 10… | Entries, 11 bytes each |

Entry, 88 bits, least significant bit first:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–7 | Position, −100…+100 | 8 |
| 8–15 | Left duty %, negative = reverse | 8 |
| 16–23 | Right duty % | 8 |
| 24–31 | Info: mode, side, side source, wheel hint (2 bits each) | 8 |
| 32–67 | Raw L, C, R ADC readings | 3 × 12 |
| 68–87 | Left and right wheel counts in the 50 ms, clamped to ±511 | 2 × 10 |

TEXT packet: header, then text.

## Use

One-time setup:

1. Copy `src/wifi_config.h.example` to `src/wifi_config.h` and set the
   hotspot name and password. The file is ignored by Git. The `.uf2`
   contains the password as plain text; do not share it.
2. Build and flash: `cmake --build build --target steer_test`.

Each session:

1. Turn on the phone hotspot (2.4 GHz) and connect the laptop to it.
2. Open UDP port 4243 on the laptop until the next restart:

   ```sh
   sudo firewall-cmd --add-port=4243/udp
   ```

3. Start the receiver:

   ```sh
   python3 scripts/steer_log_receiver.py
   ```

4. Power the robot from VIN. The LED comes on when it joins; the receiver
   prints `[wifi] up, IP …`.
5. GP20 spin, GP21 start, GP21 stop. The receiver prints the text and the
   live log. After the stop it receives the replay and saves the run to
   `build/steer_logs/run_<time>_boot<id>_run<n>.txt`, with the number of
   entries received live and the number of lost packets.

## Host tests

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Isrc tests/test_steer_log.c \
   src/steer_log.c -o build/test_steer_log
./build/test_steer_log
python3 scripts/steer_log_receiver.py --self-test
```

Both sides use the same 11-byte test vector. The C test checks the vector,
round trips at the field limits, the ±511 clamp, the 12-bit raw mask, and
the header bytes. The Python test decodes the vector and runs one live and
one replay packet through the receiver, with one lost packet.

Result on 2026-10-29: all tests pass. Five C rule changes (field moved by
one bit, no sign extension, wrong clamp, bits not cleared, high header
byte lost) and three Python rule changes (no sign extension, field moved,
lost packets not counted) each make a test fail. A loopback send to the
receiver printed the text and the entry.

## Open items

- First test on the robot: join time, packet loss on the track, and any
  effect on the 1 ms sampling.
- Persist the firewall rule if the log is used often.
