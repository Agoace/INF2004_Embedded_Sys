# Steering Wi-Fi log

Status: works. 2026-10-29: after run 1 (broadcast lost most packets) it
sends to the receiver directly; run 4 of the steering test arrived live,
1315 of 1320 entries.

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
   UDP port 4243, to the laptop                   scripts/steer_log_receiver.py
                 ◀── HELLO broadcast once a second ──
```

- The receiver broadcasts a HELLO packet once a second. The robot then
  sends to the address of the HELLO (unicast); until then it sends to the
  subnet broadcast address. The laptop needs no fixed IP address, and its
  address is not in the firmware.
- Unicast, not broadcast, for the log: Wi-Fi repeats a lost unicast
  frame, but sends a broadcast frame once, at the lowest rate, with no
  repeat. With broadcast only, run 1 (below) lost about 75 % of the live
  packets and all of the replay.
- Any device on the network can send a HELLO and receive the log. This is
  acceptable on a private hotspot.
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
| printf output goes to a 2 KB buffer, sent from the main loop once the receiver is known | Text that does not fit is dropped, never waited for; text sent by broadcast before the HELLO was lost |
| Live log: 5 entries every 250 ms | 4 small packets per second |
| Replay: the whole kept log (up to 20 s) is sent again when a run stops, and when the receiver is first heard with a saved run; a replay waits for the receiver's HELLO | The receiver fills in lost live packets; a broadcast replay lost 6 packets |
| The receiver saves a run when the last replay packet arrives, or 3 s after the last replay packet | A lost last packet does not lose the run |

Without `src/wifi_config.h` the firmware builds without Wi-Fi, with a
compiler warning.

## Packet format

The format is in [`src/steer_log.h`](../src/steer_log.h). Each field has
only the bits it needs (Change Requirement 1).

Header, 5 bytes:

| Bytes | Field |
| --- | --- |
| 0 | Bits 0–3 type (1 LOG, 2 TEXT, 3 HELLO, 4 BYPASS), bits 4–7 version (1) |
| 1–2 | Boot id (random at each start) |
| 3–4 | Sequence number (finds lost packets) |

LOG packet: header, then:

| Bytes | Field |
| --- | --- |
| 5 | Run number (0: a saved run from before the restart) |
| 6 | Flags: bit 0 replay, bit 1 last replay packet, bit 2 `bypass_test` run (mode 3 is BYPS) |
| 7–8 | Index of the first entry (time = index × 50 ms) |
| 9 | Number of entries, at most 20 |
| 10… | Entries, 11 bytes each |

Entry, 88 bits, least significant bit first:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–7 | Position, −100…+100 | 8 |
| 8–15 | Left duty %, negative = reverse | 8 |
| 16–23 | Right duty % | 8 |
| 24–31 | Info: mode (0 FOLW, 1 SRCH, 2 STRT straight across a junction, 3 CALM following with the calm settings), side, side source, wheel hint (2 bits each) | 8 |
| 32–67 | Raw L, C, R ADC readings | 3 × 12 |
| 68–87 | Left and right wheel counts in the 50 ms, clamped to ±511 | 2 × 10 |

TEXT packet: header, then text.

BYPASS packet (`bypass_test`, added 2026-10-30): header, then run number
(1 byte), index of the first record (2 bytes), number of records (1 byte,
at most 40), then records of 5 bytes. One record per ultrasonic reading of
the bypass ([OBSTACLE_BYPASS.md](OBSTACLE_BYPASS.md)), 40 bits, least
significant bit first:

| Bits | Field | Width |
| --- | --- | ---: |
| 0–12 | Time since the run start, 10 ms units (to 81.91 s) | 13 |
| 13–18 | Servo angle of the reading + 30 | 6 |
| 19–29 | Reading: ultrasonic distance (9 bits) and status (2 bits) | 11 |
| 30–31 | Bypass state after the reading: 0 PIVOT, 1 AROUND, 2 DONE, 3 FAILED | 2 |
| 32–37 | Steering angle after the reading + 30 | 6 |
| 38–39 | Spare, 0 | 2 |

The fault streak (3 bits) is not sent: a streak ends the bypass, and the
run summary says so. In a `bypass_test` run, mode 3 of the 88-bit entry is
BYPS and its position field is the steering angle in degrees.

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
5. GP21: spin, then the run (45 s, or GP21 again to stop). The receiver prints the text and the
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
the bypass record vector (clamps, spare bits, header), round trips at the field limits, the ±511 clamp, the 12-bit raw mask, and
the header bytes. The Python test decodes the vector, runs one live and
one replay packet through the receiver with one lost packet, and checks
the save 3 s after a replay whose last packet is lost. It also decodes
the bypass record vector, checks the reading names, and saves a
`bypass_test` run with its bypass table.

Result on 2026-10-29: all tests pass. Five C rule changes (field moved by
one bit, no sign extension, wrong clamp, bits not cleared, high header
byte lost) and three Python rule changes (no sign extension, field moved,
lost packets not counted) each make a test fail. A loopback send to the
receiver printed the text and the entry.

Update 2026-10-30 (bypass records): all tests pass. Five C rule changes
(servo field moved, no time clamp, no angle clamp, spare bits not cleared,
wrong header byte) and five Python rule changes (servo field moved, flag
ignored, TOO_NEAR shown as 0 mm, no bypass table, wrong mode name) each
make a test fail.

## Track test

### Run 1: 2026-10-29, broadcast only

The robot joined the iPhone hotspot in a few seconds (`172.20.10.8`) and
sent the saved run from flash: 380 of 400 entries. During a 78 s run the
receiver got about 400 of about 1565 live entries, in gaps of 0.25–3 s,
and none of the summary or the replay after the stop. The cause is
broadcast: no Wi-Fi repeats. The HELLO and unicast were added after this
run.

### Fixes after run 1, 2026-10-29

| Problem seen | Fix |
| --- | --- |
| The replay at start was sent before the HELLO, by broadcast: 6 packets lost | The replay waits for the receiver |
| Text was sent by broadcast before the HELLO and lost | Text waits for the receiver |
| The first 11 packets to the receiver were lost and not counted: lwIP keeps only one packet while ARP (finding the laptop's hardware address) is open; `ARP_QUEUEING` did not help | The robot sends nothing to the receiver until ARP is done; the receiver counts packets missing from the start |
| The summary printed the current settings, not the settings of the saved run | The run stores its settings |

After these fixes a restart sent all the text and the saved run: 400 of
400 entries, 0 packets lost.

## Open items

- First test on the robot: join time, packet loss on the track, and any
  effect on the 1 ms sampling.
- Persist the firewall rule if the log is used often.
