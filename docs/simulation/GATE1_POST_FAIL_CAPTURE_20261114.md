# Attempt15 post-failure evidence — 2026-11-14

**TRIAL / NOT GATE 1 PASS.** Operator-approved diagnostic capture, not recovery.
No new source, SENSOR, HELLO, ARM, epoch or lease renewal is allowed after FAIL.
Original failure and pre-tail active elapsed time remain unchanged.

## Probe UART: same owner

Add `--post-fail-rx-ms 1000` to the approved soak command. Choices are0(disabled,
unchanged default),500,750 or1000ms. After local inhibit and the existing bounded
DISARM attempt, the SAME reader collects RX only before closing. There is no
second reader, reconnect or call to the live transport decoder/clock estimator.
A131072-byte buffer and1000 chunk records bound storage; deadline checks bound
polling. Actual elapsed time is recorded: Linux/syscall delays can overrun a
requested wall deadline, so this is not a scheduling/WCET promise. Post-fault
polling pauses1ms; the active source-loop50us sleep is unchanged.

After close, `post-fail-rx.bin/json` retain bytes, RAW chunk times/offsets, original
failure, requested/actual interval, capacity/error/full flags and any decoded
CRC-valid BOOT nonce/sequence. A partial frame at the start is retained, not
promoted to a BOOT. Buffer/error/partial evidence stays incomplete. Exporting
this tail does not mean DISARM arrived or Pico PWM is known. Other offline
correlation tools must not treat these separately stored bytes as live source
receive/qualification evidence.

## Separate Pico CDC: reopen only this identified device

Use a separate terminal/process, never a second Probe UART reader:

```sh
python3 sim/webots/tools/gate1_pico_cdc_capture.py \
  --hardware-authorized --device "$PICO_BY_ID" \
  --output "$EVIDENCE/pico-cdc" --duration-seconds 700
```

`PICO_BY_ID` must be the operator-selected exact
`/dev/serial/by-id/usb-micro_T-Kernel_Forum_MTK3_Thread-Metric_RP2040_MTK3PICO-if00`
path for the MTK3 image (VID:PID2e8a:000a, serial MTK3PICO). The legacy
`/dev/serial/by-id/usb-Raspberry_Pi_Pico_*` names remain supported. Debug Probe
and all CMSIS-DAP names are rejected, including names with a Pico prefix. The tool opens RX-only/nonblocking, applies the same
CDC line settings as the prior reader, and never calls write. It prints
PICO_USB_CAPTURE_READY only after opening. On HUP/error it closes and watches
the SAME by-id identity for re-enumeration. There are at most16 segments,
4MB/4096 chunks each, and700s total configured duration. This is explicitly
approved CDC-only reopen, NOT UART auto-rearm.

Numbered `pico-usb-NNN.bin` files retain initial and post-reset data separately.
`cdc-capture.json` records open/close/chunk RAW times and complete reset records.
It stops after a complete reset record on a reopened segment, or a complete
stopped D1 plus stack line followed by1s RX/reopen reset-watch grace. D1 alone
must not close the reader before a later watchdog reset. The global duration
still bounds the configured watch; partial/lost records remain UNKNOWN. CLI exit2 never
means Gate1 PASS; retain the reason/segments, not just an exit status. Stop only
the owned process if operator cleanup is required.

## SW6 resetA format and image

Firmware commit `288d53df0773a2baf3c821852232f3dcfe46dd4e`, parent `40e7196`.
UF2 is248832B, SHA256
`a0a53f1a8ec419d433542ad8d0116f64f42a0ca6ca555ab0e6d551405d2c3da9`.
No image is flashed by this tooling/tests.

RESET1 is exactly122 bytes: `[PILG1RESET1 `,12 lowercase8-hex words with single
spaces, `]` and literal LF. No NUL/CRC/END row or CRLF conversion. Word order:
nonce, immutable boot session0, raw WATCHDOG.REASON, raw CHIP_RESET, scratch0..7.
Reason is NOT a D1 stop reason. Raw registers are copied before SDK scratch4
mutation; ROM may already have affected scratch. The row is not persistent
history or a HardFault breadcrumb. Once enqueued does not mean delivered.

Independent checks of SW6's actual C fixtures (122/5602/2393B) found one exact
row each: nonce7b/session0/reason1/chip100/scratch20..27. Partial/uppercase/wrong
session rows are not complete records. D1/RX1/UART v1 remain unchanged.

Compare new CDC nonce to Probe-tail BOOT only when BOTH are captured; otherwise
identity/cause remains unknown. A watchdog reason alone does not distinguish
HardFault, starvation or another missed feed. Missing text is not no-reset.
