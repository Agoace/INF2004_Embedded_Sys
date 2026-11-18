# TEST capture: reconnect USB before the frozen report

## Scope

This change is based on `f3e7d45`, not a staging merge. It changes only the
exclusive USB `TEST=1 BARCODE_CAPTURE=1` image. It does not change barcode
thresholds, decoding, travel limits, robot behaviour or the console layer.
B1c noise/motion/quiet-zone follow-up remains deferred.

## Operator procedure

1. Connect USB and open the capture terminal. Perform all three GP20
   floor/white/black ranges while connected; save their values.
2. Keep VIN power on. Unplug USB if needed, then press/release GP21 for
   the pass. Lift wheels for the first safety check before floor work.
3. When the robot stops, reconnect USB and open the terminal. A Linux
   `stty raw` plus `cat /dev/ttyACM*` session asserts CDC DTR on open.
4. Keep the terminal open. After one continuous second of DTR, the task
   prints the existing frozen stop report, decode results and full raw
   dump, with unchanged pacing. Save through `[BC_DUMP] END` and READY.
5. Release GP21, then use a fresh press for the next pass.

Do not remove VIN while waiting: the capture is volatile RAM, not a file.
Merely connecting a cable is insufficient; a terminal that never asserts
DTR leaves the task waiting. Contrast results are not queued/replayed.
There is no automatic replay if USB disconnects during an ongoing dump.
No LED or GP20 mode is added. UART capture builds fail explicitly because
this workflow requires `CONSOLE=usb_cdc`; other UART profiles are unchanged.

## Implementation

- Successful checked stop, ADC freeze and the final coast observation all
  precede the wait. The existing overrun/inhibition checks still apply.
- The hardware adapter reads the console's exported `tm_usb_state()`.
  DTR is the host signal used here to indicate an open terminal.
  State 3 means configured plus DTR; state 2 means enumeration only.
  This reads a cached state, not TinyUSB from a second task.
- Poll every 20 ms while yielding to USB. Any observed disconnect resets
  the one-second interval. Unsigned source-clock subtraction handles wrap.
- No report/dump or waiting notice is printed before the gate. Data stays
  frozen, zero drive stays latched and GP21 cannot queue a pass. Pending
  cyclic wakes are drained; button state is resynchronised after the dump.
- The console's global enumeration-based output policy is unchanged. DTR
  is required only to release this retained TEST report.

## Validation

Host cases cover an absent host, delayed DTR, disconnect during the settle
interval, GP21 pressed/released while waiting, GP21 held through READY,
source-clock wrap, unchanged frozen ADC/motor bytes and no queued restart.
The hardware double checks that only state 3 releases the gate. Existing
stop/fault, decoder and dump tests remain in the full host suite.

Candidate checks on the `f3e7d45` base:

| Check | Result |
| --- | --- |
| Full host suite | PASS, 180/180 (five new USB-wait cases) |
| Clang ASan/UBSan | PASS, 37 owner scenarios and hardware adapter |
| Firmware | PASS, warning-free radio-off robot and capture builds |
| Capture SDK link proof | PASS; never-run proof image, not for flashing |
| Strict ARM capture units | PASS, six; conversion/shadow/format checks |
| Normal robot ELF | No barcode/workspace/wait symbols |
| Capture text / data / BSS | 76576 / 152 / 158524 bytes |
| Owner / wait / PEAK-driver compiler frames | 328 / 24 / 64 bytes |

Static RAM stays 158676 bytes, unchanged from the base. Compiler frames
are not runtime high-water proof. No task stack or template file changed.
Validation logs are local in `build/usb-wait-validation/`.

Image: `build_make/mtk3pico_smp0_usb_cdc_test_barcode_capture.uf2`.
Powered unplug/reconnect and Linux-terminal timing checks are **Not run**.
No worker push or flash was made.
