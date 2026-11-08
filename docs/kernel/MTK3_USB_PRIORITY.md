# USB console task priority — STEP 8 option A

## Change

The template USB console service task ran at priority 3, above every
application task. The local robot now creates it at **21**. This is the
eighth path in the [template ledger](../architecture/TEMPLATE_CHANGES.md). Only the task
priority constant changes. The 4096-byte drop-on-full output ring, USB IRQ
level 0, the IRQ-to-task wake, the 10 ms fallback poll, core-0 pinning and
TinyUSB configuration are unchanged.

## Why

The DIAG=1 capture of 2026-11-07 measured the motion delay
([hardware record](../archive/MTK3_STEP8_RECORD.md#diag1-motion-timing-capture--2026-11-07)).
While a GP20 dump drained, the motion cyclic release stayed on time
(release interval 9995 us), but motion entered 4488 us after release.
Mutex wait was at most 15 us. The USB task at 3 can drain the ring
continuously: each USB IRQ queues a new wake, so `tk_slp_tsk` returns at
once while host ACKs continue. At 3 that drain outranks motion 10.

## Choice of 21

Lower numbers run first. Priorities on core 0 after the change:

| Priority | Task | Relation to USB 21 |
| ---: | --- | --- |
| 1 | Initial task during startup | Above; startup output is buffered, as before |
| 10–13 | motion, line, sonar (BENCH 11), vehicle | Above; console output never delays driving |
| 20 | Wi-Fi service; TEST runner | Above; network and test work are not delayed by output |
| **21** | **USB console** | — |
| 30 | telemetry (GP20 dump producer) | Below; the drain preempts the dump producer |
| 31 | health | Below |
| 32 | CPU meter (always ready) | Below; USB must stay above it or it never runs |

21 is the first free level below Wi-Fi and the TEST runner. A level at or
below 30 would let the dump producer refill the ring before it drains, and
32 would starve USB behind the always-ready CPU meter.
`tests/test_task_priorities.py` checks this order from the source constants.
It fails on the old value 3.

## Consequences checked

| Area | Analysis | Hardware check |
| --- | --- | --- |
| GP20 dump | Producer 30 is below drain 21. The dump already yields 20 ms every 8 rows (312 B per KR chunk, measured); its largest unyielded segment is DIAG print, header, counts and stack report: 1152 B in the second DIAG dump. No new pacing or busy-wait is needed | `tm_usb_dropped` 0 and dump delta 0 |
| BENCH dump | Producer 11 is above USB. It yields 20 ms after each step line and every 8 MB rows (about 400 B) | Bench dump delta 0 |
| TEST output | Runner 20 is now above USB. Its largest burst between sleeps is about 1.5 KB (S2-U01 report, PROBE, Wi-Fi lines and S2-U02 warning before the 2 s wait), below the 4096-byte ring. S2-U05 keeps its 1 s delivery sleep before suppressing feeds | Full TEST log present, including both S2-U05 lines |
| Boot output | Unchanged: the initial task at 1 always outranked USB. Output queues in the ring until enumeration | Banner and boot lines present after cold power |
| Enumeration | The IRQ still wakes the task at once. Now the task runs only when no 1–20 task is ready. In steady state those tasks are short (Wi-Fi work max about 3 ms). One risk: CYW43 firmware download (about 224 KB in 64-byte blocking DMA blocks) has no yield, so USB can wait for it during boot. Linux allows 5 s per control transfer, but this is not measured | Cold-power enumeration of MQTT and MQTT TEST images; no repeated `device descriptor read` errors |
| picotool / BOOTSEL | No USB reset interface or 1200-baud reboot is built in. BOOTSEL is the ROM button path, independent of firmware priorities | Unchanged |
| Driving and watchdog | No control, watchdog or network-ownership change | Repeat DIAG capture: no dump-phase dispatch delay |

## Validation plan on hardware

1. Build `make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 DIAG=1` on this candidate.
2. Repeat the 2026-11-07 capture: run, MQTT ESTOP, GP20, wait about 10 s,
   GP20 again. The pass condition is zero dump-phase observations and a lifetime gap
   inside 9000..11000 us after both dumps.
3. Check `tm_usb_dropped` 0 and dump delta 0 on both dumps.
4. Cold-power the MQTT TEST image; check enumeration and the full TEST log.

## Hardware result (2026-11-07): PASS

MQTT DIAG image built on `94015fd` with the local secrets, flashed with
`picotool load -f` and `picotool verify`.

| Check | Result |
| --- | --- |
| U1: cold power, USB only | `ttyACM0` enumerates; the boot log starts at the Wi-Fi join, as before the change; MQTT connects |
| U2: robot run, then GP20 dump 1 | 206 rows, 7 events; `tm_usb_dropped` 0, dump delta 0 |
| U2: GP20 dump 2 (reports dump 1) | `[MOTION-DIAG] lifetime observations=0`; gap 9963..10041 us |
| Before the change (same procedure) | 18 observations during the first dump; worst gap 14467 us |
| Dashboard DUMP after the run | Complete, 206 rows, 7 events |

The run stop used GP21. The MQTT ESTOP was not tested in this run: the
browser page held the CSRF token of an earlier dashboard process, and the
dashboard refused the commands with 403. This is a dashboard issue, not a
firmware issue. The full MQTT TEST image log under the new priority is
**Not run**; the largest TEST burst (about 1.5 KB) fits the 4096 B ring.

The per-run gap window (option B) is dropped: the dump delay is gone, so
the lifetime gap is no longer skewed by dumps.

## Software validation

Candidate `mtk3-staging-next`, worktree `/tmp/mtk3-usbprio`. Base `5bda9ee`;
`831c9c0` (package 2, SCAN reverse and reattempt) merged with `--no-ff`.
Local only: no push or flash.

| Check | Result |
| --- | --- |
| Host CTest | 117/117 PASS, including the new priority-order check |
| Python decoder/log and dashboard suites | 63 and 138 PASS against the C fixture emitter |
| Priority-order check on old value 3 | Correctly fails (USB above driving, Wi-Fi and TEST) |
| Firmware profiles, warning-free with `-j4` | 7/7: default, robot, TEST, BENCH, MQTT, MQTT TEST, MQTT DIAG |
| Strict ARM invocations on changed C units since `5bda9ee` | 39/39 PASS |
| SDK link proof | 7/7 PASS |
| Binary check | `tm_usb_console_start` stores 21 into `T_CTSK.itskpri` (offset 12) in TEST, MQTT DIAG and SMP default images |
| Docs, template ledger, `git diff --check` | PASS; template ledger 8 paths |

The hardware result is in the section above.
