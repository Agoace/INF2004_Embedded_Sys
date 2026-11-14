# Gate 1 — priority9 transport worker

## Decision and scope

This is the historical priority9 checkpoint. The current image also fixes
[IRQ dispatch requests](PIL_GATE1_IRQ_WAKE.md); use the current
[build handoff](PIL_GATE1_BUILD.md) for flashing, not the old hashes below.

The operator approved optionA after attempt6 localized late SENSOR staging to
Pico worker scheduling. SENSOR28 reached UART at1215509, about1.8ms before the
failed alarm1217309; RX-to-worker delay was1631us. Other recorded delays were
842,978 and920us, versus typical17..80us. This identifies worker scheduling
interference, but not the specific task consuming that time.

That checkpoint changed ONLY the Gate1 worker's initial priority16->9 in
[`pil_gate1_task.c`](../../app_program/sim/pil_gate1_task.c). Lower numbers have
higher priority. Keep [RX1 diagnostics](PIL_GATE1_RX_TIMING.md), all application
tasks, real alarm0/kernel time/watchdog, priority limits, deadlines,250us clock
limit, live UART v1 and D1 v1 binary framing. Do not implement task suppression
(optionB) or IRQ staging (optionC). No native application/template C changes.

## Source-derived task inventory

For the UP, MQTT0, TEST0, BENCH0 Gate1 image:

| Task | Priority | Relevant work |
| --- | ---: | --- |
| Initial/usermain |1|Sleeps indefinitely after startup |
| Motion |10|10ms control release; feeds real100ms watchdog |
| Line |11|1kHz wake; SIM ADC unavailable, sample queue empty |
| Sonar |12|10ms service release |
| Vehicle |13|Motion wakes and20ms idle/button gate |
| IMU |14|10ms release; NACK still requires bookkeeping/publication |
| PIL |9|Bounded RX/decode/stage/TX service, previously16 |
| USB CDC |21|USB event bursts plus10ms polling |
| Telemetry |30|Waits on telemetry message buffer when idle |

Barcode has no separate task. MQTT0 excludes health/idlemeter/network tasks.
TEST/BENCH0 excludes their runner, bench and stress tasks. USB/telemetry do not
normally preempt the old priority16 worker. The inventory is derived from
source/profile, not a runtime task-state census or individual-task CPU profile.

Priority9 puts PIL above robot tasks10..14, but not above the initial task,
interrupts, a kernel-lock owner or dispatch exclusion. PIL has no application
mutex dependency. Under excessive load it can delay the motion watchdog keeper;
retain the real100ms watchdog and verify the operator run rather than assuming
that raising priority establishes a timing guarantee.

## RED and GREEN evidence

- Runtime creation tests feed the ACTUAL requested worker priority into the
  production `kernel/tkernel/ready_queue.h` and `include/sys/queue.h`. Only the
  consumed TCB fields and kernel/CPU boundary are modelled. Mixed ready tasks
  have priorities10,11,12,13,14,21,30. Priority16 failed the insert/top assertion
  in `build/pil-gate1/priority-red.log` before the production change.
- Priority9 passes. Tests also verify initial priority1 and a kernel-lock owner
  can still take precedence, and that removal restores application selection.
  They do not prove real IRQ-to-PendSV latency or target execution time.
- Literal runtime creation assertion is9, stack2048/core0 unchanged. Three
  explicit priority cases join all18 existing UP/split/control runtime cases.
- Strict GCC589/589 and Clang ASan/UBSan589/589 tests pass. Logs:
  `build/pil-gate1/priority-{gcc,clang}-{build,tests}.log`.
-45 strict ARM units pass across UP/split/control. RAM delta is ZERO versus
  the RX1 image: g12744 + worker stack2048 + native timestamp4 =14796B added
  storage. Whole UP data696 / bss140076 / text123236B.
- C frames unchanged: UART72, native hook40, service440B. Reviewed additional
  IRQ maxima196B UP /240B SMP, worker896B. No new target reserve is claimed.
-38 ordinary firmware builds and SDK proofs pass. All76 literal ELF/UF2 hashes
  including debug data equal retained baseline451e562:
  `build/pil-proof/base/hashes.txt` = `gate1-priority9/hashes.txt` there.
- Runtime clean and missing-lookahead C fixtures are byte-identical to the RX1
  checkpoint; the existing PC parser/schema still applies. No clock/serial
  parameter was changed. All RX1 phase meanings are unchanged.

## Historical UP image and operator handoff

These are the priority9-only artifacts, not the current flash handoff.
File: `build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2`,248320B.
UF2 SHA256: `4b57af9da21c1d68719efe0c3acf4738b18e750f46501a325aa52dd43cb4f993`.
ELF SHA256: `e51655db34096911217a4020429ea59df6489124d91a70f8ece6ec40b33b9cd0`.
Retained artifacts: `build/pil-gate1/artifacts/priority9/`.

Only the operator flashes/resets and opens ports. Use a fresh nonce/session and
matching PC tools; retain RX1 plus PCAP evidence. Inspect RX-to-worker and decode/
stage intervals and native jitter; do not infer success from priority alone.
If latency spikes remain, inspect interrupt/dispatch exclusion or wake/reschedule
paths rather than relaxing bounds or automatically implementing optionsB/C.
A complete600s cohort is still required. No physical transport or sensor/physics
PASS is supplied by these host checks. No target access, serial access, flashing
or push occurred in this checkpoint.
