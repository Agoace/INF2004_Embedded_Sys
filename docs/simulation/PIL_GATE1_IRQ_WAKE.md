# Gate 1 — request dispatch after IRQ exclusion

## Finding and approved scope

Attempt7 used the [priority9 image](PIL_GATE1_PRIORITY.md). It had no measured
Pico fault or underflow. The PC stopped it with DISARM7 after a source-phase
error above250us. This was not a transport PASS.

PRIMASK is the CPU mask:0 permits normal interrupts;1 masks them. Exception
entry does not itself set PRIMASK1. PendSV is the kernel's deferred task-switch
exception. The UP kernel marks a woken task ready, but requests PendSV only if
its critical section's SAVED PRIMASK was0 and dispatch is enabled:

- `kernel/sysdepend/cpu/core/armv6m/cpu_status.h:61-70` — dispatch condition.
- `kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c:193-196` — set PendSV.
- `kernel/sysdepend/cpu/core/armv6m/interrupt.c:40-66` — HLL exit only reduces
  the task-independent nesting count; it has no extra dispatch check.

The Gate1 UART IRQ called `tk_wup_tsk` inside DI. Its saved mask was1. Restoring
interrupts after that call did not request the missing dispatch. Priority9
could be ready but wait for an unrelated kernel call, often the next native
line alarm's unmasked listener wake. This is a Gate1 call-site correction,
not a global kernel change or proof that a lower-priority task ran for0.9ms.

The operator approved the same correction for the Gate1 lease IRQ and native
alarm hook. Only [pil_gate1_task.c](../../app_program/sim/pil_gate1_task.c)
changes target behavior:

1. Publish FIFO/queue/session changes under DI.
2. Restore the saved mask with EI.
3. Call `tk_wup_tsk` once outside that exclusion. UART and native-hook causes
   use a local `need_wake` flag; the lease IRQ always needs one wake.

UART metric11 still brackets the wake call. A second short DI section protects
its final metric update. It does not clear the pending PendSV. The bracket ends
before that update, as before; it is not a hardware entry/exit measurement.
The native alarm done timestamp also retains the wake bracket, with a second
short DI for its final update. No formatting, CRC or decode moves into an IRQ. Priority9, RX1, real native tasks/watchdog, bounded queues,
UART v1, D1 v1 and all timing limits stay unchanged. OptionsB/C remain closed.

## Attempt7 retained timing evidence

Source: `home-gate1-20261114T072530Z/latency_final.json` in Main's ignored
`build/webots/` evidence. These are retained records, NOT whole-run quantiles.
SENSOR RX-to-worker: n63, min15, median83, mean199.746, nearest-rank p90=797,
p95=836, p99=max879us. Bins: below100=44;100..249=8;250..499=0;500..749=3;
750..999=8. All11 long records had RX depth1.

| Serial | SENSOR sequence | RX time, Pico us | RX-to-worker us |
| ---: | ---: | ---: | ---: |
|167|153|1459498|834|
|172|158|1464451|879|
|176|162|1468486|852|
|179|165|1471604|726|
|182|168|1474576|753|
|190|176|1482451|879|
|198|184|1490641|690|
|200|186|1492494|836|
|203|189|1495489|590|
|212|198|1504493|836|
|219|205|1511551|797|

Ten starts were32..51us after the next native grid point; sequence189 was the
exception. Decode took81..111us, dispatch wait2..6us and dispatch28..32us in
these records. The principal recorded delay preceded the worker. This supports
the source finding; it does not measure target PendSV latency, USB/XIP stalls
or every possible interrupt/dispatch exclusion interval.

## RED and GREEN

- [generate_pil_critical.py](../../tests/generate_pil_critical.py) copies the
  ACTUAL production BEGIN/END critical macros into an ignored test header.
  The register/kernel double models PRIMASK, ready selection, dispatch-disable
  state and the PendSV request. It does not copy the dispatch condition.
- Runtime modes `wake`, `wakelease`, `wakenative` each failed before the target
  correction: ready selected, no PendSV. Retained RED logs:
  `build/pil-gate1/irqwake-red-{wake,wakelease,wakenative}.log`.
- Nine UP/split/control cases now pass. They also show restoring mask0 alone
  is insufficient, dispatch-disable is respected, RX publication precedes wake,
  TX completion wakes, and simultaneous UART error/completion coalesces to one
  wake. No raw SCB write or kernel-contract bypass is added to Gate1.
- GCC598/598 and Clang ASan/UBSan598/598 pass. Logs:
  `build/pil-gate1/irqwake-{gcc,clang}-{build,tests}.log`.
-45 strict ARM compilations pass.38 ordinary builds/SDK proofs pass; all76
  literal ELF/UF2 hashes including debug metadata equal baseline451e562 at the
  retained public path: `build/pil-proof/gate1-irqwake/hashes.txt`.
- RAM delta0: added14796B = g12744 + stack2048 + timestamp4. Whole UP image:
  data696, bss140076, text123268B. UART frame80B, native hook40, lease16,
  service440; reviewed worker896B unchanged. Additional IRQ maximum204B UP
  (UART80 + FIFO48 + note24 + fail40 + trace12),248B SMP
  (UART80 + wake16 + release8 + ready16 + reschedule48 + assign72 + mask8).
  Both remain below256B. Existing exception/HLL/native stack is separate.
- Clean/missing-lookahead C fixtures remain byte-identical to RX1/priority9:
  SHA256 `e27320790310ba1ff99b446a1d3f20c541595040e75093cfbd3404814909f1b3`
  and `3b11631a8f561df600a8b1e784fedc243a7e5b3333207e9416971d482a110e04`.
- The double is not a real kernel execution, SMP-lock/IPI test or physical
  latency proof. No new target stack reserve is claimed.

## Short robot/core-live audit — read only

Confirmed SW7's audit at core-live revision4d583c61ccf4feb50d21a70ceb6bf5d7f57efc9f
(`build/wip/p2/sw6_primask_dispatch_check.md`). No contradictory robot ISR trigger
was found. Paths below refer to this tree unless labelled core-live.

| Site | Result on UP and same-core SMP |
| --- | --- |
| `app_program/subsystem3/drv_line.c:35` (core-live44) | Listener wake outside DI |
| `app_program/platform/drv_gpio_irq.c`; `subsystem2/motion.c:78` | Dispatcher/encoder has no masked kernel wake; motor has no separate wake ISR |
| `app_program/subsystem5/drv_ultrasonic.c:74` | Echo callback wakes outside DI |
| `app_program/subsystem4/imu_service.c:291` | Cyclic release; no IMU I2C ISR |
| `app_program/subsystem2/motion.c:283`, `subsystem5/sonar_service.c:265`, `subsystem1/robot_health.c:85` | Cyclic wakes run under timer's outer critical section; `kernel/tkernel/timer.c:238` supplies dispatch on exit |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c:49` | Wrapper wake outside explicit DI; assumes SDK handler restores its incoming mask |
| core-live `app_program/platform/run_timing.c:353-369,372` | P1/P2 hooks mask plain stores only and restore the mask; no kernel call inside |
| `device/adc/sysdepend/rp2040/adc_rp2040.c:59-68` | Positive template hit: `tk_set_flg` inside ADC_LOCK; compiled but not opened by robot app |

Same-core SMP has the same saved-mask rule. `ready_queue_smp.c:355-360` sends
IPIs only to OTHER cores; those cross-core wakes do request receiver dispatch
and do not have this lost local request. The ADC template remains untouched.
TinyUSB internals were not audited line by line. This audit does not attribute
the robot's vehicle latency to the Gate1 defect. No robot/template/kernel edits.

## Image and operator handoff

File: `build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2`,248320B.
UF2 SHA256: `772d4e433242329264a327c0d24799ec89409b83ac7ce0af16041b8248a5907a`.
ELF SHA256: `6870689f252026daf14a4aa357b529ed28cb026ffe8abb75894849af37270ded`.
Retained artifacts: `build/pil-gate1/artifacts/irq-wake/`.

Use [restricted flags and handoff](PIL_GATE1_BUILD.md). Only the operator may
flash/reset/open ports. Use fresh identity and matching PC tools; retain D1,
RX1, complete stopped stack line and PCAP. Check RX-to-worker/decode/stage,
alarm jitter, watchdog and full cohorts. A complete600s transport run is still
required; no physical transport or sensor/physics PASS is supplied. No target
access, serial access, flashing or push occurred in this checkpoint.
