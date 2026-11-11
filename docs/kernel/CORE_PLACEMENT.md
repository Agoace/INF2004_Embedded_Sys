# Core placement and lock discipline

Status: Phase 1 record for the MTK3 core migration. Reference branch
`mtk3-core-migration`. No device result is claimed here.

## Placement seam

`app_program/app.h` defines `app_core_t` (`APP_CORE_0`, `APP_CORE_1`) and
`app_prc_core0()` / `app_prc_core1()`. `TP_PRC1` is core 0, so core 1 is
`TP_PRC1 + 1`; in a single-core build both helpers return `TP_PRC1` because
there is nowhere else to run.

`app_program/platform/app_task.c` owns the placement table. Each `slot_t`
records the core that task is allowed to run on, and `app_task_start()`:

- requires the **creator** to be on core 0,
- accepts `TP_PRC1 + 1` only when `TK_SUPPORT_SMP` and `TK_MAX_CORE > 1`,
- rejects any other PRC with `E_PAR`,
- rejects a request that disagrees with the slot table with `E_CTX`,
- sets `c.assprc = core`, so the kernel attribute follows the request.

The last point is a repair. The attribute used to be a fixed `TP_PRC1`, which
would have pinned every task to core 0 no matter what a caller asked for.

A cyclic handler must use the same core as the task it feeds. Use
`app_prc_core0()` / `app_prc_core1()` for `T_CCYC.assprc` rather than a literal.

Moving a task to core 1 is therefore two edits in one commit: its `slot_t` entry
and its call site. A call-site edit alone is refused.

## Lock rule

`include/tk/smp.h` states the constraint this rule encodes: masking interrupts on
one core says nothing about the other, and a lock taken by a task and by a
handler **on the same core** must use the interrupt-preserving variant, or the
handler spins against the task it interrupted and neither proceeds.

1. A lock reachable from a handler **and** a task on the same core uses
   `ISpinLock` / `ISpinUnlock`. Never plain `SpinLock`.
2. A lock reachable only from tasks may use `SpinLock`.
3. Locked atomics (`atomic_*`) are hardware spinlock claims plus an
   interrupt-disable window on Cortex-M0+, which has no LDREX/STREX. Use them for
   low-rate state and counters. Do **not** take one per sample on the 1 ms Line
   path; use a single-writer board with an aligned word instead.
4. Register publication uses a single writer plus a sequence, not a lock, so a
   reader can detect a torn or skipped read without ever blocking a writer.

## DI/EI inventory

`DI()`/`EI()` protect against this core only. Each site below is recorded with a
verdict: **keep** (no cross-core reader is introduced), **replace** (the mask
must give way to a cross-core contract in a later phase), or **convert** (reached
by a handler and a task on the same core once a task moves).

| Site | Protects | Touched today by | Verdict |
| --- | --- | --- | --- |
| `platform/app_task.c:209,214` | stack high-water marks | caller of `app_task_stacks()` (Health, core 0) | **convert** when a core-1 task's stack is measured (Phase 2) |
| `platform/drv_gpio_irq.c:47-54` | GPIO IRQ client table | init, core 0 | keep - boot-time, one core |
| `platform/idle_meter.c:45,51` | core-0 CPU meter | idle task and meter reader, core 0 | keep - core 0 by definition; add a separate core-1 meter |
| `subsystem1/log_stream.c:468,538` | RAM log rows and index | Telemetry owner and Wi-Fi reader | **replace** with the two-word contract in Phase 3 |
| `subsystem1/network_queue.c:34,170,192` | network queue counters | Wi-Fi task and producers | **replace** in Phase 4, or keep if the task moves wholesale |
| `subsystem1/robot_mqtt.c:90,256` | MQTT metrics | Wi-Fi task only | keep - single-task owner, moves wholesale in Phase 4 |
| `subsystem1/telemetry.c:133,144,215,346,361` | stats, busy, loss counters, `log_generation` | Telemetry task and core-0 readers | **replace** in Phase 3 - this is the main cross-core contract |
| `subsystem2/motion.c:96,157` | encoder counts, GP20 copy buffer | motion task and encoder IRQ, both core 0 | **keep** - the IMU reads the published Motion board, not the counts |
| `subsystem3/drv_line.c:59-66` | line ring buffer | 1 ms ADC IRQ and line task, both core 0 | keep - core 0 by placement |
| `subsystem5/drv_ultrasonic.c:44` | listener id | echo IRQ and sonar task, both core 0 | keep - core 0 by placement |

The encoder note is a decision, not an oversight: the counts stay owned by the
core-0 IRQ and the Motion task, so their local `DI()`/`EI()` needs no conversion
merely because a core-1 task later reads a published board.

## Handler-versus-task stress test

`app_program/platform/app_lock_stress.c` builds only with `make TEST=1` and runs
from `app_main.c` with the other on-target tests.

It creates a 1 ms cyclic handler that runs in handler context and calls
`ISpinTryLock()` on a lock the test task also takes. The task takes the lock with
`ISpinLock()`, records that it is held, spins long enough that a 1 ms handler
would certainly run if it could, and releases.

The pass condition is that the handler attempts the lock many times, **never**
observes it held, and the run completes. That is the rule made observable: with
`ISpinLock` the task has interrupts masked, so the handler cannot preempt it while
the lock is held.

The test deliberately does not try the plain `SpinLock` variant. On this port the
handler would spin against the task it interrupted and the board would hang, so
demonstrating the failure would not be a recoverable test.

## Phase 2: IMU on core 1

`app_task.c` records `SLOT("imu_task", imu_stack, APP_CORE_1)`, and
`imu_service.c` starts the task with `app_prc_core1()` and sets the same PRC on
its cyclic handler, so the handler and the task it feeds share a core. In a
single-core build both helpers collapse to `TP_PRC1`, the pair still agrees, and
the image is unchanged.

The IMU writer now runs on core 1 while Vehicle and Health read from core 0.

### Board publication: verified, not rebuilt

`platform/app_board.h` already supplied the contract this phase needs.

- **One writer per board.** The three IMU boards are published only from
  `imu_service.c`: the snapshot `board` at line 261, `hump_board` at 252 and
  `diag_board` at 179. No other file publishes them.
- **A sequence.** The writer makes it odd, writes the words, then makes it even.
  A reader rejects a zero or odd sequence and requires it to be unchanged after
  the copy, so a torn read is detected rather than accepted.
- **The barrier.** Every word access is an aligned 32-bit `__atomic_*` operation
  with `__ATOMIC_SEQ_CST`. That supplies the ordering RP2040 needs, since it has
  no data cache, so no separate `mp_memory_barrir()` was added: an explicit
  barrier on top would be redundant and future readers would have to reason
  about which one does the work.
- **Unavailable stays unavailable.** A failed copy is not a value. Callers keep a
  scratch object and commit only on success, and `imu_snapshot_try_read()`,
  `imu_hump_try_read()` and `imu_hump_diag_try_read()` return `E_OBJ`, `E_PAR` or
  `E_TMOUT` rather than a zero or a stale value with a fresh timestamp.

Only the header's "core-0 publication" wording was stale, and it is corrected.

### Stack marks

`app_task_stacks()` takes `ISpinTryLock` around the high-water update and reports
the last mark on contention. Scanning a core-1 task's stack from core 0 is still
a sampled read, so the mark is approximate and monotonic, never exact.

## Open validation items

None of these is closed, and none may be reported as a PASS without device
evidence.

1. The stress test has not been run. It needs `make TEST=1` on a Pico W with the
   radio off, and a console capture.
2. No core-1 task exists yet, so the slot-table core field is exercised only for
   core 0. Phase 2 is the first real exercise.
3. Cross-core load, tick gaps, line losses, IMU read/work and Wi-Fi poll gaps are
   unmeasured. The Phase 0 images exist on this branch; the dynamic numbers do
   not.
4. `ISpinLock` costs a hardware spinlock claim plus a mask on every acquisition.
   Its cost on the 1 ms Line path is unmeasured, which is why rule 3 keeps
   locked operations off that path.
5. Phase 2 has not run on a device. Nothing shows that a core-1 IMU task
   executes, that its cyclic handler fires at 10 ms on core 1, that samples
   reach the Vehicle board intact, or that the stack mark for a core-1 task is
   usable. The compile passes prove only that both profiles build.
6. The other boards still have writer and readers on core 0 until their phases
   land: `motion.c`, `line_service.c`, `robot_health.c`, `telemetry.c`,
   `junction_source.c` and the five boards in `vehicle_task.c`. Their `DI()`/`EI()`
   and board contracts are unchanged by Phase 2.
