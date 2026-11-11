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
