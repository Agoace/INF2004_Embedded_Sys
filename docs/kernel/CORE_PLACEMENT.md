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
| `subsystem1/log_stream.c:468,538` | DUMP request state (`s_dump`) | requested from Vehicle on core 0 (`vehicle_task.c:404`), consumed by the Wi-Fi task on core 1 | **converted in Phase 4**: `dump_lock` guards the request and its consumption. The RAM-log row contract was closed in Phase 3 |
| `subsystem1/network_queue.c` | queue loss and busy counters, unavailable mask | Wi-Fi task on core 1; Health copies on core 0 | **converted in Phase 4**: `queue_lock` guards `lost()`, the board-result recorder and `robot_queue_metrics()` |
| `subsystem1/robot_mqtt.c` | MQTT metrics and counters | Wi-Fi task on core 1; Health copies on core 0 | **converted in Phase 4**: `metrics_lock` guards `inc()` and the `robot_network_metrics()` snapshot, which releases the lock before calling `log_stream_dump_active()`, `cyw43_utk_get_status()` or `cyw43_utk_timing_read()`, so no lock is nested inside another |
| `subsystem1/telemetry.c` | stats, busy, loss counters, `log_generation` | Telemetry task on core 1 and core-0 readers | **replaced in Phase 3** - rows/events/reset use `log_write_seq`, the multi-writer `lost`/`busy` use `stats_lock`; no `DI()`/`EI()` remains in the file |
| `subsystem2/motion.c:96,157` | encoder counts, GP20 copy buffer | motion task and encoder IRQ, both core 0 | **keep** - the IMU reads the published Motion board, not the counts |
| `subsystem3/drv_line.c:59-66` | line ring buffer | 1 ms ADC IRQ and line task, both core 0 | keep - core 0 by placement |
| `subsystem5/drv_ultrasonic.c:44` | listener id | echo IRQ and sonar task, both core 0 | keep - core 0 by placement |

The encoder note is a decision, not an oversight: the counts stay owned by the
core-0 IRQ and the Motion task, so their local `DI()`/`EI()` needs no conversion
merely because a core-1 task later reads a published board.

## Handler-versus-task stress test

`app_program/platform/app_lock_stress.c` builds only with `make TEST=1` and runs
from `app_main.c` with the other on-target tests.

It creates a cyclic handler at **one kernel tick** that runs in handler context and
calls `ISpinTryLock()` on a lock the test task also takes. `CNF_TIMER_PERIOD` in
`config/config.h` is 10, so `cyctim = 1` is a 10 ms period, not 1 ms; an earlier
revision of this record said 1 ms and was wrong. The task takes the lock with
`ISpinLock()`, records that it is held, spins briefly, and releases.

The pass condition is that the handler attempts the lock at least
`LOCK_STRESS_MIN_TRIES` (50) times, **never** observes it held, and the run
completes. Requiring a single attempt would have proved only that the handler was
created, not that the overlap condition was exercised. The interval is
deliberately NOT lengthened to force an overlap: the claim is that the handler
cannot run while the lock is held, not that it is starved. With `ISpinLock` the
task has interrupts masked, so the handler cannot preempt it while the lock is
held.

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
the last mark on contention. Every access to the mark is an atomic load or store,
because another core's scan can update it while this one reads; plain reads had no
cross-core ordering even though the aligned word could not tear, and could miss a
LOW MARGIN report. The update is monotonic: a smaller value is never stored, so a
mark can lag but never regress.

Scanning a core-1 task's stack from core 0 is a sampled read. It cannot establish
an exact instantaneous free-stack value, so the mark is approximate and monotonic
rather than exact, and later stack acceptance reports must keep that limit.

## Phase 3: Telemetry on core 1

Two contracts, deliberately different, because the fields have different writer
counts.

### The log payload: a write sequence

`log_write_seq` is a second word, not a reuse of `log_generation`.
`log_generation` stays the capture-run identity that MQTT live and DUMP compare
against. The sequence marks a row, event or reset mutation in progress - odd
during, even after - with `__atomic_*` and `__ATOMIC_SEQ_CST`, and it sits outside
the `TM_MQTT` guard because the writer and its readers exist in every profile
while the run identity is MQTT-side.

Readers now verify stability **after** the copy rather than before it. Each of
`telemetry_log_row()` and `telemetry_log_event()` samples the sequence, rejects an
odd value, copies into a local scratch object, re-reads the sequence and rejects a
change, then re-opens the index and requires the same run identity. Only then is
scratch committed to the caller's buffer. Otherwise the read fails with `E_TMOUT`
after three attempts: a bounded failure, never a row that may not be stable. That
covers overwrite during a new run, and event reads as well as row reads.

### The stats counters: a lock, and why not the sequence

`stats.lost` is incremented by submitters on core 0 and by the task on core 1, and
`stats.busy` is set by a submitter and cleared by the dump task. A sequence cannot
serve those fields: it assumes a **single** writer, so two concurrent odd/even
pairs can leave the sequence even while a mutation is still in flight. This was
found while doing the move, and it is the reason the stats group did not simply
join the sequence. `stats_lock` - a `T_SPLOCK` taken with plain `SpinLock`,
because these are task-only - guards them on the submit path, on the dump path,
and for the `telemetry_stats()` group copy. The struct holds 32-bit diagnostic
counters that never gate control, so a short spin is acceptable; routing every
submitter through the task instead would have changed the message interface.

The task moved with `app_prc_core1()` and `SLOT("telemetry_task", ...,
APP_CORE_1)`. It has no cyclic handler, so there is no handler affinity to match.

### log_stream

`log_stream.c`'s two `DI()`/`EI()` sites protect its own `s_dump` live/dump
progress, not the RAM rows, and the only caller of `log_stream_dump_request()` is
`robot_mqtt.c:300` - the Wi-Fi task, still on core 0. They are **kept** rather than
converted, and become Phase 4's problem when the Wi-Fi task moves.

## Phase 4: Wi-Fi/MQTT on core 1

### The audit retired the risk I raised

RT-0043 worried about CYW43 async-context core ownership. It does not apply to this
port: there is no `async_context` or `cyw43_arch` use in `lib/libwifi`, and
`cyw43_utk_compat.c:196-205` supplies `irq_add_shared_handler`,
`irq_remove_handler` and `irq_set_enabled` as weak stubs, so the SDK IRQ path is
overridden rather than used. The radio is served by one polling task, and PIO and
DMA are core-independent hardware on RP2040. Nothing in the driver requires core 0;
what it requires is one permanent owner.

### What actually had to change

- `cyw43_utk_kernel.c`: `timing_lock` replaces every `DI()`/`EI()` over the timing
  and window state - `poll_begin`, `poll_end`, `timing_read`, `window_begin`,
  `window_end` and `start_status`. The begin/end window is a shared state machine,
  not a snapshot, so a lock is the right primitive and a seqlock would be wrong.
- `cyw43_utk_start()` now requests core 1 (`TP_PRC1 + 1u`) under SMP. It creates the
  task with `tk_cre_tsk` directly, so the Phase 1 slot table does not reach it; the
  placement decision and its reasoning sit at that call.
- `robot_mqtt.c`: `metrics_lock` guards `inc()` and the `robot_network_metrics()`
  snapshot, released before `log_stream_dump_active()`, `cyw43_utk_get_status()` and
  `cyw43_utk_timing_read()`. The previous code masked interrupts across all three,
  which would have nested a lock inside a mask.
- `network_queue.c`: `queue_lock` guards `lost()`, the board-result recorder and
  `robot_queue_metrics()`.
- `log_stream.c`: `dump_lock` guards the DUMP request and its consumption, because
  Vehicle requests from core 0 and the Wi-Fi task consumes on core 1.

Every lock added here is a plain `SpinLock`, which is the correct variant under the
lock rule because all of these callers are task context.

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
6. Remaining boards still on one core until their phases land: `line_service.c`,
   `robot_health.c`, `junction_source.c` and the five boards in `vehicle_task.c`.
   The **Motion board is already cross-core and is not in that list**:
   `imu_service.c:101` calls `motion_snapshot_try_read()` from core 1 while Motion
   publishes from core 0. It uses the same `app_board` sequence contract, so no
   second writer is introduced, but it needs an explicit cross-core audit rather
   than an assumption.
7. Phase 3 has not run on a device. Nothing shows a core-1 Telemetry task
   receiving messages, rows and events passing the sequence check under load, the
   bounded `E_TMOUT` path ever being taken, or `stats_lock` contention staying
   short. The rejected-torn-read claim is an argument from the code, not an
   observed result.
8. Phase 4 has not run on a device. Nothing shows the radio task actually executing
   on core 1, MQTT traffic surviving the move, the poll-path spinlock staying cheap
   under load, or `radio_status.owner_prc` reporting core 1 as it should.
9. The Wi-Fi task sits outside the registry: `cyw43_utk_start()` builds it with
   `tk_cre_tsk`, so it has no slot and no stack high-water measurement. Its 6-8 KB
   stack is therefore untested against the 256-byte free minimum.
