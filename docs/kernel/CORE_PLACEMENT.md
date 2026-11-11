# Core placement and lock discipline

Status: current record for the MTK3 core migration on the `mtk3-core-migration`
branch. It describes the migration as built - placement, lock audit, control profile
and tests - not as measured. No device result is claimed anywhere in it: every
dynamic figure is listed as open at the end.

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

1. A lock held by a task on one core and by a handler on that same core uses
   `ISpinLock` / `ISpinUnlock`. Never plain `SpinLock`.
2. The same applies whenever two holders on ONE core can preempt each other: two
   tasks of different priority on that core, or a task and a handler. A plain
   `SpinLock` is correct only when every holder sits on a different core from every
   other holder, or when all holders are the same task. Reachability from a handler
   is therefore not the test - preemption within a core is.
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
| `platform/app_task.c:209,214` | stack high-water marks | caller of `app_task_stacks()` (Health, core 0; Telemetry on core 1) | **converted in Phase 2**: `stacks_lock` with `ISpinTryLock` and a bounded fallback. A core-1 task's stack really is measured now, so the mark is a sampled read and the record says so |
| `platform/drv_gpio_irq.c:47-54` | GPIO IRQ client table | init, core 0 | keep - boot-time, one core |
| `platform/idle_meter.c:45,51` | core-0 CPU meter | idle task and meter reader, both core 0 | keep for core 0; Phase 5 **added** the separate core-1 meter and moved sampling to a single Health-owned publication (RT-0069/C2) |
| `subsystem1/log_stream.c:571,587` | DUMP admission state (`s_admit`: request id, IDLE/PENDING/ACTIVE) | requested from Vehicle on core 0 (`vehicle_task.c:404`), consumed by the Wi-Fi task on core 1 | **converted in Phase 4, repaired in RT-0069**: `log_stream_dump_request()` takes one `ISpinTryLock` and returns `E_TMOUT` rather than waiting; `dump_active()` and the two Wi-Fi holders use `ISpinLock`. The transfer stage, cursors and acknowledgements stay Wi-Fi-owned and are never read from core 0 |
| `subsystem1/network_queue.c` | queue loss and busy counters, unavailable mask | Wi-Fi task on core 1; Health copies on core 0; Vehicle counts a loss on the ESTOP route | **no lock remains (RT-0071/F3)**: every counter is updated with the kernel atomic RMW, so the ESTOP route cannot wait at all. The unavailable mask keeps a single writer |
| `subsystem1/robot_mqtt.c` | MQTT metrics and counters | Wi-Fi task on core 1; Health copies on core 0 | **converted in Phase 4, repaired in RT-0069**: every writer goes through a locked setter and the `robot_network_metrics()` snapshot releases the lock before calling `log_stream_dump_active()`, `cyw43_utk_get_status()` or `cyw43_utk_timing_read()`, so no lock is nested inside another |
| `subsystem1/telemetry.c` | RAM-log payload and index; control state separately | Telemetry task on core 1; Wi-Fi readers on core 1; core-0 decision readers use atomics only | **replaced in Phase 3, repaired in RT-0056/RT-0063/RT-0071**: `stats_lock` covers the payload copies and index publication only - readers take one immediate `ISpinTryLock` and return `E_TMOUT`, holders use `ISpinLock`. The control words (`busy`, `storage_full`, `lost`) are separate atomics, and `telemetry_send()`/`telemetry_stats()`/`lose()` take no lock at all |
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

One contract, because review showed two mechanisms could not both be authoritative.

### The log: one lock, and why not a write sequence

An earlier revision used `log_write_seq`, an odd/even sequence beside
`log_generation`, with readers retrying up to three times. It was removed in
RT-0056 and the removal is final. A sequence can reject a copy that changed
underneath it, but it cannot make a byte-wise copy of `rows[]` or a struct copy of
`events[]` atomic, and it left `publish_index()` outside the mutation interval. A
second mechanism that no longer carries the contract would only invite doubt about
which one does.

`stats_lock` is now the single mechanism **for the payload**: it covers the row and
event copies and the index publication, so a reader never sees a payload that has
moved while the index still points at the old place. It is deliberately *not* the
mechanism for control state - see the next subsection, where `busy`, `storage_full`
and `lost` are atomics because Vehicle decides on them.

Two variants, chosen by role:

- **Readers** - `telemetry_log_row()` and `telemetry_log_event()`, called by the
  Wi-Fi task on the same core as the Telemetry writer - take ONE immediate
  `ISpinTryLock` and return `E_TMOUT` when it is busy. No retry loop is allowed
  inside that call, because Telemetry cannot resume on that core until the Wi-Fi
  task yields (RT-0063).
- **Writers and the core-0 submitter** use `ISpinLock`/`ISpinUnlock`, which calls
  `disint` before acquisition, so the holder cannot be preempted for its section.
  Sections stay short and hold no `tk_*` wait, printf or network call.

`log_generation` remains the capture-run identity that MQTT live and DUMP compare
against. It is a value, not a synchronisation device.

### Control state and diagnostics are different contracts

RT-0071/F1 settled this: `busy`, `storage_full` and `lost` **do** gate control -
Vehicle decides DUMP admission and the storage-full stop on them - so they cannot
be a best-effort snapshot, and they cannot sit behind the payload lock.

`telemetry_send()` takes **no lock at all**. A DUMP claims its reservation with one
`atomic_cmpxchg` on `admit_busy`, so a second DUMP is refused rather than admitted
beside the first, and it releases with `atomic_xchg` only when it actually
reserved, so a failed send cannot clear another DUMP's reservation or leave `busy`
false while a DUMP is pending. `TMO_POL` is unchanged, and only a real
message-buffer failure returns an error, which is the one case that may stop a run.
`m.loss_base` is an aligned read of `admit_lost`.

`telemetry_stats()` reads no locked state. The control fields come from the three
atomics (`admit_busy`, `admit_full`, `admit_lost`) and are exact. The diagnostic
counters (`rows`, `events`) come from `stats_pub`, a snapshot the Telemetry task
publishes under `stats_lock` after every mutation.

`stats_lock` therefore guards only the payload and that snapshot. Its holders use
`ISpinLock`; the two log readers take one immediate `ISpinTryLock`. The earlier
design - a try-lock with an unlocked fallback copy - was wrong because it applied a
diagnostic contract to control state.

The task moved with `app_prc_core1()` and `SLOT("telemetry_task", ...,
APP_CORE_1)`. It has no cyclic handler, so there is no handler affinity to match.

### log_stream

`log_stream.c`'s two `DI()`/`EI()` sites protect its own `s_dump` live/dump
progress, not the RAM rows, and the only caller of `log_stream_dump_request()` is
`robot_mqtt.c:300` - the Wi-Fi task, still on core 0. They are **kept** rather than
converted, and become Phase 4's problem when the Wi-Fi task moves.

## Phase 4: Wi-Fi/MQTT on core 1

**Ownership, recorded so it is not re-derived.** One task owns the radio: the
CYW43439 PIO/DMA and pin state, the poll loop, and the DUMP transfer stage (stage,
frame cursors, acknowledgements) - the Wi-Fi task, on core 1. Vehicle owns the DUMP
*request* and nothing else of it: admission only, one word of state. Health owns the
reads of radio status, timing, queue counters and MQTT metrics, and never reads the
transfer stage. USB service and its interrupt stay on core 0.

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
- `network_queue.c`: **no lock at all**. Every counter is atomic (RT-0071/F3), so
  the ESTOP loss path cannot be made to wait.
- `log_stream.c`: `dump_lock` guards the DUMP admission state, because Vehicle
  requests from core 0 and the Wi-Fi task consumes on core 1.

"Task context only" was NOT sufficient. That is the lesson of RT-0061 and
RT-0069: a plain `SpinLock` deadlocks when a more urgent task on the same core
holds it and is then preempted. Every lock with same-core holders now uses
`ISpinLock`, and a caller that must not wait uses `ISpinTryLock` or an atomic.

## Phase 5: per-core load and stack margin

Health stays on core 0, as decided, and the heartbeat's `cpu_load_permille` keeps
its original core-0 meaning. Nothing in this phase introduces a dual-core total or
changes the wire format.

- **Core-1 load.** `idle_meter.c` now holds two meters, one per core.
  `idle_meter_start()` and `idle_meter_load_permille()` keep their previous
  behaviour exactly for core 0; `idle_meter_start_core1()` and
  `idle_meter_load_permille_core1()` add the core-1 pair and exist only under
  `TK_SUPPORT_SMP`. The guard matters: in a single-core build the second counting
  task would land on core 0 and corrupt the very figure it is meant to complement.
- **Shared calibration, stated exactly.** `idle_meter_calibrate()` runs ONCE, from
  `app_main.c:64` during usermain, as a single 4 ms window with interrupts disabled.
  There is no second calibration anywhere in the tree. The RP2040's two cores run
  the same clock, so that one measurement supplies the peak rate for both meters. A
  per-core clock difference would appear as a constant load offset, which is one
  reason these figures are diagnostics rather than control inputs.
- **Reading it.** The getters are pure reads of a published snapshot written once
  per Health sample (RT-0069/C2). `idle_meter_sample_all()` advances both meters and
  is called only by Health at 1 Hz; `idle_meter_load_permille()` and
  `idle_meter_load_permille_core1()` return the published word and advance nothing,
  so a diagnostic on either core cannot race the sampling state or reset the Health
  interval. Each value is an aligned 16-bit word, so no figure can tear, though the
  pair may come from different samples when read mid-update. A fully busy core reads
  1000 permille; 65535 means no calibration, the first sample, or an interval too
  short to interpret.
- **Stack margin per core.** `app_stack_t` gained a `core` field, filled from the
  placement table, and `app_task_report()` prints it, so a core-1 task's margin is
  reported beside its name instead of inferred. The core-1 meter has its own
  registry slot (`idle_meter_1`, `APP_CORE_1`), so its stack is measured like
  every other task's rather than assumed.
- **The Wi-Fi task remains outside the registry** (open item 9). Phase 5 does not
  close that: `cyw43_utk_start()` still creates it with `tk_cre_tsk`, so it has no
  slot, no high-water mark, and no row in the per-core stack report.

## Open validation items

None of these is closed, and none may be reported as a PASS without device
evidence.

1. The stress test has not been run. It needs `make TEST=1` on a Pico W with the
   radio off, and a console capture.
2. Core-1 tasks exist and both placement profiles build, but the on-target affinity
   audit has not been run: no capture yet confirms which processor each task and
   cyclic handler actually reported. That evidence is Phase 6.
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
   receiving messages, rows and events surviving the lock contract under load, the
   bounded `E_TMOUT` reader path ever being taken, or `stats_lock` contention
   staying short. The coherence claim is an argument from the code, not an observed
   result.
8. Phase 4 has not run on a device. Nothing shows the radio task actually executing
   on core 1, MQTT traffic surviving the move, the poll-path spinlock staying cheap
   under load, or `radio_status.owner_prc` reporting core 1 as it should.
9. The Wi-Fi task sits outside the registry: `cyw43_utk_start()` builds it with
   `tk_cre_tsk`, so it has no slot and no stack high-water measurement. Its 6-8 KB
   stack is therefore untested against the 256-byte free minimum.
10. Phase 5 has not run on a device. No figure exists for core-1 load, no
   core-0 against core-1 comparison has been made, and no core-1 task's stack
   margin has actually been read from the per-core report. Unverified with it: that
   a fully busy core reports 1000 permille rather than 65535, that the published
   idle count advances between Health samples, and that a 100 ms count pass leaves
   the meter accurate.

## Lock audit by lock (RT-0069)

### Inventory — verified

`grep -rn "static T_SPLOCK" app_program lib/libwifi --include=*.c` finds nine locks.
Holder counts are acquisition sites per lock.

| Lock | Declaration | Acquisitions | Reached from |
| --- | --- | --- | --- |
| `stats_lock` | `subsystem1/telemetry.c:35` | 11 | RAM-log payload, published snapshot |
| `dump_lock` | `subsystem1/log_stream.c:102` | 4 | DUMP admission request, active read, Wi-Fi step |
| `timing_lock` | `libwifi/.../cyw43_utk_kernel.c:22` | 9 | CYW43 poll, timing window, start status |
| `metrics_lock` | `subsystem1/robot_mqtt.c:69` | 11 | MQTT writers, Health snapshot |
| `status_lock` | `libwifi/.../cyw43_utk.c:82` | 2 | radio snapshot publish and copy |
| `stacks_lock` | `platform/app_task.c:208` | 1 | stack scan (Health, Telemetry) |
| `claim_lock` | `libwifi/.../cyw43_utk_compat.c:37` | — | Wi-Fi task only, not yet audited |
| `stress_lock` | `platform/app_lock_stress.c:42` | — | test only |

### Task, core and priority map used by the audit

| Core 0 | Priority | Core 1 | Priority |
| --- | --- | --- | --- |
| Motion | 10 | IMU | 14 |
| Line | 11 | Wi-Fi | 20 |
| Sonar | 12 | Telemetry | 30 |
| Vehicle | 13 | idle_meter_1 | 32 |
| USB | 21 | | |
| Health | 31 | | |
| idle_meter | 32 | | |

Lower number is more urgent. A task on the left can preempt another task on the left with a higher number. Cross-core pairs cannot preempt each other; they only contend.

### Unsafe pairs — reasoned from the map, not yet verified per site

A plain `SpinLock` is unsafe when two holders sit on the SAME core and the more urgent one can preempt the other. From the map:

- `dump_lock`: Vehicle 13 and Health 31, both core 0. **Was unsafe; repaired in RT-0069** - Vehicle's request fails fast and Health plus the two Wi-Fi holders are interrupt-preserving.
- `queue_lock`: **removed in RT-0071/F3.** The counters are atomic, so there is no lock here to be unsafe.
- `timing_lock`: Vehicle 13 and Health 31, both core 0. **Was unsafe; repaired in RT-0069 and RT-0071** - Vehicle's window calls fail fast and a missed close is applied under the lock.
- `stats_lock`: already interrupt-preserving on every holder after `26d306d`.
- `metrics_lock`: Wi-Fi 20 on core 1 writes; Health 31 on core 0 reads. Different cores, so contention is bounded by section length. Not a same-core pair.
- `status_lock`: Wi-Fi 20 on core 1; readers on core 0. Different cores.
- `stacks_lock`: Health 31 on core 0 and Telemetry 30 on core 1. Different cores.

### Per-holder rows — verified

Variant abbreviations: `ISL` = `ISpinLock`/`ISpinUnlock` (masks interrupts, holder not
preemptible), `IST` = `ISpinTryLock` (fails fast), `SL` = plain `SpinLock`.
"Unsafe" means two holders share a core and the more urgent one can preempt the
other, so a plain spin can deadlock. Measured maximum wait is blank everywhere: no
device evidence exists.

#### `stats_lock` — `subsystem1/telemetry.c:35`

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `telemetry_log_row` | 105 | IST | core 1, 20 (Wi-Fi, via `log_stream`) | index read + one row copy | 1 struct + 1 row (64 B) |
| `telemetry_log_event` | 140 | IST | core 1, 20 (Wi-Fi) | index read + one event copy | 1 struct + 1 event |
| `telemetry_task` cases | 314, 328, 347, 377 | ISL | core 1, 30 | one row/event write + `publish_index` + `publish_stats` | 1 row or 1 event + index |

Four callers that used to hold this lock no longer do, and the table is the place
that says so: `telemetry_stats()`, `lose()`, `dump()` and `telemetry_send()` take
no lock at all after RT-0071. `lose_locked()` is called only from inside a task
case, and still takes nothing because the caller holds it. No section contains a
`tk_*` wait, printf or network call.

#### `dump_lock` — `subsystem1/log_stream.c:102` — repaired in RT-0069

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `log_stream_dump_request` | 571 | IST (E_TMOUT) | core 0, 13 (Vehicle) | admission accept | 2 words |
| `log_stream_dump_active` | 587 | ISL | core 0, 31 (Health) | admission read | 1 word |
| `dump_service` | 498 | ISL | core 1, 20 (Wi-Fi) | pending to active | 2 words |
| `dump_apply` | 625 | ISL | core 1, 20 (Wi-Fi) | return to IDLE | 1 word |

#### `queue_lock` — removed in RT-0071/F3

The lock no longer exists. All three holders were counter updates, so they became
the kernel's atomic RMW: `lost()` and `robot_command_lost()` are lock-free, which
is the only way the ESTOP route can meet "no blocking wait in Vehicle". The
board-result recorder uses an atomic increment for the counters it touched, and the
unavailable mask keeps its single writer. The audit therefore lists one lock fewer
than the eight it started with.

#### `timing_lock` — `libwifi/.../cyw43_utk_kernel.c:22` — repaired in RT-0069 and RT-0071

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `cyw43_utk_poll_begin` | 58 | ISL | core 1, 20 (Wi-Fi) | timing add, apply deferred close | 1 struct |
| `cyw43_utk_poll_end` | 70, 84 | ISL | core 1, 20 (Wi-Fi) | timing add, window close | 2 structs |
| `cyw43_utk_timing_read` | 101 | ISL | core 0, 31 (Health) | copy out | 2 structs |
| `cyw43_utk_timing_window_begin` | 110 | IST (E_TMOUT) | core 0, 13 (Vehicle) | open window, apply deferred close | 3 words |
| `cyw43_utk_timing_window_end` | 131 | IST (E_TMOUT) | core 0, 13 (Vehicle) | close window, or leave a close request | 2 structs |
| `cyw43_utk_start_status` | 145 | IST (atomic read) | core 0 readers, core 1 writer | status read | 1 word |
| `cyw43_utk_start` | 198, 205 | ISL | core 1, 20 (boot) | reset state, kernel call outside | 3 words |

#### `metrics_lock` — `subsystem1/robot_mqtt.c:69` — different cores, not a same-core pair

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `inc` | 96 | SL | core 1, 20 (Wi-Fi) | 1 counter | 1 counter |
| nine metric setters | 110-172 | SL | core 1, 20 (Wi-Fi) | 1 field | 1 field |
| `robot_network_metrics` | 348 | SL | core 0, 31 (Health) | snapshot | 1 struct |

#### `status_lock` — `libwifi/.../cyw43_utk.c:82` — different cores

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `cyw43_utk_task` publish | 363 | SL | core 1, 20 (Wi-Fi) | snapshot publish | 1 struct |
| `cyw43_utk_get_status` | 377 | SL | core 0 (Health, Vehicle) | snapshot copy | 1 struct |

#### `stacks_lock` — `platform/app_task.c:208` — different cores

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `app_task_stacks` | 242 | IST | core 0, 31 (Health); core 1, 30 (Telemetry) | one row update | 1 row |

#### `claim_lock` — `libwifi/.../cyw43_utk_compat.c:37`

| Holder | Line | Variant | Core, priority | Held across | Static bound |
| --- | --- | --- | --- | --- | --- |
| `hw_claim_lock` | 44 | ISL | believed core 1 only | SDK claim | 1 word |

### Lock order — flat, no nesting

No lock is held while another is acquired, in any of the nine. That makes order
inversion impossible between these locks, and it is deliberate rather than
accidental:

- `robot_network_metrics()` releases `metrics_lock` before it calls
  `log_stream_dump_active()`, `cyw43_utk_get_status()` and `cyw43_utk_timing_read()`,
  which take `dump_lock`, `status_lock` and `timing_lock`. Holding `metrics_lock`
  across those calls would open exactly the ordering question this table exists to
  close.
- `telemetry.c` no longer nests anything: `lose()` and `telemetry_stats()` are
  atomic-only after RT-0071, and the task cases take `stats_lock` once for the
  payload, the index publication and the snapshot together.
- `dump()` takes no lock; it reads the published counters and prints.
- `app_task_report()` prints the load line outside `app_task_stacks()`, so
  `stacks_lock` is not held during printf.

Two consequences follow, and both are limits rather than guarantees: a future
change that takes a second lock inside a section breaks this property silently, and
nothing in the code enforces it. Any new lock must be added to this table with its
holders before it is used.

### Still open in this audit

- `claim_lock` reachability is believed Wi-Fi-task-only, from the SDK hardware-claim
  path. That is an assumption; a call-site grep of the SDK is not done.
- `lost()` in `network_queue.c` has callers on both cores. Each caller needs its core
  attributed before the lock variant can be chosen.
- Measured maximum wait per lock. Blank by design.
