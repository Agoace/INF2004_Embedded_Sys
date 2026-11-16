# UP SMP utility shim — review M7

## Scope

Base: `7ecfd9d`. This repair changes the UP implementation in
[`smp_single_core.c`](../../app_program/platform/smp_single_core.c).
The contract is the public [SMP header](../../include/tk/smp.h), the
[kernel utility implementation](../../kernel/tkernel/smp_lock.c), and the
[RP2040 atomic implementation](../../kernel/sysdepend/cpu/rp2040/smp_atomic.c).
No kernel or template file changes are required.

UP means one active core. PRIMASK is the local interrupt mask. DMB is the ARM
data memory barrier. RMW means read a word, compute a result, and write it back.

The previous shim ignored lock words. A try could succeed on a held lock;
plain unlock enabled interrupts without preserving the caller's state. Four
arithmetic operations read their return value again after restoring interrupts,
so an ISR could replace the result. The explicit memory barrier was only a
compiler barrier, unlike the real kernel DMB.

## All 16 public functions

| Function group | Contract now supplied by the real UP shim |
| --- | --- |
| `InitSpinLock` | Store zero with DMB before and after; do not change PRIMASK. |
| `SpinLock`, `SpinTryLock` | Claim an aligned zero word as one inside a short local IRQ mask. Restore the incoming mask after the attempt. A held try returns `E_BUSY`; blocking acquire retries with the original mask restored. |
| `SpinUnlock` | Release the word before returning, with DMB on both sides; do not change PRIMASK. |
| `ISpinLock`, `ISpinTryLock` | Save and disable IRQs before claiming. Success leaves IRQs disabled. A failed try restores PRIMASK and leaves the caller's output token unchanged. |
| `ISpinUnlock` | Release the word, then restore the exact saved PRIMASK. |
| `atomic_inc`, `atomic_dec`, `atomic_add`, `atomic_sub` | One IRQ-masked RMW, with DMB before and after. Capture this operation's new value before restoring IRQs. Arithmetic is unsigned modulo 2^32. |
| `atomic_xchg`, `atomic_cmpxchg` | Return the old value captured inside the section. CAS arguments remain address, replacement, compare; a failed comparison does not store. |
| `atomic_bitset`, `atomic_bitclr` | Return the old value, not the updated value; perform the bit update inside the section. |
| `mp_memory_barrir` | Execute a real DMB. Retain the public API spelling. |

The seven lock functions retain the kernel's `E_PAR` checks. Atomic operands
remain subject to the kernel's valid, aligned 32-bit address precondition; this
repair does not add a different error API for invalid atomic addresses.

### Holding a lock and masking IRQs are different

Plain locks remain preemptible after acquisition, as in the real SMP kernel.
They must not be shared by same-core holders that can preempt each other.
Use the interrupt-preserving API for that case. B1 already removed all plain
lock callers from the application and Wi-Fi service; this repair does not
reintroduce any. A recursive blocking lock can still deadlock, as it can in the
real kernel; a recursive try now correctly fails.

The UP claim masks IRQs only around a read/conditional store of the existing
lock word. This is valid only because `!TK_SUPPORT_SMP` excludes a second
processor. It is not a cross-core exclusion mechanism and must not be copied
into a two-core packed-field writer.

### Section content and bounds

| Path | Maximum protected content |
| --- | --- |
| Plain claim / one try | Two DMB instructions, one word read, conditional one word store. |
| Interrupt-preserving claim | The same attempt, with IRQs disabled until unlock. A blocking wait is not time-bounded. |
| Release / initialization | Two DMB instructions and one aligned word store; no RMW and no mask change. |
| Arithmetic / exchange / bit update | Two DMB instructions, one word read, one word store and fixed arithmetic. CAS has one conditional store. |

No section calls a kernel wait, console output, allocation or network operation.
Blocking lock acquisition may spin; its latency is not measured or guaranteed.
The local mask covers the RMW, not the caller's work with a plain lock held.

## Tests first: actual source, not the M1 API model

[`smp_single_core_arch.h`](../../app_program/platform/smp_single_core_arch.h)
separates only PRIMASK access, IRQ disable/restore, DMB and spin relaxation.
Production uses the RP2040 instructions. A host build can replace those
instructions with test hooks; the public functions are not replaced, extracted
or copied. ARM builds reject the host-hook option.

[`shim_checks.cmake`](../../tests/shim_checks.cmake) compiles the actual
`smp_single_core.c` and [`test_smp_single_core.c`](../../tests/test_smp_single_core.c).
The target explicitly removes inherited link libraries: it does **not** link
`host_smp_double`. Host headers supply the existing 32-bit types and prototypes;
ARM checks use the real kernel headers. The same target is available from the
full host suite and from the independent `tests/core_migration` build.

Before changing API logic, the old implementation was built with only the
instruction seam added. Result: **16 failures and 3 passes out of 19 cases**.
The four restore-time ISR tests returned the ISR's replacement value instead
of the operation's result. Existing correct old-value, wrap and null-argument
cases passed. After the repair, **19/19 pass**.

Coverage includes lock words and held tries; both initial IRQ masks; unchanged
failed-try tokens; nested distinct locks and an atomic inside them; release
before IRQ restoration; explicit barrier calls; all atomic return conventions;
CAS success/failure; modulo wrap; and invalid lock arguments. A restore hook
changes the operand to 999 at the first IRQ-enable opportunity. Each arithmetic
call must return its earlier captured result, even though memory now holds 999.

The wait test supplies deterministic progress at the relaxation hook. It checks
retry and IRQ-state behavior, not a legal UP task schedule or target wait time.
No test deliberately hangs the Pico on a self-held blocking lock.

Retained local red artifacts are under `build/review/`:

- `m7-red-shim.c` and `m7-red-arch-only.patch`: old logic with the instruction seam.
- `m7-red-tests.log`: 16 failing / 3 passing tests.
- `m7-green-isolated-tests.log`: the independent suite, now 23/23 passing.
- `m7-real-shim-link.txt`: real-source link inputs, without the M1 API library.

Run either test entry point:

```sh
cmake -S tests -B build/host
make -C build/host -j4 --output-sync=target
ctest --test-dir build/host --output-on-failure

cmake -S tests/core_migration -B build/core-blockers
make -C build/core-blockers -j4 --output-sync=target
ctest --test-dir build/core-blockers --output-on-failure
```

## Software proof and RAM

- Full strict GCC suite: **321/321**, versus 302 tests at the base; 19 added.
- Full Clang ASan/UBSan suite: **321/321**.
- Independent blocker suite: **23/23** under GCC and Clang ASan/UBSan,
  including the 19 real-shim cases. The original four tests are unchanged.
- All 24 public-example firmware profiles pass without compiler warnings:
  UP, split SMP and forced-core0 SMP, each with USB, TEST, BENCH, Wi-Fi only,
  lwIP, MQTT, MQTT TEST and MQTT BENCH.
- Nine strict ARM checks pass: all three placements with USB, MQTT and MQTT
  TEST, using `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`.
- UP ARM disassembly shows DMB, exact PRIMASK restoration, and no operand reload
  after PRIMASK restoration in any of the four arithmetic return paths.
- Symbol inspection finds all 16 definitions in the UP object and none in the
  SMP shim object. SMP gets its 16 definitions from the unchanged kernel.
  Firmware symbol checks find no duplicate API provider or host-hook import.

RAM comparison uses a separate detached checkout of exact base `7ecfd9d`, the
same public example settings and toolchain, and GNU map attribution including
alignment. These are MQTT, non-TEST, non-BENCH images:

| Placement | GNU text before → after | `.data` | `.bss` | Static RAM before → after |
| --- | --- | --- | --- | --- |
| UP | 412232 → 412328 | 968 | 186308 | 187276 → 187276 B |
| Split SMP | 419952 → 419952 | 1008 | 191220 | 192228 → 192228 B |
| Forced-core0 SMP | 419824 → 419824 | 1008 | 191220 | 192228 → 192228 B |

M7 costs **96 bytes of UP flash and zero static RAM** in these images. It adds
no static lock storage, task, queue or heap allocation. The existing caller lock
words now carry their defined state. SMP code and RAM sizes do not change.

Reports, maps, assembly and logs remain under `build/review/`:
`m7-{base,current}-{up,split,control}-ram.json`, corresponding size reports,
`m7-matrix.log`, `m7-strict-arm.log`, `m7-provider-proof.log`, and
`m7-strict-up-mqtt.asm`. No settings secrets are part of this record.

## Acceptance limit

No hardware run, flash or push is claimed. Host instruction hooks prove C API
behavior, not RP2040 timing. ARM assembly proves the selected instructions,
not a measured interrupt-off duration. Target lock waits, ISR latency and stack
margins remain hardware acceptance items.
