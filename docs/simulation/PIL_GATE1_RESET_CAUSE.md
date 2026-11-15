# Gate 1 — early reset-cause boot record

## Approved scope

This is the historical optionA checkpoint. The current image adds separately
approved [retained diagnostic B](PIL_GATE1_BREADCRUMBS.md). RESET1 stays frozen.

Attempt14 lost Pico USB, then re-enumerated. Its last observed OUTPUT preceded
USB shutdown by91.832ms, but Probe capture closed about5.3ms after that OUTPUT:
about86ms were unobserved. This does not prove watchdog starvation, a HardFault
or even distinguish CPU reset from a USB-only failure. The initial boot banner
was captured; no post-event reset cause was available.

For this checkpoint the operator approved optionA ONLY: a Gate1 early raw reset-register snapshot
and one versioned USB boot record. No persistent feed/fault breadcrumbs
(optionB), watchdog change, new watchdog feed, native application/template edit,
UART text or D1/RX1 schema change is supplied.

## Snapshot and ownership

[pil_gate1_task.c](../../app_program/sim/pil_gate1_task.c) calls
`pil_gate1_reset_snapshot` FIRST in `__wrap_usermain`, before time/identity
initialization, native application startup, watchdog cause capture and watchdog
start. The SDK-only read function in
[pil_gate1_uart_hw.c](../../app_program/sim/pil_gate1_uart_hw.c) copies40 bytes:
WATCHDOG.REASON, VREG_AND_CHIP_RESET.CHIP_RESET and all eight WATCHDOG scratch
words. It does not write/clear a register, call a watchdog API or claim scratch
ownership. ROM/kernel boot already occurred; this is not a pre-ROM snapshot.

The existing native driver still caches its reset category before SDK watchdog
start replaces scratch4. Its TEST_MAGIC handling can clear scratch0. The new
record retains the earlier raw values without changing either behavior. It is
ordinary per-boot RAM, not persistent history across another reset. The next
successful boot takes its own fresh snapshot. OptionA alone retains no RAM crash frame; current B storage is separate.

[pil_gate1_reset.h](../../app_program/sim/pil_gate1_reset.h) defines the small
record and122-byte format. Formatting in
[pil_gate1_dump.c](../../app_program/sim/pil_gate1_dump.c) runs in startup task
context after nonce generation, before native startup. Session is explicitly0.
The formatted boot identity remains immutable if a later HELLO starts a session.

## RESET1 USB schema

```text
[PILG1RESET1 nonce session reason chip_reset scratch0 scratch1 scratch2 scratch3 scratch4 scratch5 scratch6 scratch7]\n
```

- Exactly122 bytes: fixed prefix,12 eight-digit lowercase hexadecimal words,
  single spaces, closing `]` and literal LF. No NUL, CRC, END row or D1 envelope.
  The `]\n` terminator completes the record. Direct `tm_snd_dat` does not apply
  stdio newline conversion. No CRLF is emitted by this firmware.
- `nonce` matches this boot's UART BOOT nonce; `session` is `00000000`, not a
  stopped D1 session. Remaining words are unmodified raw32-bit registers.
- One task-context console append after CDC state3 and pending bytes<=512.
  Until then, service retries the readiness check without waiting. In this UP
  image, IRQ exclusion covers admission and the single append into the4096-byte
  USB ring;122 bytes fit without another task filling it between the operations.
- Once appended, the sent flag prevents duplicates, including after USB reconnect
  without CPU reset. There is no delivery ACK or reprint command. A later USB
  failure can lose queued data; partial/missing records mean UNKNOWN, not proof
  of no reset. Startup/task/USB failure can prevent reporting altogether.
- Only the restricted UP image is supplied. Three-placement compilations and
  single-thread doubles are not a cross-core console-admission proof.
- The compile guard rejects a UART console mirror. UART v1 stays binary. D1
  binary, stopped RX1 and the stack line stay unchanged; CDC now has an extra
  boot row that its reader must retain and label separately.

## Interpretation limits

WATCHDOG.REASON timer/force bits identify the watchdog reset family. Raw scratch4
can support the SDK enable-magic distinction at the early capture point, but
neither it nor a watchdog flag identifies starvation versus HardFault, deadlock
or stack corruption. The current HardFault reporter can loop until the armed
watchdog expires, and UP USB is not a reliable fault-context console.

CHIP_RESET HAD_POR is the power-on OR brown-out family; HAD_RUN and
HAD_PSM_RESTART provide separate raw reset-family evidence. Do not claim that
HAD_POR distinguishes a fresh power application from a brown-out. Preserve
unknown bits and the raw row. These fields are not the D1 session stop reason.
No source-age, clock, physical sensor or transport PASS follows from this row.

## RED and validation

- Runtime RED first: old code failed the assertion that the snapshot was read
  before `__real_usermain` (`build/pil-gate1/reset-red.log`).
- New UP/split/control reset cases check early ordering, CDC unavailable/busy
  deferral, exact boot identity/line and no duplicate append. Hardware register
  doubles check reason0/1/2/3, all eight scratch words, no register writes and
  preservation after a later simulated SDK scratch4 overwrite. Formatter tests
  check lowercase/full-width values, literal LF and destination canaries.
- GCC601/601 and Clang ASan/UBSan601/601 pass;45 strict ARM units pass.
  Logs: `build/pil-gate1/reset-{gcc,clang}-{build,tests}.log`,
  `reset-strict.log`.38 ordinary builds/SDK proofs and all76 literal ELF/UF2
  hashes equal baseline451e562: `build/pil-proof/gate1-reset/hashes.txt`.
- Added storage14960B: g12908 + worker2048 + timestamp4, an increase of164B.
  Whole UP image: data696, bss140236, text123716B. BSS increased160B because
  existing alignment absorbs4B. Storage remains below16KiB.
- Frames: snapshot0B, formatter48B, startup wrapper56B. UART80/native40/lease16,
  service440/worker32 remain unchanged. Reviewed additional IRQ204B UP/248B SMP
  and worker896B remain unchanged. Startup formatter/wrapper and the new USB
  append branch are smaller than that worker maximum. No new target painted
  reserve or IRQ latency measurement is claimed.
- Actual C fixtures: `reset-boot-line.txt`122B, `reset-runtime-usb.bin`5602B,
  `reset-missing-usb.bin`2393B under `build/pil-gate1/`. Removing the122-byte boot
  prefix leaves byte-identical D1/RX1/stack data versus the IRQ-wake checkpoint.

## Exact image and operator handoff

File: `build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2`,248832B.
UF2 SHA256: `a0a53f1a8ec419d433542ad8d0116f64f42a0ca6ca555ab0e6d551405d2c3da9`.
ELF SHA256: `fafd09ddfad2d44d9eb3b38c28d904b49bbdf093875c360d8430ea3dfcee1aaa`.
Retained artifacts: `build/pil-gate1/artifacts/reset-cause/`.
Use the [restricted build/operator handoff](PIL_GATE1_BUILD.md). Capture CDC
before ARM, including separate initial/reopened segments and timestamps.

The PC owner separately implements the approved bounded RX-only tail in the
SAME Probe reader after inhibition/DISARM, and CDC reopen capture. No second
UART reader, SENSOR/HELLO/rearm/lease renewal or automatic session continuation.
Keep the original FAIL. Only the operator flashes/resets/opens ports; coding
workers do not. No target access, flash or push occurred in this checkpoint.
