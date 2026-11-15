# Gate1 reset diagnostic B — retained progress and fault records

## Scope and evidence

The operator approved reset diagnostic B after repeated watchdog resets. This
is a Gate1-only diagnostic. It does not change the100ms motion-only watchdog,
priorities, task load, alarm0, fault checks or frozen UART/D1/RX1 protocols.
No coding worker used hardware, opened ports, flashed or pushed this change.
No physical PASS or root cause is claimed.

Attempt17's last complete OUTPUT1002/ACK902 was active, queue2, with all four
transmitted u16 counters zero. SYNC_REPLY57 followed, then26 unverified bytes
of candidate OUTPUT1003. USB shutdown followed95.928ms after the last complete
OUTPUT and95.202ms after the last partial bytes. The same Probe reader retained
a1.000045s tail with six new CRC-valid BOOTs. The new nonce matched reopened
RESET1 reason1. Unlike attempt14, this interval had no reader-close blind gap.
Capture drops were UNKNOWN, not zero. This fits loss of core/IRQ progress, but
cannot distinguish HardFault, long IRQ exclusion, task blocking or corruption.
The four wire counters are not full D1 coverage or all26 counters.

## Storage and startup

Sources: [breadcrumb module](../../app_program/sim/pil_gate1_breadcrumb.c),
[contract](../../app_program/sim/pil_gate1_breadcrumb.h), and
[Gate1 linker fragment](../../app_program/sim/pil_gate1_retained.ld).

The112-byte live array is in KEEP/NOLOAD `.pil_retained`, after `.data` and
before `.bss`. Link ASSERTs check its size, RAM bounds and exclusion from both
initialization ranges and the heap. Current `.noinit` is inside the BSS clearing
range with USE_NOINIT=0. Enabling USE_NOINIT would also expose a clear-length
problem in existing startup. This change does neither; shared startup/linker,
kernel, config and robot source remain unchanged. No watchdog scratch is written.

`__wrap_usermain` still takes RESET1's raw snapshot first. After nonce generation,
B copies the previous live array before any new B write, validates it, formats
an immutable boot row, then clears/reinitializes the live record for this boot.
After normal data/BSS initialization, the active flag is zero: early task/tick/
fault hooks cannot overwrite the old record before this copy. There is no new
pre-C reset hook. A fault before RAM stub copy or BSS initialization is outside
both capture and preservation guarantees. Failures before arming B may leave
no B record.

Previous data is trusted only with watchdog REASON bits0/1, known magic/producer,
nonzero owner nonce and a correct header check. Power/RUN/debug/flash can invalidate
history. Match the previous owner nonce to the failed run's old BOOT/CDC identity
and the exact image before attributing it to that run. A first B boot can contain
no history or unrelated history. Invalid data is UNKNOWN, not zero activity.

## Bounded writers

- The Gate1 link wraps the existing motion `drv_watchdog_feed` call. It marks
  that subrecord odd before calling the original ONCE, then publishes the
  completion-observation time and caller task ID. No other path feeds. A fault
  or interruption before publication leaves UNKNOWN, not a plausible old feed.
  This stamp can follow the actual hardware reload if preempted. Its age is a
  lower bound on actual feed age, not an exact reload time.
- The PIL loop publishes ENTER1 and BEFORE_SLEEP2. Its packed value is task ID
  in bits31..8 and phase in bits7..0; UINT32_MAX means unavailable ID/state.
  Counted wakes can make sleep return immediately. Continued loop stamps do not
  prove CPU ownership or explain why motion did not feed.
- The existing native alarm0 hook publishes its original1ms entry stamp and
  `tk_get_tid()` thread ID. This is not the10ms kernel SysTick. The getter is a
  bounded current-TCB read with no SVC, lock or scheduling action. It assumes
  valid kernel state: corrupt TCB state can fault in this diagnostic read.
  A sampled thread ID is not CPU accounting or necessarily the faulting thread.
- A Gate1 vector link wrapper runs a176-byte naked RAM stub before the existing
  strong ASM HardFault handler. It marks entry first, records time/SP/EXC_RETURN/
  PRIMASK/IPSR/core ID, and reads stacked PC/LR/xPSR only if all32 frame bytes are
  aligned and inside physical SRAM. It uses no stack pushes, C calls, locks,
  USB, scratch or watchdog feed. LR and SP are preserved before delegation.
  Frame-valid means RAM-readable, not proof that the stack was uncorrupted.
  Inaccessible RAM, failed exception stacking or bus lock-up can prevent entry
  or completion. No marker does NOT exclude a fault. The original reporter may
  still loop until watchdog reset; no reliable fault-context USB is assumed.

Each normal subrecord uses aligned u32 stores: odd/in-progress, body/check,
DMB, even commit. Records have separate checks; interruption does not invalidate
unrelated groups. The fault marker1 is useful even without a complete body.
Checks detect partial/ordinary corruption, not all possible corruption.

RAM instructions and literals are copied with `.data`. Its ELF flags remain WA;
RP2040 executes SRAM without ELF page permissions. Explicit FUNC/Thumb typing
is essential: final vector word is `20000169`, not the invalid even address.
No shared exception table source or startup provider is added.

## BC1 companion boot row

RESET1 stays exactly122 bytes. The next separate atomic task append is:

```text
[PILG1BC1 current_nonce valid_mask raw0 raw1 ... raw27]\n
```

Exactly281 bytes:30 lowercase eight-digit hex words, single spaces, `]` and
literal LF; no NUL, CRC, END or D1 envelope. Raw words always preserve the copied
previous array, even when untrusted. The first word is the CURRENT boot nonce;
raw2 is the PREVIOUS owner nonce. Do not subtract previous times from new TIMER.

| Raw index | Value |
| --- | --- |
|0..3|magic50424731, producer/layout00010001, owner nonce, header check|
|4..7|motion sequence, completion-observation time, caller ID, check|
|8..11|PIL sequence, time, packed task ID/phase, check|
|12..15|alarm0 sequence, original entry time, thread ID, check|
|16..27|fault status, time, PC, LR, xPSR, SP, EXC_RETURN, PRIMASK, IPSR, core ID, check, inverse check|

Salt is9e3779b9. Header check is salt XOR raw0 XOR raw1 XOR raw2.
For base4/8/12: sequence must be nonzero/even; check is salt XOR base XOR
sequence XOR time XOR value. Fault check is salt XOR raw16 XOR all raw17..25;
raw27 must be its bitwise inverse. Status0=none,1=entered/incomplete,
2=frame captured,3=frame address rejected (PC/LR/xPSR are ffffffff).
Only2/3 can have complete fault integrity. An incomplete entry's time is UNKNOWN,
even if its raw slot looks plausible. Checks are not cryptographic identity.

Mask bits, recomputed from raw data and the SAME boot's RESET1 reason:

| Bit | Meaning |
| --- | --- |
|0|Trusted header; without it the ENTIRE mask is0|
|1/2/3|Valid motion/PIL/alarm0 publication|
|4|Complete fault record with correct check and inverse|
|5|Entry marker observed: status1/2/3; can be set without bit4|
|6|Readable frame: bit4 and status2|

Bits7..31 are zero. Bit5 alone is an unchecked entry marker: corruption can
mimic it; it is weaker than a complete checked fault record. Invalid subrecords
expose raw words but no trusted fields. Missing/partial BC1 is UNKNOWN. Once appended there is no reprint or delivery
ACK; queue admission does not guarantee USB delivery. CDC state3/pending<=512
and UP IRQ exclusion cover each append. D1/RX1 output waits until both boot rows
are admitted, including a late CDC open after stop. Session/fault freeze and
all live processing continue unchanged. Readers must not stop at RESET1: wait
bounded time for matching-current-nonce BC1 and retain missing as UNKNOWN.
Only the restricted UP image is supplied; three placements do not prove SMP
console admission or simultaneous cross-core fault publication.

## Validation and resource proof

- RED: old startup failed new-record-before-native assertion (`bc-red.log`).
  A second RED found D1 could precede BC1 on late CDC open (`bc-order-red.log`).
  Boot-row admission now gates only stopped diagnostic emission, not controls.
- GCC611/611 and Clang ASan/UBSan611/611;48 strict ARM units (all45 existing plus
  new module on UP/split/control). Tests cover partial/check/producer/power
  history, exactly one real feed, interrupted feed invalidation, inactive feed,
  snapshot-before-clear, late/busy CDC and no duplicate output.
- [Linked ELF checker](../../tests/check_pil_gate1_breadcrumb_elf.py) verifies
  NOLOAD/init ranges, sentinel survival, RAM stub and final Thumb vector. Its
  fail-closed small interpreter runs the actual linked Thumb bytes for18 cases:
  MSP/PSP, boundary/invalid frames, inactive and partial entry. It checks every
  frame read/write, no stack use/call/feed, preserved LR and original-handler
  delegation. This is NOT a general ARM emulator or hardware exception trial.
- All38 ordinary builds/SDK proofs and76 literal ELF/UF2 hashes match baseline
  at the retained public path: `build/pil-proof/gate1-breadcrumb/hashes.txt`.
  Shared template deviations remain8. Logs: `build/pil-gate1/bc-*`.
- Added RAM688B: live112 + old snapshot112 + line284(aligned) + active4 + RAM
  stub/literals176. Total Gate1 added storage15648B, below16KiB; Reset-B remains
  below800B. Compile guards reserve800B; the ELF checker sums actual storage.
  Whole image: text124588, data872(includes RAM code), bss140748B.
- Frames: B begin56 + validator16; startup56; PIL/tick/feed wrapper16 each.
  Getter has no stack/call on either placement. Native40 + tick16 is smaller
  than existing IRQ paths. UART80/lease16/service440/worker32 are unchanged.
  Reviewed additional IRQ maxima remain204UP/248SMP; worker896. Motion gains
  a16B wrapper frame; no new emergency stack or painted-reserve claim is made.
- Actual C row fixtures `bc-{empty,valid,partial,badframe,power}.txt` are281B
  each, with masks0/7f/2d/3f/0. Combined clean/missing USB fixtures are5883/2674B.
  Independent PC review verifies boot identity, D1/RX1 CRC/cohorts and stopped
  stack detection. They are synthetic and do not qualify a physical run.

## Exact image

`build_make/mtk3pico_smp0_usb_cdc_sim_gate1.uf2`,251392B.

UF2 SHA256: `f0d5e5508740fc532c037b3c8e873d36e179b99cb6ff05ad6ee35c97e3d19777`.
ELF SHA256: `3178fe1e7dc4891ca4e2c1cd9c9c288d37613a2353ed04c7aa70ffd8277bea9b`.
Retained map/size/image/checkpoint: `build/pil-gate1/artifacts/breadcrumb/`.
Use the [restricted build/operator handoff](PIL_GATE1_BUILD.md). The operator
must verify identity/image, capture initial/reopened CDC segments and the same
Probe tail, and retain the original FAIL. No automatic rearm or lease renewal.
