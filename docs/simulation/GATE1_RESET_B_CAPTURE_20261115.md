# Reset-B companion capture — 2026-11-15

**TRIAL / NOT GATE 1 PASS.** This is diagnostic evidence, not recovery.
RESET1, D1, RX1, UART v1, source timing and all native age checks stay unchanged.
No hardware access or firmware change is performed by these parser tests.

## Frozen BC1 row

RESET1 remains exactly122 bytes. The firmware sends a separate BC1 row after it:
`[PILG1BC1 `,30 lowercase eight-digit hexadecimal words separated by one space,
`]` and literal LF. BC1 is exactly281 bytes, without NUL, CRLF or whole-row CRC.
Queue admission is not delivery acknowledgement.

| Words | Meaning |
| --- | --- |
| 0 | Current boot nonce; must match the preceding RESET1 in this segment. |
| 1 | Validity mask; only bits0..6 are defined. |
| 2..29 | 28 raw words from the PREVIOUS boot retained record. |

Previous raw layout:

| Raw words | Meaning |
| --- | --- |
| 0..3 | Magic50424731, producer/layout00010001, owner nonce, header check. |
| 4..7 | Motion publication sequence, feed completion-observation time, caller task ID, check. |
| 8..11 | PIL sequence, phase time, packed task ID shifted8 bits plus phase, check. |
| 12..15 | Native ALARM0 sequence, entry time, interrupted thread ID, check. |
| 16..27 | Fault status, time, PC, LR, xPSR, original SP, EXC_RETURN, PRIMASK, IPSR, core ID, check, inverse check. |

The parser independently recomputes the exact native validity mask. Salt is
9e3779b9. Header check is salt XOR raw0 XOR raw1 XOR raw2. Header trust also
requires RESET1 WATCHDOG.REASON bits0 or1, the exact magic/producer and nonzero
previous owner nonce. Each normal publication at base4/8/12 needs a nonzero even
sequence and check = salt XOR base XOR sequence XOR time XOR value. Fault check
is salt XOR status XOR raw17..25, with its32-bit inverse in raw27.
These checks detect some incomplete/corrupt writes, not all corruption.

| Mask bit | Native claim, independently checked |
| --- | --- |
| 0 | Header trusted under the captured raw reset reason. |
| 1/2/3 | Motion/PIL/ALARM0 publication valid. |
| 4 | Fault publication complete with valid check/inverse and status2 or3. |
| 5 | Entry marker observed: raw status1,2 or3; complete integrity is not required. |
| 6 | Complete fault publication with status2, the native frame-captured flag. |

Fault status1 is entered/incomplete;2 is frame captured;3 is invalid frame.
Entry status is stored BEFORE fault time. Incomplete-entry time/registers are
UNKNOWN even if stale raw words contain plausible values. Status3 exposes no
trusted PC/LR/xPSR frame. A missing entry marker does not rule out a fault.
PIL phases1/2 mean ENTER/BEFORE_SLEEP. Unrepresentable packed task ID ffffffff
has unknown decoded task/state. Task IDs are samples, not CPU accounting.
ALARM0 is not the kernel SysTick.

The feed sequence becomes odd BEFORE the single original hardware feed call.
An interrupted publication remains UNKNOWN. Its final timestamp observes call
completion; preemption can place this AFTER hardware reload. Age measured from
that timestamp is only a LOWER bound on actual hardware-feed age, not an exact
reload time. Frame-valid means the native handler could read32 RAM bytes; it
does not prove an uncorrupted hardware stack frame.

## Parser and capture JSON

`parse_breadcrumb_records` in `pil/gate1_reset_capture.py` requires a preceding
same-nonce RESET1. Offline standalone rows can supply an explicit uint32
`watchdog_reason`; without either context they remain UNKNOWN. Wrong framing,
version, case, reserved mask bits or independently recomputed mask mismatch is
rejected. RESET1 parsing itself is unchanged.

Each CDC JSON segment now contains `breadcrumb_records` and `breadcrumb_status`.
A record keeps all `raw_previous_words`, the current nonce, validity mask,
trusted `previous_owner_nonce`, publication fields, offset/length and RAW
completion time. Invalid header/publications are null, not fabricated zero
history. `fault.entry_marker_observed`, `integrity_valid` and `frame_valid`
are separate claims. `COMPLETE_ROW` means the latest RESET1 has a complete,
matching BC1 row, not that every previous publication or the fault frame is valid.

The existing CDC command needs no new flag. After the first reopened RESET1,
the reader waits up to1s for its companion, within the global duration/budgets.
It closes early only after a complete matching BC1 row. Missing, partial,
wrong-nonce or rejected rows remain UNKNOWN with raw bytes retained. Legacy
RESET1-only firmware therefore gets a bounded extra wait. The old D1/stack
reset-watch deadline must not close this companion wait early. CDC remains
RX/reopen only, with no Probe reader, write, ARM or estimator input.

All stored progress/fault times belong to the previous boot's32-bit timer.
Do not subtract them from current boot time or infer reset time from them.
A frame provides candidate addresses for the EXACT firmware ELF, not an
automatic root cause. Match the previous owner nonce to the failed run's OLD
BOOT/CDC segment before attributing its history; first-B-boot history can be
unknown or unrelated. Retain the image SHA/ELF, both nonces and original FAIL.

## Actual C fixtures and tests

SW6's `test_pil_gate1_breadcrumb.c` generated the five tracked281-byte
`sim/webots/tests/fixtures/pil_gate1_bc_*.txt` rows. Expected masks:
empty0, valid7f, partial2d, badframe3f, power0. Power uses reason0, even though
its raw previous fields look valid. Other standalone fixtures use reason1.
They are synthetic C fixtures, not captured physical fault frames.

SW6's actual runtime stream fixtures are also retained unchanged:

| Fixture | Bytes | D1 reason | RX1 packet/alarm rows |
| --- | --- | --- | --- |
| pil_gate1_bc_runtime.bin | 5883 | 1 | 14/10 |
| pil_gate1_bc_missing.bin | 2674 | 256 | 4/3 |

Both have RESET1 nonce7b followed by BC1 nonce7b/mask0 and preserve D1/RX1
identity, cohort and stack framing. Their SHA256 values are respectively
`cb666d0b3958595e43db4e18937c18fb9a15a2e00907ded1518c63b7fa72531c` and
`70d9a4143882f05d42e9d700f725c223ca5db62386072ec89426867c12e2d0da`.

`tests/test_gate1_breadcrumb_capture.py` covers these actual C rows, independent
checks, strict framing, missing reason context, partial publication semantics,
delayed/split companion delivery, nonce mismatch, bounded legacy wait and
unchanged complete D1/RX1/stack stop detection. No-device tests are not Gate1
acceptance or a future Linux/USB timing bound.
