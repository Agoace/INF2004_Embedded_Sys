# CR1 KR1 RAM row: 16 B to 15 B

## Status

| Item | Value |
| --- | --- |
| Approval | Operator approved option (a) with the time guard |
| Archive tag | `archive/mtk3-cr1-kr1row`, base `b68ce6b` (staging) |
| Commits | `d30a73b` red tests and row codec; `2269320` storage change; `6b6408d` this record; then a `--no-ff` merge of staging `290439c` (minimum-bit proof) and the KR1 proof row update |
| Scope | RAM storage only. One `--no-ff` merge into staging |
| Revert | `git revert -m 1 <merge commit>` removes the full change |
| Robot | Substep: not flashed. Later combined image 12 checked rows, timing and stacks (2026-11-19); see [project status](PROJECT_STATUS.md#areas) |

Source of the field audit: SW7 report `/tmp/SW7_KR1_ROW_CHECK.md`
(temporary, not committed). This record repeats the facts that the code
uses.

## What changes and what does not

Telemetry stores each KR1 RAM-log row in 15 private bytes:
`rows[4501][15]`. The Telemetry task packs the row when it admits it,
under `stats_lock`. The two readers unpack it to the unchanged 16-byte
row:

- `telemetry_log_row` (robot/log MQTT rows), under `stats_lock`.
- The USB dump `KR` lines (Telemetry task, dump exclusion).

These items do not change:

- The 16-byte KR1 row from `vr_pack` and `telemetry_message_t`.
- robot/log row bytes, live `kr1` in the vehicle message, USB `KR` text.
- `scripts/robot_log_decode.py`, the MQTT decoders, `MQTT_WIRE.md`.
- Queue sizes, message sizes, rates and admission atomics.

There is no build switch.

## Field audit

| Field | 16 B bits | Stored bits | Reason |
| --- | ---: | ---: | --- |
| time (elapsed_ms / 20) | 16 | 13 | Index-anchored offset; see [time](#time-and-guard) |
| state | 2 | 2 | Kept |
| module | 3 | 3 | Kept |
| raw L, raw C | 12 + 12 | 24 | Kept |
| position | 8 | 8 | Kept |
| asked L/R, measured L/R | 4 × 10 | 40 | Kept (the clamp in `vr_pack` is unchanged) |
| echo | 16 | 14 | Bits 14..15 are always 0 (`ultrasonic.h` masks 0x01FF, 0x0600, 0x3800) |
| servo | 7 | 7 | Kept |
| events | 8 | 8 | Kept |
| flags | 4 | 1 | `vr_pack` sets only bit 0 (speed clamp) |
| Total | 128 | 120 | 15 bytes |

## Stored layout

Little-endian bit order, the same as `vehicle_log.h`.

| Stored bits | Content |
| --- | --- |
| 0..12 | d = (time16 − index) mod 65536, d < 8192 |
| 13..103 | 16-byte row bits 16..106 (state to echo bit 13) |
| 104..119 | 16-byte row bits 109..124 (servo, events, flag bit 0) |

The unpack writes 0 to row bits 107..108 (echo 14..15) and 125..127
(flags 1..3). The code is `app_program/subsystem1/kr1_row_pack.h`. It
uses straight-line 32-bit shifts: no loop, no division, no kernel call.

## Time and guard

The unpack sets time16 = (index + d) mod 65536. This is the only value
in [index, index + 8191] with that offset.

The vehicle writes a row only when 20 ms or more have gone since the
last row. Row i therefore has time ≥ i ticks. A dropped or late row only
makes the offset larger. Lossless storage needs time − index < 8192
ticks: the total lateness over the stored rows must stay below
163.84 s. The code does not enforce this, so the guard does:

- When d ≥ 8192, `kr1_row_pack` refuses the row and writes nothing.
- When a zero-only bit (row bits 107..108 or 125..127) is set, it also
  refuses the row.
- Telemetry counts a refused row through the existing `lose_locked()`
  path. The row is not stored and the index does not advance.
- A refused row does not set `admit_full`. The log is not full.

Thus a stored row always reads back as the exact admitted 16 bytes. A
possible error becomes a counted row loss. The Vehicle's existing policy
for a reported loss does not change.

Wrap: the 16-bit time field wraps after 65536 ticks (21.8 min). The
guard and the decode use the stored 16-bit field, so the output stays
byte-identical: a wrapped row is stored only if (time16 − index) mod
65536 < 8192.

## Producer proof

`tests/test_kr1_row_pack.c` sends values through the real `vr_pack`,
then packs, unpacks and compares all 16 bytes. 695728 round trips pass:

- Each field over its full domain at indices 0, 1, 2, 255, 4095, 4499
  and 4500: state 0..3, module 0..7, raw 0..4095 (each), position
  −128..127, servo −64..63, events 0..255, echo 0..0x3FFF.
- Speeds: every value −599..599 for each of the four fields, plus the
  int16 extremes and every 256th value. The clamp sets flag bit 0.
- Time: every offset 0..8255 at indices 0, 1, 3, 1000 and 4500, with
  0 ms and 19 ms remainders. Offsets below 8192 store; the others are
  refused.
- Guard edges: offset 8191 stores, 8192 is refused, time below the
  index is refused.
- Wrap: time16 5 at index 3 stores (true time 65541 ticks); time16 1 at
  index 4 is refused; arithmetic wrap of the decode.
- Each single row bit 16..127: kept bits round-trip; bits 107, 108 and
  125..127 are refused. A row with all kept bits set round-trips.
- 400000 random `vr_values_t` values (full C domain of each field, echo
  in its 14-bit range, time within the window) round-trip. An echo of
  0x4000 is refused, not truncated.

The real producer is `save_row` in `vehicle_task.c` (`.elapsed_ms = now −
run_start`). The echo comes from `drv_ultrasonic.c` (9 + 2 + 3 bits) or
`ULTRASONIC_RESULT_NONE`. `vehicle_task.c` and `vehicle_log.c` do not
change.

## APP_TEST rows

Before this change, APP_TEST code sent rows with arbitrary bytes. With
the guard, a row with an invalid time is refused. These changes keep the
tests correct:

| Code | Behaviour now |
| --- | --- |
| `subsystem1_test.c` S1-U02 burst | Each row carries time = accepted count. The PASS rule does not change |
| `telemetry_contention_test.c` storage-full test | Each row carries time = accepted count. Rows are stored in FIFO order, so the stored index is never above it, and all rows are accepted until the log is full |
| `TELEMETRY_TEST_ROW` probe | `discarded` now means "not stored": the log is full or the guard refused the row. Before, only a full log was possible |

The probe token is the row's first four bytes. A token whose low 16 bits
are 8192 or more ticks after the index is refused and reports discarded.
The contention test sends its probe only when the log is full, so its
result does not change. These APP_TEST builds were not run on the robot.

## Tests

| Gate | Result |
| --- | --- |
| Red first | `d30a73b`: `test_telemetry_log` (storage size) and `test_telemetry_probe_*` (guard probe) fail; 5 of 699 fail |
| Host, GCC | 699/699 pass |
| Host, Clang ASan + UBSan | 699/699 pass, 0 runtime errors |
| Full buffer | 4501 valid rows store; row 4502 is a full-log loss |
| Guard via Telemetry | offset 8192, echo bit 14 and flags bit 1 are each refused: lost +1, rows unchanged, log not full; offset 8191 with flag bit 0 stores exactly |
| USB text | The dump prints the exact 16 bytes of two dense rows |
| Public sizes | `scripts/message_sizes.py` output is identical before and after |

## RAM

Demo build (MQTT image, the CR1 demo line). 0 warnings. RAM_PROOF PASS
79/79, 21212 B in both builds.

| Item | Before `b68ce6b` | After | Change |
| --- | ---: | ---: | ---: |
| `.bss.rows` (telemetry.o) | 72016 | 67515 | −4501 |
| S1 module RAM | 78882 | 74381 | −4501 |
| Linker alignment | 943 | 940 | −3 |
| Static RAM | 239304 | 234800 | −4504 |
| Flash text | 468260 | 468620 | +360 |

The linker alignment saving is separate from the data saving. Do not
count it twice.

## Lock-time cost

Method: the pack, unpack and the old 16-byte copy loop are compiled with
the demo flags (`-mcpu=cortex-m0plus -O2`) as standalone functions. An
instruction emulator counts Cortex-M0+ cycles (SW7's cycle table, warm
flash cache, 125 MHz). Label: DERIVED(emulator). It was not measured on
the robot. The inline code inside the Telemetry task can differ by a few
cycles.

| Path | Cycles | Time at 125 MHz |
| --- | ---: | ---: |
| Old 16-byte copy (store and read) | 128 | 1.02 µs |
| Pack, row stored | 164 | 1.31 µs |
| Pack, row refused | 69 | 0.55 µs |
| Unpack | 143 | 1.14 µs |

The store path under `stats_lock` is about 36 cycles (0.29 µs) longer.
The `telemetry_log_row` read under the lock is about 15 cycles (0.12 µs)
longer. The emulated round trip gives the identical 16 bytes. Host
timing (x86, diagnostic only): pack 17.9 ns, unpack 19.4 ns.

## Minimum-bit proof row

After the merge of staging `290439c`, the KR1 record in
`tests/min_bit_records.inc` proves the 15-byte stored row with the real
`vr_pack` and `kr1_row_pack`. The proof packs row 0, where the stored time
offset equals the time. All 11 fields are at their minimum width: time
13 bits (offset 0..8191 from the row index), echo 14 bits, flags 1 bit.
Time 8192 and echo 0x4000 are refused. The record has 0 spare bits and
4501 × 15 = 67515 B, so the earlier KR1 gap (echo and flags) is closed.
The doc block in [CR1_MIN_BIT_PROOF.md](CR1_MIN_BIT_PROOF.md) was
regenerated with `test_min_bit_proof --markdown`. After the merge: host
701/701, Clang ASan + UBSan 701/701, demo 0 warnings, RAM_PROOF 79/79,
static RAM 234800 B, S1 74381 B (the same as `2269320`).

## Open items

This work is merged. The current stored row is 15 B; the public KR1 row is
still 16 B. MEASUREMENTS is generated; send corrections to the ledger owner.
Combined image 12 checked the rows on the robot. Row-guard loss remains
visible in existing loss counts; that run is not a worst-case lateness proof.
The demo now runs up to 300 s and retains only the first 90 s in RAM;
a full log no longer stops driving or MQTT. See
[demo-kernel-v2](../release/DEMO_KERNEL_V2.md).

## Evidence

Ignored files under `build/wip/kr1row/`:

- `red-host.log`, `green-host.log`, `asan.log` and their build logs.
- `before/` and `after/`: map, ELF, UF2, RAM proof, `ram_report.txt`,
  `message_sizes.txt`.
- `roundtrip.txt`: round-trip count and host timing.
- `cycles/`: `kr1_cycles.c`, `cycles_kr1.py`, `cycles.txt`.
