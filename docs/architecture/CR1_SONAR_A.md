# CR1 Sonar/obstacle A: compact sonar and obstacle boards

## Scope and decision

Operator approved category A on 2026-11-18: sonar board 44 to 40 B,
sonar history 704 to 640 B, obstacle board 36 to 28 B. Replace only the
stored payload of these three shared stores. Keep `sonar_snapshot_t`,
`sonar_history_entry_t`, `rm_obstacle_t`, all reader APIs, all messages,
the sonar request queue and all MQTT bytes unchanged. Do not change the
sonar, scan or detour timing or behaviour.

Category B items are not approved: keep `boot_id`, the detour
`duration_ms`, the `dr`/`dt` configuration copies and `latest_debug`.
The detour board packs to 36 B, the same as now, so it has no A change.
Private `ss_t`, `dr_t` and `dt_t` packing is not in this round.

Commits: `30db558` (sonar), `8a9af09` (obstacle). Base `7350da0`.

## Stored representation

No C bit-fields and no packed structs: masks and shifts on aligned
`uint32_t` words only. Every u32 counter and time keeps 32 bits, because
it wraps and is compared modulo 2^32.

### Sonar board and history

Source: [`sonar_pack.h`](../../app_program/subsystem5/sonar_pack.h).
Single writer: the sonar task. Access: the existing guard word plus
aligned atomic words. Readers copy all words, check the guard, then
unpack the local copy.

| Stored field | Valid range | Bits | Location |
| --- | --- | ---: | --- |
| generation, id, ping_sequence, result_sequence, trigger_ms, timestamp_ms, deadline_ms, ping_generation, ping_id | Full u32, including wrap | 32 each | Words 0–8 |
| echo | `ultrasonic_result_t`, all 16 bits | 16 | Word 9, bits 0–15 |
| angle | -30..30; code is angle + 30 | 6 | Bits 16–21 |
| ping_angle | -30..30; code is angle + 30 | 6 | Bits 22–27 |
| status | `sonar_status_t` 0..3 | 2 | Bits 28–29 |
| pending, valid | 0/1 | 1 each | Bits 30, 31 |

There are 320 bits in ten words (40 B), with no spare bits.
`sonar_request()` rejects an angle outside -30..30 before the queue, so
the stored angles are always in range. If a value does not pack, the
writer publishes nothing and the guards do not change.

### Obstacle board

Source: [`obstacle_pack.h`](../../app_program/vehicle/obstacle_pack.h).
Single writer: the Vehicle task, once per completed scan. Reader:
`vehicle_obstacle_try_read()` through the existing app_board sequence.

| Stored field | Valid range | Bits | Location |
| --- | --- | ---: | --- |
| scan_id, uptime_ms, boot_id | Full u32 | 32 each | Words 0–2 |
| flags | 4 flags, stage 1..2, attempt 0..2 | 8 | Word 3, bits 0–7 |
| decision | `obstacle_side_t` 0..2 | 2 | Bits 8–9 |
| closest_angle_deg | -30..30; code is angle + 30 | 6 | Bits 10–15 |
| closest_mm | Distance code 0..511 | 9 | Bits 16–24 |
| geometry_flags | 5 defined bits | 5 | Bits 25–29 |
| profile_word | 15 bits; bit 15 is always 0 | 15 | Word 4, bits 0–14 |
| right_free_step, left_free_step | 0..7 | 3 each | Bits 15–17, 18–20 |
| right_clear_deg, left_clear_deg | 0, 5, ..., 30 or 255; code deg / 5, 7 for 255 | 3 each | Bits 21–23, 24–26 |
| width_mm, left_clear_mm, right_clear_mm | 0..1022; code 1023 = NONE | 10 each | Word 5, bits 0–29 |

There are 183 logical bits in six words (24 B). Unused bits are 0.
Geometry bound: the distance code is at most 511 mm and the scan spans
60 degrees, so the width and the clearances are at most about 511 mm.
The 10-bit field has a margin of two.

## Before and after: build measurements

Public example secrets, Arm GNU 14.3.Rel1, demo line:

```text
make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
  KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
```

Before is the shared CR1 baseline `7350da0`. After is `8a9af09`.

| Quantity | Before | After | Change |
| --- | ---: | ---: | ---: |
| Static RAM, map parser | 241228 B | 241148 B | -80 B |
| Main RAM not statically reserved | 29108 B | 29188 B | +80 B |
| subsystem5 linked inputs | 1013 B | 945 B | -68 B |
| Vehicle linked inputs | 2516 B | 2508 B | -8 B |
| Sonar history (`history_words`) | 704 B | 640 B | -64 B |
| Sonar board (`board_words`) | 44 B | 40 B | -4 B |
| Obstacle board, sequence included | 36 B | 28 B | -8 B |
| ELF text (`size`) | 454596 B | 455364 B | +768 B |
| ELF bss (`size`) | 219060 B | 218980 B | -80 B |
| RAM functions | 79 / 21004 B | 79 / 21004 B | 0 |

The A saving is 76 B. The other 4 B is alignment padding.
The ARM message manifest (`scripts/message_sizes.py`) is byte-identical.
No message, queue or wire size changed. Of 224 objects, only
`sonar_service.o` and `vehicle_task.o` differ (strip-debug compare).
Public-example UF2 SHA-256 prefix: `9f48e723d2d9dba7`.

## Software checks

- Host tests: 679/679 (677 before, plus two new tests), 0 warnings.
- `test_sonar_pack`: all angle, ping angle, status, pending and valid
  combinations; all 65536 echo words; each u32 field at 0, 1, 2^31 - 1,
  2^31 and the maximum, with other fields unique; out-of-range input
  returns false and leaves the words unchanged.
- `test_sonar_history`: size budget 40/640 B; torn-read injection. A full
  writer publication runs between each of the ten payload word loads of
  the board and of the history. The reader returns `E_TMOUT`, and the next
  read returns the new value.
- `test_obstacle_pack`: every value of each field; unused bits 0; 13
  out-of-range inputs rejected; every edge pair at 511 mm and 200000
  random profiles through `obstacle_report_make()` always pack; torn read
  after each of the six board words leaves the output unchanged.
- Demo build: 0 warnings, RAM_PROOF PASS 79/79, 21004 B.

## Stack and timing limits

Pack time under the sonar mutex: `board_publish` from the demo ELF, run in
an instruction emulator with the Cortex-M0+ cycle table at 125 MHz and a
warm flash cache. Label: DERIVED(emulator). It was not measured on the
robot.

| Path | Before | After |
| --- | --- | --- |
| Board only | 434 cycles, 3.47 us | 312 cycles, 2.50 us |
| Board and history | 876 cycles, 7.01 us | 521 cycles, 4.17 us |
| Pack only | not present | 72 instructions, about 0.7 us |

The publication is shorter than before, because the byte copy is gone.
It is below 10 us, so the store stays under the mutex. With a cold flash
cache the code touches 40 eight-byte flash lines (39 before).

Compiler-local frames from `-fstack-usage`. These are **not whole-stack
measurements**.

| Function | Before | After |
| --- | ---: | ---: |
| sonar `board_publish` (with `store_snapshot`) | 24 + 64 B | 88 B |
| sonar `load_snapshot` | 56 B | 72 B |
| Vehicle `scan_step_observed` | 200 B | 232 B |
| `vehicle_obstacle_try_read` | 64 B | 112 B |

Task stacks are not changed. A robot stack high-water check is still open.

## Shared access and deferred work

There is one writer for each store. No writer does a read-modify-write on
a shared packed word. The echo ISR and its `latest_result` word do not
change. The sequentially consistent word stores stay valid if a reader
moves to core 1 later.

Deferred, each needs a separate approval: category B items above,
private scan and detour state, and an MQTT wire repack of
`robot/sonar`, `robot/obstacle` and `robot/detour`.
