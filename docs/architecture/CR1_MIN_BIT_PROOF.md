# CR1 minimum-bit proof

## Status

This record proves, field by field, that each CR1 packed record uses the
minimum number of bits for the values it can hold. A host test measures
each field from the real pack code. A second test makes sure that the
tables in this document are the test output.

- Proof test: [`test_min_bit_proof.c`](../../tests/test_min_bit_proof.c)
  with the tables in
  [`min_bit_records.inc`](../../tests/min_bit_records.inc).
- Document test: [`test_min_bit_doc.py`](../../tests/test_min_bit_doc.py)
  (ctest `test_min_bit_doc`).
- Base: `mtk3-staging` `b68ce6b`. The proof adds tests and documents only.
  The firmware is byte-identical.
- The KR1 row is recorded as it is on staging (16 B). The approved 15 B
  row is on branch `mtk3-cr1-kr1row`. Its owner updates the KR1 table
  after this proof is merged.
- The `GAP` rows are a report only. The fault snapshot and STOP capture
  gaps get a fix on a separate branch. The operator accepted three other
  items with a written reason (`SMALL_GAIN`, `ELEMENT_ALIGNED`).

The module records keep the design detail. This record does not repeat
it: [CR1.md](CR1.md) (motion), [CR1_SONAR_A.md](CR1_SONAR_A.md),
[CR1_SONAR_F.md](CR1_SONAR_F.md), [CR1_VEHICLE_A1.md](CR1_VEHICLE_A1.md),
[CR1_VEHICLE_F.md](CR1_VEHICLE_F.md), [CR1_BARCODE_A.md](CR1_BARCODE_A.md)
and [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md).

## Rule

| Term | Meaning |
| --- | --- |
| n | Number of values the field can hold: the values the pack code accepts. When the pack code does not check a field, n comes from the producer (noted in the range). |
| Min bits | ceil(log2 n). A "no value" code (for example NONE or UNKNOWN) counts as one value. |
| Independent bits | k flags that can be set in any mix have n = 2^k, so k bits. |
| Full-range field | A wrapping u32 counter or time, a random id, or an exact binary32 float has n = 2^32, so 32 bits is the minimum. |
| Bits used | Width of the field slot in the stored element. |
| Reason | Necessary when bits used > min bits. The list is closed: `WORD_SLACK`, `GAP` or `SMALL_GAIN`. A record can also state `ELEMENT_ALIGNED`. |
| `WORD_SLACK` | Fewer bits save 0 B over the WHOLE record, all elements included. The test computes this; a field in an array cannot use it when the array total can become smaller. |
| `GAP` | Fewer bits save bytes. Report only. The gap table gives the bytes. |
| `SMALL_GAIN` | Fewer bits save a small number of bytes, and the operator accepted it. The row states the bytes; the test makes sure that the figure is the true saving. |
| `ELEMENT_ALIGNED` | Record reason for an array. Every field is minimal, but each element starts on a whole byte or word, so that the code can get element i directly (O(1)) under the lock. The record states the bytes that one bit stream would save; the test makes sure that the figure is exact. |

The test fails when a reason is missing, when a reason is on a field
that has the minimum width, when `WORD_SLACK` hides a saving, when
`GAP` or `SMALL_GAIN` saves 0 B, when a `SMALL_GAIN` or
`ELEMENT_ALIGNED` figure is not the computed bytes, and when an array
has alignment bytes with no `ELEMENT_ALIGNED` figure.

## What the test does

For each field, the test starts from a valid value, changes only that
field and packs with the real code. Every value is tried when n ≤ 4096.
Otherwise the test tries the end values and the values 2^k and 2^k − 1.

| Check | Failure that it finds |
| --- | --- |
| The changed stored bits (footprint) are inside the slot. | A field writes outside its slot. |
| The footprint has exactly min-bits bits. | The code uses more bits than the minimum, or fewer than the values need. |
| Width = min bits, or a valid reason. | A wider slot with no reason, or a stale reason. |
| Each value unpacks to the same value. | Lost or changed values. |
| Listed out-of-range values are refused, and the output stays unchanged. | A pack that accepts more values than n. |
| Slots do not overlap; other bits stay 0. | Overlap, dirty spare bits. |
| Sum of widths + spare = element bits; byte totals are as listed. | A layout change with no table change. |

The tables between the markers below are the output of
`test_min_bit_proof --markdown`. Each record gives its byte totals:
stored, the least bytes at the used widths, and the least bytes at the
minimum widths. "Whole elements" keeps each element in whole bytes or
words. "One bit stream" is possible only for one plain array.

## Before and after

Bytes in RAM, from the module records. Boards include their 4 B sequence.

| Record | Before | After | Source |
| --- | ---: | ---: | --- |
| Motion observation board | 72 | 64 | [CR1.md](CR1.md) |
| Sonar board | 44 | 40 | [CR1_SONAR_A.md](CR1_SONAR_A.md) |
| Sonar history (16) | 704 | 640 | [CR1_SONAR_A.md](CR1_SONAR_A.md) |
| Obstacle board | 36 | 28 | [CR1_SONAR_A.md](CR1_SONAR_A.md) |
| Vehicle board | 44 | 40 | [CR1_VEHICLE_A1.md](CR1_VEHICLE_A1.md) |
| Barcode frame boards (8) | 832 | 736 | [CR1_BARCODE_A.md](CR1_BARCODE_A.md) |
| Barcode capture edges (3 slots) | 4608 | 4176 | [CR1_BARCODE_A.md](CR1_BARCODE_A.md) |
| Telemetry events (512) | 6144 | 4608 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) |
| Telemetry index board | 24 | 20 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) |
| Stopped scan rows (48) | 576 | 528 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) |
| Retained INFO | 108 | 104 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) |
| Health payload | 120 | 112 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) |
| S1 pending sonar copy | 40 | 32 | [CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md) |
| KR1 RAM-log rows (4501) | 72016 | 72016 | not changed on staging |

The F fault mask adds 104 B (four 8 B owner boards, the 36 B STOP
capture, the 16 B working views and 20 B in the Health board). Static
RAM of the demo image: 241228 B at `7350da0`, 239304 B at `b68ce6b`
(−1924 B). The detail is in
[CR1_TELEMETRY_A.md](CR1_TELEMETRY_A.md).

## Proof tables

<!-- min-bit:begin -->
### Motion A1 observation

Source: `app_program/subsystem2/motion_pack.h`. 1 element(s) of 60 B (480 bits, 22 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| counts[2] | int32 (wrapping count) | 4294967296 | 32 | 32 |  |
| travel_mm[2] | int32 | 4294967296 | 32 | 32 |  |
| heading_cdeg | int32 | 4294967296 | 32 | 32 |  |
| ticks | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| tick_max_us | u32 | 4294967296 | 32 | 32 |  |
| gap_min_us | u32 | 4294967296 | 32 | 32 |  |
| gap_max_us | u32 | 4294967296 | 32 | 32 |  |
| late_wakes | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| missed_wakes | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| publication us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| speed_mm_s[2] | int16 (code clamp INT16_MIN..MAX) | 65536 | 16 | 16 |  |
| duty_permille[2] | -800..800 | 1601 | 11 | 11 |  |
| servo_deg | -36..36 | 73 | 7 | 7 |  |
| servo_us (pulse code) | 73 pulses of -36..36 deg | 73 | 7 | 7 |  |
| servo_settled | bool | 2 | 1 | 1 |  |
| faults | 4 independent bits | 16 | 4 | 4 |  |
| watchdog_armed | bool | 2 | 1 | 1 |  |

Bits per element: used 458, minimum 458. Layout: separate allocations. Bytes: stored 60; least at used widths 60; least at minimum widths 60 (each element alone in whole words: 60). The pulse code repeats the servo angle; removing it is category B5 (not approved).

### Sonar board

Source: `app_program/subsystem5/sonar_pack.h`. 1 element(s) of 40 B (320 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| generation | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| id | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| ping_sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| result_sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| trigger_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| timestamp_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| deadline_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| ping_generation | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| ping_id | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| echo | ultrasonic word: distance 9, status 2, streak 3 bits | 16384 | 14 | 16 | WORD_SLACK |
| angle | -30..30 | 61 | 6 | 6 |  |
| ping_angle | -30..30 | 61 | 6 | 6 |  |
| status | 0..3 | 4 | 2 | 2 |  |
| pending | bool | 2 | 1 | 1 |  |
| valid | bool | 2 | 1 | 1 |  |

Bits per element: used 320, minimum 318. Layout: separate allocations. Bytes: stored 40; least at used widths 40; least at minimum widths 40 (each element alone in whole words: 40). Echo keeps all 16 bits of the native word; the top 2 bits are always zero. Word 9 stays one word, so 14 bits save 0 B.

### Sonar history

Source: `app_program/subsystem5/sonar_pack.h`. 16 element(s) of 40 B (320 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| generation | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| id | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| ping_sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| result_sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| trigger_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| timestamp_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| deadline_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| ping_generation | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| ping_id | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| echo | ultrasonic word: distance 9, status 2, streak 3 bits | 16384 | 14 | 16 | SMALL_GAIN (4 B) |
| angle | -30..30 | 61 | 6 | 6 |  |
| ping_angle | -30..30 | 61 | 6 | 6 |  |
| status | 0..3 | 4 | 2 | 2 |  |
| pending | bool | 2 | 1 | 1 |  |
| valid | bool | 2 | 1 | 1 |  |

Bits per element: used 320, minimum 318. Layout: one array. Bytes: stored 640; least at used widths 640; least at minimum widths 636 (each element alone in whole words: 640). Same layout as the board. Only a bit stream over all 16 entries can use the 2 echo bits (4 B). Accepted by the operator: 4 B needs a new history layout.

### Obstacle board

Source: `app_program/vehicle/obstacle_pack.h`. 1 element(s) of 24 B (192 bits, 9 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| scan_id | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| uptime_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| boot_id | u32 (random) | 4294967296 | 32 | 32 |  |
| flags | 8 independent bits | 256 | 8 | 8 |  |
| decision | 0..2 | 3 | 2 | 2 |  |
| closest_angle_deg | -30..30 | 61 | 6 | 6 |  |
| closest_mm | 0..511 | 512 | 9 | 9 |  |
| geometry_flags | 5 independent bits | 32 | 5 | 5 |  |
| profile_word | 0..0x7fff | 32768 | 15 | 15 |  |
| right_free_step | 0..7 | 8 | 3 | 3 |  |
| left_free_step | 0..7 | 8 | 3 | 3 |  |
| right_clear_deg | 0,5,..,30 or 255 | 8 | 3 | 3 |  |
| left_clear_deg | 0,5,..,30 or 255 | 8 | 3 | 3 |  |
| width_mm | 0..1022 or NONE | 1024 | 10 | 10 |  |
| left_clear_mm | 0..1022 or NONE | 1024 | 10 | 10 |  |
| right_clear_mm | 0..1022 or NONE | 1024 | 10 | 10 |  |

Bits per element: used 183, minimum 183. Layout: separate allocations. Bytes: stored 24; least at used widths 24; least at minimum widths 24 (each element alone in whole words: 24).

### Vehicle A1 snapshot

Source: `app_program/vehicle/vehicle_pack.h`. 1 element(s) of 36 B (288 bits, 10 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| timestamp_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| wake_lost | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| overruns | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| response_max_us | u32 | 4294967296 | 32 | 32 |  |
| handoffs | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| command_id | u32 (remote id) | 4294967296 | 32 | 32 |  |
| left_mm_s | int16 | 65536 | 16 | 16 |  |
| right_mm_s | int16 | 65536 | 16 | 16 |  |
| servo_deg | -36..36 | 73 | 7 | 7 |  |
| state | 0..2 | 3 | 2 | 2 |  |
| module | 0..4 | 5 | 3 | 3 |  |
| phase | 0..3 | 4 | 2 | 2 |  |
| command_opcode | 0..4 | 5 | 3 | 3 |  |
| command_status | 0..5 | 6 | 3 | 3 |  |
| active | bool | 2 | 1 | 1 |  |
| calibrated | bool | 2 | 1 | 1 |  |

Bits per element: used 278, minimum 278. Layout: separate allocations. Bytes: stored 36; least at used widths 36; least at minimum widths 36 (each element alone in whole words: 36).

### KR1 RAM-log row

Source: `app_program/vehicle/vehicle_log.h`. 4501 element(s) of 16 B (128 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| time (20 ms ticks) | 0..65535 (wraps; no run time limit in code) | 65536 | 16 | 16 |  |
| state | 0..2 | 3 | 2 | 2 |  |
| module | 0..5 (5 = scan reverse) | 6 | 3 | 3 |  |
| raw L, C[2] | ADC 0..4095 | 4096 | 12 | 12 |  |
| position | -100..100 | 201 | 8 | 8 |  |
| asked L, R[2] | -512..511 (clamped) | 1024 | 10 | 10 |  |
| measured L, R[2] | -512..511 (clamped) | 1024 | 10 | 10 |  |
| echo | ultrasonic word: distance 9, status 2, streak 3 bits | 16384 | 14 | 16 | GAP |
| servo | -36..36 | 73 | 7 | 7 |  |
| events | 8 defined bits (lost, found, echo, ping, outcome 3, fault) | 256 | 8 | 8 |  |
| flags | bit 0 only (speed clamp) | 2 | 1 | 4 | GAP |

Bits per element: used 128, minimum 123. Layout: one array. Bytes: stored 72016; least at used widths 72016; least at minimum widths 69203 (each element alone in whole bytes: 72016). Slack: echo 2 bits and flags 3 bits. A row stays 16 B (123 bits) unless the time also drops 3 bits (120 bits), which needs a decode rule that the code does not have.

### Barcode frame boards

Source: `app_program/subsystem3/barcode_evidence_pack.h`. 8 element(s) of 88 B (704 bits, 10 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| epoch | u32 | 4294967296 | 32 | 32 |  |
| sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| read_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| read_counts[2] | int32 | 4294967296 | 32 | 32 |  |
| frame_end_counts[2] | int32 | 4294967296 | 32 | 32 |  |
| eligibility_revision | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| command | 0..4 | 5 | 3 | 3 |  |
| valid | 0..1 | 2 | 1 | 1 |  |
| source | 0..2 | 3 | 2 | 2 |  |
| letter | NUL or 44 Code39 letters | 45 | 6 | 6 |  |
| direction | 0..2 | 3 | 2 | 2 |  |
| flags | 8 bits | 256 | 8 | 8 |  |
| version | 0..1 | 2 | 1 | 1 |  |
| index | u32 | 4294967296 | 32 | 32 |  |
| reason | 0..10 or 100..107 | 19 | 5 | 5 |  |
| start_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| edge_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| frame_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| eligible_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| terminal_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| control_age_ms | u32 | 4294967296 | 32 | 32 |  |
| record_age_ms | u32 | 4294967296 | 32 | 32 |  |
| spin_white | u32 | 4294967296 | 32 | 32 |  |
| leading_white | 0..220 | 221 | 8 | 8 |  |
| narrow_mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| wide_mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| leading_mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| rise | 0..350 | 351 | 9 | 9 |  |
| fall | 0..275 | 276 | 9 | 9 |  |

Bits per element: used 694, minimum 694. Layout: separate allocations. Bytes: stored 704; least at used widths 704; least at minimum widths 704 (each element alone in whole words: 704). Eight separate boards (barcode_live_source.c FRAME_ROWS).

### Barcode failed-capture frames

Source: `app_program/subsystem3/barcode_evidence_pack.h`. 3 element(s) of 88 B (704 bits, 10 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| epoch | u32 | 4294967296 | 32 | 32 |  |
| sequence | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| read_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| read_counts[2] | int32 | 4294967296 | 32 | 32 |  |
| frame_end_counts[2] | int32 | 4294967296 | 32 | 32 |  |
| eligibility_revision | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| command | 0..4 | 5 | 3 | 3 |  |
| valid | 0..1 | 2 | 1 | 1 |  |
| source | 0..2 | 3 | 2 | 2 |  |
| letter | NUL or 44 Code39 letters | 45 | 6 | 6 |  |
| direction | 0..2 | 3 | 2 | 2 |  |
| flags | 8 bits | 256 | 8 | 8 |  |
| version | 0..1 | 2 | 1 | 1 |  |
| index | u32 | 4294967296 | 32 | 32 |  |
| reason | 0..10 or 100..107 | 19 | 5 | 5 |  |
| start_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| edge_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| frame_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| eligible_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| terminal_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| control_age_ms | u32 | 4294967296 | 32 | 32 |  |
| record_age_ms | u32 | 4294967296 | 32 | 32 |  |
| spin_white | u32 | 4294967296 | 32 | 32 |  |
| leading_white | 0..220 | 221 | 8 | 8 |  |
| narrow_mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| wide_mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| leading_mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| rise | 0..350 | 351 | 9 | 9 |  |
| fall | 0..275 | 276 | 9 | 9 |  |

Bits per element: used 694, minimum 694. Layout: separate allocations. Bytes: stored 264; least at used widths 264; least at minimum widths 264 (each element alone in whole words: 264). One frame in each of the three capture slots.

### Barcode failed-capture edges

Source: `app_program/subsystem3/barcode_evidence_pack.h`. 64 element(s) of 24 B (174 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| motor_us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| left | u32 count | 4294967296 | 32 | 32 |  |
| right | u32 count | 4294967296 | 32 | 32 |  |
| mm | binary32 bits (exact) | 4294967296 | 32 | 32 |  |
| raw | ADC 0..4095 | 4096 | 12 | 12 |  |
| black | 0..1 | 2 | 1 | 1 |  |
| phase | WINDOW 2 or TAIL 3 | 2 | 1 | 1 |  |

Bits per element: used 174, minimum 174. Layout: one bit stream. Bytes: stored 1392; least at used widths 1392; least at minimum widths 1392 (each element alone in whole words: 1392). Edges are packed back to back (174 bits each). There is one array in each of the three capture slots.

### Telemetry events

Source: `app_program/subsystem1/telemetry_pack.h`. 512 element(s) of 9 B (72 bits, 6 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| word | u16 payload | 65536 | 16 | 16 |  |
| detail | u8 payload | 256 | 8 | 8 |  |
| kind | 1..10 | 10 | 4 | 4 |  |
| from | 0..5 | 6 | 3 | 3 |  |
| to | 0..5 | 6 | 3 | 3 |  |

Bits per element: used 66, minimum 66. Layout: one array. Bytes: stored 4608; least at used widths 4224; least at minimum widths 4224 (each element alone in whole bytes: 4608). ELEMENT_ALIGNED: 384 B. Each event starts on a byte, so 6 zero bits per event. Accepted by the operator: whole events give O(1) indexed access under the lock.

### Telemetry log index

Source: `app_program/subsystem1/telemetry_pack.h`. 1 element(s) of 16 B (128 bits, 9 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| generation | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| lost | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| newest_ms | u32 (wrapping time) | 4294967296 | 32 | 32 |  |
| rows | 0..4501 | 4502 | 13 | 13 |  |
| events | 0..512 | 513 | 10 | 10 |  |

Bits per element: used 119, minimum 119. Layout: separate allocations. Bytes: stored 16; least at used widths 16; least at minimum widths 16 (each element alone in whole words: 16).

### Stopped scan rows

Source: `app_program/subsystem1/telemetry_pack.h`. 48 element(s) of 11 B (88 bits, 7 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| scan_id | u32 (wrapping) | 4294967296 | 32 | 32 |  |
| duration_ms | u32 | 4294967296 | 32 | 32 |  |
| fine_mask | 13 independent bits | 8192 | 13 | 13 |  |
| coarse_hits | 0..5 | 6 | 3 | 3 |  |
| attempt | 1..2 | 2 | 1 | 1 |  |

Bits per element: used 81, minimum 81. Layout: one array. Bytes: stored 528; least at used widths 486; least at minimum widths 486 (each element alone in whole bytes: 528). ELEMENT_ALIGNED: 42 B. Each row starts on a byte, so 7 zero bits per row. Accepted by the operator: whole rows give O(1) indexed access under the lock.

### Retained INFO

Source: `app_program/subsystem1/telemetry_pack.h`. 1 element(s) of 104 B (832 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| reason[48] | text bytes | 256 | 8 | 8 |  |
| wake_lost | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| overruns | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| response_max_us | u32 | 4294967296 | 32 | 32 |  |
| gap_min_us | u32 | 4294967296 | 32 | 32 |  |
| gap_max_us | u32 | 4294967296 | 32 | 32 |  |
| sampled | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| missed | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| dropped | u32 (wrapping counter) | 4294967296 | 32 | 32 |  |
| isr_max_us | u32 | 4294967296 | 32 | 32 |  |
| line_max_us | u32 | 4294967296 | 32 | 32 |  |
| counts[2] | int32 | 4294967296 | 32 | 32 |  |
| floor[2] | ADC 0..4095 | 4096 | 12 | 12 |  |
| line[2] | ADC 0..4095 | 4096 | 12 | 12 |  |
| raw_bc | ADC 0..4095 | 4096 | 12 | 12 |  |
| faults | 4 independent bits | 16 | 4 | 4 |  |

Bits per element: used 832, minimum 832. Layout: separate allocations. Bytes: stored 104; least at used widths 104; least at minimum widths 104 (each element alone in whole bytes: 104).

### Health payload

Source: `app_program/subsystem1/network_pack.h`. 1 element(s) of 104 B (832 bits, 23 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| uptime_ms | u32 | 4294967296 | 32 | 32 |  |
| connect_attempts | u32 | 4294967296 | 32 | 32 |  |
| telemetry_lost | u32 | 4294967296 | 32 | 32 |  |
| event_lost | u32 | 4294967296 | 32 | 32 |  |
| command_lost | u32 | 4294967296 | 32 | 32 |  |
| sonar_history_missed | u32 | 4294967296 | 32 | 32 |  |
| vehicle_overruns | u32 | 4294967296 | 32 | 32 |  |
| motion_late | u32 | 4294967296 | 32 | 32 |  |
| motion_missed | u32 | 4294967296 | 32 | 32 |  |
| mqtt_hook_max_us | u32 | 4294967296 | 32 | 32 |  |
| publish_ok | u32 | 4294967296 | 32 | 32 |  |
| publish_failed | u32 | 4294967296 | 32 | 32 |  |
| line_busy | u32 | 4294967296 | 32 | 32 |  |
| sonar_busy | u32 | 4294967296 | 32 | 32 |  |
| motion_busy | u32 | 4294967296 | 32 | 32 |  |
| vehicle_busy | u32 | 4294967296 | 32 | 32 |  |
| health_busy | u32 | 4294967296 | 32 | 32 |  |
| latest_sonar_history_sequence | u32 | 4294967296 | 32 | 32 |  |
| wifi_poll_gap_max_us | u32 | 4294967296 | 32 | 32 |  |
| wifi_poll_gap_samples | u32 | 4294967296 | 32 | 32 |  |
| boot_id | u32 | 4294967296 | 32 | 32 |  |
| board_unavailable | u32 | 4294967296 | 32 | 32 |  |
| wifi_start_status | int32 status | 4294967296 | 32 | 32 |  |
| stack_min_free | u16 bytes | 65536 | 16 | 16 |  |
| health_age_ms | u16 ms | 65536 | 16 | 16 |  |
| cpu_load_permille | 0..1000 or UNKNOWN | 1002 | 10 | 10 |  |
| flags | 7 independent bits | 128 | 7 | 7 |  |
| reset_reason | 0..2 | 3 | 2 | 2 |  |
| wifi_link_status | -3..3 | 7 | 3 | 3 |  |
| mqtt_state | 0..3 | 4 | 2 | 2 |  |
| wifi_requested_priority | 0..32 | 33 | 6 | 6 |  |
| wifi_observed_priority | 0..32 | 33 | 6 | 6 |  |
| board_unavailable_mask | 5 independent bits | 32 | 5 | 5 |  |

Bits per element: used 809, minimum 809. Layout: separate allocations. Bytes: stored 104; least at used widths 104; least at minimum widths 104 (each element alone in whole words: 104). The Health board also holds a u32 time, the fault snapshot below and a u32 phase (132 B with its sequence).

### S1 pending sonar copy

Source: `app_program/subsystem1/network_pack.h`. 1 element(s) of 32 B (256 bits, 2 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| history_sequence | u32 | 4294967296 | 32 | 32 |  |
| generation | u32 | 4294967296 | 32 | 32 |  |
| request_id | u32 | 4294967296 | 32 | 32 |  |
| trigger_ms | u32 | 4294967296 | 32 | 32 |  |
| timestamp_ms | u32 | 4294967296 | 32 | 32 |  |
| ping_sequence | u32 | 4294967296 | 32 | 32 |  |
| boot_id | u32 | 4294967296 | 32 | 32 |  |
| status | 0..3 | 4 | 2 | 2 |  |
| valid | bool | 2 | 1 | 1 |  |
| pending | bool | 2 | 1 | 1 |  |
| angle | -30..30 | 61 | 6 | 6 |  |
| ping_angle | -30..30 | 61 | 6 | 6 |  |
| echo | ultrasonic word, 14 bits | 16384 | 14 | 14 |  |

Bits per element: used 254, minimum 254. Layout: separate allocations. Bytes: stored 32; least at used widths 32; least at minimum widths 32 (each element alone in whole words: 32).

### F owner word: Vehicle

Source: `app_program/platform/fault_mask.h`. 1 element(s) of 4 B (32 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| owner word | bits 4, 6 | 4 | 2 | 32 | WORD_SLACK |

Bits per element: used 32, minimum 2. Layout: separate allocations. Bytes: stored 4; least at used widths 4; least at minimum widths 4 (each element alone in whole words: 4). One writer stores the whole word atomically; owners cannot share a word without a shared read-modify-write.

### F owner word: Sonar

Source: `app_program/platform/fault_mask.h`. 1 element(s) of 4 B (32 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| owner word | bit 5 | 2 | 1 | 32 | WORD_SLACK |

Bits per element: used 32, minimum 1. Layout: separate allocations. Bytes: stored 4; least at used widths 4; least at minimum widths 4 (each element alone in whole words: 4). One writer stores the whole word atomically; owners cannot share a word without a shared read-modify-write.

### F owner word: Telemetry

Source: `app_program/platform/fault_mask.h`. 1 element(s) of 4 B (32 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| owner word | bit 7 | 2 | 1 | 32 | WORD_SLACK |

Bits per element: used 32, minimum 1. Layout: separate allocations. Bytes: stored 4; least at used widths 4; least at minimum widths 4 (each element alone in whole words: 4). One writer stores the whole word atomically; owners cannot share a word without a shared read-modify-write.

### F owner word: Wi-Fi

Source: `app_program/platform/fault_mask.h`. 1 element(s) of 4 B (32 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| owner word | bits 8, 9 | 4 | 2 | 32 | WORD_SLACK |

Bits per element: used 32, minimum 2. Layout: separate allocations. Bytes: stored 4; least at used widths 4; least at minimum widths 4 (each element alone in whole words: 4). One writer stores the whole word atomically; owners cannot share a word without a shared read-modify-write.

### F Health fault snapshot

Source: `app_program/platform/fault_mask.h`. 2 element(s) of 16 B (128 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| now | 10 fault bits | 1024 | 10 | 32 | GAP |
| run | 10 fault bits | 1024 | 10 | 32 | GAP |
| valid | 10 fault bits | 1024 | 10 | 32 | GAP |
| epoch | u32 run epoch | 4294967296 | 32 | 32 |  |

Bits per element: used 128, minimum 62. Layout: separate allocations. Bytes: stored 32; least at used widths 32; least at minimum widths 16 (each element alone in whole words: 16). Two copies: in the Health board and in the Health working views. The producer limits the masks (fault_mask_combine); the store does not.

### F STOP capture

Source: `app_program/subsystem1/robot_health.c`. 1 element(s) of 32 B (256 bits, 0 spare).

| Field | Valid range | n | Min bits | Bits used | Reason |
| --- | --- | ---: | ---: | ---: | --- |
| owner Motion | bits 0..3 | 16 | 4 | 32 | GAP |
| owner Vehicle | bits 4, 6 | 4 | 2 | 32 | GAP |
| owner Sonar | bit 5 | 2 | 1 | 32 | GAP |
| owner Telemetry | bit 7 | 2 | 1 | 32 | GAP |
| owner Wi-Fi | bits 8, 9 | 4 | 2 | 32 | GAP |
| valid | 10 fault bits | 1024 | 10 | 32 | GAP |
| epoch | u32 run epoch | 4294967296 | 32 | 32 |  |
| us | u32 (wrapping time) | 4294967296 | 32 | 32 |  |

Bits per element: used 256, minimum 84. Layout: separate allocations. Bytes: stored 32; least at used widths 32; least at minimum widths 12 (each element alone in whole words: 12). Plain u32 words (36 B board with its sequence). The producer limits each owner word (capture_owner_values).

### Report-only gaps

| Record | Fields (GAP) | Stored B | Least B | Saving B |
| --- | --- | ---: | ---: | ---: |
| KR1 RAM-log row | echo, flags | 72016 | 69203 | 2813 |
| F Health fault snapshot | now, run, valid | 32 | 16 | 16 |
| F STOP capture | owner Motion, owner Vehicle, owner Sonar, owner Telemetry, owner Wi-Fi, valid | 32 | 12 | 20 |

### Accepted with a reason

| Record | Reason | Fields | Bytes not saved |
| --- | --- | --- | ---: |
| Sonar history | SMALL_GAIN | echo | 4 |
| Telemetry events | ELEMENT_ALIGNED | all (6 zero bits per element) | 384 |
| Stopped scan rows | ELEMENT_ALIGNED | all (7 zero bits per element) | 42 |

<!-- min-bit:end -->

## Gaps

The `GAP` rows above are a report only.

| Record | Why the gap stays now |
| --- | --- |
| KR1 RAM-log row | Echo (2 bits) and flags (3 bits) are wider than needed. A whole row stays 16 B at 123 bits; only one bit stream saves 2813 B. 15 B rows need the time to drop 3 bits too, with an index-anchored decode. That change is on `mtk3-cr1-kr1row`. |
| F Health fault snapshot, F STOP capture | The fault words are plain u32 words. Each holds at most 10 meaningful bits. The producers mask the values; the store does not check them. Packing saves 16 B and 20 B (36 B). A fix is on a separate branch. |

Accepted by the operator, with the written reason:

| Record | Reason | Bytes not saved | Why |
| --- | --- | ---: | --- |
| Sonar history | `SMALL_GAIN` | 4 | Echo keeps the 16-bit native word. Only one bit stream over all 16 entries can use the 2 free bits; that needs a new history layout for 4 B. |
| Telemetry events | `ELEMENT_ALIGNED` | 384 | Every field is minimal. Each event starts on a byte (6 zero bits), so the code gets event i directly under the lock. |
| Stopped scan rows | `ELEMENT_ALIGNED` | 42 | Every field is minimal. Each row starts on a byte (7 zero bits), so the code gets row i directly under the lock. |

## Not packed or out of scope

| Item | Reason |
| --- | --- |
| Line path record (`lp_t`), line board and line queue | Locked line path. CR1 does not change it. |
| IMU records | Owned by the IMU worker. The IMU table is added here when the IMU packing is merged. |
| Public native types, MQTT wire formats, queue messages | Category B is not approved. No wire change without an operator decision. |
| `boot_id`, detour `duration_ms`, `dr`/`dt` configuration copies, `latest_debug` | Category B is not approved; kept as they are. |
| Pulse code in the motion record | Copy of the servo angle (category B5, not approved). It is minimal for its own 73 values. |
| Barcode rejected-ADC rows (raw16/flags16) | Kept intact as raw evidence. |
| Task stacks, kernel, USB, CYW43 and lwIP RAM | Not application data records. |
| Event `word`/`detail` and Health stack/age values | The pack accepts the full u16/u8 range, so the proof uses that range. The producer range was not proved smaller. |

## IMU

Placeholder. The IMU table is added when the IMU packing is merged.

## How to update

When a pack layout changes, change `min_bit_records.inc` first (red),
then the code. Run `test_min_bit_proof --markdown` and paste its output
between the markers. `test_min_bit_doc` fails until the two agree.

## Evidence

Date: 2026-11-19. Base `b68ce6b`; proof commits `c9218ef`, `4551dc6` and the reason update after `4551dc6`.

| Check | Result |
| --- | --- |
| Host tests | 700/700, including `test_min_bit_proof` and `test_min_bit_doc` |
| Clang ASan/UBSan run of `test_min_bit_proof` | Pass; the same `--markdown` output as GCC |
| Firmware identity (demo MQTT image from `git archive`) | 225/225 objects the same as `b68ce6b` after debug strip; UF2 SHA-256 prefix `83f9192a09038d8f` in both |
| Demo image | Static RAM 239304 B; text 468260 B; RAM_PROOF PASS 79/79, 21212 B; 0 warnings |

Red checks. Each change below made `test_min_bit_proof` fail. Each
change was then removed.

| Change | Failure message |
| --- | --- |
| Motion duty slot 11 → 12 bits, no reason | width 12 min 11 reason 0 |
| Motion duty range to 900 | 801 rejected |
| Motion settled offset −1 | overlaps |
| Sonar echo range to 0xffff with `WORD_SLACK` | stale reason |
| Sonar board echo `GAP` | GAP but 0 B can be saved |
| Sonar history echo `WORD_SLACK` | WORD_SLACK but 4 B can be saved |
| KR1 flags `WORD_SLACK` | WORD_SLACK but 2813 B can be saved |
| Events `ELEMENT_ALIGNED` figure 384 → 383, or removed | element alignment 384 B, stated 383 B / 0 B |
| Scan rows `ELEMENT_ALIGNED` removed | element alignment 42 B, stated 0 B |
| Motion record states 4 B alignment | element alignment 0 B, stated 4 B |
| Sonar history `SMALL_GAIN` figure 4 → 3 | SMALL_GAIN 3 B stated, 4 B saved |
| Sonar board echo `SMALL_GAIN` | SMALL_GAIN 4 B stated, 0 B saved |
| KR1 flags `SMALL_GAIN` beside echo `GAP` | SMALL_GAIN stated, 2813 B saved |
| KR1 time range 0..8191 | width 16 min 13 reason 0 |
| KR1 total +1 B | totals differ |
| Barcode frame boards as one array | totals differ |
| Barcode reason slot 5 → 6 bits | width 6 min 5 reason 0 |
| Telemetry index refused value set to a valid value | pack accepted it |
| S1 sonar copy echo range to 0x7fff | width 14 min 15 |
| Motion fault mask 0x7 | width 4 min 3 |
| `vehicle_pack.c` state shift 7 → 8 (firmware, removed) | round trip 1 → 2 |
