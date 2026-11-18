# CR1 Vehicle A1: compact observation board

## Scope and decision

Operator approved this substep on 2026-11-18. Replace only the stored
Vehicle observation payload. Keep `vehicle_snapshot_t`, its reader API,
the native writer copy, all messages and all MQTT bytes unchanged.
Do not change the controller, junction action, live action, SEARCH grace,
1000 mm expiry, detour handover, fault policy, task periods or stacks.
Private `vsm_t`, `ja_result_t`, `ja_t` and `jlive_t` packing is not approved.

This is a partial CR1 result, not completion of Week 8 sections 33–38.
The historical 13-state set needs 4 bits. The current shell has three
states and a separate five-value module ID. Keep that distinction.
The shell fault is one terminal cause. It is not an independent-fault mask.
Motion fault bits and IMU safety conditions are unchanged.

## Stored representation

Sources: [`vehicle_pack.h`](../../app_program/vehicle/vehicle_pack.h),
[`vehicle_pack.c`](../../app_program/vehicle/vehicle_pack.c),
[`vehicle_board.h`](../../app_program/vehicle/vehicle_board.h).
Each row has one writer: the Vehicle task. Access is through the existing
aligned atomic-word observation board. Readers decode a successful local
copy only.

| Stored field | Valid range | Bits | Location |
| --- | --- | ---: | --- |
| timestamp_us, sequence, wake_lost, overruns, response_max_us, handoffs, command_id | Full u32, including wrap | 32 each | Words 0–6 |
| left_mm_s, right_mm_s | -32768..32767 each | 16 each | Word 7, low/high half |
| servo_deg | -36..36, code is angle + 36 | 7 | Word 8, bits 0–6 |
| state | IDLE 0, RUNNING 1, FAULT 2 | 2 | Bits 7–8 |
| module | NONE 0 through DETOUR 4 | 3 | Bits 9–11 |
| phase | Idle 0, calibration 1, run 2, halt 3 | 2 | Bits 12–13 |
| command_opcode | Initial 0; accepted queue values 1..4 | 3 | Bits 14–16 |
| command_status | 0..5, including duplicate 4 | 3 | Bits 17–19 |
| active, calibrated | 0 or 1 each | 1 each | Bits 20, 21 |
| Unused storage | Always zero | 10 | Bits 22–31 |

There are 278 logical bits, stored in nine aligned words (36 B).
The sequence guard adds 4 B. All retained fields have separate storage;
no derived field or duplicate copy is removed.

The current servo API returns an accepted angle within -36..36, from the
1100..1900 us robot limits and 111/10 us per degree mapping. This is not
an assumed scan-only angle range. `network_queue.c` rejects opcodes
outside 1..4 before the Vehicle receives them. The initial opcode is zero.
`vehicle_remote_policy` and `handle_command` produce result values 0..5.
Full signed-16 speeds and full clock/counter widths remain unchanged.

Both codec functions reject null pointers, invalid codes and nonzero
unused bits without changing their output. An invalid publication is not
published. A failed board copy or failed decode returns `E_TMOUT` and
leaves the reader's output unchanged. Valid robot values retain the
previous decoded result exactly.

## Before and after: build measurements

The only before baseline is SW7's `7350da0`, not the older `168d705`
tables in [CR1.md](CR1.md). Source: local
`~/Projects/cr1-sw7-sonar/build/wip/cr1/BASELINE_7350da0.md`,
`mod_vehicle.txt` and `message_sizes.txt`. No second baseline image was
built. These are build measurements, not robot measurements.

Compiler: ARM GNU 14.3.Rel1, GCC 14.3.1, Cortex-M0+ Thumb, `-O2`.
Both builds use public example secrets and this demo profile:

```sh
make -C build_make --output-sync=target -j12 \
  SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 \
  BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
```

| Measurement | Before | After | Delta |
| --- | ---: | ---: | ---: |
| Static RAM, map parser | 241228 B | 241224 B | -4 B |
| Main RAM not statically reserved | 29108 B | 29112 B | +4 B |
| Vehicle linked inputs | 2516 B | 2512 B | -4 B |
| Vehicle board, guard included | 44 B | 40 B | -4 B |
| Stored observation payload | 40 B | 36 B | -4 B |
| Public decoded snapshot | 40 B | 40 B | 0 |
| ELF text (`size`) | 454596 B | 455012 B | +416 B |
| ELF bss (`size`, includes RAM vector) | 219060 B | 219056 B | -4 B |
| RAM functions | 79 / 21004 B | 79 / 21004 B | 0 |
| Vehicle stack allocation | 4096 B | 4096 B | 0 |
| Telemetry queue message | 116 B | 116 B | 0 |
| Network event / command message | 20 / 8 B | 20 / 8 B | 0 |

Category A saving is **4 B**, once. Category B saving is **0 B**.
Do not add the board saving to the module or static saving: they are the
same four bytes. Unreserved RAM is not measured runtime free RAM.
All other module groups and the 927 B output alignment total are unchanged.
The complete ARM message manifest differs only by the new 36 B packed
type and the Vehicle board expression (44 to 40 B). No wire format changed.

Local evidence in this worktree, under ignored `build/wip/`:
`demo-build.log`, `demo-relink.log`, `ram-after.txt`, `vehicle-after.txt`,
`message-after.txt`, and `demo/` (ELF, map, UF2).
Public-example UF2 SHA-256:
`e10f086710f707a40bbe0dce8ca0590897c4d00bce8cac6172036d506cd08c05`.
This is not an operator-secret image identity.

## Software checks

- GCC strict host suite: **680/680**, the existing 677 plus three codec
  cases (ranges, invalid input, board interleaving).
- Clang ASan/UBSan with leak checks and halt-on-UB: **680/680**.
- The red check used a false-return codec stub. The range test failed at
  `vehicle_pack`. Final codec cases pass. The stub and logs remain local
  (`build/wip/pack-red.log`); the stub is not firmware source.
- Exhaustive round trips cover each signed-16 speed, all 73 servo angles,
  and all combinations of state, module, phase, opcode, status and flags.
  Clock/counter tests include zero, one, signed endpoints and u32 wrap.
- Rejection tests cover every unused code for each bounded field, invalid
  native values, all ten unused storage bits and null pointers. Failed
  operations keep the output unchanged.
- A new complete publication is injected after each of the nine payload
  loads. The reader rejects every torn copy. Its decoded cache stays
  unchanged; the next bounded read returns the new value.
- Board tests retain odd, initial-zero and wrapping sequence behavior.
  At wrap, guard zero remains unavailable for one publication, as before.
- The complete existing suite retains junction, live action, SEARCH grace,
  1000 mm expiry, detour handover and network failure tests.
- Demo build: **0 warnings**, including case-insensitive assembler warning
  checks; **RAM_PROOF PASS 79/79**, 21004 B.
- Codec ARM compile passes `-Wall -Wextra -Wconversion -Wshadow
  -Wformat=2 -Werror`. No C bit-fields or unaligned packed structs are used.

## Stack and timing limits

Compiler-local frames, from `-fstack-usage`; these are **not whole-stack
bounds**. The Vehicle task probe uses the demo compile flags. The before
probe compiles SW7's unchanged source to a local temporary object only;
it does not rebuild or change the baseline image.

| Function | Before | After |
| --- | ---: | ---: |
| vehicle_pack | Not present | 56 B |
| vehicle_unpack | Not present | 56 B |
| publish_vehicle | 24 B | 64 B |
| vehicle_snapshot_try_read | 72 B | 64 B |

Evidence: `build/wip/vehicle_pack.su`, `vehicle_task_before.su`,
`vehicle_task_after.su`. A caller can also retain its frame while the codec
runs. Do not add or compare these local numbers as measured high-water.

The 10 ms task period is unchanged. Packing adds work at publication;
decoding adds work to readers. Copying one less board word does not prove
lower total execution time. No target work-time, response-time or stack
acceptance is claimed. The operator must agree a work-time regression
limit, then check loaded MQTT/USB, wakes, overruns and named stack margins
before hardware acceptance. No helper flash was performed.

## Shared access and deferred work

The Vehicle task remains the only board writer. It builds the payload
locally, then uses `app_board_publish`. Readers use one bounded
`app_board_try_read`, then decode locally. Atomic accesses, barriers and
sequence validation remain unchanged. No ISR writes this payload. A later
core-1 reader uses the same board contract. No shared packed read-modify-
write or local-interrupt-only lock is added.

Keep the separate calibration/run epoch atomic words. Do not combine a
Vehicle flag with a LINE, IMU or ISR-owned word. S1 owns any future shared
fault combination. SW8 owns wire/decoder changes; none are needed here.

The USB `VE` record and its type-specific 16-bit word stay unchanged;
`vc_event_t` and `vsm_outcome_t` also stay 16-bit. The transient row-event
`pending_bits` can include fault flags and is not a proposed four-bit
stored mode word. No historical mode-flag design is added to the running
controller. Further private-state packing or duplicate removal needs a
separate proposal and approval.
