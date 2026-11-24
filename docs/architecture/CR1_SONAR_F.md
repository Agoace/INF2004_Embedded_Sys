# CR1 Sonar F: sonar owner fault word

## Scope and decision

Operator approved the diagnostic fault mask F on 2026-11-18. The shared
interface is [`fault_mask.h`](../../app_program/platform/fault_mask.h)
(SW8 F1, `1809787`). This record covers only the sonar owner word:
`FAULT_OWNER_SONAR` = bit 5, `FAULT_SONAR_SENSOR`. Health alone combines
the owner words. The word is a diagnostic mirror. It is not an actuator
input and it adds no latch or clear rule.

Commit: `4e953a6`, kept at tag `archive/mtk3-cr1-sonar` (parent `1809787`).

## Rule

| Item | Rule |
| --- | --- |
| Storage | `static fault_mask_board_t fault_board` in `sonar_service.c`: 8 B (sequence and one word) |
| Single writer | The sonar task, in `publish()`, after the sonar mutex is released |
| When | Each ECHO result: the whole word is written |
| Bit 5 set | `ultrasonic_result_fault(echo)`: the existing driver streak is at least `ULTRASONIC_FAULT_STREAK` (5) |
| Bit 5 clear | Any other ECHO result. NO_RETURN and VALID reset the driver streak, so they clear the bit |
| DEADLINE, CANCELLED | No new reading: the word does not change |
| Before the first ECHO | Not published: the reader returns false (unavailable, not all-clear) |
| Reader | `sonar_fault_mask_try_read()`: bounded; false with output unchanged on a torn or absent copy |

The word keeps its last value while the sonar is stopped. It mirrors the
last real reading, not the current sensor health.

## Tests

`test_sonar_fault` (red first: it did not build before the publisher
existed):

- All 65536 echo words: bit 5 is set exactly when the streak is at least 5.
- Driver-like sequence: NO_RISE streak 1 to 4 gives 0, streak 5 sets the
  bit; DEADLINE and CANCELLED keep it; NO_RETURN clears it; STUCK_HIGH
  streak 5 sets it; VALID clears it.
- CANCELLED before the first ECHO does not publish.
- A read torn by a writer returns false and leaves the output unchanged.
- One writer: one `fault_mask_publish(` call site on one file-local board.
- Mutation check: publishing on every status fails the test.

Host tests: 686/686 (685 at `1809787`, plus this test), 0 warnings.

## Build cost

Demo line, public example secrets, Arm GNU 14.3.Rel1. Before is `1809787`.

| Quantity | Before | After | Change |
| --- | ---: | ---: | ---: |
| Static RAM, map parser | 240808 B | 240816 B | +8 B |
| Main RAM not statically reserved | 29528 B | 29520 B | -8 B |
| subsystem5 linked inputs | 945 B | 953 B | +8 B |
| ELF text (`size`) | 458196 B | 458364 B | +168 B |
| RAM functions | 79 / 21212 B | 79 / 21212 B | 0 |
| Sonar `publish` compiler-local frame | 32 B | 40 B | +8 B |

The 8 B board is new storage, a cost, not a packing saving. Of 225
objects, only `sonar_service.o` differs. Messages and MQTT bytes do not
change. The frame value comes from the compiler's per-function stack report.
Open: whole-stack measurement for the isolated substep; no robot run was
made. Later combined CR1 image 12 checked fault-mask
output, timing and stacks; see [project status](PROJECT_STATUS.md#areas).
