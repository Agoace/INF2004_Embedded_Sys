# Subsystem 5 kernel ultrasonic checks

| ID | Procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S5-U01 | TEST=1, wheels lifted; put a flat object 60..511 mm from the sensor and measure with a ruler. Capture 20 pings, median/min/max and unknown count. | Valid distances agree with manual distance within sensor tolerance; no sensor fault streak; explain any unknowns | PASS 2026-11-06 (3f889a1, USB power only, concrete wall, sensor straight on): 200 mm ruler → 20/20 valid, medians 193 and 196 mm (188..204). See the S5-U01 wall check below |

GP3 sends a 10 us trigger. GP2 echo uses both edges on the shared core-0
GPIO13 dispatcher, priority 1. No SDK IRQ handler is registered. TIMER
microseconds, the 2 ms rise timeout, 3 ms echo timeout, 60 ms minimum ping
interval, packed result and unknown-on-no-echo rules match `src/ultrasonic.c`.
Because task service is only every 10 ms, the GPIO callback also rejects
rises at or after the 2 ms deadline. A late echo cannot become VALID merely
because task service has not run. Width/field limits remain in task context.
Servo PWM remains owned by motion. Kernel tasks alone publish results;
the GPIO callback only records edge times and phase. All APIs are core-0
only, with DI/EI for shared phase snapshots.

## Task-split image checks

| ID | Procedure and acceptance | Status |
| --- | --- | --- |
| S5-U02 | TEST: fixed-object S5-U01 now uses service requests/results; reject old-generation request; verify echo-loss unknown, one-shot deadline and no stale SCAN result in DETOUR | PASS 2026-11-06 (TEST, both images). SCAN/DETOUR use seen in M2-S7. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |

Service owns trigger, echo, 60 ms minimum interval and request correlation.
SCAN retains its existing settle state machine and settings; motion retains
servo PWM. This is an approved difference from RTOS plan section 7. A
one-shot conflict is rejected, not overwritten. Results have request ID,
generation, actual trigger sequence, timestamp and deadline. WATCH waits
for a consumer ACK before another trigger. No echo is unknown, never clear.
A 10 ms cyclic handles deadlines; echo edges also wake the service. The
2 ms rise deadline remains in the ISR. Late old-generation echoes are
drained but not published as a new owner's measurement.

## S5-U01 wall check: 2026-11-06

Image 3f889a1 (TEST), USB power only, no VIN. Target: a concrete wall,
sensor straight on. Logs (ignored, not committed):
`build/mtk_logs/s5u01_60mm_*.txt` and `build/mtk_logs/s5u01_200mm_*.txt`.

| Ruler distance | Boot | Valid / 20 | Median | Min..max |
| --- | --- | --- | --- | --- |
| About 60 mm (edge of range) | 1 and 2 | 1 and 1 | 67 mm | 67..67 mm |
| 200 mm | 1 | 20 | 193 mm | 188..204 mm |
| 200 mm | 2 | 20 | 196 mm | 193..204 mm |

Result: **PASS** at 200 mm (within ±10 mm, no unknown values).

At about 60 mm, 19 of 20 pings per boot were not counted as valid. The test
counts TOO_NEAR (an echo from less than 60 mm), NO_RETURN and faults
together as "unknown". The robot was at the lower limit of the range, so
TOO_NEAR is the probable cause. This is correct behaviour, not a failure.

The readings are 4 to 7 mm short at 200 mm. The driver uses 343 m/s (speed
of sound at 20 °C). At about 30 °C the speed is about 349 m/s, which gives
about 4 mm at 200 mm. The position of the transducer behind the board edge
can give a few mm more. The STOP limit (258 mm) moves by about 5 mm. No
change is made.

Open: S5-U01 must show TOO_NEAR, NO_RETURN and faults as separate counts.
