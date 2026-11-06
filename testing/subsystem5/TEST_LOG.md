# Subsystem 5 kernel ultrasonic checks

| ID | Procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S5-U01 | TEST=1, wheels lifted; put a flat object 60..511 mm from the sensor and measure with a ruler. Capture 20 pings, median/min/max and unknown count. | Valid distances agree with manual distance within sensor tolerance; no sensor fault streak; explain any unknowns | Partial 2026-11-06 (445084c): 20/20 valid, median 209–210 mm (object, not checked with a ruler). 3f889a1: no fixed object. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |

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
