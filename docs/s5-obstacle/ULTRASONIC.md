# Current ultrasonic driver and service

## What it is

The HC-SR04+ supplies bounded range observations, not a clear-path decision.
The kernel robot uses `drv_ultrasonic_*` through the priority-12 sonar
service. The older `src/ultrasonic.c` driver remains for bare-metal benches
and simulation; do not link both driver owners into the robot.

## How it works

| Signal | Connection / owner |
| --- | --- |
| Echo | GP2, both edges on shared core-0 GPIO13 dispatcher, IRQ level 1 |
| Trigger | GP3, 10 us HIGH pulse |
| Supply | Recorded HC-SR04+ setup at 3.3 V; no divider in this setup |
| Servo | Motion-owned PWM; sonar does not move it or add settle delays |

The task checks for Echo already HIGH, arms a measurement and sends the
short trigger. A rising edge records time; a falling edge records width
and wakes the service. The task converts/publishes the result later.
Missing edges use deadlines, not a blocking echo wait. The rise ISR also
rejects an edge at or after the 2 ms deadline, even if service is late.

WATCH requests require result acknowledgement before another trigger.
ONCE requests are used for scan points. Requests carry generation, ID,
angle, not-before time and deadline. Old physical echoes may finish after
cancellation, but cannot become a new generation's result. A real trigger
acknowledgement is distinct from successful queue insertion.

## Interface

| API | Contract |
| --- | --- |
| `drv_ultrasonic_trigger/service/busy/latest` | Core-0 task API; only the sonar owner calls trigger/service |
| `drv_ultrasonic_listen()` | Echo-edge task wake |
| `sonar_request()` | Bounded nonblocking request copy; a full queue is an error |
| Service result | Inherited-mutex copy plus event flag; validate identity/age before consumption |
| Public latest/history reads | Non-consuming observers; must not change the service state machine |

Sources: [`drv_ultrasonic.h`](../../app_program/subsystem5/drv_ultrasonic.h),
[`drv_ultrasonic.c`](../../app_program/subsystem5/drv_ultrasonic.c),
[`sonar_service.c`](../../app_program/subsystem5/sonar_service.c) and
[`ultrasonic.h`](../../app_program/subsystem5/ultrasonic.h).

### Packed result

The result is one 16-bit word with explicit masks/shifts, not C bit-fields.

| Bits | Field | Values |
| --- | --- | --- |
| 0–8 | Distance | VALID: 60–511 mm; 0 = TOO_NEAR; 1–59 unused; non-VALID distance has no meaning |
| 9–10 | Status | VALID 0, NO_RISE 1, STUCK_HIGH 2, NO_RETURN 3 |
| 11–13 | Consecutive fault streak | 0–5, saturates at five |
| 14–15 | Reserved | Zero |

NO_RISE means no rise within 2 ms. STUCK_HIGH means Echo was HIGH before
the trigger. They increment the streak. VALID and NO_RETURN reset it.
NO_RETURN means the sensor answered but no echo returned within range;
it is not a fault and is never proof of clear space. The monitor's fault
latch is separate from this driver streak.

Initial word 0x0200 is NO_RISE with streak zero: no result yet, not a real
failed measurement. A VALID echo below 60 mm becomes code zero; the blind
zone can still give a wrong VALID value. Use status before distance.
Conversion is `(pulse_us * 343 + 1000) / 2000`, rounded mm, assuming about
20°C. Timer/pulse arithmetic remains 32-bit before bounded publication.

## Settings

| Setting | Value / rule |
| --- | --- |
| Trigger pulse | 10 us |
| Rise / echo timeout | 2 / 3 ms |
| Minimum trigger gap | 60 ms |
| Typical forward WATCH interval | 100 ms |
| WATCH age bound | Actual trigger + twice the interval + 20 ms |
| SCAN one-shot deadline | 100 ms; includes trigger-gap wait |
| No-edge deadline wake | 10 ms cyclic; edge timestamps still define the range |
| Published range | 60–511 mm; farther echo becomes NO_RETURN |

A 511 mm echo takes about 2.98 ms; the 3 ms timeout corresponds to about
515 mm, but the converted 511 mm field limit still applies. Do not widen
the wire range by treating timeout distance as usable range.

## Results

| Evidence | Result and scope |
| --- | --- |
| Bench rise latency | 212–217 us, stable |
| Echo quantization | About 25 us, one 40 kHz cycle: approximately 4.3 mm, despite 1 mm field units |
| Four-status open-space check | 50/50 NO_RETURN, streak zero; rise 212–216 us |
| Trigger disconnection | Fifth miss gives fault streak five in about 0.5 s at 100 ms period; reconnect clears streak on valid/no-return |
| Hand-held 100–400 mm targets | Standard deviation 1.6–5.2 mm; errors not proportional to distance, so no scale correction applied |
| 20 mm near-target check | Mostly wrong VALID 40–55 mm, sometimes farther; lowest 40 mm; sensor cannot flag its blind-zone error |
| Front-contact offset | 80/80 VALID, mean 57.9 mm, standard deviation 2.2 mm; about 58 mm echo offset, ruler confirmation still needed |
| Kernel S5-U01, 3f889a1 | Fixed wall: 20/20 valid at 200 mm, medians 193/196 (188–204); at 400 mm medians 394/393 (390–407) |
| Kernel S5-U02 | Old-generation request rejected; request/result use exercised in integration |

## Limits and links

Fabric, slanted faces, narrow objects and open space can all give NO_RETURN.
The beam shape is not calibrated. Repeat fixed-distance/material/angle
checks before applying an offset or scale correction. This setup's 3.3 V
wiring is not approval to connect an arbitrary 5 V Echo output to RP2040.
No source/API test establishes actual stop latency or all surfaces.

- [Obstacle monitor/profile rules](OBSTACLE.md).
- [S5 TEST_LOG](../../testing/subsystem5/TEST_LOG.md).
- [Integration TEST_LOG and hardware tables](../../testing/integration/KERNEL_ROBOT.md).
- [Complete original driver and all seven bench runs](../archive/s5-obstacle/ULTRASONIC_DRIVER.md) retain every plateau, count, pulse and unfilled material/angle check.
- [CR1 notes](../architecture/week8_change_requirement_1_notes.md) remain historical wording; the packed result above is the current implementation.
