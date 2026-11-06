# Subsystem 5 kernel ultrasonic checks

| ID | Procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S5-U01 | TEST=1, wheels lifted; put a flat object 60..511 mm from the sensor and measure with a ruler. Capture 20 pings, median/min/max and unknown count. | Valid distances agree with manual distance within sensor tolerance; no sensor fault streak; explain any unknowns | Not run |

GP3 sends a 10 us trigger. GP2 echo uses both edges on the shared core-0
GPIO13 dispatcher, priority 1. No SDK IRQ handler is registered. TIMER
microseconds, the 2 ms rise timeout, 3 ms echo timeout, 60 ms minimum ping
interval, packed result and unknown-on-no-echo rules match `src/ultrasonic.c`.
Servo PWM remains owned by motion. Kernel tasks alone publish results;
the GPIO callback only records edge times and phase. All APIs are core-0
only, with DI/EI for shared phase snapshots.
