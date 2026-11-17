# Current obstacle monitor, scan and avoidance

## What it is

The robot stops for a confirmed forward obstacle, scans, chooses a side,
and runs DETOUR. The five brief stages below describe the intended work
and the actual implementation separately. The SCAN module now uses a
coarse/fine schedule with legacy-bin completion. Geometry reporting is
separate from the unchanged avoidance decision.

The pure modules return requests to the Vehicle Controller shell. Motion
owns the servo and wheel outputs. The [ultrasonic service](#ultrasonic-driver-and-service)
owns trigger/echo timing; it does not choose the avoidance route.

## How it works

### Forward monitor thresholds

Distances are from the sensor face. The measured echo offset to the robot
front is about 58 mm, not a ruler-confirmed geometry correction.

| Rule | Current value |
| --- | --- |
| STOP | Two consecutive readings below 258 mm: nominal 200 mm front clearance + 58 mm |
| NEAR | Two below 400 mm |
| Release STOP | Two VALID readings at least 278 mm; then APPROACHING, not directly CLEAR |
| Release NEAR | Two readings at least 420 mm, or two NO_RETURN readings |
| Hysteresis | 20 mm; a dead-band read resets confirmation counts |
| Sensor fault | Five consecutive NO_RISE/STUCK_HIGH results latch SENSOR_FAULT |

TOO_NEAR is STOP evidence. NO_RETURN is unknown and never releases STOP.
A single fault below the streak limit does not change confirmation counts;
initial no-result word 0x0200 is ignored. `obstacle_init()` resets the
monitor. LINE resets it on start and acts on STOP/fault; it does not slow
to a separate NEAR speed. Planned HUMP_TRAVERSE suppression is not a
current robot feature.

### 1 — Coarse scan

The robot stops for 300 ms, then surveys −30, −15, 0, +15 and +30 degrees,
with two readings at each angle. Negative is right, positive is left.
The first angle settles for 150 ms. Between points, settling is 60 ms for
a 5-degree move, plus 15 ms per additional 5 degrees, capped at 150 ms.
The sonar service retains its minimum 60 ms trigger interval. Return to
centre settles for 150 ms. These settle values are scheduling assumptions,
not a new servo-speed measurement.

### 2 — Fine scan and decision-bin completion

After the coarse sweep, the nearest VALID coarse distance sets a frozen
hit threshold: that distance plus 150 mm, inclusive. Refine in 5-degree
steps from one coarse step outside the outermost hits on each side,
clipped to −30..+30. A coarse sweep without VALID hits has no fine region.
Completed coarse points are reused, never pinged again. Within the region,
the nearest unvisited index is selected first to limit return travel.

The unchanged DETOUR free-step calculation needs all 13 original bins.
After refinement, fill the remaining unvisited bins, nearest first. No
missing ray is interpolated or marked free. Each finished profile therefore
has 13 unique angles and 26 readings, including coarse samples. Geometry
resolution is 5 degrees; the wide sonar beam does not have 5-degree edge
accuracy. A narrow object missed by all coarse rays can still appear during
compatibility completion, but this is not a proven detection guarantee.

A 10 ms fake scheduler with 60 ms sonar spacing gives a worst duration of
2560 ms (fixed-order baseline 2230 ms under the same clock) including the
300 ms stop over all contiguous hit intervals and the
empty case. This host result is accepted for the first bench; it is not a
measured robot bound. Actual echo and navigation latency can add time.
Historical fixed-scan bench times were 1829/1833 ms excluding the initial
stop. The `d44bd25` floor run measured2519/2509 ms, including stopped wait
(2026-11-17). Retain settling and sonar spacing; this is not a worst-case
robot bound.

Host proof, stopped USB scan diagnostics, resource changes and the scoped
floor result (stationary bench replaced by operator decision) are in
[COARSE_FINE_VALIDATION.md](COARSE_FINE_VALIDATION.md).

### Mounted servo limits

| Quantity | Value / finding |
| --- | --- |
| Centre pulse | 1500 us; exact forward alignment still needs confirmation |
| Direction | Below centre turns right; above turns left |
| Touch points | About 1040/1960 us, about ±41° |
| Software clamp | 1100–1900 us, about ±36° |
| Scan endpoints | 1167/1833 us, ±30°; about 11° from touch points |
| Beam | Nominal full 30°; overlapping points cover roughly ±45°, not a measured fixed edge correction |

Pulse is `1500 + angle * 1000 / 90` us, rounded. The complete 13-pulse
table remains in `OBSTACLE_SCAN.md` in the [obstacle history](../archive/s5-obstacle/HISTORY.md#files).

### 3 — Profiling

Two VALID readings retain the nearer one. VALID plus unknown/failure keeps
the VALID value. With no VALID value, any NO_RETURN gives NO_RETURN;
two failures give no data. A fault streak of five fails the scan and
centres the servo.

### Geometry estimates

The v3 observation report adds estimated width and two observed lateral
spans. Closest distance and angle remain the sensor-frame location: forward
`x = r cos(theta)`, left `y = r sin(theta)`. This is not a world coordinate.
Geometry never selects a detour or changes the PROFILE.

The nearest VALID final distance sets `r_min`. Candidate hits are VALID,
positive distances at most `r_min + 150 mm`. Use the outermost candidates as
the edges. Width is the chord between these range/angle points, calculated
with the law of cosines and rounded to the nearest mm. One hit cannot
establish width. A zero-distance TOO_NEAR or absent closest sample makes
all geometry unknown. Disjoint hits can include empty space in the reported
region width; the sonar does not identify individual objects.

An edge at −30 or +30 degrees is clipped by the field of view. A calculated
width with either edge clipped is only a lower bound. Beyond each edge,
inspect the first adjacent outward ray. A VALID ray at least 150 mm farther
than the closest hit, or NO_RETURN, supports an observed span. Unknown and
error samples do not. A VALID ray must also reach the edge's forward plane.
At that plane the span is `abs(x_edge * tan(theta_free) - y_edge)`.
No outward ray means no estimate; clipped-edge clearance is unknown.
NO_RETURN supports an observed gap, not a measured far boundary.

Unknown dimensions use 65535 and explicit validity flags. The geometry
assumes one useful obstacle region, no motion during scan, and representative
sonar returns. Wide beams and sloping surfaces can give poor estimates.
These are not body-clearance or passability claims. Dashboard rendering is
separate. See the [v3 wire table](../s1-telemetry/MQTT_WIRE.md#obstacle-profile);
v1/v2 remain readable without geometry.

### Profile analysis and the PROFILE event

Obstacle distance is the nearest VALID point, or 511 if none is VALID.
For each side, search outward from centre. NO_RETURN is free for this
profile decision only; a VALID point is free if it is at least 150 mm
farther than the obstacle. No data is never free. The next outward point
must also be free; the outermost 30° point can be free alone.

| Profile choice | Rule |
| --- | --- |
| Shorter way | Smaller free step, 0–6 representing 0–30° |
| Tie | More room at 30°; full tie chooses LEFT |
| Both blocked | Step 7 on both sides; side NONE |
| PROFILE word | Distance bits 0–8, right free step 9–11, left 12–14, bit 15 zero |

The decision does not use estimated width or observed spans. Side selection
is not a proof of body clearance.

### 4 — Avoidance decision, reverse and reattempt

A free side starts DETOUR. NONE starts a bounded reverse and full rescan:

| Setting | Current value |
| --- | --- |
| Reverse | Both wheels −100 mm/s, servo 0 |
| Distance | 100 mm, mean wheel delta 1241 counts at 12414 counts/m |
| Timeout | 3 s; FAILED 2, summary stop reason 13 |
| After reverse | Stop/centre 300 ms, then the whole 13-point scan |
| Bound | At most two reverses, three complete profiles; attempts 0/1/2 |
| Final NONE | Passed to DETOUR, which fails with NO_SIDE |

No extra centre-only ping is added. The new sweep supplies DETOUR's nearest
distance. KR1 module 5 labels REVERSE for observers; the shell module is
still SCAN. Event kind 8 reports reattempt; this is not a new route.
ESTOP, motion faults and operator stop retain the normal safety path.

DETOUR keeps v1.8 logic: turn-away sweep, straight, turn-toward, fixed
back-off, return toward the line, then alignment. Main settings are:

| Setting | Current value |
| --- | --- |
| Straight / non-sweep turn | 131 mm/s; no heading hold on straights |
| Sweep | 40°/s, about 39 mm/s per wheel; ping every 60 ms, two seen readings |
| Motion-change pause / look | 250 / 150 ms |
| Back-off | Fixed 25° |
| Straight-length setting / sweep distance | 350 (2026-11-13, with the trim; was 400) / 150 mm |
| First straight | wall triangle: (sensor-ahead + obstacle + back) / cos(turn away) + 50 mm, at most 700 mm |
| Turn-away trim | back to the smallest heading that passes the last seen corner by 110 mm (half width 80 + margin 30) |
| Margin / sensor-ahead / line-ahead | 30 / 65 / 123 mm |
| Seen range / turn step | 400 mm / 5° |
| Away / toward limits | 90 / 180° (toward ≥ away + return heading) |
| Return heading / max distance | 90° / 1000 mm |
| Max straights / total time | 5 / 40 s |
| Encoder turn scale | 24.05 difference counts/degree |

The sweep was 20°/s until 2026-11-13. A pivot bench on the course mat
(3 s per step, BENCH image) measured the longest wheel stop per speed:

| Wheel speed | Longest stop | Duty (permille) |
| --- | --- | --- |
| 19 mm/s | 870 ms | 102-206 |
| 28 mm/s | 330 ms | 126-194 |
| 38 mm/s | 0 ms | 128-209 |

The stall rule stops after 500 ms at 150 permille or more with no counts.
At 19 mm/s the wheels stick and slip; detours stopped with
`MC_FAULT_STALL_LEFT`. The sonar now gets one ping per about 2.4° of sweep.

The toward limit was 150° until 2026-11-13. Two detours turned away 62° and
63°; the turn back then needed 152° and 153° to reach the return heading
(-90°) and failed with TURN_LIMIT. The limit is now away + return = 180°.

[Robot configuration](../../app_program/vehicle/vehicle_robot_config.h) is
the settings source. SCAN reattempt did not tune DETOUR or the profile rule.

### 5 — Line recovery

On DETOUR DONE, the module resets line tracking, waits one stopped step,
then returns DONE_LINE if the line is seen, otherwise DONE_NO_LINE with
search direction toward the obstacle. SEARCH turns at 131 mm/s through
316 counts each way, then fails if it finds no line. The shell routes
FOUND back to LINE. See [LINE](../s3-line-barcode/LINE.md) for detector and
search limits. A dark mark can still cause a false line decision.

## Interface

| API/data | Contract |
| --- | --- |
| `obstacle_update()` | New packed echo to event; event distance bits 0–8, event code 9–11, 12–15 zero |
| `ss_start/step()` | Pure scan state; servo request, ping acknowledgement, fresh echo and encoder movement |
| SIDE payload | Distance bits 0–8; side bits 9–10: NONE 0, RIGHT 1, LEFT 2 |
| `dr_start/step()` | Pure detour adapter; wheel speeds, servo request and shell outcomes |
| Sonar request | Generation/ID/angle, not-before and deadline; real trigger acknowledgement, not queue admission |

Sources: [`obstacle.c`](../../app_program/subsystem5/obstacle.c),
[`servo_scan.c`](../../app_program/subsystem5/servo_scan.c),
[`obstacle_profile.c`](../../app_program/subsystem5/obstacle_profile.c) and
[`detour_run.c`](../../app_program/subsystem5/detour_run.c).

## Results

| Evidence | Result and scope |
| --- | --- |
| Monitor bench, 2026-10-27 | STOP on approach to 106 mm; removal stays STOP/unknown until reset; some individual bench steps not recorded |
| Scan run 2 | 1833 ms; nearest 197 mm, free steps R5/L4, LEFT; 383 mm reading exercised the 150 mm gap rule |
| R3, 450 mm straight setting | Two DONE_LINE, no contact/fault; detours 23.2/22.5 s; not the retained 400 mm setting |
| R4, retained 400 mm | Two DONE, one DONE_LINE and one DONE_NO_LINE; SEARCH recovered in 0.25 s; no blocked wheel, faults zero; false mark remains a limit |
| M2-S7 kernel | Basic SCAN/DETOUR/SEARCH path; robot left tiles; stable line reacquisition not tested |
| M2 reattempt, 48c69d3 | PASS for reverse/rescan only: nine runs, five initially blocked, seven reverses in those five runs, at most two per run |
| Reattempt distance evidence | Centre echo increases +101,+114,+102,+115,+115,+102,+102 mm; one +27 mm at tile edge; last-run reverse about −100 mm/s for 1 s |
| Reattempt outcome | Free side in 5/5 blocked runs at 329–435 mm; no contact; event/attempt/summary counts correct; no 3 s timeout observed |
| DETOUR after reattempt | Four runs stalled or were operator-stopped after leaving the two-tile area; not a full detour PASS |
| Host module mutation check | 45/45 caught; reverse timeout/stop/ESTOP and three-blocked-profile bounds covered by host tests |

## Limits: the wide-cone finding

In M2 reattempt the 12 cm wide, 17 cm tall box at about 220 mm still gave
an echo about 20° off the sensor axis. Its edges were about ±15°, so both
30° endpoints could still see it. At 434 mm the edges were about ±8° and
an endpoint became free. Do not subtract a fixed beam angle: surface,
incidence and shape change the return. Reverse/rescan reduces angular
size without assuming a universal beam correction.

The coarse/fine floor check covers two encounters with one off-centre box,
including one clipped edge. Complete material/angle characterization,
precise centre alignment and safe clearance for every object are not
established. NO_RETURN remains unknown. Test the reverse
with room behind the robot; this code does not add a rear sensor.

## Ultrasonic driver and service

The HC-SR04+ supplies bounded range observations, not a clear-path decision.
The kernel robot uses `drv_ultrasonic_*` through the priority-12 sonar
service. The older `src/ultrasonic.c` driver remains for bare-metal benches
and simulation; do not link both driver owners into the robot.

### Sonar pins and timing

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

### Sonar interface

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

### Packed sonar result

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

### Sonar settings

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

### Sonar results

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

### Sonar limits

Fabric, slanted faces, narrow objects and open space can all give NO_RETURN.
The beam shape is not calibrated. Repeat fixed-distance/material/angle
checks before applying an offset or scale correction. This setup's 3.3 V
wiring is not approval to connect an arbitrary 5 V Echo output to RP2040.
No source/API test establishes actual stop latency or all surfaces.

## Evidence and history

- [S5 TEST_LOG](../../testing/subsystem5/TEST_LOG.md).
- [Kernel integration results and hardware tables](../../testing/integration/KERNEL_ROBOT.md).
- [Week 8 CR1 notes](../archive/architecture/HISTORY.md#files) (history) remain historical wording; the packed result above is the current implementation.
- [Obstacle history](../archive/s5-obstacle/HISTORY.md#files): full monitor evidence, scan/profile measurements,
  R1–R4 and M2-reattempt tables, the original ultrasonic driver with all
  seven bench runs, and the v1.6 detour test plan.
