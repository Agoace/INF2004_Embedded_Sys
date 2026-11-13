# Current obstacle monitor, scan and avoidance

## What it is

The robot stops for a confirmed forward obstacle, scans, chooses a side,
and runs DETOUR. The five brief stages below describe the intended work
and the actual implementation separately. There is no separate adaptive
coarse/fine scan or reliable obstacle-width estimate.

The pure modules return requests to the Vehicle Controller shell. Motion
owns the servo and wheel outputs. The [ultrasonic service](ULTRASONIC.md)
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

The implemented first survey is the complete fixed 13-point scan: −30°
(right) to +30° (left), in 5° steps. It is not a separate coarse sweep.
The robot waits stopped for 300 ms, settles the first point for 150 ms,
then takes two readings at each point. Each next step settles for 60 ms;
the 60 ms trigger minimum still applies. Return to centre settles 150 ms.
Standalone bench scans measured 1829 and 1833 ms, excluding the robot's
initial 300 ms stop wait.

### 2 — Fine scan: not implemented

There is no edge-triggered second pass, smaller-angle refinement or
coarse-to-fine controller. The same 13 points at 5° spacing are all that
run. The beam is much wider than a step, so 5° spacing is not 5° edge
accuracy. Do not label current profiles as precise obstacle boundaries.

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
table remains in the [original scan record](../archive/s5-obstacle/OBSTACLE_SCAN.md#scan-points).

### 3 — Profiling

Two VALID readings retain the nearer one. VALID plus unknown/failure keeps
the VALID value. With no VALID value, any NO_RETURN gives NO_RETURN;
two failures give no data. A fault streak of five fails the scan and
centres the servo.

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

No width is calculated. Side selection is not a proof of body clearance.

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
| Straight-length setting / sweep distance | 400 / 150 mm |
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

Fine scan, width measurement, complete material/angle characterization,
precise centre alignment and safe clearance for every object are not
implemented or established. NO_RETURN remains unknown. Test the reverse
with room behind the robot; this code does not add a rear sensor.

- [S5 TEST_LOG](../../testing/subsystem5/TEST_LOG.md).
- [Kernel integration results](../../testing/integration/KERNEL_ROBOT.md).
- [Full monitor evidence](../archive/s5-obstacle/OBSTACLE_MONITOR.md).
- [Full scan/profile measurements](../archive/s5-obstacle/OBSTACLE_SCAN.md).
- [Full R1–R4 and M2-reattempt tables](../archive/s5-obstacle/VC_DETOUR_RUN.md).
