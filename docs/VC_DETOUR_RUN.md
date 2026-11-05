# Vehicle Controller detour run (step c)

Step (c) of the [Vehicle Controller state machine](VEHICLE_STATE_MACHINE.md):
the obstacle scan and the obstacle detour as modules in the shell, wheel
speeds through the [motor controller](MOTOR_CONTROLLER.md). Branch:
`yx-detour` (from `yx-line-steer`). Target: `vc_line_test` (the same
target as the [Vehicle Controller line run](VC_LINE_RUN.md), now with all
four modules).

Status: built and host-tested on 2026-11-04; first track run R1 on 2026-11-05: **one detour done**, see [Run R1](#run-r1-2026-11-05-boot-a21a-run-1).
Scope of the first track test (operator 2026-11-04): the port only. The
open detour problems B, D, E and F
([detour handoff](OBSTACLE_DETOUR_HANDOFF.md#open-problems)) are not
fixed; the test also shows whether E (the low-duty sweep stall) is gone.

- Scan module: [`src/servo_scan.c`](../src/servo_scan.c), [`src/servo_scan.h`](../src/servo_scan.h)
- Detour module: [`src/detour_run.c`](../src/detour_run.c), [`src/detour_run.h`](../src/detour_run.h)
- Detour logic (unchanged, v1.8): [`src/obstacle_detour.c`](../src/obstacle_detour.c)
- Ping request: [`src/vehicle_inputs.h`](../src/vehicle_inputs.h)
- Pico glue: [`src/vc_line_test.c`](../src/vc_line_test.c)
- Host unit test: [`tests/test_obstacle_modules.c`](../tests/test_obstacle_modules.c)

## Route

```text
 LINE ──OBSTACLE (STOP < 258 mm)──▶ SCAN ──SIDE (side, distance)──▶ DETOUR
  ▲                                                                   │
  ├────────────────────── DONE_LINE ──────────────────────────────────┤
  └── FOUND ── SEARCH ◀── DONE_NO_LINE (search toward the obstacle) ──┘
 FAILED (scan: echo fault; detour: dt_fail_t) ─▶ FAULT, the robot stops
```

These are the routes of `VSM_DEFAULT_ROUTES`; nothing new in the shell.

## Modules

| Module | Does | Outcomes |
| --- | --- | --- |
| SCAN | Wheels 0, servo centre for 300 ms. Then 13 points (30° right to 30° left, 5° steps), 2 readings per point, the settle times of the old scan (150 ms first, 60 ms per step, 150 ms back to the centre). Builds the PROFILE word | SIDE: payload bits 0–8 nearest distance, bits 9–10 `obstacle_side_t`. FAILED 1: echo fault streak |
| DETOUR | Runs `obstacle_detour.c` unchanged. Command word → wheel speeds (table below). At DONE: resets the line memory, waits one step stopped, then decides | DONE_LINE; DONE_NO_LINE (payload: side to search, toward the obstacle); FAILED (payload `dt_fail_t`) |

The old scan `src/obstacle_scan.c` calls `servo.c` itself and cannot
share a target with the motor controller. The new scan only uses its
pure parts (`obstacle_scan.h` and `obstacle_profile.c`).

### Detour wheel speeds

| Command | Before (`detour_test`, duty) | Now (mm/s) |
| --- | --- | --- |
| STRAIGHT (and BACK, ON_LINE) | 20 % with a count heading hold | 131 mm/s on both wheels, **no heading hold** (operator) |
| Sweep (TURN_AWAY, TURN_TOWARD before the box is seen) | Duty rate control, 14–30 %; turned at 40–43 °/s | 20 °/s as a wheel speed: **19 mm/s** per wheel (`dr_turn_mm_s`) |
| Other turns (back off, ALIGN) | 20 % | 131 mm/s |
| STOP (look, coast) | 0 | 0 |
| New motion | 250 ms at 0 | 250 ms at 0 |

Back off: half of the turn toward the obstacle, at most 30° (run R1).
From 2026-11-05 (operator, after R1): a **fixed 25°** (`back_off_fn`
NULL, `back_off_deg` 25). With a turn toward the obstacle of less than
25° (R1, second detour: 10°), the back off turns the robot past the
heading of the straight before it.

**Risk: 19 mm/s is below every speed tested on the motor controller**
(lowest: 62 mm/s in DM1). It is 2.4 counts per 10 ms tick. The start
kick helps at the start; the motor controller stall rule (≥ 150 ‰ for
500 ms with no counts) stops the run if a wheel does not turn.

## Ping request (ADR rule 3)

The detour reads the ultrasonic only when it is ready (servo settled,
robot stopped for a look, every 60 ms in a sweep), and the scan reads as
fast as the driver allows. Before, the glue did this for each phase.
Now the running module asks:

| Field (`vsm_inputs`) | Meaning |
| --- | --- |
| `ping_ms` | Set to 100 ms by the glue before each step; the module may change it. 0: no pings. `VSM_PING_ASAP` (1): at the next time the driver allows (60 ms) |
| `ping_sent` | A ping went out since the last step |

The glue does not ping while an unread result waits. So when `ping_sent`
and `echo_new` are both true, the echo is the result of that ping, and
the modules read them in that order (`dt_triggered`, then `dt_reading`).
LINE and SEARCH do not set `ping_ms`: they keep 100 ms.

Timing change: the request is updated each 10 ms step, so a reading can
start up to 10 ms later than in `detour_test`.

## Glue (`vc_line_test`)

- Modules: LINE, SEARCH, SCAN, DETOUR (before: LINE and SEARCH only; an
  obstacle stopped the run with NO_MODULE).
- Text log: each handoff, the scan profile and side, each detour step
  with the pose, sweep ends, back offs, the fail reason.
- Steering log, mode 3 (BYPS): the position field is the scan point + 10
  (SCAN) or the detour step (DETOUR).
- Summary: scans, detours started and done, detour speeds.

## Tests

Host test `tests/test_obstacle_modules.c`:

- SCAN: wait, settle times, pings only in MEASURE, stray echoes ignored,
  nearer reading kept, RETURN, fault, restart, three whole scans (left,
  right, blocked), payload fields.
- DETOUR: sweep speed 19 mm/s, servo toward the obstacle, settle then
  ping, the pause, a whole TURN_AWAY with seen then lost readings, the
  straight with no heading hold, back off speed and direction, a look
  with no new pause, echo fault, BACK, DONE with and without the line,
  no sensor (NULL).
- Shell: LINE sees a box → SCAN → DETOUR (3 handoffs), DONE → LINE,
  DONE without the line → SEARCH toward the obstacle, detour FAILED →
  FAULT.

Mutation check: 45 of 45 mutations killed (`/tmp/mut_obs.py`). The first
check (46) left 7 survivors: 5 test gaps (checks added), 1 dead line (a
direction reset; removed), and 1 hidden by the test helper (it gave the
wheels 0 already; it now gives junk speeds, so each module must set
them). ARM sizes: `servo_scan` 392 B, `detour_run`
540 B.

## Track test plan

1. Box on the track. Same procedure as the line runs (VIN, GP21).
2. Look for: the scan side, the sweeps at 20 °/s without a stall, the
   back off, the return to the line, then line tracing again.
3. Record the run here: detours done, stalls, sweep rates, and whether
   problem E is gone.

## Run R1 (2026-11-05, boot a21a run 1)

Log `build/steer_logs/run_20261105-135825_boota21a_run1.txt` (1135 of
1140 entries) and the text log. Box on the track; 57 s. Operator: "it
works", then the robot ran into a wall at the end ("not a problem").

| Time | Event |
| --- | --- |
| 17.4 s | LINE: OBSTACLE at 227 mm → SCAN |
| 19.6 s | Scan: 228–245 mm from 30° right to 15° left, NR from 20° left. Free step R 7 (blocked), L 4 → detour LEFT |
| 19.6–22.8 s | TURN_AWAY sweep, lost at +49° |
| 22.8–25.7 s | STRAIGHT 350 mm |
| 25.7–31.8 s | TURN_TOWARD sweep, seen at −48° (turned 97°); back off 30° |
| 31.8–34.8 s | STRAIGHT at −17° |
| 34.8–39.1 s | TURN_TOWARD to −90°, no echo: RETURN |
| 39.1–42.0 s | Return straight, ON_LINE, ALIGN to 0°: **DONE** at (563, −53) mm |
| 42.0 s | DONE_NO_LINE → SEARCH (right, toward the box side) → FOUND after 120 ms → LINE |
| 45.5 s | Second OBSTACLE at 247 mm (the wall); scan → detour RIGHT |
| 47.8–55.3 s | TURN_AWAY to +44°, STRAIGHT, TURN_TOWARD seen after 10° (back off 5°), STRAIGHT |
| 57.0 s | Detour FAILED: too near (echo < 80 mm), the wall. FAULT, robot stopped |

No motor controller faults (0x00); heartbeat ISR 103 µs max, 3 missed
frames; Wi-Fi 3 packets lost.

### Wheel speeds in the detour

| Motion | Asked | Measured | Note |
| --- | --- | --- | --- |
| Sweeps (4) | 19 mm/s per wheel (20 °/s) | 16–18 mm/s, 16.5–18.4 °/s | **No stall** (problem E's low-duty stall did not happen). Jerky: one wheel at 0 in 10–17 of the 50 ms rows (the start kick and 2.4 counts per tick) |
| Sweep, second detour TURN_TOWARD | 19 | 9–10 mm/s | 950 ms only; ended seen |
| Straights | 131 mm/s | 123–132 mm/s, turn 0–2.4 °/s | No heading hold needed for these legs |
| Back off, ALIGN | 131 mm/s | 97–125 mm/s | Short (200–700 ms): mostly the rise |

### Result

- The port works: LINE → SCAN → DETOUR → SEARCH → LINE on the track.
- Problem E (sweep stall at low duty): not seen. The sweeps ran 15–20 %
  below 20 °/s, with short wheel stops.
- The second detour failed on the wall; the operator does not count it.
- Not tested: DONE_LINE (the first detour ended just off the line, the
  search found it).

## Run R2 (2026-11-05, boot dcb8 run 1): fixed 25° back off

Log `build/steer_logs/run_20261105-143811_bootdcb8_run1.txt`. Real box
about 160 mm across the line and 250 mm along it.

- Scan `487 236 227 227 218 218 218 227 227 236 NR NR NR`: detour LEFT.
- TURN_TOWARD saw the side only at −48° (turned 97°): the box echoes
  only near normal incidence (operator: "the flare is really bad").
- Back off 25° → −22°; straight 2 ran the full 350 mm and never saw the
  box; return to −90°, ON_LINE, ALIGN: **DONE_LINE** (the first one).
- In ALIGN the right wheel stopped (asked 132 mm/s, 0 counts, 40.1–40.6 s):
  the robot touched the rear of the box. 1.7 s later in LINE: STALL_L
  (fault 0x01), SAFETY stop.

Simulator check (`detour_test` sim with a narrow beam, see
[narrow beam](SIMULATION.md#narrow-beam-real-box)): the same box, same seen angle (−52°),
gives 22 mm clearance at 350 mm, 54 mm at 400, 91 mm at 450 and 500;
a 40° back off at 350 mm hits the box.

Change (operator 2026-11-05): `vehicle_mm` 350 → **450 mm** in
`vc_line_test`; back off stays fixed 25°.
