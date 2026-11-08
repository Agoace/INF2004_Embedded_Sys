# Vehicle Controller detour run (step c)

Step (c) of the [Vehicle Controller state machine](../../architecture/ARCHITECTURE.md):
the obstacle scan and the obstacle detour as modules in the shell, wheel
speeds through the [motor controller](../../s2-motion/MOTOR_CONTROLLER.md). Branch:
`yx-detour` (from `yx-line-steer`). Target: `vc_line_test` (the same
target as the [Vehicle Controller line run](../../s3-line-barcode/LINE.md), now with all
four modules).

Status: built and host-tested on 2026-11-04; first track run R1 on 2026-11-05: **one detour done**, see [Run R1](#run-r1-2026-11-05-boot-a21a-run-1).
Scope of the first track test (operator 2026-11-04): the port only. The
open detour problems B, D, E and F
([detour handoff](../OBSTACLE_DETOUR_HANDOFF.md#open-problems)) are not
fixed; the test also shows whether E (the low-duty sweep stall) is gone.

- Scan module: [`app_program/subsystem5/servo_scan.c`](../../../app_program/subsystem5/servo_scan.c), [`app_program/subsystem5/servo_scan.h`](../../../app_program/subsystem5/servo_scan.h)
- Detour module: [`app_program/subsystem5/detour_run.c`](../../../app_program/subsystem5/detour_run.c), [`app_program/subsystem5/detour_run.h`](../../../app_program/subsystem5/detour_run.h)
- Detour logic (unchanged, v1.8): [`app_program/subsystem5/obstacle_detour.c`](../../../app_program/subsystem5/obstacle_detour.c)
- Ping request: [`app_program/vehicle/vehicle_inputs.h`](../../../app_program/vehicle/vehicle_inputs.h)
- Pico glue: [`src/vc_line_test.c`](../../../src/vc_line_test.c)
- Host unit test: [`tests/test_obstacle_modules.c`](../../../tests/test_obstacle_modules.c)

## Route

```text
 LINE ──OBSTACLE (STOP < 258 mm)──▶ SCAN ──SIDE (side, distance)──▶ DETOUR
  ▲                                                                   │
  ├────────────────────── DONE_LINE ──────────────────────────────────┤
  └── FOUND ── SEARCH ◀── DONE_NO_LINE (search toward the obstacle) ──┘
 FAILED (scan: echo fault or reverse timeout; detour: dt_fail_t) ─▶ FAULT, the robot stops
```

These are the routes of `VSM_DEFAULT_ROUTES`; nothing new in the shell.

## Modules

| Module | Does | Outcomes |
| --- | --- | --- |
| SCAN | Wheels 0, servo centre for 300 ms. Then 13 points (30° right to 30° left, 5° steps), 2 readings per point, the settle times of the old scan (150 ms first, 60 ms per step, 150 ms back to the centre). Builds the PROFILE word. If the side is NONE and fewer than 2 reattempts ran: servo centre, reverse 100 mm at 100 mm/s, stop and settle 300 ms, then a new full sweep. Each completed profile is published | SIDE: payload bits 0–8 nearest distance, bits 9–10 `obstacle_side_t`; NONE only after 3 blocked profiles. FAILED 1: echo fault streak; FAILED 2: reverse leg timeout (3 s) |
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

## SCAN reverse and reattempt

Approved option C is in the SCAN module only. DETOUR is unchanged. The
profile and side rules are unchanged. The added states are:

| State | Drive | Exit |
| --- | --- | --- |
| REVERSE | Servo 0; both wheels `-SS_REVERSE_MM_S` (-100 mm/s), through `VSM_ACT_SPEED` | Mean encoder movement reaches 1241 counts back: 100 mm at 12414 counts/m. A 3 s timeout gives FAILED 2 and the shell stops the motors. |
| WAIT | Wheels 0, servo 0 for 300 ms (the same settle as the first scan) | Starts a new full 13-point sweep |

No separate centre reading is taken. The full sweep measures 0 degrees, and
the nearest point of the new profile is the distance that DETOUR receives.

The attempt counter uses two logical bits: 0 initial, 1 first retry, 2 second
retry. `SS_MAX_REATTEMPTS` is 2, so there are at most three complete profiles.
The last NONE goes to DETOUR, which fails with `DT_FAIL_NO_SIDE` as before.
ESTOP and motion faults use the normal shell safety path.

The GP20 KR1 row stays 16 bytes. Its 3-bit module field shows 5 while SCAN
is in REVERSE. Module 5 is a log value only; the shell still runs SCAN. The
asked wheel fields show -100 mm/s and the servo field shows 0.

## Tests

Host test `tests/test_obstacle_modules.c`:

- SCAN: wait, settle times, pings only in MEASURE, stray echoes ignored,
  nearer reading kept, RETURN, fault, restart, whole scans, payload fields,
  blocked → reverse → free, three blocked profiles → NONE, reverse timeout,
  ESTOP and operator stop during reverse, and encoder distance math.
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

For option C, add a no-push, no-flash checklist before the next approved
hardware run: put a 17 cm box at about 212 mm, start one MQTT-observed run,
and record each `robot/obstacle` scan ID and attempt, each event kind 8, the
KR1 module 5 rows, and the final route. Pass: a reverse is 100 ± 10 mm,
the robot does not contact the box, and a free side reaches DETOUR, or
three blocked profiles stop with `DT_FAIL_NO_SIDE`. A reverse timeout must
stop with summary reason 13.

### Result M2-reattempt (2026-11-07, MQTT image `48c69d3`): PASS

Box upright (12 cm wide, 17 cm tall), robot stops at about 220 mm.
Nine runs, observed through MQTT; GP20 dump of the last run
(`tm_usb_dropped` 0).

| Item | Result |
|---|---|
| First scan blocked on both sides | 5 of 9 runs; the other 4 found a free side only at the 30° point |
| Reverse when blocked | 7 reverses in 5 runs; never more than 2 per run |
| Reverse leg (KR1 module 5 rows, run 9) | about -100 mm/s for about 1 s, then stop |
| Centre echo change per reverse | +101, +114, +102, +115, +115, +102, +102 mm; one +27 mm (run 6, attempt 2, robot at the tile edge) |
| Free side after the reverses | 5 of 5 runs, at 329-435 mm |
| Box contact | None |
| Events kind 8, attempt numbers, summary count | Correct in every run |
| Reverse timeout (reason 13) | Not seen (no reverse took 3 s) |

The DETOUR after a reattempt stalled in 4 runs or the operator stopped
it: the 2-tile test area is too short for a detour that starts at
330-435 mm, and the robot left the tiles. DETOUR is not changed by this
package. The reattempt is the scope of this check.

Cause of "both sides blocked": the ultrasonic cone still returns an echo
from an edge about 20° away from the sensor axis. At 220 mm the 12 cm box
edges are at ±15°, so the 30° points still see the box. At 434 mm the
edges are at ±8° and the 30° point is free.

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
[narrow beam](../../simulation/SIMULATION.md#narrow-beam-real-box)): the same box, same seen angle (−52°),
gives 22 mm clearance at 350 mm, 54 mm at 400, 91 mm at 450 and 500;
a 40° back off at 350 mm hits the box.

Change (operator 2026-11-05): `vehicle_mm` 350 → **450 mm** in
`vc_line_test`; back off stays fixed 25°.

## Run R3 (2026-11-05, boot 0491 run 1): straight 450 mm

Log `build/steer_logs/run_20261105-151729_boot0491_run1.txt`, console
`text_0491.txt`. Same box, two detours in one 90 s run. Back off fixed
25°, `vehicle_mm` 450. Result: **2 of 2 DONE_LINE, no contact, faults
0x00**.

| | Detour 1 | Detour 2 |
|---|---|---|
| Box face toward the robot | 160 mm (short side) | 250 mm (long side) |
| Scan | `423 226 221 229 212 212 212 212 233 NR NR NR NR` | `246 230 233 233 229 229 233 238 246 NR NR NR NR` |
| Free steps → side | R 6, L 3 → LEFT | R 7, L 3 → LEFT |
| Turn away (sweep lost) | +49° | +44° |
| Straight 1 end | (295, 339) | (350, 338) |
| Side seen at (turned) | −50° (99°) | −56° (100°) |
| Heading after back off 25° | −25° | −31° |
| Straight 2 end | (702, 149) | (736, 106) |
| ALIGN end, DONE | (702, −55) | (736, −65) |
| Detour time | 23.2 s | 22.5 s |

Wheels during the return and ALIGN: no row with a wheel asked to move
and 0 counts, except the first row after each start pause (R2 had a
blocked right wheel for 0.5 s in ALIGN).

Estimated closest gap from the robot body to the box (mm). Box edges
from the scan (sensor face 65 mm ahead of the pose point; detour 1 box
x 277–527, y −105–+55; detour 2 x 294–454, y −197–+53); robot body from
pose: front 123, rear 67, half width 80. Odometry only, so about ±20 mm:

| Leg | Detour 1 | Detour 2 |
|---|---|---|
| Straight 1 | 93 | 86 |
| Straight 2 | 79 | 111 |
| Return | 95 | 202 |
| ALIGN spin | **71** | 178 |

The sim prediction for the 160 mm face was 91 mm ([narrow
beam](../../simulation/SIMULATION.md#narrow-beam-real-box)); the robot got 71–95 mm.

Other notes:

- The sweep side angle is still steep (−50°, −56°): the flare
  over-turn is unchanged; the longer straight 2 makes up for it.
- Line following: 39 LINE_LOST → SEARCH → FOUND pairs (R2: 8), in
  bursts every 250–300 ms at 7.5–9.7 s, 39–43 s, 55–60 s and 86–90 s,
  about the same track place on each lap. Each search found the line
  in 10–140 ms. Not a detour problem; not examined.

Change (operator 2026-11-05, after R3): `vehicle_mm` 450 → **400 mm**
("travels a little too far"). Risk: about 45 mm less x at the return,
so the R3 detour 1 ALIGN gap of 71 mm becomes about 25–35 mm; the sim
gave 54 mm at 400 for the 160 mm face.

## Run R4 (2026-11-05, boot aff0 run 1): straight 400 mm

Log `build/steer_logs/run_20261105-153125_bootaff0_run1.txt`, console
`text_aff0.txt`. Same box, two detours; box face per detour not
recorded. Result: 2 of 2 DONE (1 DONE_LINE, 1 DONE_NO_LINE), no blocked
wheel, faults 0x00. Operator: "close but far enough"; 400 mm kept.

| | Detour 1 | Detour 2 |
|---|---|---|
| Scan | `225 221 217 212 212 217 217 221 230 NR NR NR NR` | `233 229 229 220 220 220 220 229 229 254 NR NR NR` |
| Free steps → side | R 7, L 3 → LEFT | R 7, L 4 → LEFT |
| Turn away / side seen | +44° / −54° (98°) | +48° / −54° (102°) |
| Straight 2 end | (637, 84) | (623, 116) |
| ON_LINE / ALIGN end | (637, 68) / (637, −55) | (623, 116) / (623, −7) |
| Outcome | DONE_LINE | DONE_NO_LINE; SEARCH found the line in 0.25 s |

Estimated closest gap (method as in R3): 42–50 mm if the 250 mm face
was toward the robot, 0–10 mm if the 160 mm face was. Both scans read
the box from 30° right, so the face cannot be told from the scan.

Detour 2 false line (problem D): at the end of the turn to −90°, the L
sensor read about 2000 (C about 150) for 300 ms while stopped, about
48 mm before the real line. ON_LINE then drove its fixed 123 mm, crossed
the real line (L and C about 1100 at 78.0 s), and stopped about 80 mm
past it; the ALIGN spin did not see the line. The dark mark is unknown.
