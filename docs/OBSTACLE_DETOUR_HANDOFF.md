# Obstacle detour — handoff (2026-10-30)

This record gives the state of the obstacle detour at the end of the 2026-10-30 session. The detour replaces the shelved edge-following bypass ([OBSTACLE_BYPASS.md](OBSTACLE_BYPASS.md)). A full design record is not written yet; this handoff is the current reference.

## State

| Item | Value |
| --- | --- |
| Branch | `yx`, pushed |
| Firmware on the robot | Still `detour_test` v1.4 (commit `5efeacd`); v1.5 is not flashed or run on the track yet |
| Current source | `detour_test` v1.6 (see [Version 1.6](#version-16-sim-tested-not-run-on-the-robot)); v1.5 added `line_ahead_mm` 123 mm |
| Module | [`src/obstacle_detour.c`](../src/obstacle_detour.c), [`src/obstacle_detour.h`](../src/obstacle_detour.h) |
| Track test | [`src/detour_test.c`](../src/detour_test.c) (CMake target `detour_test`) |
| Host test | [`tests/test_obstacle_detour.c`](../tests/test_obstacle_detour.c): passes; mutation check 76 of 76 caught (commit `4284a9e`) |
| Receiver | [`scripts/steer_log_receiver.py`](../scripts/steer_log_receiver.py) knows `detour_test` runs (log flag `SL_FLAG_DETOUR`, step names). It is **not running**: the laptop restarted after the last runs |
| Run logs | `build/steer_logs/` (not committed), table in [Track runs](#track-runs) |

## Version 1.6 (sim-tested, not run on the robot)

Source: `detour_test` v1.6. Branch `yx`. The robot still runs v1.4 until the operator flashes v1.6. Test plan: [DETOUR_V1_6_TEST_PLAN.md](DETOUR_V1_6_TEST_PLAN.md).

### What changed and why

| Change | Why |
| --- | --- |
| Back-off formula: half of the turned angle, at most 25° (`back_off_for`). `back_off_deg` 25 is used only when `back_off_fn` is NULL | A fixed 40° back-off gave contact and turn limits in runs D6–D18 |
| Return to the line when the obstacle is behind: an unseen turn at −90° (`return_heading_deg`) starts a STRAIGHT back to the line, at most 1000 mm (`return_max_mm`) | The robot ended in a spiral and hit the turn limit when it passed the obstacle |
| After DONE: line memory reset (`lp_reset_tracking`). If no line is seen, a search starts toward the obstacle side | Problem C (D10, D11, D16) |
| Continuous sweep turns: PI rate control on the wheel counts. Target 20°/s, readings every 60 ms, 2 consecutive readings decide "seen" or "unseen", duty 12..30 % | Step-and-stop turns are slow and the stops add coast errors. `sweep_deg_s` 0 gives the old stepped turns |

### New config values

| Field | Value |
| --- | ---: |
| `back_off_fn` | `back_off_for` (min(turned/2, 25)) |
| `back_off_deg` (fallback) | 25 |
| `return_heading_deg` | 90 |
| `return_max_mm` | 1000 |
| `sweep_deg_s` | 20 |
| `sweep_ping_ms` | 60 |
| `sweep_seen_count` | 2 |
| `sweep_min_duty` | 12 (loaded value not measured) |
| `sweep_max_duty` | 30 |
| `total_max_ms` | 40000 |
| `RUN_LIMIT_MS` | 90000 |

### Problems addressed and not addressed

| Problem | v1.6 |
| --- | --- |
| C (no search after DONE) | Addressed |
| Spiral and turn limit | Addressed by the back-off formula and the return to the line |
| Wide boxes | Not fixed: the scan gives NO_SIDE |
| D (line ignored in turns) | Not fixed: the line is still ignored during turns |
| Blind back-off coast | Not fixed: the back-off is still a blind coast |
| A, B, E, F | Not changed |

### Evidence

The evidence is simulation results. The simulator uses marked guesses (motor curve, coast, echo model, loaded speed 740 mm/s at 100 % from operator recollection). These are not robot results. In the simulator, a 120 × 120 mm box on a straight gave DONE with no contact. The branch `sim` of this repository holds the simulator and the traces.

## Design, version 1.5

Only straight drives and turns on the spot. No arcs. The servo is fixed 30° toward the obstacle during the detour.

```
TURN_AWAY ──▶ STRAIGHT ──▶ TURN_TOWARD ──▶ back off 40° ──▶ STRAIGHT ──▶ ... ──▶ line seen
 until the      350 mm after      until the                                         │
 obstacle was   the nearest       obstacle is                                       ▼
 seen, then     reading           seen                                     ON_LINE 123 mm
 not seen                                                                  ALIGN to heading 0
```

| Step | Ends when |
| --- | --- |
| BACK | Only when the obstacle is inside the turn circle (150 + 30 mm round the rear axle): reverse until it is not |
| TURN_AWAY | The obstacle was seen and then is not seen (the sensor passed its edge). Limit 90° |
| STRAIGHT | 350 mm (vehicle length 190 + width 160) after the nearest reading of this straight; 350 mm from the start with no reading. A line seen in a straight goes to ON_LINE. Limit 5 straights |
| TURN_TOWARD | The obstacle is seen, then the robot turns 40° back away, by wheel counts with no look. Limit 150° |
| ON_LINE | 123 mm more (the rear axle on the line) |
| ALIGN | Turn back to heading 0. The heading is the sum of all turns, from the wheel counts |

- Turns go in 5° steps: turn, stop 150 ms, one reading, then decide.
- "Seen" = a valid echo nearer than 400 mm. A valid echo nearer than 80 mm in a straight fails (TOO_NEAR).
- The line counts only after the line sensors lost it once (the robot starts on the line). The line is **ignored in turns**.
- Turns 20 % duty; straights 20 % with a heading hold on the wheel-count difference; 250 ms motor stop when the motion changes.
- Detour time limit 25 s; run limit 30 s.

### Version history (operator changes)

| Version | Commit | Change |
| --- | --- | --- |
| first build | `ab6ff5b` | Planned four sides, fixed 45° turns (never run) |
| simple build | `371b816` | Fixed 45° turns; straights end 190 mm after the nearest reading (never run) |
| v1 | `aa46970` | Turns end on what the sensor sees, 5° steps |
| v1.1 | `4284a9e` | Back off 10° after TURN_TOWARD sees the obstacle |
| v1.2 | `a21237e` | Back off 15°; straights 350 mm |
| v1.3 | `65da406` | Back off 30° |
| v1.4 | `5efeacd` | Back off 40° |
| v1.5 | — | `line_ahead_mm` 100 → 123 mm from [robot geometry](CALIBRATION_RESULTS.md#robot-geometry) (calculated from operator dimensions and the echo offset); not run on the track yet |

## Track runs

Obstacle: box or bottle on a straight part of the line. All runs went LEFT. File names are in `build/steer_logs/`; the robot text is in `text_<boot>.txt`.

| Run | Version | File | Result |
| --- | --- | --- | --- |
| D1 | v1 | `run_20261030-171439_boot38d8_run1.txt` | Second straight at −16°: readings 119 → 103 mm, clipped the corner; left wheel stall at 21.6 s |
| D2 | v1 | `…171448_boot38d8_run2.txt` | GP21 at 0.4 s, no detour |
| D3 | v1 | `…171523_boot38d8_run3.txt` | As D1; then TURN_TOWARD crossed the line (ignored) and failed at the 150° limit |
| D4 | v1.1 | `…172447_boot0273_run1.txt` | STOP only at 27.3 s; run time limit during TURN_AWAY |
| D5 | v1.1 | `…172535_boot0273_run2.txt` | Second straight no echo; TURN_TOWARD never saw the box, crossed the line (ignored); right wheel stall |
| D6 | v1.2 | `…173033_boot3075_run1.txt` | Left wheel stall in the first TURN_TOWARD |
| D7 | v1.2 | `…173113_boot3075_run2.txt` | TURN_TOWARD never saw the box: turn limit |
| D8 | v1.2 | `…173211_boot3075_run3.txt` | **DONE** (line at −40°); line following then lost the line 5 times; run time limit |
| D9 | v1.2 | `…173323_boot3075_run4.txt` | Left wheel stall in the second straight at −35° |
| D10 | v1.3 | `…173623_bootc2e0_run1.txt` | False line in the first straight at +36°, about 200 mm off the line (right sensor 3230); DONE off the line; follower pivoted 2.3 s; search failed |
| D11 | v1.3 | `…173713_bootc2e0_run2.txt` | **DONE** (line at −91°); ALIGN swept past the line; follower pivoted 1.3 s, then followed to the run time limit |
| D12 | v1.4 | `…174537_boot65ed_run1.txt` | Left wheel stopped in a TURN_TOWARD step (contact) at 13.2 s |
| D13 | v1.4 | `…174558_boot65ed_run2.txt` | TURN_AWAY ended at only +6°; TOO_NEAR in the straight |
| D14 | v1.4 | `…174638_boot65ed_run3.txt` | **DONE** (line at −3°); followed the line to the next lap's obstacle (STOP at 23.6 s); run time limit in that detour |
| D15 | v1.4 | `…174746_boot65ed_run4.txt` | Right wheel stall in the first TURN_TOWARD |
| D16 | v1.4 | `…174818_boot65ed_run5.txt` | Line seen 112 mm into the first straight at +36° (probably false, as D10); DONE; left wheel stall at 10.8 s |
| D17 | v1.4 | `…174850_boot65ed_run6.txt` | Both wheels stopped in the first straight (contact); the STRAIGHT print is missing (text buffer drop) |
| D18 | v1.4 | `…174956_boot65ed_run7.txt` | **DONE** (line at −20°); followed to the run time limit |

Summary: 17 detours started (all but D2). 6 reached DONE: D8, D11, D14 and D18 on the track line; D10 and D16 probably on a false line. With the back off at 40° (D12–D18), 3 of 7 reached DONE and 3 ended in contact stalls.

## Open problems

| # | Problem | Seen in | Proposed fix (not built) |
| --- | --- | --- | --- |
| A | A line seen while heading away from the line (heading > 0) is accepted: false lines about 200 mm off the track line | D10, D16 | Accept the line only in a straight with heading < 0 |
| B | ALIGN turns by the wheel-count angle and sweeps past the line | D11 | Stop ALIGN when the centre line sensor sees the line, else at the angle |
| C | After DONE the follower keeps its old fade memory and pivots on one wheel with no line; no search starts (a search starts only on LINE_LOST) | D10, D11, D16 | After DONE, reset the line memory; if no line, start the search |
| D | The line is ignored in turns: TURN_TOWARD crosses the line and runs to the limit or a stall | D3, D5 | Count the line in TURN_TOWARD: go to ON_LINE |
| E | Wheel stalls in TURN_TOWARD steps and in the first straight: the corner touches the box (count logs show one wheel stops first) | D6, D12, D15, D17 | Not analysed; check the turn circle against the box with the 40° back off |
| F | TURN_AWAY can end at a small angle when the box edge is near the 30° ray at the start | D13 | A minimum TURN_AWAY angle |
| G | Wi-Fi text buffer drops the newest lines when full | D17 | Known flaw ([STEER_WIFI_LOG.md](STEER_WIFI_LOG.md)) |

The operator decided to fix the line problems (A–D) later, after the 40° back off runs.

## Values to measure

| Value | In the code | Note |
| --- | ---: | --- |
| Turn counts per degree | 24.05 | Calculated from track 111 mm; wheel slip not measured; see [robot geometry](CALIBRATION_RESULTS.md#robot-geometry) |
| Front-corner circle (rear axle to farthest corner) | 150 mm | Calculated, 146.7 mm; see [robot geometry](CALIBRATION_RESULTS.md#robot-geometry) |
| Line sensors ahead of the rear axle | 123 mm | Calculated 123 mm (line sensors at the robot front); used since v1.5; see [robot geometry](CALIBRATION_RESULTS.md#robot-geometry) |

## Commands

```sh
cmake --build build --target detour_test
fuser -k /dev/ttyACM0; picotool load -f -x build/detour_test.uf2
(setsid python3 -u scripts/steer_log_receiver.py >> /tmp/wifi_recv.log 2>&1 &)
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_obstacle_detour.c src/obstacle_detour.c -o build/test_obstacle_detour
./build/test_obstacle_detour
```

Track procedure: VIN on with USB off, wait for the LED, put the robot on the line with the obstacle at least 60 cm ahead, press GP21. Keep the robot on for about 2 s after it stops so that the replay reaches the receiver.

## Next steps

1. Restart the receiver before the next track run.
2. Decide on fixes A–D (operator: later).
3. Analyse the stalls (E) with the count logs.
4. Write the full design record (`docs/OBSTACLE_DETOUR.md`) when the design is stable.
