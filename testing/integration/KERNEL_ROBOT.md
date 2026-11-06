# R5 kernel robot track check

| ID | Check | Pass condition | Status |
| --- | --- | --- | --- |
| R5 | Same 90 s track and box procedure as bare-metal R4, with the kernel robot image | Line followed; two detours end DONE_LINE or DONE_NO_LINE followed by successful SEARCH; no contact, no motor/sensor faults, no lost samples/wakes, no USB dump loss | Not run (no track access; see M1/M2 below) |

## Preconditions and procedure

1. Record the exact image commit, SMP/radio profile, supply, box dimensions,
   face orientation, track layout and ambient light. The R4 record did not
   identify which box face was presented; record it for R5.
2. With wheels lifted, cold-power TEST and capture S0-U01, S2, S3-U01 and
   S5-U01. Move a card over both line sensors for spread. Use a fixed object
   and ruler for the 20 ultrasonic pings. Do not proceed after any failure.
3. Cold-power the normal single-core radio-off robot image with wheels
   lifted; check GP21 calibration and emergency stop. Then use the same
   track/box as [R4](../../docs/VC_DETOUR_RUN.md#run-r4-2026-11-05-boot-aff0-run-1-straight-400-mm),
   VIN supply and USB disconnected. Keep access to GP21. Press GP21 once:
   calibration spin, 1 s wait, then the run. GP21 again latches ESTOP.
4. Observe two detours and recovery to line without contact. Stop for unsafe
   motion, repeated loss, sensor faults or stalls. Do not change tuning.
5. Connect USB without resetting and press GP20. Save the complete KR1/VE
   dump, header, fault/gap counters and final USB drop delta. Decode rows
   and events with `scripts/robot_log_decode.py`; verify contiguous row
   indexes and no speed saturation. GP21 erases the previous saved run.
6. Compare outcomes, recovery and clearance with R4: 2/2 DONE, one
   DONE_LINE and one DONE_NO_LINE with SEARCH finding the line in 0.25 s,
   no blocked wheel and faults 0. R4 had a possible false-line dark mark;
   document whether R5 sees it. A build/simulator pass is not an R5 pass.

## Results

No R5 hardware run has been made; M1 and M2 below are basic checks on two tiles. Save the log and operator observations
with the tested commit before changing this status. The kernel phase is
not the bare-metal phase-9 schedule; evaluate stop distance and task/ADC
jitter on hardware, not from host test duration.

## Basic checks on two tiles: 2026-11-06

No track was available, only one straight tile and one cross tile. Image:
`mtk3-staging-next` 752105d, `make SMP=0 WIFI=none`, cold VIN start, USB
unplugged during the run, dump with GP20 afterwards. Logs (ignored, not
committed): `build/mtk_logs/m1_line_*` and `build/mtk_logs/m2_obst_*`
(`.txt` dump, `.csv` from `scripts/robot_log_decode.py`).

| ID | Setup | Result | Status |
| --- | --- | --- | --- |
| M1 | Robot on the straight tile, line ahead, cross tile next, no box. GP21 to stop after SEARCH started | Calibration (L floor/line 178/3595, C 150/3577). LINE 0–3.3 s; one line loss at 1.01 s recovered by SEARCH in 60 ms; line end at 3.31 s → SEARCH; GP21 ESTOP at 4.14 s. 0 faults before the stop, 57100 line samples with 0 missed / 0 dropped, 0 lost events/wakes, task gap 9650..10355 us, 207 rows, USB dump loss 0 | PASS (basic). Too short to judge steering quality against bare metal |
| M2 | Same start, box on the line about 30–40 cm ahead | LINE stopped at echo 221 mm (STOP below 258). SCAN: servo −30° to +30° in 5° steps; box echoes 207–258 mm from −30° to +20°, no echo at +25° and +30°. DETOUR 3.4–25 s: back-off sweep, 400 mm straight, turn back, straight, final turn and straight; the box was cleared (operator), the robot left the tiles. GP21 ESTOP at 25.0 s (fault 0x04, expected). 1251 rows, 0 lost events/wakes, 0 missed / 0 dropped samples, task gap 9262..10741 us, USB dump loss 0 | PASS (basic). Return to the line not tested (no tiles there) |
| M1-W | M1 again with the watchdog robot image (dd75b28) | Header `reset power/RUN` (cold VIN). LINE 0–3.1 s; line loss at 0.85 s recovered in 30 ms; line end at 3.10 s → SEARCH; GP21 ESTOP at 3.93 s. 0 faults before the stop, 51350 samples with 0 missed / 0 dropped, 0 lost events/wakes, task gap 9944..10051 us, 197 rows, USB dump loss 0 | PASS (basic) |

Task gap in M2 (±0.74 ms) was wider than in the motion bench runs (±0.05 ms)
and still below the 11 ms limit. Watch it in R5.
