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

## STEP 7 task-split image: new acceptance

Prior M1/M2/M1-W passes are old-image evidence, not acceptance of this port.

| ID | Procedure and acceptance | Status |
| --- | --- | --- |
| S7-U01 | Cold-power TEST, wheels lifted: S0-U01 and S2 tests pass; S1-U02 saturation returns without blocking; S3-U02 shows <=10 ms line age, no dropped/missed samples and measured ISR/task elapsed costs; S5-U01/U02 request checks; per-app-task stack free >=256 bytes; S2-U05 last resets near 100 ms and next boot does not repeat | PASS 2026-11-06 with 3f889a1 (see the results below) |
| M1-S7 | Repeat M1 on task-split UP/no-radio image: calibration, LINE/SEARCH recovery and GP21 stop; no line losses from queue/scheduling, no vehicle overruns or telemetry loss; decode contiguous KR/VE; USB dump delta0; record all stack/CPU/ISR counters | PASS (basic) 2026-11-06 |
| M2-S7 | Repeat M2: SCAN real-trigger acknowledgements and one-shot deadlines, late old-generation results not consumed by DETOUR; box clearance and stable line reacquisition where tiles permit; no contact/stall/fault, lost samples/wakes, overruns or telemetry loss; dump/stack checks as M1-S7 | PASS (basic) 2026-11-06; the robot left the tiles, return to the line not tested |

GP28 analog visibility and ISR timing also need S3-U03. Preserve the tested
revision/profile, both TEST boots and full GP20 output. These checks need
explicit hardware authorization; no flash or physical test was performed
by the STEP 7 change. Follow the safety preconditions above.

## Step 7 hardware results: 2026-11-06

UP/no-radio images, cold VIN start, wheels lifted for TEST. Logs (ignored,
not committed) in `build/mtk_logs/`: `s7_test_20261106-215149.txt`
(445084c TEST), `s7_fix1_test_20261106-224419.txt` (3f889a1 TEST),
`m1_s7_*` and `m2_s7_*` (robot; the 3f889a1 robot image is byte-identical
to the 445084c robot image).

### TEST image

| Check | 445084c (two boots) | 3f889a1 (two boots) |
| --- | --- | --- |
| S0-U01 clock | PASS (second boot) | PASS (second boot), TIMER 1009836 us against 1010 ms |
| S1-U02 telemetry saturation | PASS: 34 accepted, 66 lost | PASS: 34 accepted, 66 lost |
| S2-U01 motion timing | **FAIL**: lifetime gap 8991..11024 and 8535..11475 us, late 1 | **PASS**: window gap 9984..10016 and 9791..10210 us, work max 41 / 28 us, late 0, missed 0. Lifetime INFO 9978..10020 and 9756..10247 us, late 0 |
| S2-U02 stale stop | PASS | PASS |
| S3-U01 line sampling | PASS: 1009 samples, 0 missed | PASS: 1009 samples, 0 missed |
| S3-U02 line task and ISR | PASS: age 945/952 us; task max 207/104 us per sample, 13 per-mille; ISR max 114 us, 75 per-mille | PASS: age 993/983 us; task max 84/152 us, 7 per-mille; ISR max 110/108 us, 69 per-mille |
| Barcode raw (GP28, floor) | 187..249, 211..230 | 178..230, 196..217 |
| S5-U01 pings | 20/20 valid, median 209 / 210 mm (object present, not checked with a ruler) | No fixed object: 0/20 and 3/20 valid |
| S5-U02 old request rejected | PASS | PASS |
| S2-U05 watchdog | PASS, 99 ms | PASS, 99 ms |

Stack high-water (bytes used of size), worst value of all runs: motion
536/3072, line 368/2048, sonar 376/2048, vehicle 1328/4096, telemetry
696/4096. All free margins are above 256 bytes.

PROBE A (3f889a1): the late count is 0 at all eight stages on both boots.
On the second boot the lifetime gap widened from 9965..10017 us to
9756..10247 us between `S1_burst_end` and `after_S1_yield`. This agrees
with the analysis: the priority-1 S1 burst delayed motion in 445084c. With
the TEST runner at priority 20 the delay is below 0.25 ms.

### Robot image

| Run | Result |
| --- | --- |
| M1-S7 | LINE 0–3.29 s with three short line losses (found again in 20–110 ms; start position off the line), SEARCH at the tile end, GP21 ESTOP at 4.30 s. Gap 9952..10050 us, overruns 0, response max 740 us, telemetry loss 0, lost wakes 0, faults 0, 32019 samples with 0 missed / 0 dropped, ISR max 110 us, USB dump loss 0 |
| M2-S7 | STOP at 239 mm, SCAN −30° to +30° (box 218..255 mm to +20°, no echo at +25°/+30°), DETOUR 2.8–20.2 s (the robot left the tiles and was stuck about 0.6 s, then free), DONE_NO_LINE → SEARCH at 20.23 s, GP21 ESTOP at 21.55 s. Gap 9946..10056 us, overruns 0, response max 998 us, telemetry loss 0, faults 0, 101929 samples with 0 missed / 0 dropped, USB dump loss 0 |

## STEP 8 MQTT acceptance addition

Repeat cold-power S0-U01 on all four role families and S2-U01 with Wi-Fi20.
M1/M2 remain independent local-drive criteria: loss of AP/broker must not
stop or degrade driving. Network S1-U04/U05/U06 separately cover reconnect,
rates/loss/decode and NORMAL-robot command policy. All are **Not run** on
the STEP 8 image. Use [the runbook](../../docs/MQTT_RUNBOOK.md).
