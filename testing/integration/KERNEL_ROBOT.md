# R5 kernel robot track check

| ID | Check | Pass condition | Status |
| --- | --- | --- | --- |
| R5 | Same 90 s track and box procedure as bare-metal R4, with the kernel robot image | Line followed; two detours end DONE_LINE or DONE_NO_LINE followed by successful SEARCH; no contact, no motor/sensor faults, no lost samples/wakes, no USB dump loss | Not run |

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

No R5 hardware run has been made. Save the log and operator observations
with the tested commit before changing this status. The kernel phase is
not the bare-metal phase-9 schedule; evaluate stop distance and task/ADC
jitter on hardware, not from host test duration.
