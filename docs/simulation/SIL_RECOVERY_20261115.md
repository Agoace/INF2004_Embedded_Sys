# Straight SIL recovery — 2026-11-15

Same C position/steering/configuration,18mm strip and10s/1ms run. No tuning.
“Acquired” means either native normalised sensor>=300, NOT calibration's
initial line-seen flag. Settled: |pair-midpoint error|<3mm continuously through
run end, at least500ms. Overshoot: greatest error opposite its initial side.
Yaw changes the initial sensor error: e=offset+119sin(yaw)+7.25cos(yaw)−7.25mm.
Line loss latches stop; no search. Completed experiment does NOT mean recovery.

| Offset mm | Yaw° | Initial e mm | Acquired | Settle ms | Overshoot mm | Lost |
|---:|---:|---:|:---:|---:|---:|---:|
|0|0|0.00|Y|1|0.00|0|
|0|5|10.34|Y|—|0.00|0|
|0|−5|−10.40|Y|—|0.00|0|
|5|0|5.00|Y|—|0.00|0|
|5|5|15.34|Y|—|16.14|1|
|5|−5|−5.40|Y|—|0.00|0|
|−5|0|−5.00|Y|—|0.00|0|
|−5|5|5.34|Y|—|0.00|0|
|−5|−5|−15.40|Y|—|16.31|1|
|10|0|10.00|Y|—|4.13|0|
|10|5|20.34|Y|—|17.26|1|
|10|−5|−0.40|Y|—|0.00|0|
|−10|0|−10.00|Y|—|4.11|0|
|−10|5|0.34|Y|—|0.00|0|
|−10|−5|−20.40|Y|—|17.43|1|
|15|0|15.00|Y|—|17.64|1|
|15|5|25.34|N/out of reach|—|0.00|1|
|15|−5|4.60|Y|—|5.30|0|
|−15|0|−15.00|Y|—|17.83|1|
|−15|5|−4.66|Y|—|5.29|0|
|−15|−5|−25.40|N/out of reach|—|0.00|1|

Evidence: `build/webots/sil-recovery-final-20261115T120113Z/`.
21/21 experiments10000 samples, clean exits/logs, independent C replay true.
Earlier grid is retained separately. Out-of-reach classification is for this
ASSUMED fitted model, not measured physical reach. ±5mm/yaw0 stays offset:
native position±4 rounds duty to20/20; not every failure is oscillation.

## Fidelity check before tuning

| Step response | Raw t90 |50ms-mean t90 |
|---|---:|---:|
| Robot B3 bootede2,150mm/s |110/110ms L/R |140/130ms L/R |
| Current SIL,129mm/s |11/11ms after command |60/60ms (10ms bins) |

Robot source: main `build/steer_logs/text_ede2.txt`, MB command row50,
first raw90% row61; [B3 report](../archive/s2-motion/MOTOR_CONTROLLER.md).
SIL first differential yaw reaches90% about21ms after command. Current torque-
limited velocity plant is not instantaneous, but much faster and lacks native
motor_ctl kick/rise8/feedback. This is not an identical-input system comparison;
DM1 supplies steady encoder speeds, not lag. Plant/optics/quantisation mismatch
can explain failures; these runs do NOT isolate or condemn the real controller.
Next approved fidelity work has separate results; no tuning applied here.

Run: approved DISPLAY=:0/XAUTHORITY, then
`python3 sim/webots/tools/sil_recovery_smoke.py --output build/webots/recovery-NEW`.
No robot-source edits or hardware; bounded logging and owned-PGID cleanup retained.
