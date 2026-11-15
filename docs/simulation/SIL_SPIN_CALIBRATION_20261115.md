# Simulated spin calibration — 2026-11-15

## What runs

One aligned simulated robot performs the calibration sequence in `app_program/vehicle/vehicle_task.c:689–788`: negative spin to OUT633 counts, pause200 ms, positive spin to the opposite OUT633, pause200 ms, calibration end, negative spin HOME or centre-stop. Constants come from `vehicle_robot_config.h` through the C adapter, not copied tuning.

The unchanged native motor controller drives both wheels through measured signed DM1 and the existing fitted lag. Actual encoder feedback ends each spin. Actual optical ADC samples feed public `lp_calib_begin/add/end`; the collection/update split follows `line_service.c:141–161`. Homing uses the public detector's centre norm and observed peak, with the robot's900 /500 /50 thresholds.

**Seam:** sequencing is simulation-owned Python equivalent to those SDK-task functions, not execution of the SDK task itself. Button polling, RTOS acquisition queues and timing are absent. Existing flat optical and coast/braking limits remain.

## Independent start poses

Keep `spin-result/calibration.json` and `samples.csv`. Each recovery trial starts a fresh world at one of the same21 poses, replays the exact sampled calibration rows through public calibration APIs and checks the resulting endpoints. This resets pose/physics between trials; it does not claim the physical robot starts off-line immediately after its homing step. Profile and sampled-row SHA256 checks reject mismatched evidence.

```sh
python3 sim/webots/tools/sil_spin_calibration.py --output build/webots/NEW_SPIN
python3 sim/webots/tools/sil_recovery_smoke.py --line-search \
  --spin-calibration build/webots/NEW_SPIN/spin-result/calibration.json \
  --output build/webots/NEW_GRID
```

Both commands use the approved existing-display environment. No hardware or serial.

## Interim check, before SW4 level refit

- `build/webots/sil-spin-A-20261115T130348Z/`:2260 ms,1740 collected ADC rows; L200..1698 / C200..1698, valid public calibration. No synthetic3700 endpoint.
- OUT− ends440 ms; reverse begins640; OUT+ ends1540; homing begins1740 and ends2260. Clean Webots exit/log.
-319 tests OK,3 existing expected failures. Unit checks cover config, two OUT goals, homing, timeout/fault and sampled-endpoint replay.
- Final21-start results are held for SW4's measured optical-level refit. This interim result does not validate the optical footprint or physical recovery.
