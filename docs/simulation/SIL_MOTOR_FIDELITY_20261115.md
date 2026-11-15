# SIL motor fidelity — 2026-11-15

## Path and fit

- Unchanged `line_position.c` → unchanged `steer_mm_update` → `mc_set_speed` → unchanged `motor_ctl.c` → measured forward DM1 interpolation → effective lag → Webots wheels.
- P7 settings come directly from `app_program/vehicle/vehicle_robot_config.h`: base146 mm/s, gain85%, D85%, min62, max378, far threshold50, far gain340%. No recovery-based tuning.
- `MC_ROBOT_CONFIG` supplies10 ms tick, rise8 counts/tick/tick,460 mm/s speed cap,12% start kick, PI, stale and stall checks. Motor tick precedes vehicle steering command, as in the robot's motion notification path.
- Effective first-order lag80 ms is **fitted to ede2**, not a unique measured motor constant. Reference: B3 bootede2 MB row50→61, raw speed t90=110 ms each wheel at150 mm/s. The fit uses the complete motor-controller/plant loop, not recovery outcomes.
- Mapping accepts exact0 or measured forward5–100%; other values fail and log the requested duties. No clamping, extrapolation or reverse model. High-duty DM1 wheelie limits remain.

| Raw step t90 | Left | Right |
|---|---:|---:|
| B3 measured |110 ms|110 ms|
| Native controller + fitted host plant |113 ms|107 ms|
| Native controller + fitted Webots plant |113 ms|107 ms|
| Allowed ±20% |88–132 ms|88–132 ms|

## Evidence and checks

- Webots step: `build/webots/sil-motor-final-20261115T122521Z/`; clean exit/log, owned process group gone.
- `sil-build.json` records native source/header hashes, compiler, DM1 rows and fit source. `motor-step.csv` records duty, integer encoder counts, actual speed and lag output.
- Red test first failed on the missing motor module. The first50 ms trial then failed the t90 bound; fitting used only the measured step target.
- Unchanged `tests/test_motor_ctl.c`: all tests pass. Full Webots suite:313 tests OK,3 existing expected failures. New checks cover actual P7 settings, calibration, step response and rejected duty.
- Frozen replay:158 RX /58 TX MATCH; Gate1 remains **TRIAL INCOMPLETE / NOT GATE1 PASS**.

## Explicit seams

The simulation calls `steer_mm_update` directly; it does not run the full vehicle task, line-search or obstacle state machines. Line loss latches stop. ADC/physics runs1 ms, motor/steering10 ms; synchronous simulation does not prove hardware scheduling. Encoder counts are nearest ties-even joint-angle projections, not GPIO edges. Stop is velocity decay under the fitted model, not measured braking. Optical footprint remains fitted/assumed; height is operator-measured approximately8 mm. Source replay checks recorded inputs, not physical fidelity.

Commands:

```sh
python3 sim/webots/tools/sil_motor_bench.py --output build/webots/NEW_STEP
python3 sim/webots/tools/sil_recovery_smoke.py --motor-fidelity --output build/webots/NEW_GRID
```

Use the approved existing-display Webots environment for both commands.
