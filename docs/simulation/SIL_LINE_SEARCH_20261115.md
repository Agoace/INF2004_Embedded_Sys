# SIL LINE/SEARCH wiring — 2026-11-15

## Native path

`line_position` → `vehicle_sm` with `line_trace` (LINE) and `line_search` (SEARCH) → shell drive → `motor_ctl` → signed measured DM1 → fitted lag → Webots.

- Settings: `app_program/vehicle/vehicle_robot_config.h`, `LINE_CFG` (P7) and `SEARCH_CFG` (131 mm/s,2535/8 counts). Registration/default routes follow `vehicle_task.c:1303–1332`, except the approved LINE registration is `lt_module`, not the robot's junction wrapper. Only LINE/SEARCH are registered; no stubs.
- Motion ticks before the10 ms vehicle step. ADC runs1 ms; the last LOST/FOUND event waits for the next shell step, as in `vehicle_task.c`. The real shell handles handoff, search exhaustion and latched motor faults.
- Sonar inputs are absent (`echo_new=false`); the pure obstacle helper linked by `line_trace` receives no readings. SCAN/DETOUR/junction wrapper are not registered. A route to an absent module faults, not a silent fallback.
- All measured forward/reverse DM1 rows are used per wheel, including the historically wheelie-limited high rows already present forward. Exact zero and the0–5% segment interpolate zero endpoints: **DM1 dead band, steady-state only; dynamic braking UNMODELLED**. Magnitude>100% and nonfinite values reject and log, never clamp.
- World: straight test strip, white margins **ASSUMED (real mat wider)**. Three rows of315 mm tiles, ten columns;945×3150 mm. Centre row carries the measured18 mm line, side rows are pure white. Same21 starts. Tile seams are not modelled. No sampler override. Search gross optical-offset guard250 mm permits the119 mm sensor arm to sweep; the old non-search guard stays100 mm.

## Checks and current limit

- Red tests precede mapping/shell implementation and the dead-band extension. Tests cover measured rows, interpolation, band edges0/4.9/5/6/7%, actual shell LOST→SEARCH→FOUND→LINE, exhausted search, motor safety and explicit white-margin geometry.
-317 tests OK,3 existing expected failures. Unchanged native `test_line_trace`, `test_vehicle_sm` and `test_motor_ctl` pass.
- The first broad-floor diagnostic crosses the line in all8 search cases, but synthetic200/3700 calibration makes the fitted thin-strip peak lower than FOUND500. It is **not a final recovery result**. Final results are held for real simulated spin calibration and SW4's measured-level refit.

```sh
python3 sim/webots/tools/sil_recovery_smoke.py --line-search --output build/webots/NEW_SEARCH
```

Use the approved existing-display environment. Simulation only; no hardware/serial. Gate1 remains **TRIAL INCOMPLETE / NOT GATE1 PASS**.
