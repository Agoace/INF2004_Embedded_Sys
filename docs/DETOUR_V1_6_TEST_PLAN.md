# Detour v1.6 — physical test plan

Target: `detour_test` v1.6 (see [Version 1.6](OBSTACLE_DETOUR_HANDOFF.md#version-16-sim-tested-not-run-on-the-robot)). The sim results are not robot results.

## a. Build

| Step | Action |
| --- | --- |
| 1 | Use the other laptop. It has `src/wifi_config.h` |
| 2 | `cmake -S . -B build -DPICO_BOARD=pico_w && cmake --build build --target detour_test` |
| 3 | Without `wifi_config.h` the build gives a warning and has no Wi-Fi log (use USB serial) |
| 4 | Flash `build/detour_test.uf2` and start the receiver (see [Commands](OBSTACLE_DETOUR_HANDOFF.md#commands)) |

## b. Bench checks (robot on the floor, first)

1. Run GP20 (spin calibration only). Check that the spin works.
2. Start a run and watch one sweep turn at the 12 % minimum duty. Check that both wheels turn the whole sweep.
3. If a wheel stalls: raise `sweep_min_duty` (try 14, then 16). If it still stalls, raise `sweep_deg_s`.

## c. Track runs

| Run | Box | Place |
| --- | --- | --- |
| 1 | 120 × 120 mm | On a straight, as in the D runs |
| 2 | 120 × 120 mm | Shifted 20 mm left or right of the line |
| 3 | 160 × 160 mm | On a straight |
| 4 | 120 × 120 mm | On a curve |

Log lines to read:

| Line | Meaning |
| --- | --- |
| `[detour] sweep lost/seen at heading ...` | A sweep ended: rate and coast |
| `back off X of turned Y deg` | The back-off formula result |
| `RETURN to line at heading ...` | The obstacle was behind: straight to the line |
| `DONE: line not seen, search ...` | Search after DONE |
| `[run] ... stopped by ...` | The run end reason |

## d. Pass and fail

Pass: DONE, back on the line, no contact, no stall.

| Failure | Change |
| --- | --- |
| Contact | `back_off_for` (cap 25) or `return_heading_deg`, `return_max_mm` |
| Stall | `sweep_min_duty`, or `sweep_deg_s` |
| TIME (failed: TIME) | `total_max_ms` |
| Turn overshoot | `sweep_deg_s` (lower) |
| NO_SIDE (wide box) | Not fixed in v1.6 |

## e. Measurements still owed to the simulator

| Value | Now |
| --- | --- |
| IR sensor spacing | Marked guess |
| Sensor-to-front distance | Marked guess |
| θ_max (echo incidence cut) | Assumed 20° |
| Beam width | Assumed |
| Corner echo | Assumed off |
| Loaded speed at 100 % | 740 mm/s (operator recollection) |
| Loaded on-spot minimum duty | Assumed 12 % |
| Coast | Assumed 50 ms constant |
