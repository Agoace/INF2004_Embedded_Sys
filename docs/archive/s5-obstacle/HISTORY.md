# Obstacle history

The obstacle records below were removed from the tree on 2026-11-16.
The current monitor, scan, avoidance and ultrasonic service are in
[OBSTACLE.md](../../s5-obstacle/OBSTACLE.md). Each removed file is still in
Git, unchanged.

Read one file:

```sh
git show 569672a:docs/archive/s5-obstacle/VC_DETOUR_RUN.md
```

Commit `569672a` is on `origin/mtk3-staging`. Use the path from the table.
Relative links inside an old file can point to files that have moved.

## Files

| Path at `569672a` | What it holds | Now in |
| --- | --- | --- |
| `docs/s5-obstacle/ULTRASONIC.md` | Sonar pins, request/result contract, packed 16-bit result, settings, bench and kernel results | [OBSTACLE.md](../../s5-obstacle/OBSTACLE.md#ultrasonic-driver-and-service) (full text) |
| `docs/archive/s5-obstacle/ULTRASONIC_DRIVER.md` | First driver: wiring, timing constants, seven bench runs 2026-10-27 (rise latency, quantization, 100–400 mm targets, 20 mm blind zone, front offset 57.9 mm) | History only; summary in [Sonar results](../../s5-obstacle/OBSTACLE.md#sonar-results) |
| `docs/archive/s5-obstacle/OBSTACLE_MONITOR.md` | Forward monitor thresholds, states/events, 2026-10-27 bench | History only |
| `docs/archive/s5-obstacle/OBSTACLE_SCAN.md` | Mounted servo limits, scan points table, sequence/timing, two-read combine, PROFILE event, bench, no beam-width correction decision (2026-11-07) | History only |
| `docs/archive/s5-obstacle/VC_DETOUR_RUN.md` | Detour route and modules, track plan, runs R1–R4 (2026-11-05), M2 reattempt tables (2026-11-07) | History only; summary in [Results](../../s5-obstacle/OBSTACLE.md#results) |
| `docs/archive/DETOUR_V1_6_TEST_PLAN.md` | Detour v1.6 physical test plan; 740 mm/s loaded-speed recollection | History only |

[OBSTACLE_BYPASS.md](../OBSTACLE_BYPASS.md) and
[OBSTACLE_DETOUR_HANDOFF.md](../OBSTACLE_DETOUR_HANDOFF.md) stay in the
tree: robot and `src/` code comments cite them.
