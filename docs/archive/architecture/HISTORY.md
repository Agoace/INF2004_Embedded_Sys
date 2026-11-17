# Architecture history

The architecture records below were removed from the tree on 2026-11-16.
Each removed file is still in Git, unchanged.

Read one file:

```sh
git show 569672a:docs/architecture/week8_change_requirement_1_notes.md
```

Commit `569672a` is on `origin/mtk3-staging`. Use the path from the table.
Relative links inside an old file can point to files that have moved.

## Files

| Path at `569672a` | What it holds | Now in |
| --- | --- | --- |
| `docs/architecture/CR1_EVIDENCE.md` | CR1 baseline, RAM/queue/MQTT before, A1/B1/B2 evidence, A2 plan, M9 accounting | Renamed to [CR1.md](../../architecture/CR1.md) (full text) |
| `docs/s2-motion/CR1_MOTION_PLAN.md` | CR1 motion inventory, A1 field table, controller/ISR inventory, A2, tests | [CR1.md](../../architecture/CR1.md#motion-module-plan) (full text; its shared-access rule is the same as the CR1.md rule) |
| `docs/architecture/week8_change_requirement_1_notes.md` | Week 8 Tribunal CR1 analysis (1738 lines): §1 requirement, §2–7 range/width/packing/race rules, §8 MQTT, §9 motion, §10 line/barcode, §11 IMU snapshot, §12–32 Spatial Optimizer, ultrasonic timing/status, servo, width geometry, clearance, §33–38 vehicle state and mode bits | History only; requirement summary and §11, §33–38 in [CR1.md](../../architecture/CR1.md#requirement-details) |
| `docs/architecture/BRANCH_LAYOUT.md` | Branches, merge rules, mtk3 layout, checks, branch state 2026-11-16, archive tags | [PROJECT_STATUS.md](../../architecture/PROJECT_STATUS.md#branch-layout-and-integration-rules) (full text) |
| `docs/archive/architecture/BRANCH_LAYOUT.md` | Earlier branch layout: active/archived branches, host tests, integration status, kernel bring-up, motion step 1, robot port, STEP 7 split, STEP 8 MQTT | History only |
| `docs/architecture/ADR_MOTOR_CONTROLLER.md`, `ADR_HEARTBEAT.md`, `ADR_VEHICLE_CONTROLLER.md` | The three accepted ADRs | [ARCHITECTURE.md](../../architecture/ARCHITECTURE.md#decision-records) (full text, one section each) |
| `docs/archive/architecture/CONTROL_ARCHITECTURE.md` | Older planned two-core control proposal, pin update (line ADC never ADC2) | History only |
| `docs/archive/architecture/INTERRUPT_CONVENTIONS.md` | Original interrupt and polling conventions | History only; current rules in [ARCHITECTURE.md](../../architecture/ARCHITECTURE.md#interrupt-and-polling-rules) |
| `docs/archive/architecture/VEHICLE_STATE_MACHINE.md` | First Vehicle Controller shell, host unit test | History only; current shell in [ARCHITECTURE.md](../../architecture/ARCHITECTURE.md#vehicle-controller-shell) |
| `docs/archive/architecture/robo_pico_rtos_multicore_plan.md` | Robo Pico RTOS concurrency and multicore proposal at `3ef9e4b` (2026-11-05), not implemented | History only |

[TEMPLATE_CHANGES.md](../../architecture/TEMPLATE_CHANGES.md) stays a separate
file: `scripts/check-template-changes.py` reads it.
