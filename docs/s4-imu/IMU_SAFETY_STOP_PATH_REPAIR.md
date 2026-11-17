# IMU safety stop path repair

## What it is

This record answers the review findings RT-0092, RT-0093 and RT-0094 on the
IMU safety stop path. It states what each defect was, what changed, and what
evidence supports the change. The work is on the branch `mtk3-imu-safety`.
The base for the repair is commit `2adcdca`.

The stop path is the chain that turns a confirmed IMU reason into a stopped
robot that cannot restart by itself. That chain has four links:

1. The IMU task steps the pure detector and publishes a small board.
2. Vehicle reads the board on every wake, latches the first reason, and
   requests the motor E-Stop.
3. Vehicle publishes a persistent shell FAULT.
4. Only the GP21 safe restart can release the latch.

## Findings and repairs

| ID | Defect | Repair |
| --- | --- | --- |
| F1 | A GP21 clear released the Vehicle latch *before* the IMU writer applied it. For up to 100 ms the safety check returned "permit", so calibration could clear the motor E-Stop and command wheel speed. | The clear is now a *request*. Every latch stays in force until the writer acknowledges on a fresh board that reports no trigger. A pending or refused clear blocks the start and every motor-fault clear. The final start gate also rechecks the board latch. |
| F2 | An IMU trigger called the E-Stop and set `cancelled`, then the run loop ended before `vsm_step()` could see the motor fault. `vsm_stop()` then set IDLE. The published state and the run summary could stay IDLE or RUNNING. | A new `vsm_set_fault()` enters FAULT from any state. The IMU latch calls it and publishes the vehicle board at once. `vsm_stop()` does not clear FAULT. Only `vsm_reset()` after an acknowledged safe restart clears it. A new fault code `VSM_FAULT_IMU` carries the trigger as its detail. |
| F3 | The safety clear timer received the encoder-only `wheels_still`, not `input.wheels_still`. A wheel that is driven but stalled counted as still. | The safety input now uses `input.wheels_still`, which requires unchanged counts AND zero commanded duty. |
| RT-0094 F1 | `elapsed_ms()` credited up to 100 ms between observations to the all-clear window. A gap over the freshness limit did not restart the window, so sparse samples could add up to the claimed continuous 1 s. | A gap longer than `IMU_SAFETY_CLEAR_GAP_MS` (30 ms, three missed observations) restarts the window. |
| RT-0094 F2 | An IMU loss while driving stopped the robot but kept no latch, so an MQTT START could resume without GP21 once data returned. | Owner decision: latch after motion has started, GP21 only. An unavailable IMU now latches a FAULT when `active` is true. A cold boot before the first sample still only blocks, so the boot fix stays. |
| RT-0092 F4 | The `!near_one_g` gate on the vertical-change rule removes lift coverage for a combined tilt and lift that keeps `|a|` near 1 g. | Kept, and documented as a deliberate limitation in `imu_safety.h`. Removing the gate makes a plain tilt latch the LIFT reason. See the evidence below. |
| RT-0092 F5 | The run log claimed a PASS for the IMU start gate on a step where `vehicle_network_policy.c` rejects START before that gate. | The step is marked NOT RUN and records the command status. |

## Owner decision on recovery

RT-0094 asked for a recovery rule before more motor-enabled tests. The owner
decided:

- If the IMU is lost while the robot is moving, the stop is a fault.
- The fault needs the GP21 safe restart.
- A cold boot with no IMU sample is not a fault. It blocks the start only.

`active` is the "motion has started" signal. Vehicle sets it before the line
calibration, so an IMU loss during calibration is also a fault.

The safe restart now takes two GP21 presses. The first press requests the
clear and prints `IMU safety clear requested, hold upright; press GP21
again`. The second press starts the run. The latch is released by the writer
acknowledgement, not by either press.

## Host evidence

The host test target is `cmake -S tests -B build/host` and
`ctest --test-dir build/host`.

New and changed tests:

| Test | What it proves |
| --- | --- |
| `test_vehicle_imu_clear_acknowledged` | With a writer that applies the clear on a later observation, the requesting press does not start a run. The first wheel command comes after the acknowledged clear and the next press. |
| `test_vehicle_imu_retrigger_before_start` | A trigger that appears between the safety wake and the start gate is caught by the final gate: no start, no clear request, zero wheel commands. |
| `test_vehicle_imu_refused_clear_relatches` | A refused clear keeps the latch, causes exactly one stop, and leaves zero wheel commands. |
| `test_vehicle_imu_persistent_fault` | An IMU trigger during a run publishes `VSM_FAULT` in the vehicle snapshot and `VSM_FAULT_IMU` with the trigger in the run summary. A refused clear keeps that state. |
| `test_vehicle_imu_loss_latches_fault` | An IMU loss during a run latches a FAULT with `IMU_SAFETY_IMU_UNAVAILABLE`, and the GP21 press cannot request a clear while the data is absent. |
| `test_imu_service` safety block | Unchanged counts with 100 permille commanded duty keep `clear_ms` at 0. Zero duty advances it to the full window. |
| `test_imu_safety` gap tests | A 40 ms gap restarts the window. Sparse observations alone never complete it. |
| `test_imu_safety` tilt tests | A sudden 66.4 deg tilt at 999.5 mg does not latch the LIFT reason. An 80 deg tilt latches OVERTURN. |
| `test_mqtt_dashboard` shell faults | Every shell fault code from 0 to 6 has a name. |

Mutation checks catch a revert of each repair:

| Mutation | Result |
| --- | --- |
| Remove the clear-window gap reset | 4 test failures. |
| Remove `!near_one_g` from the vertical rule | 13 test failures, including a false LIFT latch on a plain tilt. |
| Revert `input.wheels_still` to the encoder-only copy | `test_imu_service` fails on `clear_ms == 0`. |
| Release the latch when the clear is requested | `test_vehicle_imu_clear_acknowledged` fails. |
| Remove the explicit `vsm_set_fault()` call | `test_vehicle_imu_persistent_fault` fails on `state == VSM_FAULT`. |

Full host result at the repair commit: 370 tests, 3 failures. The failures are
the same three that fail at the base in this environment: `test_robot_contract`
and the two make-based target checks `test_imu_source_make` and
`test_junction_make`. `test_network_queue` does not link at the base
(`undefined reference to robot_network_metrics`) and is excluded from the run.
`test_mqtt_dashboard` needs `PYTHONUTF8=1` in this Windows environment. It
then reports one Windows loopback failure in a security test
(`ConnectionAbortedError`), not an IMU safety failure.

## Limits

- The host tests use synthetic accelerations and a stub board. They prove the
  rule structure, the ordering and the latch, not detection accuracy.
- The candidate bounds (600 mg upright, 75 deg overturn, 0.4 g free fall,
  0.5 g vertical change) still need recorded lift and overturn traces from the
  robot.
- The two-press restart is new. The operator run log must record it.
- No motor-enabled hardware test may run before this repair is reviewed
  (RT-0093).
- The branch shares base `a03f9e5` with `mtk3-staging`, which has moved on.
  Integration is a separate step.

## Related records

- [IMU_SAFETY_STOP_RUN_LOG.md](../../testing/subsystem4/IMU_SAFETY_STOP_RUN_LOG.md)
  — the hardware session on this repair. Step 2 is a PASS; the lift, overturn
  and recovery steps are NOT RUN.
- [IMU_SAFETY_RUN_LOG.md](../../testing/subsystem4/IMU_SAFETY_RUN_LOG.md) —
  the first hardware session.
- [IMU_SAFETY_TRACE_PROCEDURE.md](IMU_SAFETY_TRACE_PROCEDURE.md) — capture and
  replay procedure.
- [IMU.md](IMU.md) — the report-only hump detector.
