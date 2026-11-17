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
| Remove the board-latch recheck at the final start gate | `test_vehicle_imu_retrigger_before_start` fails on `vehicle_calibration_epoch()==0`. Before that assertion the scenario still passed, because the next safety wake stopped the robot before any wheel command. |
| Remove the explicit `vsm_set_fault()` call | `test_vehicle_imu_persistent_fault` fails on `state == VSM_FAULT`. |

Full host result at the repair commit: 370 tests, 3 failures. The failures are
the same three that fail at the base in this environment: `test_robot_contract`
and the two make-based target checks `test_imu_source_make` and
`test_junction_make`. `test_network_queue` does not link at the base
(`undefined reference to robot_network_metrics`) and is excluded from the run.
`test_mqtt_dashboard` needs `PYTHONUTF8=1` in this Windows environment. It
then reports one Windows loopback failure in a security test
(`ConnectionAbortedError`), not an IMU safety failure.

## Cost and optimisation

The device has limited RAM and CPU, so the added code was reviewed for cost.
The measured numbers below are a HOST proxy, from `build_bench_imu.c` with
`gcc -O2` on the x86 host, 2,000,000 samples per case, three runs. They are not
device cycles. A device measurement needs one more flash cycle and is not done.

| Path | Change | Host proxy before | after | Source saving |
| --- | --- | --- | --- | --- |
| Upright sample (the running case) | The tilt angle is not computed inside the upright bounds: there `Z >= 600 mg` and `|X|,|Y| <= 400 mg` cap the tilt at 43.3 deg, so the 75 deg overturn limit cannot be reached. | 0.042 to 0.044 us per sample | 0.0075 to 0.0085 us per sample | one `atan2f` and one `sqrtf` per upright sample |
| Tilted sample | The angle is still computed, but |a| is passed in instead of repeated. | 0.046 to 0.047 us | 0.045 to 0.047 us | one `sqrtf` per tilted sample |
| Invalid sample | The magnitude is not computed after the unusable input returns. | 0.0025 to 0.0030 us | 0.0025 to 0.0030 us | one `sqrtf`; not visible on the host, where `sqrtf` is one instruction |
| `imu_safety_now()` in Vehicle | The board is read straight into the cached copy. `imu_safety_try_read()` writes its output only on success, so the scratch copy was redundant. | | | 28 bytes of stack on every Vehicle wake |
| `imu_safety_step()` frame | Unchanged by the angle change. | 128 bytes | 128 bytes | `-fstack-usage` |
| Dead timeout branch | A pending clear is only raised from a latched state, so the `imu_latched == NONE` re-latch on timeout could not run. | | | code size only |

The detector work that remains is the gravity normalisation in
`vertical_change_mg()`: it costs one `sqrtf` and one division per usable
sample. Caching the reference magnitude costs 4 bytes and saves nothing while
`update_gravity()` runs every quiet sample, so it was not changed.

`tilt_ddeg` now reads 0 while the sample is inside the upright bounds. The
field is diagnostic only, it is printed once per latch, and no test asserts a
non-zero value for an upright sample.

## Unavailability cause

`imu_safety_evaluate()` reports one value for every way the IMU can stop being
usable, so a field report could not say which one happened (RT-0099).
`imu_safety_unavailable_cause()` separates them, and the value is carried in
the detail byte of the live `robot/event` and printed on the USB line:

| Detail | Meaning |
| --- | --- |
| 1 | Stale: a copy exists but is older than `IMU_SAFETY_MAX_AGE_MS` |
| 2 | Invalid: the newest sample is not valid or not calibrated |
| 3 | Absent: no copy was ever published |
| 4 | Pose: the sample is fresh and valid but outside the upright bounds. A refused start only; the cause helper never returns it |

A stopped robot stores the cause once, at the first latch, because the live
event suppresses repeats (RT-0101/F3). A later refused start while the latch is
in force reuses that saved cause instead of reporting NONE (RT-0101/F2).

The IMU `compute_*` counter is **wall elapsed time**, including preemption by
tasks at priority 10 to 13, not exclusive CPU time. `work_*` also includes
preemption and adds the probe and the final publication. Neither counter alone
can attribute CPU load (RT-0101/F1).

## Limits

- The host tests use synthetic accelerations and a stub board. They prove the
  rule structure, the ordering and the latch, not detection accuracy.
- **The lift stop does not fire on a real hand lift.** The hardware session
  recorded no lift trigger for a quick lift or a 3 s lift, because a hand
  supports the robot: the free-fall rule needs a release and the vertical rule
  needs about 5 m/s2 sustained for 30 ms. This is a measured coverage gap, not
  only a written one. A recorded raw lift trace is required before the bounds
  can be changed.
- The overturn bound has **one** measured latch: 75.7 deg at 979 mg, 9 ms old.
  One sample is not a margin.
- A ±Z rule with a direction guard instead of the `!near_one_g` magnitude gate
  is a candidate improvement. It must be checked against hump false stops
  before it replaces the current rule.
- An IMU loss while driving now latches a GP21-only fault. The IMU task has the
  lowest priority and `IMU_SAFETY_MAX_AGE_MS` is 50 ms, so a scheduling gap over
  50 ms during a run becomes a fault. The idle Wi-Fi gap is about 21 ms. The
  running case is not measured. If it proves sensitive, debounce the
  unavailability before latching.
- The GP21 two-press flow is enforced on the button path. After an
  acknowledged clear, a remote START is accepted, because the owner rule is
  that GP21 is the only way to release the latch, not that every start needs
  two presses.
- The branch shares base `a03f9e5` with `mtk3-staging`, which has moved on.
  Integration is a separate step.

## Related records

- [IMU_SAFETY_STOP_RUN_LOG.md](../../testing/subsystem4/IMU_SAFETY_STOP_RUN_LOG.md)
  — the hardware session on this repair. Cold boot and overturn are PASS, the
  GP21 two-press recovery is PASS, and the lift stop is NOT DETECTED on a real
  hand lift. The upright-gate refusal is NOT RUN.
- [IMU_SAFETY_COURSE_RUN_LOG.md](../../testing/subsystem4/IMU_SAFETY_COURSE_RUN_LOG.md)
  — the first course run. The stop was classified as IMU unavailable and the
  image could not report the cause; RT-0099 asked for that instrumentation.
- [IMU_SAFETY_RUN_LOG.md](../../testing/subsystem4/IMU_SAFETY_RUN_LOG.md) —
  the first hardware session.
- [IMU_SAFETY_TRACE_PROCEDURE.md](IMU_SAFETY_TRACE_PROCEDURE.md) — capture and
  replay procedure.
- [IMU.md](IMU.md) — the report-only hump detector.
