# IMU safety remote course test checklist

Status: planned device test on `mtk3-imu-safety`. The operator drives the car;
the Coder runs the remote Wi-Fi recorder. Use a normal MQTT robot image, not a
TEST image. Keep detector bounds fixed for this session. Mark each result PASS,
FAIL, or NOT OBSERVED. Do not infer a device result from a host test.

## Identify the session

| Field | Record |
| --- | --- |
| Date, operator, Coder | |
| Branch, commit, UF2 name and SHA-256 | |
| Build flags and image type | |
| Broker, recorder host, raw and decoded log paths | |
| Boot ID, start uptime, course and intended speed | |
| Camera or motor-output timing method, if available | |

## Coder: preflight and logging

- [ ] Verify the flashed UF2 hash and source commit. Record a fresh boot ID.
- [ ] Start the recorder before any GP21 press. Save host receive time, topic,
  raw payload, decoded fields, and retained flag for each packet. Capture
  `robot/state`, `robot/event`, `robot/summary`, `robot/heartbeat`, and
  `robot/status`; keep `robot/log` if available. Keep recording until the
  final stop. Preserve packet gaps and reconnect information.
- [ ] Exclude retained summaries from an earlier boot. Match all verdicts to
  boot ID and run ID. Check that the initial robot is IDLE (`state=0`),
  inactive, and has `motion_faults=0`. Confirm current state and heartbeat
  traffic before motion.
- [ ] Record baseline `stack_min_free`, `cpu_load_permille`, motion late/missed
  counts, and Wi-Fi poll-gap maximum. Note that these heartbeat values are
  not a named IMU-task stack measurement.
- [ ] Keep an operator action log with wall-clock time for each GP21 press,
  course start, overturn start/end, upright-and-still start, lift start/end,
  and final manual stop. Film the car and a visible clock if possible. MQTT
  receive time is not physical event time.
- [ ] Keep a physical power-off method ready. Support the car during overturn
  and lift. Keep hands clear of the wheels. Do not drop it. Stop the session
  if control or usable logging is lost.

`robot/imu` is reserved and is not published by this image. Remote MQTT can
show the stop state and reason, but it cannot show the calibrated XYZ samples
that explain a missed lift. A detector-rate trace requires a separate capture.

## Run 1: course, then overturn

- [ ] Put the car upright on the start line. Press GP21. Record calibration
  success and a new active course run. If calibration fails, stop and record
  the reason; the overturn trial has not started.
- [ ] Record a normal course segment under the first run ID: active flag, state,
  module, motion faults, heartbeat, and any hump event.
- [ ] While the run is active, perform one controlled overturn past 75 degrees
  with the chassis supported. Mark the action time. Put it upright and still
  after it stops. Do not press GP21 yet.
- [ ] Expect IMU overturn event kind 3, trigger detail 1. If USB is available,
  save the `IMU overturn detected` line. Expect `state=2` (FAULT),
  `motion_faults` mask `0x04` (`MC_FAULT_ESTOP`), stopped wheels, and inactive after
  cleanup. The matching run summary should have stop reason 15, shell fault
  6, and trigger detail 1.
- [ ] Leave the car upright and still for at least one second of fresh IMU
  observations. Confirm FAULT/E-Stop persists and no automatic restart occurs.
  Record any reboot or network gap.

## Recovery: two GP21 presses

- [ ] Press GP21 once after the upright-and-still period. This press requests
  the IMU clear and must not start calibration or command wheels. If USB is
  available, save `IMU safety clear requested, hold upright; press GP21 again`.
- [ ] Wait for the writer acknowledgement and a fresh state report. Record
  the transition out of FAULT. A message received before the press cannot
  prove the acknowledgement. If clear is refused or times out, the car must
  stay stopped; mark FAIL and do not make the second press.
- [ ] Press GP21 again. In current firmware this press starts line calibration
  and then automatically starts a new course run if calibration succeeds.
  Verify the E-Stop bit clears, `active=1`, calibration succeeds, and a new
  run ID appears. If calibration aborts, do not claim the second course run.

## Run 2: course, then supported hand lift

- [ ] Record a normal course segment under the new run ID with no IMU fault.
- [ ] Perform one supported lift with both wheels clear of the floor, using
  the gesture that previously missed detection. Mark first contact, full
  wheel-clear time, hold duration, and set-down time. Keep the car controlled.
- [ ] Check for IMU lift event kind 3, trigger detail 2, `state=2`, E-Stop mask
  `0x04`, or summary stop reason 14. If any occurs, record LIFT DETECTED. Do not
  label an unexpected detection a failure.
- [ ] If the wheels keep turning, record LIFT NOT DETECTED for this gesture.
  Press GP21 to stop the run while the chassis is supported. Confirm the
  wheels stop and save the GP21 run summary. Do not keep the wheels running
  in the air to wait for a trigger.

## Coder: close and classify

- [ ] Save raw MQTT frames, decoded CSV, operator action log, optional USB
  console/video, image hash, and one table of expected versus observed
  transitions. Keep the two run IDs and any new boot IDs separate.
- [ ] Mark separately: initial start; overturn reason; E-Stop; persistent
  FAULT; first-press no motion; acknowledged clear; second-press calibration
  and run; lift outcome; final GP21 stop; network continuity; no reboot.
- [ ] Mark the 60 ms confirmed-trigger-to-motor-output-off gate NOT MEASURED
  unless motor-output timing is captured. Approximately one-second MQTT state
  updates cannot prove that limit. Record physical-onset-to-stop separately
  if video permits it.
- [ ] Record lowest application `stack_min_free` (gate: at least 256 B), CPU
  load, motion late/missed counts, and Wi-Fi poll gap. Mark named IMU-task
  stack NOT MEASURED unless a task-specific reading exists.
- [ ] Record each hump crossing and any false IMU stop. One course run does
  not close the full H3 or ten-lap no-false-stop gates.
- [ ] Do not combine different images or boots in one result. Do not change
  detector thresholds during the session.

## Result table

| Gate | Expected | Observed | Verdict |
| --- | --- | --- | --- |
| Run 1 | Calibrate, then active course run | | |
| Overturn | Detail 1, FAULT, E-Stop, summary reason 15 | | |
| Fault holds | No motion until GP21 safe restart | | |
| GP21 press 1 | Clear request, no motion | | |
| GP21 press 2 | Recalibrate, then new active course run | | |
| Lift | Record detected or not detected for this gesture | | |
| Final stop | GP21 stops the wheels if lift does not | | |
| Connectivity | One boot and usable continuous log | | |
| Stop latency | At most 60 ms from confirmed trigger to motor output off | | |
| Stack | At least 256 B free; named IMU task measured separately | | |
