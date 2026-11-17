# IMU safety recovery and lift sensitivity plan

Status: Planner draft after RT-0104. No detector threshold or recovery behavior
has changed. The owner requires IMU-only lift detection and wants the hump
detector left unchanged. This plan separates a measured failure from a proposed
cause so that the next repair does not add work without evidence.

## Evidence and limits

- The `be4d14f` device image produced four LIFT stops. One console report had
  `g_mg=295`, `vertical_mg=-1026`, and `age_ms=17`. This proves detection of
  those gestures, not detection of slow supported lifts.
- Several active runs stopped with `IMU_UNAVAILABLE`, detail 2 (INVALID). One
  captured console line had `age_ms=5`. These are fresh, unusable observations,
  not evidence of a scheduling gap. Detail 2 does not yet distinguish a bad
  accelerometer read from an untrusted calibration or invalid vector.
- After a LIFT latch on boot `3768582656`, repeated GP21 presses did not
  recover. The MQTT event carried the old LIFT reason, but not the failed gate.
  The USB port for the refusal line was wrong. The failed recovery is observed;
  its cause is not. The 100 ms clear-request expiry is a real code limit, but
  the record does not show whether a request was ever queued.
- The reported 5.7 mm and 12.8 mm hump events came from lift handling. There
  is no measured normal hump crossing with safety enabled. The absence of a
  hump false stop is not established.

## Phase 1: make the recovery failure observable

1. On each refused GP21 press, record a bounded live event that distinguishes:
   no fresh board; invalid sample; not upright; outside the near-1 g band;
   wheels or commanded duty not still; one-second clear window incomplete;
   clear request pending; writer refused or timed out. Record `clear_ms`, board
   age, and the effective latch in the USB line. Emit on the press or state
   change only, never each 10 ms tick.
2. Prefer the existing `robot/event` packet and a documented detail encoding
   over a new high-rate topic. A high-bit refusal marker with a low-bit gate
   code is one candidate. Preserve the existing kind-3 cause values and update
   the decoder, documentation, and host tests for any new encoding. The Coder
   should present the exact byte table before implementation.
3. Split INVALID into sensor-read failure, calibration not trusted, and other
   invalid vector if the writer has enough information. Preserve detail 2 as
   the generic/legacy meaning. Add new detail values only with a documented
   mapping. Do not add an I2C read merely to classify a failure.
4. Reproduce a LIFT latch on a restrained rig. Put the car upright and still.
   Capture the refusal gate and writer acknowledgement with the correct USB
   port and remote MQTT log. Check zero duty and encoder-count movement while
   the clear window forms. Record whether a request was queued before the
   100 ms limit is blamed.

## Phase 2: repair the demonstrated recovery cause

- If the safe window never completes, correct the specific gate only after
  its input is measured. Do not weaken upright, near-1 g, or stillness checks
  by assumption.
- If a safe request was queued but the writer missed the 100 ms deadline,
  retain one bounded operator intent for this latch and retry only while the
  safe gate holds. A new trigger, invalid IMU, timeout, or reboot must cancel
  that intent. Keep the Vehicle and motor fault latches until a fresh writer
  acknowledgement. The first press must never move the wheels; the second
  press still starts calibration and the run.
- Test early press, completed window, delayed acknowledgement, refused clear,
  interrupted window, new trigger during pending clear, and two-press restart.
  Assert zero motor commands before acknowledgement. Measure the new hot-path
  cost and stack use; avoid a per-tick event or unbounded retry queue.

## Phase 3: lower lift threshold without changing hump reporting

The current LIFT paths are `|a| < 400 mg` for two 10 ms observations, or
`|vertical change| > 500 mg` for three observations while `|a|` is more than
200 mg from 1 g. Keep the proven high-energy and free-fall paths. Lowering
only the 500 mg value below 200 mg cannot detect a near-1 g gentle lift,
because the separate magnitude gate still rejects it.

1. Capture the exact 10 ms bias-corrected mapped XYZ that the detector reads,
   plus gravity reference, magnitude, vertical change, tilt, validity, FIFO
   count, and timestamps. Use bounded RAM capture or a test profile whose CPU
   and RAM cost is measured. Dump after the motion; do not stream 100 Hz over
   MQTT during a course run.
2. Capture slow, moderate, and brisk supported lifts, a one-wheel lift,
   controlled overturn, and no-lift controls: flat joins, braking, turns,
   SEARCH, and measured 10 mm one-wheel hump passes at the actual run speed.
   Record true wheel-clear and hump-contact times with video. Use several
   independent repetitions and reserve some for validation.
3. Replay candidate lower-energy rules offline. Vary vertical bound and
   duration, and test an IMU-only direction/tilt guard against pure tilt and
   hump traces. Do not use hump detector state or estimated hump height as a
   safety input. Keep the existing high-energy and free-fall triggers active.
4. Select a rule only if the validation traces show the required lift gestures
   and zero false stops on the measured hump, flat, turn, brake, and course
   controls. Then run at least 10 known-hump passes and 10 no-hump laps on the
   device. Record lift misses and false stops. If the signals overlap, keep
   the conservative rule and state the IMU-only limit; a threshold cannot
   identify ground contact from acceleration alone.

## Order and checkpoint

Repair recovery and INVALID diagnosis before a lower threshold can create
more faults. The next firmware image must have a traceable commit and UF2 hash.
Do not call the lift sensitivity or hump false-stop gate complete from the
four LIFT events in RT-0104. Keep the 60 ms confirmed-trigger-to-output-off
gate and named IMU stack gate open until measured.
