# Lift rule study: model, not evidence

## Status

This is a **model**. It is not a measurement, not a hardware result and not a
threshold decision. It exists because the lift stop did not fire on a real hand
lift, and the next action must not be a guess.

The only measured data in this record is the overturn sample from the stop run
log: 75.7 deg, 979 mg, 9 ms old, at the latch.

## The question

The owner asked whether a plus-minus Z threshold could detect a lift. The
question is fair: inside the upright bounds the tilt is limited and the Z axis
carries gravity, so a vertical rule looks natural.

Three sub-questions:

1. What vertical acceleration does a human hand lift produce?
2. What vertical acceleration does a 10 mm tile hump produce?
3. Can one threshold separate them and stay below the hump level?

## Model and assumptions

[`scripts/lift_rule_study.py`](../../scripts/lift_rule_study.py) computes the
acceleration profile of several events and reports which rule fires.

| Profile | Basis |
| --- | --- |
| hand lift 0.30 m in 1.0 s | trapezoidal velocity, peak 2d/T^2 = 61 mg |
| hand lift 0.30 m in 0.5 s | same, peak 245 mg |
| hard yank 0.30 m in 0.3 s | same, peak 680 mg; upper bound for a hand |
| release and catch | free fall, measured \|a\| = 0 during the fall |
| hump, raised cosine | rigid wheel, height 5 or 10 mm, length 40 or 60 mm, speed 0.3 or 0.5 m/s |

The hump profile assumes a **rigid** wheel. The real wheel has a foam tyre on a
tile surface, which smooths the pulse, so every hump number here is an **upper
bound**. The model also ignores the 10 ms mean of about four 400 Hz samples,
which attenuates a short pulse.

## Results

`python scripts/lift_rule_study.py` prints this table.

| Profile | Peak mg | Duration ms | Current rule | Z 250 mg | Z 300 mg | Z 500 mg |
| --- | ---: | ---: | --- | --- | --- | --- |
| hand lift 0.30 m / 1.0 s | 61 | 990 | no | no | no | no |
| hand lift 0.30 m / 0.5 s | 245 | 510 | no | no | no | no |
| hard yank 0.30 m / 0.3 s | 680 | 300 | FIRE | FIRE | FIRE | FIRE |
| release and catch 150 ms | 1000 | 160 | FIRE | FIRE | FIRE | FIRE |
| hump 10 mm / 40 mm / 0.30 m/s | 1132 | 130 | FIRE | FIRE | FIRE | FIRE |
| hump 10 mm / 40 mm / 0.50 m/s | 3145 | 80 | FIRE | FIRE | FIRE | FIRE |
| hump 10 mm / 60 mm / 0.30 m/s | 503 | 200 | no | FIRE | FIRE | no |
| hump 10 mm / 60 mm / 0.50 m/s | 1398 | 120 | FIRE | FIRE | FIRE | FIRE |
| hump 5 mm / 40 mm / 0.30 m/s | 566 | 130 | FIRE | FIRE | FIRE | no |

"FIRE" means the consecutive-observation count for that rule is reached.

## What the results mean

1. **A plus-minus Z rule does not solve the observed failure.** A slow lift of
   0.30 m in 1.0 s is about 61 mg. A brisk hand lift is about 245 mg. A Z rule
   needs a threshold below 61 mg to catch the first case. That level is far
   below any hump model and would fire on line bumps, tile joins and handling.
   The measured session-2 lift was described as slow, so the 61 mg case is the
   relevant one.

2. **The current rule already covers the events that matter for safety.** A
   release gives free fall and the free-fall path fires. A hard yank gives about
   680 mg and the 500 mg path fires. "The robot was dropped or knocked" is
   covered. "A person picked it up slowly" is not, and no accelerometer rule at
   a usable threshold can cover it.

3. **The hump numbers are a warning, not a result.** The rigid-wheel model puts
   a 10 mm hump above the current 500 mg vertical rule in most cases. If the
   real pulse is even half of the model, the current rule can latch a safety
   stop on a hump while the robot drives. The tile has an 18 mm line and 10 mm
   height, and the hump detector exists because that height is measurable. This
   interaction has **never been measured**: no session has run the robot over a
   hump with the safety stop enabled. It is the highest-risk open item in the
   stop path, and it is a false-stop risk, not a missed-stop risk.

4. **The decision needs two traces, not a threshold argument.** One real lift
   with the operator's actual motion, and one real track run over humps at the
   running speed. The analyser in
   [IMU_SAFETY_TRACE_PROCEDURE.md](IMU_SAFETY_TRACE_PROCEDURE.md) already
   replays a capture through the detector and reports the peak values.

## Options if a slow hand lift must be caught

None of these is a threshold change, and each needs its own test.

| Option | Signal | Cost and limit |
| --- | --- | --- |
| Traction loss | Commanded duty against measured wheel speed. Off the ground the wheels accelerate with no load, so measured speed exceeds the coast model. | Needs a threshold on the motion model, not on the IMU, and a coast/load model that is valid while the robot is in the air. Cross-subsystem: Vehicle would combine S2 and S4. |
| Ground contact | The line sensors lose the floor and the line while the robot was following. | A lifted robot over a line can still see the line, so this is weak near the track. |
| Duration and shape | A hand lift gives a long, low, non-symmetric vertical change; a hump gives a short, symmetric pulse. | Needs the two traces first. Fragile at low magnitude. |
| Accept the limit | Keep the current bounds and state that a slow hand lift is not detected. | No new risk. The safety stop then covers drops, knocks and overturns, not a deliberate gentle lift. |

## Recommendation

1. Do **not** change the lift bounds now. This model shows that the obvious
   change either does nothing for the observed lift or moves the hump false-stop
   risk in the wrong direction.
2. Measure the hump interaction before any other stop-path test, because a false
   stop at speed is more dangerous than a missed slow lift.
3. If the owner wants a slow hand lift to stop the robot, add the traction-loss
   signal, which is the only one in the list that separates "in the air" from
   "on a hump" for the same vertical pulse.
4. Keep the plus-minus Z idea for the report and the direction guard, not for
   the trigger threshold.

## Limits of this study

- Every profile is an assumption. No measured lift or hump trace exists.
- The hump model is an upper bound with a rigid wheel.
- The 10 ms mean of about four FIFO samples is not modelled, so a short pulse is
  weaker in reality than in the table.
- `+Z` and `-Z` are treated as one magnitude. The sign was not used.
- The detector also runs a gravity reference smoothing; this study compares
  against a fixed reference.
