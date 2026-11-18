# Forward-band Webots driving smoke — 2026-11-15

## Scope and one mapping

Operator-approved simulation only: `home_mat_1_assumed`,1ms basicTimeStep,
10.000s simulation, headless on existing DISPLAY=:0. No Pico, serial,
firmware control, navigation or line following. This extends the
[passive step](WEBOTS_PASSIVE_REPLAY_20261115.md), not its acceptance scope.

**Mapping: decoded OUTPUT PWM -> duty -> steady wheel speed -> velocity target.**
It is NOT direct torque/PWM control. Only the16–20% forward band is taken from
measured data for this mapping. Reverse and braking are UNMODELLED.

Source: [loaded duty map](../s2-motion/MOTION.md#duty-map), **Run DM1,
2026-11-04, boot5d51, revision7a65c0e, forward columns of table rows16% and20%**:

| PWM (protocol full scale6250) | Duty | Forward L mm/s | Forward R mm/s |
| --- | --- | --- | --- |
| 1000 | 16% | +96 | +95 |
| 1250 | 20% | +129 | +129 |

DM1 reports loaded steady encoder-derived speed using12414 counts/metre,
NOT independently measured ground travel. Its reverse data and all other duty
bands are NOT used here. Endpoints above are measured; linear interpolation
between these endpoints is ASSUMED/UNMEASURED. The script uses endpoints only.

For each wheel independently:

- Exact PWM0 -> velocity0.
- PWM1000..1250 -> interpolate that wheel's two DM1 speeds in mm/s.
- Divide speed by ASSUMED/UNMEASURED radius32.5mm -> target rad/s.
- ANY other PWM, including negative or nonzero below1000, is REJECTED, not
  clamped, extrapolated or silently converted to zero. `rejections.json` stores
  slot, both raw PWM values and reason. The controller then requests zero
  velocity, exports incomplete evidence and exits nonzero.

The frozen protocol6250 scale is a wire constant, not a fitted motor constant.
Radius comes from the existing nominal65mm diameter in
`sim/webots/calibration/robot_assumed.json`. Available torque cap0.12Nm is also
ASSUMED/UNMEASURED from that profile. The controller uses `setPosition(inf)`,
`setAvailableTorque(0.12)` and `setVelocity(target)`; no `setTorque` drive path.

**Velocity0 is Webots' velocity-servo hold, NOT a measured hardware coast or
brake model.** Startup, acceleration and stopping transients are unmeasured.
No12% start kick,50ms drive/coast fit or80% extension from the profile is used.
Existing mass, inertia, friction and caster assumptions still affect physics;
the manifest retains their provenance. Motor thermal/current/battery effects,
reverse response and hardware braking are outside this model.

## Fixed script and observation order

Script timings and check windows are ASSUMED smoke inputs, not measured robot
parameters. The1ms/10s run settings are the operator-prescribed test scope.
Webots' velocity-servo/acceleration behaviour is also UNMEASURED for this robot.

| Relative command source time | Command | L/R PWM | L/R duty |
| --- | --- | --- | --- |
| 0–1s | stop | 0/0 | 0/0 |
| 1–2s | straight | 1250/1250 | 20/20% |
| 2–2.5s | gentle left | 1000/1250 | 16/20% |
| 2.5–3s | gentle right | 1250/1000 | 20/16% |
| 3–10s | stop | 0/0 | 0/0 |

Both wheels start in zero-velocity mode. Each1ms step samples actual encoder,
servo, accel, compass, pose, orientation and body velocity. Real SENSOR is
encoded; real scripted OUTPUT is decoded/range-checked; only its resulting
wheel velocity targets are issued to Webots. Servo stays centred.

The observation is BEFORE the new command acts. Source0 corresponds to the
first sample at absolute simulation1ms, as in the passive step. Command issue
is at source+1ms; the next `robot.step(1)` advances physics under that command.
Thus a sample at source+2ms observes its first effect. No extra drain/physics
step or retrospective application is added at the10s endpoint.

All raw samples, sample RAW brackets and ordered SENSOR/OUTPUT_CONSUMED event
RAW stamps remain in `samples.csv`, `io.csv`, `io.bin`. `motion.csv` joins by
slot/simulation time and adds decoded PWM, velocity targets, command-issue RAW
time, yaw, upright orientation and six-component body velocity. Pose and wheel
angles are in the corresponding samples row. Truth pose/orientation is used
ONLY for these simulation checks; it never enters SENSOR or feedback control.
Only encoder wire channels can be VALID; unsupported channels/timing stay CLEAR.
Logging is bounded to10000 rows and deferred until stepping ends. No new
writer process/thread, scheduling, GC or timer setting is introduced.

## Checks and their limits

Forward means world+x at the existing yaw0 start; left means positive world+z
yaw. Both +Y wheel axes must give positive encoder increments when moving
forward. Checks also require right encoder travel>left on the left turn, and
left>right on the right turn. Sensor/script input is never silently repaired.

All smoke thresholds are **ASSUMED/UNMEASURED acceptance values**, not measured
robot limits. They are emitted in `summary.json`:

- All inputs finite; displacement from start<=1m, height15..150mm,
  upright cosine>=0.9. These are gross failure checks, not a stability proof.
- Straight observation1.05..1.95s: +x>50mm, lateral change<50mm,
  absolute yaw change<0.1rad, both encoders positive.
- Left2.05..2.45s yaw>+0.02rad; right2.55..2.95s yaw<−0.02rad;
  forward encoder signs and differential ordering must agree.
- Stop observation4.0..4.2s and9..10s: planar speed<5mm/s and
  absolute yaw rate<0.05rad/s. Final1s drift<2mm and encoder drift<2 counts.
- Exactly10000 samples/commands,10.000s simulation, no rejected OUTPUT.

The inherited unpaced RAW-vs-source timing counters remain diagnostic and
NEVER abort the run. Finite/order/codec/mapping failures do fail it. Byte replay
rebuilds commands, mapping, packets, event order and checks from recorded
observations; it does NOT reproduce or establish deterministic physics or wall
scheduling. Changing a stored target without its measured command fails replay.

## Run and evidence

```sh
DISPLAY=:0 XAUTHORITY="$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)" \
  python3 sim/webots/tools/driving_smoke.py \
  --output build/webots/driving-NEW --timeout-seconds 90
```

Uses the same shared bounded launcher as the passive tool: exactly
`timeout 90s webots --batch --mode=fast --no-rendering --minimize --stdout --stderr <world>`.
Existing Xwayland only; no auth contents logged, server/install/wrapper changes,
or process-name kills. Only the created PID/process group is cleaned up.

Final exact-source evidence: `build/webots/driving-home1-final-20261115T092935Z/`,
owned PGID220701 (confirmed gone); earlier successful run
`build/webots/driving-home1-20261115T091835Z/`, PGID220134, is also retained.
Both had exit0, clean log,10000 samples/commands, all11 checks true. Straight observed
+x116.098mm; left yaw+0.103947rad, right yaw−0.107097rad. Final pose
(0.437682,0.164683,0.042493)m, yaw−0.005557rad; both encoder totals positive
(about2994.55/2987.98 angle-derived counts; wire totals are rounded integers).
Maximum speed4..4.2s was5.39e−11m/s in this model, not a physical stop claim.

Recorded-input replay SHA256:
`a59b330ce5c5670e678d2b4a2cb07452c3d0da9162affe203ad8b530e1d5bf9e`.
Timing counts: early9999, positive late1, phase>250us10000/>600us9998,
rolling/burst excess0; these are unpaced diagnostics, not timing acceptance.

Red-first tests cover mapping endpoints/interpolation/rejections, exact script
boundaries, generated artifact hashes, synthetic differential-drive direction/
stop/controller flow, nonfinite failure/zero requests, and corrupted-target
replay rejection. Webots observations above, not the synthetic test robot,
establish this simulation's direction/stop result. No physical fidelity claim.
