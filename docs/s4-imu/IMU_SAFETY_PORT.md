# IMU safety port and UP validation

## Scope and provenance

Manual port on `mtk3-imu-int`, based on staging `d3890d0`. Do not merge or
change the teammate branch. Source: smojjofy, `8ac6e38..038ff23` (initial
implementation `8ac6e38` included). The source branch's run logs are not
acceptance evidence for this UP integration. No robot flash or acceptance
is claimed here.

## Port steps

- A: pure detector and synthetic tests, from `038ff23`. Retain thresholds
  and physics. Reject non-finite/degenerate vectors and future observations.
- B: IMU service board and explicit applied-request token. Preserve staging
  task/core placement and existing hump/diagnostic behavior.
- C: small pure Vehicle safety gate, with bounded glue at existing wake,
  start and stop boundaries. A stop latches once; only an explicit writer
  acknowledgement can release it. No console output in the running path.
- D: additive summary stop reasons and UI labels; preserve staging barcode,
  obstacle, decoder and token-file behavior.
- E: matched UP resources, stack frames, task budgets and spare-Pico plan.

## Detector contract

The IMU task owns the detector. Inputs are mapped, bias-corrected acceleration
(X right, Y forward, Z up), never the encoder-compensated hump estimate.
There is no kernel, I/O, allocation, output or actuator call in the detector.
Loops have fixed bounds. Calibration pose acceptance rejects infinity too.

Triggers count consecutive usable observations, not continuous wall time.
A missing observation neither increments nor resets a trigger count. A
large timestamp gap does not turn five observations into a five-period
continuous tilt claim. Reader freshness stops an active vehicle separately.
An unusable observation resets the detection counters. Host tests cover the
missing-observation case explicitly. The clear window, unlike the trigger
counts, requires continuous fresh, upright, near-1 g, stationary evidence.

The port adds finite-vector validation even when a caller incorrectly sets
`accel_valid`. Invalid/zero vectors cannot report VALID. A reader rejects a
negative signed age as unavailable; ordinary uptime wrap remains supported.
The thresholds and gravity/lift separation stay those of the source. Slow
level lifting and some combined tilt/lift motions remain blind spots; this
is not an independent guarantee that a vehicle is on the floor.

## Status

Step A in progress. Integration, host matrix, resources and device profile
will be recorded in the commits that implement them.
