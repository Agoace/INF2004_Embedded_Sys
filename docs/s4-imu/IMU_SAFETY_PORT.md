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

## Integration contracts

The sole IMU writer publishes a coherent safety report. The report contains
an applied-request token. A request has a nonzero token and is consumed
once. Only a successful clear updates the applied token. An empty FIFO does
not refresh the observation or apply a request. A refused clear does not
change the receipt. A failed or interrupted board read leaves its caller's
output unchanged. The service keeps staging's task placement seam.

The pure Vehicle gate has no kernel or actuator dependency. Freshness is
50 ms. A local press starts a bounded 1000 ms recovery intent only after a
1000 ms safe-clear window. Requests retry at 100 ms. A matching nonzero
receipt and a fresh, valid, upright, near-one-g pose release the latch.
An old NONE report, another token, or an unsafe matching receipt cannot
release it. Unsafe input or expiry cancels the intent. Recovery does not
start a run: a later press is required. Remote commands cannot clear it.

A first latch queues ESTOP and one fault event. Later observations retain
the stop without repeating either action. The shell remains in IMU FAULT
through ordinary cleanup. Only a successful recovery resets it. Start,
calibration, delay and run paths use the gate. Idle ticks still process
commands when the IMU is missing; for example, DUMP remains available.
The task parks before the safety check, so a stale board cannot create a
busy loop. Console output occurs only after the active run has stopped.

Summary v3 adds reasons 14–16 and shell fault 6. The decoder retains the
v1/v2 limits. Dashboard changes add labels only. No packet layout, barcode,
scan geometry, configuration-token reader or live snapshot contract changes.

## Status

- A: `d1495c6`, pure detector and defensive tests.
- B: `66ed213`, sole-writer board and successful-clear receipt.
- C: `2f2b182`, pure Vehicle gate, bounded glue and persistent shell FAULT.
- D: additive labels and tests; also retain idle command handling while
  safety is unavailable and correct an unsigned-cast compiler warning.
- Fresh host checks: GCC 600/600 and Clang ASan/UBSan 600/600 CTest entries
  pass. Tests include refused clear/retry, empty FIFO, unchanged failed-read
  output, pre-request NONE, wrong receipt, unsafe receipt, future age,
  expiry, first-edge stop, absent-IMU cold start, persistent shell FAULT,
  summary reason and legacy version limits. These are host results, not
  UP timing or real-robot safety acceptance.
- E: UP resource, kernel-service audit and device profile remain pending.
  No teammate capture, SMP measurement or host duration is UP evidence.
