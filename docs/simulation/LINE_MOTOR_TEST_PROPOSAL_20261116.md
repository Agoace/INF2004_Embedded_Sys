# Proposed loaded motor-response test — review only

**Status: NOT AUTHORIZED. Do not execute, flash or connect hardware.**

## Purpose

Separate the remaining motor-response uncertainty from line-sensor feedback.
The retained-data analysis is in
[LINE oscillation analysis](LINE_OSCILLATION_ANALYSIS_20261116.md).
The home command replay predicts 62.5–64.5 deg/s wheel-odometry amplitude where
the real prefix gives 50.1 deg/s. A phase difference of 20 ms is not resolved
by the present logs. This proposal does not assume which component is wrong.

## Proposed conditions

Subject to separate operator approval:

- Use the fully loaded robot on a flat floor, with a safe clear area and an
  operator-controlled stop. Record floor, payload, battery voltage, firmware
  image/config hashes and motor calibration. Check the measurement ledger
  before asking for an existing value.
- Disable LINE, SEARCH, scan, detour and barcode actions. Use the unchanged
  native motor speed controller. Do not replace it with fixed duty.
- Use a fixed timed command sequence. Do not change parameters between steps.
- Verify the logging path can capture the required common-clock 100 Hz data
  **before** a trial is approved. Existing KR and slow MQTT state are not
  sufficient. Any logging/firmware change needs its own review.

## Proposed sequence

Requested speeds are left/right in mm/s. Total duration is 8 s, followed by a
confirmed safe stop. This is a specification, not an executable command file.

| Time s | Request |
|---|---|
| 0–1 | 0 / 0, stopped baseline |
| 1–2 | 146 / 146 |
| 2–3 | 110 / 180 |
| 3–4 | 180 / 110 |
| 4–5 | 146 / 146 |
| 5–7 | Five cycles: 110 / 180 for 0.2 s, then 180 / 110 for 0.2 s |
| 7–8 | 0 / 0; confirm stop |

## Required evidence

At a common-clock cadence of 100 Hz, retain:

- Request timestamp and requested left/right speed.
- Actual application/publication timestamp and sequence or age.
- Applied signed duty, raw cumulative left/right counts and motor fault flags.
- Actual motor tick time, not just a nominal period.
- Logging loss/overflow indicators. Any loss must be explicit.

Retain pre/post motor configuration and image hashes with the recording.
Counts and timestamps are primary; recompute speed without rounding early.
Battery and surface metadata must remain attached to this trial. Independent
body heading would help distinguish wheel odometry from slip, but must not be
claimed if unavailable. Do not use a 4–5 Hz MQTT plot as phase evidence.

## Planned analysis and stop conditions

Replay the same requests through native motor control and the existing
unchanged surrogate. Compare per-wheel rise/fall times, asymmetric response,
amplitude, phase and quantization on the same time grid. Keep held versus
linear missing-command sensitivity separate if a loss occurs. Do not fit
parameters merely to make the result agree.

An operator stop, fault, unsafe motion or recording overflow ends the trial;
it does not authorize an automatic retry. Main must approve hardware access,
the logging method, safety conditions and the final sequence before execution.
This test cannot confer GP28 or hardware Gate1 acceptance.
