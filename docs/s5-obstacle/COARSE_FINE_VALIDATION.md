# Coarse/fine scan validation

## State

Software candidate, 2026-11-17. No new robot or floor acceptance is claimed.
Module commits: `75c5d85`, `c2a9a6a`. Integration code: `3f569b2`.
The [scan record](OBSTACLE.md#2--fine-scan-and-decision-bin-completion)
defines the schedule; [geometry](OBSTACLE.md#geometry-estimates) defines the
assumptions. The [wire contract](../s1-telemetry/MQTT_WIRE.md#obstacle-profile)
defines v3 and legacy decoding.

## Host evidence

- A red schedule test failed when the old second point was index1 rather
  than coarse index3. Geometry tests initially failed because the old report
  had no geometry fields. A red USB test failed because scan evidence was
  not retained. Local logs are in `build/scan-proof/`.
- GCC: 551/551. Clang ASan/UBSan: 551/551. Strict CMake warning checks pass.
- `test_scan_stages`: all91 contiguous hit intervals plus the empty case;
  coarse order, unique rays, 26 pings, coarse reuse, fine mask, duration,
  conservative settle intervals, unchanged PROFILE and no forward drive.
- Bounds tests: no hit, centre, inclusive nearest+150 mm, disjoint hits,
  clipping and TOO_NEAR. Geometry tests: empty, single hit, symmetric and
  asymmetric edges, clipped edges, unknown outward ray and TOO_NEAR.
- C encoder/Python decoder round trips include valid and unknown geometry,
  reserved-byte and validity rejection, boot offset20, retries and v1/v2.
- USB tests verify that scan lines wait for DUMP, use copied values, stop
  at48 rows with explicit omissions, and do not carry into the next run. Existing KR1 and event layouts are unchanged.

The 10 ms fake-clock scan has worst2560 ms including300 ms stopped wait;
fixed-order `b9a47b5` gives2230 ms under the same clock. This is a host
comparison, not a bound on robot echo or task latency. Main accepted2560 ms
for the first bench. Do not shorten settling or the60 ms sonar interval to
meet a nominal2.5 s target. Measure actual duration below.

## Firmware resources

Same public-example build for baseline `b9a47b5` and candidate `3f569b2`:

```sh
make -C build_make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
  KERNEL_RAM=1 BARCODE_MAP='F=LEFT'
```

| Resource | Baseline | Candidate | Change |
| --- | ---: | ---: | ---: |
| GNU size text, bytes | 437928 | 440424 | +2496 |
| GNU size BSS, bytes | 200088 | 200708 | +620 |
| All static RAM sections, bytes | 219432 | 220052 | +620 |
| Obstacle C value / wire, bytes | 24 /24 | 32 /32 | +8 /+8 |
| Obstacle sequence board, bytes | 28 | 36 | +8 |
| Telemetry queue value, bytes | 116 | 116 | 0 |
| Scan state `ss_t`, bytes | 60 | 80 | +20 |

Both RAM proofs pass:75 listed functions in RAM,18424 bytes. No new task,
queue, heap allocation, ping limit or timer frequency. The diagnostic array
uses48 rows of12 bytes, plus two4-byte counters:584 B. Other growth is scan
state and expanded obstacle values. Geometry uses bounded libm calls after
completion; these remain in flash, not the RAM critical-function list.

ARM GCC `-fstack-usage` gives local frames, not whole call-chain bounds:

| Function | Baseline bytes | Candidate bytes |
| --- | ---: | ---: |
| `vehicle_task` | 1016 | 1016 |
| `scan_step_observed` | 64 | 200 |
| `obstacle_report_make` | 40 | 64 |
| `observed_clearance` | absent | 32 |
| `ss_step` | 56 | 48 |
| `telemetry_task` | 408 | 432 |

Vehicle and telemetry each retain4096 B allocated stacks. Local frames do
not prove free stack under nested kernel/libm calls. Read actual stack high
water, response times and overruns on the robot. The navigation task sends
one nonblocking116 B diagnostic value; it never prints the line itself.

## Stopped USB evidence

Each completed physical sweep, including a retry, enqueues one diagnostic.
A stopped GP20/DUMP prints one line per stored sweep:

```text
[S5_SCAN] id 7 attempt 1 coarse_hits 3 fine_mask 01b6 duration_ms 2560
```

`coarse_hits` counts coarse VALID samples within nearest coarse+150 mm.
`fine_mask` marks only newly measured fine-region angles, not reused coarse
samples or later compatibility-fill angles. Bit0 is−30 degrees; bit12 is
+30 degrees; add5 degrees per bit. Duration runs from stopped-wait entry to
completion after centre settling; reverse between sweeps is excluded.
Zero fine mask can be legitimate when no coarse hit defines a fine region.
The MQTT stage is final stage1 or2, not a live scan-progress stream.

Storage is bounded at48 lines per run, reset by TELEMETRY_BEGIN. Excess
lines produce `[S5_SCAN] omitted N`; a full message queue increments normal
lost telemetry. Reject evidence with omissions or unexplained telemetry
loss. Repeated DUMP repeats the saved evidence; it is not another scan.
USB scan lines are not new MQTT RAM-log record types.

## Operator stationary bench — not run

1. Use an operator-controlled stationary setup. Keep the wheels clear or
   arrange the course so motion cannot cause a collision. Use the normal
   sonar/servo harness and retain the mounted safe limits.
2. Present a broad face near centre, then a narrower/off-centre face. Capture
   USB, MQTT and the stopped dump. Record the actual face dimensions and
   distances in [MEASUREMENTS](../architecture/MEASUREMENTS.md).
3. Check one scan line per completed profile. A centre coarse hit should
   produce nonzero fine mask, no repeated coarse rays in that mask, and a
   sensible duration. Record duration, stack high water and lost/overrun
   counters. Abort for a sweep outside safe servo limits or unintended drive.
4. Move the face to a scan limit: expect clipping and lower-bound width or
   unknown width, with unknown outward clearance on that side. A single hit
   must not show zero-width certainty. Empty/invalid evidence must not show
   valid dimensions. Compare rough estimates only; no accuracy claim is set.
5. Confirm last-completed stage/attempt and closest sensor-frame point on
   the combined dashboard. A missing network must not affect scan decisions.

## Two-detour floor regression — not run

Use the operator's unchanged two-obstacle course and retained motor/LINE
settings. Capture one complete run with two attempted detours. For each,
check PROFILE free steps and chosen side, reverse/retry behavior, completion
and line reacquisition. Confirm no added forward motion during scan, no
missing scan lines, no new faults/overruns, and reasonable observed geometry.
Compare against the prior2/2 detour result without calling it a new pass.
Record the candidate SHA, both durations, outcomes, logs and anomalies here
and in the measurement ledger after the operator run.
