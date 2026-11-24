# Coarse/fine scan validation

## State

Floor check completed on staging `d44bd25`, school, 2026-11-17.
The operator replaced the separate stationary bench with one floor run.
Both encounters completed their detours; scan1 also covered clipped geometry.
This is a scoped functional result, not a width-accuracy or passability proof.
Module commits: `75c5d85`, `c2a9a6a`. Integration code: `3f569b2`.
The [scan record](OBSTACLE.md#2--fine-scan-and-decision-bin-completion)
defines the schedule; [geometry](OBSTACLE.md#geometry-estimates) defines the
assumptions. The [wire contract](../s1-telemetry/MQTT_WIRE.md#obstacle-profile)
defines v3 and legacy decoding.

Current (2026-11-24): coarse/fine scanning remains in the demo kernel.
Later image 30 run 6 completed two detours; image 31 demo-day detours ended
DONE_NO_LINE then line search. These runs do not turn the skipped A2/A3
placements into passes or prove width accuracy. See
[the demo record](../release/DEMO_KERNEL_V2.md).

## Host evidence

This evidence is for the scan-integration revision, not the current full suite.

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

Historical scan-integration comparison, not the current demo RAM budget.
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

## Robot floor result — 2026-11-17

Image: staging `d44bd25`. Location: school. One 90 s run met the same box
twice, not two distinct test objects. The operator supplied a box width of
210 mm and said it was not centred. The separate stationary bench was
replaced by this floor run. The operator skipped the separate A2/A3
placements because scan1 already covered the clipped case.

Source worktree: `~/Projects/mtk3-core-review`. Local logs, not committed:

- `build/school_stg_d44bd25/robot_STG_1.final.log`
- `build/school_stg_d44bd25/mqtt_STG_1b.csv` (new decoder)

The MQTT obstacle/detour records below share boot ID `4132446305`.
Other retained boot records in the CSV are not part of this result.
Repeated USB dumps repeat the same two scan IDs, not additional sweeps.
All robot rows below are **MEASURED(robot log,2026-11-17)**.
The box reference is **MEASURED(operator,2026-11-17)**, not a sonar result.

| Observation | Scan1 | Scan2 |
| --- | --- | --- |
| Closest sensor-frame range and angle | 233 mm at +5 deg | 221 mm at 0 deg |
| Estimated region width | 193 mm, lower bound | 174 mm, valid and not clipped |
| Geometry flags | 19: width/left span valid, right edge clipped | 7: width and both spans valid |
| Observed edge-plane spans | left24 mm; right unknown | left25 mm; right23 mm |
| Coarse hits | 4 | 3 |
| Fine mask | `0db6` | `0db6` |
| Completed sweep duration, including stopped wait | 2519 ms | 2509 ms |
| Final stage / attempt | 2 /0 | 2 /0 |
| Detour result | success, 17900 ms | success, 17950 ms |
| Maximum reported lateral distance | 291 mm | 309 mm |

```text
[S5_SCAN] id 1 attempt 0 coarse_hits 4 fine_mask 0db6 duration_ms 2519
[S5_SCAN] id 2 attempt 0 coarse_hits 3 fine_mask 0db6 duration_ms 2509
```

Mask `0db6` contains the eight non-coarse angles: −25, −20, −10, −5,
+5, +10, +20 and +25 degrees. Together with the five reused coarse angles,
this shows both stages ran and all13 original decision bins were covered.
Both completed on the initial attempt. The measured durations are below the
accepted2560 ms host case, but two observations do not establish a worst-case
robot bound. Settling and minimum60 ms ping spacing were not shortened.

| Run health / stack | Measured value |
| --- | --- |
| Completed detours | 2/2 successful |
| Vehicle overruns / lost wakes / faults | 0 /0 /0 |
| Lost telemetry | 0 |
| Maximum vehicle response | 2374 us |
| Vehicle task stack high water | 1712/4096 B; 2384 B free |
| Telemetry task stack high water after dump | 1096/4096 B; 3000 B free |

The initial dump reports telemetry high water864 B; use the later1096 B
value, which includes dump work. No scan omission line is present. The scan
widths are estimates of observed regions, not the operator's full box width.
In particular, scan1 must remain marked lower-bound with unknown right span.
No numeric width-accuracy threshold or guaranteed body clearance is claimed.

## Remaining scope

The separate A2/A3 placements were skipped by operator decision, not passed.
Single-hit, empty/invalid rays and their unknown geometry are covered by host
tests only, not this robot run. This run does not exercise reverse/retry,
DONE_NO_LINE or STALL_RIGHT. It does not prove all sonar surface/angle cases
or network-loss independence. The recorded two-detour floor regression is
successful for this run; retain these limits when reporting the result.
