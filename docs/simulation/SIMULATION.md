# Current host simulation and evidence limits

## What it is

The separate host simulator compiles the selected bare-metal sources from
the checkout, including `src/detour_test.c` (currently v1.8), not the kernel
robot. The recorded milestone and box-sweep results below used the earlier
v1.5 snapshot b9dcb6f; the build does not pin or restore that snapshot.
Milestones 1–4, timed ultrasound, call observations, offline playback and
the optional local editor are implemented. Full contact response is not.

The simulator compares firmware under explicit model assumptions. A complete
40 s trace means the scenario reached its deadline, not that DETOUR passed.
It does not validate Pico IRQ latency, kernel scheduling or physical safety.

## How it works

| Layer | Implementation |
| --- | --- |
| Firmware | Selected portable modules and real control loop; no simulator steering or outcome decision |
| Devices | Replacement motion, ADC, servo, ultrasound and local log sink; no real Wi-Fi |
| Clock | Integer simulated microseconds; default 100 us clock-read quantum, 50 us sensitivity comparison |
| Motion | Differential drive, separate wheel curves, fractional counts and first-order coast |
| IR | Line-band/footprint overlap to raw ADC; firmware performs calibration and line decisions |
| Ultrasound | Nearest admitted surface in a sector; timed results, trigger-gap and fault injection |
| Observation | Per-source external-call hooks; call real function once, preserve arguments/return |
| Trace/viewer | Saved world pose and firmware estimate shown separately; inferred phase is labeled |
| Contact | Conservative both-wheel latch; no separating motion after contact |

The rear axle is the origin: local x forward, y left, positive heading
left. A hook records call completion, not a private assignment inside the
caller. Hooked/unhooked comparisons cover native non-hook events. Do not
infer success from text alone or use a decoder to control the robot.

## Interface and operation

Use the standalone build and scenario runner from the repository root:

```sh
cmake -S sim -B sim/build -DSIM_FIRMWARE=detour_test
make -C sim/build -j4 --output-sync=target
sim/build/robot_sim --scenario sim/scenarios/box_straight.json --output sim/output/box
ctest --test-dir sim/build --output-on-failure
```

[sim/README.md](../../sim/README.md) is the command/schema/trace reference.
[`sim/targets/`](../../sim/targets/) selects sources and hook boundaries;
[`sim/core/`](../../sim/core/) owns clock/world/scenario logic;
[`sim/devices/`](../../sim/devices/) owns replacements. Firmware headers
remain the contract; unknown SDK dependencies must fail the host build,
not silently receive a success stub. No Pico SDK startup/runtime is linked.

Scenarios specify geometry, assumptions, units, buttons, seed and fault
events. A guessed value carries `value`, `assumed` and `note`; the trace
and viewer retain these. Missing values are not silent defaults. Traces
keep source hashes, settings, exact-time commands/echoes/hooks/text and
10 ms world snapshots. All generated output belongs in ignored `sim/output/`.

Open generated `run.html` directly from a file URL. It embeds assets/data
and needs no server, network request or adjacent JSON. Regenerate with
`python3 sim/viewer/render.py sim/output/box/trace.json`.
The optional `sim/viewer/serve.py` is a loopback-only environment editor,
not a live robot controller: validate draft, batch run, replay. It cannot
change firmware or robot-model settings. Do not expose it through a proxy.

## Settings and model assumptions

| Quantity | Value and status |
| --- | --- |
| Track / wheel | 111 mm / 65 mm diameter, operator dimensions |
| Encoder / circumference | 2535 counts/turn / 204.204 mm, shared firmware calibration |
| Body rectangle | Conservative 160 × 190 mm; front x 123, rear −67, half-width 80 mm |
| Sensor axis / IR point | x 65 / 119 mm; axis-to-face offset 0 is provisional; board tip x 123 |
| Alternate front offset | 112 mm sensitivity case; 58 mm echo offset is not ruler-confirmed |
| Tiles | Published 315 × 315 × 10 mm, black 18 mm line; flat model, no ramp physics |
| Tile curve | Assumed quarter circle, radius 157.5 mm |
| Pair scenarios | 14.5 mm spacing; assumed 16 mm spot; `gap_check: false` |
| Earlier three-sensor scenarios | Assumed 22 mm spacing / 3 mm spot with gap consistency check; not current robot wiring |
| Reference ADC endpoints | Physical historical L 202/1645, C 185/2615, R 191/3424 (floor/line); old pin layout, not current three-line-sensor wiring |
| Motor profiles | Corrected unloaded table, provisional-loaded transform, optional loaded-linear |
| Provisional loaded curve | Zero shifted to 12%; symmetric reverse; 50 ms coast/acceleration assumption |
| Loaded-linear option | 740 mm/s at 100%, zero 12%, 50/50 ms response: operator recollection plus assumptions, not logged calibration |
| Unloaded fitted coast | 24.744078742 / 20.466978133 ms from 27.5/23 coast counts; mixed USB/VIN evidence, not measured time constants |
| Integration / contact | At most 100 us motion substep; conservative contact bound within 2 us; latch stops pose/counts |
| Servo | 1100–1900 us clamp, 1500 centre, positive left; no measured slew model |
| Ultrasound | Nominal full 30° sector (allowed 0.5–30°); fixed 215 us rise, 10 us trigger, 2/3 ms timeouts, 60 ms gap |
| Result | Same 16-bit status/range contract: below 60 mm TOO_NEAR, above 511 mm unknown NO_RETURN |

No slip, tyre deformation, battery sag, acoustic multipath or 3D hump
physics is modeled. Native probes verify the implemented equations, not
the truth of these assumptions. The full speed table, correction factors,
coast equations and probe outputs remain in the
[original simulation record](../archive/simulation/SIMULATION.md#motors-counts-and-coast).

## Results

### Echo fit summary

Specular material admits a surface only within its incidence cut; corner
returns have a separate range limit. Existing boxes assume a 20° incidence
cut and zero corner range. Ideal/legacy sector echoes remain selectable.
There is no random dropout added to force a fit.

| Comparison | Result / limit |
| --- | --- |
| Scan run 1, face about 165 mm away | Assumed face can match visibility bins; nearest-point plateau misses end ranges by up to 30 mm |
| Bypass run 11, slanted face | Model can show no return at −20/−25/−30°; unmeasured pose means this is not a recovered trajectory |
| Bottle, assumed diameter 66 mm | Central range about 186 mm matches; simulated visibility too broad for recorded 15° span |
| Narrow-box scenario | Assumed full beam 5°, incidence 16°, corner range 300 mm, box 160 × 250 at (800,−20); free steps R6/L4 match R2, not a measured beam calibration |
| Narrow-box clearance | Fixed 25° back-off, straight settings 350/400/450/500 mm give modeled gaps 22/54/91/91 mm; 40° at 350 contacts; duty model, not kernel timing |

[Full echo-fit tables](../archive/simulation/SIM_ECHO_FIT.md) preserve every
recorded/simulated angle, geometry assumption and mismatch. The
[original narrow-box section](../archive/simulation/SIMULATION.md#narrow-beam-real-box)
retains the matching profile. A fitted narrow beam must not replace the
real robot's wide-cone/reverse-rescan finding.

### Box sweep summary

The seven cases use the fixed v1.5 firmware/model snapshot, not current
SCAN reattempt. Front face x is 740 mm, box centre y is zero, scenario
end 40 s. Every STOP occurs at 7.681700 s with a 224 mm sensor reading.

| Box width × depth mm | Side | Straights | Firmware outcome | Firmware end s |
| --- | --- | ---: | --- | ---: |
| 120 × 120 | LEFT | 3 | TIME | 34.810300 |
| 120 × 240 | LEFT | 3 | TURN_LIMIT | 34.790300 |
| 120 × 480 | LEFT | 2 | TURN_LIMIT | 26.800300 |
| 240 × 120 | NONE | 0 | NO_SIDE | 9.810200 |
| 240 × 240 | NONE | 0 | NO_SIDE | 9.810200 |
| 240 × 480 | NONE | 0 | NO_SIDE | 9.810200 |
| 360 × 480 | NONE | 0 | NO_SIDE | 9.810200 |

All seven have no contact, stall or DONE; no successful line rejoin can be
reported. `python3 sim/box_sweep.py --run` runs the size sweep against the
checkout; do not expect the historical table from changed firmware.
[Full sweep record](../archive/simulation/SIM_BOX_SWEEP.md) retains scenario
names, exact headings, console, hook times and viewer captures.

### Other retained checks

| Evidence | Result and scope |
| --- | --- |
| No-box reference | 30 s following, no search/contact; axle lateral error max 10.066755148 mm, mean 9.513485314 mm under stated assumptions |
| Hooked reference | Eight detour transitions match eight printed steps; hooked/unhooked non-hook events equal |
| Contact-latch reference | Contact at 8.542512 s; real firmware later reports stall at 9.042200 s; not full contact dynamics |
| Offline viewer reference | 4001 world samples, 7590 commands, 2718 detour observations; HTML 1868873 bytes; zero browser errors/network requests in recorded file-URL test |
| Historical simulator checks | Six CTest groups passed in the echo-fit record; rerun against the checkout, and run optional browser checks separately |

## Limits and links

Current kernel SCAN reverse/reattempt and real-time task behavior are not
simulated by this target. Old milestone TIME and later box-sweep NO_SIDE
results use their stated material/snapshot settings; do not combine them
into one claimed replay. Synthetic pose and integer firmware pose may
diverge; the viewer displays both without correcting either.

- [Original complete simulation design/results](../archive/simulation/SIMULATION.md) retains every measured value, equation, exact pose and sensitivity result.
- [Current obstacle record](../s5-obstacle/OBSTACLE.md) describes physical wide-cone evidence and M2 reattempt scope.
- [Current line record](../s3-line-barcode/LINE.md) distinguishes locked duty settings from kernel P7.
- [Integration TEST_LOG](../../testing/integration/TEST_LOG.md) and [physical results](../../testing/integration/KERNEL_ROBOT.md) are hardware evidence, not simulator outputs.
