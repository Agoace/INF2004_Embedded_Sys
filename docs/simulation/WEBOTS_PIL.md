# Webots PIL host implementation

## Status and limits

Stage 0 is host-only. The branch starts at `451e562`. No Pico is flashed and no
serial port is opened by these tests. A software result is not Probe timing,
physical sensor accuracy, stack, CPU, deadline, stop latency or safety acceptance.

The current implementation contains an immutable optical scene, explicit robot
profile, duty-driven plant, PC wire codec, conservative clock bounds and bounded
host session/transport core. Course/world generation and a robot PROTO are the
next implementation slice. The operational serial worker and a Pico sensor/edge
runtime are not implemented here. Do not treat the host state machine as firmware.

SW6 owns the frozen PIL v1 protocol and C codec. SW4 owns `pil/s3_*` geometry,
optics and assets. SW5 owns other `sim/webots/` source files. Main owns integration,
shared documentation indexes, images and hardware operations. No file under
`app_program/` changes in this work.

## Run host tests

From the repository root:

```sh
PYTHONPATH=sim/webots python3 -m unittest discover -s sim/webots/tests -v
```

The tests use the Python standard library and do not need Webots or a serial
package. The independent SW6 golden fixture is copied without changes into
`sim/webots/tests/fixtures/pil_wire_v1.json`. The cross-codec tool requires that
this copy is byte-identical to the authoritative fixture in the codec checkout.
It compiles the real C source on the host, not a replacement C decoder:

```sh
python3 sim/webots/tools/check_c_interop.py \
  --codec-root /path/to/checkout-with-SW6-codec \
  --output build/webots/c-interop
```

Initial validation: 47 host tests passed; 3026 C/PC comparisons passed using
26 golden vectors plus 3000 CRC-valid semantic mutations. Invalid C decode left
its destination unchanged. These numbers describe this slice, not hardware or
Webots execution. Local logs are in `build/wip/pil_plan/`.

## Entry points

| File below `sim/webots/` | Purpose |
| --- | --- |
| `pil/scene.py` | Convex planar surfaces, oriented boxes, spatial broadphase and immutable first-hit snapshots. |
| `pil/robot_profile.py` | Required numeric parameters, sources and assumption flags; mount-coordinate conversion. |
| `calibration/robot_assumed.json` | Explicit provisional model, not a calibrated robot. |
| `pil/motor_plant.py` | Loaded DM1 interpolation, assumed torque/coast response, wheel-count quantization and servo slew. |
| `pil/wire.py` | COBS, CRC32C, explicit LE fields, payload/range validation and bounded incremental decoding. |
| `pil/clock.py` | Offset intervals, drift expansion, fixed apply timeline and lookahead checks. |
| `pil/transport.py` | Host control lifecycle, bounded queues, leases and physics-applied acknowledgements; no actual serial I/O. |
| `tools/check_c_interop.py` | Real C/PC interoperability and CRC-valid semantic mutation checks. |

## Coordinates and scene API

Use ENU: world X east, Y north, Z up. Body x is forward, y is left and z is up;
yaw is left-positive. JSON/model geometry uses millimetres. Webots uses metres
and radians. The rigid body frame origin is the driven axle midpoint, at wheel
centre height. With nominal 65 mm wheels, it is 32.5 mm above a flat supporting
surface. For a surveyed flat sensor clearance, subtract that radius exactly
once to obtain axle-relative mounting Z. The sensor does not follow the terrain.

Support Z is zero and nominal EVA top is Z=10 mm. Tile size is 315 x 315 x 10 mm;
line width is 18 mm and nominal curve radius is 157.5 mm. SW4 canonical geometry
is the only source for optical strokes and assets; do not recreate its masks.

`SceneSnapshot.from_patches(patches)` copies structural SW4 `PolygonPatch`
fields `surface_id`, `material_id`, `vertices_mm` and `layer`. Convex planar
vertices are world mm and CCW as seen from the outward normal. Tilted triangles
are supported. Unsupported, non-planar, non-convex and duplicate-ID surfaces
are rejected at construction. Scene snapshots contain no mutable source list.

`first_hit(origin_mm, direction, max_distance_mm)` always returns `SurfaceHit`.
It normalizes a nonzero finite direction and returns physical mm distance.
Successful fields are `point_mm`, `normal`, `distance_mm`, `material_id`,
`local_uv_mm`, `surface_id`, `valid=True`, `reason=""`. Invalid fields are `None`,
`valid=False` with `miss`, `invalid_ray` or `ambiguous_material` reason. Never
convert a miss into a white floor reading.

UV origin is patch vertex zero, U follows its first edge, and V is normal cross
U. It is not necessarily tile-centred UV. Nearest physical surface wins. Within
1e-7 mm distance, higher material layer wins; different materials at the same
layer are ambiguous. Same-material ties select the lowest surface ID. Layers
are optical metadata, not raised collision surfaces. Material IDs include
`MAT_WHITE`, `MAT_LINE`, `PAPER_WHITE`, `PAPER_INK`; opaque boxes may use `BOX`.
The scene returns that material without inventing an optical ADC calibration.

## Assumptions to replace with operator inputs

All entries in `robot_assumed.json` retain a value, source and `assumed` flag.
The following table is the replacement checklist. A default value permits host
model tests; it does not make the value measured or physically accepted.

| Quantity | Current value or rule | Evidence limit / required input |
| --- | --- | --- |
| Track / encoder ratio | 111 mm / 2535 counts per revolution | Operator track and unloaded 50-turn ratio; not tyre slip acceptance. |
| Wheel / distance conversion | Nominal 65 mm / firmware 12414 counts per metre | Effective loaded radius and ground travel are unmeasured. |
| Total mass | **ASSUMED 1 kg** | Weigh the robot with its current battery and fixtures. |
| Wheel / caster / head mass | **ASSUMED 40 g each / 20 g / 20 g** | Measure component mass or approve the approximation. |
| Body envelope | 190 x 160 mm; front reach **ASSUMED 123 mm** | Full width is conservative, not a chassis CAD model; front offset is uncertain. |
| Body height / COM | **ASSUMED** 35 mm box, box centre Z25 mm above axle; COM x30/Z20 mm | Survey mass distribution and actual collision shape. |
| Caster | **ASSUMED** x80 mm, radius10 mm | Retained WB80 is a hypothesis, not a caster survey. |
| Contact / inertia / torque | **ASSUMED** friction0.8, wheel inertia2.1125e-5 kg m², torque cap0.12 Nm | Loaded transient and contact measurements are absent. |
| Motor response | **ASSUMED** drive/coast50 ms; static threshold12 percent | DM1 used a12 percent kick; static breakaway and time constants were not measured. Zero drive means passive coast, not active braking. |
| Motor running map | DM1 loaded VIN, separate wheels/directions, 0–70 percent | Recorded steady speeds; not the old unloaded host table. |
| High-duty model | **ASSUMED / UNVALIDATED** linear extension of60–70 percent slope up to80 percent | High-duty wheelie/unsettled rows are excluded. Larger duty fails model validation; this does not raise firmware drive permission. |
| Servo / sonar | Axis x65 mm measured; axis Z20 mm and face offset0 **ASSUMED**; slew277.78 deg/s **ASSUMED** | Settle fit is not a measured angular response or face survey. |
| GP26/GP27 line pose | x119 mm / clearance9 mm **ASSUMED**; separation14.5 mm physical | Do not relabel old fitted20.9 mm spacing or derived x123 mm as a survey. |
| GP28 pose | x80 +/-5 mm approximate; y77 mm left; flat clearance5.5 mm nominal | Standoff response outside calibration is unvalidated; no terrain-following mount. |
| IMU pose | x60 mm and axle Z97.5 mm **ASSUMED** | Assumes retained130 mm height was floor-relative; original datum is unconfirmed. |
| Magnetic field | **ASSUMED** world0/0.3/-0.4 gauss | Illustrative strength/inclination, not local calibration. |
| Encoder phase | **ASSUMED** nearest/ties-away half-count phase | Counts derive from joint rotation, not ground displacement; fractional phase is retained. |
| Home mats / props / station | Await confirmed layouts and dimensions | Future illustrative layouts must carry an assumption label, not claim a survey. |

The plant returns torque, not requested wheel speed. It does not duplicate the
real firmware PI, startup kick or rise limiter. Encoder endpoint differences
above16 counts per wheel per1ms slot invalidate the model, not create an
unbounded simulated-edge loop. Servo slew uses <=1ms steps.

## Wire and timing boundary

The SW6 record `docs/simulation/PIL_PROTOCOL.md` is authoritative once integrated.
Version1 uses COBS + zero delimiter, CRC32C, schema50494c01, 40 B SENSOR and32 B
OUTPUT payloads. SENSOR is66 wire bytes and OUTPUT58 bytes: at9216008N1 they
consume71.6 and62.9 percent respectively, in separate FULL-DUPLEX directions.
Those rates do not prove USB/Probe latency. UART carries no ASCII console.

Kernel TIMER stays real. Fixed total source-to-Pico apply delay is4ms, source
expiry5ms, and endpoint availability must be <=3ms from source capture. The
endpoint supplies simulated Gray transitions in the PRECEDING1ms interval; late
edges are never compressed or replayed. Pure simulated pending edges use no PIO.
A1ms physics step does not imply strict hardware lockstep.

The PC codec is stateless. `HostTransport` additionally checks session/nonce,
handshake, neutral initial frames, progressing snapshots, bounded queues and
leases. ARM requests enter pending; only matching ARMED OUTPUT acknowledging
SENSOR0 or later accepts the run. GPIO key pulses eventually need100ms minimum.
No navigation command is synthesized by the transport. Skeleton capabilities
cannot arm. Reset/new session is required after a fault; no automatic reconnect
or rearm is implemented.

SYNC uses conservative offset intervals, intersections of eight observations
and100ppm expansion, including observation-span rounding. It never divides
jitter by sqrt(N). The candidate250us half-width and phase limits can fail.
A refresh must arrive within1s AND before uncertainty headroom expires. No
minimum serialization subtraction is used without a separate proven bound.
The timeline epoch never slides when clock estimates change.

PC writes and actual device staging are different events. The reference
`validate_available` requires actual fake/device availability time; it cannot
infer it from a completed PC write. A device timestamp/diagnostic is needed for
Gate1 evidence. `physics_applied` is explicit and only then advances SENSOR ACK.
Repeated OUTPUT reports can have unchanged10ms actuator timestamps; leases use
new1kHz snapshot time and sequence instead. Duplicate or stale reports never
renew progress. Raw Pico PWM and a PC model-local safety zero must remain
separate observations.

Evidence queues fail closed when full. The caller must drain them to retained
logs. A blocked/paused controller must inhibit before advancing physics;
best-effort DISARM must not delay local inhibition. The serial worker must not
call Webots APIs. Corrupt packets are discarded until a delimiter; there is no
search inside a damaged packet for a hidden command.

## Remaining work and hardware gate

Course generation, robot PROTO and shared S3 integration follow this core slice.
Scenario verdicts must use only firmware MQTT/USB events and declared expected
sequences/counts/reasons/bounds. Simulator pose, optical truth or contact cannot
manufacture PASS. Keep simulator diagnostics separate. Default live actions stay
disabled; optical source2 is SHADOW. No SIM approval authorizes physical actions.

Gate1 waits for the operator and hardware. It must measure actual Probe baud,
full-duplex load, source/apply ages and uncertainty, native alarm jitter, queue
fill, clock drift, maximum—not only percentile—latency, sustained10-minute runs,
pause/unplug/fault/wrap and reset/no-auto-rearm. Stop and report a missed bound;
do not enlarge4/5ms limits silently. No hardware gate is closed by Stage0 tests.
