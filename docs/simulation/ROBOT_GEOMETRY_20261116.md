# Operator geometry update — 2026-11-16

This record applies to future simulation projects. Completed manifests, worlds,
calibration data and results remain unchanged. Main approved task A geometry
before three bounded straight-line simulations. No physical test is authorized.

## Measurement record

All values below came from the operator on 2026-11-16. `MEASURED` means an
operator ruler or scale value; it does not claim instrument calibration.
`PHOTO_ESTIMATED` values have about ±1 mm uncertainty. Coordinates are forward
and left from the rear driven axle; heights are above the floor.

| Quantity | Value mm unless stated | Type | Use |
|---|---:|---|---|
| Contact-centre track | 111–112; use111.5 | MEASURED | Driven-wheel separation |
| Wheel diameter | 65, confirmed | MEASURED | Radius32.5; loaded rolling radius unknown |
| Wheel width | 16 | MEASURED | Existing PROTO width16 unchanged |
| Outer wheel face to opposite outer face | 128 | MEASURED | Envelope;111.5+16=127.5 agrees within rounding |
| Body length | 180 | MEASURED | Collision-box longitudinal extent |
| Rear axle to body rear | 52–53; use52.5 | MEASURED | Body front127.5 and box centre37.5 derived |
| Total robot height | 134 | MEASURED | Envelope, not solid collision-box thickness |
| IMU height | 132 | MEASURED | Axle-relative height99.5 derived |
| Rear axle forward to caster | 82, confirmed | MEASURED | Existing caster model; trail/shape still assumed |
| GP28 module beyond left outer wheel face | 17–18; midpoint17.5 | MEASURED | Module envelope; not the LED centre |
| Rear axle forward to sonar back | about70±3 | MEASURED | Envelope check, not servo-axis location |
| Rear axle forward to sonar front | 85±3 | MEASURED | Face offset20 from retained servo axis65 |
| Rear axle forward to GP26/27 centre | about122 | MEASURED | Replaces119; yaw lever arm122 |
| Rear axle forward to GP28 | about73 | MEASURED | Replaces80±5 PHOTO_ESTIMATED on2026-11-11 |
| GP28 LED centre left of centreline | 73 | MEASURED | FINAL operator2026-11-16 clarification; replaces77 |
| GP26/27 LED-tip floor clearance | 9±1 | PHOTO_ESTIMATED | Replaces8 for task A |
| GP28 LED-tip floor clearance | 5.2±1 | PHOTO_ESTIMATED | Replaces5.5 |
| GP26/27 module centre spacing | 14.7±1 | PHOTO_ESTIMATED | Replaces14.5; left sensor from centre sensor |
| Total mass | 671.6g | MEASURED | Existing0.6716kg profile retained |

## Assumptions not replaced by these measurements

- **FINAL GP28 clarification before any task A run:** the operator specifies
  forward73mm **and** sideways73mm, both MEASURED2026-11-16. This supersedes
  the intermediate73/77,80/73 and80/77 interpretations. No task A run used
  those intermediate values. Do not infer LED centre from housing arithmetic.
- Collision chassis width160mm and thickness35mm remain ASSUMED. The previous
  width source described the total mount envelope, not the chassis. Total
  height134mm does not justify a solid134mm-tall body collision box.
- Body collision vertical centre, COM, inertia, component mass allocation,
  contact friction, caster trail and servo height remain unchanged assumptions.
- The old servo forward coordinate65mm was not remeasured. Sonar offset20mm
  therefore combines the new face measurement and that retained coordinate.
  The assumed sonar collision shape is not a new ruler model of the module.
- Firmware encoder scale12414counts/m and2535counts/revolution remain unchanged.
  No physical firmware geometry/config is edited by this simulation update.

The profile and PROTO defaults now agree on the changed coordinates. The
profile is the authoritative source for generated worlds. Frozen line optical
profiles and historical mount files are not rewritten. Task A must explicitly
project the new9mm clearance and122mm lever arm into its SIL sampler; changing
only the robot visual mount does not change the controller's optical input.
Any9/8 footprint scaling is an explicit angular-divergence assumption, not a
new optical calibration. The actual spin calibration retained from the old
geometry must be labelled historical, not a new9mm spin measurement.

Old removed hardware provenance is addressed as
`569672a:docs/s2-motion/HARDWARE.md`, not as a link to a current file.
No geometry value was selected to obtain a desired oscillation.
