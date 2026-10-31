# Echo material model and recorded-scan fit limits

## Rule and settings

The model keeps the closed ±15° sector. `ideal` returns the nearest geometric
surface. Legacy `"echo": "ideal-sector"` and `"echo": "ideal"` remain valid.
New per-obstacle settings use:

```json
"echo": {
  "material": "specular",
  "max_incidence_deg": {"value": 20, "assumed": true, "note": "Conservative cut from the documented 20-25 degree flat-face loss; geometry is incomplete"},
  "corner_echo_max_mm": {"value": 0, "assumed": true, "note": "Off: no isolated corner-return range was recorded"}
}
```

For specular material, a surface point can return only when its outward
normal is within `max_incidence_deg` of P→sensor. This is equivalent to
comparing the incoming ray with the inward normal. The normal must face
the sensor. The nearest admitted point in the sector wins. A circle uses
its radial normal; its nearest radial point has normal incidence. A box
corner in the sector can return irrespective of incidence if its distance
is at most `corner_echo_max_mm`; zero disables this exception. Ideal
material ignores both settings. There is no random dropout.

The seven existing box scenarios and newly added editor obstacles default
to specular, 20°, corner range 0 mm. Both numeric defaults are **assumed**,
not measured constants. The editor can change them per obstacle. A new
`ultrasonic_no_echo` observation records the nearest sector surface when
all candidates fail incidence. The viewer draws a faint cross and label
at this sampled surface; it is not an echo hit or a firmware input.

## What the records constrain

[Scan run 1](OBSTACLE_SCAN.md#run-1-object-straight-ahead) records a face
about 165 mm away, visible from −25° to +20°, with rising end readings.
It explicitly does not record object type or width. The same record says
flat faces can lose returns beyond about 20–25°. We choose the conservative
20° end of that statement. It is not a fitted measurement of incidence.

[Bypass run 11](OBSTACLE_BYPASS.md#runs-911-2026-10-30-version-31-turn-more-when-not-seen)
records no echo at −20°, −25°, −30° after one 183 mm reading. The robot was
moving; face pose and incidence were not recorded. A servo angle is not a
face incidence. No isolated corner-return range is recorded, so corners
are off by default rather than inventing a range.

**These records do not identify a unique numerical fit. The specified
nearest-point sector model does not reproduce all the real readings.**
The reconstructions below expose the mismatch instead of adding angle
exceptions or changing firmware to obtain a result.

## Native-probe comparison

Reproduce with the existing native ultrasonic probe, not a second geometry
implementation:

```sh
python3 sim/tests/test_specular.py sim/build/robot_sim --fit
```

Distances are the real driver's integer pulse-to-mm results. NR means
NO_RETURN. Angles are commanded angles; the native probe uses the actual
rounded servo pulses.

### Scan run 1

Assume a thin flat face 165 mm ahead of the sensor, 44.116834 mm across,
centre y = −7.332559 mm, no yaw. Its end bearings (−10.1°, +5.1°) are chosen
only to match the recorded visibility bins with the ±15° sector. This is
not a recovered object width. The model sensor is at world (65, 0) mm;
box centre is (230.5, −7.332559) mm, size (1, 44.116834) mm.

| Angle | Recorded mm | Simulated mm |
| ---: | ---: | ---: |
| −30° | NR | NR |
| −25° | 187 | 168 |
| −20° | 175 | 166 |
| −15° | 166 | 165 |
| −10° | 166 | 165 |
| −5° | 162 | 165 |
| 0° | 166 | 165 |
| +5° | 167 | 165 |
| +10° | 171 | 165 |
| +15° | 183 | 165 |
| +20° | 196 | 166 |
| +25° | NR | NR |
| +30° | NR | NR |

Visibility matches, but the broad nearest-point plateau misses the
increase toward the ends: up to 30 mm low. Do not describe this as an
exact range fit or proof of the face geometry.

### Bypass run 11, later no-return readings

An illustrative box is 120 × 120 mm, centre (305.038168, −111.931636) mm,
yaw +20°. Its nearest corner is 180 mm from the sensor at bearing −25°;
the two adjacent faces are at 45° incidence there. These dimensions and
pose are assumptions, not a replay of the measured robot path.

| Servo angle | Recorded | Simulated |
| ---: | --- | --- |
| −20° | NR | NR |
| −25° | NR | NR |
| −30° | NR | NR |

This demonstrates slant loss at that distance. It does not reconstruct the
earlier 183 mm echo or identify how the face pose changed during motion.

### Bypass bottle run 1

Assume diameter 66 mm (the repository bottle estimate), nearest radial
range 186 mm, centre bearing +7.5°. Model centre is (282.126425,
28.585236) mm. The nearest range and bearing are selected from the valid
cluster; bottle pose during the real scan was not measured.

| Angle | Recorded mm | Simulated mm |
| ---: | ---: | ---: |
| −30° | NR | NR |
| −25° | NR | NR |
| −20° | NR | NR |
| −15° | NR | NR |
| −10° | NR | 187 |
| −5° | NR | 186 |
| 0° | 190 | 186 |
| +5° | 186 | 186 |
| +10° | 186 | 186 |
| +15° | 206 | 186 |
| +20° | NR | 186 |
| +25° | NR | 187 |
| +30° | NR | NR |

The bottle remains visible and its central distance matches, but simulated
visibility is too wide. A stationary normal-facing point remains visible
through a 30° sector sweep; an incidence cut cannot reduce that to the
recorded 15° valid span. The flat-face cutoff is therefore not a calibrated
bottle beam model. No random dropout or undocumented beam change is used
to hide this limitation.

## Checks

Six small unit checks cover a square face, a slanted face and saved
no-echo marker, a bottle, corner on/off/range boundary, ideal/legacy event
equivalence, and parameter limits. CTest on `sim` passes all six groups;
the optional browser suite is not run. Existing ideal-trajectory tests
now explicitly select ideal material, so their original expected steps,
phase sequence and hook equivalence stay checked. Default specular material
has its own unit checks. No firmware algorithm is changed on `sim`.
