# Project requirements: working summary

## Public summary and local course files

This is the team's own navigation summary, not a copy of the course material.
The repository is public. Only this README is tracked in this directory.
Course PDFs and Word files belong in the local ignored directory; do not add
or force-add them to Git, reproduce their pages here, or publish private logs.
Use the original course material to resolve a requirement or marking detail.

| Local document | What to check there |
| --- | --- |
| Project write-up | Required robot capabilities and the written submission |
| Project briefing | Delivery scope, demonstration arrangements and assessment context |
| Barcode and line specification | Track layout, barcode geometry and line-following constraints |
| Barcode sample | Example pattern for sensor and decoder checks |
| Change Requirement 1 (CR1) | Compact representations and the valid range of each shared value |

## Requirements to retain during implementation

- The course demonstration is in week 10. Check the current course notice
  for the exact date, required evidence and submission instructions.
- The robot must read the specified barcode and follow the track. The
  reference barcode is approximately 141 mm long, with a 20 mm gap to the
  line; the line is 18 mm wide. Check the original drawing before changing
  mounting or distance assumptions. These are task dimensions, not a claim
  about a measured sensor footprint or decoder performance.
- The course includes speed humps of varying heights. Detecting only one
  fixed test hump is not sufficient: the robot must report the highest hump
  peak it encounters. The small hump used in bring-up is test equipment,
  not the full allowed range or an approved detection threshold.
- Choose data widths from the values that the application permits. Keep
  units, sign, unavailable values and reserved bits explicit. See the team's
  [CR1 interpretation](../architecture/week8_change_requirement_1_notes.md).
- Distinguish course requirements, design decisions and hardware evidence.
  A host test or successful build does not establish that a physical task
  has passed. Record the tested revision and the operator result.

## Implementation navigation

Use the [documentation index](../README.md) for the current subsystem records
and the [agent map](../AGENT_MAP.md) for source locations. The
[IMU record](../s4-imu/IMU.md) explains the measured acceleration bias
and why calibration and hump thresholds still need separate work.
