# Motor rotation drift and counts-per-revolution record

## Result

The shared encoder value is **2535 counts per wheel revolution**.

| Motor | Combined count at visual alignment | Revolutions | Result |
| --- | ---: | ---: | ---: |
| Motor 1 | 126750 | 50 | 2535.000 counts/revolution |
| Motor 2 | 126747 | 50 | 2534.940 counts/revolution |

The 0.06-count difference is within the approximately +/-0.06-count
measurement precision. Use one shared value, 2535. The motor is unmarked and
has no available datasheet, so this is an empirical value.

The value is not divisible by four. This indicates that the encoder is on the
motor shaft before a non-integer gear ratio. A round ratio must not be assumed.

## Method

Each motor was tested separately with:

- 50 continuous forward revolutions.
- 10% duty.
- The wheel lifted and unloaded.
- The motor powered from the Raspberry Pi 400 USB supply.
- The stop decision made in the encoder interrupt.
- A deliberate 300-count undershoot before the expected visual mark.
- Forward-only, count-based jogging to the next visual mark.

The operator aligned a wheel-rim mark with a fixed chassis mark. The measured
value was calculated as:

```text
counts/revolution = (final absolute count + forward jog counts) / revolutions
```

The 3-count fine jog is intended for the final approach. Jogging in reverse is
not allowed because it engages gearbox backlash.

## Why 50 revolutions were used

An error in counts per revolution grows linearly with the number of
revolutions, while the operator's visual alignment error stays about constant.
The 50-revolution run turned an otherwise hidden 5-count error into about a
40-degree wheel offset. This made the error measurable without relying on an
estimated angle.

The count-domain drift value alone cannot validate counts per revolution. The
stop decision is triggered by encoder counts, so the final count is the target
plus coast, both in the same encoder domain. That result is circular evidence.
Only the physical wheel alignment is independent evidence.

## Superseded values

The earlier values of 2540 for motor 1 and 2539 for motor 2 are superseded.
They came from a chosen 2500-count stop threshold plus observed coast, checked
against a single-revolution visual alignment with about +/-20 counts of visual
resolution. That method could not resolve a 5-count error. Therefore, 2540 was
not contradicted by its data; it was too imprecise for this question.

The old 2540 value was high by 5 counts, or 0.197%. Over 10 metres, this is
about 2 cm of systematic error. The error accumulates and does not average out.

## Coast and jog measurements

Coast is a separate measurement. At 10% duty with the wheel lifted and the
Raspberry Pi 400 USB supply, measured coast was:

| Motor | Coast |
| --- | ---: |
| Motor 1 | 27–28 counts |
| Motor 2 | 23 counts |

Coast is per-motor, speed-dependent, and measured unloaded. It is expected to
be smaller under load. It is not a shared or fixed constant and must not be
used as a target correction without a new measurement under the same
conditions.

The count-based jog measurements were:

- A 3-count request moved 3–7 counts.
- A 40-count request moved 56–57 counts.
- The practical alignment floor is set by the stiction-breaking burst plus
  coast.
- Time-based 8–10 ms pulses did not move this motor reliably, so the jog is
  count-based.

## Speed observations

These unloaded measurements used 10% duty and the Raspberry Pi 400 USB supply:

| Motor | Mean time per revolution | Speed |
| --- | ---: | ---: |
| Motor 1 | 2491 ms/revolution | 24.08 RPM |
| Motor 2 | 2433 ms/revolution | 24.66 RPM |

Motor 2 was faster, consistent with the existing low-duty tables. Revolution
1 was about 60–90 ms slower than the later revolutions because the motor
started from rest. Both motors sped up slightly as they warmed. Motor 1 changed
from about 2602 ms/revolution to about 2530 ms/revolution.

Motor 2 produces negative encoder counts in the forward direction. Motor 1
produces positive counts. The combined-total calculation adds the signed jog
to the signed settled count and then takes the absolute value, so both
orientations work.

## Power and safety limitation

The motors were powered from the Raspberry Pi 400 USB supply for this
measurement. This is a bounded exception to the normal VIN procedure: one
motor at 10% duty with the wheel lifted was acceptable for this test. The
Raspberry Pi 400 USB ports still share a 1.2 A limit with the external SSD.
The firmware stall guard is required.

The available VIN source is an unmarked power bank. It cuts output after about
10 seconds of low current draw, so it would end this long test prematurely
and cannot be used for this measurement.

Do not generalise this exception to two motors, a loaded wheel, or higher duty.
See [USB power safety](USB_POWER_SAFETY.md) for the bounded exception and the
over-current recovery procedure.

## Follow-up work

- Re-measure coast under the robot's normal load.
- Confirm 2535 counts per revolution under load.
- Measure wheel slip separately because encoder counts cannot detect it.
