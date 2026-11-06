# Subsystem 3 kernel line acquisition

| ID | Procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S3-U01 | TEST=1, wheels lifted. Move a black/white card under GP27 L and GP26 C during the 1 s window. | Actual TIMER window count within 1%; both raw means 0..4095 and nonzero spread; no missed conversions/alarms | Not run |

Alarm 0 / TIMER IRQ0 runs on core 0 at priority 2, below GPIO IRQ13 at 1.
Each 1 ms sample is the integer mean of eight ADC conversions per channel,
L then C; right is zero, matching the bare-metal two-sensor path. A 128-slot
queue transfers timestamped samples to task context. TEST deliberately
leaves the queue undrained during measurement; queue drops in that window
do not mean acquisition missed samples. A robot run must have no new
queue drops or missed alarms.

The app releases ADC reset and selects PLL_USB=48 MHz / 1 for CLK_ADC.
The template ADC device `adca` stays unopened. No template device,
configuration or kernel source is changed. GP29/ADC3 is never selected.
