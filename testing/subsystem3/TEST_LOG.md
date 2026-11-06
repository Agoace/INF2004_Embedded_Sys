# Subsystem 3 kernel line acquisition

| ID | Procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S3-U01 | TEST=1, wheels lifted. Move a black/white card under GP27 L and GP26 C during the 1 s window. | Actual TIMER window count within 1%; both raw means 0..4095 and nonzero spread; no missed conversions/alarms | PASS 2026-11-06: 1009 samples in 1 s, 0 missed (no card moved; spread from the floor only). [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |

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

## Task-split image checks

| ID | Procedure and acceptance | Status |
| --- | --- | --- |
| S3-U02 | TEST: line task processes timestamped samples, no queue drops; snapshot age <=10000 us; report task elapsed max/sample, total and per-mille window cost; report three-channel ISR max/total/per-mille and GP28 ADC2 range | PASS 2026-11-06: age <= 993 us, task 7–13 per-mille, ISR 69–75 per-mille, ISR max 114 us. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| S3-U03 | Bench only, do not drive: GP28 analog barcode input, VBAT jumper open; show bars at intended height and record raw range, narrow-bar visibility and ISR cost | Not run |

S3 now averages eight ADC conversions on each of L/ADC1, C/ADC0 and
barcode/ADC2 every 1 ms (24 total). Barcode is raw observation only; no
decoder or Ziven source is changed. Pure line logic receives L/C/zero,
never ADC2. See the [control architecture](../../docs/CONTROL_ARCHITECTURE.md)
for the GP28 wiring and bar timing constraints. ISR timing excludes wrapper
entry/exit and its final statistics stores; task timing is elapsed work,
including any higher-priority preemption, an upper bound on CPU cost.
