# Barcode reading ideas

Status: **ideas only, 2026-11-06.** Nothing is built. The barcode code
(Ziven) is not changed. We take over the barcode work when we get to it.

This record explains two problems with the barcode read and lists ideas to
solve them. Use it as input when the barcode work starts.

## Known facts

| Item | Value | Source |
| --- | --- | --- |
| Barcode type | Code 39 | Barcode interface (Ziven) |
| Barcode location | Beside the line, on the left | Operator |
| Sensor location | Left front of the robot. It does not move | Operator |
| Sensor height | About 5.5 mm or more above the floor, so that it does not touch the ramp | Operator |
| Sensor output | Analog, GP28 (ADC2). See the [pin update](../archive/architecture/CONTROL_ARCHITECTURE.md) | Operator |
| Narrowest bar | About 5 mm (large barcode), 2–3 mm (small barcode) | Operator |
| Bar height | The sensor stays on the bars, also when the robot weaves | Operator |
| Content | Probably 3 characters for each copy, for example `*A*` (start, letter, stop) | Operator, not sure |
| Copies | 3 copies in alternate directions: forward, backward, forward, or backward, forward, backward | Operator |
| Commands | Turn left, turn right, go straight, U-turn | Project brief |

Unknown: the exact sensor position (mm ahead of the axle, mm left of the
centre line), the wide-to-narrow ratio, and the gap between copies.

## Problem 1: more than one bar in view

At 5.5 mm height, the sensor sees a spot that can be wider than a 2 mm bar.
A fixed digital switching level can then miss a narrow bar. Driving slower
does not help, because the blur comes from the spot size, not from time.

Decision: use the analog output on GP28, with a switching level from the
measured minimum and maximum. A blurred narrow bar still gives a small dip.

Planned check: a bench test that logs the analog value over a 2 mm bar and
a 5 mm bar at 5.5 mm height. This shows if the small barcode is readable.

## Problem 2: the weave of the line tracing

The robot weaves across the line, and the sensor is at the left front,
about 123 mm ahead of the wheels. The front of the robot moves more than
its middle. This causes three effects:

| # | Effect | Result |
| --- | --- | --- |
| 1 | The speed of the sensor point over the floor changes during a turn | The decoder measures bar width as time. A bar crossed during a slow swing looks wide. A bar crossed during a fast swing looks narrow |
| 2 | The angle between the sensor path and the bars changes | At angle θ a bar looks 1/cos θ wider. A constant angle is not a problem. A changing angle changes the width ratios |
| 3 | In a sharp turn the sensor point can move backwards | The sensor crosses a bar two times |

The sensor stays on the bars (see the known facts), so the sensor does not
leave the barcode at the side.

## Barcode length

A Code 39 character is about 13 to 16 narrow widths: 6 narrow and 3 wide
elements, and a narrow gap. This estimate uses a wide element of about 2.5
narrow widths.

| Narrowest bar | 1 copy, 3 characters | 3 copies |
| --- | --- | --- |
| 2 mm | About 90 mm | About 270 mm |
| 5 mm | About 225 mm | About 680 mm |

If each copy has 3 letters between the start and stop characters, the
barcode is about 1.7 times longer.

## Ideas

### Reduce the weave

| Idea | Method | Note |
| --- | --- | --- |
| Calm steering | In the barcode zone, follow the line with a lower gain and a smoothed steering command | Possible because the barcode is beside the line: the line sensors stay on the line |
| Copy 1 as trigger | The first copy starts the barcode mode: slow speed and calm steering. Copies 2 and 3 are then read in better conditions | Copy 1 is also decoded, with less trust |
| Less weave in general | Improve the line tracing settings | Open work. It helps all parts |
| Rejected: hold the heading | Stop the steering and drive straight on the encoders over the barcode | Rejected: 270–680 mm without line feedback is too long. The robot can drift off the line |

### Measure distance, not time

| Idea | Method |
| --- | --- |
| Distance tags | Give each sample the distance that the sensor point moved. Calculate it from the two wheel speeds, the turn rate and the sensor position. Then the bar widths are in mm. They do not change with the speed changes from the weave |
| Projection | Use only the movement along the line direction. Then a change of angle does not change the measured width |

Before the sensor position is measured, use an estimate from the robot
dimensions (front 123 mm, half-width 80 mm). To calibrate the position,
measure one bar of known width two times: in a straight drive and in a
turn. Change the position until the two widths are equal.

### Make the decoder tolerant

| Idea | Method |
| --- | --- |
| Compare in each character | A Code 39 character has 9 elements and 3 of them are wide. Sort the 9 widths. The 3 largest are wide. This works for each wide-to-narrow ratio, and slow changes of scale have no effect |
| Only valid commands | Compare the result with the 4 commands only. Reject all other results |
| Record, then decode | Record the profile, tagged with distance, to the quiet zone after the barcode. Then decode all of it. Optional: compare it with the 4 expected profiles with dynamic time warping (a method that stretches and compresses one profile to fit another) |
| Use the start and stop character | Each copy starts and ends with `*`. `*` is not symmetric, so it shows the direction of each copy. A gap between copies is then not necessary |
| Vote | Decode each copy alone. Accept a command only if 2 of the 3 copies agree |
| Direction check | The direction must change from each copy to the next one (forward, backward, forward or backward, forward, backward). Other orders show a read error. The two orders are the same barcode read from the two ends, so the robot can come from the two sides |
| Reject | If the distance goes backwards, if 2 copies do not agree, or if the result is not a command: report "unreadable". Continue to follow the line. Do not turn on a guess |

The direction of the copies also shows the direction in which the robot
crossed the barcode.

### Rejected: move the sensor

A sensor near the centre of the axle has no side movement from a turn. The
operator prefers to keep the sensor at the left front.

## Proposed chain

```text
* X *   * X *   * X *     each copy framed by *, its direction from the *
  |       |       |
copy 1: start barcode mode (slow, calm steering), decode with less trust
copy 2: decode, widths in mm
copy 3: decode, widths in mm
  |
2 of 3 vote + direction check + must be 1 of the 4 commands
  |
command, or "unreadable" (continue to follow the line)
```

## Open items

1. Measure the sensor position: mm ahead of the axle, mm left of the centre line.
2. Find the wide-to-narrow ratio and the gap between copies from a real barcode.
3. Confirm the content of each copy: `*X*` or more letters.
4. Do the bench test of problem 1 at 5.5 mm height.
5. Update the barcode interface for the analog input. It now reads digital GP7.
