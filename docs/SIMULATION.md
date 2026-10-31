# 2D firmware simulation design

## Status and purpose

Milestones 1–4 are implemented in [`sim/`](../sim/README.md): host execution,
motion/IR, servo, timed sector echoes, module-call observations, the real
scan/detour loop, and the self-contained offline viewer. An optional local
environment editor now runs that same built target and opens its saved viewer.
It cannot change firmware, model, or control values. The target remains unchanged `detour_test` v1.5 at `b9dcb6f`.
No `src/` or firmware CMake changes were made.

Real GP21 calibration succeeds. The no-box scenario follows for 30 s without
searches. With the box on the line, the firmware now enters FOLLOW → STOP →
SCAN → DETOUR → FAILED (its own 25 s detour time limit), without contact.
No simulator logic steers, chooses a side, or decides detour success. See the
[milestone 3 results](#milestone-3-results) for times, poses, and limitations.
A complete trace means the scenario deadline, not detour success.

Hook observations include exact module state and live world pose. The
separate phase decoder marks inferred/caller-pending boundaries and idle
intervals. The eight hooked step transitions match all eight printed steps;
module poses/headings also match the prints. A separate unhooked build gives
identical native non-hook events, including commands, sensor results, text,
and snapshots. The runner now writes `run.html` beside every `trace.json`,
including incomplete runs. It shows true world pose and the saved firmware
estimate in separate colours. The heading gap is displayed, not investigated
or corrected. Full contact response and fault dynamics remain later work;
contact still uses the milestone 2 safety latch. See the
[host instructions](../sim/README.md) for opening and regenerating the viewer.

The complete design has two parts: a host program that runs firmware against
a virtual floor, and an HTML viewer that plays the saved run.

## Separation from firmware

- Put all simulator code, scenarios, tests, build files, and generated files
  under a new top-level `sim/` directory. This note stays in `docs/`.
- Treat `src/` as read-only input. Do not change firmware sources for a test.
- Do not add simulator dependencies to firmware targets or root CMake files.
- Build with the host C compiler, not the Pico cross compiler or Pico SDK.
- Select firmware sources through a target description. The world, clock,
  trace writer, and viewer must not contain detour or bypass decisions.
- Use a target-specific trace decoder only to label existing firmware output.
  It must not drive the robot or decide when the algorithm has succeeded.

### First source set

The source list comes from the `detour_test` target in
[`CMakeLists.txt`](../CMakeLists.txt). Preserve these files unchanged:

| Sources from `src/` | Role |
| --- | --- |
| `detour_test.c` | Entry point and full control loop |
| `obstacle_detour.c`, `obstacle.c`, `obstacle_profile.c` | Detour, forward watch, profile logic |
| `obstacle_scan.c` | Real timed scan; only its clock and devices are replaced |
| `line_steer.c`, `line_position.c`, `line_spin.c` | Real steering, line processing, and spin calibration |
| `steer_log.c` | Real packet packing |

Replace only `drive.c`, `ultrasonic.c`, `servo.c`, `line_sensors.c`, and
`wifi_log.c` with simulator implementations of their existing headers.
Do not link both versions of a driver. Keep the firmware headers unchanged.

A later target description can select `bypass_test`, `steer_test`, or a new
entry point and its non-hardware sources. It also declares extra SDK calls
and hardware replacements. A target that needs flash, for example, needs
an explicit flash device model; it must not silently receive a success stub.

## SDK surface

This inventory is for the source set above, including the five original
drivers. Calls inside a replaced driver do not need host SDK stubs. They
are listed to make the replacement boundary explicit. Standard C functions
such as `printf`, `memcpy`, and `abs` are not Pico SDK functions.

### SDK calls kept in the host build

| Group and headers | Symbols used | Host action |
| --- | --- | --- |
| Time: `pico/stdlib.h`, `pico/time.h` | `absolute_time_t`, `get_absolute_time`, `to_ms_since_boot`, `time_us_32`, `time_reached`, `delayed_by_us`, `make_timeout_time_ms`, `sleep_ms`, `sleep_us` | Fake integer microsecond clock; preserve deadline and 32-bit wrap semantics |
| Buttons: `hardware/gpio.h` | `gpio_init`, `gpio_set_dir`, `gpio_pull_up`, `gpio_get`, `GPIO_IN` | Setup records pin configuration; `gpio_get` reads scenario GP20/GP21 levels, active low |
| Console: `pico/stdlib.h`, `pico/stdio_usb.h` | `stdio_init_all`, `stdio_usb_connected` | Init succeeds without a device; connection is a scenario value, default false; capture text independently |

The direct users are `detour_test.c`, `line_spin.c`, and
`obstacle_scan.c`. Other retained modules use standard C and portable local
headers. `NULL`, integer types, booleans, and `_Static_assert` come from C,
not a separate simulated peripheral.

### SDK calls removed with hardware drivers

| Driver and headers | Complete used surface | Treatment |
| --- | --- | --- |
| `drive.c`: `hardware/gpio.h`, `hardware/pwm.h`, `hardware/sync.h` | `gpio_get`, `gpio_init`, `gpio_set_dir`, `gpio_disable_pulls`, `gpio_set_function`, `gpio_set_irq_enabled_with_callback`, `gpio_set_irq_enabled`; `GPIO_IN`, `GPIO_FUNC_PWM`, `GPIO_IRQ_EDGE_RISE`, `GPIO_IRQ_EDGE_FALL`; `pwm_config`, `pwm_get_default_config`, `pwm_config_set_clkdiv`, `pwm_config_set_wrap`, `pwm_init`, `pwm_gpio_to_slice_num`, `pwm_set_gpio_level`; `save_and_disable_interrupts`, `restore_interrupts` | No PWM or encoder IRQ emulation initially. Fake `drive_*` commands wheels and returns signed counts |
| `ultrasonic.c`: `hardware/gpio.h`, `hardware/irq.h`, `hardware/sync.h`, `pico/stdlib.h` | `uint`, `__not_in_flash_func`; `gpio_init`, `gpio_set_dir`, `gpio_put`, `gpio_get`, `gpio_disable_pulls`, `gpio_get_irq_event_mask`, `gpio_acknowledge_irq`, `gpio_add_raw_irq_handler`, `gpio_set_irq_enabled`; `GPIO_IN`, `GPIO_OUT`, `GPIO_IRQ_EDGE_RISE`, `GPIO_IRQ_EDGE_FALL`; `irq_set_enabled`, `IO_IRQ_BANK0`; `time_us_32`, `busy_wait_us_32`, `save_and_disable_interrupts`, `restore_interrupts` | Replace with timed `ultrasonic_*` API; no GPIO echo interrupts or critical sections |
| `servo.c`: `hardware/clocks.h`, `hardware/gpio.h`, `hardware/pwm.h` | `clock_get_hz`, `clk_sys`, `gpio_set_function`, `GPIO_FUNC_PWM`; `pwm_config`, `pwm_get_default_config`, `pwm_config_set_clkdiv`, `pwm_config_set_wrap`, `pwm_init`, `pwm_gpio_to_slice_num`, `pwm_gpio_to_channel`, `pwm_set_enabled`, `pwm_set_chan_level` | Fake `servo_*` clamps pulse and sets virtual angle; no PWM registers |
| `line_sensors.c`: `hardware/adc.h` | `adc_init`, `adc_gpio_init`, `adc_select_input`, `adc_read` | Fake `line_sensors_*` samples floor; no ADC registers |

Do not supply a broad collection of silent SDK no-ops. The retained button
setup and stdio init are the only initial setup-only stubs. Unknown symbols
must fail the host build. This detects new firmware dependencies.

### Wi-Fi surface

The real `wifi_log.c` has a no-config branch and an enabled branch. The host
always replaces the file, so it needs neither credentials nor network SDK
headers. The enabled branch has this additional surface:

| Headers | Symbols used by the original driver |
| --- | --- |
| `pico/cyw43_arch.h` | `cyw43_arch_init`, `cyw43_arch_enable_sta_mode`, `cyw43_arch_wifi_connect_async`, `cyw43_arch_lwip_begin`, `cyw43_arch_lwip_end`, `cyw43_arch_gpio_put`, `cyw43_tcpip_link_status`, `cyw43_state`; `CYW43_AUTH_WPA2_AES_PSK`, `CYW43_ITF_STA`, `CYW43_WL_GPIO_LED_PIN`, `CYW43_LINK_DOWN`, `CYW43_LINK_JOIN`, `CYW43_LINK_NOIP`, `CYW43_LINK_UP`, `CYW43_LINK_FAIL`, `CYW43_LINK_NONET`, `CYW43_LINK_BADAUTH` |
| `pico/rand.h` | `get_rand_32` |
| `pico/stdio/driver.h` | `stdio_driver_t` (`out_chars`, conditional `crlf_enabled`), `stdio_set_driver_enabled`, `PICO_STDIO_ENABLE_CRLF_SUPPORT` |
| `pico/stdlib.h` | `get_absolute_time`, `to_ms_since_boot` |
| `lwip/etharp.h`, `lwip/ip4_addr.h`, `lwip/netif.h`, `lwip/pbuf.h`, `lwip/udp.h` | `struct eth_addr`, `struct netif`, `struct pbuf` (`tot_len`, `payload`), `struct udp_pcb`, `ip_addr_t`, `ip4_addr_t`, `u16_t`; `etharp_find_addr`, `etharp_query`, `ip_addr_cmp`, `ip_addr_copy`, `ip_addr_set_ip4_u32`, `ip4_addr_get_u32`, `ip4addr_ntoa_r`, `ipaddr_ntoa_r`, `ip_2_ip4`, `netif_default`, `netif_ip4_addr`, `netif_ip4_netmask`, `pbuf_alloc`, `pbuf_copy_partial`, `pbuf_free`, `udp_new`, `udp_bind`, `udp_recv`, `udp_sendto`; `IP_ANY_TYPE`, `IPADDR_STRLEN_MAX`, `PBUF_TRANSPORT`, `PBUF_RAM`, `ERR_OK` |

These network symbols get no host definitions because their source is not
compiled. Fake `wifi_log_init` creates a local trace sink;
`wifi_log_poll` does no network work; `wifi_log_up` and
`wifi_log_receiver_known` return true for a virtual receiver.
`wifi_log_send` records packet type and bytes and returns success.
`wifi_log_dropped` returns zero unless a test injects a sink failure.
`wifi_log_mirror_text` records the requested setting; it does not suppress
the separate console capture. This tests firmware packet output and replay,
not Wi-Fi delivery, ARP, retries, or the real text buffer.

## Entry, exit, and clock

### Entry and exit choice

Build `detour_test.c` alone with `-Dmain=firmware_main`. A host `main` loads
the scenario and calls `firmware_main`. Do not apply this define to the host
entry point. Run one scenario per process so firmware static state starts
clean, including calibration and button state.

Options for the infinite firmware loop:

| Option | Assessment |
| --- | --- |
| Edit firmware to return at a deadline | Reject: breaks the read-only boundary |
| `longjmp` to a host wrapper | Possible in C, but skips active stack cleanup and complicates later targets |
| Finalise trace and call `exit` from the clock service | Choose: simple process boundary, no return into firmware |

At scenario end, advance only to the deadline, write a final snapshot,
flush and close trace and HTML output, then exit. Record `scenario_end`,
not firmware success. A completed firmware run can stay in its idle loop
until that deadline. A host CPU watchdog aborts stuck code as an error,
not as a simulated result. Signals and file errors must leave an explicitly
incomplete trace rather than a false success.

### Clock choice

Keep time as unsigned 64-bit integer microseconds in the simulator.
Expose SDK deadlines through a compatible `absolute_time_t` wrapper and
32-bit functions through truncation. No wall clock, threads, or real sleep
may affect physics or button delivery.

- `sleep_ms` and `sleep_us` advance by the requested duration.
- Each actual clock read (`get_absolute_time`, `time_us_32`, and the clock
  check inside `time_reached`) advances by a fixed 100 µs quantum. This is
  a simulator setting, not a measured Pico execution time.
- `to_ms_since_boot` and `delayed_by_us` only convert supplied values.
  `make_timeout_time_ms` reads time once and adds its interval.
- The clock service advances the world in steps no larger than 100 µs,
  splitting at button, echo, contact, sample, and end-time events.
- Use a fixed event order at equal times: integrate previous commands,
  apply scenario inputs, complete device events, record snapshot, return
  to firmware. Record this order and the quantum in the trace header.
- Driver functions use an internal non-advancing clock peek. A trace write
  must not advance time or cause recursive clock reads.

Advancing only on sleep would hang the busy run loop and spin calibration.
Advancing on reads avoids that. Its cost is that source changes which add
clock reads can alter simulated timing. Repeatability means the same source,
scenario, compiler/build settings, and simulator version, not every host
platform. Check sensitivity with 50 µs and 100 µs quanta before trusting a
contact result. The operator approved 100 µs with a 50 µs comparison.
These are simulator settings, not hardware measurements.

## World and physics

Use millimetres, seconds, and radians internally. The rear axle centre is
the robot origin; local x points forward, local y points left; positive
heading turns left. Record units explicitly. The viewer may show degrees.

### Geometry

Use the main [robot geometry record](CALIBRATION_RESULTS.md#robot-geometry):

| Model value | Source and qualification |
| --- | --- |
| Track 111 mm; wheel diameter 65 mm | Operator dimensions, 2026-10-30 |
| Servo axis x = 65 mm | Operator rear-axle offset |
| Front and IR board tips x = 123 mm | Calculated 65 + 58; boards at the front, operator follow-up; sensing points are provisionally 4 mm behind the tips |
| Width 160 mm; half-width 80 mm each side | Includes the left barcode reader mount; symmetric outline is conservative, body alone is narrower |
| Length 190 mm | Operator overall length; with x front 123, rectangular model rear x = −67 mm is a calculation, not a separate measurement |
| Front-corner radius 146.7 mm | Calculated sqrt(123² + 80²); firmware sweep is 150 mm |

The operator approved the conservative 160 × 190 mm rectangle for the first
release. Draw the same shape used for contact. Keep the outline configurable;
do not claim that the left mount
fills the full rectangle. The 58 mm offset is an echo mean, not a ruler
measurement. [Ultrasonic run 7](ULTRASONIC_DRIVER.md#run-7-sensor-to-robot-front-offset)
says readings at this short range were about 11 mm high. A 47 mm true
offset gives front x = 112 mm and corner radius about 138 mm. Provide a
sensitivity scenario for that alternative. Servo axis and sensor face are
not proved to coincide: use zero face offset from the axis only as an
explicit provisional model setting, pending a measurement.

IR lateral spacing, line width, and sensor footprint size are not stated
as measured dimensions in [LINE_POSITION.md](LINE_POSITION.md) or
[LINE_STEERING.md](LINE_STEERING.md). The operator approved marked guesses
because the robot is not available. Require explicit scenario values, not
silent defaults. Require physical L/C/R order, with the centre at y = 0.
The provisional sensing points are x = 119 mm, not the board-tip x = 123 mm.
This does not change the real firmware's `line_ahead_mm` 123.

### Published track specification

The operator identified the mat as Cytron line tiles. The
[Micro Robotics listing for CK-RE-TRACK-COMO-16](https://www.robotics.org.za/CK-RE-TRACK-COMO-16)
gives these published specifications, not assumed values:

| Value | Published spec |
| --- | --- |
| Material | EVA foam |
| Tile size | 315 × 315 mm |
| Thickness | 10 mm |
| Line | Black on white, 18 mm wide |
| Tile types | Straight, curve, and cross |

Build tracks on a 315 mm grid with tile position, type, and rotation.
Support blank tiles as a simulator convenience, not a claimed supplied
tile type. The first detour scenarios use a row of straight tiles; closed
loops come later. Retain arbitrary polylines and arcs for other tests.
Treat the surface as flat: record the 10 mm thickness without adding seam
or ramp physics. The provisional curve shape is a quarter circle centred
on a tile corner, radius 157.5 mm (half a tile); mark that shape as assumed
until checked against an actual tile. Do not label the published 18 mm
line width or 315 mm tile size as guesses.

### First scenario guesses

These values are approved starting assumptions, not new measurements.
Store their status and reasons in the scenario and trace. Replace them
with measurements later.

| Value | First setting | Reason and source |
| --- | --- | --- |
| IR module board size and orientation | About 32 × 14 mm, long axis forward | Operator photo description: common 4-pin MH TCRT5000 board, A0/D0/GND/VCC |
| IR centre-to-centre spacing | 22 mm; local y = +22, 0, −22 mm for L/C/R | Guess: 18 mm line + 3 mm footprint + 1 mm clearance; the recorded gap test conflicts with touching 14 mm boards |
| IR sensing-point x | 119 mm = 123 − 4 mm | Guess: TCRT5000 is about 4 mm behind the front board tip; the board tip is at the robot front |
| IR footprint width | 3 mm | Guess; not measured |
| Bottle diameter | 66 mm | Derived from the 33 mm radius used for the bottle centre in [bypass track tests](OBSTACLE_BYPASS.md#track-tests); still an assumed scenario size |
| Box size | 120 × 120 mm | Pure guess; no size is recorded |
| Curve tile shape | Quarter circle, radius 157.5 mm, centre at a tile corner | Guess from half the 315 mm tile size; check against a tile |
| Servo axis to sensor face | 0 mm | Provisional guess; no measurement available |
| Rear outline x | −67 mm = 123 − 190 mm | Calculated from the approved rectangle, not a rear-overhang measurement |

An 18 mm line conflicts with the operator's description of touching boards
and 14 mm spacing. At that spacing a gap-centred line covers both sensing
footprints, whereas the recorded C–L gap test gave under 1 % rise. That
record also says the sensors were already at the highest and closest mount
position, which suggests a mount with set positions and gaps. With an
18 mm line and 3 mm footprint, spacing must be about 21 mm or more to
leave the sensing footprints clear. The provisional 22 mm spacing gives
1 mm total clearance. Measure the actual centre-to-centre spacing; the
board width alone does not establish it.

Add a consistency scenario for every spacing/line-width/footprint set:
centre the line in the C–L gap and require under 1 % rise above the floor
relative to each sensor's floor-to-line range, with noise disabled. Compare
with [the recorded gap test](LINE_POSITION.md#the-gaps-between-the-sensors).
With the published 18 mm line and guessed 3 mm footprint, the 14 mm spacing
case must fail this check and the 22 mm case must pass. A failed check must
be reported as an inconsistent model, not hidden by changing ADC endpoints. This check constrains the guesses; it does not
measure the actual board layout.

### Motors, counts, and coast

Use separate left and right speed curves. The initial reference is the
[5 V VIN unloaded speed table](CALIBRATION_RESULTS.md#unloaded-motor-speed-at-5-v-vin):

| Duty % | Left RPM (motor 1) | Right RPM (motor 2) |
| ---: | ---: | ---: |
| 10 | 26.253 | 26.556 |
| 20 | 53.881 | 54.352 |
| 30 | 81.486 | 81.948 |
| 40 | 109.006 | 109.328 |
| 50 | 136.648 | 136.527 |
| 60 | 163.417 | 163.136 |
| 70 | 190.842 | 188.965 |
| 80 | 217.029 | 214.298 |
| 90 | 242.664 | 239.560 |
| 100 | 260.829 | 257.860 |

When implementing, copy all entries directly from the calibration record
and check them mechanically. The record's historical RPM used 2540 counts
for motor 1 and 2539 for motor 2. Apply its correction factors 2540/2535 and
2539/2535 once. Keep that correction in model metadata.

The [minimum unloaded duty record](CALIBRATION_RESULTS.md#minimum-unloaded-motor-duty)
reports VIN movement at 6 % left and 7 % right; it recommends 8 % common
minimum and a 10 % start kick. The [loaded steering record](LINE_STEERING.md#how-it-works)
uses about 12 % as the loaded start threshold. Do not implement a start
kick unless the firmware commands one. The operator approved the unloaded
speed table plus a provisional loaded profile, and symmetric reverse.
Offer both as named profiles. In the loaded profile, shift the curve's zero
to 12 % and preserve the corrected high-duty endpoint; mark this transform
as an assumption, not a loaded speed measurement. Interpolate between
points; clamp duties to ±100. Reverse uses the same magnitude initially,
which is another unmeasured assumption.

Wheel speed = corrected RPM × 204.204 / 60 mm/s. Use 2535 counts per wheel
turn, from [shared encoder calibration](CALIBRATION_RESULTS.md#current-shared-calibration)
and the firmware wheel circumference in
[`obstacle_detour.h`](../src/obstacle_detour.h). Accumulate fractional counts
and publish signed integer counts, forward positive on both sides.
Integrate differential drive: centre speed = (vL + vR)/2; turn rate =
(vR − vL)/111. Use bounded substeps or the exact circular update.

Use a configurable first-order speed response for acceleration and coast;
a zero command must not force speed instantly to zero. The
[coast measurements](MOTOR_ROTATION_DRIFT.md#coast-and-jog-measurements) give
27–28 counts left and 23 right at 10 % duty, wheels lifted, USB supply.
They are not loaded coast values. Set response times from explicit scenario
parameters; fit an unloaded reference profile to those distances, and sweep
loaded coast parameters. Do not treat 250 ms firmware pauses as measured
motor time constants. Continue counts during coast, as the real firmware
expects.

#### Implemented motion equations and coast fit

The production model is [`sim/core/world.c`](../sim/core/world.c).
Unloaded curves have explicit zero-speed anchors at 6 % left and 7 % right,
then linear interpolation through the corrected table. For the loaded
profile, map duty magnitude d above 12 % to the unloaded curve at
`u = u0 + (d − 12) × (100 − u0) / (100 − 12)`, where u0 is 6 or 7.
At or below 12 % the target speed is zero. Reverse negates the target.
This approved curve transform is a provisional model, not loaded test data.

For a constant command over dt seconds, use exact first-order wheel updates:

```text
v1 = target + (v0 − target) × exp(−dt / tau)
ds = target × dt + (v0 − target) × tau × (1 − exp(−dt / tau))
counts += ds × 2535 / 204.204
```

The loaded scenarios explicitly assume tau = 50 ms for each wheel, for
both acceleration and coast. Fractional counts accumulate; firmware reads
signed counts truncated toward zero. The body uses the mean wheel travel
and `(dsR − dsL) / 111` turn, integrated as a circular arc per substep of
at most 100 µs. General scenario wheel diameter scales the 204.204 mm
circumference in proportion to the nominal 65 mm diameter.

For the unloaded reference, a settled 10 % command gives count rates
`26.253 × 2540 / 60 = 1111.377` left and
`26.556 × 2539 / 60 = 1123.7614` right counts/s. A first-order stop travels
`rate × tau` counts. Fit tau to 27.5 counts left (the 27–28 midpoint) and
23 right: 24.744078742 ms and 20.466978133 ms. The
[`unloaded_reference` scenario](../sim/scenarios/unloaded_reference.json)
marks this fit as assumed: the coast source used USB power, while the speed
table used VIN. These are synthetic fit parameters, not measured time
constants; using the same tau for acceleration is another assumption.

The independent model probe checks a loaded 20 % command for 1 s from rest:
wheel travel 125.732983276/135.094957331 mm and fractional counts
1560.856362291/1677.076437455. Different wheel curves give a slight arc,
not perfectly straight motion. The on-the-spot command (+20/−20) turns
−134.633695278°; the computed scale is 24.049943761 difference counts/degree,
which agrees with firmware's rounded 24.05. Reverse gives the negative
forward counts. A loaded stop after 1 s adds 82.150334510/88.267180545 counts
over the next second; unloaded reference stops add 27.5/23 counts.

### Servo and ultrasonic

- Keep `servo_set_pulse_us` return/clamp behaviour: 1100–1900 µs, centre
  1500 µs, positive angle left. Use angle = (pulse − 1500) × 90/1000 degrees
  ([servo record](CALIBRATION_RESULTS.md#servo), `servo.h`).
- Initially move to the commanded angle without a separate mechanical
  delay. Real scan/detour settle waits remain in the firmware. A configurable
  slew model can follow later; no measured slew rate is available here.
- Use a 30° full beam, ±15° about the servo direction
  ([scan beam](OBSTACLE_SCAN.md#scan-points)). Find the nearest surface in
  the sector, not only on the centre ray. Draw the centre ray, sector, and
  actual echo segment separately so off-axis hits are visible.
- The operator approved ideal nearest-surface echoes in the 30° sector
  for the first release, with opaque rectangle and circle surfaces.
  Reflection and dropout come later. This is not a claim about real
  ultrasound: tilted flat faces and thin objects can give no echo; see
  [driver representation rules](ULTRASONIC_DRIVER.md#packed-result).
- Preserve the `ultrasonic.h` 16-bit result: bits 0–8 distance, 9–10 status,
  11–13 fault streak, 14–15 zero. VALID below 60 mm has distance code 0;
  60–511 mm is rounded distance. Beyond 511 or no detectable return gives
  NO_RETURN, not a sensor fault. NO_RISE and STUCK_HIGH increment the
  saturating streak to 5; VALID and NO_RETURN reset it. Publish the initial
  NONE word and debug values as specified by the header.
- Preserve trigger rejection while busy or within the 60 ms minimum gap.
  STUCK_HIGH injection publishes immediately with trigger success and no
  busy state. Other results become available only through service after
  their scheduled completion; do not return an echo synchronously.
- Timing source: [driver constants](ULTRASONIC_DRIVER.md#timing-constants)
  and `ultrasonic.c`: 10 µs trigger, measured rise latency 212–217 µs,
  2 ms rise timeout, 3 ms echo timeout, sound speed 343 m/s. Choose 215 µs
  fixed latency within that measured range. Pulse duration is 2d/343 with
  units converted to µs. Schedule at most one result per trigger, including
  NO_RETURN at the echo timeout and injected NO_RISE at the rise timeout.
  Sample world geometry at trigger time and document that simplification.

Milestone 3 implements these devices in
[`sim/devices/ultrasonic.c`](../sim/devices/ultrasonic.c) and the world model.
The face is on the x = 65 mm axis with the explicit assumed zero face offset.
The conservative robot contact rectangle does not occlude its own sensor.
For a box, test each edge's endpoints, perpendicular nearest point, and
intersections with both sector boundaries. For a circle, test the nearest
radial point and both boundary-ray intersections. Keep the nearest admitted
surface across all obstacles. This includes off-axis echoes, not just the
centre ray. The aperture is a closed ±15° sector.

Round the ideal pulse to integer microseconds, then use the real driver's
rounded pulse-to-mm conversion and packed-field limit. The ideal geometry
also rejects surfaces beyond 511 mm before rounding the result field. Thus
a surface at 511.1 mm is NO_RETURN even if diagnostics round to 511 mm.
Likewise, geometry below 60 mm gets code 0 even if diagnostics round to 60.
A completed pulse
can therefore keep diagnostic timing even if its rounded distance exceeds
511 mm. Longer/no echoes complete at 215 + 3000 µs with zero pulse diagnostic.
The clock records echo completion at its scheduled time; only a service
call publishes it. The optional echo callback runs on a completed falling
edge, not on a timeout. Servo changes during a pending echo do not change
that trigger's geometry. Trigger gaps use unsigned 32-bit subtraction,
including wrap; scheduling uses the 64-bit world clock.

Explicit scenario events can set a persistent sensor fault mode:
`{"at_us": 0, "ultrasonic_fault": "NO_RISE"}`. Modes are `NORMAL`, `NO_RISE`,
`STUCK_HIGH`, and `NO_RETURN`; they apply to the next accepted trigger, not
to an already pending measurement. `NORMAL` removes injection. NO_RISE
finishes after 2000 µs; STUCK_HIGH publishes immediately without busy.
Only these injected faults increment the streak. VALID/NO_RETURN reset it;
the streak saturates at 5. The schema rejects servo slew and unimplemented
beam/timing changes instead of ignoring them.

### IR readings

Generate raw 12-bit ADC samples, not `line_seen`, position, or events.
The unchanged `line_spin.c` must calibrate through the scenario button;
`line_position.c` must do all learning and threshold decisions.

[Line position run 1](LINE_POSITION.md#run-1-2026-10-27-spin-calibration-usb-power)
uses old labels (L = GP26, physical right; R = GP28, physical left).
Keep these recorded ADC endpoints: they were measured on this mat. The
published line width does not change them. The physical order for fake
`line_sensors_read` is therefore:

| Physical sensor | Floor ADC | Line ADC |
| --- | ---: | ---: |
| Left, GP28 | 202 | 1645 |
| Centre, GP27 | 185 | 2615 |
| Right, GP26 | 191 | 3424 |

Mix floor and line readings by footprint overlap with the scenario's line
band; clamp to 0–4095 and round consistently. A point model alone produces
unrealistically sharp edges, so footprint width must be a visible scenario
parameter. The [gap observations](LINE_POSITION.md#the-gaps-between-the-sensors)
show less than 1 % rise between sensors and about ±2 ADC noise, with floor
variation about 100 counts. Do not use broad overlap that removes the real
gaps. Begin with noise disabled. Later add seeded noise, a small optional
gap response, and floor patches. [Steering observations](LINE_STEERING.md#straight-across-a-junction)
confirm narrow overlap and mostly 0/±100 position; ramp behaviour is outside
the first flat-floor model.

Milestone 2 implements a uniform one-dimensional sensing aperture across
the nearest line band. Its overlap fraction mixes the recorded endpoints;
results are rounded and clamped to 12 bits. Tiles support straight, curve,
cross, blank, and rotation; arbitrary polyline/arc strokes also work. At
stroke overlaps the strongest coverage wins. This is an explicit optical
approximation, not an area-integrated sensor measurement. The native gap
check temporarily places a straight line in the C–L gap and runs the actual
stroke-to-footprint-to-ADC path. At 22 mm spacing it returns floor values
202/185/191 (zero rise); the 14 mm case is rejected by that same native
model, not only by the Python schema pre-check.

### Obstacles and contact

Support rotated rectangles (boxes) and circles (bottles), with scenario
sizes rather than invented D1–D18 dimensions. Detect intersection of the
robot polygon and obstacle boundaries, including edge contact. Record the
first contact time, surface, and robot pose independently of firmware logs.
Prevent tunnelling by checking swept motion or refining contact within a
substep. Do not allow the robot to pass through an obstacle.

The operator approved both-wheel contact freeze: stop both wheels and their
encoder progress while commands push into an obstacle. Permit a separating
command to move again. This must exercise the real 500 ms wheel stall check,
not stop the simulator on contact. Single-wheel stalls are explicit fault
injections only; integrate the remaining motion with collision constraints.
This is a fault-injection model, not a prediction of contact
forces. Include a record-only mode for comparison, clearly marked as
non-physical because it allows penetration.

Milestone 2 uses only a conservative both-wheel safety latch, not the full
response above. Rectangle/rectangle separation uses polygon axes and edge
distances; bottle contact uses polygon/circle distance. Bound every outline
point's speed by `(|vL|max + |vR|max) × (1/2 + corner_radius/track)` over the
substep. Advance by no more than half the current clearance divided by that
bound, with a normal 100 µs cap. Within a two-microsecond bound, record first
contact at the current safe pose and set both speeds to zero. Thus a contact
record can be a few micrometres early; its saved clearance makes this visible.
The test includes a rotated 0.001 mm box and a circle, so thin obstacles
cannot be skipped between endpoint tests.

The latch prevents further pose or encoder changes until a new scenario,
even for a reverse command. This is intentionally simpler than separating
motion or contact forces. Other contact modes are rejected until the full contact-response work.
Snapshots carry the latch; the contact event carries obstacle ID/index,
time, pose, and residual clearance. In the milestone 2 reference (only timed NO_RETURN, now reproduced with
an explicit NO_RETURN injection), safe contact is at 8.542512 s, rear axle (616.721223724, 8.691295026) mm, heading
+0.230581211°, clearance 0.001337385 mm. The actual firmware then reports a
left-wheel stall at 9.042200 s; it does not drive through the box.

## Scenario and trace formats

Choose versioned JSON for scenarios. It is available without another parser
package in the proposed Python output tool and is readable in the browser.
The host implementation can choose a small JSON parser under `sim/`.
Reject unknown required units and values that are missing entirely.
Accept an explicit marked guess in the form
`{"value": X, "assumed": true, "note": "..."}`. Its note must give the
reason or source. Write every assumed value, its field path, and its note
into the trace header. The viewer must show an **ASSUMED** list. Measurements
can replace guesses later without changing the world model. A missing
value still fails: no model parameter may get a silent default. Even a
choice such as disabled noise must be explicit in the scenario.

| Scenario field | Content |
| --- | --- |
| `schema_version`, `name`, `seed` | Format, human label, fixed noise seed |
| `firmware_target`, `end_us`, `clock_quantum_us` | Target, hard simulated deadline, clock policy |
| `track` | 315 mm tile grid with straight/curve/cross/blank cells and rotation; 18 mm published line width; assumed curve shape; arbitrary polylines/arcs also available |
| `obstacles` | ID, rectangle centre/size/angle or circle centre/radius, echo material, contact mode |
| `robot` | Initial rear-axle pose, outline, wheel profile, sensor x/y coordinates, IR footprint and ADC endpoints, servo/echo settings |
| `events` | Ordered absolute-time GP20/GP21 press and release events, optional USB connection and sensor faults |
| `noise` | Explicit enabled/disabled choice; initially disabled; seed, ADC variation and later echo dropout parameters when enabled |

Times are measured from process start, including the firmware's 2 s boot
wait, spin calibration, and 1 s start delay. Press and release durations
must overlap the actual button polling. Warn about ignored presses during
calibration. Do not bypass calibration by setting private firmware state.

Choose a JSON trace rather than CSV: nested geometry, events, text, and
packet bytes do not fit one flat table well. Keep a versioned header with
source commit, source hashes, compiler flags/version, simulator version,
scenario contents, all assumed values and notes, seed, units, and clock quantum.

Record regular 10 ms world snapshots (proposed display interval), plus
exact-time motor commands, servo commands, buttons, ultrasonic triggers and
results, contacts, packets, and text fragments. Snapshots contain pose,
wheel speeds/counts/duties, IR points/raw values, servo angle, and echo hit.
Store packet bytes as well as decoded fields so decoding can be checked.
Preserve text fragments and their timestamps; join lines for display without
losing the original fragments.

### Firmware state visibility without source edits

The operator approved per-source call hooks as the main observation method.
No firmware source edit is needed. A translation unit is one C source file
compiled with its included headers. For `detour_test.c` only, use defines
such as `-Ddt_update=sim_hook_dt_update`. Compile the real modules without
these hook defines, so their functions keep their real names. Compile each
hook implementation without the rename; it includes the real header and
calls the real function, then records its result at the current exact
simulator time. It returns the original result unchanged.

| Calls hooked from `detour_test.c` | Data to record after the real call |
| --- | --- |
| `dt_init`, `dt_update`, `dt_reading` | Inputs, step, failure, heading, pose, and returned command where present; readings can change step too |
| `obstacle_update` | Result word, especially STOP and SENSOR_FAULT |
| `obstacle_scan_start`, `obstacle_scan_service` | Start success and scan event, especially DONE and FAULT |
| `lp_update` | Line event, line-seen state, position, and caller inputs |
| `steer_update`, `steer_straight` | Input and returned wheel duties; distinguishes normal follow and junction hold |
| `steer_reset`, `obstacle_init` | Follow initialisation/reset, including return from search or detour |

The target description lists the hook names, signatures, per-source defines,
and state-label rules. A different target supplies its own list without
changing the world or clock core. Calls inside `line_spin.c` or other real
modules are not renamed by the `detour_test.c` hook options.

Infer the run phase from call order and results, not from a second control
algorithm: follow initialisation or steering means FOLLOW; LINE_LOST from
FOLLOW means SEARCH; LINE_FOUND in SEARCH precedes follow reset; watch STOP
precedes STOP; a successful scan start precedes SCAN; scan DONE followed by
`dt_init` means DETOUR. `dt_update` DONE precedes follow reset. Record exact
call boundaries and inferred phase separately. A callback runs just before
the caller performs its local phase assignment; this is not direct access
to that variable. Do not claim an instruction-level phase timestamp. While
the caller executes between hooks, use the last confirmed phase and retain
pending transition evidence. The initial idle/calibration and ended-run
states must not be labelled FOLLOW merely because an old run was FOLLOW.
Use the existing console run markers and device stop events to check those
boundaries; mark any unresolved phase interval rather than inventing one.

Hook limits and checks:

- Preserve argument types, evaluation count, side effects, return values,
  and function semantics. Observation must not call advancing clock APIs.
- Header declarations in the caller are renamed too; give the hook exactly
  that signature. The real declarations stay unchanged in other units.
  Verify no accidental recursion, duplicate definition, or rename leakage.
- Static inline helpers such as `dt_step`, `dt_move`, `dt_fail`, and
  `dt_servo_deg` cannot be intercepted as external calls by this method.
  A macro rename can rename their local definitions too. Do not wrap them;
  hooks read the public result/state after the external call instead.
- Check header inline bodies and macro expansion for each proposed rename.
  A rename must not change the meaning of a call or internal header logic.
- The hooks expose module state at exact call completion times. They do
  not expose every private local assignment or a transition inside an
  unhooked module. Extend the target hook list only after a semantic check.

Capture `printf` through a host build rename to `sim_printf` on firmware
translation units, with host format checking and printf built-ins disabled
so calls are not changed to uncaptured `puts`. Keep simulator text output
separate. Test fragment handling and the `long` format widths.
Use the existing 50 ms `steer_log` packets and `[scan]`, `[obst]`, and
`[detour]` labels as cross-checks, not the main state source. Packets are
batched and replayed; retain their original sample time/index, not delivery
time. Check that hooked detour step order agrees with printed step labels,
allowing a print to omit a short-lived state between control updates.

## Viewer choice

The offline canvas viewer is implemented in
[`sim/viewer/render.py`](../sim/viewer/render.py), its HTML template, CSS,
and JavaScript. `run.html` embeds all display data and code, without external
libraries or assets. Open it through `file:`; no server, network, installation,
or neighbouring JSON fetch is needed. Regenerate from a saved trace without
building or running firmware:

```sh
python3 sim/viewer/render.py sim/output/box/trace.json
```

The floor is top-down in mm, +y up. It shows tile lines at their scenario
width (18 mm in the reference), arbitrary strokes, boxes/circles, the true
rectangle, normalized/raw IR points, the current servo ray/sector, sampled
echo segment/point/distance, past trail, and timed contact markers. Fit all,
focus robot, drag pan, and pointer-centred wheel zoom change only the camera.
Controls include play/pause, 0.25×/1×/4×/16×, a microsecond scrubber,
previous/next detour transition, and space to play/pause.

The orange **firmware estimate** uses only public module x/y/heading values,
held at their saved hook times. They are start-of-step values, not a live
integrated estimate. At each `dt_init`, take the saved world rear-axle pose
as origin (x0, y0, h0). For side S = +1 left / −1 right:

```text
world_x = x0 + module_x cos(h0) − S module_y sin(h0)
world_y = y0 + module_x sin(h0) + S module_y cos(h0)
world_heading = h0 + S module_heading
```

Do not add unrecorded travel or turn angles. The blue **true pose** uses
world snapshots; only its drawing position/heading is interpolated. Counts,
ADC, commands, phase, steps, console, and results change at saved times.
A missing world interval longer than 10 ms is shown as a gap, with no true
pose or trail bridge. No extrapolation is made beyond saved snapshots.
The estimate is hidden outside inferred DETOUR or when its frame is missing.
The side panel marks phases inferred and preserves the evidence/pending
boundary in a tooltip; it shows sample times, last hooked state/failure,
last published ultrasonic word, duties/counts, text-only console, assumptions,
scenario, and source. Incomplete data has a clear banner.

Display data retains every world sample, dt observation, step transition,
duty/servo command (including repeated commands), text fragment, trigger,
result, and contact. Other hook calls, packets, and completion events remain
in the full `trace.json`. Compact arrays and omitted unused fields keep the
reference HTML under 3 MB. Sorted JSON makes regeneration byte-identical.
Data escapes `<`, `>`, `&`, and non-ASCII code points before embedding, so
`</script>` cannot close the JSON script element. Dynamic text uses
`textContent` or canvas text, never HTML. The content policy blocks network
connections and external assets. Browser tests exercise file URLs with the
network disabled, step/console fidelity, pose-only interpolation, controls,
gaps, and literal `</script><b>` text. Playwright is optional for tests only;
the viewer has no runtime dependency.

Milestone 4 reference check: the 40 s `box_straight` HTML is **1,868,873
bytes (1.7823 MiB)**. It keeps 4,001 world samples, 7,590 duty/servo commands,
2,718 dt observations, and all eight detour transitions. Chromium
149.0.7827.55 opened an isolated HTML copy from a file URL, with network
disabled and no neighbouring trace. There were zero console/page errors
and zero network requests. Screenshots at 7.8 s (STOP), 8.6 s (SCAN),
12.0202 s (first STRAIGHT), 21.55 s (near 22 s), and 34.8102 s (FAILED)
are in ignored `sim/output/milestone4/`, with the browser report and build/
test evidence. The heading gap remains a displayed result only. The four
CTest groups (27 Python tests plus native probes), all ten strict C host
tests, documentation checks, unchanged firmware hashes, and repeat trace
checks pass.

## Optional local environment editor

The offline viewer remains independent of a server. The earlier no-live-mode
choice still applies to firmware execution and replay: there is no streaming
controller, browser physics, or browser steering. The optional
[`sim/viewer/serve.py`](../sim/viewer/serve.py) adds **edit, batch run, replay**
on the local machine. It uses only Python's standard library and the already
built `robot_sim`. It does not build or flash anything.

```sh
python3 sim/viewer/serve.py --build sim/build --port 8765
# Open http://127.0.0.1:8765
```

The editor HTML, CSS and JavaScript are separate from the unchanged offline
assets. An environment-preview canvas supports object dragging, snap, pan,
zoom and dimensions; the existing viewer loads each completed `run.html` in
a frame and starts playback. Editing a draft never changes a saved trace.
The session history lists box dimensions and outcomes; click two entries in
turn to compare their saved end views and return to editing either draft.

The server permits changes only to obstacles, track tiles, robot start pose,
and scenario end time. Robot geometry/IR/servo/ultrasonic/motor assumptions,
firmware target, clock, seed, noise, button/fault events, track specifications,
origin and primitives, and other fields are read-only. These values come
from the selected base file. The server validates with the existing
`unwrap`/`validate` rules, then compares every non-environment field against
that base. A modified request cannot bypass the read-only display.

Each run gets a new `sim/output/web/SESSION-COUNT-HASH/` directory with a
10-character scenario hash. It holds `scenario.json`, `trace.json`, `run.html`,
and console/error text. The response includes the full trace. A shared lock
permits one simulator process at a time; concurrent runs or saves are refused
with a busy message. Native CPU/wall watchdogs remain unchanged; the server
also has a 25 s outer process-group timeout. Missing/stale binaries or build
configuration produce a build command, without starting a run. Freshness
checks use selected C sources, compiler header dependencies, launcher template,
and CMake configuration timestamps; non-Make generators use a conservative
header fallback. Python and web assets do not require rebuilding the native
binary.

Save as scenario uses the same validation and allows names with 1–64
characters from `[a-z0-9_-]`. It publishes a complete file atomically and
refuses existing files or symlinks. The save lock also prevents a new source
file from racing the simulator's source-hash capture.

The server binds only `127.0.0.1` and checks the peer address, Host and Origin.
It has no file-browser route or client-supplied executable/output paths.
Only fixed page assets, valid base names, and this session's known run IDs
are served. Requests have a 1 MiB limit and a 10 s socket timeout. Child
commands use argument arrays, never a shell. Do not expose this development
server through a proxy or on a network interface.

`sim_environment_server` checks the validation boundary, missing/stale build
messages, busy lock, exclusive save/reload, path/Host/Origin/body limits,
and rejection of a real socket peer bound to `127.0.0.2`. The optional
[`editor_browser.cjs`](../sim/tests/editor_browser.cjs) checks the full UI,
CLI trace equivalence, an offset-box rerun, field errors with no simulation,
save/reload/refusal, quick history switching, and offline output. Evidence
and timings are saved in ignored `sim/output/web_check/`.

## Layout and commands

The standalone build, target description/hooks, minimal SDK headers,
clock/runtime, Python scenario reader/runner, replacement drivers, scenarios,
and tests now exist. Motion, geometry, contact safety, and IR world physics
also exist. Servo/echo geometry, observational decoding, and the offline
viewer now exist. Full contact response remains outside this milestone.

| Path | Content |
| --- | --- |
| `sim/CMakeLists.txt` | Standalone host build; no Pico import |
| `sim/targets/` | Source lists, compile options, hook lists/implementations, and state-label rules per firmware target |
| `sim/include/pico/`, `sim/include/hardware/` | Minimal SDK-compatible headers actually needed by retained firmware |
| `sim/core/` | Host entry/clock, world motion/contact/IR, JSON scenario reader, private native protocol, trace output |
| `sim/devices/` | Five replacement drivers and retained SDK call implementations |
| `sim/scenarios/` | Input JSON and documented assumed parameters |
| `sim/viewer/` | Offline HTML generator/assets and optional loopback environment editor/server |
| `sim/tests/` | Device contracts, geometry tests, repeatability checks |
| `sim/build/`, `sim/output/` | Ignored host build and generated runs |

Build from the repository root:

```sh
cmake -S sim -B sim/build -DSIM_FIRMWARE=detour_test
cmake --build sim/build
sim/build/robot_sim --scenario sim/scenarios/box_straight.json --output sim/output/box
```

The host build prepends `sim/include`, then reads `src/` headers. Use C11
and strict warnings for simulator code. Use per-source defines to rename
firmware entry, console calls, and target-specific external calls. A local
output generator may use Python's
standard library; the generated viewer needs only a browser. Every selected
source and replacement must appear in build metadata. Hash `src/` before
and after build/run to check the read-only rule.

## Limits and risks

- No slip, tyre deformation, mass distribution, battery sag, gearbox
  backlash, ramps, 3D beam shape, or acoustic multipath in the first model.
- Unloaded speed and coast data do not predict loaded turns accurately.
  Results are comparisons under stated assumptions, not track success rates.
- Noise is absent unless explicitly enabled and seeded. A perfect sector
  echo can hide the real box/bottle failures.
- No real Wi-Fi, USB transport, IRQ latency, Pico instruction timing, or
  PWM/ADC electrical behaviour. Replacement-driver tests do not test the
  original hardware drivers.
- Host `long`, pointer width, alignment, and printf behaviour differ from
  ARM. Keep packed formats under tests; do not serialise host structs.
- Inline functions in real headers must resolve against the portable SDK
  substitutes. A new direct SDK use, flash write, assembly statement, or
  RTOS dependency can block a target. Fail rather than remove that logic.
- Current `detour_test` has direct SDK button/time/USB calls. Those need the
  precise small compatibility layer above, not just five fake drivers.
- The existing host detour test uses different configuration values from
  v1.5. It remains useful but does not replace a full firmware scenario.

## Operator decisions and remaining measurements

The operator closed the first-release choices in the follow-up to this
note. The original question numbers are kept for reference.

| Question | Decision |
| --- | --- |
| Q1, Q4: missing geometry and scenarios | Closed for the first release: accept explicit marked guesses, including the first-settings table above; measurements replace them later |
| Q3: motors | Closed: use unloaded reference data and the provisional loaded profile with zero shifted to 12 %, and symmetric reverse; compare algorithms, not track success rates |
| Q5: outline | Closed: conservative 160 × 190 mm rectangle |
| Q6: echoes | Closed: ideal nearest surface in the 30° sector first; reflection/dropout later |
| Q7: firmware state | Closed: per-source external-call hooks; no source edit; exact hook timestamps and separate inferred phase with the limits stated above |
| Q8: clock and contact | Closed: fixed 100 µs clock-read quantum, compare against 50 µs; both-wheel contact freeze; single-wheel stalls only as explicit fault injections |

Remaining measurement questions are not blockers:

- Q1: measure IR centre-to-centre spacing. The touching-board description
  implies 14 mm, but the 18 mm published line and recorded gap test need
  about 21 mm or more with the 3 mm footprint. Use the marked 22 mm guess
  until measured. Check the curve tile shape against an actual tile too.
- Q2: confirm rear-axle/front and sensor-face offsets with a ruler when
  the robot is available.

For the first scenarios, use front x = 123 mm, sensing x = 119 mm,
and zero servo-axis/face offset as
marked assumptions until then; retain the 112 mm front-offset sensitivity
case. Also replace guessed IR layout, footprint size, object sizes,
start poses, and loaded response/coast settings with measurements when
possible. Keep the recorded ADC endpoints unless new measurements replace
them. Response times and scenario start poses
must be explicitly supplied as marked guesses until measured; this note
does not select silent numeric defaults for them.

No firmware edit is requested. The 119 mm sensing-point guess refines the
earlier statement that the sensors are at the front: the board tips are
at the front, but the sensing elements are behind them. It differs from
the firmware's current 123 mm setting and must stay visible as an
assumption, not be used to change firmware. D1–D18 logs are unavailable on
this machine; no replay match is claimed.

## Milestone 3 results

`box_straight` retains the assumed 120 × 120 mm box at (800, 0) mm. The real
firmware stops at 7.681700 s with a 224 mm reading (2401 ms into the run).
The scan completes at 9.808000 s:

```text
221 217 215 214 214 214 214 214 214 214 215 218 NR
nearest 214 mm, free step R 7 L 6 -> detour LEFT
```

Hooked transitions below use actual call completion times. Module pose and
heading are integer S-frame values relative to the detour origin, stored at
the start of each step. They are not live world position. World pose is the
rear axle in the original scenario frame; values below are rounded for display.

| Time s | Step | Module x, y mm; heading ° | World x, y mm; heading ° |
| ---: | --- | --- | --- |
| 9.808000 | TURN_AWAY | 0, 0; 0 | 460.664, 7.831; +0.443 |
| 12.013600 | STRAIGHT | 0, 0; +51 | 462.202, 8.575; +51.214 |
| 14.870200 | TURN_TOWARD | 220, 271; +51 | 678.978, 283.487; +51.763 |
| 18.450200 | STRAIGHT | 220, 271; +21 | 682.410, 288.264; +21.980 |
| 21.300200 | TURN_TOWARD | 546, 396; +21 | 987.843, 459.860; +29.316 |
| 26.050200 | STRAIGHT | 546, 396; −39 | 991.851, 463.236; −30.185 |
| 28.900200 | TURN_TOWARD | 817, 176; −39 | 1314.685, 327.176; −22.843 |
| 34.810200 | FAILED / TIME | 817, 176; −39 | 1321.246, 328.258; −164.291 |

The first STRAIGHT is set by `dt_reading`; its print follows at 12.020200 s.
The other step prints also match the latest module state. Cross-check result:
8 printed steps, 8 hook transitions, no omitted states, no mismatches. The
run summary at 34.810300 s reports 29530 ms, detour failed (time limit),
1 obstacle stop, 0 detours done, 0 searches, counts L 20952 / R 16993.
No contact occurs, so there is no contacting corner or edge to report.

The inferred phase boundaries are FOLLOW 5.280400 s, STOP 7.681700 s,
SCAN 7.981200 s, DETOUR 9.808000 s, and not-running 34.810300 s. The
100 µs interval from the console run marker to follow reset is unresolved,
not assigned FOLLOW by assumption. Pending caller assignments are separate
from module hook completion; these are not instruction-level phase samples.

Comparison with [handoff problems A–G](OBSTACLE_DETOUR_HANDOFF.md#open-problems):

| Problem | This run |
| --- | --- |
| A: line accepted while heading away | Not observed. No ON_LINE; the line is seen only initially in TURN_AWAY before arming. |
| B: ALIGN sweeps past line | Not exercised: no ALIGN. |
| C: stale follower memory after DONE | Not exercised: no DONE or return to following. |
| D: line ignored during TURN_TOWARD | Not observed as a crossing: the model line never reappears after the initial turn. The source still ignores line in turns. |
| E: contact/stall at a corner | Not observed: no contact and no stall. The safety latch is not used. |
| F: small TURN_AWAY | Not observed: TURN_AWAY ends at module heading +51°. |
| G: Wi-Fi text loss | Cannot assess with the local sink. The captured text has no missing step labels; real Wi-Fi buffering is not modelled. |

The observed failure is the detour's own time limit during its third
TURN_TOWARD. World and integer module poses diverge during the provisional
coast model; both are kept, rather than replacing either with the other.
These are comparisons under assumptions, not replay of D1–D18 or evidence
that any handoff problem is fixed. No firmware value was tuned.

The separate real-scan probe puts the sensor at (65, 0) facing a box with
front x = 365 mm, y = ±60 mm (300 mm nearest distance). Its 13-point profile
is `NR 305 301 300 300 300 300 300 300 300 301 305 NR`. For a −25.02° pulse
angle, the nearest admitted front point is on the −10.02° sector boundary:
`300 / cos(10.02°)` mm. At −19.98°, use `300 / cos(4.98°)`; at 0°, use
300 mm. Pulse rounding gives the profile values above. The ±29.97° sectors
miss the box: the largest front-corner bearing is only atan(60/300) = 11.31°.
This checks the real scan's 13 points, two readings per point, and settle waits.

Unit checks also cover 59/60/511/512 mm on boxes and circles, a 12° off-axis
box not hit by the centre ray, no hit at 16°, no surface, busy/gap rejection,
fault streak saturation/reset, injected immediate STUCK_HIGH, 32-bit wrap,
publication only on service, callback timing, and a servo change during a
pending measurement. The hook-vs-unhooked comparison checks every native
non-hook event byte-for-byte as decoded JSON, not only the final result.

## Build plan and checks

Motion/IR and servo/echo/hook/full-loop work now have passing checks. The no-box reference
has 3000 snapshots between printed run start and stop; rear-axle absolute
lateral error from the y = 0 line has maximum 10.066755148 mm and mean
9.513485314 mm. The centre IR point has maximum 13.626529921 mm and mean
9.690098619 mm. These are simulator comparisons under the stated guesses,
not measured track accuracy. The run starts at 5.280300 s and stops by the
30 s limit at 35.280200 s; no searches or contacts occur.

| Step | Work | Check before the next step |
| --- | --- | --- |
| 1 | Fix scenario schema and measured/assumed inputs | Accept explicit value/assumed/note entries; reject missing values, invalid units, and unsorted events; trace and viewer list every assumption |
| 2 | Standalone source selection and SDK declarations | Host compiles unchanged retained sources; no firmware/root CMake edits; symbol inventory has no unexplained stubs |
| 3 | Clock, entry, exit, console capture, button events | Idle loop and spin busy loops reach deadline; sleeps advance correctly; fragments and exit traces are complete |
| 4 | Motor, encoder, geometry, coast | Straight, spin, reverse, and coast match model equations; signs and fractional counts pass; no obstacle tunnelling |
| 5 | Tile tracks, IR floor, and real spin calibration | Straight tiles join on the 315 mm grid; with 18 mm line and 3 mm footprint, the gap test rejects 14 mm spacing and accepts 22 mm; GP20/GP21 run real calibration; off-line start can fail honestly |
| 6 | Servo and timed ultrasound | Check 59/60/511/512 mm, off-axis sector hit, 60 ms trigger gap, NO_RETURN, fault streak, immediate STUCK_HIGH, and time wrap |
| 7 | Per-source hooks, real scan and full detour loop | Real functions run once and match an unhooked run; FOLLOW → STOP → SCAN → DETOUR follows firmware decisions; hook steps match printf labels; inferred phase keeps call/assignment limits |
| 8 | Full contact response and fault scenarios (later) | Both-wheel freeze triggers real stall rules; separating motion and left/right-only injection require explicit models; contact records do not depend on firmware stopping |
| 9 | Offline viewer (implemented) | Play/pause/speed/scrub work from file URLs with network disabled; true/firmware poses, inferred phase, echoes, text, trail, contacts, and ASSUMED list align with saved times |
| 10 | Repeatability and timing sensitivity | Same inputs give identical traces in the same build; compare 50/100 µs clock quanta; source hashes stay unchanged |
| 11 | Second firmware target | Select sources and hooks without changing world/clock core; inventory and implement only extra hardware needs |

Run existing strict host tests and `python3 scripts/check-docs.py` alongside
these checks. Hardware tests remain necessary after simulator results.
