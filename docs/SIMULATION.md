# 2D firmware simulation design

## Status and purpose

Design only. No simulator is implemented. The first firmware target is
`detour_test` v1.5 at `b9dcb6f`. The simulator must execute the real firmware
control code, not a second implementation of its rules.

The simulator has two parts: a host program that runs the firmware against
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
contact result. These are proposed test settings, not hardware measurements.

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
| Front and IR sensors x = 123 mm | Calculated 65 + 58; IR sensors at the front, operator follow-up |
| Width 160 mm; half-width 80 mm each side | Includes the left barcode reader mount; symmetric outline is conservative, body alone is narrower |
| Length 190 mm | Operator overall length; with x front 123, rectangular model rear x = −67 mm is a calculation, not a separate measurement |
| Front-corner radius 146.7 mm | Calculated sqrt(123² + 80²); firmware sweep is 150 mm |

Start with that conservative rectangle and draw the same shape used for
contact. Keep the outline configurable; do not claim that the left mount
fills the full rectangle. The 58 mm offset is an echo mean, not a ruler
measurement. [Ultrasonic run 7](ULTRASONIC_DRIVER.md#run-7-sensor-to-robot-front-offset)
says readings at this short range were about 11 mm high. A 47 mm true
offset gives front x = 112 mm and corner radius about 138 mm. Provide a
sensitivity scenario for that alternative. Servo axis and sensor face are
not proved to coincide: use zero face offset from the axis only as an
explicit provisional model setting, pending a measurement.

IR lateral spacing, line width, and sensor footprint size are not stated
as measured dimensions in [LINE_POSITION.md](LINE_POSITION.md) or
[LINE_STEERING.md](LINE_STEERING.md). Make them required scenario values;
do not invent a measured default. Require physical L/C/R order, with the
centre at y = 0. All three start at the front x offset above.

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
kick unless the firmware commands one. Offer named unloaded-reference and
provisional-loaded profiles. In the loaded profile, shift the curve's zero
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
- Start with opaque rectangle and circle surfaces. Ideal nearest-return
  echoes are a model option, not a claim about real ultrasound. Tilted flat
  faces and thin objects can give no echo; see
  [driver representation rules](ULTRASONIC_DRIVER.md#packed-result).
  Add explicit reflection/dropout options rather than always treating a
  missed ray as physically empty space.
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

### IR readings

Generate raw 12-bit ADC samples, not `line_seen`, position, or events.
The unchanged `line_spin.c` must calibrate through the scenario button;
`line_position.c` must do all learning and threshold decisions.

[Line position run 1](LINE_POSITION.md#run-1-2026-10-27-spin-calibration-usb-power)
uses old labels (L = GP26, physical right; R = GP28, physical left).
The physical order for fake `line_sensors_read` is therefore:

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

### Obstacles and contact

Support rotated rectangles (boxes) and circles (bottles), with scenario
sizes rather than invented D1–D18 dimensions. Detect intersection of the
robot polygon and obstacle boundaries, including edge contact. Record the
first contact time, surface, and robot pose independently of firmware logs.
Prevent tunnelling by checking swept motion or refining contact within a
substep. Do not allow the robot to pass through an obstacle.

Default contact mode freezes both wheels and their encoder progress while
commands push into an obstacle. Permit a separating command to move again. This must exercise the real 500 ms wheel
stall check, not stop the simulator on contact. An explicit test option can
stall left or right alone and integrate the remaining motion with collision
constraints. This is a fault-injection model, not a prediction of contact
forces. Include a record-only mode for comparison, clearly marked as
non-physical because it allows penetration.

## Scenario and trace formats

Choose versioned JSON for scenarios. It is available without another parser
package in the proposed Python output tool and is readable in the browser.
The host implementation can choose a small JSON parser under `sim/`.
Reject unknown required units and missing measured-or-assumed parameters.

| Scenario field | Content |
| --- | --- |
| `schema_version`, `name`, `seed` | Format, human label, fixed noise seed |
| `firmware_target`, `end_us`, `clock_quantum_us` | Target, hard simulated deadline, clock policy |
| `track` | Polylines and circular arcs with line width in mm; joined strokes form junctions; explicit open/closed paths |
| `obstacles` | ID, rectangle centre/size/angle or circle centre/radius, echo material, contact mode |
| `robot` | Initial rear-axle pose, outline, wheel profile, sensor x/y coordinates, IR footprint and ADC endpoints, servo/echo settings |
| `events` | Ordered absolute-time GP20/GP21 press and release events, optional USB connection and sensor faults |
| `noise` | Disabled by default; seed, ADC variation and echo dropout parameters when enabled |

Times are measured from process start, including the firmware's 2 s boot
wait, spin calibration, and 1 s start delay. Press and release durations
must overlap the actual button polling. Warn about ignored presses during
calibration. Do not bypass calibration by setting private firmware state.

Choose a JSON trace rather than CSV: nested geometry, events, text, and
packet bytes do not fit one flat table well. Keep a versioned header with
source commit, source hashes, compiler flags/version, simulator version,
scenario contents, model assumptions, seed, units, and clock quantum.

Record regular 10 ms world snapshots (proposed display interval), plus
exact-time motor commands, servo commands, buttons, ultrasonic triggers and
results, contacts, packets, and text fragments. Snapshots contain pose,
wheel speeds/counts/duties, IR points/raw values, servo angle, and echo hit.
Store packet bytes as well as decoded fields so decoding can be checked.
Preserve text fragments and their timestamps; join lines for display without
losing the original fragments.

### Firmware state visibility without source edits

The run context is local/static inside `detour_test.c`; there is no state
callback. Capture `printf` through a host build rename to `sim_printf` on
firmware translation units, with host format checking and printf built-ins
disabled so calls are not changed to uncaptured `puts`. Keep simulator text
output separate. Test fragment handling and the `long` format widths.

Use the existing 50 ms `steer_log` packets for FOLLOW/SEARCH and detour
step samples, plus `[scan]`, `[obst]`, and `[detour]` text for labels. A
target adapter handles these labels. Packets are sent in batches and replay;
retain their original sample time/index, not delivery time. Mark reconstructed
labels as last observed, with observation time and unknown intervals.
Do not present them as an exact trace of every internal transition: short
STOP/SCAN changes and dropped prints can be ambiguous. If exact per-moment
internal phase is required, this is an operator decision before implementation;
see the open questions. Physics snapshots remain exact at their saved times.

## Viewer choice

Choose the proposed offline HTML canvas viewer. Generate `run.html` with
all CSS, JavaScript, scenario data, and trace embedded; also save `trace.json`
for analysis. Open the HTML directly through a `file:` URL. No server,
network fetch, external libraries, installation, or live mode is needed.
Do not fetch a neighbouring JSON file, because browser local-file rules vary.
Escape embedded text/data so firmware text cannot close a script element or
insert HTML. Render console output as text only.

The viewer shows the floor line, obstacles, conservative robot outline,
IR sample points, servo beam/echo hit, path trail, counts/duties, saved
firmware labels with their observation times, contacts, and printf history.
Controls are play/pause, playback speed, and a time scrubber. Scrubbing uses
saved data only; it never reruns firmware. Interpolate pose only for drawing;
commands, contact, sensor events, and firmware labels change at saved times.
Show gaps rather than invent data. Large traces can have a display-only
reduced trail while the full event data stays available.

## Proposed layout and commands

These paths and commands are planned, not available now. Do not add empty
implementation files as part of this design change.

| Planned path | Content |
| --- | --- |
| `sim/CMakeLists.txt` | Standalone host build; no Pico import |
| `sim/targets/` | Source lists, compile options, and trace label adapters per firmware target |
| `sim/include/pico/`, `sim/include/hardware/` | Minimal SDK-compatible headers actually needed by retained firmware |
| `sim/core/` | Host entry, clock, events, geometry, motion, scenario reader, trace output |
| `sim/devices/` | Five replacement drivers and retained SDK call implementations |
| `sim/scenarios/` | Input JSON and documented assumed parameters |
| `sim/viewer/` | Self-contained HTML template and output generator |
| `sim/tests/` | Device contracts, geometry tests, repeatability checks |
| `sim/build/`, `sim/output/` | Ignored host build and generated runs |

Proposed build from the repository root:

```sh
cmake -S sim -B sim/build -DSIM_FIRMWARE=detour_test
cmake --build sim/build
sim/build/robot_sim --scenario sim/scenarios/box.json --output sim/output/box
```

The host build prepends `sim/include`, then reads `src/` headers. Use C11
and strict warnings for simulator code. Use per-source defines to rename
firmware entry and console calls. A local output generator may use Python's
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

## Open questions for the operator

1. What are the measured IR centre-to-centre spacings, line width, sensor
   footprint, and rear overhang? Confirm physical L/C/R ADC endpoints on
   the current mat, rather than only the old swapped-label run.
2. Measure rear axle to sensor face and front with a ruler. Is 123 mm the
   right model offset, or should the first sensitivity model use 112 mm?
   How far is the sensor face from the servo axis as it turns?
3. Which loaded duty/speed and coast measurements can be used? Approve
   provisional response times and the symmetric reverse-speed assumption
   before interpreting stall/contact outcomes.
4. Supply bottle radius, box dimensions, start poses, and track geometry.
   D1–D18 logs are unavailable on this machine; no replay match is claimed.
5. Is the conservative 160 mm rectangle sufficient, or should the outline
   include the actual asymmetric left barcode mount and front bumper?
6. Approve ideal-sector echoes for the first tests, or require flat-face
   reflection and angular dropout in the first release?
7. Is last-observed firmware state with uncertainty acceptable? Exact state
   access has no API today. No firmware edit is required for the initial
   design. If exact phase at every transition is mandatory, ask separately
   whether a read-only debugger adapter is acceptable, or whether a future
   firmware observation callback in `src/detour_test.c` may be approved.
   The latter would change firmware and is outside this design's rules.
8. Approve a fixed clock-read quantum and both-wheel contact freeze as the
   first model. Which single-wheel stall cases should be fault injections?

## Build plan and checks

| Step | Work | Check before the next step |
| --- | --- | --- |
| 1 | Fix scenario schema and measured/assumed inputs | Reject missing sensor spacing, line width, invalid units, unsorted events; operator reviews assumptions |
| 2 | Standalone source selection and SDK declarations | Host compiles unchanged retained sources; no firmware/root CMake edits; symbol inventory has no unexplained stubs |
| 3 | Clock, entry, exit, console capture, button events | Idle loop and spin busy loops reach deadline; sleeps advance correctly; fragments and exit traces are complete |
| 4 | Motor, encoder, geometry, coast | Straight, spin, reverse, and coast match model equations; signs and fractional counts pass; no obstacle tunnelling |
| 5 | IR floor and real spin calibration | GP20 and GP21 run the real calibration; all three sensors cross the line; off-line start can fail honestly |
| 6 | Servo and timed ultrasound | Check 59/60/511/512 mm, off-axis sector hit, 60 ms trigger gap, NO_RETURN, fault streak, immediate STUCK_HIGH, and time wrap |
| 7 | Real scan and full detour loop | Scenario causes FOLLOW → STOP → SCAN → DETOUR; firmware rather than simulator chooses side and DONE/FAILED |
| 8 | Contact and fault scenarios | Frozen counts trigger real left/right stall rules; trace records contact even if firmware does not stop immediately |
| 9 | Trace decoder and offline viewer | Play/pause/speed/scrub work with network disabled; beam hit, text, trail, and state uncertainty align with saved times |
| 10 | Repeatability and timing sensitivity | Same inputs give identical traces in the same build; compare 50/100 µs clock quanta; source hashes stay unchanged |
| 11 | Second firmware target | Select bypass or steering sources without changing world/clock core; inventory and implement only extra hardware needs |

Run existing strict host tests and `python3 scripts/check-docs.py` alongside
these checks. Hardware tests remain necessary after simulator results.
