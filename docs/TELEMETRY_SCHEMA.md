# Telemetry JSON schema

Status: Implemented for the current telemetry targets

## Contract

Each telemetry packet is one JSON object followed by one newline. The object
must fit in one UDP datagram. Firmware and the receiver use schema version
`1`.

The maximum packet size is **1400 bytes, including the newline**. This stays
below the usual 1500-byte Ethernet MTU while leaving room for IP and UDP
headers. Firmware builds the line in a bounded 1400-byte buffer and drops a
packet if formatting would exceed it. The receiver rejects packets larger than
1400 bytes before JSON parsing.

The worst-case `rotation_drift` packet, with every optional field present at
maximum integer width, is about 1243 bytes. This leaves roughly 157 bytes of
headroom. Adding several more optional fields could exceed the limit. Firmware
will then drop the packet and count an error instead of sending malformed JSON.
A contributor who adds fields must recheck this size.

All JSON numbers are integers. The protocol does not use floating point.
Scaled values use an explicit suffix in the key name:

- `mdeg` is millidegrees. Divide by 1,000 for degrees.
- `milli` in `*_milli` is a value multiplied by 1,000. For example,
  `counts_per_rev_milli` of `2535000` means 2535.000 counts per revolution.
- `*_us` is microseconds.
- `*_ms` is milliseconds.
- `duty_pct` is a percentage from 0 to 100.

Identifiers and state strings use only ASCII letters, digits, `-`, and `_`.
They must not contain spaces, commas, quotes, control characters, or JSON
escape characters. Device and test identifiers are compile-time constants.
The receiver also bounds all received strings before it prints them.

## Envelope

Every packet has these fields at the top level:

| Key | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `ver` | integer | none | Schema version. The current value is `1`. |
| `device` | string | none | Device identifier. |
| `session` | integer | none | Boot session identifier. It is stable for the session and changes between boots. |
| `seq` | integer | none | Packet sequence number. It increases for every send attempt in the session. |
| `ms` | integer | milliseconds | Pico monotonic timestamp when the packet was built. |
| `link` | integer | CYW43 status code | Wi-Fi link status when the packet was built. |
| `test` | string | none | Test-type discriminator. It selects the payload schema. |
| `payload` | object | none | Test-specific fields. Fields in this object cannot collide with fields in another test payload. |

All envelope fields are required. A receiver must reject a `ver` value it does
not understand. It must not guess how to decode an unknown version. A new
optional field inside `payload` is not a breaking change. A changed meaning,
type, required field, envelope field, or removed field requires a new schema
version.

## `rotation_drift` payload fields

The field table defines the type, unit, and meaning. The state sets below define
which fields are required for each kind of packet. A field not listed for the
current state is absent unless it is marked as a truthful optional group.

| Key | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `state` | string | none | Packet state. |
| `direction` | string | none | Motor direction, normally `forward` or `reverse`. |
| `startup` | string | none | Startup state, such as `grace` or `complete`. |
| `stall` | string | none | Stall state, such as `clear` or `active`. |
| `emergency` | string | none | Emergency-stop state, such as `clear` or `active`. |
| `error` | string | none | Error state, or `none`. |
| `motor` | integer | motor number | Motor number, normally 1 or 2. |
| `duty_pct` | integer | percent | PWM duty. |
| `revolutions_requested` | integer | revolutions | Requested test revolutions. |
| `revolution_index` | integer | revolution index | Current completed revolution index during progress. |
| `absolute_count` | integer | encoder counts | Current absolute encoder count. |
| `elapsed_ms` | integer | milliseconds | Main-loop or result elapsed time. |
| `ideal_counts` | integer | encoder counts | Ideal count target before undershoot or compensation. |
| `target_counts` | integer | encoder counts | ISR stop target. |
| `count_at_stop` | integer | signed encoder counts | Signed count read by the main loop after the stop. |
| `absolute_count_at_stop` | integer | encoder counts | Absolute version of `count_at_stop`. |
| `count_at_trigger` | integer | signed encoder counts | Signed count captured by the ISR at the stop trigger. |
| `absolute_count_at_trigger` | integer | encoder counts | Absolute version of `count_at_trigger`. |
| `count_after_settle` | integer | signed encoder counts | Signed count after brake/coast settling. |
| `absolute_count_after_settle` | integer | encoder counts | Absolute version after settling. |
| `coast_overshoot_counts` | integer | encoder counts | Count movement after the ISR stop trigger. |
| `boundary_count` | integer | boundary count | Number of revolution boundaries captured. |
| `undershoot_counts` | integer | encoder counts | Deliberate target undershoot. |
| `drift_counts` | integer | encoder counts | Final count minus ideal count. |
| `drift_mdeg` | integer | millidegrees | Drift angle multiplied by 1,000. |
| `drift_per_rev_mdeg` | integer | millidegrees per revolution | Drift per requested revolution multiplied by 1,000. |
| `elapsed_us` | integer | microseconds | Result elapsed time from ISR boundary timing. |
| `mean_us_per_rev` | integer | microseconds per revolution | Mean elapsed time per requested revolution. |
| `mean_ms_per_rev` | integer | milliseconds per revolution | Millisecond form of the mean time. |
| `counts_per_rev_milli` | integer | millicounts per revolution | Counts per revolution multiplied by 1,000. |
| `sensitivity_millicounts_per_degree` | integer | millicounts per degree | CPR change per degree multiplied by 1,000. |
| `jog_counts` | integer | signed encoder counts | Cumulative forward jog count. |
| `jog_final_absolute_count` | integer | encoder counts | Absolute count after the latest jog. |
| `jog_combined_count` | integer | encoder counts | Settled count combined with the jog count. |
| `smallest_jog_movement_counts` | integer | encoder counts | Smallest movement from one jog pulse. |
| `alignment_resolution_cpr_milli` | integer | millicounts per revolution | Jog resolution expressed as millicounts per revolution. |
| `jog_primary_cpr_milli` | integer | millicounts per revolution | Jog combined count over the requested revolutions, scaled by 1,000. |
| `jog_secondary_cpr_milli` | integer | millicounts per revolution | Jog combined count over one more revolution, scaled by 1,000. |
| `split_index` | integer | zero-based index | Index of this split packet. |
| `split_total` | integer | split packets | Total split packets expected. |
| `split_revolution` | integer | revolution number | One-based revolution number. |
| `split_absolute_count` | integer | encoder counts | Absolute count at this boundary. |
| `split_elapsed_us` | integer | microseconds | Boundary timestamp from the ISR. |
| `split_delta_counts` | integer | encoder counts | Count difference from the previous boundary. |
| `split_delta_us` | integer | microseconds | Time difference from the previous boundary. |
| `completed` | integer | flag, 0 or 1 | `1` marks a terminal result or result split. `0` marks progress or jog data. |

### Required field sets by state

The following sets describe the fields sent by the current firmware:

- `running` requires `state`, `direction`, `startup`, `stall`, `emergency`,
  `error`, `motor`, `duty_pct`, `revolutions_requested`, `revolution_index`,
  `absolute_count`, `elapsed_ms`, and `completed`. It uses `completed: 0`.
- `jogging` requires `state`, `direction`, `motor`, `duty_pct`,
  `absolute_count`, `jog_counts`, `jog_final_absolute_count`,
  `jog_combined_count`, `smallest_jog_movement_counts`,
  `alignment_resolution_cpr_milli`, `jog_primary_cpr_milli`,
  `jog_secondary_cpr_milli`, and `completed`. It uses `completed: 0`.
- `completed` requires `state`, `direction`, `startup`, `stall`, `emergency`,
  `error`, `motor`, `duty_pct`, `revolutions_requested`, `ideal_counts`,
  `target_counts`, `count_at_stop`, `absolute_count_at_stop`,
  `count_after_settle`, `absolute_count_after_settle`,
  `coast_overshoot_counts`, `boundary_count`, `elapsed_ms`, and `completed`.
  A valid drift result also requires `count_at_trigger`,
  `absolute_count_at_trigger`, `undershoot_counts`, `drift_counts`,
  `drift_mdeg`, `drift_per_rev_mdeg`, `elapsed_us`, `mean_us_per_rev`,
  `mean_ms_per_rev`, `counts_per_rev_milli`, and
  `sensitivity_millicounts_per_degree`. The jog field group is optional because
  the first completed summary is sent before jog alignment finishes; it is
  present after jog alignment.
- `completed_split` requires `state`, `direction`, `motor`, `split_index`,
  `split_total`, `split_revolution`, `split_absolute_count`,
  `split_elapsed_us`, `split_delta_counts`, `split_delta_us`, and `completed`.
  It uses `completed: 1`. These packets are individually addressable so the
  receiver can reassemble the split series and report missing splits.
- `stalled`, `emergency_stop`, `safety_timeout`, and `incomplete` require
  `state`, `direction`, `startup`, `stall`, `emergency`, `error`, `motor`,
  `duty_pct`, `revolutions_requested`, `ideal_counts`, `target_counts`,
  `count_at_stop`, `absolute_count_at_stop`, `count_after_settle`,
  `absolute_count_after_settle`, `coast_overshoot_counts`, `boundary_count`,
  `elapsed_ms`, and `completed`. They use `completed: 1`. They do not contain
  trigger, drift, CPR, or derived timing fields because there is no valid
  completed drift measurement.

The receiver rejects a packet when a field required by its state is missing.
The receiver may accept a future optional field without treating it as a
schema break, but it must not invent a meaning for a changed field.

## Test types

The `probe` type is implemented by `wifi_telemetry_probe`. Its payload has a
required `state` string, normally `running`.

These names are reserved. Their payloads are not implemented yet:

| Test value | Intended use |
| --- | --- |
| `encoder_calibration` | Motor encoder repeatability and angular-drift calibration. |
| `duty_calibration` | Minimum and stepped motor-duty calibration. |
| `speed_calibration` | Motor RPM versus duty calibration. |
| `board_diagnostics` | Robo Pico sensor and wiring diagnostics. |

Reserved payloads must still use the common envelope and a nested `payload`
object. Their initial payload contract is a required `state` string. A future
implementation must define any additional payload fields here before it sends
them.

## Worked packets

In-run rotation-drift progress:

```json
{"ver":1,"device":"pico-w-rotation-drift","session":305419896,"seq":12,"ms":12000,"link":3,"test":"rotation_drift","payload":{"state":"running","direction":"forward","startup":"complete","stall":"clear","emergency":"clear","error":"none","motor":1,"duty_pct":10,"revolutions_requested":50,"revolution_index":4,"absolute_count":10140,"elapsed_ms":12000,"completed":0}}
```

Rotation-drift completed summary:

```json
{"ver":1,"device":"pico-w-rotation-drift","session":305419896,"seq":60,"ms":131000,"link":3,"test":"rotation_drift","payload":{"state":"completed","direction":"forward","startup":"complete","stall":"clear","emergency":"clear","error":"none","motor":1,"duty_pct":10,"revolutions_requested":50,"ideal_counts":126750,"target_counts":126450,"count_at_stop":126450,"absolute_count_at_stop":126450,"count_at_trigger":126450,"absolute_count_at_trigger":126450,"count_after_settle":126475,"absolute_count_after_settle":126475,"coast_overshoot_counts":25,"boundary_count":49,"undershoot_counts":300,"drift_counts":-275,"drift_mdeg":-39000,"drift_per_rev_mdeg":-780,"elapsed_ms":128000,"elapsed_us":128000000,"mean_us_per_rev":2560000,"mean_ms_per_rev":2560,"counts_per_rev_milli":2529500,"sensitivity_millicounts_per_degree":704,"completed":1}}
```

One rotation-drift split:

```json
{"ver":1,"device":"pico-w-rotation-drift","session":305419896,"seq":61,"ms":131001,"link":3,"test":"rotation_drift","payload":{"state":"completed_split","direction":"forward","motor":1,"split_index":0,"split_total":49,"split_revolution":1,"split_absolute_count":2535,"split_elapsed_us":2600000,"split_delta_counts":2535,"split_delta_us":2600000,"completed":1}}
```

Jog-mode update:

```json
{"ver":1,"device":"pico-w-rotation-drift","session":305419896,"seq":110,"ms":140000,"link":3,"test":"rotation_drift","payload":{"state":"jogging","direction":"forward","motor":1,"duty_pct":10,"absolute_count":126478,"jog_counts":28,"jog_final_absolute_count":126478,"jog_combined_count":126478,"smallest_jog_movement_counts":3,"alignment_resolution_cpr_milli":60,"jog_primary_cpr_milli":2529560,"jog_secondary_cpr_milli":2480,"completed":0}}
```

Probe status:

```json
{"ver":1,"device":"pico-w-telemetry-probe","session":305419896,"seq":0,"ms":1000,"link":3,"test":"probe","payload":{"state":"running"}}
```

Reserved target examples:

```json
{"ver":1,"device":"pico-w-encoder","session":305419896,"seq":0,"ms":0,"link":3,"test":"encoder_calibration","payload":{"state":"reserved"}}
```

```json
{"ver":1,"device":"pico-w-duty","session":305419896,"seq":0,"ms":0,"link":3,"test":"duty_calibration","payload":{"state":"reserved"}}
```

```json
{"ver":1,"device":"pico-w-speed","session":305419896,"seq":0,"ms":0,"link":3,"test":"speed_calibration","payload":{"state":"reserved"}}
```

```json
{"ver":1,"device":"pico-w-board","session":305419896,"seq":0,"ms":0,"link":3,"test":"board_diagnostics","payload":{"state":"reserved"}}
```
