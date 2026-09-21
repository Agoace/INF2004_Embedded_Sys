# Telemetry JSON schema

Status: Implemented for the current telemetry targets

## Purpose

Telemetry is an instrument panel. It reports physical values that a person can
read at a glance. It is not a data pipeline and it is not a transport for
calibration records.

The Pico keeps all calibration work in its USB report and RAM. This includes
counts per revolution, drift, coast overshoot, ISR boundary timestamps, jog
arithmetic, and every completed result. Wi-Fi telemetry does not affect those
measurements.

The Pico converts encoder counts to physical speed before it sends a packet.
The Pico owns `COUNTS_PER_REVOLUTION` (`2535`). The server does not convert
counts. This keeps one calibration constant and one conversion site.

## Contract

Each packet is one JSON object followed by one newline. Firmware and the
receiver use schema version `2`. The envelope is shared by every target. The
`test` value is a display label only; it does not select a different payload
schema.

Every envelope field is required:

| Key | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `ver` | integer | none | Schema version. The current value is `2`. |
| `device` | string | none | Device identifier. |
| `session` | integer | none | Boot session identifier. |
| `seq` | integer | none | Packet sequence number. It increases for every send attempt. |
| `ms` | integer | milliseconds | Pico monotonic timestamp when the packet was built. |
| `link` | integer | CYW43 status code | Wi-Fi link status when the packet was built. |
| `test` | string | none | Target label for the dashboard. It does not select a schema. |
| `readings` or `notice` | object | none | One instrument reading object or one terminal notice object. Exactly one is present. |

A receiver must reject a version it does not understand. It must not guess how
to decode it. A new optional field inside `readings` is not a breaking change.
A changed meaning, type, required envelope field, or removed field requires a
new schema version.

Identifiers and state strings use only ASCII letters, digits, `-`, and `_`.
They must not contain spaces, quotes, control characters, or JSON escape
characters. The receiver bounds all received strings before it prints them.
All JSON numbers are integers. The suffixes in field names state the unit:

- `mrpm` is milli-RPM. Divide by 1,000 for RPM.
- `mm` is millimetres.
- `mv` is millivolts.
- `pct` is a percentage from 0 to 100.

## Instrument readings

A normal packet contains a `readings` object. `state` is required. All other
fields are optional because a target may not have that hardware or value.
There are no test-specific payloads and no state-dependent required sets.

| Key | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `state` | string | none | Current state, such as `running`, `idle`, `stopped`, `stalled`, or `emergency_stop`. |
| `motor_1_speed_mrpm` | integer | milli-RPM | Motor 1 wheel speed converted on the Pico from encoder counts using 2535 counts per revolution. |
| `motor_2_speed_mrpm` | integer | milli-RPM | Motor 2 wheel speed converted on the Pico from encoder counts using 2535 counts per revolution. |
| `motor_1_duty_pct` | integer | percent | Motor 1 commanded effort as a duty percentage. |
| `motor_2_duty_pct` | integer | percent | Motor 2 commanded effort as a duty percentage. |
| `motor_1_direction` | string | none | Motor 1 direction, such as `forward`, `reverse`, or `stopped`. |
| `motor_2_direction` | string | none | Motor 2 direction, such as `forward`, `reverse`, or `stopped`. |
| `range_mm` | integer | millimetres | Ultrasonic obstacle range. |
| `ir_mv` | integer | millivolts | IR sensor reading. |
| `supply_mv` | integer | millivolts | Reserved supply voltage field. It is not implemented because GP28 is allocated to a line-tracing IR sensor instead of VBAT sense. |
| `distance_travelled_mm` | integer | millimetres | Reserved. This is absent until wheel diameter is measured. Do not invent a diameter. |
| `progress_pct` | integer | percent | Short progress indicator when a target has progress to show. |

The dashboard does not receive encoder counts, PWM register values, echo pulse
microseconds, or calibration measurements.

### Supply voltage hardware note

The Robo Pico VBAT sense jumper routes `VIN/2` to GP28/ADC2. GP28 is allocated
to a line-tracing IR sensor, so the jumper and the supply reading are not used.
The `supply_mv` key remains reserved and absent. A future supply change may
populate it without a schema version bump, but it will require rewiring GP28
and adding the firmware reading.

### Distance status

`distance_travelled_mm` is reserved but is not sent. Wheel diameter, tyre
compression, and slip have not been measured. The field remains absent until
wheel diameter is measured and the conversion is implemented on the Pico.

## Terminal notice

When a run ends, the Pico may send a separate `notice` object:

| Key | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `outcome` | string | none | Run outcome, such as `completed`, `stalled`, `safety_timeout`, `emergency_stop`, or `incomplete`. |
| `fault` | integer | flag, 0 or 1 | `1` means the outcome is a fault. |

A terminal notice contains only the outcome and fault indicator. It contains no
measured values. This gives the instrument panel a status light beside its
gauges, so the panel can show `run finished, completed` or `run finished,
stalled` instead of going silent.

## Packet size

The 1400-byte limit includes the newline and stays below the usual 1500-byte
Ethernet MTU while leaving room for IP and UDP headers. The worst-case version
2 readings packet, with every defined reading present, maximum integer widths,
and maximum bounded string widths, is about **723 bytes**. This includes the
reserved `distance_travelled_mm` field. It leaves about **677 bytes of
headroom**. Firmware drops an over-limit packet and counts an error; it never
sends malformed JSON. Recheck this size when adding fields.

## Worked packets

Full instrument readings from the motor target:

```json
{"ver":2,"device":"pico-w-rotation-drift","session":305419896,"seq":12,"ms":12000,"link":3,"test":"rotation_drift","readings":{"state":"running","motor_1_speed_mrpm":22800,"motor_1_duty_pct":10,"motor_1_direction":"forward","progress_pct":42}}
```

A probe target uses the same readings shape. It has no motor or sensor values:

```json
{"ver":2,"device":"pico-w-telemetry-probe","session":305419896,"seq":0,"ms":1000,"link":3,"test":"probe","readings":{"state":"idle"}}
```

A terminal notice has no readings:

```json
{"ver":2,"device":"pico-w-rotation-drift","session":305419896,"seq":60,"ms":131000,"link":3,"test":"rotation_drift","notice":{"outcome":"completed","fault":0}}
```
