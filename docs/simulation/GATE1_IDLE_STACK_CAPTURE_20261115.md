# Gate1 idle stack evidence — 2026-11-15

**TRIAL / NOT GATE 1 PASS.** This parser change supports the operator-approved
Gate1 application-idle A/B image. It does not implement that task, change
source timing, access hardware or qualify physical Gate1.

## Frozen stack line

The existing line can have this exact optional suffix before its LF:

```text
[PILG1] stopped; reset required; worker stack untouched N/2048 bytes; idle stack high-water U/512 bytes
```

N and U are decimal. Legacy lines without the suffix remain supported.
LF and CRLF are accepted; missing newline, other suffixes or other denominators
are not complete stack evidence. N must be0..2048 and U must be0..512.

`gate1_measurement.py` keeps `stopped_stack_line_complete` and adds:
- `stopped_worker_stack_untouched_bytes`: N, bytes still untouched, not usage.
- `stopped_idle_stack_high_water_bytes`: U, painted-stack high-water usage.

Both values are null for an invalid/incomplete line. On a valid legacy line,
only the idle value is null. U=0 from a host double does not show that the real
task uses no stack. A watermark alone does not prove stack safety or a margin.
All measurement verdict and original fault checks remain unchanged.

The CDC STOPPED prefix matcher already accepts the appended text on the SAME
line and waits for the final newline. It does not close at a separate earlier
stack row. Full D1/stack detection still requires the existing reset-watch
grace; RESET1/BC1 and their companion wait remain unchanged. D1/RX1 schemas,
cohort identities and CRC checks are unchanged.

## Actual C fixture checks

SW6 supplied three unchanged actual C runtime streams, tracked under
`sim/webots/tests/fixtures/`:

| File | Bytes | Idle U | RX1 packets/alarms |
| --- | --- | --- | --- |
| pil_gate1_idle_runtime.bin | 5918 | 0 | 14/10 |
| pil_gate1_idle_missing.bin | 2709 | 0 | 4/3 |
| pil_gate1_idle_painted.bin | 5919 | 96 | 14/10 |

All report N=2048. The first two host doubles do not execute on the provided
stack. The painted fixture explicitly overwrites96 top bytes. These are
synthetic C tests, not physical stack measurements.

`tests/test_gate1_idle_stack_capture.py` independently checks all three D1/RX1
identities/cohorts, unchanged122-byte RESET1 and281-byte BC1, measurement JSON
and split-read CDC completion. Red tests preceded implementation. Range, exact
suffix, legacy, missing newline and CRLF cases are also checked. No native
firmware or shared navigation files are changed by this PC commit.
