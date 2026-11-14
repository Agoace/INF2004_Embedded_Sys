# Gate1 D-TRIAL decision — 2026-11-14

**TRIAL / NOT GATE 1 PASS.** This is an operator-approved experiment, not a new
default or a change to physical Gate1 acceptance.

## Decision and scope

The operator approved an explicit `--trial-clock-400-sync20` mode after attempt9
could not qualify the default250us clock at bootstrap. Its eight bootstrap
exchanges had net RTT1453–1892us. Earlier loaded evidence also failed to sustain
250us. These observations do not prove a new600s bound.

The initial trial implementation `cbe65c2` starts at C1 `005db04` only.
After attempt10's source-phase failure, the operator approved attempt11 with
C2 `52d0d44` applied on top of that trial. That attempt captured first after
inhibit/expiry checks and used synchronous C1 logging. RX faults could cancel
queued SENSOR before TX, with a second expiry/lease check after RX. C3 was
excluded from that attempt. Keep these experiment results separate.
After attempt11, the operator also approved C2b on `81c5898`: fresh capture
checks after RX/drain BEFORE SYNC request generation, then again after SYNC
construction. Each check first uses fresh time for inhibit/expiry. An iteration
can capture at most one source slot across all three opportunities. A capture
made after a received SYNC uses fresh start/end timestamps and the same cone
state; it never revalidates an older capture using a newer observation. RX
faults still cancel queued TX. No catch-up, skip, epoch shift or early capture
is added. A fresh post-SYNC expiry check protects TX even if the initial check
already captured. Sleep remains50us; no timer-slack/deadline-wait/GC/scheduler
or C3 change is included. C2b restores a missed post-RX opportunity, not a
latency guarantee.

After attempt12, the operator approved C3 `3326d91` on top of `a6ee177` as
its own integration. The current manifest and PC metadata identify
**C1+C2+C2b+C3**, the same three capture checks, and bounded queued logging.
There is NO C2c pre-sleep check. Source-loop sleep stays50us; timer slack,
priority, clock limits, native firmware and UART v1 stay unchanged.

C3 snapshots evidence into the existing8192-entry single-producer queue; one
writer owns JSON/RAW-byte hex/CSV/flush. Full/error inhibits and fails. Reserved
stop/rejection evidence and immutable snapshots remain. Inhibit, bounded DISARM
attempt and serial close precede the bounded writer wait. Writer timeout/error
cannot produce a successful close acknowledgement or TRIAL_COMPLETE. File close
acknowledgement is not native DISARM delivery. Status files carry trial labels.
A missing numeric tail when the writer does not quiesce is explicitly incomplete.
Concurrent writer/GC observer stamps can be suppressed; the legacy
`nested_callbacks_not_recorded` count is not a GC-only count. The diagnostic
ring is not full-run proof, and the writer still shares the Python GIL.

Before writer creation, the calling thread now reads PR_GET_TIMERSLACK once.
Run metadata and summary record `timer_slack` with status, nanoseconds, source
and calling native thread ID. Unsupported/failed reads remain UNKNOWN, not an
assumed50000 or zero. No PR_SET_TIMERSLACK or scheduler call is added. This
snapshot describes this run, not the ended attempt12 thread.
The paper review remains operator workspace material:
`build/wip/pil_plan/GATE1_BOUND_REVIEW_PAPER.md`.

| Item | Default | D-TRIAL |
| --- | --- | --- |
| Aged clock half-width |250us|400us, explicitly opted in |
| Maximum adaptive SYNC request period |100ms|20ms; bootstrap/near-deadline fast refresh remains |
| Source midpoint phase |250us|250us, unchanged |
| Native entry/commit jitter |250us with existing outward+1|Unchanged |
| Availability / nominal pipeline / expiry |3/4/5ms|Unchanged |
| Instantaneous future clock guard |100ppm|Unchanged |
| UART v1, framing, queues, native leases/faults |Existing contract|Unchanged |

Default invocation and the class default remain250us/100ms. There is no general
arbitrary-bound parameter. D-TRIAL requires the explicit transport-only cap4
profile. It does not authorize device access without `--hardware-authorized`.

## Source policy and proof obligations

Every trial capture must have a qualified aged clock at actual capture start AND
end, with the same cone state used to record its source interval. The estimated
start midpoint must not precede its unchanged intended source target. The
shared timeline validator enforces this, including direct callers that bypass
the soak pacing loop. No early-aim correction, learned bias, catch-up, skipped
slot or epoch shift is permitted. Actual source time in the Pico domain can
still be early within clock uncertainty; this is not a physical sensor capture.

The no-early-midpoint invariant gives H<=400us for the oldest source interval;
with native J250, its sufficient apply-age ceiling is4650us. This does not waive
actual evidence. Keep exact D1 upper AND lower age/order checks,3ms availability,
5ms source expiry, native lookahead and native250us jitter. A negative lower age
is still rejected. Full simulator cache use needs its own original-source-age
proof, not just receipt age. Gate1 cap4 has no physical sensor/physics ACK claim.

## Artifacts and reports

All trial summaries, run metadata, attribution output, rejection evidence and
reports identify `trial_clock_400_sync20=true` and **TRIAL / NOT GATE 1 PASS**.
`trial-manifest.json` records the selected bounds and unchanged chain. Keep that
sidecar with external raw USB/pcap captures: their immutable binary bytes are
not altered to insert text or change UART v1.

A measured fault remains FAIL; missing proof remains INCOMPLETE. Completion can
be labelled at most TRIAL_COMPLETE. CLI exit2 also applies to TRIAL_COMPLETE;
it is NOT success/acceptance exit0. Physical Gate1 PASS remains impossible.
The D1 report never sets the standard `transport_subset_qualified` flag true for
a trial; any qualified trial subset uses a separate explicitly trial field.
Reporters check recorded mode/bounds and no-early source evidence; default250us
source qualification is not silently widened when reading old files.

## Pico review — no image change

SW6 confirmed the existing IRQ-wake image `40e7196` needs no change for20ms SYNC.
Its UF2 SHA256 is
`772d4e433242329264a327c0d24799ec89409b83ac7ce0af16041b8248a5907a`.
There is no native offset-half/phase or SYNC-cadence gate to alter. Do NOT change
the native249 entry/commit checks: outward+1 gives250. SYNC does NOT renew the
5ms SENSOR progress lease. Native current+next readiness and all fault paths stay.

At50 SYNC/s, nominal directional traffic is67700B/s PC→Pico and60100B/s back,
about73.46%/65.21% of nominal8N1 capacity. These are throughput estimates, NOT
burst/WCET/queue/latency proof. Shared RX/TX queues remain eight entries; service
limits and worker priority remain unchanged. RX_MUT/TX_MUT include SYNC put/drop;
active OUTPUT/ENQUEUED/FIFO counts do not include SYNC. D1/RX1 schemas and all
identity/error checks remain unchanged. D1 has no clock-bound field; PC metadata
must bind this trial to the native evidence.

## Operator command difference

Use the existing approved capture/reset/image/soak procedure with a fresh
output directory and the same verified image. Add only:

```text
--trial-clock-400-sync20 --transport-only-cap4
```

Keep `--measure-io` for the C1 baseline. Retain the manifest with the external
capture directory before interpreting any binary artifact. Do not reuse an
existing output or infer a PASS from a generated JSON report. No hardware action
is performed by tests or by reading this record.
