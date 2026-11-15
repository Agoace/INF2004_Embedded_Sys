# Process evidence recorder — 2026-11-15

## Scope

Operator-approved host-only step3, based on `edc9dfb`.
No Pico/serial access, hardware trial, Webots launch or timing-policy change.
The closed physical trial remains unaccepted. This change moves ACTIVE-loop
JSON/CSV/hex conversion and evidence file writes out of `gate1.run_soak`.
Validation, metrics, original I/O timestamps and capture checks stay inline.
Existing post-close exporters stay in the parent, as separately approved.
There is no operational Webots/Pico bridge yet; the transport binary-drain API
is the shared interface for a future bridge, not a claim that one was built.

```text
loop -> bounded binary normal ring -> recorder PROCESS -> binary spool
                 stop ring --------> ordered merge          |
parent close -> fixed close mailbox                  CSV after close
```

`gate1_process_log.py` has no writer thread, manager or Queue feeder thread.
It uses the explicit multiprocessing spawn context; the child receives no I/O
backend/serial handle. As with other spawn programs, caller entry points must
use an `if __name__ == '__main__'` guard. Standard multiprocessing runtime
helpers are not additional application writers.

## Ownership, bounds and publication

| Item | Bound |
| --- | --- |
| Normal ring | 8192 slots x8192 bytes =64MiB |
| Independent stop ring | 256 slots x8192 bytes =2MiB |
| Independent final-close mailbox | 40 bytes |
| Record | Versioned24-byte header plus bounded typed payload,8192 bytes total |
| General payload | At most512 cells, depth12; no silent truncation |
| Packing caches | At most512 string symbols and128 numeric formats |

The normal transport read is at most4096 bytes, so raw chunks fit one slot;
no new transport splitting or device framing rule is needed. Each publication
has a global sequence, used-byte count and diagnostic CRC32. This CRC is NOT
the wire CRC32C; the frozen wire codec is unchanged. Common numeric/raw events
have fixed pack_into layouts; rare/nested fields use bounded typed cells.
Missing/None/bool/int/float remain distinct, including exact JSON number types.
Only used bytes are spooled; unused fixed-slot tails are never exported.

Single producer owns a local head for each ring. The consumer owns local tails.
Producer acquires a free-slot semaphore NONBLOCKING, packs/copies the record,
then publishes a ready credit and wake credit. Consumer acquires ready credits,
merges the two rings by global sequence and copies the record. It releases the
slot ONLY after the complete unbuffered spool write returns. Shared raw counters
are telemetry/recovery only, never an unsynchronised queue-index protocol.
There is no producer wait, retry, busy spin or fallback to synchronous logging
inside the loop. Kernel calls/scheduling still have no proved latency bound.

The recorder must report READY before the run starts. During the loop it only
checks framing/CRC and writes `evidence.bin`. After the parent has attempted
DISARM and closed I/O, it renders the original `events.csv` header/details JSON
with the original sorted-key/CSV formatting. Parent waits boundedly for this
child ONLY AFTER I/O close, then appends existing post-close events/exports.

Internal RX payloads remain bytes. Public `HostTransport.drain_events()` keeps
its legacy hex shape; the soak requests `binary=True`. Original read-return t4
is not moved to the recorder timestamp. Legacy attribution `serialize_*` now
brackets binary packing/publication; `flush_*` brackets nonblocking liveness
polling, NOT JSON writing or a disk flush. Interpret these using `pc_logging`.

## Failure and stop proof

- Full normal ring: retain the failed record in the independent reserve,
  raise FAIL, inhibit, and use the existing failure stop path (no planned drain).
- Stop ring full: raise FAIL/end drain, count the unretained publication and
  mark incomplete. NEVER silently drop, overwrite or report a successful seal.
- Fixed close mailbox is independent of both rings. After close attempt it
  records time, publication count, failed-publication position, close/inhibit
  flags and loss count. Parent persists `recorder-close.bin` even if child died.
- If read-timing logging fails after an OS read, its raw bytes are additionally
  offered to the reserve before raising. Another failure stays explicitly
  latched; it cannot become a complete-evidence result.
- Child death, write/CRC/sequence/export failure, missing close/inhibit proof,
  interrupted publication or bounded finalization timeout means incomplete/FAIL.
  The current post-close render/join budget is5s (test/API override available).
  This bench does NOT establish that5s is sufficient for600s logs. Large/slow
  exports can time out and retain binary recovery evidence; budget/progress
  design must be reviewed before using this for such a run.
  Even a normal overflow with its failed record retained is conservatively
  incomplete: later validation/event construction was aborted.
- Child ignores SIGINT so the parent owns operator inhibit/close. SIGTERM/KILL
  cleanup targets only the owned child PID, with bounded joins. No process-name
  discovery/kills. No shared Event/mutex must be recovered after killing it.

`process-recorder.json` requires matching sent/acknowledged/exported counts,
no loss/failure, close+inhibit proof and clean owned-child exit for
`evidence_complete=true`. `process-recorder-owned.json` records ownership.
Failures retain `evidence.bin`, close proof and `evidence-pending.bin` snapshots.
A death between spool write and acknowledgement can leave duplicate or partial
recovery data: merge by sequence/CRC offline; NEVER treat it as automatic success.
Write completion is not fsync/power-loss durability. Disk exhaustion can prevent
any final file write; no complete-evidence claim is then available.

`logging_mode='process'` is the soak default. Explicit synchronous mode remains
only for host comparison. Standalone `Recorder` retains its synchronous API.
Post-close summary/rejection/attribution/host-spans/tail exporters are unchanged.
There is no change to noearly, per-slot cones, thresholds, source continuity,
lease/session/CRC/readiness/queue/fault checks or hardware acceptance labels.

## Host-only measurements

```sh
python3 sim/webots/tools/gate1_process_bench.py \
  --output build/webots/process-bench-NEW --repeats 10
```

Evidence: `build/webots/process-bench-release-20261115T103519Z/bench.json`.
Before is the exact Gate1/HostTransport source loaded from commit
`edc9dfb495551f0f1357b739e12cb6bbcf112560`; shared clock/wire code is unchanged.
Both replay the frozen158RX/58TX fixture, SHA256
`15bd5058dd6ecbaf9441fb2472c2c21aa278cd2221929e2ba409c5ea54d7299e`.
Fake time advances identically. Measured cost uses RAW wall and thread CPU;
nearest-rank percentiles; no overhead subtraction or GC/timer/priority/affinity
changes. Setup, fake sleeps and post-close export are excluded from loop costs.
Active-loop samples start at the fixture's first SENSOR write:8870 samples
across10 sequential runs per arm. OS load/order/thermal effects are uncontrolled.

All values below are **p50 / p99 / max, microseconds**:

| Measurement | Before | Process after |
| --- | --- | --- |
| Active-loop wall | 8.797 /69.802 /108.564 | 9.919 /72.406 /116.039 |
| Active-loop thread CPU | 8.826 /69.601 /105.639 | 9.949 /72.226 /116.099 |
| Record-call wall | 4.258 /7.264 /15.599 | 5.881 /12.403 /35.998 |
| Record-call thread CPU | 4.278 /6.963 /15.770 | 5.901 /12.123 /36.259 |
| Mixed-record wall | 7.635 /76.314 /83.637 | 5.480 /15.700 /81.644 |
| Mixed-record thread CPU | 7.695 /75.101 /83.737 | 5.520 /15.820 /80.943 |

Mixed CPU bench:2000 fixed records alternating source capture,4096-byte RX,
PC write and32-byte OUTPUT payload. Baseline HEX preparation is OUTSIDE that
microbenchmark, deliberately favourable to baseline. After includes pack,
publication and liveness poll. Observed maximum backlog242/8192; recorder spool
CPU15.314ms and post-close render CPU44.738ms. Extra process/spool/render work
is NOT free. Small-record/loop medians and p99 got worse; large-payload tail
improved. No universal speedup, whole-path bound or hardware result is claimed.
Initial generic-cell regression results remain in ignored WIP logs; the small
fixed-layout path improved packing without changing policy or architecture.

## Equality and tests

Literal CSV files differ because truthful `pc_logging` metadata changes from
`synchronous` to `process`. **Every non-metadata CSV row is byte-identical**.
Normalizing ONLY that metadata value gives byte equality for the whole CSV,
and summary/event semantic equality across all20 runs. Artifact schemas are
unchanged; binary spool/ownership/close/recorder sidecars are additive.

Red tests preceded the module and real-soak integration. Host tests cover all
fixed schemas and numeric types, CRC/size bounds, wrap/order, nonblocking full
normal/stop rings, actual-soak inhibit/close, raw pending recovery, killed child,
SIGINT, interrupted/oversized publication, partial writes, disk-error injection,
hung-child timeout, startup death/I/O close, and parent-loop JSON/CSV/file-write
guards. Existing frozen,
transport, measurement, trial and simulation tests remain required.
