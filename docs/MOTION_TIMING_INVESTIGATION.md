# Motion timing investigation — STEP 8 follow-up

## Evidence and conclusion

The operator reported a lifetime start-gap range of 9916..10087 us on the
first MQTT run and 8952..11038 us on the later ESTOP run. The latter exceeds
the 11000 us gate by 38 us. The later M2 run reported 9902..10099 us.
These are measurements, not a causal trace. No measured cause can yet be
assigned to MQTT, ESTOP, SCAN, a mutex, USB service or interrupt masking.

The existing timestamp in `motion_task` is taken before `tk_loc_mtx`. Thus
mutex waiting increases that tick's wall time but does not directly shift
its already-recorded entry timestamp. A long hold could delay a later tick;
that still needs evidence. Public motion calls hold the same mutex; network
reads normally use the bounded boards instead of this mutex.

The USB header reports lifetime extrema. Run 1's header is printed before
its long GP20 dump. A new extreme caused by that dump can first appear in
run 2's header. Idle/calibration and the first dump are therefore real
confounders: the existing log cannot locate the 11038 us event inside the
second driving run. Normal task priorities remain USB 3, motion 10, sensors
11/12, vehicle 13, Wi-Fi 20, telemetry 30, idle meter 32. These priorities
identify possible preemption, not its measured occurrence.

## Approved diagnostic-only build

```sh
cd build_make
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 DIAG=1 -j4
```

The artifact ends in `_diag.uf2`, distinct from ordinary firmware. `DIAG=0`
is the default and removes every diagnostic path at preprocessing. It does
not change controller settings, priorities, release periods, watchdog feeds,
network ownership or state-machine decisions. No image was flashed here.

With `DIAG=1`, the cyclic callback stores actual TIMER release time and
release interval. Motion records entry, acquisition, phase and queued wakes.
It saves fixed latest and worst-gap snapshots when the entry gap exceeds
11000 us or mutex wait exceeds 1000 us. This is not a queue. Capture has no
new mutex, print, allocation or message send. Only the existing motion mutex
protects controller work. Instrumented API unlocks also retain the previous
owner name and wall-clock hold duration. The hold includes any preemption;
it is not the owner's exclusive CPU time.

GP20 causes a bounded snapshot copy under interrupt masking, then printing
with interrupts restored. There is no diagnostic print from an ISR, cyclic
callback, motion tick, MQTT poll or autonomous periodic path. The print can
itself cause later timing changes; its timing is not a clean production
acceptance measurement.

Phase low byte: 0 idle, 1 calibration, 2 start delay, 3 navigation, 4 stopping.
During navigation, bits 8–15 hold the current shell module (3 SCAN, 4 DETOUR).
Bit 16 marks the USB-dump producer, including its diagnostic print. It does
not prove when the priority-3 USB task actually drained the output ring;
USB activity can continue after the producer returns. Phase changes and
release reads are best-effort core-0 observations, not an atomic trace of
all tasks. Bit 17 marks a changed release during the snapshot, or a release
timestamp later than the recorded entry; do not interpret dispatch delay
from that point. This prevents an entry/release boundary race from appearing
as a huge unsigned delay. Queued wakes also make attribution less exact;
preserve those fields.

| Snapshot values | What they can distinguish |
| --- | --- |
| `gap` | Current entry minus previous entry; same definition as lifetime gap |
| `release_gap` | Actual cyclic dispatch interval; a spike starts before task entry |
| `dispatch = entry - release` | Delay after cyclic dispatch, before the pre-mutex entry timestamp |
| `wait = acquired - entry` | Mutex acquisition plus tiny probe overhead, separate from entry gap |
| `prior`, `hold` | Previous motion mutex owner and wall hold; context, not proof of causality |
| `phase`, `queued` | Navigation/dump context and discarded queued wakes |

All TIMER subtraction is unsigned and handles the 71.6-minute microsecond
wrap. A first release interval has no preceding dispatch and is not a valid
period sample. The latest snapshot may replace an earlier event; the
worst-gap snapshot persists. This intentionally avoids a high-rate trace.

## Operator capture proposal — not executed

1. Use the distinct DIAG=1 NORMAL image only after review. Keep GP21 available.
2. Save one idle GP20 snapshot. Run the same short line test with MQTT on;
   use MQTT ESTOP and then GP20. Record exact phase, power and image hash.
3. Repeat with a scan and with MQTT/broker absent. Keep physical placement
   and commands the same. Do not infer causality from unmatched runs.
4. Press GP20 once more after a quiet interval: this can expose a late event
   from the preceding dump. Do not hide it by resetting lifetime counters.
5. If cyclic dispatch is late, measure the relevant interrupt-off/ISR path
   next. If post-release dispatch is late, instrument the higher-priority
   USB path next. If mutex wait is large, inspect the named API hold next.
   These are proposals, not implemented scheduler or driving changes.

Report those measurements and a proposed narrow fix to the operator before
changing a driving path. The available min/max log is insufficient to choose
that fix now. Do not widen the 11000 us gate.

## Proposed per-run acceptance window — not implemented

Add a per-run gap window beside, not instead of, lifetime values. Reuse the
TEST window principle: count starts inside the window and only complete
gaps whose two entries are inside it; exclude the crossing gap. Begin after
calibration/start delay when navigation starts, and end after the final
stop completes. Record calibration/dump/idle separately. Show window ticks,
gap min/max, late/discarded wakes and work max, while keeping lifetime
extrema/counters visible. This would make run acceptance numbers local to
the actual run without concealing startup/dump anomalies. The exact stop
boundary and new public API require operator approval; this patch does not
silently add that behavior.

## Software proof

- Scripted host cases separate 1038 us dispatch delay from a 2 us mutex wait,
  1086 us mutex wait from a 10000 us entry gap, and late cyclic dispatch from
  post-release delay; wrap, phases, queued wakes and default macro side
  effects are checked. These are synthetic tests, not hardware measurements.
- Six existing DIAG=0 profiles were built at `50d2e52` before and `164f616`
  after the instrumentation (before independent observation-accounting changes):
  923 object/ELF records, zero allocated code/data, relocation, symbol or
  storage-size differences. DWARF/debug/comment attributes and their `.group`
  section-index metadata are excluded; BSS is compared by symbol/size, not
  objdump's synthetic zero display. This is not a whole-file debug-byte claim.
- Negative control: temporarily change watchdog timeout 100 to 101 ms in
  DIAG=0 MQTT, rebuild, and compare. The comparator rejects the changed code
  and ELF. Restore 100 immediately; no mutation image is delivered/flashed.
- DIAG=1 MQTT firmware builds warning-free and has a distinct artifact name.
  Further strict/profile results belong in the final follow-up validation.
