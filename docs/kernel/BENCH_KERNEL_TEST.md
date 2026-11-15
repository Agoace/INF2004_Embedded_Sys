# On-chip kernel bench

| Item | State |
| --- | --- |
| Approved scope | Spare Pico; real robot tasks; UP/split; on-chip stimulus |
| First run | Encoder seam,10min; no reset; delivered edges match native counts; generator cost below3% |
| Next run | Line/barcode model and GP21 calibration/run;90s x10 per image |
| Fidelity target | Matched robot P1 rates within25%; also inspect bursts/missed wakes |
| Excluded | MQTT phase, Webots, hardware access by coding workers |

## Statistics package

- Files: `app_program/bench_kernel/bench_stats.{c,h}` and
  `bench_stats_publish.h`; test: `tests/test_bench_kernel_stats.c`.
- Core0 is the sole, non-reentrant publisher. `bench_stats_read` is any-core,
  one attempt, no busy wait. False leaves the destination unchanged.
- Sequence and fields use atomic32 access with release/acquire ordering.
- Fields: scheduled/delivered edges, coalescence/overflow, generator sum/max/calls,
  window start/end and explicit stopped flag. Counters are32-bit.
- Windows above60min invalidate publication. Modulo timestamps permit timer wrap;
  end timestamp0 does not mean running. Read stopped data for the final report.
- `bench_stats_get` is reserved for the core0 runtime owner, not implemented by
  this publication module. Cross-core readers must use `bench_stats_read`.
- This package does not start a generator or provide an operational bench image.

## Ownership and gates

- SW6: stimulus/providers, source selector, isolated reset record and tests.
- SW7: P1/P2, stack/latency measurement, `bench_report`, integration.
- All bench objects are excluded from ordinary links.76 ordinary ELF/UF2 hashes,
  including debug data, must match. New runtime packages require separate checks.
- No per-slot Webots timing rules or transport acceptance claims apply here;
  use the actual robot deadlines and preserve the real motion watchdog.
