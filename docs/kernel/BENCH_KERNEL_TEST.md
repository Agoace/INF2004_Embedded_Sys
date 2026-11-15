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

## Report package

- Files: `app_program/bench_kernel/bench_report.{c,h}`; test:
  `tests/test_bench_report.c`. Link flag (BENCH_KERNEL=1 only):
  `-Wl,--wrap=rt_timeline_report`.
- One line after the GP20 `[timeline]` dump; it uses `bench_stats_read` only.

| Output | Meaning |
| --- | --- |
| `[bench] no snapshot` | No valid publication, or the read overlapped one |
| `[bench] window open` | Generator still running; counters are not read |
| `[bench] window_us W sched S delivered D coalesced C overflow O gen_calls N gen_us_sum U gen_us_max M cost_bp B V` | Stopped window; B = U x 10000 / W |

- V: `OK` (B < 300), `COST_HIGH` (B >= 300), `BAD_WINDOW` (W = 0 or W > 60 min).
- Timeline sites (bench only): 10 = generator exit (arg = edges), 11 = pending
  overflow.

## Virtual GPIO seam

- `bench_gpio.{c,h}` owns AB states and virtual pending bits; `bench_gpio_hw.c`
  supplies the existing pending/ACK/enable API. Native GPIO code is unchanged.
- Core0, interrupt-serialized access only. Reject two-bit jumps, unsubscribed
  edges and a second edge while that wheel has an unacknowledged event.
- Delivered count increments only when an ACK clears a published edge; duplicate
  or wrong-polarity ACK does not count. No native encoder-count assignment.
- GCC, Clang ASan/UBSan and strict ARM checks pass; host test covers these seams.
- No NVIC producer, timer scheduling or complete bench image is included yet.

## Ownership and gates

- SW6: stimulus/providers, source selector, isolated reset record and tests.
- SW7: P1/P2, stack/latency measurement, `bench_report`, integration.
- All bench objects are excluded from ordinary links.76 ordinary ELF/UF2 hashes,
  including debug data, must match. New runtime packages require separate checks.
- No per-slot Webots timing rules or transport acceptance claims apply here;
  use the actual robot deadlines and preserve the real motion watchdog.
