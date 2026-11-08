# Heartbeat

The heartbeat is the one time base for all modules on core 0. Decision:
[ADR: heartbeat](../architecture/ADR_HEARTBEAT.md). Branch: `yx-heartbeat`.

- Core (pure, host-tested): [`src/heartbeat.c`](../../src/heartbeat.c), [`src/heartbeat.h`](../../src/heartbeat.h)
- Pico layer (timer and watchdog): [`src/heartbeat_pico.c`](../../src/heartbeat_pico.c), [`src/heartbeat_pico.h`](../../src/heartbeat_pico.h)
- Host unit test: [`tests/test_heartbeat.c`](../../tests/test_heartbeat.c)
- Bench target: `heartbeat_test` ([`src/heartbeat_test.c`](../../src/heartbeat_test.c)); no motors move

## How it works

```text
 hardware timer, 1 ms (start-to-start)
   └─ interrupt: hb_tick()
        1. hooks that are due (inside the interrupt: ADC sample, mc_tick)
        2. flags that are due: posted[i] + 1
      watchdog restart
      hb_isr_done(): time of this interrupt's work
 main loop:
   if (hb_take(&hb, flag_10ms)) { detectors → running module → Vehicle Controller }
```

| Part | Rule |
| --- | --- |
| Tick | 1 ms (`HB_TICK_US` 1000) |
| Hook | `hb_add_hook(hb, period, phase, fn, ctx)`: `fn` runs in the interrupt when `tick % period == phase`. Up to 4 hooks |
| Flag | `hb_add_flag(hb, period, phase)`: posted when `tick % period == phase`. Up to 8 flags |
| Order in one tick | Hooks first, then flags. A main loop that sees a flag also sees that tick's hook work done |
| Phase | Spreads work over the ticks (for example the 20 ms button flag at phase 1, away from the 10 ms work at phase 0) |
| Set-up | Add all hooks and flags before `heartbeat_start()`; the tables do not change after that |
| Flag sharing | No interrupt-off section. The interrupt writes only `posted[]`, the main loop only `taken[]`. `hb_take()` returns `posted - taken` (16-bit, wraps) |
| Watchdog | Restarted only in the heartbeat interrupt. If the interrupt stops, the chip resets after `watchdog_ms`. A main-loop hang does not reset the chip: the hooks keep running |

## Health counters

| Counter | Counts |
| --- | --- |
| `gap_overruns` | Two ticks more than 1.5 ms apart (a tick was late or lost). The first tick does not count |
| `isr_overruns` | The interrupt work of one tick took 1 ms or more |
| `isr_max_us` | Longest interrupt work seen |
| `missed_frames` | `hb_take()` found more than one post: the main loop was late for that flag. Counts each extra post |

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_heartbeat.c src/heartbeat.c -o build/test_heartbeat
./build/test_heartbeat
```

The test covers registration limits, hook and flag period and phase,
take-and-clear, missed frames, the 16-bit counter wrap, the gap and
interrupt-time boundaries (1500/1501 µs, 999/1000 µs), counter reset in
`hb_init()`, and the hook-before-flag order.

Mutation check (2026-11-04): 24 of 25 mutants caught. The survivor
(`flag >= HB_MAX_FLAGS` in place of `flag >= flag_count` in `hb_take()`)
is equivalent: unused flags are never posted, so they return 0.

ARM size (`-Os`, Cortex-M0+): 376 bytes of code for the core.

## Bench test

```sh
cmake --build build --target heartbeat_test
picotool load -f -x build/heartbeat_test.uf2
```

USB serial prints one line per second: tick count, 10 ms frames, the
gap range of the 1 ms and 10 ms hooks, `isr_max_us`, the overrun
counters, and `missed_frames`.

| Action | Expected |
| --- | --- |
| No action | Ticks +1000 per second; gaps near 1000 and 10000 µs; overruns 0; missed 0 |
| GP20: hang the main loop 3 s | No reset; ticks continue; missed goes up by about 299 (10 ms flag) + 149 (20 ms flag) + 2 (1 s flag) |
| GP21: stop the heartbeat | Watchdog reset after 50 ms; the next boot prints "Last reset: watchdog" |

### Bench result (2026-11-04, `2b64bf2`, USB power)

| Check | Result |
| --- | --- |
| Idle, 45 s | Ticks +1000 per second; 1 ms hook gap 1000..1000 µs, 10 ms hook gap 10000..10000 µs; interrupt work at most 30 µs (3% of a tick); overruns 0; missed 0 |
| GP20: main loop hung 3 s | No reset. Ticks went on (34000 → 37542 at the first line after the hang). Missed frames 450 = 299 (10 ms) + 149 (20 ms) + 2 (1 s), as expected. Tick gaps stayed 1000 µs during the hang |
| GP21: heartbeat stopped | Reset in less than 2 s of log time; the next boot printed "Last reset: watchdog" |
| After the watchdog reset | First second: gaps 998..1000 µs (USB start-up); then 1000..1000 µs |

The 30 µs interrupt time includes only the two bench hooks and the
watchdog restart. Measure it again when the ADC sample and `mc_tick()`
are hooked in.
