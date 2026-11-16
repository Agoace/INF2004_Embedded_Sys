# Live barcode counts snapshot

## Purpose and ownership

Vehicle can read S3 terminal outcome counts without USB output, a LINE lock,
a reset call or a wait. This is a read-only application API, not a new MQTT
field or a change to either live record/control wire format. Vehicle owns
any later summary integration.

[`barcode_live_source.h`](../../app_program/subsystem3/barcode_live_source.h)
defines:

```c
typedef struct {
    uint32_t epoch, frames, attempts, failures;
} barcode_live_counts_t;
int barcode_live_counts_try_read(barcode_live_counts_t *out);
```

The LINE task is the sole writer. Any core can attempt one coherent copy.
No reader consumes, clears or resets a counter. The existing producer and
GP20 lifetime counters are unchanged.

## Meaning of each value

| Field | Meaning |
| --- | --- |
| `epoch` | Most recent coherent vehicle control epoch observed by LINE |
| `frames` | Decoded records successfully published after the final source recheck, including letters mapped to NONE |
| `failures` | Terminal failures of bound candidates, including context/source faults and cancellation |
| `attempts` | Terminal outcomes only: `frames + failures` modulo2^32 |

An unfinished candidate is not yet an attempt in this snapshot. Merely
seeing a dark edge without a bound candidate does not increment failures.
A decoded record refused by the final source/control recheck increments
failures, not frames. The existing SHADOW valid0 restriction does not turn
a correctly published decoded record into a failure.

These counters are totals for the named epoch, not lifetime totals from
which Vehicle must subtract a baseline. Lifetime producer publications,
failures and GP20 frame history remain separate diagnostics.

## Epoch boundary and freshness

1. On the first coherent new control epoch, LINE starts zero epoch totals.
2. It processes the sample and attributes a terminal event only when
   `event.record.epoch` equals the current counter epoch.
3. A delayed cancellation from the previous epoch stays in lifetime evidence
   but cannot increment the new epoch's totals.
4. LINE publishes the board on EVERY epoch or matching terminal outcome
   change. It does not wait for the100-sample CPU-summary cadence.

Before LINE observes a new epoch, a reader can still see the old epoch.
Vehicle must accept only its own current run epoch. A failed or wrong-epoch
copy must never replace a matching cached snapshot. The getter is one
bounded attempt, not a retry loop.

A matching snapshot describes terminal work already observed by LINE. It is
NOT an acknowledgement that future stop/cancellation work has drained. If
Vehicle samples before it disables the source, a later disabled-context
cancellation cannot appear in that earlier sample. Do not claim an exact
post-drain final count without a separate acknowledgement protocol.

## Return values and bounds

| Result | Meaning |
| --- | --- |
| `E_OK` | Coherent16-byte copy |
| `E_PAR` | Null output pointer |
| `E_OBJ` | Linked reader, before source initialization |
| `E_TMOUT` | Linked reader, no counter publication yet or interrupted copy |
| `E_NOSPT` | Reader absent: TEST, BENCH or the source-only host stub |

All failure results leave the caller's bytes unchanged. `E_NOSPT` is an
explicit capability result, including before initialization in an absent
reader build. A linked reader that has not observed a control epoch yet
returns `E_TMOUT`, not an invented successful empty run.

The API uses the existing aligned atomic-word board protocol. It copies four
words once and commits a local scratch value only after the sequence check.
There is no allocation, mutex, wait, decoder call, output or board mutation
on the read path. The16-byte writer value and20-byte board add36 bytes of
source storage; the existing4096-byte live-storage assertion includes them.
This type/board accounting is not a runtime-stack claim. Matched ARM
UP USB and UP Wi-Fi builds against `95c885d` measure +240 bytes text and
+40 bytes BSS (the36 bytes plus linker alignment). TEST and BENCH sizes
match. These source-only images do not yet call the getter from Vehicle;
getter retention and the vehicle cache need a separate integration budget.

## Regression evidence

GCC and Clang ASan/UBSan each pass541/541 host tests. Documentation and the
eight-template-deviation checks pass. No flash or hardware test was done.

The new tests first failed to compile on the missing type/API (RED log under
`build/live-counts/red.log`), before the implementation. Targeted tests cover:

- Null, pre-init and unpublished reads with output preservation.
- Correct decoded A and F/NONE counts at the EXACT record publication sample.
- Final control recheck failure counted as a failure, not a frame.
- Same-epoch bound-candidate cancellation counted immediately.
- New-epoch zero totals despite a delayed old-epoch cancellation; old lifetime
  diagnostic failure remains visible.
- Repeated getter calls and idempotent source initialization do not reset data.
- An interrupted four-word read returns once, preserves caller bytes, then
  permits a clean read of the new epoch. The test uses a real LINE-writer
  epoch transition, not a production counter setter.
- TEST and BENCH explicitly report absent reader without changing output.

Entry points:
[`test_barcode_live_feed.c`](../../tests/test_barcode_live_feed.c),
[`test_barcode_live_source.c`](../../tests/test_barcode_live_source.c), and
[`test_barcode_live_counts_absent.c`](../../tests/test_barcode_live_counts_absent.c).
The read-interruption hook is host-test-only. No counter setter is exposed
in production. No hardware or dashboard behavior is proved by these tests.
