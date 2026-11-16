# Barcode counters in the run summary

## Meaning

The vehicle task adds SHADOW reader counts to the existing MQTT run summary:

| Field | Meaning |
| --- | --- |
| `barcode_ok` | Decoded frames successfully published during this run, including letters mapped to NONE |
| `barcode_failed` | Terminal failed bound candidates, including source/context faults and cancellation |
| `RM_SUMMARY_BARCODE_IMPLEMENTED` | A coherent reader-count snapshot for this run was available |

Both wire counters saturate at 255. An unfinished candidate is not a terminal
outcome. These fields do not count junction turns or commands accepted by the
vehicle. A decoded NONE letter is still a successful barcode read.

## Ownership and run boundaries

S3 alone owns the producer and its epoch counters. The public bounded
`barcode_live_counts_try_read()` API copies the reader's count board. The
vehicle task never reads the mutable producer, resets S3 counters, or parses
GP20 text. The original lifetime diagnostic totals remain unchanged.

The vehicle clears its local copy at each run start. During RUN, it keeps
only coherent snapshots whose epoch matches that run. It takes one final
snapshot before `junction_source_stop()` changes the reader context. A busy
read retains the previous same-run copy; it cannot substitute another run's
counts. Thus the summary uses the latest successfully read snapshot, not a
blocking handshake with the reader. A candidate terminated only after STOP
is not added retroactively to an already finished summary.

Without any valid same-run snapshot, both counts remain zero and the flag
stays clear. TEST/BENCH or absent-reader providers cannot set the flag. An
available reader with no terminal attempts reports implemented with 0/0.
These conservative unavailable results must not be treated as proof that no
barcode was encountered.

## Validation and impact

The vehicle tests cover successful/failed reads, linked zero attempts,
absent/busy readers, wrong epochs, saturation, two consecutive runs, and a
busy final read. The real vehicle task supplies the summary; test doubles
supply sensor data and the bounded count getter. S3 tests cover writer-owned
terminal counts and delayed old-context events.

No wire field, message version, steering rule, decode threshold, command
permission or navigation decision changes. Existing dashboards can use the
summary flag and counters without a new protocol. This is an intentional
ordinary-robot behavior change, so ordinary ELF/UF2 byte identity is not an
acceptance condition for this change. Compare against staging `569672a` and
record all changed profiles and resource deltas. Hardware acceptance is not
implied by host tests or builds.
