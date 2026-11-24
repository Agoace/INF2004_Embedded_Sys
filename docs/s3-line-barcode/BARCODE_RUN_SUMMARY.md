# Barcode counters in the run summary

## Meaning

The vehicle task adds barcode reader counts to the MQTT run summary in
both fallback SHADOW builds and the LIVE demo build:

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

## Measured software results

Candidate code `c10d215`, compared with staging `569672a`:

- GCC and Clang ASan/UBSan: 550/550 tests each, including nine new vehicle
  cases. The initial summary test failed on the missing implemented flag.
- Unmodified `build38.sh`: 38 profiles, zero warnings, for each revision.
- 76-file comparison: 74 differ, two match. All 38 ELF files differ.
  UF2 files match only for `up-bench` and `up-mqtt-test`.
- The complete changed-profile list is every combination of placement
  `up`, `split`, `control` with profile `usb`, `test`, `bench`, `wifi`,
  `lwip`, `mqtt`, `mqtt-test`, `mqtt-bench`, `mqtt-jA`, `mqtt-jB`,
  `mqtt-jC`, `mqtt-jD`, plus `capture-f0` and `capture-f1`, with the two
  matching UF2 exceptions above. This states the ELF and UF2 lists exactly.
- Normal reader images change because the merged S3 reader/API and vehicle
  summary behavior change. TEST/BENCH/capture image sizes do not change;
  their byte differences include linker veneer placement and branch
  encodings, not reader execution. The S3
  [API record](LIVE_COUNTS_SNAPSHOT.md) contains the normalized capture-f1
  function comparison. Debug-data changes also affect ELF hashes.

Matched UP MQTT resource comparison (`arm-none-eabi-size`, bytes):

| Revision / change | Text | BSS | Text delta | BSS delta |
| --- | ---: | ---: | ---: | ---: |
| Staging `569672a` | 436372 | 200032 | 0 | 0 |
| Reader/API merge `4beceb2` | 436660 | 200072 | +288 | +40 |
| Vehicle summary `c10d215` | 436980 | 200088 | +320 | +16 |
| Combined versus staging | — | — | +608 | +56 |

Each revision row's delta is relative to the preceding revision. Initialized
RAM data is zero in all three builds. Other ordinary reader-linked profiles
add 40 B BSS without MQTT summaries and 56 B with summaries; all remaining
profile BSS totals are unchanged. Text deltas vary with linking.

The demo UP MQTT `KERNEL_RAM=1` image also builds without warnings:
text 437928 B, BSS 200088 B; `RAM_PROOF PASS`, all 75 listed symbols in RAM,
18424 B listed code. This does not establish run-time stack margin or timing.

Local evidence is under `build/wip/barcode-summary/` in the vehicle worktree:
red/green host logs, sanitizer logs, matrix hashes/full diff list, resource
parent build, and demo RAM-proof log. The final API documentation merge and
this result record do not change the candidate firmware sources.
