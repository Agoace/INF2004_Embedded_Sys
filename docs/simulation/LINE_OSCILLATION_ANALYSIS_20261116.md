# Retained LINE oscillation analysis — 2026-11-16

## Result

**The first observed SIL/robot difference is already in the raw line-sensor
pair, before position and P7 steering. This is not proof of an optical cause.**
The raw pair is inside the feedback loop: motion, surface, sensor geometry,
optics and adaptive normalization can all change it. Independent real pose and
100 Hz command/count history are absent. Do not tune an optical or motor model
from this result alone.

The home straight prefix has a 380 ms period, raw C-L amplitude 2108 ADC,
position amplitude 59, and encoder-derived yaw-rate amplitude 50.1 deg/s.
Aligned SIL has near-constant position and commands. The unchanged native P7
can produce every tested real command when the missing previous 10 ms position
is allowed to vary. Native motor-control replay through the existing DM1/80 ms
surrogate produces large wheel oscillations when given the real commands.
Thus a different P7 formula is not needed to explain the difference. The motor
surrogate is not fully validated: it overpredicts home yaw amplitude by about
12–14 deg/s in the tested prefix. Its 20 ms lag difference from the recording
is below this analysis's conservative 40 ms phase uncertainty.

No Webots, hardware, serial, flash, installation or display activity was used.
No firmware, optics, plant or control parameter was changed. Hardware Gate1
remains **TRIAL INCOMPLETE / NOT GATE1 PASS**; physical GP28 remains **FAIL**.

## Inputs and provenance

The reproducible output is
`build/webots/line-osc-analysis-20261116T142730Z/analysis.json`, SHA256:

```text
bd531d80dccb5df05b4fca188b9b5ff181b0db068eb992addccbab714918b15e
```

This ignored file contains input file hashes, packed-record hashes, source
locations, run boundaries, every analysis window, replay results and native
source/library hashes. Historical evidence is not modified.

Input roots:

- Recent images: `/home/yangxuanlee/Projects/mtk3-core-review/build/`.
- School and older captures:
  `/home/yangxuanlee/Projects/School-Repos/personal/INF2004_Embedded_Sys/build/`.
- Existing SIL: this worktree's `build/webots/`.

The recent GP20 final logs use the **last connect session**. The connect text
has a time, for example `=== 20:51:03 connect`. The four old cumulative USB
files contain byte-identical shared captures. They yield eight unique KR
blocks, not four independently repeated experiments. A packed index/payload
hash prevents counting the same dump twice. MQTT `robot/log` contains packed
high-rate KR rows; it is not the same as the slow `robot/state` topic. Those
rows are matched to dumps by index and payload. Unmatched boot/generation
groups remain separate. Fewer than 100 unmatched rows are context only.
Conflicting payloads at the same boot/generation/index cause an error.

The school boot/image ledger is `build/school_20261113/SESSION_NOTES.md` in the
school input root. Split-image boots 3250257766, 1442838703 and 941374591 are
excluded, including their MQTT rows. The P1 force0 stop-path DEADLINE is not
silently reclassified as the excluded split-image failure.

Recent source snapshots and retained UF2 build files were checked read-only by
SW7. P7, motor control, position, trace and vehicle-task sources are identical
across 451e562, d8adbcc, e06667d, 2413c4a, b0c3a9a and 7a6cb1c. Changes to the
vehicle config between these images are detour-only. The analysis independently
checks the P7/motor/position/KR source bytes and LINE_CFG against Git history
before image-specific native replay. Later school detour images have the same
checked LINE code/config. `381b7dc-v350` means that source with operator v350
detour settings; it is not a claim of whole-image binary identity.

RAM2 boot `e09fc7fc` has no image identifier, flash record or useful GP20 dump.
Its directory/timeline suggests RAM, but does not prove it. It stays **image
unresolved**, with no image-specific replay claim. Older cumulative captures
and the two before-P1 captures also have unresolved image identity. Recent
KRAM directory names do not establish location; only the named home run and
the documented school sessions have location labels here.

## Method and limits

- KR1 is 16 packed little-endian bytes. Its time is elapsed-ms divided by 20,
  floored. Row index is not time. Module 5 is SCAN reverse, not LINE.
- The vehicle runs at 100 Hz, but KR rows are nominally 50 Hz, with some 40 ms
  intervals. KR measured speeds are the latest encoder-derived **10 ms**
  snapshots, quantized by about 8.06 mm/s. KR has no raw wheel counts, applied
  duty or per-row command-publication age.
- Speed-derived yaw is `(vR-vL)/111 * 180/pi`. This is wheel odometry, **not
  an independent gyro or observed body heading**. All amplitudes here are
  half of the 95th-minus-5th percentile range, not a peak maximum.
- The GP20 printed floor/line endpoints are adapted values at **dump time**,
  not spin-calibration endpoints or run-start values. TELEMETRY_DUMP replaces
  the earlier info record. Adaptation runs near 1 kHz, including idle, with
  approximately 256-sample time constant. Full raw-to-position replay is not
  identifiable from 50 Hz raw samples and unknown adaptive initial state.
  Raw C-L is therefore the primary balance metric. Dump-endpoint normalized
  balance, single-sensor fraction and high-both mask are diagnostics only.
- Candidate sections require RUNNING and LINE. SEARCH, SCAN, DETOUR, SCAN
  reverse, stopped/reversing-pivot rows and packed clamps are excluded. Gaps
  over 40 ms split sections. **One zero wheel is retained:** P7 legitimately
  commands it during strong LINE correction; removing it removes zigzag peaks.
- Fixed 2 s windows start 500 ms after each section, at 1 s stride. Windows
  overlap; their count is not an independent sample count. Period uses local
  autocorrelation maxima from 120 ms to at most 660 ms, at least three cycles,
  correlation at least 0.3, and position amplitude at least 5. All windows,
  including quiet ones, are retained in the output. A dash means no qualifying
  cycle/window, not proof of smooth motion. A longer period can be a harmonic.
- School LINE state does not identify a straight tile. School numbers below
  are **unlocated LINE candidates**, not certified straight-track results.
  No segment is chosen because its yaw is small. School/home/unknown images
  and separate runs are not pooled.
- Positive phase delay means the second signal follows the first. Correlation
  is searched within half a detected period, capped at 200 ms. Delays have at
  least ±40 ms uncertainty from timestamp/measurement age and omitted 10 ms
  states. Periodic ambiguity remains. Slow MQTT state (~4–5 Hz) supplies
  context only, not phase evidence for these cycles.

The home run's raw pair becomes high on both channels near 3 s; position then
holds at 19. It must not be described as a full 6.88 s straight oscillation.
The fixed **500–2500 ms** prefix is the primary home comparison. The wider
window inventory includes the transition and explicitly reports quiet/held
windows rather than discarding them for a better result.

## Per-run results

`N` is detected-cycle windows / all eligible windows. `B` is raw C-L amplitude
in ADC counts. `Y` is wheel-odometry yaw-rate amplitude in deg/s. `P` and delays
are ms. `S→U` is raw pair balance to requested wheel difference; `U→V` is
requested to measured wheel difference. Values are medians over detected
windows, except the explicitly marked home primary prefix. Each row with a
cycle differs from aligned SIL already at the **raw pair** stage. A dash row
has no defensible cycle-stage comparison. Unknown images get no native claim.

### Recent final GP20 dumps

Paths are `<directory>/robot_<name>.final.log` in the recent input root.

| Directory / name | N | P | Y | B | S→U | U→V |
|---|---:|---:|---:|---:|---:|---:|
| kram_robot_e06667d / BASE | 21/26 | 260 | 45.9 | 2045 | 0 | 40 |
| kram_robot_e06667d / RAM | 16/19 | 290 | 52.1 | 2113 | 0 | 40 |
| kram_robot_2413c4a / BASE1 | 12/13 | 280 | 49.9 | 2173 | 0 | 40 |
| kram_robot_2413c4a / RAM1 | 26/30 | 280 | 48.0 | 2111 | 0 | 40 |
| kram_robot_b0c3a9a / BASE | 22/25 | 260 | 48.9 | 2065 | 0 | 40 |
| kram_robot_b0c3a9a / RAM | 23/26 | 280 | 48.0 | 2015 | 0 | 40 |
| home_size_7a6cb1c / S210_1, whole inventory | 2/5 | 370 | 42.7 | 2083 | 10 | 40 |
| Same home, primary 500–2500 ms | 1 window | 380 | 50.1 | 2108 | 20 | 40 |

The primary home position amplitude is 59. Its L and C raw amplitudes are
1069 and 1066 ADC; raw C-L mean is +113 ADC. Requested L-R amplitude is
168 mm/s. Dump-endpoint normalization gives amplitude 0.710 and single-sensor
fraction 88.1%, but these are **not recovered run calibration**. This larger
robust yaw value does not contradict earlier selected-window estimates of
40–46 deg/s: the window and amplitude statistic differ.

### Older cumulative captures, identity/location unresolved

`old08` through `old15` are the eight unique blocks in order of first
appearance. old08–11 appear in s5_b2a_j5, sys_s1a, sys_s1b and b2b_shadow_F;
old12–13 first appear in sys_s1a; old14 in sys_s1b; old15 only in the new
b2b_shadow_F tail. Filename date does not establish embedded-run identity.

| Block | N | P | Y | B | S→U | U→V |
|---|---:|---:|---:|---:|---:|---:|
| old08 | 45/46 | 320 | 56.3 | 2241 | 0 | 40 |
| old09 | 39/41 | 320 | 52.1 | 2183 | 0 | 40 |
| old10 | 0/0 | — | — | — | — | — |
| old11 | 0/0 | — | — | — | — | — |
| old12 | 1/1 | 620 | 56.3 | 2109 | 0 | 40 |
| old13 | 0/0 | — | — | — | — | — |
| old14 | 0/0 | — | — | — | — | — |
| old15 | 30/32 | 300 | 50.1 | 2186 | 0 | 40 |

### School GP20 dumps

Paths are `school_20261113/<name>_gp20_usb.txt`, except before-P1.

| Block / source / image | N | P | Y | B | S→U | U→V |
|---|---:|---:|---:|---:|---:|---:|
| old16 / t3_force0 / 451e562 | 6/6 | 300 | 48.9 | 2158 | 0 | 40 |
| old17 / t3b_force0 / 451e562 | 0/0 | — | — | — | — | — |
| old18 / p1_force0 / d8adbcc | 54/55 | 300 | 45.7 | 2060 | 0 | 40 |
| old19 / p1_smp0 / d8adbcc | 26/30 | 280 | 42.8 | 1976 | 0 | 40 |
| old20 / ctrl_smp0 / 451e562 | 20/20 | 340 | 58.3 | 2202 | 20 | 40 |
| old21 / usb_console_before_p1, block1 / unknown | 22/23 | 280 | 42.8 | 2083 | 0 | 40 |
| old22 / usb_console_before_p1, block2 / unknown | 21/22 | 280 | 45.9 | 2067 | 0 | 40 |

### Additional packed MQTT runs

Generation is a separate log stream/run boundary, not a repeat inferred from
filename. The 112618 CSV is `robot-mqtt-20261113-112618.csv`; all other school
rows below are from `robot-mqtt-20261113-155932.csv`. Matched GP20 rows are not
counted again here. RAM2 is from `kram_robot_e06667d/mqtt_RAM2.csv`.

| Boot / generation / image | N | P | Y | B | S→U | U→V |
|---|---:|---:|---:|---:|---:|---:|
| e09fc7fc / 2 / unresolved RAM2 | 10/11 | 310 | 51.1 | 2180 | 0 | 40 |
| 05aeb54c / 2 / 451e562, 112618 | 9/9 | 320 | 58.3 | 2128 | 0 | 40 |
| 7e181f3f / 2 / 451e562 | 1/1 | 180 | 43.9 | 2005 | -20 | 40 |
| 7e181f3f / 3 / 451e562 | 0/0 | — | — | — | — | — |
| 738300d7 / 2 / f65aaf8 | 10/10 | 310 | 57.3 | 2152 | 0 | 40 |
| 738300d7 / 3 / f65aaf8 | 0/0 | — | — | — | — | — |
| 738300d7 / 4 / f65aaf8 | 0/0 | — | — | — | — | — |
| 738300d7 / 6 / f65aaf8 | 17/18 | 280 | 43.9 | 2111 | 0 | 40 |
| 738300d7 / 7 / f65aaf8 | 0/0 | — | — | — | — | — |
| 7ff0007f / 2 / 39321d9 | 3/3 | 440 | 45.7 | 1973 | 0 | 40 |
| af1fc00f / 2 / 381b7dc | 22/22 | 290 | 48.0 | 2130 | 0 | 40 |
| 92564d9f / 2 / 381b7dc-v350 | 12/13 | 300 | 55.2 | 2035 | 0 | 40 |
| 92564d9f / 3 / 381b7dc-v350 | 20/23 | 290 | 48.0 | 2107 | 0 | 40 |

### Existing SIL recordings

Same 20 ms analysis grid and 10 ms encoder-count speed reconstruction; no new
physics. Amplitudes below use **all** windows, so quiet runs do not disappear.

| Existing recording | Mass kg | N | P | Y | Body yaw-rate amplitude | B | Position amplitude | Requested L-R amplitude |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A prefix, along1, 20261116T074205Z | 1 | 0/3 | — | 4.65 | 0.266 | 0 | 0 | 0 |
| B aligned, 20261115T130824Z | 1 | 0/8 | — | 4.65 | 0.122 | 24 | 0 | 0 |
| F +5mm/-5deg settling, 20261116T122633Z | 0.6716 | 1/1 | 180 | 4.65 | 1.574 | 268 | 5.5 | 8 |

Y remains about 4.65 deg/s even when body heading barely changes, because
integer encoder counts create unequal wheel-speed snapshots. Do not interpret
that as real 4.65 deg/s body zigzag. F is a small settling oscillation, not the
real home oscillation and not a 90-degree turn exit. A is still its original
**INCOMPLETE** prefix. The 1 kg and 0.6716 kg simulations remain separate.

## Native replay and first differing stage

The host-only `line_metrics_adapter.c` uses the existing motor adapter and
unchanged C P7/motor code. Before replay, source and LINE_CFG equality are
checked against each attributed image. No steer formula is reimplemented in
Python. The native steering test enumerates all possible previous positions
-100..100 for each recorded current position; the missing 10 ms derivative
state is not invented. All 19 image-attributed tested windows are 100%
compatible with this envelope. Naively using the preceding 20 ms KR position
instead produces mean wheel-command errors of roughly 1.6–4.0 mm/s. This is
**compatibility under bounded missing state**, not exact raw/RTOS replay.

For the first qualifying window per attributed run, recorded requests drive
native motor control at 10 ms through the existing DM1/80 ms lag surrogate at
1 ms. Missing requests are reconstructed two ways: held and linear. Each
replay has a 1 s warm-up; initial real motor state, battery, contact load and
intermediate counts are unknown. This is not a complete Webots physics replay.

Representative requested-command replay (yaw-rate amplitude, deg/s):

| Input window | Recorded | Predicted held / linear | Recorded / predicted delay ms |
|---|---:|---:|---:|
| e06667d BASE, 500–2500 | 45.9 | 50.1 / 52.1 | 40 / 60 |
| e06667d RAM, 500–2500 | 45.9 | 45.9 / 48.0 | 40 / 60 |
| 2413c4a BASE1, 500–2500 | 48.0 | 45.9 / 45.9 | 40 / 60 |
| 2413c4a RAM1, 500–2500 | 41.8 | 45.9 / 45.9 | 40 / 40–60 |
| Home primary, 500–2500 | 50.1 | 62.5 / 64.5 | 40 / 60 |
| School t3, 500–2500 | 45.9 | 48.0 / 50.1 | 40 / 60 |
| School P1 force0, 500–2500 | 41.8 | 45.9 / 41.8 | 40 / 60 |

The other individual results remain in the sealed JSON; they are not fitted
together. No conclusion that RAM placement causes or removes oscillation is
supported: route and surface are not paired, and control sources are equal.

The observed stage chain is:

```text
raw pair already differs
  -> adaptive normalized position (initial state not recovered)
  -> P7 command (native envelope compatible)
  -> motor/count response (large response reproduced; home residual remains)
  -> real pose/contact (not observed independently)
```

The existing aligned SIL does not supply the large alternating raw/position
input that drives real P7. But the loop does not establish whether that input
originates in optics/height, contact/motor response, line geometry or adaptive
normalization. A fitted motor time constant, a new optical profile or a new
controller gain is **not justified** by these data alone.

## Reproduce and validation

From this worktree, use a fresh output directory:

```sh
python3 sim/webots/tools/sil_line_oscillation.py \
  --robot-build /home/yangxuanlee/Projects/mtk3-core-review/build \
  --main-build /home/yangxuanlee/Projects/School-Repos/personal/INF2004_Embedded_Sys/build \
  --sil-build build/webots --output build/webots/line-osc-analysis-new
PYTHONPATH=sim/webots python3 -m unittest discover \
  -s sim/webots/tests -p test_sil_line_oscillation.py -v
```

Validation: 8 new analysis tests pass; full Python suite 360 tests passes with
3 expected failures. Fresh host build and 614 C tests pass. Frozen no-device
replay matches 158 RX / 58 TX; verdict remains INCOMPLETE, not hardware PASS.
One initial frozen invocation from Python stdin failed multiprocessing spawn;
the supported `gate1_dry_run.py` CLI then passed. No physical retry occurred.
