# LINE_P_NEAR simulation prediction — 2026-11-16

## Result

**At20ms added ADC transport, P60 and P40 decay to near-zero body yaw. At30ms,
P85 and P60 sustain approximately stationary oscillations over4–10s; P40 has
two SEARCH recoveries before a small residual ripple. None reproduces the real
46–52deg/s wheel-derived yaw amplitude.** P85/30ms has360ms period and P60/30ms
has380ms period, within the real260–380ms period range. These are finite-run
simulation predictions, not hardware tuning acceptance or mathematical proof
of an asymptotic limit cycle.

The result is not monotonic in added transport delay. Do not infer that delay
is the unique real cause, that a lower gain is safe on curves, or that a good
straight result proves SEARCH/junction/barcode/course acceptance.

## Source isolation and unchanged conditions

Main authorized exactly five new runs after the three original excited-P7
runs were sealed at `3ab4baa`. See
[original excitation experiment](SIL_EXCITED_P7_20261116.md) and
[operator geometry](ROBOT_GEOMETRY_20261116.md).

The candidate firmware sources come from SW9's exact code commit:

```text
89b7935a91de915582d264e094339d9ccb9ec1d0
```

`7810d07` adds documentation only. **Neither commit is merged into this
branch.** The host builder extracts `app_program/` read-only with `git archive`
into each new generated project's `libraries/sw9-source/`. Simulator-owned
native ABI adapters are copied from this branch. All compiled firmware C and
headers, including `app_program/vehicle/line_p_near.h`, come from SW9's commit.
Source hashes, compiler commands, library hash and native configuration are
sealed in each `libraries/sil-build.json` and launch manifest.

The real SW9 adapter copies LINE_CFG and changes only `.steer.gain_pct`.
For60/40, `-DLINE_P_NEAR=<gain> -include <line_p_near.h>` applies **only** to
`search_adapter.c`, which owns LINE_CFG initialization and includes the motor
ABI adapter in that translation unit. Other native sources are unchanged.
P85 gets no adapter/flags, matching the default-image policy. No steering
formula is rewritten in Python and no firmware file in this worktree is edited.

The native metadata is checked against `[146,P,85,62,378,50,340]` in every
build. D85, clipped per-update delta, min/max speeds, base speed, normalization,
SEARCH, motor control and the continuous extra340 gain beyond|position|50
remain unchanged. Base P is reduced at **all positions**, not switched off
outside a near region. These remain ASSUMED trial settings, not accepted tunes.

All cases start+10mm/−10deg, run10000ms, use the same measured geometry,
9/8 fixed angular footprint assumption, historical8mm B spin rows, line-only
fast rectangle sampler and DM1/80ms fitted whole-loop response as the sealed
experiment. No new optical fit, friction, mass, motor time constant or plant
parameter is selected from an outcome. No extra40ms motor transport is added;
the existing loop already gives roughly40–60ms command-to-wheel phase.

The input queue adds exactly20or30ms transport. Every fresh/delivered ADC pair
is audited against its predecessor at that exact age (first sample held during
startup). The maximum queue size is31 pairs. Correlation phase at20ms-grid
resolution need not equal the exact transport delay: the30ms cases often give
20ms fresh-sensor→command correlation lag, with at least±40ms uncertainty.

## Runs and post2s metrics

Five launches, sequential, each with a1200s wall cap, no retry:

| Case | Wall s | Owned PGID, now gone | SEARCH count |
|---|---:|---:|---:|
| P60 /20ms | 2.7216 | 1126146 | 0 |
| P40 /20ms | 2.8207 | 1126257 | 0 |
| P85 /30ms | 2.8214 | 1126367 | 0 |
| P60 /30ms | 2.8722 | 1126477 | 0 |
| P40 /30ms | 2.7212 | 1126589 | 2 |

All five finish10000rows, native recorded-input replay matches, exit0,
LINE/RUNNING/fault0. Native source replay is not repeated physics. All four
post2s windows are retained. Amplitude is half the95th-minus5th percentile
range. Body yaw rate is from Webots body-heading change. Wheel yaw is the
KR-like10ms integer-count speed snapshot sampled on a20ms grid, using
12414counts/m and111.5mm track. Raw C-L is delivered ADC.

| P / added delay | Window s | LINE period ms | Body yaw rate deg/s | Wheel yaw deg/s | Raw C-L ADC |
|---|---|---:|---:|---:|---:|
| 60 /20ms | 2–4 | — | 0.333 | 4.625 | 36.5 |
| same | 4–6 | — | 0.123 | 4.625 | 10.5 |
| same | 6–8 | — | 0.121 | 4.625 | 11.5 |
| same | 8–10 | — | 0.146 | 4.625 | 23 |
| 40 /20ms | 2–4 | — | 0.280 | 4.625 | 33.5 |
| same | 4–6 | — | 0.138 | 4.625 | 8 |
| same | 6–8 | — | 0.138 | 4.625 | 9 |
| same | 8–10 | — | 0.193 | 4.625 | 20.5 |
| 85 /30ms | 2–4 | 360 | 36.835 | 43.678 | 2203 |
| same | 4–6 | 360 | 36.045 | 41.623 | 2171.5 |
| same | 6–8 | 360 | 36.000 | 41.623 | 2171 |
| same | 8–10 | 360 | 35.911 | 41.623 | 2174 |
| 60 /30ms | 2–4 | 380 | 14.570 | 16.957 | 1276 |
| same | 4–6 | 380 | 13.673 | 16.957 | 1202 |
| same | 6–8 | 380 | 13.582 | 16.957 | 1202.5 |
| same | 8–10 | 380 | 13.752 | 16.957 | 1204.5 |
| 40 /30ms | 2–4, MIXED recovery | **not a LINE cycle** | 70.303 | 78.364 | 2173 |
| same | 4–6 | 540, small ripple | 1.035 | 4.625 | 174.5 |
| same | 6–8 | 560, small ripple | 1.036 | 4.625 | 175 |
| same | 8–10 | 560, small ripple | 1.029 | 4.625 | 175.5 |

P40/30ms has90non-LINE rows in2–4s. Its unfiltered aggregate period520ms and
large yaw values include SEARCH; they are retained in the JSON but **must not
be presented as a P7 limit cycle or a match to real straight oscillation**.
It settles into the line band at2947ms. All later windows are LINE, with
position amplitude4 and body-angle amplitude about0.109deg. The small ripple
is not a real-amplitude reproduction. The4.625deg/s wheel metric in otherwise
quiet runs is count quantization, not body oscillation.

P85/30ms has body-angle amplitudes2.078,2.079,2.092deg in4–6,6–8,8–10s;
P60/30ms has0.811,0.811,0.813deg. Their raw and rate amplitudes are also
approximately stationary on these windows. This supports sustained finite
oscillation at these settings. A single10s run per condition does not prove
asymptotic stability, repeatability or a uniquely identified real mechanism.
The earlier P85/20ms result remains a separate, slowly decaying record:
360→300ms, body39.15→33.44deg/s, wheel41.62→39.57deg/s. No baseline is rerun.

## Evidence and validation

Ignored evidence: `build/webots/taskA-candidates-20261116T151447Z/`.
Supplemental audit verifies every sealed launch artifact, exact20/30ms ADC
sequence, measured geometry and absence of all five owned process groups.
Audit SHA256:

```text
5390b68009ea2aecd9113876807e9da43bff23940caf023b591799caef94e97d
```

Sample CSV SHA256, in run-table order:

```text
9df4731cf910947be7f8090760a050e0bfbd278e8cfe6ce06e645aaa45defd1d
860d08530018809268aa8be67006f91d8bf4382518e4361671fee9116fc73c7f
4edae668f5d613727542284540d3f38c1084da1b71d98dd8e7af0f7108ed5bb3
73a82daa21cebe7fc2214d09484eae2bea3c8987d476b1bf2b2fb2cd67ffbb8b
b73d913b7e92972d44373c4e522c28a3c449d132bf8d077e1ff14467b163e74a
```

Offline audit and host tests do not start Webots:

```sh
python3 sim/webots/tools/sil_excited_line_audit.py \
  --root build/webots/taskA-candidates-20261116T151447Z \
  --candidate-comparison --output build/wip/taskA-candidate-audit-check.json
PYTHONPATH=sim/webots python3 -m unittest discover \
  -s sim/webots/tests -p test_sil_line_candidates.py
```

The host tests compile SW9's **original** `test_line_p_near.c` against its
original steering source at85/60/40, then exercise the simulation native ABI
and exact30ms queue. Missing SW9 Git history disables that source test rather
than substituting an unproven image. No firmware merge, hardware access,
serial access, display-server start or physical tuning trial occurred.
Validation:365Python tests pass with3expected failures;614native C tests pass;
frozen no-device replay matches158RX/58TX with verdict INCOMPLETE; documentation
checks pass137files with0broken links/anchors; whitespace checks pass.
Hardware Gate1 remains INCOMPLETE/NOT PASS; physical GP28 remains FAIL.
