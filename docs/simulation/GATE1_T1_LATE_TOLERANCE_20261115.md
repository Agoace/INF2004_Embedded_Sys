# T1 rare late-capture trial — operator decision, 2026-11-15

**TRIAL / NOT GATE1 PASS.** Operator approved this separate opt-in experiment,
after C4. It is not standard Gate1 qualification or a change to physical ages.
Defaults remain phase250us, clock250us and adaptive SYNC100ms. B400/SYNC20
alone still requires phase250us. No hardware action is performed by this code.

## Approved quota revision after attempt19

On2026-11-15 the operator raised the rolling quota from1 to3, after attempt19
stopped on late slots1011(269.5us) and1132(287us),121 slots apart, with no Pico
reset reported. Only the quota changes. Historical quota1 artifacts retain their
original policy/verdict; do not relabel them as quota3 evidence.

## Selection and rules

Add `--trial-tolerate-late` ONLY with `--trial-clock-400-sync20` and
`--transport-only-cap4`. Real device access still separately requires
`--hardware-authorized`. The flag is false by default, including API callers.

Every capture retains its original source slot/target/epoch and actual RAW
start/end. Both endpoints require the SAME qualified clock cones, aged half
<=400us, and the START midpoint must not precede the target. Phase includes the
end midpoint's existing outward+1us once. No early estimated aim or learned bias.

- Phase<=250us: ordinary capture, no tolerance consumed.
- 250<phase<=600us: permitted ONLY if the previous slot was not late AND there
  are fewer than3 tolerated late captures in the previous999 slots.
- Phase>600us, consecutive late, or a fourth late in ANY rolling1000 slots: FAIL.

Late at0,2,4 is permitted; another at999 fails, while one at1000 is permitted
because slot0 has left the rolling window. Late at998,1000,1002 is permitted,
but another at1004 fails even across a fixed-bucket boundary. This is not
fixed1000-slot buckets, a time window, a percentile allowance or skip/catch-up.
Missing/replayed slots remain fatal; sequence continuity is unchanged.

## P_MAX=600us: exact rationale and SW6 confirmation

Let A be intended apply and T=A-4000 nominal source target. For actual source
interval[c_lo,c_hi], H=max(T-c_lo), L=max(c_hi-T). Qualified B400 at actual
start/end plus no-early MIDPOINT gives H<=400 independent of positive late phase.
End phase<=600 (including outward+1 already) gives L<=400+600=1000.

With unchanged native J250, worst apply-age UPPER<=4000+250+400=4650us,
**350us margin** to5000. Apply-age LOWER>=4000-1000=3000us. P_MAX600 is NOT the
unique maximum derived from expiry: it is the additional conservative choice
1000us slot minus B400, keeping latest source interval endpoint<=T+1000=A-3000.
That leaves2ms to the BARE native staging deadline A-1000, not a transport-delay
guarantee. SW6 independently confirmed these numeric statements and conditions.

Exact D1 age checks remain unchanged and fatal:

- Availability upper qMax+H<=3000; lower qMin-L>=0, with
  qMax=max(a_hi+1-T), qMin=min(a_lo-T).
- Apply upper4000+jMax+H<=5000; lower min(p_lo-T)-L>=0.
- Sufficient worst-envelope availability requires qMax<=2600 AND qMin>=1000.

A normal fast stage can have qMin<1000, making the global conservative lower-age
check FAIL after a late capture. T1 DOES NOT waive that check or guarantee every
permitted phase qualifies. Actual margins use measured extrema and source
intervals, not excursion count. Rarity cannot amortize ONE age/deadline failure.
Cache expiry remains min(A+1000,c_lo+5000), worst-envelope A+600, NOT an automatic
extra1ms of physical cache life. Cap4 remains a synthetic source, not physics.

All native3ms availability/4ms pipeline/5ms expiry, progress/output leases,
current+next readiness, queue bounds, jitter/order/CRC/session/clock faults and
D1 upper AND lower ages stay fatal. No Pico image, deadline or timer change.

## Evidence and verdicts

Each tolerated capture emits `source_capture_tolerated` with full source/target,
RAW start/end, both endpoint clock intervals/midpoint errors, phase/uncertainty,
quantization, last SYNC, sequence, excess over250,600us cap and trial policy.
The ordinary source_capture/source_clock_bounds records are still emitted.
After inhibit/DISARM/serial close, `tolerated-late-captures.json` retains ALL
full records, count, RAW clock-domain identification and export-close status.
Storage is bounded by the1800s source horizon and at most3 events per1000 slots.
The policy checker keeps only the latest3 late slots; the total count and full
capture records are retained independently.

Run metadata, summary, trial manifest, attribution metadata and host observations
identify T1 explicitly. Summary has `tolerated_late_count`, counts.tolerated_late
and the exact policy. A600s completed run with any tolerated capture is labelled
`TRIAL_COMPLETE_WITH_TOLERATED_LATE`, with `completion_label` including n. CLI
still exits2. Zero-event completion remains TRIAL_COMPLETE but retains T1 mode.
Physical Gate1 PASS and standard transport_subset_qualified remain false.

Offline source proof audits the rolling rule and full CSV/sidecar agreement;
missing records/proof are incomplete, recorded rule breaches fail. D1/measurement
reports retain mode/count labels. Full physical Gate1 acceptance is not issued.

Red-first boundary tests include250/600 endpoints, fractional excess, no-early,
clock expiry at capture END, consecutive/rolling windows, API gating, retained
records/counters, distinct completion, unchanged lookahead/output-lease faults
and a negative D1 lower availability age that remains FAIL.
