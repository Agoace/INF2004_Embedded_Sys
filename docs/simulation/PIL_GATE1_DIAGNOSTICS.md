# Gate 1 diagnostics D1

Status: approved host-only implementation contract (Main, SW6 and SW8).
No hardware timing result or permission to open serial/flash is supplied.
The frozen [PIL v1 protocol](PIL_PROTOCOL.md) is unchanged.

## Joint evidence design: no frozen UART change

Do NOT add UART message IDs, payload bytes, caps, fields or diagnostic packets.
SENSOR stays66 wire bytes, OUTPUT58; SYNC uses the existing frozen fields.
No live USB evidence traffic: after DISARM/first fault, freeze an immutable
summary, retain bounded optional trace, and dump over the Pico's own USB CDC.
A 10-minute per-frame trace would be >28MB: impossible in Pico RAM. Instead
EVERY admitted stage/apply/alarm event contributes to counts and extrema. Trace
is first32 + rolling last32/fault-boundary records, NEVER proof of full coverage.
Percentiles of device arrival/apply/jitter remain UNMEASURED, not inferred.

For n, Bn=E+1000*n-4000. Capture TIMER immediately before and after release of
READY: [Alo,Ahi]. Ahi is AFTER full CRC/range/session/deadline validation and slot
copy/publication, not UART arrival or successful PC write. Before unmasking,
use conservative Ahi+1 against the deadline; late publication is revoked/faulted
and must NOT count as an admitted stage or renew input progress.
Similarly bracket atomic logical transport apply with [Plo,Phi]. Retain actual
raw TIMER reads, not ideal slot time masquerading as actual application time.
Native demand T is the saved programmed alarm0 deadline; J is the original
first-C-entry real TIMER read described above. Record missing grid slots too.

PC retains EVERY source capture interval [lo_n,hi_n], clock/pacing uncertainty
and outward timestamp quantization. Pico TIMER upper bracket gets +1us outward.
PC computes H=max(Bn-lo_n), L=max(hi_n-Bn). All-event safe bounds are:
- maximum availability age <= max(Ahi-Bn)+1+H;
- maximum apply/source age <= max(Phi-Bn)+1+H;
- availability lower bound >= min(Alo-Bn)-L;
- apply/source lower bound >= min(Plo-Bn)-L.
Use exact matching boot/session/E, contiguous prefixes, counts and valid clocks.
Independent extrema may overestimate bounds: report FAIL/INCOMPLETE, never
replace them with unrelated percentiles to get a pass. PC source timestamps
must bracket the actual capture, not merely label an earlier computation.

### Proposed separate USB diagnostic format D1 (requires approval)

Explicit LE integers, no native struct transmission. Each USB record is:
00 || COBS(header || payload || CRC32C) || 00. The leading delimiter recovers
from ordinary ASCII console traffic. CRC covers header+payload. Emit each
record in one existing USB-console enqueue call, paced by pending/link state;
no TinyUSB/template API change. Truncation/no USB host leaves evidence INCOMPLETE.

Header24B: magic u32=0x31444750 (PGD1) @0; format u16=1 @4; kind u16 @6;
record_seq u32 @8; boot_nonce u32 @12; session u32 @16; payload_bytes u32 @20.
Envelope parsing permits zero nonce/session for preHELLO/startup failure
records. Full cohort qualification requires the matching nonzero negotiated
identity; an accepted envelope is NOT proof of qualified payloads or a run.
Maximum payload104B, decoded record132B. This is NOT a PIL v1 packet/extension.
Use a dedicated D1 parser/bound: NEVER raise frozen v1 MAX_BODY128 for D1.
Kinds1 CONTEXT,2 COUNTS,3 EXTREMUM,4 TRACE,5 MANIFEST; sequences contiguous from0.

CONTEXT: 26 u32 words (104B), in order:
schema,caps,epoch,grid_anchor,uart_clock_hz,requested_baud,IBRD,FBRD,
step,pipeline,expiry,slot_limit,first_stage_seq,last_stage_seq,
first_apply_seq,last_apply_seq,first_demand_seq,last_demand_seq,
run_start_TIMER,freeze_TIMER,stop_reason,final_state_flags,coverage_flags,
profile_flags,metric_count,trace_count.
Coverage flags: bit0 contiguous stages,1 contiguous applies,2 every demanded
slot accounted,3 complete reductions,4 output cohort drained,5 first latch was
accepted DISARM,6 counter overflow,7 unexpected Pico fault,8 raw queue loss,
9 native grid gap,10 immutable snapshot complete; bits11..31 reserved zero.
Pass requires bits0..5 and10 set and6..9 clear. A controlled DISARM latch is NOT
an unexpected fault. Host reset/nonce/identity detection is independently
mandatory; Pico cannot attest that it did not reset from one surviving snapshot.
Profile flags: bit0 gate1,1 SMP,2 forced-core0,3 radio off,4 network off,
5 MQTT off,6 physical actuator pins disabled; bits7..31 zero. Required0,3..6;
placement bits must match the selected image and operator image hash metadata.

COUNTS: 26 u32 words (104B), in order:
staged,applied,demanded,output_generated,output_enqueued,output_FIFO_loaded,
RX_packetq_mutations,TX_packetq_mutations,RX_packetq_overflow,TX_packetq_overflow,
dropped_input,bad_frame,missed_schedule,UART_overrun_observations,
UART_framing,UART_parity,UART_break,SENSOR_sequence_gap,stage_slot_overflow,
endpoint_deadline_miss,source_expiry,timing_invalid,control_refusal,
cancelled_slots,counter_overflow,input_progress_lease_expiry.
Wide counts do not silently wrap; v1 OUTPUT retains its specified low16 counters.
FIFO_loaded is NOT proof of PC receipt or physical wire completion.
Counts stage/apply/demand/output identify the armed run cohort (including its
initial staged frames before E). Other error/queue-operation counts cover boot
to final freeze, not subsequent repeats of the immutable dump. After first
stop, stop generating cohort OUTPUTs, cancel accepted input tail only for a
controlled DISARM, and enqueue a final inhibited status outside that cohort.
Drain work/TX queues and UART BUSY before final freeze/USB dump. Preserve all
cohort bytes; no safety-dependent waiting for a drain. A broken drain is timed
out and marked INCOMPLETE, never claimed delivered. Subsequent stopped status
or diagnostic responses cannot change the frozen summary. PC receipt remains
independent evidence.

EXTREMUM: 12 words (48B): metric_id,count,min_i32,min_sensor_seq,
min_raw_lo,min_raw_hi,min_reference,max_i32,max_sensor_seq,
max_raw_lo,max_raw_hi,max_reference. Min/max values are signed bounded modulo
microsecond differences (or queue depth); raw stamps permit arg-extreme checks.
Metric IDs1..12: availability-B,apply-B,endpoint deadline slack,
apply-nominal-slot,native-C-entry-demand,apply-native-entry,
queue-after-stage,queue-after-apply,RX-packetq-depth,TX-packetq-depth,
UART-IRQ-service-walltime,worker-service-walltime.
Raw stamps AND stored extrema differences do NOT include the outward +1us.
The verifier adds upper-endpoint quantization ONCE (lower slack endpoints
subtract it ONCE); H/L already contain the PC capture/clock outward rounding.
Compute signed differences by half-range unwrap near E/source reference;
reject any ambiguous or >=2^31 span, never subtract raw unsigned timestamps
as if the Pico clock could not wrap. Service/queue metrics9..12 have argseq
NO_ACK and reference0, not an invented SENSOR identity. Their event times still
unwrap near the run. For count0, min/max/raw/reference words are0 and both
argseq words are NO_ACK; every metric ID is still present. A clean soak cannot
use count0 for required coverage. Device percentiles are not claimed.

TRACE: 16 u32 words (64B): event_code,sensor_seq,source_us,demand_TIMER,
event_TIMER,Alo,Ahi,Plo,Phi,queue_depth,state_flags,dropped,bad,missed,
overrun,reason. event_TIMER is native J for native-entry/slot events, otherwise
the real event capture TIMER; unavailable bracket/reference words are0 and
non-slot sensor_seq is NO_ACK. At most64 entries, ordered/labelled partial.
Event codes:1 stage,2 apply,3 native entry,4 accepted DISARM,5 first fault,
6 lease IRQ,7 UART error,8 queue overflow,9 startup failure,10 final freeze.
Stop reason0 means none (cannot close a valid run);1..8 are the unchanged v1
DISARM reasons. Fault reasons0x100 missing endpoint,101 late endpoint,
102 stage overflow,103 source expiry,104 input lease,105 timing invalid,
106 non-neutral/bad-delta input,107 RX packet queue loss,108 TX packet queue
loss,109 hardware UART overrun,10a native grid gap,10b non-neutral output,
10c startup failure,10d session limit,10e metrics/counter overflow,
10f OUTPUT snapshot queue overflow. Other UART byte errors are counted and
invalidate the packet, not an invented immediate whole-link CRC fault.
First stop reason is immutable: a later DISARM never overwrites a fault.
MANIFEST: 4 u32 words: preceding_record_count,metric_count,trace_count,
CRC32C of concatenated preceding decoded header+payload bytes (exclude each
record's CRC). It closes the immutable dump; no manifest or mismatch =INCOMPLETE.
Repeated dumps contain the same snapshot; no mixture of different snapshots.
Record order is EXACTLY CONTEXT(seq0),COUNTS(seq1),EXTREMUM IDs1..12(seq2..13),
TRACE in retained event order(seq14..13+T),MANIFEST(seq14+T),0<=T<=64.
Preceding records=14+T; total records=15+T. Reject missing/duplicate kinds or
metric IDs, out-of-order records, changed identity, invalid flags/lengths,
unknown codes or a manifest/CRC mismatch. A repeated identical complete dump
may restart at seq0; partial snapshots cannot be joined across repetitions.

Coverage equations at a controlled stop:
- stages are exactly0..staged-1; applies exactly0..applied-1;
- staged=applied+cancelled; cancelled is exactly the accepted contiguous tail,
  cannot exceed8, and is not permission to cancel required work while running;
- demanded=applied and its source prefix matches applies; native gaps invalidate;
- metric counts1/3/7=staged,2/4/6/8=applied,5=demanded;
- metrics9/10 counts equal corresponding packet-queue mutation counters;
- output_generated=output_enqueued=output_FIFO_loaded=applied, for the active
  cohort only; no dropped/coalesced reports qualify. PC separately requires
  each of these OUTPUTs observed, not applied to nonexistent physics.
A fault may retain a useful partial snapshot but cannot satisfy clean-soak
coverage. Tail cancellation only supports a controlled DISARM stop; it must
not hide missing/late slots. Clean-soak stop reason must be1(user) or3(exit);
other DISARM reasons are separately labelled negative-campaign results.

## Exact metric interpretation

For IDs1,2,4,5,6, stored minimum is sdiff(raw_lo,reference), stored maximum
is sdiff(raw_hi,reference). For ID3 (deadline slack), minimum is
sdiff(reference,raw_hi), maximum is sdiff(reference,raw_lo). These are RAW
differences. ID5 has raw_lo=raw_hi=J and reference=demanded alarm TIMER.
ID6 has raw_lo=Plo, raw_hi=Phi, reference=J.

IDs7..10 store numeric queue depths0..8, raw_lo=raw_hi=event TIMER, reference0.
Do not compare their numeric depth to a time difference or add1 to a depth.
IDs11/12 store service duration sdiff(raw_hi,raw_lo), reference0. Their
conservative interval is [max(0,duration-1),duration+1]. Relative-J metric6
likewise uses a lower raw difference-1 and upper raw difference+1.
Arg-extreme ties retain the FIRST observation. Every raw bracket is forward
and shorter than half a TIMER period; ambiguous brackets are invalid.

## Acceptance boundary

Main approved4000us as the FIXED NOMINAL schedule, not exact actual source
age. Actual source-to-apply bounds are measured/reported separately. The
source-age upper gate remains5000us, and availability upper remains3000us
AND its preceding-slot deadline. Do not silently raise any limit.

Native first-C-entry jitter (ID5) and logical apply jitter (ID4) each have
a CANDIDATE250us upper threshold, including outward quantization. Revisit
only after P1 hardware data and explicit review; never raise it silently.
Clock halfwidth and source-phase error retain their separate250us gates.
These values and all device metrics are NOT measured by a host-only pass.

Transport-only cap4/no-physics images must NOT be labelled as functional
ADC/encoder/IMU/sonar acceptance. Device arrival/apply/jitter percentiles remain
UNMEASURED with D1 extrema. The optional64-record trace is partial evidence.
