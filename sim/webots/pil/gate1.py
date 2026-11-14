"""Transport-only Gate1 recorder. No Webots, physics, navigation or device auto-open.

Pico staging/apply/alarm diagnostics await an approved channel. Therefore this
version can FAIL measured gates, but cannot issue a physical Gate1 PASS.
"""

from collections import Counter
import csv
import json
import math
from pathlib import Path
import secrets
import struct
import sys
import time

from .clock import ClockError, unwrap32
from .transport import HostTransport, TransportFault
from .wire import Type, u16
from .gate1_trial import labels, manifest, TRIAL_LABEL
from .gate1_timer_slack import read_timer_slack


class RealClock:
    domain = 'CLOCK_MONOTONIC_RAW'

    def now_us(self):
        if not hasattr(time, 'CLOCK_MONOTONIC_RAW'):
            raise ClockError('CLOCK_MONOTONIC_RAW unavailable; no slewed-clock fallback')
        return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) // 1000

    def sleep_us(self, delay):
        time.sleep(max(0, delay) / 1_000_000)


class GateFailure(RuntimeError):
    pass


class Histogram:
    """Integer-us histogram, bounded independently of run duration."""
    def __init__(self):
        self.counts = Counter()

    def add(self, value):
        value = math.ceil(value)  # Never round an upper bound down.
        if value not in self.counts and len(self.counts) >= 20000:
            raise GateFailure('metric histogram capacity exceeded')
        self.counts[value] += 1

    def summary(self):
        total = sum(self.counts.values())
        if not total:
            return {'count': 0, 'p50': None, 'p95': None, 'p99': None, 'max': None}
        result = {'count': total, 'max': max(self.counts)}
        for label, fraction in (('p50', .5), ('p95', .95), ('p99', .99)):
            target, accumulated = math.ceil(total * fraction), 0
            for value, count in sorted(self.counts.items()):
                accumulated += count
                if accumulated >= target:
                    result[label] = value
                    break
        return result


class Recorder:
    def __init__(self, output):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.stream = (self.output / 'events.csv').open('x', newline='')
        self.csv = csv.writer(self.stream)
        self.attribution = None
        self.trial_clock_400_sync20 = False
        self.timer_slack = None
        self.interpreter_switch_interval_s = None
        self.csv.writerow(('event', 'pc_us', 'details_json'))
        self.metrics = {name: Histogram() for name in (
            'sync_rtt_us', 'output_age_upper_us', 'source_phase_us',
            'clock_halfwidth_us', 'source_uncertainty_us', 'output_interarrival_us')}
        self.counters = Counter()
        self.previous = {}
        self.last_output_pc = None
        self.first_offset = None
        self.drift = None
        self.queue_max = 0
        self.stopped_status_observed = False
        self.image_sha256 = None
        self.profile_flags = None
        self.pc_clock_domain = None
        self.bootstrap_bad_frames = 0
        self.bootstrap_crc_errors = 0
        self.bootstrap_oversize_frames = 0
        self.startup_discarded_bytes = 0
        self.startup_discard_us = 300000
        self.bootstrap_first_boot_seq = None
        self.bootstrap_last_boot = None
        self.bootstrap_consecutive_boots = 0

    def record(self, event, pc_us, **details):
        if self.attribution:
            self.attribution.mark('serialize_enter')
        try:
            self.csv.writerow((event, pc_us, json.dumps(details, sort_keys=True)))
        finally:
            if self.attribution:
                self.attribution.mark('serialize_exit')

    def flush(self):
        if self.attribution:
            self.attribution.mark('flush_enter')
        try:
            self.stream.flush()
        finally:
            if self.attribution:
                self.attribution.mark('flush_exit')

    def observe(self, event, host):
        event = dict(event)
        name, pc = event.pop('event'), event.pop('pc_us', None)
        self.record(name, pc, **event)
        if name == 'source_capture':
            self.metrics['source_phase_us'].add(event['phase_us'])
            self.metrics['source_uncertainty_us'].add(event['uncertainty_us'])
            self.record('source_clock_bounds', pc, source_us=event['source_us'],
                        capture_end_pc_us=event['capture_end_pc_us'],
                        lower_pico_us=event['source_lower_pico_us'],
                        upper_pico_us=event['source_upper_pico_us'],
                        intended_pico_us=host.timeline.source_target(event['source_us']))
        if name != 'rx_complete':
            return
        kind, seq = event['type'], event['seq']
        # BOOT is a separate reset/session observation, not the1kHz stream.
        if kind == Type.BOOT:
            nonce = struct.unpack_from('<I', bytes.fromhex(event['payload_hex']))[0]
            seq = event['seq']
            previous = self.bootstrap_last_boot
            consecutive = previous is not None and nonce == previous[0] and seq == ((previous[1] + 1) & 0xffffffff)
            self.bootstrap_consecutive_boots = self.bootstrap_consecutive_boots + 1 if consecutive else 1
            self.bootstrap_last_boot = (nonce, seq)
            if self.bootstrap_first_boot_seq is None:
                self.bootstrap_first_boot_seq = seq
            self.record('bootstrap_boot_candidate', pc, nonce=nonce, seq=seq,
                        consecutive=self.bootstrap_consecutive_boots,
                        pair_ready=self.bootstrap_consecutive_boots >= 2)
            return
        if event['session'] != host.session:
            self.counters['wrong_session'] += 1
            raise GateFailure('unexpected session')
        previous = self.previous.get(kind)
        if previous is None and seq != 0:
            self.counters['missing_frames'] += seq
            raise GateFailure('initial frame sequence gap')
        if previous is not None:
            delta = (seq - previous) & 0xffffffff
            if delta == 0 or delta >= 0x80000000:
                self.counters['duplicate_or_reordered'] += 1
                raise GateFailure('duplicate/reordered frame')
            if delta != 1:
                self.counters['missing_frames'] += delta - 1
                raise GateFailure('frame sequence gap')
        self.previous[kind] = seq
        payload = bytes.fromhex(event['payload_hex'])
        if kind == Type.OUTPUT:
            self.counters['outputs'] += 1
            if struct.unpack_from('<hh', payload) != (0, 0):
                raise GateFailure('nonzero PWM in transport-only profile')
            self.queue_max = max(self.queue_max, payload[10])
            if self.last_output_pc is not None:
                self.metrics['output_interarrival_us'].add(pc - self.last_output_pc)
            self.last_output_pc = pc
            counters = tuple(u16(payload, off) for off in (24, 26, 28, 30))
            if any(counters):
                raise GateFailure('Pico loss/bad-frame/missed-schedule/overrun counter nonzero')
            if host.clock.observations >= 8 and (host.state != 'FAULT' or payload[7] & 1):
                bounds = host.clock.pico_range(pc)
                snapshot = unwrap32(event['source_us'], round(bounds.midpoint))
                age = bounds.upper - snapshot
                self.metrics['output_age_upper_us'].add(age)
                if snapshot > bounds.upper or age > 5000:
                    raise GateFailure('OUTPUT source age outside5ms bound')
        elif kind == Type.SYNC_REPLY:
            sent = struct.unpack_from('<Q', payload)[0]
            self.metrics['sync_rtt_us'].add(pc - sent)
            interval = host.clock.bounds_at(pc, qualified=False)
            self.metrics['clock_halfwidth_us'].add(interval.halfwidth)
            self.record('clock_bounds', pc, lower_us=interval.lower, upper_us=interval.upper)
            if self.first_offset is None:
                self.first_offset = (pc, interval)
            start, first = self.first_offset
            if pc > start:
                elapsed = pc - start
                self.drift = {'lower_ppm': (interval.lower - first.upper) * 1e6 / elapsed,
                              'upper_ppm': (interval.upper - first.lower) * 1e6 / elapsed}
                # An interval wholly outside the100ppm assumption disproves it.
                if self.drift['lower_ppm'] > 100 or self.drift['upper_ppm'] < -100:
                    raise GateFailure('measured drift contradicts100ppm clock bound')

    def finish(self, host, reason, elapsed_us):
        self.flush()
        trial = self.trial_clock_400_sync20
        completed = (trial and not reason and elapsed_us >= 600_000_000 and
                     self.stopped_status_observed)
        summary = {
            **labels(trial), 'sync_max_period_us': 20000 if trial else 100000,
            'pc_capture_order': 'capture-first', 'pc_logging': 'synchronous',
            'pc_capture_revision': 'C1+C2+C2b+C2c',
            'timer_slack': self.timer_slack,
            'interpreter_switch_interval_s': self.interpreter_switch_interval_s,
            'schema': 1, 'verdict': 'FAIL' if reason else ('TRIAL_COMPLETE' if completed else 'INCOMPLETE'),
            'failure': reason, 'elapsed_us': elapsed_us,
            'scope': 'transport-only synthetic source; no physics ACK or firmware verdict',
            'candidate_bounds_us': {'pipeline': 4000, 'expiry': 5000,
                                    'endpoint_available': 3000, 'clock_halfwidth': 400 if trial else 250,
                                    'source_phase': 250},
            'metrics_us': {name: hist.summary() for name, hist in self.metrics.items()},
            'observed_offset_drift_interval': self.drift,
            'sequence_coverage': {str(int(kind)): seq for kind, seq in self.previous.items()},
            'identity': {'host_session': host.session,
                         'boot_nonce': (int.from_bytes(host.boot.payload[:4], 'little')
                                        if host.boot else None),
                         'epoch_pico_us': host.timeline.epoch_pico_us if host.timeline else None},
            'stopped_status_observed': self.stopped_status_observed,
            'image_sha256': self.image_sha256, 'profile_flags': self.profile_flags,
            'pc_clock_domain': self.pc_clock_domain,
            'required_capabilities': host.required_caps,
            'clock_proven_min_tx_us': host.proven_min_tx_us,
            'clock_proven_min_rx_us': host.proven_min_rx_us,
            'clock_minima_proof_verdict': 'INCOMPLETE' if host.proven_min_tx_us else 'NOT_USED',
            'clock_minima_condition': 'matching stopped D1; 125MHz IBRD8 FBRD31; BAD11/OE13/FE14/PAR15/BRK16 all zero',
            'serial': {'baud': 921600, 'data_bits': 8, 'parity': 'none',
                       'stop_bits': 1, 'flow_control': False},
            'counts': dict(self.counters), 'input_queue_depth_max': self.queue_max,
            'decoder_bad_frames': host.decoder.bad_frames - self.bootstrap_bad_frames,
            'decoder_crc_errors': host.decoder.crc_errors - self.bootstrap_crc_errors,
            'decoder_oversize_frames': host.decoder.oversize_frames - self.bootstrap_oversize_frames,
            'decoder_total_bad_frames': host.decoder.bad_frames,
            'decoder_total_crc_errors': host.decoder.crc_errors,
            'decoder_total_oversize_frames': host.decoder.oversize_frames,
            'startup_discard_us': self.startup_discard_us,
            'startup_discarded_bytes': self.startup_discarded_bytes,
            'bootstrap_first_valid_boot_seq': self.bootstrap_first_boot_seq,
            'bootstrap_last_valid_boot_seq': self.bootstrap_last_boot[1] if self.bootstrap_last_boot else None,
            'bootstrap_consecutive_boots': self.bootstrap_consecutive_boots,
            'bootstrap_discarded_bad_frames': self.bootstrap_bad_frames,
            'bootstrap_discarded_crc_errors': self.bootstrap_crc_errors,
            'bootstrap_discarded_oversize_frames': self.bootstrap_oversize_frames,
            'unmeasured': ['Pico endpoint staging/source-to-availability',
                           'Pico actual apply/source-to-apply', 'native alarm jitter',
                           'pause/unplug/reset/no-auto-rearm hardware cases'],
            'ten_minute_duration_met': elapsed_us >= 600_000_000,
            'physical_gate1_pass': False,
        }
        temporary = self.output / 'summary.json.tmp'
        temporary.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
        temporary.replace(self.output / 'summary.json')
        self.stream.close()
        return summary


def receive_chunk(host, recorder, data, pc_us, timing_clock=None):
    """Only before valid BOOT, retain/count malformed startup fragments.

    Boundaries are delimiters, NEVER a byte-offset search inside a failed
    packet. A good first BOOT is accepted even without a leading delimiter.
    Process spans separately so post-BOOT errors in the same read stay fatal.
    """
    if not isinstance(data, (bytes, bytearray)) or len(data) > host.MAX_RX_CHUNK:
        host.receive(data, pc_us)  # Existing fail-closed input bounds.
        return
    start = 0
    while start < len(data):
        delimiter = data.find(b'\x00', start)
        end = len(data) if delimiter < 0 else delimiter + 1
        bootstrap = host.boot is None
        before = (host.decoder.bad_frames, host.decoder.crc_errors, host.decoder.oversize_frames)
        try:
            host.receive(data[start:end], pc_us)
        except TransportFault:
            if timing_clock is not None:
                recorder.record('transport_receive_failed_timing', timing_clock.now_us(),
                                received_pc_us=pc_us, span_bytes=end-start,
                                failure_layer='transport validation; raw bytes retained')
            raise
        processing_done = timing_clock.now_us() if timing_clock is not None else None
        delta = tuple(after - old for after, old in zip(
            (host.decoder.bad_frames, host.decoder.crc_errors, host.decoder.oversize_frames), before))
        if delta[0]:
            if not bootstrap:
                if processing_done is not None:
                    recorder.record('transport_receive_failed_timing', processing_done,
                                    received_pc_us=pc_us, span_bytes=end-start,
                                    failure_layer='CRC/codec; raw bytes retained')
                raise GateFailure('invalid received frame')
            recorder.bootstrap_bad_frames += delta[0]
            recorder.bootstrap_crc_errors += delta[1]
            recorder.bootstrap_oversize_frames += delta[2]
            recorder.record('bootstrap_discard', pc_us, bad_frames=delta[0],
                            crc_errors=delta[1], oversize_frames=delta[2],
                            readiness_or_lease_renewed=False)
        for event in host.drain_events():
            if processing_done is not None and event['event'] == 'rx_complete':
                recorder.record('transport_receive_timing', processing_done,
                                received_pc_us=pc_us, type=int(event['type']),
                                seq=event['seq'], session=event['session'],
                                includes_decode_and_transport_validation=True)
            if bootstrap and event['event'] == 'rx_complete' and event['type'] != Type.BOOT:
                recorder.record('bootstrap_frame_ignored', pc_us, frame=event)
            else:
                recorder.observe(event, host)
        if bootstrap and host.boot is not None:
            recorder.record('first_valid_boot', pc_us, seq=host.boot.sequence,
                            source_us=host.boot.source_us)
        start = end


def neutral_payload(source_us):
    """Deliberately unavailable optical/IMU/sonar models, neutral buttons/counts.

    Only timing, encoder endpoints and released buttons are valid. This is a
    transport stimulus, never a calibrated sensor population or navigation run.
    """
    payload = bytearray(40)
    struct.pack_into('<H', payload, 36, 0x800 | 0x400 | 0x18)
    return bytes(payload)


def run_soak(io, output, *, clock=None, duration_us=600_000_000, session=None,
             transport_only=False, image_sha256=None, profile_flags=None, measure_io=False,
             trial_clock_400_sync20=False):
    """Own the injected nonblocking I/O until close; tests supply fake I/O/time.

    No catch-up slots or automatic reconnect. Drain evidence every iteration.
    No-Webots mode observes OUTPUT but never labels it physics-applied or ACKs it.
    """
    if type(duration_us) is not int or not 1000 <= duration_us <= 1_700_000_000:
        raise ValueError('soak duration must be1ms..1700s')
    if type(trial_clock_400_sync20) is not bool:
        raise ValueError('trial mode must be an explicit boolean')
    if trial_clock_400_sync20 and not transport_only:
        raise ValueError('TRIAL requires explicit transport-only cap4')
    clock = clock or RealClock()
    host = HostTransport(session or secrets.randbelow(0xffffffff) + 1,
                         transport_only=transport_only,
                         trial_clock_400_sync20=trial_clock_400_sync20)
    timer_slack = read_timer_slack()  # GET only, calling thread.
    switch_interval = sys.getswitchinterval()  # Read only; never set.
    try:
        recorder = Recorder(output)
    except BaseException:
        io.close()
        raise
    recorder.trial_clock_400_sync20 = trial_clock_400_sync20
    recorder.timer_slack = timer_slack
    recorder.interpreter_switch_interval_s = switch_interval
    recorder.image_sha256, recorder.profile_flags = image_sha256, profile_flags
    recorder.pc_clock_domain = getattr(clock, 'domain', 'INJECTED_TEST_CLOCK')
    recorder.record('run_metadata', clock.now_us(), **labels(trial_clock_400_sync20),
                    pc_capture_order='capture-first', pc_logging='synchronous',
                    pc_capture_revision='C1+C2+C2b+C2c', timer_slack=timer_slack,
                    interpreter_switch_interval_s=switch_interval,
                    clock_halfwidth_limit_us=host.clock.MAX_HALFWIDTH_US,
                    sync_max_period_us=20000 if trial_clock_400_sync20 else 100000,
                    source_phase_limit_us=250, no_early_midpoint_required=trial_clock_400_sync20,
                    pc_clock_domain=recorder.pc_clock_domain,
                    image_sha256=image_sha256,
                    profile_flags=profile_flags, required_capabilities=host.required_caps,
                    clock_proven_min_tx_us=host.proven_min_tx_us,
                    clock_proven_min_rx_us=host.proven_min_rx_us,
                    clock_minima_proof_verdict='INCOMPLETE' if host.proven_min_tx_us else 'NOT_USED')
    start = clock.now_us()
    end = None
    active_start = None
    reason, hello_sent, armed_once = None, False, False
    serial_closed = False
    next_sync = start
    tx_bytes = rx_bytes = 0

    def clock_pair():
        # usbmon binary timestamps use REALTIME, NOT MONOTONIC_RAW. This is
        # diagnostic only. These samples NEVER enter the live clock estimator.
        if recorder.pc_clock_domain == 'CLOCK_MONOTONIC_RAW':
            before = clock.now_us()
            realtime = time.time_ns() // 1000
            after = clock.now_us()
            recorder.record('measurement_clock_pair', after, realtime_us=realtime,
                            raw_before_us=before, raw_after_us=after)

    attribution = None
    host_spans = None

    def mark(kind, value=0):
        if attribution is not None:
            attribution.mark(kind, value=value)

    def sleep(delay):
        mark('sleep_enter', delay)
        span = host_spans.begin('sleep', attribution.loop if attribution else 0) if host_spans else None
        try:
            clock.sleep_us(delay)
        finally:
            if host_spans: host_spans.end(span)
        mark('wake', delay)

    def read_input():
        nonlocal rx_bytes
        mark('read_enter')
        if not measure_io:
            data = io.read(4096)
            mark('read_exit', len(data))
            return data, clock.now_us() if data else None
        entered = clock.now_us()
        if callable(getattr(io, 'read_timed', None)):
            if getattr(io, 'host_spans_supported', False):
                data, timing = io.read_timed(4096, clock.now_us, observer=host_spans,
                                             loop=attribution.loop if attribution else 0)
            else:
                data, timing = io.read_timed(4096, clock.now_us)
        else:
            span = host_spans.begin('backend_read', attribution.loop if attribution else 0) if host_spans else None
            try:
                data = io.read(4096)
            finally:
                if host_spans: host_spans.end(span)
            timing = {'select_layer': 'UNKNOWN', 'read_return_pc_us': clock.now_us()}
        returned = timing['read_return_pc_us'] if 'read_return_pc_us' in timing else clock.now_us()
        mark('read_exit', len(data))
        if data:
            clock_pair()
            recorder.record('io_read_timing', returned, call_enter_pc_us=entered,
                            bytes=len(data), stream_start=rx_bytes,
                            stream_end=rx_bytes + len(data), **timing)
            rx_bytes += len(data)
        return data, returned

    capture_used = False

    def capture_if_due():
        nonlocal capture_used
        checked = clock.now_us()
        if host.state not in ('ARM_PENDING', 'ARMED'):
            return checked
        # Expiry/inhibit checks apply even after this loop's capture is used.
        host.tick(checked)
        mark('tick_exit')
        if capture_used:
            return checked
        source = host.next_sensor * 1000
        bounds = host.clock.pico_range(checked)
        if bounds.midpoint >= host.timeline.source_target(source):
            mark('capture_enter', source)
            capture_start = clock.now_us()
            payload = neutral_payload(source)
            capture_end = clock.now_us()
            mark('capture_exit', source)
            # Fresh capture after any preceding SYNC; never revalidate an old
            # capture using a newer cone. Shared validator keeps phase/no-early.
            host.sensor(payload, source, capture_start, capture_end_pc_us=capture_end)
            capture_used = True
            # sensor() advances host time through capture END. Subsequent SYNC
            # must not receive the older pre-capture check timestamp.
            return clock.now_us()
        return checked

    try:
        if measure_io:
            from .gate1_attribution import AttributionRing
            from .gate1_host_observer import HostSpans, host_snapshot
            (recorder.output/'host-before.json').write_text(json.dumps(
                {**host_snapshot(), **labels(trial_clock_400_sync20)}, indent=2)+'\n')
            host_spans = HostSpans(clock, labels=labels(trial_clock_400_sync20))
            attribution = AttributionRing(clock)
            attribution.qualification = TRIAL_LABEL if trial_clock_400_sync20 else None
            recorder.attribution = attribution
            attribution.install()
            recorder.record('measurement_enabled', start, production_cadence_changed=False,
                            observer_overhead_unmeasured=True,
                            pcap_clock='CLOCK_REALTIME; mapping conditional on no backward steps')
            clock_pair()
        if not callable(getattr(io, 'reset_input_buffer', None)):
            raise GateFailure('I/O backend lacks input-buffer reset')
        io.reset_input_buffer()
        discard_start = clock.now_us()
        discard_until = discard_start + recorder.startup_discard_us
        recorder.record('startup_input_reset', discard_start,
                        discard_until_pc_us=discard_until, minimum_discard_us=300000)
        while True:
            if attribution is not None:
                attribution.loop += 1
            mark('loop')
            now = clock.now_us()
            if end is not None and now >= end:
                break
            if not armed_once and now - start > 5_000_000:
                raise GateFailure('bootstrap timeout')
            # At most one capture per iteration; no catch-up or skipped slot.
            # RX faults still cancel queued SENSOR before any write below.
            capture_used = False
            now = capture_if_due()
            chunk, received = read_input()
            if chunk:
                recorder.record('raw_rx', received, packet_hex=chunk.hex())
                if now < discard_until:
                    recorder.startup_discarded_bytes += len(chunk)
                    recorder.record('startup_discard', received, packet_hex=chunk.hex(),
                                    bytes=len(chunk), used_for_bootstrap=False)
                else:
                    receive_chunk(host, recorder, chunk, received,
                                  timing_clock=clock if measure_io else None)
            mark('receive_exit')
            for event in host.drain_events():
                recorder.observe(event, host)
            mark('drain_exit')
            # Restore the opportunity if the target crossed during RX. Check
            # expiry first and capture BEFORE SYNC construction can delay it.
            now = capture_if_due()
            if host.state == 'ARMED' and active_start is None:
                active_start, end = now, now + duration_us
            if host.state == 'BOOT' and not hello_sent and recorder.bootstrap_consecutive_boots >= 2:
                host.hello(now)
                hello_sent = True
            if host.ack_frame is not None and now >= next_sync and not host.sync_pending:
                host.sync_request(now)
                if host.clock.observations < 8:
                    next_sync = now + 1000
                else:
                    headroom = host.clock.refresh_deadline_pc_us() - now
                    next_sync = now + min(20_000 if trial_clock_400_sync20 else 100_000,
                                          max(50, headroom // 2))
            if host.state == 'NEGOTIATED' and host.clock.observations >= 8:
                host.arm(now)
                armed_once = True
            mark('sync_exit')
            # A target may also cross during SYNC construction. Use NEW time
            # for expiry/qualification, never the pre-SYNC cached timestamp.
            now = capture_if_due()
            if host.state in ('ARM_PENDING', 'ARMED'):
                # No physics exists in this harness. Leave applied_output=NO_ACK.
                while host.outputs:
                    frame = host.outputs.popleft()
                    recorder.record('output_observed_no_physics', now, seq=frame.sequence,
                                    pwm=struct.unpack_from('<hh', frame.payload))
            packet = host.peek_tx()
            if packet:
                if measure_io:
                    clock_pair()
                entered = clock.now_us() if measure_io else None
                # TX preparation/output logging can cross lease/lookahead.
                # Recheck NEW time before normal writes, never after the bytes.
                if host.state in ('ARM_PENDING', 'ARMED'):
                    host.tick(clock.now_us())
                    mark('tick_exit')
                written = io.write(packet)
                completed = clock.now_us()
                valid_write = type(written) is int and 0 <= written <= len(packet)
                if measure_io and valid_write:
                    # Retain the actual OS result even if consume_tx faults.
                    recorder.record('raw_pc_write', completed, packet_hex=packet[:written].hex(),
                                    device_available=False)
                    clock_pair()
                    recorder.record('io_write_timing', completed, call_enter_pc_us=entered,
                                    bytes=written, stream_start=tx_bytes,
                                    stream_end=tx_bytes + written)
                    tx_bytes += written
                host.consume_tx(written, completed)
                if not measure_io:
                    recorder.record('raw_pc_write', completed, packet_hex=packet[:written].hex(),
                                    device_available=False)
            for event in host.drain_events():
                recorder.observe(event, host)
            recorder.flush()
            # C2c: the target may cross during TX/log/flush. Same latch/validator;
            # a newly queued SENSOR waits for the next normal TX opportunity.
            # That iteration checks expiry and RX faults before transmitting.
            capture_if_due()
            sleep(50)
    except (OSError, ValueError, TransportFault, ClockError, GateFailure) as error:
        reason = str(error)
    except KeyboardInterrupt:
        reason = 'operator interrupted; reset/new session required'
    finally:
        active_elapsed = 0 if active_start is None else clock.now_us() - active_start
        try:
            # Local inhibit happens BEFORE bounded write/drain work. A hardware
            # gate still needs the stopped D1 reason; a write is not delivery.
            packet = host.disarm_packet(3 if reason is None else 7, clock.now_us())
            offset, deadline = 0, clock.now_us() + 5000
            for _ in range(len(packet) + 100):
                entered = clock.now_us() if measure_io else None
                count = io.write(packet[offset:])
                completed = clock.now_us() if measure_io else None
                if type(count) is not int or not 0 <= count <= len(packet)-offset:
                    raise GateFailure('invalid stop-write result')
                stop_bytes = packet[offset:offset+count]
                offset += count
                if measure_io:
                    recorder.record('raw_pc_write', completed, packet_hex=stop_bytes.hex(),
                                    device_available=False, best_effort_stop=True)
                    clock_pair()
                    recorder.record('io_write_timing', completed, call_enter_pc_us=entered,
                                    bytes=count, stream_start=tx_bytes, stream_end=tx_bytes+count,
                                    best_effort_stop=True)
                    tx_bytes += count
                recorder.record('best_effort_disarm', clock.now_us(), bytes=count,
                                delivered=False, local_model_inhibited=True)
                if offset == len(packet) or reason is not None or clock.now_us() >= deadline:
                    break
                sleep(50)
            # A planned stop preserves every final active report before the
            # final inhibited status. No new source, SYNC or ARM is generated.
            if reason is None and offset == len(packet):
                drain_end = clock.now_us() + 100_000
                while clock.now_us() < drain_end and not recorder.stopped_status_observed:
                    chunk, received = read_input()
                    if chunk:
                        recorder.record('raw_rx_after_disarm', received, packet_hex=chunk.hex())
                        for frame in host.decoder.feed(chunk):
                            if frame.type == Type.BOOT or frame.session != host.session:
                                raise GateFailure('reset/session change during stopped drain')
                            if frame.type == Type.OUTPUT:
                                recorder.observe({'event': 'rx_complete', 'pc_us': received,
                                                  'type': frame.type, 'seq': frame.sequence,
                                                  'source_us': frame.source_us,
                                                  'session': frame.session, 'ack': frame.ack,
                                                  'payload_hex': frame.payload.hex()}, host)
                                if frame.payload[7] & 8:
                                    raise GateFailure('underflow during stopped drain')
                                if frame.payload[7] & 4 and not frame.payload[7] & (1 | 16):
                                    recorder.stopped_status_observed = True
                                    recorder.record('stopped_status_observed', received,
                                                    seq=frame.sequence, ack=frame.ack)
                        if host.decoder.bad_frames > recorder.bootstrap_bad_frames:
                            raise GateFailure('invalid frame during stopped drain')
                    sleep(50)
        except (OSError, ValueError, TransportFault, GateFailure) as error:
            reason = reason or str(error)
        finally:
            try:
                io.close()
                serial_closed = True
            except OSError as error:
                reason = reason or 'serial close failed: ' + str(error)
            finally:
                if attribution is not None:
                    attribution.detach()
    if host_spans is not None:
        from .gate1_host_observer import host_snapshot
        (recorder.output/'host-after.json').write_text(json.dumps(
            {**host_snapshot(), **labels(trial_clock_400_sync20)}, indent=2)+'\n')
        host_spans.finish(recorder.output, serial_closed=serial_closed)
    # Dedicated rejection evidence bypasses ordinary transport events and is
    # serialized only AFTER local inhibit, bounded DISARM attempt and close.
    rejection = getattr(host, 'capture_rejection', None)
    if rejection is not None:
        rejection = dict(rejection, **labels(trial_clock_400_sync20),
                         pc_clock_domain=recorder.pc_clock_domain,
                         serial_closed_before_log=serial_closed)
        (recorder.output/'capture-rejected.json').write_text(json.dumps(rejection, indent=2)+'\n')
        recorder.record('source_capture_rejected', rejection['capture_end_pc_us'], **rejection)
    if attribution is not None:
        attribution.detach()
        recorder.flush()
        recorder.attribution = None
        attribution.finish(recorder.output, serial_closed=serial_closed)
    if trial_clock_400_sync20:
        (recorder.output/'trial-manifest.json').write_text(json.dumps(manifest(), indent=2)+'\n')
    # Preserve queued failure evidence even when receive() faulted mid-chunk.
    for event in host.drain_events():
        recorder.record('final_transport_event', clock.now_us(), record=event)
    return recorder.finish(host, reason, active_elapsed)
