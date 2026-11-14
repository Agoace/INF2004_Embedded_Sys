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
import time

from .clock import ClockError, unwrap32
from .transport import HostTransport, TransportFault
from .wire import Type, u16


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
        self.csv.writerow((event, pc_us, json.dumps(details, sort_keys=True)))

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
        self.stream.flush()
        summary = {
            'schema': 1, 'verdict': 'FAIL' if reason else 'INCOMPLETE',
            'failure': reason, 'elapsed_us': elapsed_us,
            'scope': 'transport-only synthetic source; no physics ACK or firmware verdict',
            'candidate_bounds_us': {'pipeline': 4000, 'expiry': 5000,
                                    'endpoint_available': 3000, 'clock_halfwidth': 250,
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


def receive_chunk(host, recorder, data, pc_us):
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
        host.receive(data[start:end], pc_us)
        delta = tuple(after - old for after, old in zip(
            (host.decoder.bad_frames, host.decoder.crc_errors, host.decoder.oversize_frames), before))
        if delta[0]:
            if not bootstrap:
                raise GateFailure('invalid received frame')
            recorder.bootstrap_bad_frames += delta[0]
            recorder.bootstrap_crc_errors += delta[1]
            recorder.bootstrap_oversize_frames += delta[2]
            recorder.record('bootstrap_discard', pc_us, bad_frames=delta[0],
                            crc_errors=delta[1], oversize_frames=delta[2],
                            readiness_or_lease_renewed=False)
        for event in host.drain_events():
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
             transport_only=False, image_sha256=None, profile_flags=None):
    """Own the injected nonblocking I/O until close; tests supply fake I/O/time.

    No catch-up slots or automatic reconnect. Drain evidence every iteration.
    No-Webots mode observes OUTPUT but never labels it physics-applied or ACKs it.
    """
    if type(duration_us) is not int or not 1000 <= duration_us <= 1_700_000_000:
        raise ValueError('soak duration must be1ms..1700s')
    clock = clock or RealClock()
    host = HostTransport(session or secrets.randbelow(0xffffffff) + 1,
                         transport_only=transport_only)
    try:
        recorder = Recorder(output)
    except BaseException:
        io.close()
        raise
    recorder.image_sha256, recorder.profile_flags = image_sha256, profile_flags
    recorder.pc_clock_domain = getattr(clock, 'domain', 'INJECTED_TEST_CLOCK')
    recorder.record('run_metadata', clock.now_us(), pc_clock_domain=recorder.pc_clock_domain,
                    image_sha256=image_sha256,
                    profile_flags=profile_flags, required_capabilities=host.required_caps,
                    clock_proven_min_tx_us=host.proven_min_tx_us,
                    clock_proven_min_rx_us=host.proven_min_rx_us,
                    clock_minima_proof_verdict='INCOMPLETE' if host.proven_min_tx_us else 'NOT_USED')
    start = clock.now_us()
    end = None
    active_start = None
    reason, hello_sent, armed_once = None, False, False
    next_sync = start
    try:
        if not callable(getattr(io, 'reset_input_buffer', None)):
            raise GateFailure('I/O backend lacks input-buffer reset')
        io.reset_input_buffer()
        discard_start = clock.now_us()
        discard_until = discard_start + recorder.startup_discard_us
        recorder.record('startup_input_reset', discard_start,
                        discard_until_pc_us=discard_until, minimum_discard_us=300000)
        while True:
            now = clock.now_us()
            if end is not None and now >= end:
                break
            if not armed_once and now - start > 5_000_000:
                raise GateFailure('bootstrap timeout')
            chunk = io.read(4096)
            if chunk:
                received = clock.now_us()
                recorder.record('raw_rx', received, packet_hex=chunk.hex())
                if now < discard_until:
                    recorder.startup_discarded_bytes += len(chunk)
                    recorder.record('startup_discard', received, packet_hex=chunk.hex(),
                                    bytes=len(chunk), used_for_bootstrap=False)
                else:
                    receive_chunk(host, recorder, chunk, received)
            for event in host.drain_events():
                recorder.observe(event, host)
            now = clock.now_us()
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
                    next_sync = now + min(100_000, max(50, headroom // 2))
            if host.state == 'NEGOTIATED' and host.clock.observations >= 8:
                host.arm(now)
                armed_once = True
            if host.state in ('ARM_PENDING', 'ARMED'):
                host.tick(now)
                source = host.next_sensor * 1000
                bounds = host.clock.pico_range(now)
                target = host.timeline.source_target(source)
                if bounds.midpoint >= target:
                    capture_start = clock.now_us()
                    payload = neutral_payload(source)
                    capture_end = clock.now_us()
                    host.sensor(payload, source, capture_start, capture_end_pc_us=capture_end)
                # No physics exists in this harness. Leave applied_output=NO_ACK.
                while host.outputs:
                    frame = host.outputs.popleft()
                    recorder.record('output_observed_no_physics', now, seq=frame.sequence,
                                    pwm=struct.unpack_from('<hh', frame.payload))
            packet = host.peek_tx()
            if packet:
                written = io.write(packet)
                completed = clock.now_us()
                host.consume_tx(written, completed)
                recorder.record('raw_pc_write', completed, packet_hex=packet[:written].hex(),
                                device_available=False)
            for event in host.drain_events():
                recorder.observe(event, host)
            recorder.stream.flush()
            clock.sleep_us(50)
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
                count = io.write(packet[offset:])
                if type(count) is not int or not 0 <= count <= len(packet)-offset:
                    raise GateFailure('invalid stop-write result')
                offset += count
                recorder.record('best_effort_disarm', clock.now_us(), bytes=count,
                                delivered=False, local_model_inhibited=True)
                if offset == len(packet) or reason is not None or clock.now_us() >= deadline:
                    break
                clock.sleep_us(50)
            # A planned stop preserves every final active report before the
            # final inhibited status. No new source, SYNC or ARM is generated.
            if reason is None and offset == len(packet):
                drain_end = clock.now_us() + 100_000
                while clock.now_us() < drain_end and not recorder.stopped_status_observed:
                    chunk = io.read(4096)
                    received = clock.now_us()
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
                    clock.sleep_us(50)
        except (OSError, ValueError, TransportFault, GateFailure) as error:
            reason = reason or str(error)
        finally:
            try:
                io.close()
            except OSError as error:
                reason = reason or 'serial close failed: ' + str(error)
    # Preserve queued failure evidence even when receive() faulted mid-chunk.
    for event in host.drain_events():
        recorder.record('final_transport_event', clock.now_us(), record=event)
    return recorder.finish(host, reason, active_elapsed)
