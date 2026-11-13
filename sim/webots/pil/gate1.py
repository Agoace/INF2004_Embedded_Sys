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
    def now_us(self):
        return time.monotonic_ns() // 1000

    def sleep_us(self, delay):
        time.sleep(max(0, delay) / 1_000_000)


class GateFailure(RuntimeError):
    pass


class Histogram:
    """Integer-us histogram, bounded independently of run duration."""
    def __init__(self):
        self.counts = Counter()

    def add(self, value):
        value = round(value)
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
            'clock_halfwidth_us', 'output_interarrival_us')}
        self.counters = Counter()
        self.previous = {}
        self.last_output_pc = None
        self.first_offset = None
        self.drift = None
        self.queue_max = 0

    def record(self, event, pc_us, **details):
        self.csv.writerow((event, pc_us, json.dumps(details, sort_keys=True)))

    def observe(self, event, host):
        event = dict(event)
        name, pc = event.pop('event'), event.pop('pc_us', None)
        self.record(name, pc, **event)
        if name == 'source_capture':
            self.metrics['source_phase_us'].add(event['phase_us'])
            bounds = host.clock.pico_range(pc)
            self.record('source_clock_bounds', pc, source_us=event['source_us'],
                        lower_pico_us=bounds.lower, upper_pico_us=bounds.upper,
                        intended_pico_us=host.timeline.source_target(event['source_us']))
        if name != 'rx_complete':
            return
        kind, seq = event['type'], event['seq']
        # BOOT is a separate reset/session observation, not the1kHz stream.
        if kind == Type.BOOT:
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
            if host.clock.observations >= 8:
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
            'serial': {'baud': 921600, 'data_bits': 8, 'parity': 'none',
                       'stop_bits': 1, 'flow_control': False},
            'counts': dict(self.counters), 'input_queue_depth_max': self.queue_max,
            'decoder_bad_frames': host.decoder.bad_frames,
            'decoder_crc_errors': host.decoder.crc_errors,
            'decoder_oversize_frames': host.decoder.oversize_frames,
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


def neutral_payload(source_us):
    """Deliberately unavailable optical/IMU/sonar models, neutral buttons/counts.

    Only timing, encoder endpoints and released buttons are valid. This is a
    transport stimulus, never a calibrated sensor population or navigation run.
    """
    payload = bytearray(40)
    struct.pack_into('<H', payload, 36, 0x800 | 0x400 | 0x18)
    return bytes(payload)


def run_soak(io, output, *, clock=None, duration_us=600_000_000, session=None):
    """Own the injected nonblocking I/O until close; tests supply fake I/O/time.

    No catch-up slots or automatic reconnect. Drain evidence every iteration.
    No-Webots mode observes OUTPUT but never labels it physics-applied or ACKs it.
    """
    if type(duration_us) is not int or not 1000 <= duration_us <= 1_700_000_000:
        raise ValueError('soak duration must be1ms..1700s')
    clock = clock or RealClock()
    host = HostTransport(session or secrets.randbelow(0xffffffff) + 1)
    try:
        recorder = Recorder(output)
    except BaseException:
        io.close()
        raise
    start = clock.now_us()
    end = None
    active_start = None
    reason, hello_sent, armed_once = None, False, False
    next_sync = start
    try:
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
                host.receive(chunk, received)
                if host.decoder.bad_frames:
                    raise GateFailure('invalid received frame')
            for event in host.drain_events():
                recorder.observe(event, host)
            now = clock.now_us()
            if host.state == 'ARMED' and active_start is None:
                active_start, end = now, now + duration_us
            if host.state == 'BOOT' and not hello_sent:
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
                    host.sensor(neutral_payload(source), source, now)
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
        # Local inhibit is immediate; only ONE nonblocking best-effort write.
        try:
            now = clock.now_us()
            packet = host.disarm_packet(3 if reason is None else 7, now)
            count = io.write(packet)
            recorder.record('best_effort_disarm', clock.now_us(), bytes=count,
                            delivered=False, local_model_inhibited=True)
        except (OSError, ValueError):
            pass
        finally:
            try:
                io.close()
            except OSError as error:
                reason = reason or 'serial close failed: ' + str(error)
    # Preserve queued failure evidence even when receive() faulted mid-chunk.
    for event in host.drain_events():
        recorder.record('final_transport_event', clock.now_us(), record=event)
    return recorder.finish(host, reason,
                           0 if active_start is None else clock.now_us() - active_start)
