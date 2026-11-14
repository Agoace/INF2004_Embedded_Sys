"""Offline PC/usbmon/Pico correlation. No hardware access or verdict promotion.

Classic pcap LINKTYPE_USB_LINUX(189)/USB_LINUX_MMAPPED(220), both byte
orders and micro/nanosecond file timestamps. usbmon bulk payloads are matched
byte for byte, not inferred from packet lengths or USB notification intervals.
"""
from bisect import bisect_left, bisect_right
from collections import OrderedDict
import csv
import json
import re
from pathlib import Path
import struct

from .wire import decode, ProtocolError, Type
from .gate1_rx_timing import parse_rx1, elapsed
from .gate1_diagnostics import decode_dump
from .clock import unwrap32


def read_usb_pcap(stream):
    header = stream.read(24)
    magics = {b'\xd4\xc3\xb2\xa1': ('<', 1), b'\xa1\xb2\xc3\xd4': ('>', 1),
              b'\x4d\x3c\xb2\xa1': ('<', 1000), b'\xa1\xb2\x3c\x4d': ('>', 1000)}
    if len(header) != 24 or header[:4] not in magics:
        raise ValueError('not classic pcap (pcapng is not supported)')
    endian, divisor = magics[header[:4]]
    major, minor, _, _, snaplen, link = struct.unpack(endian + 'HHIIII', header[4:])
    if (major, minor) != (2, 4) or link not in (189, 220) or not 0 < snaplen <= 16_777_216:
        raise ValueError('pcap version/link/snaplen unsupported')
    size = 64 if link == 220 else 48
    while True:
        record = stream.read(16)
        if not record:
            return
        if len(record) != 16:
            raise ValueError('truncated pcap record header')
        sec, sub, captured, original = struct.unpack(endian + 'IIII', record)
        if captured > snaplen or captured > original or captured < size or sub >= 1_000_000*divisor:
            raise ValueError('pcap record size/timestamp')
        packet = stream.read(captured)
        if len(packet) != captured:
            raise ValueError('truncated pcap packet')
        tag, event, transfer, ep, dev, bus, setup, flag, usbsec, usub, status, length, cap = \
            struct.unpack_from(endian + 'QBBBBHBBqiiII', packet)
        if transfer != 3:  # Do not interpret control/ISO/notification payloads as UART.
            continue
        if event not in (ord('S'), ord('C'), ord('E')) or not 0 <= usub < 1_000_000:
            raise ValueError('unknown usbmon bulk event/timestamp')
        if cap > length or cap > original-size:
            raise ValueError('usbmon payload length')
        payload = packet[size: size + cap] if flag == 0 else b''
        yield {'tag': tag, 'event': chr(event), 'endpoint': ep, 'device': dev,
               'bus': bus, 'status': status, 'length': length, 'payload': payload,
               'payload_complete': flag == 0 and cap == length and len(payload) == length,
               'pcap_us': sec*1_000_000 + sub//divisor,
               'usb_realtime_us': usbsec*1_000_000 + usub}


class ClockBrackets:
    """Order-only mapping; no fitted rate, interpolation or clock-domain equality.

    usbmon uses REALTIME (Linux mon_bin.c ktime_get_real_ts64). The resulting
    RAW interval is CONDITIONAL on no unobserved backward REALTIME step. Missing
    enclosing samples/backward samples fail closed. This is NEVER gate evidence.
    """
    def __init__(self, pairs):
        self.pairs = pairs
        self.times = [p['realtime_us'] for p in pairs]
        self.valid = bool(pairs) and all(
            p['raw_before_us'] <= p['raw_after_us'] for p in pairs)
        self.valid = self.valid and all(
            a['realtime_us'] <= b['realtime_us'] and a['raw_after_us'] <= b['raw_before_us']
            for a, b in zip(pairs, pairs[1:]))

    def map(self, timestamp):
        if not self.valid:
            return None
        left, right = bisect_left(self.times, timestamp)-1, bisect_right(self.times, timestamp)
        if left < 0 or right >= len(self.pairs):
            return None
        return (self.pairs[left]['raw_before_us']-1, self.pairs[right]['raw_after_us']+1)


def realtime_to_raw(timestamp, pairs):
    return ClockBrackets(pairs).map(timestamp)


def _frames(data, kinds):
    start = 0
    while True:
        end = data.find(b'\0', start)
        if end < 0:
            return
        if 0 < end-start < 130:
            try:
                frame = decode(bytes(data[start:end+1]))
                if frame.type in kinds:
                    yield {'frame': frame, 'start': start, 'end': end+1}
            except ProtocolError:
                pass  # Raw startup noise stays raw; never used as valid timing.
        start = end+1


def _containing(calls, offset, ends):
    # Calls are already in byte-stream order. Empty writes are not containing.
    index = bisect_right(ends, offset-1)
    return calls[index] if index < len(calls) else None


class USBStreamMatcher:
    """Bounded offline association across short, full-payload USB completions.

    Aggregate whole URBs until a unique >=16-byte anchor exists. Backfill only
    byte-exact adjacent URBs. Never searches inside/repairs a live UART packet.
    Metadata stays attached to its original URB (including delayed OUT callback).
    """
    def __init__(self, data):
        self.data, self.cursor, self.aligned = data, 0, False
        self.pending, self.unmatched = [], 0

    @property
    def pending_records(self):
        return len(self.pending)

    @property
    def pending_bytes(self):
        return sum(len(payload) for payload, _ in self.pending)

    def finish(self):
        self.unmatched += len(self.pending)
        self.pending.clear()
        self.aligned = False

    def feed(self, payload, part):
        if not payload:
            return []
        if self.aligned and self.data[self.cursor:self.cursor+len(payload)] == payload:
            part.update(stream_start=self.cursor, stream_end=self.cursor+len(payload))
            self.cursor += len(payload)
            return [part]
        self.aligned = False
        self.pending.append((payload, part))
        while self.pending_records > 64 or self.pending_bytes > 4096:
            self.pending.pop(0)
            self.unmatched += 1
        candidate = b''
        stop = min(len(self.data), self.cursor+131072)
        for index in range(len(self.pending)-1, -1, -1):
            candidate = self.pending[index][0]+candidate
            if len(candidate) < 16:
                continue
            found = self.data.find(candidate, self.cursor, stop)
            if found < 0 or self.data.find(candidate, found+1, stop) >= 0:
                continue
            # A newer unique anchor may also identify preceding short URBs.
            while index:
                previous = self.pending[index-1][0]
                before = found-len(previous)
                if before < self.cursor or self.data[before:found] != previous:
                    break
                index, found = index-1, before
            self.unmatched += index
            matched = []
            self.cursor = found
            for data, metadata in self.pending[index:]:
                metadata.update(stream_start=self.cursor, stream_end=self.cursor+len(data))
                self.cursor += len(data)
                matched.append(metadata)
            self.pending.clear()
            self.aligned = True
            return matched
        return []


def _regular(path, maximum=None):
    path = Path(path)
    if not path.is_file() or maximum is not None and path.stat().st_size > maximum:
        raise ValueError('offline input must be a regular file within its size bound: '+str(path))
    return path


def _pc(path):
    streams = {'tx': bytearray(), 'rx': bytearray()}
    calls = {'tx': [], 'rx': []}
    metadata, pairs, processing, sources = {}, [], {}, OrderedDict()
    with _regular(path/'events.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            event, at = row['event'], int(row['pc_us']) if row['pc_us'] else None
            d = json.loads(row['details_json'])
            if event == 'run_metadata':
                metadata = d
            elif event == 'measurement_clock_pair':
                pairs.append(d)
            elif event == 'transport_receive_timing' and d['type'] == int(Type.SYNC_REPLY):
                processing[(d['session'], d['type'], d['seq'])] = at
            elif event == 'rx_complete' and d.get('type') == int(Type.OUTPUT):
                payload = bytes.fromhex(d['payload_hex'])
                if payload[7] & 1 and d['ack'] != 0xffffffff:
                    metadata.setdefault('first_armed_report_pc_us', at)
            elif event == 'source_clock_bounds':
                metadata.setdefault('sensor_stream_start_pc_us', at)
                sources[d['source_us']] = (at, d)
                if len(sources) > 256:
                    sources.popitem(last=False)
            elif event in ('io_read_timing', 'io_write_timing'):
                direction = 'rx' if event == 'io_read_timing' else 'tx'
                if d['stream_end']-d['stream_start'] != d['bytes']:
                    raise ValueError('invalid I/O stream range')
                d['pc_us'] = at
                if calls[direction] and d['stream_start'] != calls[direction][-1]['stream_end']:
                    raise ValueError('noncontiguous I/O stream ranges')
                calls[direction].append(d)
            elif event == 'raw_pc_write':
                streams['tx'].extend(bytes.fromhex(d['packet_hex']))
            elif event in ('raw_rx', 'raw_rx_after_disarm'):
                streams['rx'].extend(bytes.fromhex(d['packet_hex']))
    return streams, calls, metadata, pairs, processing, sources


def correlate(path, *, pcap=None, usb_bus=None, usb_device=None, out_ep=None, in_ep=None,
              stopped=None):
    path = Path(path)
    summary = json.loads(_regular(path/'summary.json', 4_000_000).read_text())
    streams, calls, metadata, pairs, processing, sources = _pc(path)
    from .gate1_trial import labels
    trial = summary.get('trial_clock_400_sync20') is True
    if trial and summary['verdict'] not in ('FAIL', 'INCOMPLETE', 'TRIAL_COMPLETE'):
        raise ValueError('TRIAL cannot contain PASS; NOT GATE 1 PASS')
    result = {**labels(trial), 'clock_halfwidth_limit_us': 400 if trial else 250,
              'sync_max_period_us': 20000 if trial else 100000,
              'schema': 1, 'physical_gate1_pass': False,
              'original_pc_verdict': summary['verdict'], 'original_pc_failure': summary.get('failure'),
              'pc_clock_domain': metadata.get('pc_clock_domain', 'UNKNOWN'),
              'measurement_verdict': 'FAIL' if summary['verdict'] == 'FAIL' else 'INCOMPLETE',
              'capture_drop_count': 'UNKNOWN: retain and inspect tcpdump stderr',
              'urb_layer': 'UNKNOWN', 'urb_reason': 'optional usbmon capture not supplied',
              'clock_mapping': 'CONDITIONAL: REALTIME has no unobserved backward steps; interval only',
              'observer_overhead': 'unmeasured; compare with uninstrumented baseline',
              'sync': [], 'packets': [], 'diagnostic': 'INCOMPLETE'}
    rx1, dump = None, None
    if stopped is not None:
        try:
            data = _regular(stopped, 4_000_000).read_bytes()
            dump = decode_dump(data)
            result['stopped_d1_reason'] = dump.context[20]
            result['measured_device_fault'] = (dump.context[20] >= 0x100 or
                                               any(dump.counts[i] for i in (11, 13, 14, 15, 16)))
            if result['measured_device_fault']:
                result['measurement_verdict'] = 'FAIL'
            rx1 = parse_rx1(data, dump)
            identity = summary.get('identity', {})
            if identity.get('boot_nonce') != dump.boot_nonce or identity.get('host_session') != dump.session:
                raise ValueError('PC/stopped D1 identity mismatch')
            stack = re.search(rb'\[PILG1\] stopped; reset required; worker stack untouched ([0-9]+)/2048 bytes\r?\n',
                              data[rx1['end_offset']:])
            result['stopped_stack_line_complete'] = bool(stack and int(stack[1]) <= 2048)
            result['diagnostic'] = ('COMPLETE' if rx1['d1_identity_verified'] and result['stopped_stack_line_complete']
                                    else 'INCOMPLETE')
            result['rx1'] = {'header': rx1['header'], 'd1_identity_verified': rx1['d1_identity_verified'],
                             'post_lost': rx1['header'][7], 'scope': 'bounded partial trace, not all packets'}
        except ValueError as error:
            result['diagnostic_error'] = str(error)
            rx1 = None
    wanted = {(p[8] & 255, p[6], p[7]) for p in rx1['packets']} if rx1 else set()
    tx = [p for p in _frames(streams['tx'], (Type.SYNC_REQ, Type.SENSOR))
          if p['frame'].type == Type.SYNC_REQ or
          (int(p['frame'].type), p['frame'].sequence, p['frame'].source_us) in wanted]
    rx = list(_frames(streams['rx'], (Type.SYNC_REPLY,)))
    for direction, frames in (('tx', tx), ('rx', rx)):
        call_ends = [call['stream_end'] for call in calls[direction]]
        for packet in frames:
            packet['call'] = _containing(calls[direction], packet['end'], call_ends)
            packet['first_call'] = _containing(calls[direction], packet['start']+1, call_ends)
            packet['urbs'] = []
    clock = ClockBrackets(pairs)
    if pcap is not None:
        try:
            if (usb_bus is None or usb_device is None or out_ep is None or in_ep is None or
                    not 1 <= usb_bus <= 65535 or not 1 <= usb_device <= 127 or
                    not 1 <= out_ep <= 15 or not 0x81 <= in_ep <= 0x8f):
                raise ValueError('explicit valid Probe bus/device/CDC endpoints required')
            matchers = {direction: USBStreamMatcher(data) for direction, data in streams.items()}
            pending, selected, incomplete, errors, bad_clock = {}, 0, 0, 0, False
            frames_by_dir = {'tx': tx, 'rx': rx}
            ends = {d: [p['end'] for p in ps] for d, ps in frames_by_dir.items()}
            with _regular(pcap).open('rb') as stream:
                for urb in read_usb_pcap(stream):
                    if (urb['bus'], urb['device']) != (usb_bus, usb_device):
                        continue
                    ep, kind = urb['endpoint'], urb['event']
                    if ep not in (out_ep, in_ep):
                        continue
                    selected += 1
                    if urb['pcap_us'] != urb['usb_realtime_us']:
                        bad_clock = True
                    key = (urb['tag'], ep)
                    if kind == 'S':
                        if key in pending:
                            errors += 1
                        pending[key] = urb
                    else:
                        submit = pending.pop(key, None)
                        if urb['status'] != 0:
                            errors += 1
                        # Complete any matched OUT record, including reused IDs.
                        if ep == out_ep and submit is not None:
                            for part in submit.get('parts', []):
                                part['complete_realtime_us'] = urb['usb_realtime_us']
                                part['status'] = urb['status']
                                part['completed_length'] = urb['length']
                                part['completion_full_length'] = urb['length'] == submit['length']
                    if not (kind == 'S' and ep == out_ep or kind == 'C' and ep == in_ep):
                        continue
                    direction = 'tx' if ep == out_ep else 'rx'
                    if urb['length'] == 0:
                        continue  # ZLP contains no stream bytes; keep alignment.
                    payload = urb['payload']
                    if not urb['payload_complete'] or not payload or (kind == 'C' and urb['status']):
                        incomplete += 1
                        matchers[direction].finish()
                        continue
                    part = {'tag': urb['tag'],
                            'submit_realtime_us': (urb['usb_realtime_us'] if kind == 'S' else
                                                   submit['usb_realtime_us'] if submit else None),
                            'complete_realtime_us': urb['usb_realtime_us'] if kind == 'C' else None,
                            'status': urb['status'] if kind == 'C' else None,
                            'completion_full_length': kind == 'C',
                            'completed_length': urb['length'] if kind == 'C' else None}
                    urb['parts'] = [part]
                    for recovered in matchers[direction].feed(payload, part):
                        index = bisect_right(ends[direction], recovered['stream_start'])
                        for position in range(index, len(frames_by_dir[direction])):
                            packet = frames_by_dir[direction][position]
                            if packet['start'] >= recovered['stream_end']:
                                break
                            packet['urbs'].append(recovered)
            for matcher in matchers.values():
                matcher.finish()
                incomplete += matcher.unmatched
            if not selected:
                raise ValueError('no bulk URBs match the runtime Probe selectors')
            if bad_clock or not clock.valid:
                raise ValueError('pcap/header timestamp mismatch or missing/backward clock pairs')
            result.update(urb_layer='PARTIAL', urb_reason='per-frame full-byte coverage and conditional RAW intervals below',
                          usb_events_selected=selected, usb_unmatched_or_short=incomplete, usb_errors=errors)
        except (OSError, ValueError) as error:
            result['urb_reason'] = 'URB layer unknown: ' + str(error)
            for packet in tx + rx:
                packet['urbs'] = []

    def urb_summary(packet):
        parts = packet['urbs']
        cursor = packet['start']
        for part in sorted(parts, key=lambda p: p['stream_start']):
            if part['stream_start'] > cursor:
                break
            cursor = max(cursor, part['stream_end'])
        covered = cursor >= packet['end']
        completed = covered and all(p['status'] == 0 and p['completion_full_length'] for p in parts)
        output = {'byte_exact_coverage': covered, 'usb_completion_success': completed, 'parts': parts}
        for label, part in (('submit', parts[0] if parts else None),
                            ('complete', parts[-1] if parts else None)):
            stamp = part.get(label+'_realtime_us') if part and covered else None
            if label == 'complete' and not completed:
                stamp = None
            output[label+'_raw_interval_us'] = clock.map(stamp) if stamp is not None else None
        return output

    replies = {(p['frame'].session, p['frame'].ack): p for p in rx}
    sent = {(int(p['frame'].type), p['frame'].sequence, p['frame'].source_us): p for p in tx}
    for packet in tx:
        frame, call = packet['frame'], packet['call']
        if frame.type != Type.SYNC_REQ:
            continue
        reply = replies.get((frame.session, frame.sequence))
        if reply is None or call is None or reply['call'] is None:
            continue
        rf, rc = reply['frame'], reply['call']
        t1 = struct.unpack_from('<Q', frame.payload)[0]
        if struct.unpack_from('<Q', rf.payload)[0] != t1:
            continue
        t2 = struct.unpack_from('<I', rf.payload, 8)[0]
        residence = (rf.source_us - t2) & 0xffffffff
        if residence >= 0x80000000:
            result['invalid_sync_native_interval'] = result.get('invalid_sync_native_interval', 0)+1
            continue
        t4 = rc.get('read_return_pc_us', rc['pc_us'])
        row = {'seq': frame.sequence, 't1_raw_us': t1,
               'first_write_enter_raw_us': packet['first_call']['call_enter_pc_us'],
               'write_enter_raw_us': call['call_enter_pc_us'], 'write_return_raw_us': call['pc_us'],
               't4_raw_us': t4, 'rtt_us': t4-t1, 't2_pico_us': t2, 't3_pico_us': rf.source_us,
               'pico_residence_us': residence,
               't1_to_write_return_us': call['pc_us']-t1,
               'write_call_us': call['pc_us']-call['call_enter_pc_us'],
               'read_timing': rc, 'tx_urb': urb_summary(packet), 'rx_urb': urb_summary(reply)}
        done = processing.get((rf.session, int(rf.type), rf.sequence))
        row['read_to_transport_return_us'] = done-t4 if done is not None else None
        row['traffic_phase'] = ('ARMED_REPORT_SEEN' if metadata.get('first_armed_report_pc_us', t1+1) <= t1
                                else 'SENSOR_STREAM_PENDING' if metadata.get('sensor_stream_start_pc_us', t1+1) <= t1
                                else 'BOOTSTRAP')
        # Conditional conversion intervals, NEVER subtraction of clock midpoints.
        submit = row['tx_urb']['submit_raw_interval_us']
        complete = row['rx_urb']['complete_raw_interval_us']
        row['write_enter_to_usb_submit_us'] = ([submit[0]-call['call_enter_pc_us'],
                                               submit[1]-call['call_enter_pc_us']] if submit else None)
        row['usb_complete_to_read_return_us'] = ([t4-complete[1], t4-complete[0]] if complete else None)
        result['sync'].append(row)
    if rx1:
        clock_proof = (rx1['d1_identity_verified'] and metadata.get('pc_clock_domain') == 'CLOCK_MONOTONIC_RAW'
                       and (dump.context[4], dump.context[6], dump.context[7]) == (125000000, 8, 31)
                       and all(dump.counts[i] == 0 for i in (11, 13, 14, 15, 16)))
        result['cross_clock_bounds_condition'] = 'same100ppm premise; RAW and matching D1 baud/UART proof; not acceptance'
        result['cross_clock_bounds_eligible'] = clock_proof
        result['cross_clock_bounds_units'] = 'native Pico TIMER microseconds, not exact RAW duration'
        for p in rx1['packets']:
            packet = sent.get((p[8] & 255, p[6], p[7]))
            call = packet['call'] if packet else None
            row = {'serial': p[0], 'seq': p[6], 'source_us': p[7], 'type': p[8] & 255,
                   'rx_pico_us': p[1], 'rx_to_worker_us': elapsed(p[2], p[1]),
                   'decode_us': elapsed(p[3], p[2]), 'dispatch_wait_us': elapsed(p[4], p[3]),
                   'dispatch_us': elapsed(p[5], p[4]), 'reason': p[8] >> 16,
                   'post_rx': bool(p[9] & (1 << 24)), 'post_dispatch': bool(p[9] & (2 << 24)),
                   'decode_result': (p[8] >> 8) & 255, 'decode_pending': p[3] == 0xffffffff,
                   'rx_result': (p[9] >> 20) & 15,
                   'rx_depth': (p[9] >> 8) & 15, 'worker_depth': (p[9] >> 12) & 15,
                   'post_drop_depth': (p[9] >> 16) & 15,
                   'pc_write_return_raw_us': call['pc_us'] if call else None,
                   'tx_urb': urb_summary(packet) if packet else None}
            row['relative_first_stop_us'] = {}
            for name, stamp in zip(('rx', 'worker', 'decoded', 'dispatch_enter', 'dispatch_done'), p[1:6]):
                delta = ((stamp-rx1['header'][2]+0x80000000) & 0xffffffff)-0x80000000
                row['relative_first_stop_us'][name] = (None if stamp == 0xffffffff or delta == -0x80000000
                                                       else delta)
            row['pc_write_return_to_pico_rx_interval_us'] = None
            capture = sources.get(p[7]) if (p[8] & 255) == int(Type.SENSOR) else None
            if clock_proof and call and capture and p[1] != 0xffffffff:
                at, bounds = capture
                low_dt = call['pc_us'] - at
                high_dt = call['pc_us'] - bounds['capture_end_pc_us']
                if min(low_dt, high_dt) >= 0:
                    low = bounds['lower_pico_us'] + low_dt - (low_dt+9999)//10000 - 1
                    high = bounds['upper_pico_us'] + high_dt + (high_dt+9999)//10000 + 1
                    native_rx = unwrap32(p[1], (low+high)//2)
                    row['pc_write_return_to_pico_rx_interval_us'] = [native_rx-high, native_rx-low]
            # Midpoint above selects a wrap epoch ONLY; no midpoint delay is reported.
            result['packets'].append(row)
        result['alarms'] = [{'words': a, 'demand_pico_us': a[0], 'entry_pico_us': a[1],
                             'next_apply_before': a[2], 'current_ready': bool(a[3] & 1),
                             'lookahead_ready': bool(a[3] & 2), 'reason_after': a[4],
                             'done_pico_us': a[5], 'irq_work_us': elapsed(a[5], a[1])}
                            for a in rx1['alarms']]
    result['sync_summary_us'] = {}
    for phase in ('ALL', 'BOOTSTRAP', 'SENSOR_STREAM_PENDING', 'ARMED_REPORT_SEEN'):
        values = sorted(row['rtt_us'] for row in result['sync'] if phase == 'ALL' or row['traffic_phase'] == phase)
        result['sync_summary_us'][phase] = ({'count': len(values), 'p50': values[(len(values)-1)//2],
                                           'p99': values[(99*len(values)+99)//100-1], 'max': values[-1]}
                                          if values else {'count': 0})
    return result
