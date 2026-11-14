"""Dedicated stopped-USB D1 reader. Frozen PIL UART v1 limits never change.

Device extrema cover every event; optional traces are partial diagnostics only.
No device arrival/apply percentile is manufactured from extrema or OUTPUT ACKs.
"""

from dataclasses import dataclass
import csv
import json
from pathlib import Path
import struct

from .wire import NO_ACK, SCHEMA_ID, crc32c
from .clock import GATE1_PROVEN_MIN_TX_US, GATE1_PROVEN_MIN_RX_US

HEADER = struct.Struct('<IHHIIII')
LENGTHS = {1: 104, 2: 104, 3: 48, 4: 64, 5: 16}


class DiagnosticError(ValueError):
    pass


def signed_delta(value, reference):
    delta = (value - reference) & 0xffffffff
    if delta == 0x80000000:
        raise DiagnosticError('ambiguous half-range timestamp')
    return delta if delta < 0x80000000 else delta - 0x100000000


def signed(value):
    return value if value < 0x80000000 else value - 0x100000000


def uncobs(segment):
    if not 1 <= len(segment) <= 133:
        raise DiagnosticError('D1 encoded record size')
    output, cursor = bytearray(), 0
    while cursor < len(segment):
        code = segment[cursor]
        cursor += 1
        if not code or cursor + code - 1 > len(segment):
            raise DiagnosticError('D1 COBS length')
        output.extend(segment[cursor:cursor + code - 1])
        cursor += code - 1
        if code != 255 and cursor < len(segment):
            output.append(0)
        if len(output) > 132:
            raise DiagnosticError('D1 decoded record size')
    return bytes(output)


@dataclass(frozen=True)
class Dump:
    boot_nonce: int
    session: int
    context: tuple
    counts: tuple
    metrics: tuple
    trace: tuple


def decode_dump(data):
    """Accept identical complete repeats, never splice incomplete repetitions.

    Ordinary printable console lines may occur between atomic binary records.
    Unknown binary records, truncated D1 records and changed snapshots reject.
    A bounded input prevents unbounded console-noise allocation.
    """
    if not isinstance(data, bytes) or len(data) > 4_000_000:
        raise DiagnosticError('D1 input must be bytes within4MB')
    current, chain, complete = [], bytearray(), None
    for segment in data.split(b'\0'):
        if not segment:
            continue
        if all(32 <= byte <= 126 or byte in (9, 10, 13) for byte in segment):
            continue
        body = uncobs(segment)
        if len(body) < 28:
            raise DiagnosticError('D1 short record')
        magic, version, kind, seq, boot, session, length = HEADER.unpack_from(body)
        if (magic != 0x31444750 or version != 1 or kind not in LENGTHS or
                length != LENGTHS[kind] or len(body) != 24 + length + 4):
            raise DiagnosticError('D1 magic/version/kind/identity/length')
        if struct.unpack_from('<I', body, len(body)-4)[0] != crc32c(body[:-4]):
            raise DiagnosticError('D1 CRC32C mismatch')
        if seq != len(current):
            raise DiagnosticError('D1 record gap/duplicate/reorder')
        if current and (boot, session) != current[0][:2]:
            raise DiagnosticError('D1 changed identity')
        words = struct.unpack_from('<' + 'I' * (length // 4), body, 24)
        if seq == 0 and kind != 1 or seq == 1 and kind != 2:
            raise DiagnosticError('D1 context/count order')
        if 2 <= seq <= 13 and (kind != 3 or words[0] != seq - 1):
            raise DiagnosticError('D1 metric order/ID')
        if current:
            trace_count = current[0][3][25]
            expected = 3 if seq <= 13 else 4 if seq < 14 + trace_count else 5
            if seq >= 2 and kind != expected:
                raise DiagnosticError('D1 trace/manifest order')
        if kind == 1:
            if words[24] != 12 or words[25] > 64:
                raise DiagnosticError('D1 metric/trace count')
            if words[22] & ~0x7ff or words[23] & ~0x7f or words[21] & ~0x1f:
                raise DiagnosticError('D1 reserved context flags')
            if words[20] not in range(9) and words[20] not in range(0x100, 0x110):
                raise DiagnosticError('D1 unknown stop reason')
        if kind == 4:
            if (not 1 <= words[0] <= 10 or words[9] > 8 or words[10] & ~0x1f or
                    words[15] not in range(9) and words[15] not in range(0x100, 0x110)):
                raise DiagnosticError('D1 unknown trace event/flags/reason/depth')
        if kind == 5:
            if words != (len(current), 12, current[0][3][25], crc32c(chain)):
                raise DiagnosticError('D1 manifest count/chain mismatch')
            dump = Dump(boot, session, current[0][3], current[1][3],
                        tuple(row[3] for row in current[2:14]),
                        tuple(row[3] for row in current[14:]))
            if complete is not None and dump != complete:
                raise DiagnosticError('D1 changed repeated snapshot')
            complete, current, chain = dump, [], bytearray()
        else:
            current.append((boot, session, kind, words))
            chain.extend(body[:-4])
    if complete is None or current:
        raise DiagnosticError('D1 truncated/missing manifest')
    return complete


def _coverage(dump):
    c, n = dump.context, dump.counts
    if not dump.boot_nonce or not dump.session:
        raise DiagnosticError('D1 no negotiated nonzero identity')
    if (c[0] != SCHEMA_ID or c[1] != 4 or c[5] != 921600 or
            tuple(c[8:12]) != (1000, 4000, 5000, 8)):
        raise DiagnosticError('D1 schema/capability/timing constants')
    if c[22] & ~0x7ff or c[23] & ~0x7f:
        raise DiagnosticError('D1 reserved flags')
    if (c[23] & 0x79) != 0x79 or c[23] & 4 and not c[23] & 2:
        raise DiagnosticError('D1 transport-only profile flags')
    if signed_delta(c[2], c[3]) % 1000:
        raise DiagnosticError('D1 epoch not on native grid')
    if not c[4] or not c[6] or c[7] >= 64:
        raise DiagnosticError('D1 UART divisor/clock missing')
    baud = c[4] / (16 * (c[6] + c[7] / 64))
    if not .98 * 921600 <= baud <= 1.02 * 921600:
        raise DiagnosticError('D1 UART divisor outside candidate baud range')
    staged, applied, demanded = n[:3]
    if not 1 <= applied <= staged <= 1_800_000 or n[23] > 8:
        raise DiagnosticError('D1 stage/apply range or cancelled tail')
    if staged != applied + n[23] or demanded != applied:
        raise DiagnosticError('D1 cohort coverage equations')
    for count, first, last in ((staged, c[12], c[13]), (applied, c[14], c[15]),
                               (demanded, c[16], c[17])):
        if first != 0 or last != count-1:
            raise DiagnosticError('D1 noncontiguous cohort prefix')
    if tuple(n[3:6]) != (applied, applied, applied):
        raise DiagnosticError('D1 output cohort not completely drained')
    expected_counts = {1: staged, 2: applied, 3: staged, 4: applied, 5: demanded,
                       6: applied, 7: staged, 8: applied, 9: n[6], 10: n[7]}
    for row in dump.metrics:
        metric, count = row[:2]
        if metric in expected_counts and count != expected_counts[metric]:
            raise DiagnosticError('D1 incomplete all-event metric coverage')
        if count == 0:
            if row[2:] != (0, NO_ACK, 0, 0, 0, 0, NO_ACK, 0, 0, 0):
                raise DiagnosticError('D1 invalid empty metric representation')
            raise DiagnosticError('D1 required metric has no observations')
        low, high = signed(row[2]), signed(row[7])
        if low > high:
            raise DiagnosticError('D1 reversed extrema')
        for side, (value, arg, raw_lo, raw_hi, reference) in enumerate((row[2:7], row[7:12])):
            value = signed(value)
            if metric <= 8 and arg >= (staged if metric in (1, 3, 7) else applied):
                raise DiagnosticError('D1 extremum outside cohort')
            if metric >= 9 and arg != NO_ACK:
                raise DiagnosticError('D1 non-slot metric invents source sequence')
            if metric in (1, 2, 3, 4, 5):
                expected_ref = (c[2] + arg * 1000 -
                                (4000 if metric in (1, 2) else 1000 if metric == 3 else 0))
                if reference != expected_ref & 0xffffffff:
                    raise DiagnosticError('D1 extremum reference not its source/slot')
            if metric >= 7 and reference != 0:
                raise DiagnosticError('D1 non-reference metric reference must be0')
            if metric in (7, 8, 9, 10):
                if not 0 <= value <= 8 or raw_lo != raw_hi:
                    raise DiagnosticError('D1 queue depth/range')
            else:
                if signed_delta(raw_hi, raw_lo) < 0:
                    raise DiagnosticError('D1 reversed capture bracket')
                expected = signed_delta(raw_hi, raw_lo)
                is_min = side == 0
                if metric == 3:
                    expected = signed_delta(reference, raw_hi if is_min else raw_lo)
                elif metric < 7:
                    expected = signed_delta(raw_lo if is_min else raw_hi, reference)
                if value != expected:
                    raise DiagnosticError('D1 extremum/raw-reference mismatch')
                if metric == 5 and raw_lo != raw_hi:
                    raise DiagnosticError('D1 first-C-entry is one TIMER read')
    return staged, applied


def evaluate(dump, pc):
    """Combine matching all-frame evidence. A clean transport subset is not Gate1 PASS."""
    result = {'verdict': 'INCOMPLETE', 'physical_gate1_pass': False,
              'clock_minima_proven': False, 'clock_minima_proof_verdict': 'INCOMPLETE',
              'transport_subset_qualified': False,
              'device_availability_percentiles': None, 'device_apply_percentiles': None,
              'unmeasured': ['hardware negative campaigns and full Gate1 acceptance'],
              'candidate_bounds_us': {'nominal_pipeline': 4000, 'availability': 3000,
                                      'source_expiry': 5000, 'apply_jitter': 250}}
    if pc.get('verdict') == 'FAIL':
        result['verdict'], result['failure'] = 'FAIL', pc.get('failure', 'PC run failed')
        return result
    try:
        c, n = dump.context, dump.counts
        identity = pc['identity']
        epoch = identity['epoch_pico_us']
        if (identity['boot_nonce'] != dump.boot_nonce or
                identity['host_session'] != dump.session or epoch & 0xffffffff != c[2]):
            raise DiagnosticError('PC/D1 identity mismatch')
        if len(n) != 26 or any(type(value) is not int or not 0 <= value <= 0xffffffff for value in n):
            raise DiagnosticError('D1 UART counter evidence missing/invalid')
        # Normative COUNTS: BAD11, OVERRUN13, FRAMING14, PARITY15, BREAK16.
        uart_counts = {name: n[index] for name, index in
                       (('bad_frame', 11), ('overrun', 13), ('framing', 14), ('parity', 15), ('break', 16))}
        profile_proved = (c[4], c[6], c[7]) == (125000000, 8, 31)
        result['clock_minima_uart_counts'] = uart_counts
        result['clock_minima_proven'] = profile_proved and not any(uart_counts.values())
        if result['clock_minima_proven']:
            result['clock_minima_proof_verdict'] = 'ELIGIBLE'
        if (c[22] & 0x3c0 or any(n[8:23]) or n[24] or n[25] or
                c[20] not in (1, 3)):
            result['verdict'], result['failure'] = 'FAIL', 'Pico fault/error/negative-campaign stop'
            return result
        used_minima = (pc.get('clock_proven_min_tx_us', 0), pc.get('clock_proven_min_rx_us', 0))
        if used_minima not in ((0, 0), (GATE1_PROVEN_MIN_TX_US, GATE1_PROVEN_MIN_RX_US)):
            result['clock_minima_proven'] = False
            result['clock_minima_proof_verdict'] = 'INCOMPLETE'
            raise DiagnosticError('PC directional minima are outside the proved profile')
        staged, applied = _coverage(dump)
        if c[21] & ~0x1f:
            raise DiagnosticError('D1 reserved final state flags')
        if c[22] != 0x43f or c[21] & (1 | 16) or not c[21] & 4:
            raise DiagnosticError('D1 missing clean-stop coverage/inhibition')
        if (pc['active_output_count'] != applied or
                pc.get('active_output_prefix_complete') is not True):
            result['verdict'], result['failure'] = 'FAIL', 'PC active OUTPUT loss/ACK-prefix mismatch'
            return result
        sources = pc['sources']
        if not staged <= len(sources) <= staged + 8:
            raise DiagnosticError('PC source coverage differs from staged prefix')
        h, l = None, None
        for index, sample in enumerate(sources):
            if sample['source_us'] != index * 1000:
                raise DiagnosticError('PC source capture gap/duplicate')
            lo, hi = sample['lower_pico_us'], sample['upper_pico_us']
            if type(lo) is not int or type(hi) is not int or hi < lo:
                raise DiagnosticError('PC capture interval invalid')
            if index < staged:
                baseline = epoch + index * 1000 - 4000
                h = max(h if h is not None else baseline-lo, baseline-lo)
                l = max(l if l is not None else hi-baseline, hi-baseline)
        metrics = {row[0]: row for row in dump.metrics}
        # Preserve raw device extrema; +1us upper quantization is applied ONCE.
        a, p, slack, jitter, native = (metrics[k] for k in (1, 2, 3, 4, 5))
        result.update(H_us=h, L_us=l,
                      availability_age_upper_us=signed(a[7])+1+h,
                      availability_age_lower_us=signed(a[2])-l,
                      apply_age_upper_us=signed(p[7])+1+h,
                      apply_age_lower_us=signed(p[2])-l,
                      endpoint_slack_lower_us=signed(slack[2])-1,
                      apply_jitter_lower_us=signed(jitter[2]),
                      apply_jitter_upper_us=signed(jitter[7])+1,
                      native_first_C_entry_jitter_lower_us=signed(native[2]),
                      native_first_C_entry_jitter_upper_us=signed(native[7])+1)
        failures = []
        for field, maximum in (('availability_age_upper_us', 3000),
                               ('apply_age_upper_us', 5000),
                               ('apply_jitter_upper_us', 250),
                               ('native_first_C_entry_jitter_upper_us', 250)):
            if result[field] > maximum:
                failures.append(field)
        for field in ('availability_age_lower_us', 'apply_age_lower_us',
                      'endpoint_slack_lower_us', 'apply_jitter_lower_us',
                      'native_first_C_entry_jitter_lower_us'):
            if result[field] < 0:
                failures.append(field)
        result['ten_minute_duration_met'] = (pc.get('elapsed_us', 0) >= 600_000_000 and
                                           pc.get('ten_minute_duration_met') is True and
                                           applied >= 600000)
        image = pc.get('image_sha256')
        image_known = (isinstance(image, str) and len(image) == 64 and
                       all(c in '0123456789abcdefABCDEF' for c in image))
        result['image_identity_established'] = (image_known and pc.get('profile_flags') == c[23])
        if pc.get('profile_flags') is not None and pc['profile_flags'] != c[23]:
            failures.append('operator image/profile flags mismatch')
        result['transport_subset_qualified'] = (not failures and result['clock_minima_proven']
                                               and result['ten_minute_duration_met']
                                               and result['image_identity_established']
                                               and pc.get('stopped_status_observed') is True
                                               and pc.get('required_capabilities') == 4
                                               and pc.get('source_limits_verified') is True)
        if failures:
            result['verdict'], result['failure'] = 'FAIL', ', '.join(failures)
        elif not result['ten_minute_duration_met']:
            result['incomplete_reason'] = 'ten-minute active cohort not established'
        elif not result['transport_subset_qualified']:
            result['incomplete_reason'] = 'missing image/profile/capability, clock/source or stop evidence'
        else:
            result['incomplete_reason'] = 'transport subset only; hardware campaigns not closed'
    except (DiagnosticError, KeyError, TypeError, ValueError) as error:
        result['incomplete_reason'] = str(error)
    return result


def load_pc_evidence(directory):
    """Read retained host records; no live ports. D1 still supplies device coverage."""
    directory = Path(directory)
    if not all((directory / name).is_file() for name in ('summary.json', 'events.csv')):
        raise DiagnosticError('PC evidence must be regular files, not device paths')
    if (directory / 'summary.json').stat().st_size > 1_000_000:
        raise DiagnosticError('PC summary exceeds1MB')
    summary = json.loads((directory / 'summary.json').read_text())
    sources, active, active_prefix = [], 0, True
    capture_count, pending_capture, limits_verified = 0, None, True
    with (directory / 'events.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            details = json.loads(row['details_json'])
            if row['event'] == 'source_capture':
                if pending_capture is not None:
                    limits_verified = False
                capture_count += 1
                if capture_count > 1_800_000:
                    raise DiagnosticError('PC source evidence exceeds session bound')
                pending_capture = details
                limits_verified = (limits_verified and
                                   0 <= details['phase_us'] <= 250 and
                                   0 <= details['uncertainty_us'] <= 250 and
                                   details['capture_end_pc_us'] >= int(row['pc_us']))
            elif row['event'] == 'source_clock_bounds':
                limits_verified = (limits_verified and pending_capture is not None and
                                   details['source_us'] == pending_capture['source_us'] and
                                   details['lower_pico_us'] == pending_capture['source_lower_pico_us'] and
                                   details['upper_pico_us'] == pending_capture['source_upper_pico_us'])
                pending_capture = None
                sources.append(details)
                if len(sources) > 1_800_000:
                    raise DiagnosticError('PC source evidence exceeds session bound')
            elif row['event'] == 'rx_complete' and details.get('type') == 2:
                payload = bytes.fromhex(details['payload_hex'])
                if payload[7] & 1:
                    active_prefix = active_prefix and details['ack'] == active
                    active += 1
                    if active > 1_800_000:
                        raise DiagnosticError('PC OUTPUT evidence exceeds session bound')
    return {**summary, 'sources': sources, 'active_output_count': active,
            'active_output_prefix_complete': active_prefix,
            'source_limits_verified': (limits_verified and pending_capture is None and
                                       0 < capture_count == len(sources))}
