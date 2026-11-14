"""Strict stopped RX1 diagnostic reader; never changes a Gate1 verdict.

Schema: SW6 docs/simulation/PIL_GATE1_RX_TIMING.md. No serial access.
"""
import re
import struct
from .wire import crc32c


class RXTimingError(ValueError):
    pass


def elapsed(later, earlier):
    if 0xffffffff in (later, earlier):
        return None
    value = (later - earlier) & 0xffffffff
    if value >= 0x80000000:
        raise RXTimingError('RX1 backwards/ambiguous TIMER interval')
    return value


def parse_rx1(data, dump=None):
    if not isinstance(data, bytes) or len(data) > 4_000_000:
        raise RXTimingError('RX1 input exceeds4MB/type bound')
    marker = b'[PILG1RX1 BEGIN '
    if data.count(marker) != 1:
        raise RXTimingError('missing/duplicate RX1 BEGIN')
    tail = data[data.index(marker):]
    rows, consumed = [], 0
    for line in tail.splitlines(keepends=True):
        consumed += len(line)
        if not line.endswith(b'\n'):
            raise RXTimingError('truncated RX1 line')
        match = re.fullmatch(rb'\[PILG1RX1 (BEGIN|P|A|END)((?: [0-9a-f]{8})+)\]\n', line)
        if match is None:
            raise RXTimingError('invalid RX1 row/version/hex width')
        kind = match[1].decode('ascii')
        words = tuple(int(word, 16) for word in match[2].split())
        if len(words) != {'BEGIN': 8, 'P': 10, 'A': 6, 'END': 3}[kind]:
            raise RXTimingError('RX1 row word count')
        rows.append((kind, words))
        if kind == 'END':
            break
    if not rows or rows[-1][0] != 'END' or b'[PILG1RX1 ' in tail[consumed:]:
        raise RXTimingError('missing footer or extra/repeated RX1 rows')
    header, footer = rows[0][1], rows[-1][1]
    pre, post, alarms = header[4:7]
    if pre > 64 or post > 16 or alarms > 16:
        raise RXTimingError('RX1 count bound')
    expected = ['BEGIN'] + ['P'] * (pre + post) + ['A'] * alarms + ['END']
    if [kind for kind, _ in rows] != expected or footer[:2] != (pre + post, alarms):
        raise RXTimingError('RX1 order/count/footer mismatch')
    canonical = b''.join(struct.pack('<' + 'I' * len(words), *words) for _, words in rows[:-1])
    if crc32c(canonical) != footer[2]:
        raise RXTimingError('RX1 CRC32C mismatch')
    packets = [words for kind, words in rows if kind == 'P']
    alarm_rows = [words for kind, words in rows if kind == 'A']
    for index, p in enumerate(packets):
        if index and ((p[0] - packets[index-1][0]) & 0xffffffff) != 1:
            raise RXTimingError('RX1 packet serial gap/duplicate/order')
        flags, aux, meta = p[9] >> 24, p[9], p[8]
        if flags & ~3 or bool(flags & 1) != (index >= pre):
            raise RXTimingError('RX1 reserved/wrong pre/post flags')
        if (aux >> 20) & 15 not in (1, 2) or not 1 <= (aux & 255) <= 130:
            raise RXTimingError('RX1 length/RX result')
        if any((aux >> shift) & 15 not in (*range(9), 15) for shift in (8, 12, 16)):
            raise RXTimingError('RX1 depth outside queue bound')
        result, kind = (meta >> 8) & 255, meta & 255
        if result not in (0, 255, 254, 253) or kind not in (*range(9), 255):
            raise RXTimingError('RX1 decode result/type')
        if p[3] == 0xffffffff:
            if result != 255 or kind != 255 or p[6:8] != (0xffffffff, 0xffffffff):
                raise RXTimingError('RX1 pending decode fields')
        elif result != 0 and (kind != 255 or p[6:8] != (0xffffffff, 0xffffffff)):
            raise RXTimingError('RX1 failed decode invented identity')
        for earlier, later in zip(p[1:5], p[2:6]):
            elapsed(later, earlier)
    for index, a in enumerate(alarm_rows):
        if a[3] & ~3:
            raise RXTimingError('RX1 reserved readiness bits')
        elapsed(a[5], a[1])
        if index:
            elapsed(a[0], alarm_rows[index-1][0])
    verified = False
    if dump is not None:
        from .gate1_diagnostics import decode_dump
        try:
            preceding = decode_dump(data[:data.index(marker)])
        except ValueError as error:
            raise RXTimingError('RX1 must follow the complete D1 manifest') from error
        if preceding != dump:
            raise RXTimingError('RX1 preceding D1 differs from supplied snapshot')
        if header[:2] != (dump.boot_nonce, dump.session) or header[3] != dump.context[20]:
            raise RXTimingError('RX1/D1 identity or stop reason mismatch')
        stops = [row[4] for row in dump.trace if row[0] in (4, 5) and row[15] == header[3]]
        if stops and header[2] not in stops:
            raise RXTimingError('RX1/D1 first stop time mismatch')
        verified = bool(stops)
    return {'header': header, 'packets': packets, 'alarms': alarm_rows,
            'crc32c': footer[2], 'd1_identity_verified': verified,
            'end_offset': data.index(marker)+consumed,
            'scope': 'stopped diagnostic only; not successful staging or acceptance'}
