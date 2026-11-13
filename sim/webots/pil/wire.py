"""PC byte codec for SW6's frozen PIL_PROTOCOL.md v1; no clock or I/O effects.

Wire payloads are explicit LE bytes, not native structs. Changes require matching
C codec, golden fixtures and protocol review. A valid packet does not imply a
valid session, timely arrival, model calibration or an operational Pico.
"""

from dataclasses import dataclass
from enum import IntEnum
import struct

VERSION = 1
SCHEMA_ID = 0x50494C01
CAPABILITIES = 0x1F
NO_ACK = 0xFFFFFFFF
MAX_PAYLOAD = 104
MAX_BODY = 128
MAX_PACKET = 130
MAX_SOURCE_US = 1_800_000_000
HEADER = struct.Struct("<BBHIIII")
SIZES = (16, 40, 32, 24, 28, 8, 4, 8, 16)


class Type(IntEnum):
    BOOT = 0
    SENSOR = 1
    OUTPUT = 2
    HELLO = 3
    HELLO_ACK = 4
    ARM = 5
    DISARM = 6
    SYNC_REQ = 7
    SYNC_REPLY = 8


class ProtocolError(ValueError):
    pass


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def newer(value, previous):
    delta = (value - previous) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


def u16(data, offset):
    return struct.unpack_from("<H", data, offset)[0]


def u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def i16(data, offset):
    return struct.unpack_from("<h", data, offset)[0]


def due(source_us, rate):
    return source_us == 0 or source_us * rate // 1_000_000 != (
        (source_us - 1000) * rate // 1_000_000)


def sensor_validate(payload, source_us):
    if len(payload) != 40 or not 0 <= source_us <= MAX_SOURCE_US or source_us % 1000:
        raise ProtocolError("invalid SENSOR size/time")
    valid = u16(payload, 36)
    if payload[6] & 0xFC or payload[7] & 0xC0 or valid & 0xE000:
        raise ProtocolError("reserved SENSOR bits")
    for axis in range(3):
        if u16(payload, axis * 2) > 4095 or u16(payload, 16 + axis * 2) & 15:
            raise ProtocolError("invalid ADC/accel range")
        mag = i16(payload, 22 + axis * 2)
        if not -2048 <= mag <= 2047:
            if mag != -4096 or valid & 0x140:
                raise ProtocolError("invalid magnetometer value")
    bus_fault = bool(payload[7] & 12)
    for device, rate in enumerate((400, 75)):
        fresh, good = bool(valid & (1 << (5 + device))), bool(valid & (1 << (7 + device)))
        ready = due(source_us, rate)
        if (fresh and (not good or not ready)) or (good and ready and not bus_fault and not fresh):
            raise ProtocolError("invalid IMU phase/NEW")
    sonar_valid = bool(valid & 0x200)
    if valid & 0x1000 and not sonar_valid:
        raise ProtocolError("sonar NEW without VALID")
    if sonar_valid:
        if not 1100 <= u16(payload, 34) <= 1900:
            raise ProtocolError("invalid cache servo pulse")
        mode, rise, high = payload[7] & 3, u16(payload, 28), u16(payload, 30)
        if mode in (1, 2):
            if rise or high:
                raise ProtocolError("no-rise/stuck-high descriptor has edges")
        elif not 1 <= rise <= 1999 or (mode == 0 and not 1 <= high <= 60000) or (
                mode == 3 and high):
            raise ProtocolError("invalid sonar descriptor")


def output_validate(payload):
    if len(payload) != 32:
        raise ProtocolError("invalid OUTPUT length")
    flags = payload[7]
    armed, fault, inhibited, pending = (bool(flags & bit) for bit in (1, 2, 4, 16))
    if payload[6] & 0xFC or flags & 0xE0 or payload[10] > 8 or payload[11]:
        raise ProtocolError("invalid OUTPUT reserved/queue field")
    if (armed and (fault or inhibited or pending)) or ((fault or pending) and not inhibited):
        raise ProtocolError("contradictory OUTPUT state")
    if flags & 8 and not (fault and inhibited):
        raise ProtocolError("underflow requires fault/inhibit")
    for offset in (0, 2):
        value = i16(payload, offset)
        if not -6250 <= value <= 6250 or (not armed and value):
            raise ProtocolError("invalid effective PWM")
    pulse = u16(payload, 4)
    if not 1100 <= pulse <= 1900 and not (pulse == 0 and inhibited):
        raise ProtocolError("invalid OUTPUT servo")


@dataclass(frozen=True)
class Frame:
    type: int
    session: int
    sequence: int
    source_us: int
    ack: int
    payload: bytes

    def __post_init__(self):
        if type(self.payload) is not bytes:
            raise ProtocolError("payload must be immutable bytes")

    def validate(self):
        if type(self.type) not in (int, Type) or not 0 <= self.type < len(SIZES):
            raise ProtocolError("unknown type")
        for value in (self.session, self.sequence, self.source_us, self.ack):
            if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
                raise ProtocolError("header requires uint32")
        if len(self.payload) != SIZES[self.type]:
            raise ProtocolError("payload length mismatch")
        if self.type not in (Type.BOOT, Type.DISARM) and not self.session:
            raise ProtocolError("session zero")
        p = self.payload
        if self.type == Type.BOOT:
            if (self.session or self.ack != NO_ACK or not u32(p, 0) or
                    u32(p, 4) & ~CAPABILITIES or u32(p, 8) != 921600 or
                    u32(p, 12) != SCHEMA_ID):
                raise ProtocolError("invalid BOOT")
        elif self.type == Type.SENSOR:
            if self.sequence != self.source_us // 1000:
                raise ProtocolError("SENSOR sequence must equal source step")
            sensor_validate(p, self.source_us)
        elif self.type == Type.OUTPUT:
            output_validate(p)
        elif self.type in (Type.HELLO, Type.HELLO_ACK):
            if (not u32(p, 0) or u32(p, 4) != self.session or
                    u32(p, 8) & ~CAPABILITIES or
                    tuple(u32(p, offset) for offset in (12, 16, 20)) != (1000, 4000, 5000) or
                    self.ack == NO_ACK):
                raise ProtocolError("invalid HELLO/ACK")
        elif self.type == Type.ARM:
            if not u32(p, 0) or u32(p, 4) != self.session or self.ack == NO_ACK:
                raise ProtocolError("invalid ARM")
        elif self.type == Type.DISARM:
            if not 1 <= u32(p, 0) <= 8:
                raise ProtocolError("invalid DISARM reason")
        elif self.type == Type.SYNC_REQ:
            if u32(p, 0) != self.source_us:
                raise ProtocolError("SYNC_REQ timestamp mismatch")
        elif self.type == Type.SYNC_REPLY:
            if u32(p, 12) or self.ack == NO_ACK:
                raise ProtocolError("invalid SYNC_REPLY")
        return self


def cobs_encode(body):
    if len(body) > MAX_BODY:
        raise ProtocolError("body too large")
    output = bytearray((0,))
    code_at, code = 0, 1
    for byte in body:
        if byte:
            output.append(byte)
            code += 1
        else:
            output[code_at] = code
            code_at, code = len(output), 1
            output.append(0)
    output[code_at] = code
    return bytes(output) + b"\x00"


def cobs_decode(packet):
    if not 2 <= len(packet) <= MAX_PACKET or packet[-1] != 0:
        raise ProtocolError("packet size/delimiter")
    output, cursor = bytearray(), 0
    end = len(packet) - 1
    while cursor < end:
        code = packet[cursor]
        cursor += 1
        if not code or cursor + code - 1 > end:
            raise ProtocolError("invalid COBS length")
        chunk = packet[cursor:cursor + code - 1]
        if 0 in chunk:
            raise ProtocolError("embedded delimiter")
        output.extend(chunk)
        cursor += code - 1
        if code != 255 and cursor < end:
            output.append(0)
        if len(output) > MAX_BODY:
            raise ProtocolError("decoded packet too large")
    return bytes(output)


def encode(frame: Frame) -> bytes:
    frame.validate()
    body = HEADER.pack(VERSION, frame.type, len(frame.payload), frame.session,
                       frame.sequence, frame.source_us, frame.ack) + frame.payload
    return cobs_encode(body + struct.pack("<I", crc32c(body)))


def decode(packet: bytes) -> Frame:
    body = cobs_decode(packet)
    if len(body) < 24:
        raise ProtocolError("short header")
    version, kind, length, session, sequence, source, ack = HEADER.unpack_from(body)
    if version != VERSION or length > MAX_PAYLOAD or length + 24 != len(body):
        raise ProtocolError("invalid version/length")
    if u32(body, len(body) - 4) != crc32c(body[:-4]):
        raise ProtocolError("CRC32C mismatch")
    return Frame(kind, session, sequence, source, ack, body[20:-4]).validate()


class StreamDecoder:
    """Bounded incremental decoder; discard oversize input UNTIL delimiter.

    There is no byte-offset scan for a hidden command within a damaged packet.
    """

    def __init__(self):
        self.partial = bytearray()
        self.discarding = False
        self.bad_frames = 0
        self.oversize_frames = 0

    def feed(self, data):
        frames = []
        for byte in data:
            if byte == 0:
                if not self.discarding and self.partial:
                    try:
                        frames.append(decode(bytes(self.partial) + b"\x00"))
                    except ProtocolError:
                        self.bad_frames += 1
                self.partial.clear()
                self.discarding = False
            elif not self.discarding:
                self.partial.append(byte)
                if len(self.partial) >= MAX_PACKET:
                    self.partial.clear()
                    self.discarding = True
                    self.bad_frames += 1
                    self.oversize_frames += 1
        return frames
