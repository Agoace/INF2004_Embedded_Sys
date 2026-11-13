import json
from pathlib import Path
import random
import struct
import unittest

from pil.wire import (Frame, NO_ACK, ProtocolError, StreamDecoder, Type, cobs_decode,
                      cobs_encode, crc32c, decode, due, encode, newer)

FIXTURE = Path(__file__).parent / "fixtures/pil_wire_v1.json"
VECTORS = json.loads(FIXTURE.read_text())["vectors"]


class WireTests(unittest.TestCase):
    def test_independent_sw6_golden_vectors(self):
        for vector in VECTORS:
            with self.subTest(name=vector["name"]):
                packet = bytes.fromhex(vector["wire_hex"])
                if not vector["accepted"]:
                    with self.assertRaises(ProtocolError):
                        decode(packet)
                    continue
                frame = decode(packet)
                self.assertEqual((frame.type, frame.session, frame.sequence, frame.source_us,
                                  frame.ack, frame.payload.hex()),
                                 tuple(vector[k] for k in
                                       ("type", "session", "sequence", "source_us", "ack",
                                        "payload_hex")))
                self.assertEqual(encode(frame), packet)
                self.assertEqual(cobs_decode(packet).hex(), vector["decoded_hex"])

    def test_crc_check(self):
        self.assertEqual(crc32c(b"123456789"), 0xE3069283)

    def test_incremental_packets_all_split_points(self):
        for vector in VECTORS:
            if not vector["accepted"]:
                continue
            packet = bytes.fromhex(vector["wire_hex"])
            for boundary in range(len(packet)):
                stream = StreamDecoder()
                first = stream.feed(packet[:boundary])
                second = stream.feed(packet[boundary:])
                self.assertEqual(len(first), 0)
                self.assertEqual(second, [decode(packet)])

    def test_concatenated_corruption_resynchronization(self):
        packet = bytes.fromhex(VECTORS[0]["wire_hex"])
        stream = StreamDecoder()
        frames = stream.feed(packet + b"\x02\xff\x00" + packet + b"\x00")
        self.assertEqual(frames, [decode(packet)] * 2)
        self.assertEqual(stream.bad_frames, 1)

    def test_oversize_cannot_hide_command(self):
        disarm = next(v for v in VECTORS if v["name"] == "disarm_stale_session")
        packet = bytes.fromhex(disarm["wire_hex"])
        stream = StreamDecoder()
        self.assertEqual(stream.feed(b"x" * 10000 + packet), [])
        self.assertEqual(stream.oversize_frames, 1)
        self.assertLessEqual(len(stream.partial), 129)
        self.assertEqual(stream.feed(packet), [decode(packet)])

    def test_every_single_bit_error_rejected(self):
        packet = bytes.fromhex(next(v["wire_hex"] for v in VECTORS
                                    if v["name"] == "sensor_zero"))
        for offset in range(len(packet)):
            for bit in range(8):
                changed = bytearray(packet)
                changed[offset] ^= 1 << bit
                with self.subTest(offset=offset, bit=bit), self.assertRaises(ProtocolError):
                    decode(bytes(changed))

    def test_cobs_random_bodies(self):
        rng = random.Random(50494)
        for size in range(129):
            for _ in range(3):
                raw = bytes(rng.randrange(256) for _ in range(size))
                self.assertEqual(cobs_decode(cobs_encode(raw)), raw)

    def test_header_range_and_immutable_payload(self):
        for field in ("session", "sequence", "source_us", "ack"):
            for bad in (-1, 1 << 32, True, 1.5):
                fields = dict(type=Type.DISARM, session=0, sequence=0, source_us=0,
                              ack=NO_ACK, payload=struct.pack("<I", 1))
                fields[field] = bad
                with self.subTest(field=field, bad=bad), self.assertRaises(ProtocolError):
                    encode(Frame(**fields))
        with self.assertRaises(ProtocolError):
            Frame(Type.DISARM, 0, 0, 0, NO_ACK, bytearray(4))

    def test_sequence_half_range_and_wrap(self):
        self.assertTrue(newer(0, 0xFFFFFFFF))
        self.assertFalse(newer(0xFFFFFFFF, 0))
        self.assertFalse(newer(0x80000000, 0))
        self.assertFalse(newer(0, 0))

    def test_imu_population_and_no_extra_new(self):
        accel = sum(due(n * 1000, 400) for n in range(1000))
        mag = sum(due(n * 1000, 75) for n in range(1000))
        self.assertEqual((accel, mag), (400, 75))
        self.assertFalse(due(1000, 400))
        self.assertTrue(due(3000, 400))
        self.assertTrue(due(14000, 75))


if __name__ == "__main__":
    unittest.main()
