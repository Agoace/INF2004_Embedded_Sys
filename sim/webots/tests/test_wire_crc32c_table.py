"""C4 equivalence against the original bitwise Castagnoli implementation."""
import json
from pathlib import Path
import random
import unittest
from unittest.mock import patch
from pil import wire


def bitwise(data):
    crc=0xffffffff
    for byte in data:
        crc^=byte
        for _ in range(8):crc=(crc>>1)^(0x82f63b78 if crc&1 else 0)
    return crc^0xffffffff


class CRCTableTests(unittest.TestCase):
    def test_precomputed_table_and_known_check(self):
        self.assertEqual(len(wire.CRC32C_TABLE),256)
        self.assertIsInstance(wire.CRC32C_TABLE,tuple)
        self.assertEqual(wire.crc32c(b'123456789'),0xe3069283)

    def test_random_and_all_fixture_contents_and_hex_fields(self):
        rng=random.Random(20261115)
        blobs=[bytes(rng.randrange(256)for _ in range(n))for n in list(range(257))+[1024,4096]]
        def visit(value):
            if isinstance(value,dict):
                for key,item in value.items():
                    if key.endswith('_hex')and isinstance(item,str):
                        try:blobs.append(bytes.fromhex(item))
                        except ValueError:pass
                    visit(item)
            elif isinstance(value,list):
                for item in value:visit(item)
        for path in sorted((Path(__file__).parent/'fixtures').iterdir()):
            if path.is_file():
                blobs.append(path.read_bytes())
                if path.suffix=='.json':visit(json.loads(path.read_text()))
        for data in blobs:self.assertEqual(wire.crc32c(data),bitwise(data))

    def test_valid_and_malformed_frame_outcomes_identical(self):
        vectors=json.loads((Path(__file__).parent/'fixtures/pil_wire_v1.json').read_text())['vectors']
        def outcome(packet):
            try:return wire.decode(packet)
            except wire.ProtocolError as e:return (type(e).__name__,str(e))
        for vector in vectors:
            packet=bytes.fromhex(vector['wire_hex']);packets=[packet]
            packets += [packet[:n]for n in range(len(packet))]
            for offset in range(len(packet)):
                for bit in range(8):
                    changed=bytearray(packet);changed[offset]^=1<<bit;packets.append(bytes(changed))
            for candidate in packets:
                actual=outcome(candidate)
                with patch('pil.wire.crc32c',bitwise):expected=outcome(candidate)
                self.assertEqual(actual,expected)
