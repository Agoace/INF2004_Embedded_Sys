"""Independently specified v2 bytes, malformed cases and CSV visibility."""
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import robot_wire as wire

class BarcodeV2Tests(unittest.TestCase):
    def frame(self):
        return bytes.fromhex('02010d46010002000102030405060708090a0b0c0d0e0f10bd013705')
    def test_ok_and_csv(self):
        f = wire.decode('robot/barcode', self.frame())
        self.assertEqual((f['version'], f['known'], f['letter'], f['reason_name']),
                         (2, 1, 'F', 'BW_OK'))
        self.assertEqual((f['decoded'],f['failed'],f['shadow']), (1,0,1))
        self.assertEqual((f['narrow_mm'],f['wide_mm']), (4.45,13.35))
        self.assertEqual(f['sequence'],0x04030201)
        for k in ('reason','reason_name','epoch','start_ms','narrow_mm','wide_mm','shadow','failed'):
            self.assertIn(k,wire.csv_fields())
    def test_failure(self):
        b=bytearray(self.frame());b[2]=6;b[3]=0;b[4]=101;b[6]=0
        b[24:28]=b'\xff'*4
        f=wire.decode('robot/barcode',bytes(b))
        self.assertEqual((f['failed'],f['letter'],f['reason_name']),(1,'','BP_DISABLED'))
        self.assertIsNone(f['narrow_mm']);self.assertIsNone(f['wide_mm'])
    def test_reject_malformed_and_ignore_newer(self):
        for index,value in ((1,2),(0,3)):
            b=bytearray(self.frame());b[index]=value
            self.assertEqual(wire.decode('robot/barcode',bytes(b))['known'],0)
        for index,value in ((2,0),(2,7),(2,0x8d),(3,ord('a')),(4,6),(6,3),(7,5)):
            b=bytearray(self.frame());b[index]=value
            with self.subTest(index=index,value=value),self.assertRaises(ValueError):
                wire.decode('robot/barcode',bytes(b))
        for n in (2,20,27,29):
            with self.assertRaises(ValueError):
                wire.decode('robot/barcode',(self.frame()+b'\0')[:n])
    def test_v1_remains_unchanged(self):
        b=bytearray(20);b[0]=b[1]=1
        f=wire.decode('robot/barcode',bytes(b))
        self.assertEqual((f['known'],f['version'],f['code']),(1,1,0))
if __name__=='__main__': unittest.main()
