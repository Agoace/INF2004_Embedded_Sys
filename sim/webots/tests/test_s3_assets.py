import json
import random
import struct
import tempfile
from pathlib import Path
import unittest
import zlib
from sim.webots.pil import s3_assets as a
from sim.webots.pil import s3_geometry as g


def decode_png(data):
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    chunks, offset = {}, 8
    while offset < len(data):
        length = struct.unpack('>I', data[offset:offset+4])[0]
        kind = data[offset+4:offset+8]
        payload = data[offset+8:offset+8+length]
        crc = struct.unpack('>I', data[offset+8+length:offset+12+length])[0]
        assert zlib.crc32(kind+payload) == crc
        chunks[kind] = payload
        offset += length+12
    width, height, depth, colour, _, _, _ = struct.unpack('>IIBBBBB', chunks[b'IHDR'])
    assert depth == 8
    channels = 2 if colour == 4 else 1
    raw = zlib.decompress(chunks[b'IDAT'])
    stride = width*channels+1
    assert len(raw) == height*stride
    assert all(raw[i] == 0 for i in range(0, len(raw), stride))
    return width, height, colour, tuple(raw[i+1:i+stride] for i in range(0, len(raw), stride))


class AssetsTests(unittest.TestCase):
    def test_png_crc_and_unfiltered_roundtrip(self):
        for channels, rows in ((1, (b'\0\x10\xff', b'\xff\x10\0')),
                               (2, (b'\xff\0\xff\xff',))):
            width, height, colour, decoded = decode_png(a.png_bytes(rows, channels=channels))
            self.assertEqual(decoded, rows)
            self.assertEqual(width, len(rows[0])//channels)
            self.assertEqual(height, len(rows))
            self.assertEqual(colour, 4 if channels == 2 else 0)

    def test_raster_matches_shared_geometry_at_pixel_centres(self):
        rng = random.Random(32039)
        for kind in ('STRAIGHT', 'CURVE', 'CROSS', 'T', 'BLANK'):
            tile = g.tile_geometry(kind)
            rows = a.raster_rows(tile.patches, (-157.5, -157.5, 157.5, 157.5), 5)
            for _ in range(200):
                x, y = rng.randrange(63), rng.randrange(63)
                material = g.material_at(tile.patches, -157.5+(x+.5)*5, 157.5-(y+.5)*5)
                self.assertEqual(rows[y][x], a.COLOURS[material])

    def test_clear_envelope_is_not_painted_as_extra_paper(self):
        barcode = g.barcode_geometry('F', bar_height_mm=20, paper_size_mm=(285, 30))
        bounds = (-barcode.quiet_length_mm/2, -15, barcode.quiet_length_mm/2, 15)
        rows = a.raster_rows(barcode.patches, bounds, 1, transparent=True)
        self.assertEqual(rows[0][:2], b'\xff\0')
        self.assertEqual(rows[0][2*(len(rows[0])//4):][:2], b'\xff\xff')
        self.assertEqual(decode_png(a.png_bytes(rows, channels=2))[2], 4)

    def test_export_is_explicit_and_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = a.export_assets(directory, bar_height_mm=20,
                                       paper_size_mm=(285, 30), mm_per_pixel=5)
            self.assertEqual(len(manifest['assets']), 31)
            self.assertEqual(len(list(Path(directory).glob('*.png'))), 31)
            stored = json.loads((Path(directory)/'manifest.json').read_text())
            self.assertIn('not generated', stored['station'])
            self.assertEqual(decode_png((Path(directory)/'code39_F.png').read_bytes())[2], 4)

    def test_reject_invalid_shapes_sizes_and_rows(self):
        for spacing in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                a.raster_rows((), (0, 0, 1, 1), spacing)
        with self.assertRaises(ValueError):
            a.png_bytes((b'\0', b'\0\0'))
        with self.assertRaises(ValueError):
            a.raster_rows((g.PolygonPatch('tilted', g.MAT_WHITE,
                          ((0, 0, 0), (1, 0, 1), (0, 1, 0)), 0),), (0, 0, 1, 1))


if __name__ == '__main__':
    unittest.main()
