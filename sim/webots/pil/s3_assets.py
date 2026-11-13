"""Rasterize shared convex polygons to deterministic PNG/SVG, standard library.

PNG is for Webots visuals only. ADC sampling uses material IDs, not RGB.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import zlib
from .s3_geometry import (MAT_WHITE, MAT_LINE, PAPER_WHITE, PAPER_INK,
                          barcode_geometry, geometry_svg, tile_geometry)

COLOURS = {MAT_WHITE: 238, MAT_LINE: 17, PAPER_WHITE: 255, PAPER_INK: 17}


def raster_rows(patches, bounds_mm, mm_per_pixel=.1, *, transparent=False):
    """WorldXY +Y at image top; optional (luminance, alpha) pixel pairs.

    Resolution is a maximum texel spacing. Actual spacings fit bounds exactly.
    Supply only flat asset patches; 3D occluders stay in the renderer/scene.
    """
    x0, y0, x1, y1 = bounds_mm
    if not all(math.isfinite(x) for x in (*bounds_mm, mm_per_pixel)):
        raise ValueError('nonfinite raster bounds')
    if mm_per_pixel <= 0 or x1 <= x0 or y1 <= y0:
        raise ValueError('invalid raster bounds/resolution')
    width = math.ceil((x1-x0)/mm_per_pixel)
    height = math.ceil((y1-y0)/mm_per_pixel)
    if width*height > 50_000_000:
        raise ValueError('raster size exceeds reference-tool bound')
    dx, dy = (x1-x0)/width, (y1-y0)/height
    channels = 2 if transparent else 1
    patches = tuple(patches)
    if any(any(abs(v[2]-p.vertices_mm[0][2]) > 1e-7 for v in p.vertices_mm)
           for p in patches):
        raise ValueError('raster_rows accepts flat asset patches only')
    ordered = sorted(patches, key=lambda p: (p.vertices_mm[0][2], p.layer,
                                           p.surface_id))
    background = b'\xff\x00' if transparent else b'\xff'
    rows = [bytearray(background*width) for _ in range(height)]
    for patch in ordered:
        vertices = patch.vertices_mm
        first = max(0, math.floor((y1-max(p[1] for p in vertices))/dy))
        last = min(height, math.ceil((y1-min(p[1] for p in vertices))/dy))
        for row in range(first, last):
            y = y1-(row+.5)*dy
            crossings = []
            for a, b in zip(vertices, vertices[1:]+vertices[:1]):
                if (a[1] > y) != (b[1] > y):
                    crossings.append(a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
            if not crossings:
                continue
            lo = max(0, math.ceil((min(crossings)-x0)/dx-.5))
            hi = min(width, math.ceil((max(crossings)-x0)/dx-.5))
            if hi > lo:
                pixel = (bytes([COLOURS[patch.material_id], 255]) if transparent
                         else bytes([COLOURS[patch.material_id]]))
                rows[row][lo*channels:hi*channels] = pixel*(hi-lo)
    return tuple(bytes(row) for row in rows)


def png_bytes(rows, *, channels=1):
    """Encode grayscale (channels=1) or grayscale+alpha (channels=2) PNG."""
    if channels not in (1, 2):
        raise ValueError('PNG supports grayscale or grayscale+alpha here')
    if (not rows or not rows[0] or len(rows[0]) % channels or
            any(len(r) != len(rows[0]) for r in rows)):
        raise ValueError('rectangular nonempty rows required')
    def chunk(kind, payload):
        return (struct.pack('>I', len(payload)) + kind + payload +
                struct.pack('>I', zlib.crc32(kind+payload)))
    header = struct.pack('>IIBBBBB', len(rows[0])//channels, len(rows),
                         8, 4 if channels == 2 else 0, 0, 0, 0)
    data = zlib.compress(b''.join(b'\0'+r for r in rows), 9)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) +
            chunk(b'IDAT', data) + chunk(b'IEND', b''))


def export_assets(output, *, bar_height_mm, paper_size_mm, mm_per_pixel=.1):
    """Explicit sizes are assumptions, not a survey or a quiet-zone guarantee."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    manifest = {'units': 'mm', 'mm_per_pixel': mm_per_pixel,
                'bar_height_mm_assumed': bar_height_mm,
                'paper_size_mm_assumed': paper_size_mm,
                'paper_thickness_mm_assumed': 0.0,
                'PNG_is_visual_only_not_ADC': True,
                'station': 'not generated: actual marker AND ports required',
                'geometry': []}
    for name in ('STRAIGHT', 'CURVE', 'CROSS', 'T', 'BLANK'):
        tile = tile_geometry(name)
        manifest['geometry'].append((name.lower(), tile.patches,
                                     (-157.5, -157.5, 157.5, 157.5)))
    for letter in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
        barcode = barcode_geometry(letter, bar_height_mm=bar_height_mm,
                                    paper_size_mm=paper_size_mm)
        half = max(barcode.quiet_length_mm, paper_size_mm[0])/2
        yhalf = max(bar_height_mm, paper_size_mm[1])/2
        manifest['geometry'].append((f'code39_{letter}', barcode.patches,
                                     (-half, -yhalf, half, yhalf)))
    entries = []
    for name, patches, bounds in manifest.pop('geometry'):
        (output / (name+'.svg')).write_text(geometry_svg(patches, bounds))
        alpha = name.startswith('code39_')
        (output / (name+'.png')).write_bytes(png_bytes(
            raster_rows(patches, bounds, mm_per_pixel, transparent=alpha),
            channels=2 if alpha else 1))
        entries.append({'name': name, 'bounds_mm': bounds,
                        'surface_ids': [p.surface_id for p in patches]})
    manifest['assets'] = entries
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bar-height-mm', type=float, required=True)
    parser.add_argument('--paper-width-mm', type=float, required=True)
    parser.add_argument('--paper-height-mm', type=float, required=True)
    parser.add_argument('--mm-per-pixel', type=float, default=.1)
    args = parser.parse_args()
    export_assets(args.output, bar_height_mm=args.bar_height_mm,
                  paper_size_mm=(args.paper_width_mm, args.paper_height_mm),
                  mm_per_pixel=args.mm_per_pixel)


if __name__ == '__main__':
    main()
