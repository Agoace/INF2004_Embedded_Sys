"""Shared millimetre geometry for S3 visuals and first-hit surfaces.

No Webots, scene, renderer or firmware imports. All patches are convex CCW
world-space polygons; coplanar material layers are not raised colliders.
"""
from dataclasses import dataclass
import argparse
import html
import math
from pathlib import Path
from types import MappingProxyType
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]
MAT_WHITE = 'MAT_WHITE'
MAT_LINE = 'MAT_LINE'
PAPER_WHITE = 'PAPER_WHITE'
PAPER_INK = 'PAPER_INK'
MATERIALS = (MAT_WHITE, MAT_LINE, PAPER_WHITE, PAPER_INK)
# The same nine-element masks as app_program/subsystem3/barcode.c, MSB first.
CODE39 = MappingProxyType(dict(zip('ABCDEFGHIJKLMNOPQRSTUVWXYZ*', (
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0, 0x094))))


@dataclass(frozen=True)
class PolygonPatch:
    surface_id: str
    material_id: str
    vertices_mm: Tuple[Vec3, ...]
    layer: int

    def __post_init__(self):
        vertices = tuple(tuple(float(x) for x in p) for p in self.vertices_mm)
        if (not isinstance(self.surface_id, str) or not self.surface_id or
                not isinstance(self.material_id, str) or not self.material_id or
                type(self.layer) is not int or self.layer < 0 or
                len(vertices) < 3 or any(len(p) != 3 for p in vertices)):
            raise ValueError('invalid patch metadata/vertices')
        _finite(x for p in vertices for x in p)
        object.__setattr__(self, 'vertices_mm', vertices)


@dataclass(frozen=True)
class Port:
    name: str
    point_mm: Vec3
    heading_deg: float
    width_mm: float


@dataclass(frozen=True)
class TileGeometry:
    patches: Tuple[PolygonPatch, ...]
    ports: Tuple[Port, ...]
    assumptions: Tuple[str, ...] = ()
    max_chord_error_mm: float = 0.0


@dataclass(frozen=True)
class BarcodeGeometry:
    patches: Tuple[PolygonPatch, ...]
    elements_mm: Tuple[float, ...]
    body_length_mm: float
    quiet_length_mm: float
    bar_intervals_mm: Tuple[Tuple[float, float], ...]
    assumptions: Tuple[str, ...]


def _finite(values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite geometry')


def _transform(vertices, centre, degrees):
    _finite((*centre, degrees))
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return tuple((centre[0] + c*x - s*y, centre[1] + s*x + c*y,
                  centre[2]) for x, y in vertices)


def _rectangle(x0, y0, x1, y1):
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _convex(vertices):
    if len(vertices) < 3:
        raise ValueError('polygon needs at least three vertices')
    _finite(v for p in vertices for v in p)
    crosses = []
    for i, a in enumerate(vertices):
        b, c = vertices[(i+1) % len(vertices)], vertices[(i+2) % len(vertices)]
        if math.hypot(b[0]-a[0], b[1]-a[1]) <= 1e-9:
            raise ValueError('zero-length polygon edge')
        if any((b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0]) < -1e-9
               for p in vertices):
            raise ValueError('polygon must be convex and non-self-intersecting')
        crosses.append((b[0]-a[0])*(c[1]-b[1]) -
                       (b[1]-a[1])*(c[0]-b[0]))
    if min(crosses) < -1e-9 or max(crosses) <= 1e-9:
        raise ValueError('polygon must be convex, nondegenerate and CCW')


def tile_geometry(kind, *, centre_mm=(0.0, 0.0, 10.0), rotation_deg=0,
                  tile_id='tile', station_marker_mm=None, station_ports=None):
    """315 mm tiles. N/E curve is centred on the NE CORNER, not tile centre.

    STATION requires both a convex marker and explicit cardinal ports.
    Its supplied geometry remains an assumption until separately surveyed.
    """
    kind = str(kind).upper()
    _finite((*centre_mm, rotation_deg))
    if rotation_deg % 90:
        raise ValueError('tile rotation must be a quarter-turn')
    ports_by_kind = {'STRAIGHT': 'NS', 'CURVE': 'NE', 'CROSS': 'NESW',
                     'T': 'NEW', 'BLANK': '', 'STATION': ''}
    if kind not in ports_by_kind:
        raise ValueError('unknown tile kind')
    patches = []
    def add(name, material, points, layer):
        _convex(points)
        patches.append(PolygonPatch(f'{tile_id}:{name}', material,
                                    _transform(points, centre_mm, rotation_deg),
                                    layer))
    add('base', MAT_WHITE, _rectangle(-157.5, -157.5, 157.5, 157.5), 0)
    if kind in ('STRAIGHT', 'CROSS'):
        add('ns', MAT_LINE, _rectangle(-9, -157.5, 9, 157.5), 10)
    if kind in ('CROSS', 'T'):
        add('ew', MAT_LINE, _rectangle(-157.5, -9, 157.5, 9), 10)
    if kind == 'T':
        add('n', MAT_LINE, _rectangle(-9, 0, 9, 157.5), 10)
    chord_error = 0.0
    if kind == 'CURVE':
        max_angle = 2 * math.acos(1 - .01 / 166.5)
        count = math.ceil((math.pi / 2) / max_angle)
        step = math.pi / (2 * count)
        chord_error = 166.5 * (1 - math.cos(step / 2))
        def point(radius, angle):
            return (157.5 + radius*math.cos(angle),
                    157.5 + radius*math.sin(angle))
        for i in range(count):
            a, b = math.pi + i*step, math.pi + (i+1)*step
            # Convex CCW quad: inner(a), outer(a), outer(b), inner(b).
            add(f'curve:{i:03}', MAT_LINE,
                (point(148.5, a), point(166.5, a),
                 point(166.5, b), point(148.5, b)), 10)
    assumptions = ()
    if kind == 'STATION':
        if station_marker_mm is None or station_ports is None:
            raise ValueError('station marker AND ports must be supplied')
        if len(set(station_ports)) != len(station_ports) or any(
                p not in 'NESW' for p in station_ports):
            raise ValueError('invalid station ports')
        ports_by_kind['STATION'] = station_ports
        # Explicit port arms only; no assumed N/S station spine.
        arms = {'N': (-9, 0, 9, 157.5), 'S': (-9, -157.5, 9, 0),
                'E': (0, -9, 157.5, 9), 'W': (-157.5, -9, 0, 9)}
        for name in station_ports:
            add('arm:' + name, MAT_LINE, _rectangle(*arms[name]), 10)
        if any(abs(v) > 157.5 for p in station_marker_mm for v in p):
            raise ValueError('station marker is outside tile')
        add('station', MAT_LINE, tuple(station_marker_mm), 10)
        assumptions = ('station_marker_assumed', 'station_ports_assumed')
    directions = {'N': (0, 157.5, 90), 'E': (157.5, 0, 0),
                  'S': (0, -157.5, 270), 'W': (-157.5, 0, 180)}
    names = {0: 'E', 90: 'N', 180: 'W', 270: 'S'}
    ports = []
    for name in ports_by_kind[kind]:
        x, y, heading = directions[name]
        h = (heading + rotation_deg) % 360
        ports.append(Port(names[h], _transform(((x, y),), centre_mm,
                                              rotation_deg)[0], h, 18.0))
    return TileGeometry(tuple(patches), tuple(ports), assumptions, chord_error)


def barcode_elements(letter, narrow_mm=5.8, wide_mm=17.5, reverse=False):
    if letter not in CODE39 or letter == '*':
        raise ValueError('exactly one uppercase A-Z letter is required')
    _finite((narrow_mm, wide_mm))
    if narrow_mm <= 0 or wide_mm <= narrow_mm:
        raise ValueError('invalid narrow/wide dimensions')
    result = []
    for index, symbol in enumerate('*' + letter + '*'):
        if index:
            result.append(narrow_mm)
        result.extend(wide_mm if CODE39[symbol] & (0x100 >> i) else narrow_mm
                      for i in range(9))
    return tuple(reversed(result)) if reverse else tuple(result)


def barcode_geometry(letter, *, bar_height_mm, centre_mm=(0.0, 0.0, 10.0),
                     rotation_deg=0, paper_size_mm=None, paper_thickness_mm=0,
                     reverse=False, surface_id='barcode', narrow_mm=5.8,
                     wide_mm=17.5):
    _finite((*centre_mm, rotation_deg, bar_height_mm, paper_thickness_mm))
    if bar_height_mm <= 0 or paper_thickness_mm < 0:
        raise ValueError('invalid bar height/paper thickness')
    elements = barcode_elements(letter, narrow_mm, wide_mm, reverse)
    body = math.fsum(elements)
    patches = []
    centre = (centre_mm[0], centre_mm[1], centre_mm[2] + paper_thickness_mm)
    assumptions = ['bar_height_supplied_not_surveyed',
                   'paper_thickness_supplied_or_zero_default_not_surveyed']
    if paper_size_mm is not None:
        _finite(paper_size_mm)
        w, h = paper_size_mm
        if w < body or h < bar_height_mm:
            raise ValueError('paper must contain the printed body/bars')
        patches.append(PolygonPatch(surface_id + ':paper', PAPER_WHITE,
                       _transform(_rectangle(-w/2, -h/2, w/2, h/2),
                                  centre, rotation_deg), 20))
        assumptions.append('paper_size_supplied_not_surveyed')
    else:
        assumptions.append('paper_dimensions_unspecified_ink_only')
    intervals = []
    x = -body / 2
    for i, width in enumerate(elements):
        if i % 2 == 0:
            intervals.append((x, x + width))
            patches.append(PolygonPatch(f'{surface_id}:ink:{i//2:02}',
                           PAPER_INK, _transform(_rectangle(
                               x, -bar_height_mm/2, x+width, bar_height_mm/2),
                               centre, rotation_deg), 30))
        x += width
    return BarcodeGeometry(tuple(patches), elements, body,
                           body + 20*narrow_mm, tuple(intervals),
                           tuple(assumptions))


def _inside(polygon, x, y):
    return all((b[0]-a[0])*(y-a[1]) - (b[1]-a[1])*(x-a[0]) >= -1e-9
               for a, b in zip(polygon, polygon[1:] + polygon[:1]))


def material_at(patches, x_mm, y_mm) -> Optional[str]:
    """Flat-geometry query for asset tests only; optics uses scene.first_hit."""
    patches = tuple(patches)
    if any(any(abs(v[2]-p.vertices_mm[0][2]) > 1e-7 for v in p.vertices_mm)
           for p in patches):
        raise ValueError('material_at is a flat asset helper, not a raycaster')
    candidates = [p for p in patches if _inside(p.vertices_mm, x_mm, y_mm)]
    if not candidates:
        return None
    nearest_z = max(p.vertices_mm[0][2] for p in candidates)
    candidates = [p for p in candidates
                  if abs(p.vertices_mm[0][2] - nearest_z) < 1e-7]
    layer = max(p.layer for p in candidates)
    materials = {p.material_id for p in candidates if p.layer == layer}
    if len(materials) != 1:
        raise ValueError('ambiguous coplanar material')
    return materials.pop()


def geometry_svg(patches, bounds_mm):
    """Emit exactly the shared polygons; no font or renderer-specific barcode."""
    x0, y0, x1, y1 = bounds_mm
    _finite(bounds_mm)
    if x1 <= x0 or y1 <= y0:
        raise ValueError('invalid SVG bounds')
    colours = {MAT_WHITE: '#eeeeee', MAT_LINE: '#111111',
               PAPER_WHITE: '#ffffff', PAPER_INK: '#111111'}
    rows = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{x1-x0:g}mm" '
            f'height="{y1-y0:g}mm" viewBox="{x0:g} {-y1:g} '
            f'{x1-x0:g} {y1-y0:g}">']
    for p in sorted(patches, key=lambda p: (p.vertices_mm[0][2], p.layer,
                                           p.surface_id)):
        points = ' '.join(f'{x:.9f},{-y:.9f}' for x, y, _ in p.vertices_mm)
        rows.append(f'<polygon id="{html.escape(p.surface_id, quote=True)}" '
                    f'points="{points}" fill="{colours[p.material_id]}"/>')
    return '\n'.join(rows + ['</svg>', ''])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bar-height-mm', type=float, required=True,
                        help='explicit assumed/surveyed value; no default')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ('STRAIGHT', 'CURVE', 'CROSS', 'T', 'BLANK'):
        tile = tile_geometry(name)
        (args.output / (name.lower() + '.svg')).write_text(
            geometry_svg(tile.patches, (-157.5, -157.5, 157.5, 157.5)))
    for letter in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
        barcode = barcode_geometry(letter, bar_height_mm=args.bar_height_mm)
        half = barcode.quiet_length_mm / 2
        (args.output / f'code39_{letter}.svg').write_text(geometry_svg(
            barcode.patches, (-half, -args.bar_height_mm/2,
                              half, args.bar_height_mm/2)))


if __name__ == '__main__':
    main()
