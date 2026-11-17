"""Course-path line reference: the closed track loop of a course JSON.

Trace the track from the start pose through connected tile ports. A CROSS
is driven straight through. Each tile gives one segment: a straight line
or a quarter arc (radius 157.5 mm, centre on the tile corner, as
s3_geometry.tile_geometry). Tiles with a line that are not on the loop are
'loose'. Used only by the course8 controller; the old fixed references in
sil_line.py, sil_curve.py and sil_s_curve.py are unchanged.
"""
from dataclasses import dataclass
import math

from .s3_geometry import tile_geometry

TILE_MM = 315.0
HALF = TILE_MM / 2
STEP = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
HEADING = {0: 'E', 90: 'N', 180: 'W', 270: 'S'}


@dataclass(frozen=True)
class Nearest:
    error_mm: float     # signed, left of the drive direction positive
    s_mm: float         # arc length along the loop
    tile_id: str
    segment: int


class CoursePath:
    def __init__(self, segments, ports, origin, loose_ids, tiles_at):
        self.segments = tuple(segments)
        self.tiles = tuple(s['tile'] for s in self.segments)
        self.length_mm = sum(s['length'] for s in self.segments)
        self.loose_ids = frozenset(loose_ids)
        self._origin = origin
        self._tiles_at = tiles_at

    @classmethod
    def from_course(cls, course, *, max_tiles=400):
        data = course.data
        ox, oy = data['origin_mm']
        grid, loose = {}, set()
        for t in data['tiles']:
            ports = ''.join(p.name for p in tile_geometry(
                t['kind'], rotation_deg=t['rotation_deg']).ports)
            grid[(t['column'], t['row'])] = (t['id'], t['kind'], ports)
        sx, sy = data['start']['position_mm']
        yaw = data['start']['yaw_deg'] % 360
        if yaw not in HEADING:
            raise ValueError('start yaw must be a cardinal heading')
        heading = HEADING[yaw]
        cell = (math.floor((sx - ox) / TILE_MM), math.floor((sy - oy) / TILE_MM))
        first = (cell, heading)
        segments, s = [], 0.0
        for _ in range(max_tiles):
            tile = grid.get(cell)
            if tile is None or OPP[heading] not in tile[2]:
                raise ValueError(f'open track at {cell} heading {heading}')
            tid, kind, ports = tile
            if kind == 'CROSS':
                out = heading
            else:
                rest = ports.replace(OPP[heading], '', 1)
                if len(rest) != 1:
                    raise ValueError(f'unsupported tile {tid} for path')
                out = rest
            cx = ox + (cell[0] + 0.5) * TILE_MM
            cy = oy + (cell[1] + 0.5) * TILE_MM
            ix, iy = STEP[heading]
            ex, ey = STEP[out]
            start = (cx - ix * HALF, cy - iy * HALF)
            if out == heading:
                seg = dict(kind='line', start=start, dir=(ix, iy),
                           length=TILE_MM)
            else:
                centre = (start[0] + ex * HALF, start[1] + ey * HALF)
                turn = 1 if ix * ey - iy * ex > 0 else -1   # +1 left (CCW)
                a0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
                seg = dict(kind='arc', centre=centre, a0=a0, turn=turn,
                           length=math.pi * HALF / 2)
            seg.update(tile=tid, s0=s)
            s += seg['length']
            segments.append(seg)
            cell = (cell[0] + ex, cell[1] + ey)
            heading = out
            if (cell, heading) == first:
                break
        else:
            raise ValueError('track loop not closed')
        on_loop = {seg['tile'] for seg in segments}
        for (_, _), (tid, kind, ports) in grid.items():
            if ports and tid not in on_loop:
                loose.add(tid)
        tiles_at = {k: v[0] for k, v in grid.items()}
        return cls(segments, None, (ox, oy), loose, tiles_at)

    def _segment_nearest(self, index, x, y):
        seg = self.segments[index]
        if seg['kind'] == 'line':
            (px, py), (dx, dy) = seg['start'], seg['dir']
            t = min(max((x - px) * dx + (y - py) * dy, 0.0), seg['length'])
            qx, qy = px + dx * t, py + dy * t
            # Left normal of (dx, dy) is (-dy, dx).
            err = (x - qx) * -dy + (y - qy) * dx
            dist = math.hypot(x - qx, y - qy)
            if t in (0.0, seg['length']):
                err = math.copysign(dist, err if err else 1.0)
            return dist, err, seg['s0'] + t
        cx, cy = seg['centre']
        r = math.hypot(x - cx, y - cy)
        a = math.atan2(y - cy, x - cx)
        rel = (a - seg['a0']) * seg['turn']
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        sweep = math.pi / 2
        if 0.0 <= rel <= sweep:
            # Turning left: the centre is on the left, so inside = left.
            err = (HALF - r) * seg['turn']
            return abs(r - HALF), err, seg['s0'] + rel * HALF
        best = None
        for end in (0.0, sweep):
            ang = seg['a0'] + seg['turn'] * end
            qx, qy = cx + HALF * math.cos(ang), cy + HALF * math.sin(ang)
            d = math.hypot(x - qx, y - qy)
            if best is None or d < best[0]:
                rr = (HALF - r) * seg['turn']
                best = (d, math.copysign(d, rr if rr else 1.0),
                        seg['s0'] + end * HALF)
        return best

    def nearest(self, x, y, indices=None):
        best = None
        for i in (range(len(self.segments)) if indices is None else indices):
            d, err, s = self._segment_nearest(i, x, y)
            if best is None or d < best[0]:
                best = (d, err, s, i)
        d, err, s, i = best
        return Nearest(err, s % self.length_mm, self.segments[i]['tile'], i)

    def point(self, s):
        s %= self.length_mm
        for seg in self.segments:
            if s < seg['s0'] + seg['length']:
                t = s - seg['s0']
                if seg['kind'] == 'line':
                    (px, py), (dx, dy) = seg['start'], seg['dir']
                    return px + dx * t, py + dy * t
                ang = seg['a0'] + seg['turn'] * t / HALF
                cx, cy = seg['centre']
                return cx + HALF * math.cos(ang), cy + HALF * math.sin(ang)
        return self.point(0.0)

    def tile_at(self, x, y):
        ox, oy = self._origin
        return self._tiles_at.get((math.floor((x - ox) / TILE_MM),
                                   math.floor((y - oy) / TILE_MM)))

    def is_loose(self, tile_id):
        return tile_id in self.loose_ids


class PathFollower:
    """Nearest point near the last one (window), so the two passes through
    the crossing never swap; progress is unwrapped arc length."""
    WINDOW_MM = 400.0

    def __init__(self, path, x, y):
        self.path = path
        first = path.nearest(x, y)
        self.start_s = first.s_mm
        self._s = first.s_mm
        self.progress_mm = 0.0

    def _window(self):
        n, length = len(self.path.segments), self.path.length_mm
        out = []
        for i, seg in enumerate(self.path.segments):
            lo = (seg['s0'] - self._s) % length
            hi = (seg['s0'] + seg['length'] - self._s) % length
            if (lo <= self.WINDOW_MM or lo >= length - self.WINDOW_MM - seg['length']
                    or hi <= self.WINDOW_MM):
                out.append(i)
        return out or list(range(n))

    def update(self, x, y):
        r = self.path.nearest(x, y, self._window())
        length = self.path.length_mm
        ds = (r.s_mm - self._s + length / 2) % length - length / 2
        self.progress_mm += ds
        self._s = r.s_mm
        return r

    @property
    def laps(self):
        return int(self.progress_mm // self.path.length_mm)
