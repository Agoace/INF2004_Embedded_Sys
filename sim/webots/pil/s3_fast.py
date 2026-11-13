"""F1: bounded, flat isotropic material integrals. NOT physical calibration.

Compile axis-aligned rectangles to disjoint visible cells. Runtime has no
scene ray calls, cache builds or curve approximations. Unknown domains fail.
"""
from dataclasses import dataclass
import hashlib
import math
import random
from types import MappingProxyType

from .s3_geometry import MATERIALS
from .s3_optics import OpticalSample, mean_adc

BIN_MM = 64.0
EPS = 1e-7
NORMALIZER = math.erf(4/math.sqrt(2))**2


def _bounds(vertices):
    return tuple(min(p[i] for p in vertices) for i in range(3)) + tuple(
        max(p[i] for p in vertices) for i in range(3))


def _bins(x0, y0, x1, y1):
    # Include boundary bins; sample intersection uses strict positive area.
    return ((x, y) for x in range(math.floor(x0/BIN_MM), math.floor(x1/BIN_MM)+1)
            for y in range(math.floor(y0/BIN_MM), math.floor(y1/BIN_MM)+1))


def _rectangle(p):
    v = p.vertices_mm
    if len(v) != 4 or any(abs(a[2]-v[0][2]) > EPS for a in v):
        return None
    x0, y0, z, x1, y1, _ = _bounds(v)
    if x1-x0 <= EPS or y1-y0 <= EPS:
        return None
    if any(min(abs(x-x0), abs(x-x1)) > EPS or
           min(abs(y-y0), abs(y-y1)) > EPS for x, y, _ in v):
        return None
    # Require four distinct corners, not a crossed or repeated rectangle.
    corners = {(round((x-x0)/(x1-x0)), round((y-y0)/(y1-y0))) for x, y, _ in v}
    area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(v, v[1:]+v[:1]))
    if len(corners) != 4 or area <= EPS:
        return None
    return x0, y0, x1, y1, z, p.material_id, p.layer, p.surface_id


def _compile_bin(key, rectangles):
    bx, by = key[0]*BIN_MM, key[1]*BIN_MM
    xs = sorted({bx, bx+BIN_MM} | {max(bx, min(bx+BIN_MM, r[i]))
                                  for r in rectangles for i in (0, 2)})
    ys = sorted({by, by+BIN_MM} | {max(by, min(by+BIN_MM, r[i]))
                                  for r in rectangles for i in (1, 3)})
    if len(xs)*len(ys) > 100_000:
        raise ValueError('F1 compile complexity limit exceeded')
    active, output = {}, []
    for y0, y1 in zip(ys, ys[1:]):
        row = []
        for x0, x1 in zip(xs, xs[1:]):
            x, y = (x0+x1)/2, (y0+y1)/2
            hits = [r for r in rectangles if r[0] < x < r[2] and r[1] < y < r[3]]
            if not hits:
                continue
            z = max(r[4] for r in hits)
            hits = [r for r in hits if abs(r[4]-z) <= EPS]
            layer = max(r[6] for r in hits)
            materials = {r[5] for r in hits if r[6] == layer}
            material = next(iter(materials)) if len(materials) == 1 else 'AMBIGUOUS'
            if row and row[-1][2] == x0 and row[-1][4:] == (z, material):
                row[-1] = (row[-1][0], y0, x1, y1, z, material)
            else:
                row.append((x0, y0, x1, y1, z, material))
        following = {}
        for cell in row:
            signature = (cell[0], cell[2], cell[4], cell[5])
            old = active.pop(signature, None)
            following[signature] = ((old[0], old[1], old[2], y1, old[4], old[5])
                                    if old is not None else cell)
        output.extend(active.values())
        active = following
    output.extend(active.values())
    return tuple(output)


@dataclass(frozen=True)
class FastScene:
    cells: object
    unsupported: object
    revision: str
    cell_count: int

    @classmethod
    def from_patches(cls, patches, unsupported_bounds=()):
        """Bounds are (xmin,ymin,zmin,xmax,ymax,zmax,reason)."""
        patches = tuple(patches)
        ids = [p.surface_id for p in patches]
        if len(set(ids)) != len(ids):
            raise ValueError('duplicate surface ID')
        rect_bins, rejected = {}, list(unsupported_bounds)
        for p in patches:
            r = _rectangle(p)
            if r is None:
                rejected.append((*_bounds(p.vertices_mm), 'UNSUPPORTED_NONRECTANGLE'))
                continue
            for key in _bins(r[0], r[1], r[2], r[3]):
                rect_bins.setdefault(key, []).append(r)
                if len(rect_bins) > 65536:
                    raise ValueError('F1 spatial index size limit exceeded')
        cells = {key: _compile_bin(key, rects) for key, rects in rect_bins.items()}
        if any(len(v) > 512 for v in cells.values()):
            raise ValueError('F1 cells-per-bin limit exceeded')
        invalid = {}
        for b in rejected:
            if len(b) != 7 or not all(math.isfinite(x) for x in b[:6]):
                raise ValueError('invalid unsupported bounds')
            for key in _bins(b[0], b[1], b[3], b[4]):
                invalid.setdefault(key, []).append(tuple(b))
                if len(invalid) > 65536 or len(invalid[key]) > 512:
                    raise ValueError('F1 unsupported index limit exceeded')
        canonical = tuple(sorted((p.surface_id, p.material_id, p.vertices_mm, p.layer)
                                 for p in patches)), tuple(sorted(rejected))
        revision = hashlib.sha256(repr(canonical).encode()).hexdigest()
        return cls(MappingProxyType(cells),
                   MappingProxyType({k: tuple(v) for k, v in invalid.items()}),
                   revision, sum(map(len, cells.values())))

    @classmethod
    def from_course(cls, built):
        rejected = []
        for tile in built.course.data['tiles']:
            if tile['kind'] not in ('STRAIGHT', 'CROSS', 'T', 'BLANK'):
                centre = next(c for name, c, _ in built.tile_geometries if name == tile['id'])
                x, y, z = centre
                rejected.append((x-157.5, y-157.5, z, x+157.5, y+157.5, z,
                                 'UNSUPPORTED_'+tile['kind']))
        for collider in built.colliders:
            if collider.kind not in ('tile', 'paper'):
                vertices = tuple(v for s in collider.surfaces for v in s.vertices_mm)
                rejected.append((*_bounds(vertices), 'UNSUPPORTED_OCCLUDER'))
        return cls.from_patches(built.patches, rejected)

    def sample(self, profile, origin_mm, direction=(0, 0, -1), axis_u=(1, 0, 0), *,
               sample_us=0, seed=0, expected_revision=None):
        """No runtime fallback. Conservative complete-footprint domain guard.

        Valid means evaluated EXPLORATORY math, not a calibrated physical ADC.
        The bounding square includes every yaw of the reference four-sigma
        support. Missing/unsupported/other-height regions invalidate it.
        """
        def fail(reason):
            return OpticalSample(None, False, reason)
        if expected_revision is not None and expected_revision != self.revision:
            return fail('STALE_SCENE_REVISION')
        if (len(origin_mm) != 3 or len(direction) != 3 or len(axis_u) != 3 or
                not all(math.isfinite(x) for x in (*origin_mm, *direction, *axis_u))):
            raise ValueError('nonfinite/invalid sensor pose')
        dl = math.sqrt(sum(x*x for x in direction))
        ul = math.sqrt(sum(x*x for x in axis_u))
        if dl == 0 or ul == 0:
            raise ValueError('zero optical basis')
        if (abs(direction[0]/dl) > 1e-7 or abs(direction[1]/dl) > 1e-7 or
                direction[2] >= 0 or abs(axis_u[2]/ul) > 1e-7):
            return fail('uncalibrated_height_or_tilt')
        if abs(profile.sigma_x_mm-profile.sigma_y_mm) > 1e-10:
            return fail('UNSUPPORTED_ANISOTROPIC')
        if not isinstance(sample_us, int) or sample_us < 0:
            raise ValueError('invalid sample time')
        x, y, z = origin_mm
        plane = z-profile.reference_height_mm
        sigma = profile.sigma_x_mm
        radius = 4*sigma*math.sqrt(2)
        if (not math.isfinite(radius) or
                (math.floor((x+radius)/BIN_MM)-math.floor((x-radius)/BIN_MM)+1) *
                (math.floor((y+radius)/BIN_MM)-math.floor((y-radius)/BIN_MM)+1) > 64):
            return fail('UNSUPPORTED_FOOTPRINT_SIZE')
        keys = tuple(_bins(x-radius, y-radius, x+radius, y+radius))
        if sum(len(self.cells.get(k, ())) for k in keys) > 512:
            return fail('UNSUPPORTED_QUERY_COMPLEXITY')
        area, fractions = 0.0, {}
        norm = math.sqrt(2)*sigma
        support = 4*sigma
        for key in keys:
            for b in self.unsupported.get(key, ()):
                if (b[0] < x+radius and b[3] > x-radius and b[1] < y+radius and
                        b[4] > y-radius and b[2] <= z and b[5] >= plane-EPS):
                    return fail(b[6])
            for a, b, c, d, height, material in self.cells.get(key, ()):
                dx = min(c, x+radius)-max(a, x-radius)
                dy = min(d, y+radius)-max(b, y-radius)
                if dx <= 0 or dy <= 0:
                    continue
                if abs(height-plane) > .001:
                    return fail('uncalibrated_height_or_tilt')
                if material == 'AMBIGUOUS':
                    return fail('ambiguous_material')
                if material not in MATERIALS:
                    return fail('unknown_material')
                area += dx*dy
                lo, hi = max(a, x-support), min(c, x+support)
                bottom, top = max(b, y-support), min(d, y+support)
                if hi > lo and top > bottom:
                    wx = .5*(math.erf((hi-x)/norm)-math.erf((lo-x)/norm))
                    wy = .5*(math.erf((top-y)/norm)-math.erf((bottom-y)/norm))
                    fractions[material] = fractions.get(material, 0)+wx*wy/NORMALIZER
        if area < (2*radius)**2-1e-6:
            return fail('UNSUPPORTED_ENDCAP_OR_GAP')
        total = sum(fractions.values())
        if abs(total-1) > 1e-7:
            return fail('UNSUPPORTED_COVERAGE')
        mean = mean_adc(profile, fractions)
        raw = round(mean+random.Random(f'{seed}:{profile.name}:{sample_us}').gauss(
            0, profile.noise_sigma_adc))
        return OpticalSample(max(0, min(4095, raw)), True, '', mean,
                             profile.reference_height_mm, MappingProxyType(fractions),
                             profile.assumptions+('F1_EXPLORATORY_RECTANGLES_ONLY',),
                             raw < 0 or raw > 4095, 0)
