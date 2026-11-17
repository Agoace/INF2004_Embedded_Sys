"""numpy batch of s3_optics.sample_sensor on a SceneSnapshot (course8 only).

Same rays, same float order, same EPS, layer/tie/ambiguous rules and the
same OpticalSample. Each Python sum(x*y) is written as ((0.0+a)+b)+c. The
host test proves bit-identical samples against sample_sensor (flat and
tilted poses). A conservative per-surface pre-filter only removes surfaces
that no ray of the bundle can reach; every kept (surface, ray) pair runs
the exact _clip and intersect math. Not a new optical model.
"""
import math
import random
from types import MappingProxyType

import numpy as np

from .s3_optics import OpticalSample, _quadrature, _unit, _cross, mean_adc
from .scene import EPS_MM


def _dot(a0, a1, a2, b0, b1, b2):
    return ((0.0 + a0 * b0) + a1 * b1) + a2 * b2


class NumpySceneSampler:
    """Drop-in for pil.sil_optics.FullSceneSampler.sample (no GP28)."""

    def __init__(self, built):
        self.scene = built.scene
        surfaces = self.scene.surfaces
        n = len(surfaces)
        vmax = max(len(s.vertices_mm) for s in surfaces)
        verts = np.empty((n, vmax, 3))
        for i, s in enumerate(surfaces):
            v = list(s.vertices_mm)
            v += [v[-1]] * (vmax - len(v))   # zero edges never reject
            verts[i] = v
        self.verts = verts
        nxt = np.roll(verts, -1, axis=1)
        # Padding: repeated vertices give zero edges; the last real vertex
        # is followed by vertex 0, so the closing edge is unchanged.
        self.edges = nxt - verts
        self.normal = np.array([s.normal for s in surfaces])
        self.low = np.array([s._bounds[0] for s in surfaces])
        self.high = np.array([s._bounds[1] for s in surfaces])
        self.layer = np.array([s.layer for s in surfaces])
        self.material = [s.material_id for s in surfaces]
        names = sorted(set(self.material))
        self.material_names = names
        self.mat_index = np.array([names.index(m) for m in self.material])
        self.surface_id = [s.surface_id for s in surfaces]
        order = sorted(range(n), key=lambda i: surfaces[i].surface_id)
        self.rank = np.empty(n, dtype=np.int64)
        self.rank[order] = np.arange(n)
        self.gp28_calls = None
        self._quad = {}
        self._grid = {}

    # -- exact SceneSnapshot math, batched over (surface, ray) pairs --
    def _clip_ok(self, idx, origin, rays, limit):
        low, high = self.low[idx], self.high[idx]
        near = np.zeros(len(idx))
        far = np.full(len(idx), limit)
        ok = np.ones(len(idx), dtype=bool)
        for i in range(3):
            r = rays[:, i]
            o = origin[i]
            flat = np.abs(r) <= 1e-15
            outside = (o < low[:, i] - EPS_MM) | (o > high[:, i] + EPS_MM)
            ok &= ~(flat & outside)
            with np.errstate(divide='ignore', invalid='ignore'):
                a = (low[:, i] - o) / r
                b = (high[:, i] - o) / r
            near = np.where(flat, near, np.maximum(near, np.minimum(a, b)))
            far = np.where(flat, far, np.minimum(far, np.maximum(a, b)))
        return ok & (near <= far + EPS_MM)

    def _intersect(self, idx, origin, rays, limit):
        n = self.normal[idx]
        den = _dot(rays[:, 0], rays[:, 1], rays[:, 2], n[:, 0], n[:, 1], n[:, 2])
        v0 = self.verts[idx, 0]
        num = _dot(v0[:, 0] - origin[0], v0[:, 1] - origin[1],
                   v0[:, 2] - origin[2], n[:, 0], n[:, 1], n[:, 2])
        with np.errstate(divide='ignore', invalid='ignore'):
            dist = num / den
        ok = (np.abs(den) > 1e-12) & ~(dist < -EPS_MM) & ~(dist > limit + EPS_MM)
        dist = np.where(dist > 0.0, dist, 0.0)
        point = origin[None, :] + dist[:, None] * rays
        verts, edges = self.verts[idx], self.edges[idx]
        for k in range(verts.shape[1]):
            e, v = edges[:, k], verts[:, k]
            p = point - v
            c0 = e[:, 1] * p[:, 2] - e[:, 2] * p[:, 1]
            c1 = e[:, 2] * p[:, 0] - e[:, 0] * p[:, 2]
            c2 = e[:, 0] * p[:, 1] - e[:, 1] * p[:, 0]
            ok &= ~(_dot(c0, c1, c2, n[:, 0], n[:, 1], n[:, 2]) < -EPS_MM)
        return ok, dist, point

    def _prefilter(self, origin, rays, limit):
        rz = rays[:, 2]
        if not np.all(rz < 0):
            return np.arange(len(self.low))
        slope = float(np.max(np.abs(rays[:, :2]) / -rz[:, None]))
        depth = np.clip(origin[2] - self.low[:, 2], 0.0, limit) + 1.0
        reach = depth * slope + 1.0
        keep = ((self.low[:, 0] <= origin[0] + reach) &
                (self.high[:, 0] >= origin[0] - reach) &
                (self.low[:, 1] <= origin[1] + reach) &
                (self.high[:, 1] >= origin[1] - reach) &
                (self.low[:, 2] <= origin[2] + 1.0))
        return np.nonzero(keep)[0]

    def first_hits(self, origin, directions, limit):
        """first_hit for each row of directions (m, 3), not yet unit.

        Returns (status, material index, point, normal, distance) arrays;
        status 0 miss, 1 ambiguous_material, 2 hit."""
        origin = np.array([float(x) for x in origin])
        dirs = np.asarray(directions, dtype=float)
        size = np.sqrt(_dot(dirs[:, 0], dirs[:, 1], dirs[:, 2],
                            dirs[:, 0], dirs[:, 1], dirs[:, 2]))
        rays = dirs / size[:, None]
        cand = self._prefilter(origin, rays, limit)
        m, s = len(rays), len(cand)
        ri = np.repeat(np.arange(m), s)
        si = np.tile(cand, m)
        pr = rays[ri]
        ok = self._clip_ok(si, origin, pr, limit)
        ri, si, pr = ri[ok], si[ok], pr[ok]
        ok, dist, point = self._intersect(si, origin, pr, limit)
        ri, si, dist, point = ri[ok], si[ok], dist[ok], point[ok]
        status = np.zeros(m, dtype=np.int64)
        mat_out = np.zeros(m, dtype=np.int64)
        point_out = np.zeros((m, 3))
        normal_out = np.zeros((m, 3))
        dist_out = np.zeros(m)
        if not len(ri):
            return status, mat_out, point_out, normal_out, dist_out
        nearest = np.full(m, np.inf)
        np.minimum.at(nearest, ri, dist)
        tie = np.abs(dist - nearest[ri]) <= EPS_MM
        ri, si, dist, point = ri[tie], si[tie], dist[tie], point[tie]
        layer = self.layer[si]
        top = np.full(m, -1)
        np.maximum.at(top, ri, layer)
        keep = layer == top[ri]
        ri, si, dist, point = ri[keep], si[keep], dist[keep], point[keep]
        mat = self.mat_index[si]
        lo = np.full(m, 1 << 30)
        hi = np.full(m, -1)
        np.minimum.at(lo, ri, mat)
        np.maximum.at(hi, ri, mat)
        # One winner per ray: the lowest surface_id rank among the tied.
        order = np.lexsort((self.rank[si], ri))
        first = order[np.r_[True, ri[order][1:] != ri[order][:-1]]]
        r = ri[first]
        status[r] = np.where(lo[r] != hi[r], 1, 2)
        mat_out[r] = mat[first]
        point_out[r] = point[first]
        normal_out[r] = self.normal[si[first]]
        dist_out[r] = dist[first]
        return status, mat_out, point_out, normal_out, dist_out

    def sample(self, profile, origin_mm, direction=(0, 0, -1), axis_u=(1, 0, 0),
               *, sample_us=0, seed=0, max_distance_mm=100.0, quadrature=17,
               bias_adc=0.0, allow_extrapolation=False):
        d, u = _unit(direction), _unit(axis_u)
        if abs(sum(a*b for a, b in zip(d, u))) > 1e-7:
            raise ValueError('footprint axis must be perpendicular to optical axis')
        v = _cross(u, d)
        if len(origin_mm) != 3 or not all(math.isfinite(x) for x in origin_mm):
            raise ValueError('invalid origin')
        if not math.isfinite(bias_adc) or not math.isfinite(max_distance_mm) or max_distance_mm <= 0:
            raise ValueError('invalid bias/ray range')
        if not isinstance(sample_us, int) or sample_us < 0:
            raise ValueError('sample_us must be a nonnegative integer')
        nodes = self._quad.get(quadrature)
        if nodes is None:
            nodes = self._quad[quadrature] = _quadrature(quadrature)
        h = profile.reference_height_mm
        key = (quadrature, profile.sigma_x_mm, profile.sigma_y_mm, h)
        grid = self._grid.get(key)
        if grid is None:
            xs = np.array([x for x, _ in nodes for _ in nodes])
            ys = np.array([y for _ in nodes for y, _ in nodes])
            ws = [wx*wy for _, wx in nodes for _, wy in nodes]
            grid = self._grid[key] = (xs, ys, ws)
        xs, ys, ws = grid
        dirs = np.empty((len(xs) + 1, 3))
        dirs[0] = d
        for i in range(3):
            # d + u*x*sx/h + v*y*sy/h, left to right, as sample_sensor.
            dirs[1:, i] = (d[i] + u[i]*xs*profile.sigma_x_mm/h) + \
                v[i]*ys*profile.sigma_y_mm/h
        status, mat, point, normal, dist = self.first_hits(
            tuple(origin_mm), dirs, float(max_distance_mm))
        if status[0] != 2:
            return OpticalSample(None, False, ('miss', 'ambiguous_material')[status[0]])
        c_dist = float(dist[0])
        c_normal = tuple(normal[0].tolist())
        incidence = -sum(a*b for a, b in zip(d, c_normal))
        outside = (abs(c_dist-h) > .001 or incidence < 1 - 1e-6)
        assumptions = profile.assumptions
        if outside:
            if not allow_extrapolation:
                return OpticalSample(None, False, 'uncalibrated_height_or_tilt',
                                     standoff_mm=c_dist)
            assumptions += ('uncalibrated_height_or_tilt',)
        o = [float(x) for x in origin_mm]
        axial = _dot(point[1:, 0] - o[0], point[1:, 1] - o[1],
                     point[1:, 2] - o[2], d[0], d[1], d[2])
        tilted = -_dot(d[0], d[1], d[2], normal[1:, 0], normal[1:, 1],
                       normal[1:, 2]) < 1 - 1e-6
        known = np.array([self.material_names[k] in profile.levels_adc
                          for k in range(len(self.material_names))])
        st = status[1:]
        # First failing ray decides, in sample_sensor order.
        fail = np.where(st == 0, 1, np.where(st == 1, 2, np.where(
            ~known[mat[1:]], 3, np.where((np.abs(axial-h) > .001) | tilted,
                                         4, 0))))
        if not allow_extrapolation:
            bad = np.nonzero(fail)[0]
        else:
            bad = np.nonzero((fail > 0) & (fail < 4))[0]
        if len(bad):
            k = int(bad[0])
            reason = ('', 'miss', 'ambiguous_material', 'unknown_material',
                      'uncalibrated_height_or_tilt')[int(fail[k])]
            return OpticalSample(None, False, reason, rays=k + 2)
        if np.any(fail == 4) and 'uncalibrated_height_or_tilt' not in assumptions:
            assumptions += ('uncalibrated_height_or_tilt',)
        rays = len(st) + 1
        fractions = {}
        names = self.material_names
        for k, weight in zip(mat[1:].tolist(), ws):
            name = names[k]
            fractions[name] = fractions.get(name, 0) + weight
        mean = mean_adc(profile, fractions)
        noise = random.Random(f'{seed}:{profile.name}:{sample_us}').gauss(
            0, profile.noise_sigma_adc)
        raw = round(mean + bias_adc + noise)
        clipped = raw < 0 or raw > 4095
        return OpticalSample(max(0, min(4095, raw)), True, '', mean,
                             c_dist, MappingProxyType(fractions),
                             assumptions, clipped, rays)
