"""Offline boundary-aware Gaussian ray referee; no runtime dependency.

Project ORIGINAL scene faces into the optical plane, split all boundary
intervals, and let first_hit choose their visible material. Adaptive Gaussian
integration never consults the runtime's compiled material cells. Certification
must compare two tolerances AND independent dense ray spot checks.
"""
import math
from .s3_geometry import MATERIALS
from .s3_optics import OpticalSample, mean_adc


class RefereeInvalid(Exception):
    pass


def _cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def fine_sample(scene, profile, origin_mm, direction=(0, 0, -1), axis_u=(1, 0, 0), *,
                tolerance=1e-8, max_depth=18):
    """Noise-free, normal-incidence reference. Small tolerance is mass error.

    Only the unchanged public surfaces/first_hit API is used. Boundaries are
    projected from the original scene, including occluders and layer overlaps.
    Unprojectable near-plane geometry and failed convergence are explicit.
    """
    if not 0 < tolerance <= 1e-3 or not 1 <= max_depth <= 24:
        raise ValueError('invalid referee accuracy/depth')
    if (len(origin_mm) != 3 or len(direction) != 3 or len(axis_u) != 3 or
            not all(math.isfinite(x) for x in (*origin_mm, *direction, *axis_u))):
        raise ValueError('invalid optical pose')
    dl = math.sqrt(sum(x*x for x in direction))
    ul = math.sqrt(sum(x*x for x in axis_u))
    if dl == 0 or ul == 0:
        raise ValueError('zero optical basis')
    if (abs(direction[0]/dl) > 1e-7 or abs(direction[1]/dl) > 1e-7 or
            direction[2] >= 0 or abs(axis_u[2]/ul) > 1e-7):
        return OpticalSample(None, False, 'uncalibrated_height_or_tilt')
    ux, uy = axis_u[0]/ul, axis_u[1]/ul
    vx, vy = -uy, ux
    h = profile.reference_height_mm
    sx, sy = profile.sigma_x_mm, profile.sigma_y_mm
    support_x, support_y = 4*sx, 4*sy
    faces, edges = [], []
    for surface in scene.surfaces:
        axial = [origin_mm[2]-p[2] for p in surface.vertices_mm]
        if max(axial) <= 0:
            continue
        if min(axial) <= 0:
            # Cull distant faces before rejecting an unprojectable near plane.
            radius = math.hypot(support_x, support_y)*max(axial)/h
            vertices = surface.vertices_mm
            if (min(p[0] for p in vertices) > origin_mm[0]+radius or
                    max(p[0] for p in vertices) < origin_mm[0]-radius or
                    min(p[1] for p in vertices) > origin_mm[1]+radius or
                    max(p[1] for p in vertices) < origin_mm[1]-radius):
                continue
            return OpticalSample(None, False, 'UNSUPPORTED_REFEREE_NEAR_PLANE')
        projected = tuple((((p[0]-origin_mm[0])*ux+(p[1]-origin_mm[1])*uy)*h/a,
                           ((p[0]-origin_mm[0])*vx+(p[1]-origin_mm[1])*vy)*h/a)
                          for p, a in zip(surface.vertices_mm, axial))
        if (max(p[0] for p in projected) < -support_x or
                min(p[0] for p in projected) > support_x or
                max(p[1] for p in projected) < -support_y or
                min(p[1] for p in projected) > support_y):
            continue
        faces.append(projected)
        edges.extend(zip(projected, projected[1:]+projected[:1]))
    breaks = {-support_y, support_y}
    for a, b in edges:
        breaks.update(p[1] for p in (a, b) if -support_y < p[1] < support_y)
        if abs(b[0]-a[0]) > 1e-14:
            for u in (-support_x, support_x):
                f = (u-a[0])/(b[0]-a[0])
                y = a[1]+f*(b[1]-a[1])
                if 0 < f < 1 and -support_y < y < support_y:
                    breaks.add(y)
    # Original-face edge crossings are extra topology breakpoints. They do
    # not come from the runtime partition and are classified by first_hit.
    for i, (a, b) in enumerate(edges):
        r = (b[0]-a[0], b[1]-a[1])
        for c, d in edges[i+1:]:
            s = (d[0]-c[0], d[1]-c[1])
            denominator = _cross(r, s)
            if abs(denominator) < 1e-12:
                continue
            delta = (c[0]-a[0], c[1]-a[1])
            t, v = _cross(delta, s)/denominator, _cross(delta, r)/denominator
            y = a[1]+t*r[1]
            if 0 < t < 1 and 0 < v < 1 and -support_y < y < support_y:
                breaks.add(y)
    cuts = sorted(breaks)
    # Merge roundoff-sized intervals; scene geometry tolerance is1e-7mm.
    cuts = [v for i, v in enumerate(cuts) if i == 0 or v-cuts[i-1] > 1e-10]
    rays = 0
    root2x = math.sqrt(2)*sx
    def row(v):
        nonlocal rays
        xs = {-support_x, support_x}
        for polygon in faces:
            for a, b in zip(polygon, polygon[1:]+polygon[:1]):
                if (a[1] > v) != (b[1] > v):
                    u = a[0]+(v-a[1])*(b[0]-a[0])/(b[1]-a[1])
                    if -support_x < u < support_x:
                        xs.add(u)
        xs = sorted(xs)
        values = [0.0]*len(MATERIALS)
        for a, b in zip(xs, xs[1:]):
            if b-a < 1e-10:
                continue
            u = (a+b)/2
            ray = (ux*u/h+vx*v/h, uy*u/h+vy*v/h, -1)
            hit = scene.first_hit(tuple(origin_mm), ray, 100)
            rays += 1
            if not hit.valid:
                raise RefereeInvalid(hit.reason)
            if hit.material_id not in MATERIALS:
                raise RefereeInvalid('unknown_material')
            if (abs(origin_mm[2]-hit.point_mm[2]-h) > .001 or
                    hit.normal[2] < 1-1e-6):
                raise RefereeInvalid('uncalibrated_height_or_tilt')
            values[MATERIALS.index(hit.material_id)] += .5*(
                math.erf(b/root2x)-math.erf(a/root2x))
        density = math.exp(-.5*(v/sy)**2)/(sy*math.sqrt(2*math.pi))
        return tuple(x*density for x in values)
    def gauss(a, b):
        middle, half = (a+b)/2, (b-a)/2
        offset = half/math.sqrt(3)
        left, right = row(middle-offset), row(middle+offset)
        return tuple(half*(x+y) for x, y in zip(left, right))
    def integral(a, b, coarse, budget, depth):
        middle = (a+b)/2
        left, right = gauss(a, middle), gauss(middle, b)
        refined = tuple(x+y for x, y in zip(left, right))
        error = max(abs(x-y) for x, y in zip(coarse, refined))
        if error <= budget:
            return refined
        if depth >= max_depth:
            raise RefereeInvalid('REFEREE_NOT_CONVERGED')
        l = integral(a, middle, left, budget/2, depth+1)
        r = integral(middle, b, right, budget/2, depth+1)
        return tuple(x+y for x, y in zip(l, r))
    fractions = [0.0]*len(MATERIALS)
    try:
        for a, b in zip(cuts, cuts[1:]):
            weights = integral(a, b, gauss(a, b), tolerance*(b-a)/(8*sy), 0)
            fractions = [x+y for x, y in zip(fractions, weights)]
    except RefereeInvalid as error:
        return OpticalSample(None, False, str(error), rays=rays)
    norm = math.erf(4/math.sqrt(2))**2
    fractions = {m: max(0, v/norm) for m, v in zip(MATERIALS, fractions)}
    total = sum(fractions.values())
    if abs(total-1) > max(1e-7, 10*tolerance):
        return OpticalSample(None, False, 'REFEREE_COVERAGE_ERROR', rays=rays)
    fractions = {m: v/total for m, v in fractions.items()}
    mean = mean_adc(profile, fractions)
    return OpticalSample(round(mean), True, '', mean, h, fractions,
                         ('OFFLINE_NOISE_FREE_BOUNDARY_REFINED_REFEREE',), False, rays)
