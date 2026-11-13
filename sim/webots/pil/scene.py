"""Immutable, millimetre-based optical scene; independent of Webots and firmware.

Patch coordinates and normals are world-space. Physical distance wins over
coplanar layer priority. A miss or ambiguous material is invalid, never white.
"""

from dataclasses import dataclass, field
import math
from types import MappingProxyType
from typing import Iterable, Optional, Tuple

Vec3 = Tuple[float, float, float]
Vec2 = Tuple[float, float]
EPS_MM = 1e-7


def vec3(value: Iterable[float]) -> Vec3:
    result = tuple(float(x) for x in value)
    if len(result) != 3 or not all(math.isfinite(x) for x in result):
        raise ValueError("expected three finite coordinates")
    return result


def sub(a: Vec3, b: Vec3) -> Vec3:
    return tuple(x - y for x, y in zip(a, b))


def dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def unit(a: Vec3) -> Vec3:
    size = math.sqrt(dot(a, a))
    if not math.isfinite(size) or size <= 1e-15:
        raise ValueError("direction must be finite and nonzero")
    return tuple(x / size for x in a)


@dataclass(frozen=True)
class SurfaceHit:
    point_mm: Optional[Vec3] = None
    normal: Optional[Vec3] = None
    distance_mm: Optional[float] = None
    material_id: Optional[str] = None
    local_uv_mm: Optional[Vec2] = None
    surface_id: Optional[str] = None
    valid: bool = False
    reason: str = "miss"


@dataclass(frozen=True)
class PolygonSurface:
    """Convex planar polygon, CCW viewed from its outward normal.

    UV origin is vertex zero, U points to vertex one, V = normal cross U.
    PolygonPatch from s3_geometry is accepted by SceneSnapshot.from_patches.
    """

    surface_id: str
    material_id: str
    vertices_mm: Tuple[Vec3, ...]
    layer: int = 0
    _normal: Vec3 = field(init=False, repr=False, compare=False)
    _u: Vec3 = field(init=False, repr=False, compare=False)
    _v: Vec3 = field(init=False, repr=False, compare=False)
    _bounds: tuple = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        vertices = tuple(vec3(p) for p in self.vertices_mm)
        object.__setattr__(self, "vertices_mm", vertices)
        if not self.surface_id or not self.material_id or len(vertices) < 3:
            raise ValueError("polygon requires ID, material and at least three vertices")
        if type(self.layer) is not int or self.layer < 0:
            raise ValueError("layer must be a nonnegative integer")
        normal = unit(cross(sub(vertices[1], vertices[0]), sub(vertices[2], vertices[0])))
        u = unit(sub(vertices[1], vertices[0]))
        object.__setattr__(self, "_normal", normal)
        object.__setattr__(self, "_u", u)
        object.__setattr__(self, "_v", cross(normal, u))
        bounds = (tuple(min(p[i] for p in vertices) for i in range(3)),
                  tuple(max(p[i] for p in vertices) for i in range(3)))
        object.__setattr__(self, "_bounds", bounds)
        if any(abs(dot(sub(p, vertices[0]), normal)) > EPS_MM for p in vertices):
            raise ValueError("polygon must be planar")
        signs = []
        for i, vertex in enumerate(vertices):
            edge = sub(vertices[(i + 1) % len(vertices)], vertex)
            if dot(edge, edge) <= EPS_MM ** 2:
                raise ValueError("polygon has a zero-length edge")
            turn = dot(cross(edge, sub(vertices[(i + 2) % len(vertices)], vertex)), normal)
            signs.append(turn)
            if any(dot(cross(edge, sub(p, vertex)), normal) < -EPS_MM for p in vertices):
                raise ValueError("polygon must be convex and non-self-intersecting")
        if min(signs) < -EPS_MM or max(signs) <= EPS_MM:
            raise ValueError("polygon must be convex and nondegenerate")

    @property
    def normal(self) -> Vec3:
        return self._normal

    def intersect(self, origin: Vec3, direction: Vec3, limit: float) -> Optional[SurfaceHit]:
        normal = self.normal
        denominator = dot(direction, normal)
        if abs(denominator) <= 1e-12:
            return None
        distance = dot(sub(self.vertices_mm[0], origin), normal) / denominator
        if distance < -EPS_MM or distance > limit + EPS_MM:
            return None
        distance = max(0.0, distance)
        point = tuple(a + distance * b for a, b in zip(origin, direction))
        for i, vertex in enumerate(self.vertices_mm):
            edge = sub(self.vertices_mm[(i + 1) % len(self.vertices_mm)], vertex)
            if dot(cross(edge, sub(point, vertex)), normal) < -EPS_MM:
                return None
        relative = sub(point, self.vertices_mm[0])
        return SurfaceHit(point, normal, distance, self.material_id,
                          (dot(relative, self._u), dot(relative, self._v)),
                          self.surface_id, True, "")


@dataclass(frozen=True)
class SceneSnapshot:
    surfaces: Tuple[PolygonSurface, ...]
    _cells: object = field(init=False, repr=False, compare=False)
    _bounds: tuple = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        surfaces = tuple(self.surfaces)
        if any(not isinstance(p, PolygonSurface) for p in surfaces):
            raise ValueError("unsupported surface")
        if len({p.surface_id for p in surfaces}) != len(surfaces):
            raise ValueError("surface IDs must be unique")
        object.__setattr__(self, "surfaces", surfaces)
        cells = {}
        references = 0
        for index, surface in enumerate(surfaces):
            low, high = surface._bounds
            x0, y0 = (math.floor((low[i] - EPS_MM) / 315) for i in (0, 1))
            x1, y1 = (math.floor((high[i] + EPS_MM) / 315) for i in (0, 1))
            references += (x1 - x0 + 1) * (y1 - y0 + 1)
            if references > 1_000_000:
                raise ValueError("scene spatial-index budget exceeded")
            for x in range(x0, x1 + 1):
                for y in range(y0, y1 + 1):
                    cells.setdefault((x, y), []).append(index)
        object.__setattr__(self, "_cells", MappingProxyType(
            {key: tuple(value) for key, value in cells.items()}))
        bounds = () if not surfaces else (
            tuple(min(s._bounds[0][i] for s in surfaces) for i in range(3)),
            tuple(max(s._bounds[1][i] for s in surfaces) for i in range(3)))
        object.__setattr__(self, "_bounds", bounds)

    @staticmethod
    def _clip(origin, ray, limit, bounds):
        low, high = bounds
        near, far = 0.0, limit
        for i in range(3):
            if abs(ray[i]) <= 1e-15:
                if origin[i] < low[i] - EPS_MM or origin[i] > high[i] + EPS_MM:
                    return None
            else:
                a = (low[i] - origin[i]) / ray[i]
                b = (high[i] - origin[i]) / ray[i]
                near, far = max(near, min(a, b)), min(far, max(a, b))
        return (near, max(near, far)) if near <= far + EPS_MM else None

    def _candidates(self, origin, ray, limit):
        interval = self._clip(origin, ray, limit, self._bounds) if self._bounds else None
        if interval is None:
            return ()
        near, far = interval
        cell = [math.floor((origin[i] + ray[i] * near) / 315) for i in (0, 1)]
        end = [math.floor((origin[i] + ray[i] * far) / 315) for i in (0, 1)]
        steps = [1 if ray[i] > 0 else -1 for i in (0, 1)]
        next_t, delta = [], []
        for i in (0, 1):
            if abs(ray[i]) <= 1e-15:
                next_t.append(math.inf)
                delta.append(math.inf)
            else:
                boundary = (cell[i] + (1 if steps[i] > 0 else 0)) * 315
                next_t.append((boundary - origin[i]) / ray[i])
                delta.append(315 / abs(ray[i]))
        found = set()
        while True:
            found.update(self._cells.get(tuple(cell), ()))
            if cell == end:
                break
            axis = 0 if next_t[0] <= next_t[1] else 1
            if next_t[axis] > far + EPS_MM:
                break
            cell[axis] += steps[axis]
            next_t[axis] += delta[axis]
        return tuple(sorted(found))

    @classmethod
    def from_patches(cls, patches):
        """Copy structural S3 PolygonPatch values; no runtime S3 dependency."""
        return cls(tuple(PolygonSurface(p.surface_id, p.material_id,
                                        tuple(p.vertices_mm), p.layer) for p in patches))

    def first_hit(self, origin_mm: Vec3, direction: Vec3,
                  max_distance_mm: float) -> SurfaceHit:
        try:
            origin = vec3(origin_mm)
            ray = unit(vec3(direction))
            limit = float(max_distance_mm)
            if not math.isfinite(limit) or limit <= 0:
                raise ValueError("invalid range")
        except (ValueError, TypeError, OverflowError):
            return SurfaceHit(reason="invalid_ray")
        candidates = []
        for index in self._candidates(origin, ray, limit):
            surface = self.surfaces[index]
            if self._clip(origin, ray, limit, surface._bounds) is None:
                continue
            hit = surface.intersect(origin, ray, limit)
            if hit is not None:
                candidates.append((surface.layer, hit))
        if not candidates:
            return SurfaceHit()
        nearest = min(hit.distance_mm for _, hit in candidates)
        tied = [(layer, hit) for layer, hit in candidates
                if abs(hit.distance_mm - nearest) <= EPS_MM]
        top_layer = max(layer for layer, _ in tied)
        tied = [hit for layer, hit in tied if layer == top_layer]
        if len({hit.material_id for hit in tied}) != 1:
            return SurfaceHit(reason="ambiguous_material")
        return min(tied, key=lambda hit: hit.surface_id)


def rectangle(surface_id: str, material_id: str, centre_mm: Vec3,
              size_mm: Vec2, rotation_deg: float = 0, layer: int = 0) -> PolygonSurface:
    centre = vec3(centre_mm)
    width, depth = map(float, size_mm)
    if not all(math.isfinite(x) and x > 0 for x in (width, depth)):
        raise ValueError("rectangle dimensions must be positive")
    if not math.isfinite(rotation_deg):
        raise ValueError("rotation must be finite")
    angle = math.radians(rotation_deg)
    c, s = math.cos(angle), math.sin(angle)
    points = tuple((centre[0] + c * x - s * y, centre[1] + s * x + c * y, centre[2])
                   for x, y in ((-width / 2, -depth / 2), (width / 2, -depth / 2),
                                (width / 2, depth / 2), (-width / 2, depth / 2)))
    return PolygonSurface(surface_id, material_id, points, layer)


def box_surfaces(surface_id: str, material_id: str, centre_mm: Vec3,
                 size_mm: Vec3, rotation_deg: float = 0) -> Tuple[PolygonSurface, ...]:
    centre, size = vec3(centre_mm), vec3(size_mm)
    if min(size) <= 0 or not math.isfinite(rotation_deg):
        raise ValueError("box size must be positive and rotation finite")
    angle = math.radians(rotation_deg)
    c, s = math.cos(angle), math.sin(angle)
    vertices = []
    for x, y, z in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)):
        x, y, z = x * size[0] / 2, y * size[1] / 2, z * size[2] / 2
        vertices.append((centre[0] + c * x - s * y, centre[1] + s * x + c * y,
                         centre[2] + z))
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    return tuple(PolygonSurface(f"{surface_id}:{i}", material_id,
                                tuple(vertices[j] for j in face))
                 for i, face in enumerate(faces))
