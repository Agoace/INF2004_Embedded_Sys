"""Versioned JSON grids -> canonical S3 material scene and Webots colliders.

Layouts marked assumed are illustrative, NOT surveyed home/school mat replicas.
No simulator state is an input to the firmware-output verdict evaluator.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re

from .scene import PolygonSurface, SceneSnapshot, box_surfaces, cross, dot, sub

TILE_MM = 315.0
TOP_MM = 10.0
KINDS = frozenset(("STRAIGHT", "CURVE", "CROSS", "T", "BLANK", "STATION"))
MATERIALS = frozenset(("MAT_WHITE", "MAT_LINE", "PAPER_WHITE", "PAPER_INK", "BOX"))
IDENTIFIER = re.compile(r"[a-z][a-z0-9_-]{0,47}\Z")


def keys(data, required, optional=()):
    if (not isinstance(data, dict) or
            not set(required) <= set(data) <= set(required) | set(optional)):
        raise ValueError("unknown or missing course field")


def number(value, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("course quantity must be finite numeric")
    if abs(value) > 1_000_000 or (positive and value <= 0):
        raise ValueError("course quantity outside supported range")
    return float(value)


def vector(value, count):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError("invalid vector size")
    return tuple(number(x) for x in value)


def identifier(value):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError("invalid identifier")


@dataclass(frozen=True)
class Course:
    canonical_json: str

    @property
    def data(self):
        return json.loads(self.canonical_json)

    @property
    def name(self):
        return self.data["name"]

    @classmethod
    def load(cls, path):
        path = Path(path)
        if path.stat().st_size > 1_000_000:
            raise ValueError("course JSON exceeds1MB")
        return cls.from_dict(json.loads(path.read_text(encoding="utf-8")))

    @classmethod
    def from_dict(cls, data):
        keys(data, ("schema", "name", "provenance", "origin_mm", "tiles", "start",
                    "allow_open_ports", "boxes", "humps", "barcodes", "texture_mm_per_pixel"))
        if type(data["schema"]) is not int or data["schema"] != 1:
            raise ValueError("unknown course schema")
        identifier(data["name"])
        keys(data["provenance"], ("assumed", "source"))
        if type(data["provenance"]["assumed"]) is not bool or not isinstance(
                data["provenance"]["source"], str) or not data["provenance"]["source"].strip():
            raise ValueError("course provenance required")
        if type(data["allow_open_ports"]) is not bool:
            raise ValueError("allow_open_ports must be explicit boolean")
        vector(data["origin_mm"], 2)
        keys(data["start"], ("position_mm", "yaw_deg"))
        vector(data["start"]["position_mm"], 2)
        number(data["start"]["yaw_deg"])
        resolution = number(data["texture_mm_per_pixel"], positive=True)
        if not 0.1 <= resolution <= 2:
            raise ValueError("texture spacing must be0.1..2mm; visual only")
        limits = {"tiles": 256, "boxes": 128, "humps": 32, "barcodes": 26}
        ids, occupied = set(), set()
        for group, limit in limits.items():
            if not isinstance(data[group], list) or len(data[group]) > limit:
                raise ValueError("course list size/type invalid")
            for item in data[group]:
                if not isinstance(item, dict):
                    raise ValueError("course item must be an object")
                identifier(item.get("id"))
                if item["id"] in ids:
                    raise ValueError("course IDs must be globally unique")
                ids.add(item["id"])
        if not data["tiles"] or len(data["tiles"]) * math.ceil(315 / resolution) ** 2 > 100_000_000:
            raise ValueError("empty course or texture budget exceeded")
        for tile in data["tiles"]:
            keys(tile, ("id", "column", "row", "kind", "rotation_deg"),
                 ("station_marker_mm", "station_ports"))
            if any(type(tile[k]) is not int or abs(tile[k]) > 64 for k in ("column", "row")):
                raise ValueError("grid indices must be integers in[-64,64]")
            cell = (tile["column"], tile["row"])
            if cell in occupied:
                raise ValueError("duplicate grid cell")
            occupied.add(cell)
            if not isinstance(tile["kind"], str) or tile["kind"] not in KINDS or (
                    type(tile["rotation_deg"]) is not int) or (
                    tile["rotation_deg"] not in (0, 90, 180, 270)):
                raise ValueError("unknown tile or rotation")
            if tile["kind"] == "STATION":
                if "station_marker_mm" not in tile or "station_ports" not in tile:
                    raise ValueError("STATION requires explicit marker and ports")
                if not isinstance(tile["station_marker_mm"], list):
                    raise ValueError("station marker must be a polygon")
                if not isinstance(tile["station_ports"], (str, list, tuple)):
                    raise ValueError("station ports must be a cardinal sequence")
                for point in tile["station_marker_mm"]:
                    vector(point, 2)
            elif "station_marker_mm" in tile or "station_ports" in tile:
                raise ValueError("station fields require STATION")
        for box in data["boxes"]:
            keys(box, ("id", "centre_mm", "size_mm", "rotation_deg", "material_id"))
            vector(box["centre_mm"], 3)
            if min(vector(box["size_mm"], 3)) <= 0:
                raise ValueError("box dimensions must be positive")
            number(box["rotation_deg"])
            if not isinstance(box["material_id"], str) or box["material_id"] not in MATERIALS:
                raise ValueError("unknown box material")
        for hump in data["humps"]:
            keys(hump, ("id", "centre_mm", "length_mm", "width_mm", "height_mm",
                        "segments", "rotation_deg", "material_id"))
            vector(hump["centre_mm"], 3)
            for key in ("length_mm", "width_mm", "height_mm"):
                number(hump[key], positive=True)
            if type(hump["segments"]) is not int or not 4 <= hump["segments"] <= 128:
                raise ValueError("hump segments must be4..128")
            number(hump["rotation_deg"])
            if not isinstance(hump["material_id"], str) or hump["material_id"] not in MATERIALS:
                raise ValueError("unknown hump material")
        for sheet in data["barcodes"]:
            keys(sheet, ("id", "letter", "centre_mm", "rotation_deg", "bar_height_mm",
                         "paper_size_mm", "paper_thickness_mm", "reverse"))
            if not isinstance(sheet["letter"], str) or not re.fullmatch("[A-Z]", sheet["letter"]):
                raise ValueError("barcode must be one uppercase letter")
            if vector(sheet["centre_mm"], 3)[2] != TOP_MM:
                raise ValueError("paper must rest on flat EVA; draping/floating unsupported")
            number(sheet["rotation_deg"])
            number(sheet["bar_height_mm"], positive=True)
            if min(vector(sheet["paper_size_mm"], 2)) <= 0:
                raise ValueError("explicit positive paper dimensions required")
            if number(sheet["paper_thickness_mm"]) < 0 or type(sheet["reverse"]) is not bool:
                raise ValueError("invalid paper thickness/reverse")
        return cls(json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False))


@dataclass(frozen=True)
class Collider:
    surface_id: str
    material_id: str
    surfaces: tuple
    kind: str
    centre_mm: tuple = ()
    size_mm: tuple = ()
    rotation_deg: float = 0


@dataclass(frozen=True)
class BuiltCourse:
    course: Course
    scene: SceneSnapshot
    patches: tuple
    tile_geometries: tuple
    colliders: tuple
    port_issues: tuple
    assumptions: tuple


def hump_surfaces(hump):
    """Closed sine-profile prism. Shape/segmentation are explicit assumptions."""
    x, y, z = vector(hump["centre_mm"], 3)
    length, width, height = (hump[k] for k in ("length_mm", "width_mm", "height_mm"))
    count = hump["segments"]
    angle = math.radians(hump["rotation_deg"])
    c, s = math.cos(angle), math.sin(angle)

    def world(a, b, h):
        return (x + c * a - s * b, y + s * a + c * b, z + h)

    triangles = []

    def add(points):
        area = cross(sub(points[1], points[0]), sub(points[2], points[0]))
        if dot(area, area) > 1e-14:
            triangles.append(PolygonSurface(f'{hump["id"]}:tri{len(triangles):03}',
                                            hump["material_id"], tuple(points)))

    for i in range(count):
        a, b = length * (i / count - 0.5), length * ((i + 1) / count - 0.5)
        ha = height * math.sin(math.pi * i / count)
        hb = height * math.sin(math.pi * (i + 1) / count)
        left, right = -width / 2, width / 2
        p0, p1, p2, p3 = world(a, left, ha), world(b, left, hb), world(b, right, hb), (
            world(a, right, ha))
        add((p0, p1, p2))
        add((p0, p2, p3))
        q0, q1 = world(a, left, 0), world(b, left, 0)
        add((q0, q1, p1))
        add((q0, p1, p0))
        q2, q3 = world(b, right, 0), world(a, right, 0)
        add((q2, q3, p3))
        add((q2, p3, p2))
    corners = (world(-length / 2, -width / 2, 0), world(-length / 2, width / 2, 0),
               world(length / 2, width / 2, 0), world(length / 2, -width / 2, 0))
    add((corners[0], corners[1], corners[2]))
    add((corners[0], corners[2], corners[3]))
    return tuple(triangles)


def overlap_xy(a, b):
    """Strict positive-area overlap of convex XY polygons (touching is not overlap)."""
    for polygon in (a, b):
        for i, point in enumerate(polygon):
            nxt = polygon[(i + 1) % len(polygon)]
            axis = (-(nxt[1] - point[1]), nxt[0] - point[0])
            pa = tuple(p[0] * axis[0] + p[1] * axis[1] for p in a)
            pb = tuple(p[0] * axis[0] + p[1] * axis[1] for p in b)
            if min(max(pa), max(pb)) - max(min(pa), min(pb)) <= 1e-7:
                return False
    return True


def build_course(course: Course) -> BuiltCourse:
    # Dependency owned by SW4. Never substitute a second geometry implementation.
    from .s3_geometry import barcode_geometry, tile_geometry

    data = course.data
    ox, oy = data["origin_mm"]
    patches, tile_geometries, colliders, assumptions = [], [], [], []
    geometry_by_cell = {}
    for tile in data["tiles"]:
        centre = (ox + (tile["column"] + 0.5) * TILE_MM,
                  oy + (tile["row"] + 0.5) * TILE_MM, TOP_MM)
        geometry = tile_geometry(tile["kind"], centre_mm=centre,
                                 rotation_deg=tile["rotation_deg"], tile_id=tile["id"],
                                 station_marker_mm=tile.get("station_marker_mm"),
                                 station_ports=tile.get("station_ports"))
        patches.extend(geometry.patches)
        tile_geometries.append((tile["id"], centre, geometry))
        geometry_by_cell[(tile["column"], tile["row"])] = (tile["id"], geometry)
        assumptions.extend(geometry.assumptions)
        box = box_surfaces(tile["id"] + "-solid", "MAT_WHITE",
                           (centre[0], centre[1], 5), (315, 315, 10))
        colliders.append(Collider(tile["id"] + "-solid", "MAT_WHITE", box, "tile",
                                  (centre[0], centre[1], 5), (315, 315, 10)))
    directions = {"N": (0, 1, "S"), "E": (1, 0, "W"),
                  "S": (0, -1, "N"), "W": (-1, 0, "E")}
    issues = []
    for cell, (tile_id, geometry) in geometry_by_cell.items():
        for port in geometry.ports:
            dx, dy, opposite = directions[port.name]
            neighbor = geometry_by_cell.get((cell[0] + dx, cell[1] + dy))
            peer = next((p for p in neighbor[1].ports if p.name == opposite), None) if (
                neighbor is not None) else None
            if peer is None or math.dist(port.point_mm, peer.point_mm) > 1e-6 or (
                    abs(port.width_mm - peer.width_mm) > 1e-6):
                issues.append(f"{tile_id}:{port.name} unconnected/mismatched port")
    if issues and not data["allow_open_ports"]:
        raise ValueError("course port mismatch: " + issues[0])
    paper_footprints = []
    for sheet in data["barcodes"]:
        geometry = barcode_geometry(
            sheet["letter"], bar_height_mm=sheet["bar_height_mm"],
            centre_mm=sheet["centre_mm"], rotation_deg=sheet["rotation_deg"],
            paper_size_mm=sheet["paper_size_mm"], paper_thickness_mm=sheet["paper_thickness_mm"],
            reverse=sheet["reverse"], surface_id=sheet["id"])
        paper = next(p for p in geometry.patches if p.material_id == "PAPER_WHITE")
        if any(overlap_xy(paper.vertices_mm, old) for old in paper_footprints):
            raise ValueError("overlapping paper sheets need an explicit stacking model")
        paper_footprints.append(paper.vertices_mm)
        min_col = math.floor((min(v[0] for v in paper.vertices_mm) - ox) / 315)
        max_col = math.floor((max(v[0] for v in paper.vertices_mm) - ox - 1e-7) / 315)
        min_row = math.floor((min(v[1] for v in paper.vertices_mm) - oy) / 315)
        max_row = math.floor((max(v[1] for v in paper.vertices_mm) - oy - 1e-7) / 315)
        if (max_col - min_col + 1) * (max_row - min_row + 1) > 1024:
            raise ValueError("paper footprint exceeds course support budget")
        for column in range(min_col, max_col + 1):
            for row in range(min_row, max_row + 1):
                x0, y0 = ox + column * 315, oy + row * 315
                cell = ((x0, y0), (x0 + 315, y0),
                        (x0 + 315, y0 + 315), (x0, y0 + 315))
                if (column, row) not in geometry_by_cell and overlap_xy(paper.vertices_mm, cell):
                    raise ValueError("paper extends beyond supported EVA tiles")
        patches.extend(geometry.patches)
        assumptions.extend(geometry.assumptions)
        if sheet["paper_size_mm"][0] < geometry.quiet_length_mm:
            assumptions.append(f'{sheet["id"]}: paper shorter than full quiet length; '
                               'surrounding material must be evaluated, not assumed blank')
        if sheet["paper_thickness_mm"] > 0:
            cx, cy, cz = sheet["centre_mm"]
            thickness = sheet["paper_thickness_mm"]
            size = (*sheet["paper_size_mm"], thickness)
            centre = (cx, cy, cz + thickness / 2)
            box = box_surfaces(sheet["id"] + "-solid", "PAPER_WHITE", centre,
                               size, sheet["rotation_deg"])
            colliders.append(Collider(sheet["id"] + "-solid", "PAPER_WHITE", box,
                                      "paper", centre, size, sheet["rotation_deg"]))
    for box in data["boxes"]:
        surfaces = box_surfaces(box["id"], box["material_id"], box["centre_mm"],
                                box["size_mm"], box["rotation_deg"])
        colliders.append(Collider(box["id"], box["material_id"], surfaces, "box",
                                  tuple(box["centre_mm"]), tuple(box["size_mm"]),
                                  box["rotation_deg"]))
    for hump in data["humps"]:
        surfaces = hump_surfaces(hump)
        colliders.append(Collider(hump["id"], hump["material_id"], surfaces, "hump"))
        assumptions.append(f'{hump["id"]}: static tessellated sine-profile prism; '
                           'uniform material; not surveyed or painted-line draping')
    materials = tuple(PolygonSurface(p.surface_id, p.material_id, p.vertices_mm, p.layer)
                      for p in patches)
    vertices = tuple(v for patch in materials for v in patch.vertices_mm)
    low = tuple(min(v[i] for v in vertices) - 315 for i in (0, 1))
    high = tuple(max(v[i] for v in vertices) + 315 for i in (0, 1))
    centre = ((low[0] + high[0]) / 2, (low[1] + high[1]) / 2, -5)
    size = (high[0] - low[0], high[1] - low[1], 10)
    support = box_surfaces("support", "SUPPORT", centre, size)
    colliders.append(Collider("support", "SUPPORT", support, "support", centre, size))
    solids = tuple(s for collider in colliders for s in collider.surfaces)
    scene = SceneSnapshot(materials + solids)
    return BuiltCourse(course, scene, tuple(patches), tuple(tile_geometries), tuple(colliders),
                       tuple(sorted(issues)), tuple(sorted(set(assumptions))))
