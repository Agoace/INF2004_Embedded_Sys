"""Deterministic self-contained Webots preview generation. No serial ports.

The PNGs come from SW4's canonical polygons, not a second line/barcode mask.
Only flat zero-thickness paper is baked into floor textures; raised paper gets
its own single textured top and collider, without coplanar ink polygons.
"""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile

from .course_builder import Course, build_course
from .robot_profile import RobotProfile

SOURCE = Path(__file__).resolve().parents[1]
REPO = SOURCE.parents[1]
COLOURS = {"MAT_WHITE": (0.93, 0.93, 0.93), "MAT_LINE": (0.067, 0.067, 0.067),
           "PAPER_WHITE": (1, 1, 1), "PAPER_INK": (0.067, 0.067, 0.067),
           "BOX": (0.45, 0.3, 0.15), "SUPPORT": (0.25, 0.25, 0.25)}


def floats(values):
    return " ".join(f"{x:.12g}" for x in values)


def metres(values):
    return floats(x / 1000 for x in values)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def padded_texture(rows, channels=1):
    """Pad with repeated edge pixels; keep canonical raster, avoid Webots rescale.

    UV cropping excludes padding. No material mask is generated a second time.
    """
    width, height = len(rows[0]) // channels, len(rows)
    padded_width = 1 << (width - 1).bit_length()
    padded_height = 1 << (height - 1).bit_length()
    padded = tuple(row + row[-channels:] * (padded_width - width) for row in rows)
    padded += (padded[-1],) * (padded_height - height)
    return padded, (width / padded_width, height / padded_height)


def mesh_shape(surfaces, material, *, texture=None, uv_bounds=None, texture_crop=(1, 1)):
    surfaces = tuple(surfaces)
    vertices, indices, texcoords = [], [], []
    for surface in surfaces:
        first = len(vertices)
        vertices.extend(surface.vertices_mm)
        indices.extend((*range(first, len(vertices)), -1))
        if texture is not None:
            x0, y0, x1, y1 = uv_bounds
            sx, sy = texture_crop
            texcoords.extend(((p[0] - x0) / (x1 - x0) * sx,
                              1 - sy + (p[1] - y0) / (y1 - y0) * sy)
                             for p in surface.vertices_mm)
    appearance = f"baseColor {floats(COLOURS[material])} roughness 1 metalness 0"
    if texture is not None:
        appearance = (f'baseColor 1 1 1 roughness 1 metalness 0 '
                      f'baseColorMap ImageTexture {{ url [ "../textures/{texture}" ] '
                      'repeatS FALSE repeatT FALSE }')
    texture_node = ""
    if texcoords:
        texture_node = "texCoord TextureCoordinate { point [ " + ", ".join(
            floats(uv) for uv in texcoords) + " ] }"
    coordinate = ", ".join(metres(p) for p in vertices)
    return ("Shape { appearance PBRAppearance { " + appearance + " } geometry IndexedFaceSet { "
            "coord Coordinate { point [ " + coordinate + " ] } coordIndex [ " +
            " ".join(map(str, indices)) + " ] " + texture_node + " } }")


def bounding(collider):
    if collider.kind == "hump":
        vertices, indices = [], []
        for surface in collider.surfaces:
            first = len(vertices)
            vertices.extend(surface.vertices_mm)
            indices.extend((*range(first, len(vertices)), -1))
        return ("IndexedFaceSet { coord Coordinate { point [ " +
                ", ".join(metres(p) for p in vertices) + " ] } coordIndex [ " +
                " ".join(map(str, indices)) + " ] }")
    return ("Pose { translation " + metres(collider.centre_mm) + " rotation 0 0 1 " +
            f"{math.radians(collider.rotation_deg):.12g}" +
            " children [ Box { size " + metres(collider.size_mm) + " } ] }")


def robot_node(profile, start):
    r = profile["wheel_diameter_mm"] / 2
    body_mass = (profile["total_mass_kg"] - 2 * profile["wheel_mass_kg"] -
                 profile["caster_mass_kg"] - profile["head_mass_kg"])
    fields = {
        "translation": metres((*start["position_mm"], 10 + r)),
        "rotation": "0 0 1 " + f'{math.radians(start["yaw_deg"]):.12g}',
        "track": profile["track_mm"] / 1000,
        "wheelRadius": r / 1000,
        "wheelMass": profile["wheel_mass_kg"],
        "wheelTorqueLimit": profile["stall_torque_nm"],
        "bodyMass": body_mass,
        "bodySize": metres((profile["body_length_mm"], profile["body_width_mm"],
                             profile["body_height_mm"])),
        "bodyCentre": metres((profile["body_front_mm"] - profile["body_length_mm"] / 2,
                               0, profile["body_centre_height_mm"])),
        "centreOfMass": "[ " + metres((profile["com_forward_mm"], 0,
                                        profile["com_axle_height_mm"])) + " ]",
        "casterMass": profile["caster_mass_kg"],
        "casterRadius": profile["caster_radius_mm"] / 1000,
        "casterPosition": metres((profile["caster_forward_mm"], 0,
                                   profile["caster_radius_mm"] - r)),
        "headMass": profile["head_mass_kg"],
        "servoPosition": metres((profile["servo_axis_forward_mm"], 0,
                                  profile["servo_axis_axle_height_mm"])),
        "sonarFaceOffset": profile["sonar_face_offset_mm"] / 1000,
        "servoSlew": math.radians(profile["servo_slew_deg_s"]),
        "lineCentrePosition": metres(profile.mount_mm("line_c")),
        "lineLeftPosition": metres(profile.mount_mm("line_l")),
        "barcodePosition": metres(profile.mount_mm("barcode")),
        "imuPosition": metres(profile.mount_mm("imu")),
    }
    return "RoboPico {\n" + "\n".join(
        f"  {key} {value if isinstance(value, str) else format(value, '.12g')}"
        for key, value in fields.items()) + "\n}"


def project_files(built, profile):
    from . import s3_assets, s3_geometry
    from .s3_assets import png_bytes, raster_rows
    from .scene import PolygonSurface, rectangle

    data, files = built.course.data, {}
    images, top_shapes = {}, {}
    zero_sheets = {sheet["id"] for sheet in data["barcodes"]
                   if sheet["paper_thickness_mm"] == 0 and sheet["centre_mm"][2] == 10}
    # Only patches physically on the tile top may enter its texture. A box/hump
    # never becomes a painted floor mark. Elevated paper stays a distinct plane.
    floor_patches = tuple(p for p in built.patches if p.surface_id.split(":")[0] in
                          zero_sheets or p.layer in (0, 10))
    for tile_id, centre, geometry in built.tile_geometries:
        x, y, z = centre
        bounds = (x - 157.5, y - 157.5, x + 157.5, y + 157.5)
        local = tuple(p for p in floor_patches if
                      max(v[0] for v in p.vertices_mm) >= bounds[0] and
                      min(v[0] for v in p.vertices_mm) <= bounds[2] and
                      max(v[1] for v in p.vertices_mm) >= bounds[1] and
                      min(v[1] for v in p.vertices_mm) <= bounds[3])
        image = tile_id + ".png"
        raster = raster_rows(local, bounds, data["texture_mm_per_pixel"])
        padded, crop = padded_texture(raster)
        images[image] = png_bytes(padded)
        top = rectangle(tile_id + "-visual", "MAT_WHITE", (x, y, z), (315, 315))
        top_shapes[tile_id + "-solid"] = mesh_shape((top,), "MAT_WHITE", texture=image,
                                                    uv_bounds=bounds, texture_crop=crop)
    # Paper top is one texture; ink is never an additional coplanar render mesh.
    paper_shapes = []
    for sheet in data["barcodes"]:
        if sheet["id"] in zero_sheets:
            continue
        patches = tuple(p for p in built.patches if p.surface_id.startswith(sheet["id"] + ":"))
        vertices = tuple(v for patch in patches for v in patch.vertices_mm)
        bounds = (min(v[0] for v in vertices), min(v[1] for v in vertices),
                  max(v[0] for v in vertices), max(v[1] for v in vertices))
        image = sheet["id"] + ".png"
        raster = raster_rows(patches, bounds, data["texture_mm_per_pixel"], transparent=True)
        padded, crop = padded_texture(raster, channels=2)
        images[image] = png_bytes(padded, channels=2)
        paper = next(p for p in patches if p.material_id == "PAPER_WHITE")
        top = PolygonSurface(paper.surface_id, paper.material_id, paper.vertices_mm, paper.layer)
        shape = mesh_shape((top,), "PAPER_WHITE", texture=image, uv_bounds=bounds,
                           texture_crop=crop)
        if sheet["paper_thickness_mm"]:
            top_shapes[sheet["id"] + "-solid"] = shape
        else:
            paper_shapes.append(shape)
    support = next(c for c in built.colliders if c.kind == "support")
    cx, cy, _ = support.centre_mm
    height = max(support.size_mm[:2]) / 1000 + 0.5
    rows = ["#VRML_SIM R2025a utf8", 'EXTERNPROTO "../protos/RoboPico.proto"',
            "# Generated Stage0 PREVIEW ONLY; all assumptions remain in manifest.json.",
            'WorldInfo { basicTimeStep 1 coordinateSystem "ENU" contactProperties [',
            '  ContactProperties { material1 "tyre" material2 "course" coulombFriction [ ' +
            f'{profile["contact_friction"]:.12g}' + ' ] bounce 0 } ] }',
            f"Viewpoint {{ position {cx / 1000:.12g} {cy / 1000:.12g} {height:.12g} "
            "orientation 0 0 1 0 }",
            "Background { skyColor [ 0.15 0.15 0.15 ] }",
            "DirectionalLight { direction -0.5 -0.5 -1 intensity 1 }"]
    for collider in built.colliders:
        if collider.kind in ("tile", "paper"):
            # Sidewalls only; a solid's untextured top must not fight its texture.
            sides = tuple(p for p in collider.surfaces if abs(p.normal[2]) < 0.9)
            visual = top_shapes[collider.surface_id] + mesh_shape(sides, collider.material_id)
        else:
            visual = mesh_shape(collider.surfaces, collider.material_id)
        rows.append('Solid { name ' + json.dumps(collider.surface_id) +
                    ' contactMaterial "course" children [ ' + visual +
                    " ] boundingObject " + bounding(collider) + " }")
    rows.extend(paper_shapes)
    rows.append(robot_node(profile, data["start"]))
    files["worlds/" + data["name"] + ".wbt"] = ("\n".join(rows) + "\n").encode()
    for image, content in images.items():
        files["textures/" + image] = content
    for relative in ("protos/RoboPico.proto", "controllers/stage0_preview/stage0_preview.py"):
        files[relative] = (SOURCE / relative).read_bytes()
    files["course.json"] = (built.course.canonical_json + "\n").encode()
    source_hashes = {str(p.relative_to(SOURCE)): sha(p.read_bytes()) for p in
                     sorted((SOURCE / "pil").glob("*.py"))}
    for name, module in (("s3_assets", s3_assets), ("s3_geometry", s3_geometry)):
        source_hashes["pil/" + name + ".py"] = sha(Path(module.__file__).read_bytes())
    manifest = {
        "schema": 1, "owner": "webots-pil-stage0", "webots_header": "R2025a",
        "webots_executed": False, "physical_transport_qualified": False,
        "course": data, "robot_profile_id": profile.profile_id,
        "robot_parameters": profile.assumption_manifest(),
        "assumptions": built.assumptions, "port_issues": built.port_issues,
        "proto_extra_assumptions": {
            "wheel_width_mm": 16, "caster_width_mm": 6, "caster_trail_mm": 8,
            "caster_mass_fraction_wheel": 0.75, "caster_swivel_damping": 0.0001,
            "caster_wheel_damping": 0.00001, "servo_max_torque_nm": 0.1,
            "sonar_head_size_mm": [30, 35, 20], "caster_swivel_radius_mm": 3},
        "canonical_geometry_sources_sha256": {
            name: source_hashes[name] for name in ("pil/s3_geometry.py", "pil/s3_assets.py")},
        "sources_sha256": source_hashes,
        "artifacts_sha256": {name: sha(content) for name, content in sorted(files.items())},
        "curve_max_chord_error_mm": max(g.max_chord_error_mm for _, _, g in
                                        built.tile_geometries),
        "ports": [{"tile_id": name, **asdict(port)} for name, _, g in built.tile_geometries
                  for port in g.ports],
        "texture_padding": "canonical raster edge-padded to power-of-two; cropped UV, no rescale",
        "warning": "PREVIEW ONLY. No Pico runtime, firmware verdict or hardware timing proof.",
    }
    files["manifest.json"] = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    return files


def write_project(course, profile, output):
    output = Path(output).resolve()
    build = (REPO / "build").resolve()
    if output == build or not output.is_relative_to(build):
        raise ValueError("generated projects must be below this repository's build/")
    if output.exists():
        raise ValueError("output exists; select a new generated project directory")
    files = project_files(build_course(course), profile)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".webots-stage-", dir=output.parent))
    try:
        for name, content in files.items():
            target = stage / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        stage.rename(output)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = write_project(Course.load(args.course), RobotProfile.load(args.robot), args.output)
    print(f"Generated {output}; PREVIEW ONLY, Webots execution not validated")


if __name__ == "__main__":
    main()
