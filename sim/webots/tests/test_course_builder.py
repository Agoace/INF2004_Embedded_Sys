import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

from pil.course_builder import Course, build_course
from pil.robot_profile import RobotProfile
from pil.world import project_files, write_project

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT.parents[1] / "build/wip/pil_plan/course-tests"
HAS_S3 = all(importlib.util.find_spec("pil." + name) is not None
             for name in ("s3_geometry", "s3_assets"))


def sample_course():
    return json.loads((ROOT / "courses/home_mat_1_assumed.json").read_text())


def gray_pixel(png, x, y):
    cursor, compressed, width, height = 8, bytearray(), None, None
    while cursor < len(png):
        size = struct.unpack_from(">I", png, cursor)[0]
        kind, payload = png[cursor + 4:cursor + 8], png[cursor + 8:cursor + 8 + size]
        if kind == b"IHDR":
            width, height, depth, colour = struct.unpack_from(">IIBB", payload)
            assert (depth, colour) == (8, 0)
        elif kind == b"IDAT":
            compressed.extend(payload)
        cursor += size + 12
    raw = zlib.decompress(compressed)
    assert 0 <= x < width and 0 <= y < height
    assert raw[y * (width + 1)] == 0
    return raw[y * (width + 1) + 1 + x]


class SchemaTests(unittest.TestCase):
    def test_all_examples_are_explicit_assumptions(self):
        for path in (ROOT / "courses").glob("*.json"):
            course = Course.load(path)
            self.assertTrue(course.data["provenance"]["assumed"])
            self.assertIn("assumed", course.name)
            self.assertIn("NOT", course.data["provenance"]["source"])

    def test_canonical_course_immutable_from_caller(self):
        data = sample_course()
        course = Course.from_dict(data)
        data["tiles"].clear()
        self.assertEqual(len(course.data["tiles"]), 4)
        view = course.data
        view["tiles"].clear()
        self.assertEqual(len(course.data["tiles"]), 4)

    def test_invalid_schema_indices_rotation_and_unknown_fields(self):
        changes = (
            lambda d: d.update(schema=True),
            lambda d: d.update(extra=1),
            lambda d: d.update(name="../escape"),
            lambda d: d["provenance"].update(source=""),
            lambda d: d.update(allow_open_ports=1),
            lambda d: d.update(origin_mm=[0, float("nan")]),
            lambda d: d.update(texture_mm_per_pixel=0),
            lambda d: d["tiles"][0].update(column=True),
            lambda d: d["tiles"][0].update(rotation_deg=45),
            lambda d: d["tiles"][0].update(kind="NOT_A_TILE"),
            lambda d: d["tiles"].append(copy.deepcopy(d["tiles"][0])),
            lambda d: d["tiles"][1].update(column=0),
            lambda d: d["tiles"][0].update(station_ports="NS"),
            lambda d: d["tiles"][0].update(kind="STATION"),
        )
        for change in changes:
            data = sample_course()
            change(data)
            with self.subTest(change=change), self.assertRaises(ValueError):
                Course.from_dict(data)

    def test_paper_and_box_input_limits(self):
        original = json.loads((ROOT / "courses/school_like_assumed.json").read_text())
        changes = (
            lambda d: d["boxes"][0].update(size_mm=[1, -1, 2]),
            lambda d: d["boxes"][0].update(material_id="INVENTED"),
            lambda d: d["barcodes"][0].update(letter="*"),
            lambda d: d["barcodes"][0].update(centre_mm=[600, 234.5, 11]),
            lambda d: d["barcodes"][0].update(paper_thickness_mm=-1),
            lambda d: d["humps"][0].update(segments=1000000),
        )
        for change in changes:
            data = copy.deepcopy(original)
            change(data)
            with self.subTest(change=change), self.assertRaises(ValueError):
                Course.from_dict(data)


@unittest.skipUnless(HAS_S3, "requires SW4 canonical modules; add their checkout to PYTHONPATH")
class CourseIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = RobotProfile.load(ROOT / "calibration/robot_assumed.json")
        cls.school = Course.load(ROOT / "courses/school_like_assumed.json")
        cls.built = build_course(cls.school)
        cls.files = project_files(cls.built, cls.profile)
        BUILD.mkdir(parents=True, exist_ok=True)

    def test_all_three_courses_connect_without_warning(self):
        for path in sorted((ROOT / "courses").glob("*.json")):
            built = build_course(Course.load(path))
            self.assertEqual(built.port_issues, ())
            self.assertTrue(built.scene.surfaces)

    def test_port_mismatch_and_explicit_open_end(self):
        data = sample_course()
        data["tiles"][0]["rotation_deg"] = 90
        with self.assertRaises(ValueError):
            build_course(Course.from_dict(data))
        data["allow_open_ports"] = True
        self.assertGreater(len(build_course(Course.from_dict(data)).port_issues), 0)

    def test_station_requires_marker_and_ports(self):
        data = sample_course()
        data["tiles"][0].update(kind="STATION", station_ports="NE",
                                station_marker_mm=[[-20, -20], [20, -20], [20, 20], [-20, 20]])
        built = build_course(Course.from_dict(data))
        self.assertEqual(built.port_issues, ())
        self.assertIn("station_marker_assumed", built.assumptions)

    def test_negative_origin_and_quarter_turn_ports(self):
        data = sample_course()
        data["origin_mm"] = [-1000, -2000]
        built = build_course(Course.from_dict(data))
        self.assertEqual(built.port_issues, ())
        hit = built.scene.first_hit((-1000 + 315, -2000 + 157.5, 50), (0, 0, -1), 100)
        self.assertEqual(hit.material_id, "MAT_LINE")

    def test_hump_and_box_first_hit_do_not_see_floor(self):
        scene = self.built.scene
        hit = scene.first_hit((470, 102, 50), (0, 0, -1), 100)
        self.assertAlmostEqual(hit.point_mm[2], 20)
        self.assertTrue(hit.surface_id.startswith("right_wheel_hump:"))
        hit = scene.first_hit((700, 500, 200), (0, 0, -1), 300)
        self.assertEqual(hit.material_id, "BOX")
        self.assertAlmostEqual(hit.point_mm[2], 110)
        self.assertEqual(scene.first_hit((-100, 100, 50), (0, 0, -1), 100).material_id,
                         "SUPPORT")

    def test_paper_crosses_tiles_without_squeezing(self):
        scene = self.built.scene
        self.assertEqual(scene.first_hit((410, 234.5, 50), (0, 0, -1), 100).material_id,
                         "PAPER_WHITE")
        self.assertEqual(scene.first_hit((464.25, 234.5, 50), (0, 0, -1), 100).material_id,
                         "PAPER_INK")
        # Sheet spans the x=630mm tile seam and leaves >58mm quiet on both ends.
        self.assertEqual(scene.first_hit((790, 234.5, 50), (0, 0, -1), 100).material_id,
                         "PAPER_WHITE")

    def test_unsupported_paper_overlap_extent_and_body_size_fail(self):
        for case in ("overlap", "extent", "body"):
            data = self.school.data
            if case == "overlap":
                data["barcodes"][1].update(centre_mm=data["barcodes"][0]["centre_mm"],
                                           rotation_deg=0)
            elif case == "extent":
                data["barcodes"][0]["paper_size_mm"] = [3000, 50]
            else:
                data["barcodes"][0]["paper_size_mm"] = [200, 50]
            with self.subTest(case=case), self.assertRaises(ValueError):
                build_course(Course.from_dict(data))

    def test_raised_paper_has_matching_collider_and_texture(self):
        data = self.school.data
        data["barcodes"][0]["paper_thickness_mm"] = 0.1
        built = build_course(Course.from_dict(data))
        self.assertAlmostEqual(built.scene.first_hit(
            (410, 234.5, 50), (0, 0, -1), 100).point_mm[2], 10.1)
        files = project_files(built, self.profile)
        self.assertIn("textures/barcode_a.png", files)
        world = files["worlds/school_like_assumed.wbt"].decode()
        self.assertIn('name "barcode_a-solid"', world)

    def test_box_not_baked_onto_floor(self):
        # Under the opaque box, x700/y500 is floor WHITE, not the box material.
        image = self.files["textures/cross.png"]
        x = int((700 - 630) / 0.5)
        y = int((630 - 500) / 0.5)
        self.assertEqual(gray_pixel(image, x, y), 238)

    def test_power_of_two_texture_padding_and_supported_mesh_fields(self):
        for name, image in self.files.items():
            if name.endswith(".png"):
                width, height = struct.unpack_from(">II", image, 16)
                self.assertEqual(width & (width - 1), 0)
                self.assertEqual(height & (height - 1), 0)
        world = self.files["worlds/school_like_assumed.wbt"].decode()
        self.assertNotIn("solid FALSE", world)
        self.assertIn("texCoord TextureCoordinate", world)

    def test_project_deterministic_self_contained_and_hashed(self):
        self.assertEqual(self.files, project_files(self.built, self.profile))
        manifest = json.loads(self.files["manifest.json"])
        for name, digest in manifest["artifacts_sha256"].items():
            self.assertEqual(hashlib.sha256(self.files[name]).hexdigest(), digest)
        self.assertLessEqual(manifest["curve_max_chord_error_mm"], 0.01)
        self.assertFalse(manifest["physical_transport_qualified"])
        self.assertFalse(manifest["webots_executed"])
        world = self.files["worlds/school_like_assumed.wbt"].decode()
        self.assertIn('EXTERNPROTO "../protos/RoboPico.proto"', world)
        self.assertIn('basicTimeStep 1 coordinateSystem "ENU"', world)
        self.assertIn("translation 0.35 0.1575 0.0425", world)
        self.assertIn("barcodePosition 0.08 0.077 -0.027", world)
        self.assertNotIn("/home/", world)
        self.assertNotIn("http", world)
        self.assertIn("protos/RoboPico.proto", self.files)
        preview = self.files["controllers/stage0_preview/stage0_preview.py"].decode()
        self.assertNotIn("serial", preview)
        self.assertNotIn("setVelocity", preview)
        self.assertIn("setTorque(0.0)", preview)

    def test_output_is_build_only_and_never_overwrites(self):
        with tempfile.TemporaryDirectory(dir=BUILD) as tmp:
            output = Path(tmp) / "project"
            write_project(self.school, self.profile, output)
            sentinel = output / "do-not-overwrite"
            sentinel.write_text("keep")
            with self.assertRaises(ValueError):
                write_project(self.school, self.profile, output)
            self.assertEqual(sentinel.read_text(), "keep")
        with self.assertRaises(ValueError):
            write_project(self.school, self.profile, ROOT / "must-not-be-created")


if __name__ == "__main__":
    unittest.main()
