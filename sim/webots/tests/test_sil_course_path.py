import math
import unittest

from pil.course_builder import Course
from pil.sil_course8 import course8_course, COURSE_REVISION
from pil.sil_course_path import CoursePath, PathFollower, TILE_MM


def straight_course():
    tiles = [dict(id=f't{c}', column=c, row=0, kind='STRAIGHT',
                  rotation_deg=90) for c in range(4)]
    return Course.from_dict(dict(
        schema=1, name='path_line', provenance=dict(assumed=True, source='t'),
        origin_mm=[0, 0], start=dict(position_mm=[100, 157.5], yaw_deg=0),
        allow_open_ports=True, texture_mm_per_pixel=2, tiles=tiles, boxes=[],
        humps=[], barcodes=[]))


class Course8PathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.course = course8_course()
        cls.path = CoursePath.from_course(cls.course)

    def test_pinned_overlay_and_scenario(self):
        self.assertEqual(COURSE_REVISION,
                         '3b43a1952bbb65f86cc942ee13f41aaf6280e7bb')
        data = self.course.data
        self.assertEqual((data['boxes'], data['barcodes']), ([], []))
        self.assertEqual(len(data['humps']), 1)
        hump = data['humps'][0]
        self.assertEqual((hump['length_mm'], hump['width_mm'],
                          hump['height_mm']), (195.0, 50.0, 10.0))
        # Photo start spot, driven east (L1r), so L2r follows: the ramp at
        # L2r:4790 R is under the right tyre.
        x, y = data['start']['position_mm']
        self.assertAlmostEqual(x, 789.9, delta=0.1)
        self.assertEqual(y, 2362.5)
        self.assertEqual(data['start']['yaw_deg'] % 360, 0.0)

    def test_loop_is_both_loops_straight_through_crossing(self):
        self.assertAlmostEqual(self.path.length_mm, 3442.0 + 5197.0, delta=2)
        self.assertEqual(self.path.tiles.count('r3c3'), 2)
        self.assertEqual(len(self.path.tiles), 30)
        self.assertEqual(len(set(self.path.tiles)), 29)
        self.assertEqual(sorted(self.path.loose_ids)[:2],
                         ['r0c4_loose', 'r1c4_loose'])
        self.assertEqual(len(self.path.loose_ids), 11)

    def test_nearest_signed_left_positive(self):
        sx, sy = self.course.data['start']['position_mm']
        # Driving east on r0: north = left.
        near = self.path.nearest(sx + 10, sy + 5)
        self.assertAlmostEqual(near.error_mm, 5, places=6)
        self.assertEqual(near.tile_id, 'r0c2')
        near = self.path.nearest(sx + 10, sy - 7)
        self.assertAlmostEqual(near.error_mm, -7, places=6)

    def test_arc_distance(self):
        # r0c3 is SW: arc centre at its SW corner (945, 2205), r=157.5.
        cx, cy = 3 * TILE_MM, 7 * TILE_MM
        a = math.radians(45)
        near = self.path.nearest(cx + 160 * math.cos(a), cy + 160 * math.sin(a))
        self.assertEqual(near.tile_id, 'r0c3')
        # Driving east into r0c3, it turns right (south): the centre is on the
        # right, so a point outside the radius is on the left.
        self.assertAlmostEqual(near.error_mm, 2.5, places=6)

    def test_follower_laps_and_crossing_no_jump(self):
        f = PathFollower(self.path, *self.course.data['start']['position_mm'])
        step = 5.0
        n = int(2 * self.path.length_mm / step) + 3
        s0 = f.start_s
        for i in range(1, n):
            x, y = self.path.point((s0 + i * step) % self.path.length_mm)
            r = f.update(x, y)
            self.assertLess(abs(r.error_mm), 1e-6)
            self.assertAlmostEqual(f.progress_mm, i * step, delta=1e-6)
        self.assertEqual(f.laps, 2)

    def test_loose_tile_lookup(self):
        # r4c0 is a loose NW arc tile; its centre is off the track.
        x, y = 0.5 * TILE_MM, (7 - 4 + 0.5) * TILE_MM
        self.assertEqual(self.path.tile_at(x, y), 'r4c0_loose')
        self.assertTrue(self.path.is_loose(self.path.tile_at(x, y)))
        self.assertFalse(self.path.is_loose('r3c3'))

    def test_generic_open_line_rejected(self):
        with self.assertRaises(ValueError):
            CoursePath.from_course(straight_course())


if __name__ == '__main__':
    unittest.main()
