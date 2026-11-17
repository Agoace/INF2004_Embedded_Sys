#!/usr/bin/env python3
"""Local view fixes for a generated figure-8 world (.wbt), in place.

The world comes from SW8's pil/world.py (mtk3-hil); this tool does not
change that generator. It edits only the generated file:
  * Viewpoint: top-down, north up (the generator writes 'orientation
    0 0 1 0', a horizontal camera: black screen). Known issue for SW8.
  * Ramp colour: magenta like the real ramp (the course material is
    MAT_WHITE, so it is invisible on the white tiles). Visual only; the
    contact material and geometry do not change.
The obstacle keeps the generator's BOX colour (brown), which shows on the
white tiles.

PREVIEW ONLY: the patched world no longer matches its hash in the
generated manifest.json. Do not use it for audited SIL output. The tool
checks all match counts first and writes nothing on a failure; it writes
through a temporary file and os.replace.
"""
import argparse
import json
import math
import os
import pathlib
import re
import sys
import tempfile

RAMP_RGB = "0.85 0.1 0.55"
PREVIEW = ("# PREVIEW (course8_world_patch.py): view and ramp colour patched; "
           "manifest.json hash invalid; not for audited SIL output.")
FOV_RAD = math.pi / 4      # Webots Viewpoint default fieldOfView (horizontal)
ASPECT = 16 / 9
MARGIN = 1.1
TILE_M = 0.315
VIEW = re.compile(r"Viewpoint \{ position (\S+) (\S+) \S+ orientation [^}]*\}")
RAMP_SOLID = re.compile(r'Solid \{ name "ramp_\d+" ')
RAMP = re.compile(r'(Solid \{ name "ramp_\d+" .*?baseColor )([0-9.]+ [0-9.]+ '
                  r'[0-9.]+)')


def extents(course):
    """(centre x, centre y, width, height) in metres of the tile grid."""
    ox, oy = course["origin_mm"]
    cols = [t["column"] for t in course["tiles"]]
    rows = [t["row"] for t in course["tiles"]]
    x0, x1 = ox / 1000 + min(cols) * TILE_M, ox / 1000 + (max(cols) + 1) * TILE_M
    y0, y1 = oy / 1000 + min(rows) * TILE_M, oy / 1000 + (max(rows) + 1) * TILE_M
    return (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0


def view_height(width, height, fov=FOV_RAD, aspect=ASPECT, margin=MARGIN):
    """Camera height (m) so a width x height area (+ margin) fits a
    top-down view: horizontal FOV fov, vertical from the aspect ratio."""
    vfov = 2 * math.atan(math.tan(fov / 2) / aspect)
    return max(width * margin / 2 / math.tan(fov / 2),
               height * margin / 2 / math.tan(vfov / 2))


def patch(text, area=None):
    """Return (new text, viewpoints changed, ramps recoloured, ramp solids).

    area: (cx, cy, w, h) in metres; None keeps the generator centre and
    uses the 3.6 m fallback height."""
    if area is None:
        view = r"Viewpoint { position \1 \2 3.6 "
    else:
        cx, cy, w, h = area
        view = "Viewpoint { position %.4f %.4f %.3f " % (cx, cy, view_height(w, h))
    text, views = VIEW.subn(view + "orientation -0.57735 0.57735 0.57735 2.0944 }",
                            text)
    solids = len(RAMP_SOLID.findall(text))
    lines, ramps = [], 0
    for line in text.split("\n"):
        line, n = RAMP.subn(r"\g<1>" + RAMP_RGB, line, count=1)
        ramps += n
        lines.append(line)
    if PREVIEW not in lines:
        lines.insert(1, PREVIEW)
    return "\n".join(lines), views, ramps, solids


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("world", type=pathlib.Path)
    ap.add_argument("--course", type=pathlib.Path,
                    help="course JSON (default: ../course.json of the world)")
    a = ap.parse_args(argv)
    course = a.course or a.world.resolve().parent.parent / "course.json"
    area = (extents(json.loads(course.read_text(encoding="utf-8")))
            if course.is_file() else None)
    text, views, ramps, solids = patch(a.world.read_text(encoding="utf-8"), area)
    print("viewpoint %d, ramps %d of %d" % (views, ramps, solids))
    if views != 1 or ramps != solids:
        print("not patched: expected 1 viewpoint and every ramp solid",
              file=sys.stderr)
        return 1
    fd, tmp = tempfile.mkstemp(dir=a.world.parent, prefix=".patch-")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as out:
            out.write(text)
        os.replace(tmp, a.world)
    except BaseException:
        os.unlink(tmp)
        raise
    print(PREVIEW)
    return 0


if __name__ == "__main__":
    sys.exit(main())
