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
"""
import argparse
import pathlib
import re
import sys

RAMP_RGB = "0.85 0.1 0.55"
VIEW = re.compile(r"Viewpoint \{ position (\S+) (\S+) \S+ orientation [^}]*\}")
RAMP = re.compile(r'(Solid \{ name "ramp_\d+" .*?baseColor )([0-9.]+ [0-9.]+ '
                  r'[0-9.]+)')


def patch(text):
    """Return (new text, viewpoints changed, ramps recoloured)."""
    text, views = VIEW.subn(
        r"Viewpoint { position \1 \2 3.6 orientation -0.57735 0.57735 "
        r"0.57735 2.0944 }", text)
    lines, ramps = [], 0
    for line in text.split("\n"):
        line, n = RAMP.subn(r"\g<1>" + RAMP_RGB, line, count=1)
        ramps += n
        lines.append(line)
    return "\n".join(lines), views, ramps


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("world", type=pathlib.Path)
    a = ap.parse_args(argv)
    text, views, ramps = patch(a.world.read_text(encoding="utf-8"))
    a.world.write_text(text, encoding="utf-8")
    print("viewpoint %d, ramps %d" % (views, ramps))
    return 0 if views == 1 else 1


if __name__ == "__main__":
    sys.exit(main())
