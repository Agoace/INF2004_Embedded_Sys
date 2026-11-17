#!/usr/bin/env python3
"""Tile map (text) -> course JSON for the figure-8 demo course.

Input: sim/webots/courses/course_figure8_tilemap.txt (see its header).
Output: course JSON in the schema-1 format of pil/course_builder.py
(mtk3-hil). This tool does not import that module: it only writes the
JSON, and checks the track on its own:
  * every track port meets a matching port of the next tile,
  * the track is two closed loops that share one 4-way crossing,
  * loose arcs (code with '*') are not joined to the track.
Only the line track is fixed. Obstacle, ramp and barcode are free on the
day, so they are parameters (default: none placed), given as a loop, a
distance along the line from the crossing centre in the drive direction,
and a side. L1 = loop 1 (leave the crossing north), L2 = loop 2 (leave it
east); suffix r = the other drive direction.
"""
import argparse
import json
import math
import pathlib
import sys

TILE = 315.0
STEP = {"N": (0, -1), "S": (0, 1), "E": (1, 0), "W": (-1, 0)}
OPP = {"N": "S", "S": "N", "E": "W", "W": "E"}
# course_builder: CURVE at 0 deg joins N and E; rotation is CCW.
CURVE_ROT = {"NE": 0, "NW": 90, "SW": 180, "SE": 270}


def parse(text):
    """Return {(col, row): (code, loose)} with row 0 at the north edge."""
    cells, header = {}, None
    for line in text.splitlines():
        part = line.split()
        if part[:2] == ["#", "c0"] and header is None:
            header = part[1:]
            continue
        if not part or line.lstrip().startswith("#"):
            continue
        if not part[0].startswith("r") or header is None:
            raise ValueError("bad tile map line: " + line)
        row = int(part[0][1:])
        if len(part) - 1 != len(header):
            raise ValueError("row %d has %d cells" % (row, len(part) - 1))
        for col, code in enumerate(part[1:]):
            loose = code.endswith("*")
            code = code.rstrip("*")
            if code == ".":
                continue
            if code not in ("B", "|", "-", "X") and code not in CURVE_ROT:
                raise ValueError("unknown code %r at r%d c%d" % (code, row, col))
            if loose and code not in CURVE_ROT:
                raise ValueError("only arcs can be loose (r%d c%d)" % (row, col))
            cells[(col, row)] = (code, loose)
    return cells


def ports(code):
    return {"B": "", "|": "NS", "-": "EW", "X": "NESW"}.get(code, code)


def check(cells):
    """Return a dict of facts; raise ValueError on a broken track."""
    track = {c: v[0] for c, v in cells.items() if not v[1] and v[0] != "B"}
    loose = sorted(c for c, v in cells.items() if v[1])
    for (col, row), code in track.items():
        for p in ports(code):
            dx, dy = STEP[p]
            other = track.get((col + dx, row + dy))
            if other is None or OPP[p] not in ports(other):
                raise ValueError("open track port r%d c%d %s" % (row, col, p))
    for col, row in loose:
        for p in ports(cells[(col, row)][0]):
            dx, dy = STEP[p]
            other = track.get((col + dx, row + dy))
            if other is not None and OPP[p] in ports(other):
                raise ValueError("loose arc r%d c%d joins the track" % (row, col))
    crosses = [c for c, code in track.items() if code == "X"]
    if len(crosses) != 1:
        raise ValueError("expected one crossing, found %d" % len(crosses))
    # Walk each loop: at the crossing go straight on, so start on each of
    # the two arms pairs (N/S and E/W) gives one loop each when the course
    # is a figure-8 whose loops turn at the crossing.
    loops = []
    seen = set()
    for first in "NE":
        cell, heading, length = crosses[0], first, 0
        path = []
        while True:
            dx, dy = STEP[heading]
            cell = (cell[0] + dx, cell[1] + dy)
            entry = OPP[heading]
            code = track[cell]
            length += 1
            if code == "X":
                break
            path.append(cell)
            heading = ports(code).replace(entry, "", 1)
            if len(heading) != 1:
                raise ValueError("bad track at %s" % (cell,))
            if length > len(track) + 1:
                raise ValueError("track does not return to the crossing")
        loops.append((first, entry, length, path))
        seen.update(path)
    if seen | set(crosses) != set(track):
        raise ValueError("track tiles outside the two loops")
    for first, entry, _, _ in loops:
        if OPP[first] == entry:
            raise ValueError("loop from %s goes straight through" % first)
    return {"tiles": len(cells), "track": len(track), "loose_arcs": loose,
            "crossing": crosses[0],
            "loops_tiles": [lp[2] for lp in loops]}


def centre(col, row, rows):
    """course_builder: y grows north, so map row 0 (north) to the top."""
    return ((col + 0.5) * TILE, (rows - 1 - row + 0.5) * TILE)


# Unit vectors in the course frame (x east, y north).
VEC = {"N": (0.0, 1.0), "S": (0.0, -1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}
HALF = TILE / 2
ARC = math.pi * HALF / 2  # quarter arc, radius 157.5 mm


def loop_path(cells, name):
    """Segments of one loop, in drive order, from the crossing centre.

    name: L1 (leave the crossing north), L2 (leave it east); suffix r for
    the other drive direction. Each segment is (start xy, entry dir,
    exit dir, length): entry/exit are drive headings N/E/S/W.
    """
    rows = max(r for _, r in cells) + 1
    track = {c: v[0] for c, v in cells.items() if not v[1] and v[0] != "B"}
    cross = next(c for c, code in track.items() if code == "X")
    first = {"L1": "N", "L2": "E"}[name.rstrip("r")]
    cells_in_order, heading, cell = [], first, cross
    while True:
        dx, dy = STEP[heading]
        cell = (cell[0] + dx, cell[1] + dy)
        if track[cell] == "X":
            break
        out = ports(track[cell]).replace(OPP[heading], "", 1)
        cells_in_order.append((cell, heading, out))
        heading = out
    last = heading
    if name.endswith("r"):
        cells_in_order = [(c, OPP[o], OPP[h]) for c, h, o in
                          reversed(cells_in_order)]
        first, last = OPP[last], OPP[first]
    cx, cy = centre(cross[0], cross[1], rows)
    segs = [((cx, cy), first, first, HALF)]
    for (col, row), h_in, h_out in cells_in_order:
        x, y = centre(col, row, rows)
        ex, ey = VEC[h_in]
        start = (x - ex * HALF, y - ey * HALF)
        segs.append((start, h_in, h_out, TILE if h_in == h_out else ARC))
    ex, ey = VEC[last]
    segs.append(((cx - ex * HALF, cy - ey * HALF), last, last, HALF))
    return segs


def pose(cells, name, dist_mm):
    """(x, y, heading_deg) on the line, dist_mm along loop 'name'."""
    segs = loop_path(cells, name)
    total = sum(sg[3] for sg in segs)
    if not 0 <= dist_mm < total:
        raise ValueError("%s: distance %.0f outside 0..%.0f mm" %
                         (name, dist_mm, total))
    for (sx, sy), h_in, h_out, length in segs:
        if dist_mm >= length:
            dist_mm -= length
            continue
        ix, iy = VEC[h_in]
        if h_in == h_out:
            return (sx + ix * dist_mm, sy + iy * dist_mm,
                    math.degrees(math.atan2(iy, ix)))
        ox, oy = VEC[h_out]
        # Arc centre: the tile corner on the exit side.
        cx, cy = sx + ox * HALF, sy + oy * HALF
        a = dist_mm / HALF
        rx, ry = -ox * HALF, -oy * HALF  # radius vector at the start
        turn = 1 if ix * oy - iy * ox > 0 else -1  # +1 = left (CCW)
        c, s_ = math.cos(turn * a), math.sin(turn * a)
        px, py = cx + c * rx - s_ * ry, cy + s_ * rx + c * ry
        hx, hy = c * ix - s_ * iy, s_ * ix + c * iy
        return px, py, math.degrees(math.atan2(hy, hx))
    raise AssertionError("unreachable")


def loop_length(cells, name):
    return sum(sg[3] for sg in loop_path(cells, name))


def offset(x, y, heading, left_mm):
    a = math.radians(heading)
    return x - math.sin(a) * left_mm, y + math.cos(a) * left_mm


# Placement: LOOP:MM[:SIDE[:GAP_MM]]. SIDE is L, R or C (centre); GAP_MM
# is the lateral gap from the line centre to the near edge of the object.
# Obstacle: default C, gap 0 (centred on the line).
# Ramp: default R, gap 34 mm: it lies BESIDE the line on the right of the
# drive direction (one-wheel hump). Operator estimate 2026-11-17: about
# 25 mm from the line edge (9 + 25 = 34 mm from the centre); ramp 50 x 195
# x 10 mm. (Photo estimate, not used: 69-80 mm from the centre.)
# Barcode: always 77 mm left of the line centre (SIL sheet, ASSUMED).
SIDES = {"L": 1.0, "R": -1.0, "C": 0.0}
DEFAULTS = {"obstacle": ("C", 0.0), "ramp": ("R", 34.0),
            "barcode": ("L", 77.0)}
RAMP_LENGTH_MM, RAMP_WIDTH_MM, RAMP_HEIGHT_MM = 195.0, 50.0, 10.0
BARCODE_LEFT_MM = 77.0


def placement(text, kind="obstacle"):
    """'L1:600', 'L2r:900:R' or 'L2:600:R:80' -> (loop, mm, side, gap)."""
    part = text.split(":")
    if not 2 <= len(part) <= 4 or part[0] not in ("L1", "L1r", "L2", "L2r"):
        raise ValueError("placement must be LOOP:MM[:SIDE[:GAP_MM]], LOOP "
                         "in L1 L1r L2 L2r: " + text)
    side, gap = DEFAULTS[kind]
    if len(part) >= 3:
        side = part[2]
    if side not in SIDES:
        raise ValueError("side must be L, R or C: " + text)
    if len(part) == 4:
        gap = float(part[3])
    elif side == "C":
        gap = 0.0
    if gap < 0 or (side == "C" and gap != 0):
        raise ValueError("gap must be >= 0 (and 0 with side C): " + text)
    if kind == "barcode" and (side, gap) != ("L", BARCODE_LEFT_MM):
        raise ValueError("barcode is always L, 77 mm: " + text)
    return part[0], float(part[1]), side, gap


def wheel_overlap_mm(gap_mm, width_mm=RAMP_WIDTH_MM, shift_mm=0.0):
    """Lateral overlap of the right wheel with a ramp on the right.

    Wheel centre 55.75 mm (half-track 111.5 / 2), width 16 mm, so the tyre
    spans 47.75..63.75 mm from the robot centre. shift_mm: robot centre
    right of the line centre (positive = toward the ramp)."""
    lo, hi = 47.75 + shift_mm, 63.75 + shift_mm
    return max(0.0, min(hi, gap_mm + width_mm) - max(lo, gap_mm))


def course(cells, obstacles=(), ramps=(), barcodes=(),
           start=("L1", 1190.0)):
    """Course JSON. Only the line track is fixed; objects are parameters
    (default: none placed)."""
    rows = max(r for _, r in cells) + 1
    tiles = []
    for (col, row), (code, loose) in sorted(cells.items(),
                                            key=lambda c: (c[0][1], c[0][0])):
        kind, rot = {"B": ("BLANK", 0), "|": ("STRAIGHT", 0),
                     "-": ("STRAIGHT", 90), "X": ("CROSS", 0)}.get(
                         code, ("CURVE", CURVE_ROT.get(code, 0)))
        tiles.append({"id": "r%dc%d%s" % (row, col, "_loose" if loose else ""),
                      "column": col, "row": rows - 1 - row, "kind": kind,
                      "rotation_deg": rot})
    boxes, humps, sheets = [], [], []
    for n, (loop, dist, side, gap) in enumerate(obstacles):
        x, y, h = pose(cells, loop, dist)
        # Gap to the near edge; the box is 120 mm across the line.
        lateral = gap + 60.0 if side != "C" else 0.0
        x, y = offset(x, y, h, SIDES[side] * lateral)
        # Footprint 170 x 120 mm (STEP 8 box, MEASURED): 120 across the
        # drive direction; height 120 mm PHOTO-ESTIMATED.
        boxes.append({"id": "obstacle_%d" % n, "centre_mm": [x, y, 70],
                      "size_mm": [170, 120, 120], "rotation_deg": h,
                      "material_id": "BOX"})
    for n, (loop, dist, side, gap) in enumerate(ramps):
        x, y, h = pose(cells, loop, dist)
        lateral = gap + (RAMP_WIDTH_MM / 2 if side != "C" else 0.0)
        x, y = offset(x, y, h, SIDES[side] * lateral)
        # 50 x 195 x 10 mm (operator estimate, 2026-11-17). Long axis
        # along the line.
        humps.append({"id": "ramp_%d" % n, "centre_mm": [x, y, 10],
                      "length_mm": RAMP_LENGTH_MM, "width_mm": RAMP_WIDTH_MM,
                      "height_mm": RAMP_HEIGHT_MM,
                      "segments": 16, "rotation_deg": h,
                      "material_id": "MAT_WHITE"})
    for n, (loop, dist, _side, _gap) in enumerate(barcodes):
        x, y, h = pose(cells, loop, dist)
        x, y = offset(x, y, h, BARCODE_LEFT_MM)
        # SIL sheet (ASSUMED): 400 x 50 mm, 40 mm bars, long axis along
        # the drive direction, always left of the line.
        sheets.append({"id": "barcode_%d" % n, "letter": "A",
                       "centre_mm": [x, y, 10], "rotation_deg": h,
                       "bar_height_mm": 40, "paper_size_mm": [400, 50],
                       "paper_thickness_mm": 0, "reverse": False})
    sx, sy, sh = pose(cells, *start)
    return {
        "schema": 1,
        "name": "course_figure8_photo",
        "provenance": {"assumed": True, "source": (
            "Demo course figure-8 from operator photos IMG_1960..1962 "
            "(2026-11-17), PHOTO-ESTIMATED tile grid; Cytron 315 mm tiles, "
            "18 mm line, arc radius 157.5 mm. Only the line track is fixed; "
            "obstacle, ramp, barcode and start are parameters (ASSUMED). "
            "See docs/simulation/COURSE_FIGURE8_20261117.md.")},
        "origin_mm": [0, 0],
        "start": {"position_mm": [sx, sy], "yaw_deg": sh},
        "allow_open_ports": True,
        "texture_mm_per_pixel": 0.5,
        "tiles": tiles,
        "boxes": boxes,
        "humps": humps,
        "barcodes": sheets,
    }


def main(argv=None):
    here = pathlib.Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--map", type=pathlib.Path,
                    default=here / "courses/course_figure8_tilemap.txt")
    ap.add_argument("--out", type=pathlib.Path)
    for kind in ("obstacle", "ramp", "barcode"):
        ap.add_argument("--" + kind, action="append", default=[],
                        type=lambda v, k=kind: placement(v, k),
                        metavar="LOOP:MM[:SIDE[:GAP_MM]]",
                        help="place one %s; repeat for more" % kind)
    ap.add_argument("--start", type=placement,
                    default=("L1", 1190.0, "C", 0.0),
                    metavar="LOOP:MM")
    a = ap.parse_args(argv)
    cells = parse(a.map.read_text(encoding="utf-8"))
    facts = check(cells)
    facts["loop_mm"] = {n: round(loop_length(cells, n), 1)
                        for n in ("L1", "L2")}
    text = json.dumps(course(cells, a.obstacle, a.ramp, a.barcode,
                             a.start[:2]), indent=1) + "\n"
    if a.out:
        a.out.write_text(text, encoding="utf-8")
    print(json.dumps(facts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
