"""Figure-8 demo course SIL: pinned read-only sources, no merge.

Course files come from mtk3-course8 COURSE_REVISION (git archive, read
only). Firmware C for the course8 controller comes from mtk3-line-kd
FIRMWARE_REVISION (git archive, read only). Nothing here edits either.
"""
import importlib.util
import io
from pathlib import Path
import subprocess
import tarfile
import tempfile

from .course_builder import Course
from .sil_line import ROOT

COURSE_REVISION = '3b43a1952bbb65f86cc942ee13f41aaf6280e7bb'
FIRMWARE_REVISION = 'd06a78753e17a73a27867fe703a0cbd6ced391a9'
COURSE_FILES = ('sim/webots/tools/course8_tilemap.py',
                'sim/webots/courses/course_figure8_tilemap.txt')
# Scenario (operator 2026-11-17): track only, ramp at the photo position.
# The photos put the ramp north of the r3 line, on the right when the robot
# drives west there (L2r). The start spot is the photo start (L1:1190), but
# driven the other way (L1r), so the robot then drives L2r and the ramp is
# under its right tyre.
RAMP = ('L2r', 4790.0, 'R', 34.0)
START_SPOT = ('L1', 1190.0)


def archive(revision, paths, destination):
    """Extract pinned files; returns destination."""
    data = subprocess.check_output(['git', 'archive', revision, *paths],
                                   cwd=ROOT)
    with tarfile.open(fileobj=io.BytesIO(data)) as tar:
        tar.extractall(destination, filter='data')
    return Path(destination)


def course8_module(directory):
    path = Path(directory) / COURSE_FILES[0]
    spec = importlib.util.spec_from_file_location('course8_tilemap', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def course8_data(ramps=(RAMP,)):
    with tempfile.TemporaryDirectory() as tmp:
        archive(COURSE_REVISION, COURSE_FILES, tmp)
        m = course8_module(tmp)
        cells = m.parse((Path(tmp) / COURSE_FILES[1]).read_text())
        m.check(cells)
        reverse_mm = m.loop_length(cells, 'L1') - START_SPOT[1]
        data = m.course(cells, ramps=ramps, start=('L1r', reverse_mm))
    data['name'] = 'course_figure8_sil'
    data['provenance']['source'] += (
        ' SIL scenario: track only; ramp L2r:4790 R gap 34 mm; start = photo '
        'spot L1:1190 driven as L1r (east). Sources: mtk3-course8 '
        + COURSE_REVISION + ' (read-only overlay).')
    return data


def course8_course(ramps=(RAMP,)):
    return Course.from_dict(course8_data(ramps))
