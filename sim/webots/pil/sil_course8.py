"""Figure-8 demo course SIL: pinned read-only sources, no merge.

Course files come from mtk3-course8 COURSE_REVISION (git archive, read
only). Firmware C for the course8 controller comes from staging
FIRMWARE_REVISION (git archive, read only): the school demo image source,
base 120 / D 30 / same-tick by default. Old values are cases that use the
robot's own override headers. Nothing here edits either source.
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
FIRMWARE_REVISION = '70175801256d3d5e23257c872884cf6a48bb5952'
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


# --- Native library: firmware C from FIRMWARE_REVISION, SIL adapters here --
# case: (base mm/s, D kd_pct, same-tick). FIRMWARE_REVISION config is the
# demo default 120/30 (no override header). The robot forces other values
# into vehicle_task.o with line_kd.h / line_p_near.h; the SIL forces them
# into the adapter TU (the LINE_CFG owner) the same way.
CASES = {'demo': (120, 30, True),      # demo default since 2026-11-17
         'p7': (146, 85, False),       # old default (P7, D 85)
         'kd30st': (146, 30, True)}    # SIL runs 3-4 (2026-11-17)
DEFAULT_CASE = 'demo'
CONFIG_BASE, CONFIG_KD = 120, 30
COURSE8_SOURCES = ('sim/webots/controllers/sil_line/course8_adapter.c',
                   'app_program/subsystem3/line_position.c',
                   'app_program/subsystem3/line_steer.c',
                   'app_program/subsystem2/motor_ctl.c',
                   'app_program/vehicle/vehicle_sm.c',
                   'app_program/subsystem3/line_trace.c',
                   'app_program/subsystem3/line_search.c',
                   'app_program/subsystem5/obstacle.c',
                   'app_program/vehicle/junction_action.c',
                   'app_program/vehicle/junction_line.c')
ADAPTERS = ('sim/webots/controllers/sil_line/course8_adapter.c',
            'sim/webots/controllers/sil_line/motor_adapter.c')


def build_course8_library(directory, *, case=DEFAULT_CASE):
    """Compile into directory/libsil_line.so; returns the build record."""
    import hashlib
    import json
    import shutil
    from .sil_search import SearchController
    if case not in CASES:
        raise ValueError('approved cases: ' + ', '.join(CASES))
    base, kd, same_tick = CASES[case]
    directory = Path(directory)
    snapshot = directory / 'fw-source'
    snapshot.mkdir(parents=True)
    archive(FIRMWARE_REVISION, ['app_program'], snapshot)
    for name in ADAPTERS:
        target = snapshot / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    flags = []
    if kd != CONFIG_KD:
        flags += [f'-DLINE_KD={kd}', '-include',
                  str(snapshot / 'app_program/vehicle/line_kd.h')]
    if base != CONFIG_BASE:
        flags += [f'-DLINE_BASE={base}', '-include',
                  str(snapshot / 'app_program/vehicle/line_p_near.h')]
    if same_tick:
        flags.append('-DSIL_SAME_TICK_APPLY=1')
    library = directory / 'libsil_line.so'
    command = ['cc', '-std=c11', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra',
               '-Wconversion', '-Wshadow', '-Werror', '-I', str(snapshot)]
    objects, commands = [], []
    for i, source in enumerate(COURSE8_SOURCES):
        obj = directory / f'course8-{i}.o'
        # Only the adapter TU owns LINE_CFG (as vehicle_task.o on the robot).
        cmd = [*command[:6], *command[6:], *(flags if i == 0 else []), '-c',
               str(snapshot / source), '-o', str(obj)]
        cmd.remove('-shared')
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        objects.append(obj)
        commands.append(cmd)
    link = ['cc', '-shared', *map(str, objects), '-o', str(library)]
    subprocess.run(link, check=True, capture_output=True, text=True)
    commands.append(link)
    meta = SearchController(library).metadata()
    if meta['steer'] != [base, 85, kd, 62, 378, 50, 340]:
        raise ValueError('course8 native config mismatch: ' + str(meta['steer']))
    record = dict(firmware_revision=FIRMWARE_REVISION, course_revision=COURSE_REVISION,
                  case=case, base=base, kd=kd, same_tick_apply=same_tick,
                  commands=commands,
                  source_policy='read-only git archive overlay, NOT merged',
                  sources_sha256={str(p.relative_to(snapshot)): hashlib.sha256(
                      p.read_bytes()).hexdigest() for p in sorted(snapshot.rglob('*.[ch]'))
                      if 'app_program/sim/' not in str(p)},
                  configuration=meta,
                  library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    (directory / 'sil-build.json').write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
    return record


def calibration_rows(profiles):
    """SYNTHETIC floor/line rows at the profile levels (host checks only)."""
    w = [round(profiles[n].levels_adc['MAT_WHITE']) for n in ('GP27', 'GP26')]
    b = [round(profiles[n].levels_adc['MAT_LINE']) for n in ('GP27', 'GP26')]
    return [[w[0], w[1], 0]] * 16 + [[b[0], b[1], 0]] * 4


class Course8Controller:
    """SearchController plus the junction wrapper state."""

    def __init__(self, library):
        import ctypes
        from .sil_search import SearchController
        self.native = SearchController(library)
        self.native.lib.sil_course8_junction.argtypes = [ctypes.POINTER(ctypes.c_int32)]
        self.native.lib.sil_course8_junction.restype = None
        self._ctypes = ctypes

    def junction(self):
        out = (self._ctypes.c_int32 * 7)()
        self.native.lib.sil_course8_junction(out)
        return dict(zip(('ja_phase', 'ja_reason', 'ja_command', 'ja_marks',
                         'ja_consumed', 'ja_active', 'ja_searching'), map(int, out)))


def host_drive(course, library, *, ms, profiles, mounts, rows=None):
    """HOST KINEMATIC CHECK, not Webots: unicycle on a flat floor, LagPlant
    with DM1 rows (interpolate only), numpy sampler. Returns rows of
    (ms, x, y, yaw, module, searches, ja_phase, error_mm, tile)."""
    import math
    from .course_builder import build_course
    from .sil_course_path import CoursePath, PathFollower
    from .sil_motor import LagPlant
    from .sil_ray_np import NumpySceneSampler
    built = build_course(course)
    sampler = NumpySceneSampler(built)
    try:
        path = CoursePath.from_course(course)
    except ValueError:
        path = None          # open test course: no loop reference
    ctl = Course8Controller(library)
    ctl.native.calibrate(rows or calibration_rows(profiles))
    plant = LagPlant(bidirectional=True)
    x, y = course.data['start']['position_mm']
    yaw = math.radians(course.data['start']['yaw_deg'])
    angles = [0.0, 0.0]
    track = 111.5
    follower = None
    out = []
    for t in range(1, ms + 1):
        c, s = math.cos(yaw), math.sin(yaw)
        adc = []
        for name in ('GP27', 'GP26'):
            f, l = mounts[name]['forward_mm'], mounts[name]['left_mm']
            o = (x + c * f - s * l, y + s * f + c * l, 10 + mounts[name]['flat_clearance_mm'])
            sample = sampler.sample(profiles[name], o, axis_u=(c, s, 0), sample_us=t * 1000)
            if not sample.valid:
                raise ValueError('optical sample invalid: ' + sample.reason)
            adc.append(sample.adc)
        counts = [round(a * 2535 / (2 * math.pi)) for a in angles]
        r = ctl.native.update(adc[0], adc[1], t, *counts)
        vl, vr = plant.advance(r['left_pct'], r['right_pct'])
        angles[0] += vl * .001
        angles[1] += vr * .001
        v = (vl + vr) / 2 * 32.5
        w = (vr - vl) * 32.5 / track
        x += v * math.cos(yaw) * .001
        y += v * math.sin(yaw) * .001
        yaw += w * .001
        px, py = x + c * 119 - s * 7.25, y + s * 119 + c * 7.25
        near = None
        if path is not None:
            if follower is None:
                follower = PathFollower(path, px, py)
            near = follower.update(px, py)
        j = ctl.junction()
        out.append((t, x, y, yaw, r['module'], r['searches'], j['ja_phase'],
                    near.error_mm if near else None, near.tile_id if near else None,
                    follower.progress_mm if near else None))
    return out
