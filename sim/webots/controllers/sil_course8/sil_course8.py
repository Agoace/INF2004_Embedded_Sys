"""Figure-8 SIL: simulated optical ADC -> vehicle LINE/SEARCH C -> wheels.

New controller (course8 only); sil_line.py is unchanged. Differences:
  * Line reference: the course loop (pil.sil_course_path), not a fixed
    straight/curve. A follower window keeps the two crossing passes apart.
  * Tilt: flat projection while the body is level (upright cos >= .999,
    as sil_line.py). When tilted (one wheel on the ramp) and the course has
    a hump: the sensor origin, axis and beam follow the full 3D body pose,
    and the full-scene ray model runs with explicit extrapolation
    ('uncalibrated_height_or_tilt'). Geometry (height, footprint, hit
    point) is modelled; the ADC response to height/tilt is NOT calibrated.
    Beyond TILT_LIMIT_DEG the run stops INCOMPLETE.
  * Sampler: pil.sil_ray_np (bit-identical to sample_sensor, host test).
No watchdog is fed or modelled. Plant: LagPlant with DM1 rows (interpolate
only; out-of-table duty raises).
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / 'libraries'))
from pil.course_builder import Course, build_course  # noqa: E402
from pil.s3_optics import SensorProfile  # noqa: E402
from pil.sil_course_path import CoursePath, PathFollower  # noqa: E402
from pil.sil_ray_np import NumpySceneSampler  # noqa: E402

TILT_LIMIT_DEG = 8.0
FLAT_COS = .999
TRACK_MM = 111.5
RADIUS_MM = 32.5
FIELDS = ('sim_ms', 'left_adc', 'centre_adc', 'position', 'event', 'line_seen',
          'junction', 'left_pct', 'right_pct', 'norm_left', 'norm_centre',
          'fault', 'desired_left_mm_s', 'desired_right_mm_s', 'motor_faults',
          'module', 'shell_state', 'shell_fault', 'shell_fault_detail',
          'handoffs', 'searches', 'found_by_search', 'last_outcome', 'action',
          'ja_phase', 'ja_reason', 'ja_active',
          'left_count', 'right_count', 'target_left_rad_s', 'target_right_rad_s',
          'x_m', 'y_m', 'z_m', 'yaw_rad', 'roll_deg', 'pitch_deg',
          'left_wheel_lift_mm', 'right_wheel_lift_mm', 'tilted',
          'error_mm', 'track_dist_mm', 'tile', 'under_tile', 'progress_mm', 'segment')
NATIVE_KEYS = ('position', 'event', 'line_seen', 'junction', 'left_pct',
               'right_pct', 'norm_left', 'norm_centre', 'fault',
               'desired_left_mm_s', 'desired_right_mm_s', 'motor_faults',
               'module', 'shell_state', 'shell_fault', 'shell_fault_detail',
               'handoffs', 'searches', 'found_by_search', 'last_outcome', 'action')


def controller_for(project):
    from pil.sil_course8 import Course8Controller
    from pil.sil_motor import LagPlant
    return (Course8Controller(project / 'libraries/libsil_line.so'),
            LagPlant(bidirectional=True))


def replay(project, rows):
    """Feed the recorded ADC + counts to a fresh native library; every
    native output and plant target must match the recorded row."""
    config = json.loads((project / 'sil-manifest.json').read_text())
    ctl, plant = controller_for(project)
    ctl.native.calibrate(config['calibration_rows'])
    i = {n: k for k, n in enumerate(FIELDS)}
    for row in rows:
        actual = ctl.native.update(int(row[1]), int(row[2]), int(row[0]),
                                   int(row[i['left_count']]), int(row[i['right_count']]))
        got = [actual[k] for k in NATIVE_KEYS]
        want = [row[i[k]] for k in NATIVE_KEYS]
        if got != want:
            raise ValueError(f'native replay mismatch at {row[0]} ms')
        targets = plant.advance(actual['left_pct'], actual['right_pct'])
        if list(targets) != [row[i['target_left_rad_s']], row[i['target_right_rad_s']]]:
            raise ValueError(f'plant replay mismatch at {row[0]} ms')
    return True


def body_axes(r):
    """Webots getOrientation (row-major) -> body x, y, z axes in world."""
    return (r[0], r[3], r[6]), (r[1], r[4], r[7]), (r[2], r[5], r[8])


def run(robot, project):
    project = Path(project)
    config = json.loads((project / 'sil-manifest.json').read_text())
    if robot.getBasicTimeStep() != 1:
        raise ValueError('SIL needs 1 ms step')
    course = Course.load(project / 'course.json')
    built = build_course(course)
    sampler = NumpySceneSampler(built)
    path = CoursePath.from_course(course)
    has_hump = bool(course.data['humps'])
    profiles = {n: SensorProfile(**d) for n, d in config['profiles'].items()}
    mounts = config['mounts']
    ctl, plant = controller_for(project)
    calibration = ctl.native.calibrate(config['calibration_rows'])
    if config.get('spin_endpoints') and calibration['endpoints'] != config['spin_endpoints']:
        raise ValueError('spin endpoints mismatch')
    wheels = [robot.getDevice(n) for n in ('left_motor', 'right_motor')]
    for wheel in wheels:
        wheel.setPosition(float('inf'))
        wheel.setAvailableTorque(.12)
        wheel.setVelocity(0.)
    robot.getDevice('sonar_servo').setPosition(0.)
    encoders = [robot.getDevice(n) for n in ('left_encoder', 'right_encoder')]
    for encoder in encoders:
        encoder.enable(1)
    ticks, laps_goal = config['duration_ms'], config['laps']
    tilt_cos = math.cos(math.radians(TILT_LIMIT_DEG))
    rows, error, follower, z_flat = [], None, None, None
    pair_f = mounts['GP26']['forward_mm']
    pair_l = (mounts['GP26']['left_mm'] + mounts['GP27']['left_mm']) / 2
    wall0 = time.monotonic()
    try:
        for slot in range(ticks):
            if robot.step(1) == -1 or round(robot.getTime() * 1000) != slot + 1:
                raise ValueError('SIL step sequence')
            node = robot.getSelf()
            pos = node.getPosition()
            rot = node.getOrientation()
            bx, by, bz = body_axes(rot)
            yaw = math.atan2(rot[3], rot[0])
            c, s = math.cos(yaw), math.sin(yaw)
            upright = rot[8]
            tilted = upright < FLAT_COS
            if tilted and (not has_hump or upright < tilt_cos):
                raise ValueError('outside tilt domain: upright cos %.5f' % upright)
            if not tilted:
                z_flat = pos[2] * 1000
            adc = []
            for name in ('GP27', 'GP26'):
                m = mounts[name]
                f, l = m['forward_mm'], m['left_mm']
                if not tilted:
                    origin = (pos[0] * 1000 + c * f - s * l,
                              pos[1] * 1000 + s * f + c * l, 10 + m['flat_clearance_mm'])
                    sample = sampler.sample(profiles[name], origin, axis_u=(c, s, 0),
                                            sample_us=(slot + 1) * 1000)
                else:
                    if z_flat is None:
                        raise ValueError('tilted before any flat sample')
                    hb = 10 + m['flat_clearance_mm'] - z_flat
                    origin = tuple(pos[k] * 1000 + bx[k] * f + by[k] * l + bz[k] * hb
                                   for k in range(3))
                    sample = sampler.sample(profiles[name], origin,
                                            direction=tuple(-v for v in bz), axis_u=bx,
                                            sample_us=(slot + 1) * 1000,
                                            allow_extrapolation=True)
                if not sample.valid:
                    raise ValueError('optical sample invalid: ' + sample.reason)
                adc.append(sample.adc)
            angles = [e.getValue() for e in encoders]
            counts = [round(a * 2535 / (2 * math.pi)) for a in angles]
            actual = ctl.native.update(adc[0], adc[1], slot + 1, *counts)
            j = ctl.junction()
            targets = plant.advance(actual['left_pct'], actual['right_pct'])
            for wheel, target in zip(wheels, targets):
                wheel.setVelocity(target)
            px = pos[0] * 1000 + c * pair_f - s * pair_l
            py = pos[1] * 1000 + s * pair_f + c * pair_l
            if follower is None:
                follower = PathFollower(path, px, py)
            near = follower.update(px, py)
            track_dist = abs(path.nearest(px, py).error_mm)
            roll = math.degrees(math.atan2(by[2], bz[2]))
            pitch = math.degrees(math.asin(max(-1., min(1., -bx[2]))))
            lifts = [(pos[2] * 1000 + by[2] * sgn * TRACK_MM / 2) - (10 + RADIUS_MM)
                     for sgn in (1, -1)]
            row = (slot + 1, *adc, *[actual[k] for k in NATIVE_KEYS],
                   j['ja_phase'], j['ja_reason'], j['ja_active'], *counts, *targets,
                   *pos, yaw, roll, pitch, *lifts, int(tilted), near.error_mm,
                   track_dist, near.tile_id, path.tile_at(px, py) or '',
                   follower.progress_mm, near.segment)
            rows.append(row)
            if abs(near.error_mm) > 250:
                raise ValueError('course gross motion bound (250 mm)')
            if actual['shell_state'] == 2:
                raise ValueError('vehicle shell FAULT')
            if follower.progress_mm >= laps_goal * path.length_mm:
                break
    except Exception as exception:   # recorded; never PASS
        error = str(exception)
    finally:
        for wheel in wheels:
            wheel.setVelocity(0.)
    wall = time.monotonic() - wall0
    out = project / 'sil-result'
    out.mkdir()
    with (out / 'samples.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        writer.writerows(rows)
    laps_done = bool(rows) and rows[-1][FIELDS.index('progress_mm')] >= laps_goal * path.length_mm
    proof = False
    if error is None and laps_done:
        try:
            proof = replay(project, rows)
        except ValueError as exception:
            error = str(exception)
    complete = error is None and laps_done and proof
    summary = dict(status='SIL_COURSE8_COMPLETE' if complete else 'SIL_COURSE8_INCOMPLETE',
                   error=error, samples=len(rows), laps_goal=laps_goal,
                   laps_done=laps_done, recorded_input_native_replay=proof,
                   sim_seconds=len(rows) / 1000, controller_wall_seconds=wall,
                   loop_length_mm=path.length_mm, calibration=calibration,
                   configuration=ctl.native.metadata(), simulation_only=True,
                   hardware_used=False, serial_used=False,
                   samples_sha256=hashlib.sha256((out / 'samples.csv').read_bytes()).hexdigest(),
                   limits=('ASSUMED optics (BOOT5209 fit), flat projection when level; '
                           'tilted samples use 3D ray geometry with UNCALIBRATED '
                           'height/tilt response; LagPlant DM1 rows; ramp profile '
                           'ASSUMED (generator sine hump); not hardware'))
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    robot.simulationQuit(0 if complete else 1)


if __name__ == '__main__':
    from controller import Supervisor
    run(Supervisor(), PROJECT)
