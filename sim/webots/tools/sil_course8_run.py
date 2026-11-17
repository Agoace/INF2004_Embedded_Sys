#!/usr/bin/env python3
"""Figure-8 SIL drive: one approved case, one launch, no retry.

Cases: default (base 146, P 85, D 85, no same-tick) and kd30st (D 30 +
same-tick). Track only, ramp at the photo position, 2 laps. Uses the
shared approved launcher (existing DISPLAY=:0 / XAUTHORITY, own process
group, --batch --mode=fast --no-rendering). --prepare-only builds and
checks the project without Webots.
"""
import argparse
import csv
from dataclasses import asdict, fields
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from passive_smoke import SOURCE, launch_project  # noqa: E402
from pil.robot_profile import RobotProfile  # noqa: E402
from pil.world import write_project  # noqa: E402
from pil.s3_assumed_line import load_boot5209_line_profile, load_assumed_line_mount  # noqa: E402
from pil.sil_course8 import (course8_course, build_course8_library, COURSE_REVISION,  # noqa: E402
                             FIRMWARE_REVISION, RAMP)
from pil.sil_course_path import CoursePath  # noqa: E402
from pil.sil_course8_report import report  # noqa: E402

CASES = {'default': (85, False), 'kd30st': (30, True)}
ADC_DELAYS = (0, 20)
DELAY_LABEL = 'ADC transport +20 ms (SW8 model)'
# Pinned completed full-scene spin (SW8, 2026-11-15), read only.
SPIN_SHA256 = '61c5a513d91ae9369c70b73becf2047da39d853e6ca4eb85f9272ffbdf3fb06a'
LAPS = 2
DURATION_MS = 180000
CAP_SECONDS = 1200
FLOAT_FIELDS = frozenset(('left_pct', 'right_pct', 'target_left_rad_s', 'target_right_rad_s',
                          'x_m', 'y_m', 'z_m', 'yaw_rad', 'roll_deg', 'pitch_deg',
                          'left_wheel_lift_mm', 'right_wheel_lift_mm', 'error_mm',
                          'track_dist_mm', 'progress_mm'))


def profile_values(profile):
    return {f.name: dict(getattr(profile, f.name)) if f.name == 'levels_adc'
            else getattr(profile, f.name) for f in fields(profile)}


def prepare(output, case, spin_path, adc_delay_ms=0):
    if adc_delay_ms not in ADC_DELAYS:
        raise ValueError('adc delay must be one of %s ms' % (ADC_DELAYS,))
    kd, same_tick = CASES[case]
    spin_bytes = spin_path.read_bytes()
    if hashlib.sha256(spin_bytes).hexdigest() != SPIN_SHA256:
        raise ValueError('spin JSON is not the pinned file')
    spin = json.loads(spin_bytes)
    if not spin.get('complete') or spin.get('optical_sampler') != 'full':
        raise ValueError('completed full-scene spin required')
    profiles = {n: profile_values(load_boot5209_line_profile(n)) for n in ('GP26', 'GP27')}
    if hashlib.sha256(json.dumps(profiles, sort_keys=True).encode()).hexdigest() != spin['profiles_sha256']:
        raise ValueError('spin/profile provenance mismatch')
    rows = spin['calibration_rows']
    if hashlib.sha256(json.dumps(rows, separators=(',', ':')).encode()).hexdigest() != spin['sampled_rows_sha256']:
        raise ValueError('spin sampled-row hash mismatch')
    course = course8_course()
    robot = RobotProfile.load(SOURCE / 'calibration/robot_assumed.json')
    project = write_project(course, robot, output)
    world = project / 'worlds' / f"{course.data['name']}.wbt"
    world.write_text(world.read_text().replace('RoboPico {\n', 'RoboPico {\n controller "sil_course8"\n'))
    dest = project / 'controllers/sil_course8'
    dest.mkdir()
    shutil.copyfile(SOURCE / 'controllers/sil_course8/sil_course8.py', dest / 'sil_course8.py')
    libs = project / 'libraries'
    libs.mkdir(exist_ok=True)
    (libs / 'pil').mkdir(exist_ok=True)
    for source in (SOURCE / 'pil').glob('*.py'):
        shutil.copyfile(source, libs / 'pil' / source.name)
    build = build_course8_library(libs, kd=kd, same_tick=same_tick)
    (project / 'historical-spin.json').write_bytes(spin_bytes)
    config = dict(case=case, kd=kd, same_tick_apply=same_tick, laps=LAPS,
                  duration_ms=DURATION_MS, profiles=profiles,
                  mounts={n: asdict(load_assumed_line_mount(n)) for n in ('GP26', 'GP27')},
                  calibration_rows=rows, spin_endpoints=spin['calibration']['endpoints'],
                  spin_sha256=SPIN_SHA256, optical_sampler='full (pil.sil_ray_np, bit-identical batch)',
                  optical_profile='BOOT_5209_SPIN_FIT_B', course_revision=COURSE_REVISION,
                  firmware_revision=FIRMWARE_REVISION, ramp=list(RAMP),
                  library_sha256=build['library_sha256'],
                  scope=('vehicle_sm + LINE(jl_module: junction_line/junction_action over '
                         'line_trace) + SEARCH(line_search) + motor_ctl; no barcode source '
                         '(SHADOW), no sonar; not SDK tasks or RTOS; no watchdog'),
                  motor_order='shell step then motor tick' if same_tick else 'motor tick then shell step')
    if adc_delay_ms:
        # Only a non-zero delay adds keys: the 0 ms manifest stays identical.
        config.update(adc_delay_ms=adc_delay_ms, adc_transport_label=DELAY_LABEL,
                      adc_transport_model='pil.sil_excited_line.DelayPair (SW8, imported '
                      'unchanged); diagnostic, not a measured causal transport')
    (project / 'sil-manifest.json').write_text(json.dumps(config, indent=2, sort_keys=True) + '\n')
    manifest_path = project / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['sil_scope'] = config['scope']
    for path in sorted(project.rglob('*')):
        if path.is_file() and path != manifest_path:
            manifest['artifacts_sha256'][str(path.relative_to(project))] = \
                hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    return project, world, course


def load_rows(project):
    from importlib import util
    spec = util.spec_from_file_location('c8', project / 'controllers/sil_course8/sil_course8.py')
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    names = module.FIELDS
    rows = []
    with (project / 'sil-result/samples.csv').open(newline='') as stream:
        reader = csv.reader(stream)
        if next(reader) != list(names):
            raise ValueError('samples.csv fields')
        for raw in reader:
            row = []
            for name, v in zip(names, raw):
                if name in ('tile', 'under_tile'):
                    row.append(v)
                elif name in FLOAT_FIELDS:
                    row.append(float(v))
                else:
                    row.append(int(v))
            rows.append(tuple(row))
    return module, names, rows


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--case', choices=CASES, required=True)
    p.add_argument('--spin', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--timeout-seconds', type=int, default=CAP_SECONDS)
    p.add_argument('--adc-delay-ms', type=int, choices=ADC_DELAYS, default=0)
    p.add_argument('--prepare-only', action='store_true')
    a = p.parse_args()
    if a.output.exists():
        p.error('fresh output directory required; no retries, no overwrite')
    if not 20 <= a.timeout_seconds <= CAP_SECONDS:
        p.error('timeout 20..1200 s')
    if not a.prepare_only and (os.environ.get('DISPLAY') != ':0' or not os.environ.get('XAUTHORITY')):
        p.error('existing approved DISPLAY=:0 / XAUTHORITY required; no display server is started')
    project, world, course = prepare(a.output, a.case, a.spin, a.adc_delay_ms)
    if a.prepare_only:
        print(json.dumps(dict(prepared=str(project), world=str(world))))
        return 0
    start = time.monotonic()
    status, pid, command, contents = launch_project(project, world, a.timeout_seconds,
                                                    label='course8-' + a.case + ('-adc%d' % a.adc_delay_ms if a.adc_delay_ms else ''))
    wall = time.monotonic() - start
    summary, error, proof, rep = None, None, False, None
    try:
        summary = json.loads((project / 'sil-result/summary.json').read_text())
        if hashlib.sha256((project / 'sil-result/samples.csv').read_bytes()).hexdigest() != summary['samples_sha256']:
            raise ValueError('sample hash mismatch')
        module, names, rows = load_rows(project)
        if summary['status'] == 'SIL_COURSE8_COMPLETE':
            proof = module.replay(project, rows)
        path = CoursePath.from_course(course)
        dicts = [dict(zip(names, r)) for r in rows]
        rep = report(dicts, loop_mm=path.length_mm, crossing_tile='r3c3',
                     loose_ids=path.loose_ids, wall_seconds=wall)
    except (OSError, ValueError, KeyError) as exception:
        error = str(exception)
    complete = bool(status == 0 and summary and summary['status'] == 'SIL_COURSE8_COMPLETE'
                    and proof and error is None and 'ERROR:' not in contents
                    and 'WARNING:' not in contents)
    result = dict(case=a.case, adc_delay_ms=a.adc_delay_ms,
                  adc_transport=DELAY_LABEL if a.adc_delay_ms else 'none (0 ms)', complete=complete, verdict='COMPLETE' if complete else 'INCOMPLETE',
                  exit_status=status, owned_pgid=pid, command=command, cap_seconds=a.timeout_seconds,
                  wall_seconds=wall, independent_native_replay=proof, error=error,
                  summary=summary, report=rep)
    (project / 'course8-result.json').write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(dict(case=a.case, adc_delay_ms=a.adc_delay_ms, verdict=result['verdict'], wall_seconds=wall,
                          error=error or (summary or {}).get('error')), sort_keys=True))
    print('Evidence:', project)
    return 0 if complete else 1


if __name__ == '__main__':
    raise SystemExit(main())
