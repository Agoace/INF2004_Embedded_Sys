"""Repeatable F1 accuracy/time gates. Failures are reported, never relaxed.

Generated reports/CSV belong under build/. No firmware, decoder or profile
writes. Synthetic line profiles exist only inside this evidence driver.
"""
import argparse
from array import array
from collections import Counter
from contextlib import contextmanager
import csv
import gc
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time
import tracemalloc

from . import s3_geometry as g
from .s3_fast import FastScene
from .s3_optics import SensorProfile, mean_adc
from .s3_profiles import DEFAULT_PATH, load_profile
from .s3_referee import fine_sample
from .scene import SceneSnapshot


def percentile(values, fraction):
    return sorted(values)[math.ceil(fraction*len(values))-1] if values else None


def accuracy_summary(errors, uncertainties):
    combined = [a+b for a, b in zip(errors, uncertainties)]
    return {'cases': len(combined), 'max_error_adc': max(errors) if errors else None,
            'p95_error_adc': percentile(errors, .95),
            'max_referee_delta_adc': max(uncertainties) if uncertainties else None,
            'max_error_including_referee_adc': max(combined) if combined else None,
            'p95_error_including_referee_adc': percentile(combined, .95),
            'check': 'PASS' if combined and max(combined) <= 8 and
            percentile(combined, .95) <= 2 and max(uncertainties) <= 1 and
            percentile(uncertainties, .95) <= .25 else 'FAIL'}


def time_summary(times):
    return {'samples': len(times), 'mean_us': statistics.mean(times),
            'p95_us': percentile(times, .95), 'p99_us': percentile(times, .99),
            'maximum_us': max(times), 'over_500us': sum(x > 500 for x in times),
            'check': 'PASS' if percentile(times, .95) <= 150 and
            percentile(times, .99) <= 250 and max(times) <= 500 else 'FAIL'}


def cases():
    """Fixed corpus, independent of scores. No dropping failing valid pairs."""
    for kind in ('STRAIGHT', 'CROSS', 'T'):
        patches = g.tile_geometry(kind).patches
        for x, y in ((-13.5, 0), (-9, 0), (0, 0), (9, 9), (12, 12), (0, -20)):
            for yaw in (0, 17, 45, 89):
                yield kind, patches, (x, y, 15.5), yaw
    for index, letter in enumerate('ABCDEFGHIJKLMNOPQRSTUVWXYZ'):
        for reverse in (False, True):
            barcode = g.barcode_geometry(letter, bar_height_mm=20,
                                          paper_size_mm=(285, 30), reverse=reverse)
            patches = g.tile_geometry('CROSS').patches+barcode.patches
            for x in (-133.85, 0):
                yield 'CODE39', patches, (x, 0, 15.5), (index*17+reverse*13) % 360
    patches = g.tile_geometry('CROSS').patches+g.barcode_geometry(
        'F', bar_height_mm=20, paper_size_mm=(285, 30)).patches
    for x, y in ((-133.85, -10), (-133.85, 10), (-133.85, 14.8),
                 (0, 14.8), (0, 16), (142.3, 0), (145, 0), (-142.3, 0)):
        for yaw in (0, 35, 89):
            yield 'FINITE_PAPER_BAR', patches, (x, y, 15.5), yaw
    patches = (g.tile_geometry('CROSS', tile_id='west').patches+
               g.tile_geometry('CROSS', centre_mm=(315, 0, 10), tile_id='east').patches)
    for x in (157.499, 157.5, 157.501):
        for y in (0, 9, 12):
            yield 'TILE_SEAM', patches, (x, y, 15.5), 31


def dense_strip_spot(scene, profile, origin, count=65536):
    """Independent dense first_hit midpoint rays, no polygon interval splitting.

    Only used where all material boundaries are constant in tangent direction
    throughout +/-4sigma (bar centre, not finite end or paper/line junction).
    """
    sigma = profile.sigma_x_mm
    step = 8*sigma/count
    norm = math.erf(4/math.sqrt(2))
    fractions = {}
    previous = math.erf(-4/math.sqrt(2))
    for i in range(count):
        u = -4*sigma+(i+.5)*step
        hit = scene.first_hit(origin, (u/profile.reference_height_mm, 0, -1), 100)
        if not hit.valid or hit.material_id not in g.MATERIALS:
            raise ValueError('invalid dense reference spot')
        boundary = math.erf((-4*sigma+(i+1)*step)/(math.sqrt(2)*sigma))
        weight = (boundary-previous)/(2*norm)
        previous = boundary
        fractions[hit.material_id] = fractions.get(hit.material_id, 0)+weight
    return mean_adc(profile, fractions)


def accuracy_report():
    profile = load_profile()
    rows, families, errors, deltas = [], {}, [], []
    # Reuse compilation only for identical original geometry, not stored poses.
    cache = {}
    for index, (family, patches, origin, yaw) in enumerate(cases()):
        key = repr(patches)
        if key not in cache:
            cache[key] = FastScene.from_patches(patches), SceneSnapshot.from_patches(patches)
        fast, scene = cache[key]
        angle = math.radians(yaw)
        axis = (math.cos(angle), math.sin(angle), 0)
        actual = fast.sample(profile, origin, axis_u=axis, sample_us=index*1000)
        lo = fine_sample(scene, profile, origin, axis_u=axis, tolerance=1e-6)
        hi = fine_sample(scene, profile, origin, axis_u=axis, tolerance=1e-8)
        valid = actual.valid and lo.valid and hi.valid
        error = abs(actual.mean_adc-hi.mean_adc) if valid else None
        delta = abs(lo.mean_adc-hi.mean_adc) if valid else None
        rows.append({'family': family, 'origin_mm': origin, 'yaw_deg': yaw,
                     'fast_valid': actual.valid, 'ref_valid': hi.valid,
                     'reasons': [actual.reason, lo.reason, hi.reason],
                     'error_adc': error, 'ref_delta_adc': delta, 'ref_rays': hi.rays})
        if valid:
            errors.append(error)
            deltas.append(delta)
            group = families.setdefault(family, ([], []))
            group[0].append(error)
            group[1].append(delta)
    summary = accuracy_summary(errors, deltas)
    summary['families'] = {k: accuracy_summary(*v) for k, v in families.items()}
    summary['invalid_cases'] = sum(not r['fast_valid'] or not r['ref_valid'] for r in rows)
    summary['fast_valid_ref_invalid'] = sum(r['fast_valid'] and not r['ref_valid'] for r in rows)
    # These prescribed F1 cases must ALL be supported; invalid cannot hide error.
    if summary['invalid_cases'] or any(v['check'] != 'PASS' for v in summary['families'].values()):
        summary['check'] = 'FAIL'
    patches = g.tile_geometry('BLANK').patches+g.barcode_geometry(
        'F', bar_height_mm=20, paper_size_mm=(285, 30)).patches
    scene = SceneSnapshot.from_patches(patches)
    spots = []
    for x in (-136.1, -133.85, -131.1):
        origin = (x, 0, 15.5)
        dense = dense_strip_spot(scene, profile, origin)
        fine = fine_sample(scene, profile, origin)
        spots.append({'x_mm': x, 'dense_rays': 65536,
                      'dense_vs_fine_error_adc': abs(dense-fine.mean_adc)})
    summary['dense_independent_spots'] = spots
    if max(s['dense_vs_fine_error_adc'] for s in spots) > 1:
        summary['check'] = 'FAIL'
    return {'summary': summary, 'cases': rows}


def course_coverage():
    from .course_builder import Course, build_course
    directory = Path(__file__).resolve().parents[1]/'courses'
    report = {}
    for path in sorted(directory.glob('*.json')):
        built = build_course(Course.load(path))
        fast = FastScene.from_course(built)
        reasons = Counter()
        for _, centre, _ in built.tile_geometries:
            for dx, dy in ((0, 0), (30, 30), (-60, 0), (0, 60)):
                sample = fast.sample(load_profile(), (centre[0]+dx, centre[1]+dy, 15.5))
                reasons['VALID' if sample.valid else sample.reason] += 1
        report[path.name] = {'cells': fast.cell_count, 'revision': fast.revision,
                             'sample_reasons': dict(reasons),
                             'NOT_a_full_course_acceptance': True}
    return report


class MeasurementBuffers:
    """Fixed numeric measurement storage, not retained per-frame GC tuples.

    Scalar temporaries and sampler result allocations still exist. This is
    not a claim of an allocation-free Python sensor implementation.
    """
    def __init__(self, count):
        if count < 1:
            raise ValueError('positive measurement count required')
        self.elapsed = array('d', [0])*count
        self.cpu = array('d', [0])*count
        self.valid = array('B', [0])*count
        self.start = array('Q', [0])*count
        self.end = array('Q', [0])*count

    def record(self, i, elapsed, cpu, valid, start, end):
        self.elapsed[i], self.cpu[i], self.valid[i] = elapsed, cpu, valid
        self.start[i], self.end[i] = start, end

    def rows(self):
        # Called only AFTER the timer loop when exporting CSV.
        return zip(range(len(self.elapsed)), self.elapsed, self.cpu, self.valid,
                   self.start, self.end)


@contextmanager
def timing_gc_policy(mode):
    """Experiment only; never a production sampling/transport policy."""
    if mode not in ('enabled', 'freeze-disable'):
        raise ValueError('unknown timing GC experiment')
    enabled, frozen = gc.isenabled(), gc.get_freeze_count()
    if mode == 'enabled' and not enabled:
        raise ValueError('enabled control requires initially enabled GC')
    if mode == 'freeze-disable' and frozen:
        raise ValueError('refuse to unfreeze a caller-owned frozen heap')
    info = {'mode': mode, 'initially_enabled': enabled,
            'measurement_storage': 'PREALLOCATED_NUMERIC_ARRAYS',
            'sampler_allocations_still_present': True}
    try:
        if mode == 'freeze-disable':
            gc.collect()
            gc.freeze()
            gc.disable()
        info['timed_gc_enabled'] = gc.isenabled()
        info['timed_frozen_objects'] = gc.get_freeze_count()
        yield info
    finally:
        if mode == 'freeze-disable':
            gc.unfreeze()
            if enabled:
                gc.enable()
            else:
                gc.disable()


def timing_report(output, count=100000, repeats=3, gc_mode='enabled'):
    if count < 100000 or repeats < 3:
        raise ValueError('certification requires>=100000 frames and>=3 repeats')
    patches = g.tile_geometry('CROSS').patches+g.barcode_geometry(
        'F', bar_height_mm=20, paper_size_mm=(285, 30)).patches
    tracemalloc.start()
    start = time.perf_counter_ns()
    fast = FastScene.from_patches(patches)
    setup_us = (time.perf_counter_ns()-start)/1000
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    gp28 = load_profile()
    profiles = (gp28,)+tuple(SensorProfile('ASSUMED_'+n+'_TIMING_ONLY', 5.5, 2, 2,
        {g.MAT_WHITE: 200, g.MAT_LINE: 3700, g.PAPER_WHITE: 190, g.PAPER_INK: 1600},
        assumptions=('SYNTHETIC_TIMING_ONLY_NOT_GPIO_CALIBRATION',)) for n in ('GP26', 'GP27'))
    gc_events = []
    def gc_marker(phase, info):
        gc_events.append((time.perf_counter_ns(), phase, info['generation']))
    def batch(i):
        # Included in measured batch; never replay a precomputed full pose list.
        x = 140*math.sin(i*.00142)
        angle = i*.0007
        axis = (math.cos(angle), math.sin(angle), 0)
        samples = tuple(fast.sample(p, (x+(j-1)*3, j*7, 15.5), axis_u=axis,
                                    sample_us=i*1000) for j, p in enumerate(profiles))
        return sum(s.valid for s in samples)
    for i in range(1000):
        if batch(i) != 3:
            raise ValueError('synthetic timing warmup invalid')
    reports = []
    for repeat in range(repeats):
        gc_events.clear()
        samples, valid_frames = MeasurementBuffers(count), 0
        gc.callbacks.append(gc_marker)
        try:
            with timing_gc_policy(gc_mode) as policy:
                # Initial collection/freezing is setup, not timed sampling.
                gc_events.clear()
                for i in range(count):
                    start = time.perf_counter_ns()
                    cpu_start = time.thread_time_ns()
                    valid = batch(i)
                    cpu_end = time.thread_time_ns()
                    end = time.perf_counter_ns()
                    samples.record(i, (end-start)/1000, (cpu_end-cpu_start)/1000,
                                   valid, start, end)
                    valid_frames += valid == 3
        finally:
            gc.callbacks.remove(gc_marker)
        with (output/f'timing-{repeat}.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('source_index', 'batch_wall_us', 'batch_thread_cpu_us',
                             'valid_channels', 'wall_start_ns', 'wall_end_ns'))
            writer.writerows(samples.rows())
        result = time_summary(samples.elapsed)
        result['three_valid_frames'] = valid_frames
        result['gc_policy'] = dict(policy)
        result['gc_enabled_after_restore'] = gc.isenabled()
        result['gc_events'] = list(gc_events)
        result['outliers'] = [dict(index=r[0], wall_us=r[1], thread_cpu_us=r[2],
            non_cpu_gap_us=r[1]-r[2],
            gc_overlap=any(r[4] <= e[0] <= r[5] for e in gc_events))
            for r in samples.rows() if r[1] > 500]
        result['outliers_thread_cpu_over500'] = sum(
            r['thread_cpu_us'] > 500 for r in result['outliers'])
        result['outliers_noncpu_gap_over500'] = sum(
            r['non_cpu_gap_us'] > 500 for r in result['outliers'])
        if valid_frames != count:
            result['check'] = 'FAIL'
        reports.append(result)
    return {'setup_us_tracemalloc_enabled': setup_us, 'setup_peak_bytes': peak,
            'compiled_cells': fast.cell_count, 'three_channels': [p.name for p in profiles],
            'synthetic_positions_NOT_surveyed_mounts': True, 'repeats': reports,
            'check': 'PASS' if all(r['check'] == 'PASS' for r in reports) else 'FAIL'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timing-only', action='store_true')
    parser.add_argument('--accuracy-only', action='store_true')
    parser.add_argument('--gc-mode', choices=('enabled', 'freeze-disable'), default='enabled')
    args = parser.parse_args()
    if args.timing_only and args.accuracy_only:
        parser.error('choose at most one mode')
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'classification': 'F1_EXPLORATORY_NOT_GATE1_ACCEPTANCE',
              'python': sys.version, 'platform': platform.platform(),
              'source_sha256': {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                  for name in ('s3_fast.py', 's3_referee.py', 's3_f1_check.py')},
              'profile_sha256': hashlib.sha256(DEFAULT_PATH.read_bytes()).hexdigest(),
              'real_course_coverage': course_coverage()}
    if not args.timing_only:
        report['accuracy'] = accuracy_report()
    if not args.accuracy_only:
        report['timing'] = timing_report(args.output, gc_mode=args.gc_mode)
    report['check'] = 'PASS' if all(report[k]['summary']['check'] == 'PASS' if k == 'accuracy'
        else report[k]['check'] == 'PASS' for k in ('accuracy', 'timing') if k in report) else 'FAIL'
    (args.output/'f1_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v['summary'] if k == 'accuracy' else v for k, v in report.items()
                      if k in ('accuracy', 'timing', 'check')}, indent=2))
    return 0 if report['check'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
