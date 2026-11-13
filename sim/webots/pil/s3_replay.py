"""Auditable stdlib-only 1D capture fit and independent held-out check.

Fixed Code39 dimensions, one global registration offset, Gaussian spatial
blur and a bounded power response. Not a decoder and not a pose survey.
"""
import argparse
import bisect
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import statistics

from .s3_geometry import barcode_geometry
from .s3_kinematics import Pose2, advance, point_world
from .s3_optics import stripe_coverage

# Frozen before evaluating held-out data. These are exploratory model checks,
# not firmware, sensor, or hardware acceptance limits.
CHECK_LIMITS = {'normalized_rmse_max': .10, 'edge_p95_mm_max': 1.5,
                'run_width_p95_mm_max': 2.0, 'edge_count_required': 30}
COUNTS_PER_MM = 12.414


@dataclass(frozen=True)
class Capture:
    path: str
    sha256: str
    adc: tuple  # (time relative to first ADC, raw ADC, flags)
    motor: tuple  # (same time axis, unwrapped left/right counts)


def _delta32(new, old):
    return (new-old+(1 << 31)) % (1 << 32) - (1 << 31)


def read_capture(path):
    path = Path(path)
    data = path.read_bytes()
    adc, motor = [], []
    for row in csv.reader(data.decode().splitlines()):
        if not row or row[0] not in ('BC_ADC', 'BC_MOTOR'):
            continue
        if len(row) > 1 and row[1] == 'index':
            continue
        target = adc if row[0] == 'BC_ADC' else motor
        expected = 6 if target is adc else 5
        if len(row) != expected or int(row[1]) != len(target):
            raise ValueError('capture row count/index/field mismatch')
        values = tuple(int(x) for x in row[2:5])
        if target is adc and not (0 <= values[1] <= 4095 and
                                     0 <= values[2] <= 65535):
            raise ValueError('invalid ADC/flags')
        target.append(values)
    if len(adc) < 2 or len(motor) < 2:
        raise ValueError('capture needs ADC and motor observations')
    origin = adc[0][0]
    def times(rows):
        result = []
        t = _delta32(rows[0][0], origin)
        previous = rows[0][0]
        for i, row in enumerate(rows):
            if i:
                dt = _delta32(row[0], previous)
                if dt <= 0:
                    raise ValueError('duplicate/reversed capture time')
                t += dt
            result.append((t, *row[1:]))
            previous = row[0]
        return result
    motor = times(motor)
    unwrapped = [motor[0]]
    for a, b in zip(motor, motor[1:]):
        last = unwrapped[-1]
        unwrapped.append((b[0], last[1]+_delta32(b[1], a[1]),
                          last[2]+_delta32(b[2], a[2])))
    return Capture(str(path), hashlib.sha256(data).hexdigest(),
                   tuple(times(adc)), tuple(unwrapped))


def observations(capture, axis='mean_wheel', forward_mm=80, left_mm=77):
    """Return (position_mm, ADC, source_us), plus honest coverage metadata.

    Motor states are constant-twist interpolated, never extrapolated. The
    sensor_x axis assumes no slip, initial stripe normal world+X and surveyed
    offsets that are in fact only nominal. Neither axis repairs firmware data.
    """
    if axis not in ('mean_wheel', 'sensor_x'):
        raise ValueError('unknown replay axis')
    motors = capture.motor
    times = [m[0] for m in motors]
    poses = [Pose2()]
    for a, b in zip(motors, motors[1:]):
        poses.append(advance(poses[-1], (b[1]-a[1])/COUNTS_PER_MM,
                             (b[2]-a[2])/COUNTS_PER_MM))
    def coordinate(time):
        k = bisect.bisect_right(times, time)-1
        if k == len(times)-1 and time == times[-1]:
            k -= 1
        if k < 0 or k >= len(times)-1:
            return None
        a, b = motors[k], motors[k+1]
        f = (time-a[0])/(b[0]-a[0])
        if axis == 'mean_wheel':
            return ((a[1]+a[2]) + f*(b[1]+b[2]-a[1]-a[2]))/(2*COUNTS_PER_MM)
        p = advance(poses[k], f*(b[1]-a[1])/COUNTS_PER_MM,
                     f*(b[2]-a[2])/COUNTS_PER_MM)
        return point_world(p, forward_mm, left_mm)[0]
    initial = coordinate(capture.adc[0][0])
    if initial is None:
        raise ValueError('first ADC lacks bracketed encoder pose')
    rows, outside, flags, saturated = [], 0, 0, 0
    for time, value, flag in capture.adc:
        x = coordinate(time)
        if x is None:
            outside += 1
            continue
        if flag:
            flags += 1
            continue
        if value in (0, 4095):
            saturated += 1
            continue
        rows.append((x-initial, value, time))
    return rows, {'axis': axis, 'unbracketed_adc': outside,
                  'flagged_adc': flags, 'saturated_adc': saturated,
                  'input_adc': len(capture.adc),
                  'motor_max_gap_us': max(b-a for a, b in zip(times, times[1:])),
                  'counts_per_mm': COUNTS_PER_MM,
                  'pose_assumptions': ['no_slip', 'constant_twist_between_motor',
                                       'initial_stripe_normal_world_X',
                                       'mount_80_forward_77_left_nominal']}


def crossings(rows, rise=320, fall=240):
    """Diagnostic fixed hysteresis, not a replacement for the live reader."""
    result = []
    black = False
    previous = None
    for row in rows:
        x, value = row[:2]
        threshold = fall if black else rise
        changed = value <= threshold if black else value >= threshold
        if changed:
            if previous is None:
                at = x
            else:
                px, pv = previous
                f = (threshold-pv)/(value-pv) if value != pv else 1
                at = px+max(0, min(1, f))*(x-px)
            result.append(at)
            black = not black
        previous = (x, value)
    return result


def _bars(letter):
    b = barcode_geometry(letter, bar_height_mm=1)
    return tuple((a+b.body_length_mm/2, z+b.body_length_mm/2)
                 for a, z in b.bar_intervals_mm)


@dataclass(frozen=True)
class Fit:
    letter: str
    baseline_adc: float
    gain_adc: float
    sigma_mm: float
    response_power: float
    offset_mm: float
    search_bound_hit: bool


def predict(rows, fit, offset_mm=None):
    bars = _bars(fit.letter)
    offset = fit.offset_mm if offset_mm is None else offset_mm
    return [(x, fit.baseline_adc + fit.gain_adc *
             stripe_coverage(x-offset, bars, fit.sigma_mm)**fit.response_power,
             time) for x, _, time in rows]


def _roi(rows):
    # Original physical 430 mm TEST bound. The older 600 mm trace has extra
    # off-mat dark regions after453 mm; don't fit those as barcode ink.
    selected = [r for r in rows if 0 <= r[0] <= 430]
    if len(selected) < 100:
        raise ValueError('insufficient bracketed observations in 0..430 mm ROI')
    return selected


def _start_guess(rows):
    edges = crossings(rows)
    if len(edges) < 2:
        raise ValueError('no leading barcode observation for registration')
    # First star bar is narrow. This is pose registration, not scale fitting.
    return (edges[0]+edges[1])/2 - 5.8/2


def fit_capture(rows, letter='F'):
    rows = _roi(rows)
    early = [y for x, y, _ in rows if x < 30]
    if len(early) < 16 or not 120 <= statistics.median(early) <= 220:
        raise ValueError('no credible independent leading-white baseline')
    baseline = float(statistics.median(early))
    peak = sorted(y for _, y, _ in rows)[int(.99*(len(rows)-1))]
    bounds = (.85*(peak-baseline), 1.25*(peak-baseline))
    if bounds[0] <= 0:
        raise ValueError('capture lacks contrast')
    bars, seed = _bars(letter), _start_guess(rows)
    sparse = rows[::8]
    def evaluate(sigma, power, offset, points):
        if not (.5 <= sigma <= 5 and 1 <= power <= 16):
            return (float('inf'), 0)
        shape = [stripe_coverage(x-offset, bars, sigma)**power for x, _, _ in points]
        denominator = sum(q*q for q in shape)
        gain = sum(q*(r[1]-baseline) for q, r in zip(shape, points))/max(denominator, 1e-12)
        gain = max(bounds[0], min(bounds[1], gain))
        loss = sum((baseline+gain*q-r[1])**2 for q, r in zip(shape, points))/len(points)
        return loss, gain
    best = None
    for sigma in (.5, .75, 1, 1.25, 1.5, 1.75, 2, 2.5, 3, 4, 5):
        for i in range(-12, 13):
            offset = seed+i*.5
            shape = [stripe_coverage(x-offset, bars, sigma) for x, _, _ in sparse]
            for power in (1, 2, 4, 6, 8, 10, 12, 16):
                q = [v**power for v in shape]
                gain = sum(v*(r[1]-baseline) for v, r in zip(q, sparse))/max(sum(v*v for v in q), 1e-12)
                gain = max(bounds[0], min(bounds[1], gain))
                loss = sum((baseline+gain*v-r[1])**2 for v, r in zip(q, sparse))/len(sparse)
                candidate = (loss, sigma, power, offset, gain)
                if best is None or candidate < best:
                    best = candidate
    points = rows[::3]
    for steps in ((.25, 2, .25), (.1, .5, .1), (.025, .125, .025)):
        _, sigma, power, offset, _ = best
        loss, gain = evaluate(sigma, power, offset, points)
        best = (loss, sigma, power, offset, gain)
        for _ in range(30):
            previous = best
            for axis, step in enumerate(steps):
                for sign in (-1, 1):
                    params = list(best[1:4])
                    params[axis] += sign*step
                    if abs(params[2]-seed) > 8:
                        continue
                    loss, gain = evaluate(*params, points)
                    candidate = (loss, *params, gain)
                    if candidate < best:
                        best = candidate
            if best == previous:
                break
    _, sigma, power, offset, _ = best
    _, gain = evaluate(sigma, power, offset, rows)
    bound = (sigma <= .501 or sigma >= 4.999 or power <= 1.001 or
             power >= 15.999 or abs(offset-seed) >= 7.999 or
             abs(gain-bounds[0]) < 1e-6 or abs(gain-bounds[1]) < 1e-6)
    return Fit(letter, baseline, gain, sigma, power, offset, bound)


def register_only(rows, fit):
    """Held-out nuisance translation ONLY; never refit response/blur/scale."""
    rows = _roi(rows)
    seed = _start_guess(rows)
    sparse = rows[::8]
    def loss(offset, points):
        predicted = predict(points, fit, offset)
        return sum((a[1]-b[1])**2 for a, b in zip(points, predicted))/len(points)
    best = min((loss(seed+i*.25, sparse), seed+i*.25) for i in range(-32, 33))
    for step in (.1, .025):
        best = min((loss(best[1]+i*step, rows[::3]), best[1]+i*step)
                   for i in range(-5, 6) if abs(best[1]+i*step-seed) <= 8)
    return best[1]


def _percentile(values, fraction):
    values = sorted(values)
    return values[round((len(values)-1)*fraction)] if values else None


def evaluate(rows, fit, offset=None):
    rows = _roi(rows)
    predicted = predict(rows, fit, offset)
    errors = [b[1]-a[1] for a, b in zip(rows, predicted)]
    observed_edges, model_edges = crossings(rows), crossings(predicted)
    matching = len(observed_edges) == len(model_edges) == 30
    edge_errors = [abs(a-b) for a, b in zip(observed_edges, model_edges)] if matching else []
    width_errors = [abs((b-a)-(d-c)) for a, b, c, d in
                    zip(observed_edges, observed_edges[1:], model_edges, model_edges[1:])] if matching else []
    rmse = math.sqrt(sum(x*x for x in errors)/len(errors))
    edge95, width95 = _percentile(edge_errors, .95), _percentile(width_errors, .95)
    passed = (rmse/fit.gain_adc <= CHECK_LIMITS['normalized_rmse_max'] and matching and
              edge95 <= CHECK_LIMITS['edge_p95_mm_max'] and
              width95 <= CHECK_LIMITS['run_width_p95_mm_max'])
    return {'samples': len(rows), 'rmse_adc': rmse,
            'normalized_rmse': rmse/fit.gain_adc,
            'mae_adc': sum(abs(x) for x in errors)/len(errors),
            'observed_edge_count': len(observed_edges), 'model_edge_count': len(model_edges),
            'edge_p95_error_mm': edge95, 'run_width_p95_error_mm': width95,
            'observed_edges_mm': observed_edges, 'model_edges_mm': model_edges,
            'registration_offset_mm': fit.offset_mm if offset is None else offset,
            'check': 'PASS' if passed else 'FAIL',
            'limits': dict(CHECK_LIMITS)}


def save_waveform(path, rows, fit, offset):
    predicted = predict(rows, fit, offset)
    with Path(path).open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('source_us', 'position_mm', 'observed_adc', 'model_adc'))
        for a, b in zip(rows, predicted):
            writer.writerow((a[2], a[0], a[1], b[1]))


def profile_bundle(sensor_report, mean_report, train_rows):
    """Build an EXPLORATORY frozen record, including failed held-out checks.

    No untracked capture data is read. GP26/27 missing data remains null.
    """
    if not all('training' in report and 'holdout' in report and 'fit' in report
               for report in (sensor_report, mean_report)):
        raise ValueError('incomplete replay reports')
    if (sensor_report['training']['coverage']['axis'] != 'sensor_x' or
            mean_report['training']['coverage']['axis'] != 'mean_wheel'):
        raise ValueError('both recorded axis variants are required')
    for key in ('training', 'holdout'):
        if sensor_report[key]['sha256'] != mean_report[key]['sha256']:
            raise ValueError('axis variants must use identical capture inputs')
        if any(report[key]['metrics']['limits'] != CHECK_LIMITS
               for report in (sensor_report, mean_report)):
            raise ValueError('report check limits differ from the frozen limits')
    if sensor_report['training']['sha256'] == sensor_report['holdout']['sha256']:
        raise ValueError('held-out content must differ from training')
    white = [y for x, y, _ in train_rows if 0 <= x < 30]
    median = statistics.median(white)
    mad = statistics.median(abs(y-median) for y in white)
    fit = sensor_report['fit']
    profile = {
        'name': 'GP28', 'reference_height_mm': 5.5,
        'sigma_x_mm': fit['sigma_mm'], 'sigma_y_mm': fit['sigma_mm'],
        'levels_adc': {'MAT_WHITE': 167.0, 'MAT_LINE': 2673.0,
                       'PAPER_WHITE': fit['baseline_adc'],
                       'PAPER_INK': fit['baseline_adc']+fit['gain_adc']},
        'noise_sigma_adc': 1.4826*mad,
        'paper_response_power': fit['response_power'],
        'mat_response_power': 1.0,
        'evidence_status': 'EXPLORATORY_EXPECTED_FAIL',
        'assumptions': [
            '5.5mm_reference_is_operator_nominal_not_capture_height_survey',
            'height_and_tilt_response_unmeasured',
            'transverse_sigma_assumed_equal_longitudinal',
            'mount_80_forward_77_left_nominal_no_slip',
            'mat_high2673_is_sweep_mean_not_pure_material_plateau',
            'mat_power1_and_mixed_material_response_unmeasured',
            'post8_average_noise_white_only_no_double_averaging',
            'finite_ray_sampling_has_separate_resolution_error',
            'unexplained_waveform_residual_not_added_as_noise',
            'heldout_edge_check_failed_no_physical_generalisation'],
    }
    variants = {}
    for axis, report in (('sensor_x', sensor_report), ('mean_wheel', mean_report)):
        variants[axis] = {'fit': report['fit']}
        for kind in ('training', 'holdout'):
            entry = report[kind]
            variants[axis][kind] = {
                key: value for key, value in entry.items() if key != 'metrics'}
            variants[axis][kind]['metrics'] = {
                key: value for key, value in entry['metrics'].items()
                if key not in ('observed_edges_mm', 'model_edges_mm')}
    unavailable = {
        'profile': None, 'evidence_status': 'UNMEASURED',
        'reason': 'No per-material response/footprint or current surveyed mount',
    }
    return {
        'schema_version': 1,
        'classification': 'EXPLORATORY_HOST_REFERENCE_NOT_SENSOR_CALIBRATION',
        'units': {'position': 'mm', 'time': 'us', 'ADC': 'post8_average_12bit'},
        'channels': {
            'GP28': {'profile': profile,
                     'mount': {'forward_mm_nominal': 80, 'left_mm_nominal': 77,
                               'flat_clearance_mm_nominal': 5.5,
                               'forward_uncertainty_mm': 5,
                               'surveyed_current_mount': False}},
            'GP26': {**unavailable, 'mount': {
                'forward_mm': None, 'left_mm_relative_GP26': 0,
                'current_clearance_mm': None, 'historical_height_mm_approx': 9}},
            'GP27': {**unavailable, 'mount': {
                'forward_mm': None, 'left_mm_relative_GP26': 14.5,
                'current_clearance_mm': None, 'historical_height_mm_approx': 9}},
        },
        'noise_fit': {'source': sensor_report['training']['path'],
                      'sha256': sensor_report['training']['sha256'],
                      'axis': 'sensor_x', 'ROI_mm': [0, 30],
                      'sample_count': len(white), 'median_adc': median,
                      'MAD_adc': mad, 'robust_sigma_adc': 1.4826*mad,
                      'population_sigma_adc': statistics.pstdev(white),
                      'scope': 'post8_average_white_only_not_full_waveform_error'},
        'other_material_reference': 'docs/s3-line-barcode/LIVE_CALIBRATION.md',
        'fixed_geometry_mm': sensor_report['fixed_geometry_mm'],
        'check_limits': dict(CHECK_LIMITS),
        'fit_search_bounds': {
            'sigma_mm': [.5, 5], 'response_power': [1, 16],
            'registration_mm_about_first_star': [-8, 8],
            'gain_relative_to_top1percent_contrast': [.85, 1.25],
            'no_print_scale_fit': True},
        'replay_variants': variants,
        'release_gate': 'STOP_BEFORE_GATE1',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True)
    parser.add_argument('--holdout', type=Path, required=True)
    parser.add_argument('--axis', choices=('mean_wheel', 'sensor_x'), default='sensor_x')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.train.resolve() == args.holdout.resolve():
        parser.error('training and held-out captures must differ')
    train, holdout = read_capture(args.train), read_capture(args.holdout)
    if train.sha256 == holdout.sha256:
        parser.error('training and held-out content must differ')
    a, meta_a = observations(train, args.axis)
    b, meta_b = observations(holdout, args.axis)
    if any(m[k] for m in (meta_a, meta_b)
           for k in ('flagged_adc', 'saturated_adc')):
        parser.error('flagged/saturated rows require a missing-data study')
    fit = fit_capture(a)
    offset = register_only(b, fit)
    report = {'schema_version': 1, 'fit': asdict(fit),
              'fixed_geometry_mm': {'narrow': 5.8, 'wide': 17.5, 'body': 273.5},
              'training': {'path': str(args.train), 'sha256': train.sha256,
                           'coverage': meta_a, 'metrics': evaluate(a, fit)},
              'holdout': {'path': str(args.holdout), 'sha256': holdout.sha256,
                          'refitted_parameters': ['global_pose_translation_only'],
                          'coverage': meta_b, 'metrics': evaluate(b, fit, offset)},
              'acceptance': 'exploratory_model_check_NOT_hardware_acceptance'}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'fit_report.json').write_text(json.dumps(report, indent=2)+'\n')
    save_waveform(args.output/'training_waveform.csv', a, fit, fit.offset_mm)
    save_waveform(args.output/'holdout_waveform.csv', b, fit, offset)
    print(json.dumps({'fit': asdict(fit),
                      'training': report['training']['metrics'],
                      'holdout': report['holdout']['metrics']}, indent=2))


if __name__ == '__main__':
    main()
