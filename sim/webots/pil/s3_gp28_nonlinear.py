"""Opt-in offline GP28 power-response fit with a rotated covariance family.

Along-bar sigma is NOT identified. Caller must choose a labelled assumption.
No firmware, frozen-profile fallback, or F1 anisotropic support is added.
"""
import bisect
from dataclasses import asdict
import json
import math
from pathlib import Path

from .s3_kinematics import Pose2, advance
from .s3_optics import SensorProfile, stripe_coverage
from .s3_geometry import barcode_geometry
from .s3_replay import (COUNTS_PER_MM, CHECK_LIMITS, observations, read_capture,
                        fit_capture, crossings, _bars, _roi, _start_guess,
                        _percentile)

ROOT = Path(__file__).resolve().parents[3]
PROFILE_PATH = ROOT/'calibration/s3_gp28_nonlinear_profiles.json'
ALONG_FAMILY_MM = (1., 4., 8.)


def board_axis_u(yaw_rad):
    """Board U = body-left at zero yaw:90deg from line-board reference U."""
    return (-math.sin(yaw_rad), math.cos(yaw_rad), 0.)


def projected_sigma(yaw_rad, sigma_along_mm, sigma_cross_mm):
    """Project rotated board covariance onto world-X barcode normal."""
    return math.hypot(sigma_along_mm*math.sin(yaw_rad),
                      sigma_cross_mm*math.cos(yaw_rad))


def capture_rows(capture, axis):
    rows, meta = observations(capture, axis)
    poses, times = [Pose2()], [m[0] for m in capture.motor]
    for a, b in zip(capture.motor, capture.motor[1:]):
        poses.append(advance(poses[-1], (b[1]-a[1])/COUNTS_PER_MM,
                             (b[2]-a[2])/COUNTS_PER_MM))
    result = []
    for x, adc, t in rows:
        i = min(bisect.bisect_right(times, t)-1, len(times)-2)
        fraction = (t-times[i])/(times[i+1]-times[i])
        yaw = poses[i].yaw_rad+fraction*(poses[i+1].yaw_rad-poses[i].yaw_rad)
        result.append((x, adc, t, yaw))
    meta['yaw_rad_range'] = [min(r[3] for r in result), max(r[3] for r in result)]
    return result, meta


def predict_rows(rows, fit, sigma_along_mm, offset=None):
    geometry = barcode_geometry(fit['letter'], bar_height_mm=1,
        narrow_mm=fit.get('narrow_mm', 5.8), wide_mm=fit.get('wide_mm', 17.5))
    bars = tuple((a+geometry.body_length_mm/2, b+geometry.body_length_mm/2)
                 for a, b in geometry.bar_intervals_mm)
    offset = fit['offset_mm'] if offset is None else offset
    return [(x, fit['baseline_adc']+fit['gain_adc']*stripe_coverage(
        x-offset, bars, projected_sigma(yaw, sigma_along_mm,
                                       fit['sigma_mm']))**fit['response_power'], t)
            for x, _, t, yaw in rows]


def evaluate_rows(rows, fit, sigma_along_mm, offset=None):
    rows = _roi(rows)
    predicted = predict_rows(rows, fit, sigma_along_mm, offset)
    errors = [b[1]-a[1] for a, b in zip(rows, predicted)]
    observed_edges, model_edges = crossings(rows), crossings(predicted)
    matching = len(observed_edges) == len(model_edges) == 30
    edges = [abs(a-b) for a, b in zip(observed_edges, model_edges)] if matching else []
    widths = [abs((b-a)-(d-c)) for a, b, c, d in
              zip(observed_edges, observed_edges[1:], model_edges,
                  model_edges[1:])] if matching else []
    rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
    edge95, width95 = _percentile(edges, .95), _percentile(widths, .95)
    passed = (matching and rmse/fit['gain_adc'] <= .10 and
              edge95 <= 1.5 and width95 <= 2)
    return {'samples': len(rows), 'rmse_adc': rmse,
            'normalized_rmse': rmse/fit['gain_adc'],
            'max_abs_adc': max(abs(e) for e in errors),
            'edge_p95_error_mm': edge95, 'run_width_p95_error_mm': width95,
            'observed_edge_count': len(observed_edges),
            'model_edge_count': len(model_edges),
            'registration_offset_mm': fit['offset_mm'] if offset is None else offset,
            'check': 'PASS' if passed else 'FAIL', 'limits': dict(CHECK_LIMITS)}


def fit_family(rows, along, initial):
    rows = _roi(rows)
    baseline = initial['baseline_adc']
    peak = sorted(r[1] for r in rows)[int(.99*(len(rows)-1))]
    bounds = (.85*(peak-baseline), 1.25*(peak-baseline))
    seed = _start_guess(rows)
    bars = _bars('F')
    points = rows[::3]

    def score(params, points):
        sigma, power, offset = params
        if not (.5 <= sigma <= 5 and 1 <= power <= 16 and abs(offset-seed) <= 8):
            return float('inf'), 0
        q = [stripe_coverage(x-offset, bars, projected_sigma(yaw, along, sigma))**power
             for x, _, _, yaw in points]
        gain = sum(v*(r[1]-baseline) for v, r in zip(q, points))/max(sum(v*v for v in q), 1e-12)
        gain = max(bounds[0], min(bounds[1], gain))
        return sum((baseline+gain*v-r[1])**2 for v, r in zip(q, points))/len(points), gain

    p = [initial['sigma_mm'], initial['response_power'], initial['offset_mm']]
    best = score(p, points)[0]
    for steps in ((.1, .5, .1), (.025, .125, .025), (.005, .025, .005)):
        for _ in range(40):
            improved = False
            for i, step in enumerate(steps):
                for sign in (-1, 1):
                    candidate = p.copy()
                    candidate[i] += sign*step
                    loss, _ = score(candidate, points)
                    if loss < best:
                        p, best, improved = candidate, loss, True
            if not improved:
                break
    _, gain = score(p, rows)
    sigma, power, offset = p
    return {'letter': 'F', 'baseline_adc': baseline, 'gain_adc': gain,
            'sigma_mm': sigma, 'response_power': power, 'offset_mm': offset,
            'search_bound_hit': (sigma <= .501 or sigma >= 4.999 or
                power <= 1.001 or power >= 15.999 or abs(offset-seed) >= 7.999 or
                min(abs(gain-b) for b in bounds) < 1e-6)}


def register_holdout(rows, fit, along):
    rows = _roi(rows)
    seed = _start_guess(rows)
    def loss(offset, points):
        predictions = predict_rows(points, fit, along, offset)
        return sum((p[1]-r[1])**2 for p, r in zip(predictions, points))/len(points)
    best = min((loss(seed+i*.25, rows[::8]), seed+i*.25) for i in range(-32, 33))
    for step in (.1, .025):
        best = min((loss(best[1]+i*step, rows[::3]), best[1]+i*step)
                   for i in range(-5, 6) if abs(best[1]+i*step-seed) <= 8)
    return best[1]


def build_record():
    train = read_capture(ROOT/'tests/fixtures/barcode_430_large_fwd.csv')
    holdout = read_capture(ROOT/'tests/fixtures/barcode_large_f_forward.csv')
    variants = {}
    for axis in ('sensor_x', 'mean_wheel'):
        training, tmeta = capture_rows(train, axis)
        validation, hmeta = capture_rows(holdout, axis)
        initial = asdict(fit_capture([r[:3] for r in training]))
        families = {}
        # Along width is a predeclared sensitivity family, NOT selected on holdout.
        for along in ALONG_FAMILY_MM:
            fitted = fit_family(training, along, initial)
            offset = register_holdout(validation, fitted, along)
            families[str(along)] = {'fit': fitted,
                'train': evaluate_rows(training, fitted, along),
                'holdout': evaluate_rows(validation, fitted, along, offset)}
        sensitivity = []
        for narrow, wide in ((5.2, 19.), (6.4, 16.)):
            # Photo-bounded opposite corners also satisfy body278+/-6mm.
            assumed = dict(families['4.0']['fit'], narrow_mm=narrow, wide_mm=wide)
            results = {}
            for split, rows in (('train', training), ('holdout', validation)):
                offset = register_holdout(rows, assumed, 4.)
                results[split] = evaluate_rows(rows, assumed, 4., offset)
            sensitivity.append({'narrow_mm': narrow, 'wide_mm': wide,
                'body_mm': 20*narrow+9*wide,
                'method': 'fixed nominal optical parameters, translation only',
                'results': results})
        variants[axis] = {'photo_dimension_sensitivity': sensitivity,
                          'train_source_sha256': train.sha256,
                          'holdout_source_sha256': holdout.sha256,
                          'train_coverage': tmeta, 'holdout_coverage': hmeta,
                          'families': families}
    return {'schema_version': 1, 'classification': 'EXPLORATORY_GP28_OFFLINE_ONLY',
            'label': 'fitted to barcode430 train; large-forward holdout retained',
            'reference_height_mm': 5.5, 'mount_board_rotation_deg': 90,
            'rotation_evidence': 'OPERATOR_2026_11_15_PERPENDICULAR_SAME_CYTRON_MODULE',
            'board_U_reference': 'body-left at zero yaw; line-board reference body-forward assumed',
            'along_bar_sigma_identified': False,
            'along_bar_sigma_assumed_family_mm': list(ALONG_FAMILY_MM),
            'geometry': {'letter': 'F', 'narrow_mm': 5.8, 'wide_mm': 17.5,
                         'fixed_body_mm': 273.5, 'transverse_bar_extent': 'unmeasured; infinite-stripe fit'},
            'photo_constraints_mm': {'narrow': [5.8, .6], 'wide': [17.5, 1.5],
                                     'body': [278, 6]},
            'fit_limits': 'ROI0..430mm; baseline leading-white median; gain bounded85..125percent of99th-percentile train contrast; sigmaCross0.5..5mm,power1..16; no count/width scale fit',
            'limitations': ['No transverse/height law; no physical acceptance',
                           'Small yaw range cannot identify along-bar width',
                           'Mat levels and arbitrary-yaw F1 runtime unsupported',
                           'Photo dimensions uncertain, not retuned to improve fit',
                           'MQTT and unmatched sheets not used to select parameters'],
            'variants': variants}


def load_gp28_profile(variant='sensor_x', sigma_along_mm=None, path=PROFILE_PATH):
    """Offline SensorProfile in BOARD axes; explicit unmeasured along-bar width."""
    data = json.loads(Path(path).read_text())
    if sigma_along_mm not in ALONG_FAMILY_MM:
        raise ValueError('explicit along-bar sigma assumption required:1,4,8mm')
    if data.get('classification') != 'EXPLORATORY_GP28_OFFLINE_ONLY':
        raise ValueError('not an exploratory GP28 offline profile')
    try:
        fit = data['variants'][variant]['families'][str(float(sigma_along_mm))]['fit']
    except KeyError as error:
        raise ValueError('unknown offline GP28 variant/family') from error
    return SensorProfile(name='GP28', reference_height_mm=data['reference_height_mm'],
        sigma_x_mm=float(sigma_along_mm), sigma_y_mm=fit['sigma_mm'],
        levels_adc={'PAPER_WHITE': fit['baseline_adc'],
                    'PAPER_INK': fit['baseline_adc']+fit['gain_adc'],
                    'MAT_WHITE': 164., 'MAT_LINE': 1500.},
        noise_sigma_adc=0, paper_response_power=fit['response_power'],
        mat_response_power=1, evidence_status=data['classification'],
        assumptions=('OFFLINE_ONLY_NOT_PHYSICAL_CALIBRATION',
                     'ALONG_BAR_SIGMA_ASSUMED_NOT_IDENTIFIED',
                     'BOARD_ROTATED_90_DEG_OPERATOR',
                     'MAT_LEVELS_PLACEHOLDERS_UNSUPPORTED',
                     'HEIGHT_APPROX_MEASURED_OPERATOR_2026_11_15'))


if __name__ == '__main__':
    print(json.dumps(build_record(), indent=2))
