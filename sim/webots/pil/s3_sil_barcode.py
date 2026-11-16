"""Approved SIL barcode geometry and explicit MQTT-derived mat proxies.

Observe-only, no reader admission or F1 anisotropic implementation. Scene
sampling must retain mat/paper/ink boundaries; no centre-only material switch.
"""
import csv
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import statistics

from .s3_geometry import barcode_geometry, MAT_WHITE, MAT_LINE
from .s3_gp28_nonlinear import load_gp28_profile

ASSUMPTIONS_PATH = (Path(__file__).resolve().parents[3]/
                    'calibration/s3_sil_barcode_assumptions.json')


def _record(path):
    record = json.loads(Path(path).read_text())
    if record.get('classification') != 'ASSUMED_SIL_BARCODE_OBSERVE_ONLY':
        raise ValueError('not an approved observe-only SIL barcode record')
    return record


@dataclass(frozen=True)
class SILBarcodeSheet:
    geometry: object
    sheet_centre_mm: tuple
    sheet_bounds_mm: tuple
    ink_bounds_mm: tuple
    quiet_bounds_mm: tuple
    junction_centre_x_mm: float
    assumptions: tuple


def barcode_sheet(*, sheet_start_mm, line_centre_y_mm, surface_z_mm,
                  letter='A', path=ASSUMPTIONS_PATH):
    """Return patches only for paper/ink; caller owns common floor/line scene.

    Scan direction world+X, bars world+Y. Bounds are minX,minY,maxX,maxY;
    quiet bounds are startX,endX. No automatic axle/sensor path correction.
    """
    if not all(math.isfinite(v) for v in
               (sheet_start_mm, line_centre_y_mm, surface_z_mm)):
        raise ValueError('nonfinite barcode placement')
    record = _record(path)
    paper, ink, layout = record['paper'], record['print'], record['layout']
    length, width = paper['scan_length_mm'], paper['transverse_width_mm']
    x = sheet_start_mm+length/2
    y0 = line_centre_y_mm+paper['inner_edge_left_of_line_mm']
    y = y0+width/2
    centre = (x, y, surface_z_mm)
    geometry = barcode_geometry(letter, bar_height_mm=ink['transverse_height_mm'],
        centre_mm=centre, rotation_deg=0, paper_size_mm=(length, width),
        paper_thickness_mm=paper['thickness_mm'], narrow_mm=ink['narrow_mm'],
        wide_mm=ink['wide_mm'])
    half, h = geometry.body_length_mm/2, ink['transverse_height_mm']/2
    return SILBarcodeSheet(geometry, centre,
        (sheet_start_mm, y0, sheet_start_mm+length, y0+width),
        (x-half, y-h, x+half, y+h),
        (x-half-ink['quiet_required_each_end_mm'],
         x+half+ink['quiet_required_each_end_mm']),
        sheet_start_mm+length+layout['paper_end_to_junction_centre_mm'],
        ('A4_LANDSCAPE_OPERATOR_2026_11_16', 'PRINT_CENTRED_ASSUMED',
         'INNER_EDGE_LINE_PLUS_25_MM_ASSUMED',
         'BAR_HEIGHT_160_PLUS_MINUS_10_MM_PHOTO_ESTIMATED',
         'ZERO_PAPER_THICKNESS_ASSUMED',
         'JUNCTION_SPACING_150_MM_ASSUMED', 'OBSERVE_ONLY_SHADOW'))


def load_sil_barcode_profile(*, sigma_along_mm=1, path=ASSUMPTIONS_PATH):
    """New explicit SIL profile; does not modify either GP28 fit JSON.

    Uses sensor-X paper fit and board-axis along1/4/8mm assumption. MAT proxy
    levels are measured unlocated clusters, not measured pure-material ADC.
    Linear mat coverage response is a separate stated assumption.
    """
    record = _record(path)
    profile = load_gp28_profile(sigma_along_mm=sigma_along_mm)
    proxy = record['mat_proxy']
    levels = dict(profile.levels_adc)
    levels.update({MAT_WHITE: proxy['low']['median_adc'],
                   MAT_LINE: proxy['high']['median_adc']})
    assumptions = tuple(a for a in profile.assumptions if a not in
        ('MAT_LEVELS_PLACEHOLDERS_UNSUPPORTED', 'OFFLINE_ONLY_NOT_PHYSICAL_CALIBRATION'))
    return replace(profile, levels_adc=levels,
        mat_response_power=proxy['mat_response_power'],
        evidence_status=record['classification'],
        assumptions=assumptions+('ASSUMED_SIL_NOT_PHYSICAL_CALIBRATION',
            proxy['evidence'], 'LOW_HIGH_CLUSTER_MATERIAL_ASSIGNMENT_ASSUMED',
            'MAT_RESPONSE_POWER_1_ASSUMED_NOT_FITTED',
            'MQTT_MAD_NOT_STATIONARY_NOISE', 'OBSERVE_ONLY_SHADOW'))


def mqtt_mat_proxy(path, boot_id='3162310911'):
    """Reproduce the unlocated two-median cluster summary, no material truth."""
    path = Path(path)
    values = []
    with path.open() as source:
        for row in csv.DictReader(source):
            if row.get('topic') != 'robot/state' or row.get('boot_id') != boot_id:
                continue
            try:
                value = int(row.get('raw_bc', ''))
            except (TypeError, ValueError):
                continue
            if 0 <= value <= 4095:
                values.append(value)
    if len(set(values)) < 2:
        raise ValueError('mat proxy requires at least two distinct raw observations')
    a, b = min(values), max(values)
    for _ in range(100):
        low = [v for v in values if abs(v-a) <= abs(v-b)]
        high = [v for v in values if abs(v-a) > abs(v-b)]
        if not low or not high:
            raise ValueError('empty proxy cluster')
        aa, bb = statistics.median(low), statistics.median(high)
        if (aa, bb) == (a, b):
            break
        a, b = aa, bb
    else:
        raise ValueError('proxy clustering did not converge')
    def summary(cluster):
        median = statistics.median(cluster)
        return {'median_adc': median,
                'mad_adc': statistics.median(abs(v-median) for v in cluster),
                'samples': len(cluster)}
    return {'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'boot_id': boot_id, 'samples': len(values),
            'low': summary(low), 'high': summary(high)}


if __name__ == '__main__':
    import sys
    print(json.dumps(mqtt_mat_proxy(sys.argv[1]), indent=2))
