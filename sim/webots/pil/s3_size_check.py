"""Host-only optical stripe SIL through the real C window reader.

ASSUMED Gaussian marginal along print scan, no-slip motion, flat paper with
unlimited white outside the ink. This is not the finite-paper Webots scene,
not the school 5 Hz ADC replay and not a measured minimum readable size.
Frozen sensor profiles are not loaded or written. Cross sigma=2.15 mm is an
estimate; along sigma, linear ADC response and all motion/noise are ASSUMED.
"""
import argparse
from collections import Counter
import ctypes
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess
import tempfile

from .s3_geometry import barcode_elements
from .s3_kinematics import Pose2, advance, point_world
from .s3_optics import stripe_coverage

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = r'''
#include "barcode_window.h"
static barcode_window_t state;
int reset_reader(void) {
    const barcode_cal_report_t cal = {.mode=BC_CAL_FALLBACK,
        .white_mean=160, .white_samples=32, .rise=320, .fall=240,
        .faults=BC_CAL_NO_BLACK};
    return barcode_window_begin(&state, &cal, 0);
}
void feed_reader(unsigned int us, unsigned int left, unsigned int right,
                 unsigned int raw) {
    if (us % 10000u == 0u)
        barcode_window_motor(&state, us, left, right, 0);
    barcode_window_adc(&state, us, (unsigned short)raw, 0);
    barcode_window_step(&state, BW_STEP_MAX);
}
unsigned int accepted(void) { return state.result.accepted; }
unsigned int letter(void) { return (unsigned char)state.result.letter; }
unsigned int reason(void) { return state.result.reason; }
float narrow(void) { return state.result.narrow_mm; }
'''


class Reader:
    """Temporary host build, fixed C ABI wrapper; never edits firmware."""
    def __init__(self, directory, root=ROOT):
        directory, root = Path(directory), Path(root)
        source = root / 'app_program/subsystem3'
        wrapper = directory / 'reader.c'
        wrapper.write_text(WRAPPER)
        library = directory / 'reader.so'
        subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-shared', '-fPIC', '-I', str(source), str(wrapper),
                        str(source/'barcode_window.c'), str(source/'barcode.c'),
                        '-lm', '-o', str(library)], check=True)
        self.lib = ctypes.CDLL(str(library))
        self.lib.reset_reader.restype = ctypes.c_int
        self.lib.feed_reader.argtypes = [ctypes.c_uint] * 4
        self.lib.feed_reader.restype = None
        for name in ('accepted', 'letter', 'reason'):
            getattr(self.lib, name).restype = ctypes.c_uint
        self.lib.narrow.restype = ctypes.c_float


def print_bars(letter, body_mm, reverse):
    """Scale all 29 elements, including both inter-character gaps."""
    if not math.isfinite(body_mm) or body_mm <= 0:
        raise ValueError('body must be positive')
    elements = barcode_elements(letter, reverse=reverse)
    scale = body_mm / sum(elements)
    x, bars = 80.0, []
    for i, width in enumerate(elements):
        end = x + width * scale
        if i % 2 == 0:
            bars.append((x, end))
        x = end
    return tuple(bars)


def check_case(reader, *, kind='barcode', body_mm=210, letter='F',
               reverse=False, sigma_mm=1, speed_mm_s=146, yaw_peak_dps=0,
               noise_adc=0, seed=1):
    """Infinite transverse bars; yaw uses the exact Gaussian normal marginal.

    Mean wheel speed is constant; sinusoidal yaw has peak degrees/s, 0.7 Hz.
    Encoder snapshots are quantized at 10 ms, ADC at 1 ms. The independent
    no-slip pose supplies GP28 world-X, not the reader's fitted axis output.
    No lateral/end-of-bar visibility claim is made by this stripe model.
    """
    values = (body_mm, sigma_mm, speed_mm_s, yaw_peak_dps, noise_adc)
    if not all(math.isfinite(v) for v in values) or min(
            body_mm, speed_mm_s) <= 0 or min(sigma_mm, noise_adc) < 0:
        raise ValueError('invalid SIL case')
    if kind == 'barcode':
        bars = print_bars(letter, body_mm, reverse)
    elif kind == 'line':
        bars = ((100, 118),)
    elif kind == 'junction':
        bars = ((80, 98), (180, 215), (280, 350))
    elif kind == 'white':
        bars = ()
    else:
        raise ValueError('unsupported SIL case')
    assert reader.lib.reset_reader()
    rng, pose = random.Random(seed), Pose2()
    left = right = 0.0
    # Include leading white + body + trailing proof even at 90 mm/s; a fixed
    # four-second horizon truncates the 274 mm positive before white proof.
    horizon_us = math.ceil((body_mm + 240) / speed_mm_s * 100) * 10000
    for us in range(0, horizon_us + 1, 1000):
        if us:
            omega = math.radians(yaw_peak_dps) * math.sin(
                2 * math.pi * .7 * (us - 500) / 1e6)
            dl = (speed_mm_s - omega * 111 / 2) * .001
            dr = (speed_mm_s + omega * 111 / 2) * .001
            left, right = left + dl, right + dr
            pose = advance(pose, dl, dr)
        x, _ = point_world(pose)
        x -= 80  # initial GP28 origin, independent of the decoder
        sigma = math.hypot(sigma_mm * math.cos(pose.yaw_rad),
                           2.15 * math.sin(pose.yaw_rad))
        coverage = stripe_coverage(x, bars, sigma)
        raw = 160 + 840 * coverage + rng.uniform(-noise_adc, noise_adc)
        reader.lib.feed_reader(us, round(left * 12.414),
                               round(right * 12.414), round(min(4094, max(0, raw))))
    accepted = bool(reader.lib.accepted())
    result = dict(kind=kind, body_mm=body_mm, letter_expected=letter,
                  reverse=reverse, sigma_along_ASSUMED_mm=sigma_mm,
                  speed_ASSUMED_mm_s=speed_mm_s,
                  yaw_peak_ASSUMED_dps=yaw_peak_dps,
                  noise_bound_ASSUMED_adc=noise_adc, accepted=accepted,
                  letter=chr(reader.lib.letter()) if accepted else None,
                  reason=reader.lib.reason(), narrow_mm=reader.lib.narrow())
    result['wrong_letter'] = accepted and (kind != 'barcode' or
                                           result['letter'] != letter)
    return result


def sweep(reader):
    cases = []
    # Positive, scale and direction gates across several ordinary letters.
    for body in (274, 210, 141):
        for letter in 'AFMZ':
            for reverse in (False, True):
                for sigma in (0, .5, 1, 2.15, 4):
                    cases.append(check_case(reader, body_mm=body, letter=letter,
                        reverse=reverse, sigma_mm=sigma))
    # Exploratory steering/speed/noise, not a promise to decode every pose.
    for body in (274, 210):
        for speed in (90, 146, 200):
            for yaw in (0, 25, 55):
                for sigma in (1, 4):
                    cases.append(check_case(reader, body_mm=body,
                        speed_mm_s=speed, yaw_peak_dps=yaw, sigma_mm=sigma,
                        noise_adc=10))
    for kind in ('line', 'junction', 'white'):
        for sigma in (0, 1, 2.15, 4):
            for yaw in (0, 55):
                cases.append(check_case(reader, kind=kind, sigma_mm=sigma,
                                        yaw_peak_dps=yaw, noise_adc=10))
    failures = []
    for i, case in enumerate(cases):
        if case['wrong_letter']:
            failures.append((i, 'wrong letter/negative decode'))
        if case['kind'] == 'barcode' and case['body_mm'] == 141 and case['accepted']:
            failures.append((i, 'out-of-scope 141 mm accepted'))
        if (case['kind'] == 'barcode' and case['body_mm'] in (274, 210)
                and case['sigma_along_ASSUMED_mm'] <= 1
                and case['yaw_peak_ASSUMED_dps'] == 0 and not case['accepted']):
            failures.append((i, 'straight low-blur positive rejected'))
    return {'classification': 'ASSUMED_HOST_ONLY_STRIPE_SIL_NOT_HARDWARE',
            'reader_sha256': hashlib.sha256(
                (ROOT/'app_program/subsystem3/barcode_window.c').read_bytes()).hexdigest(),
            'cases': cases, 'count': len(cases), 'failures': failures,
            'accepted': sum(c['accepted'] for c in cases),
            'rejections_by_reason': dict(Counter(c['reason'] for c in cases
                                                if not c['accepted'])),
            'gate': 'FAIL' if failures else 'PASS',
            'limits': ['infinite transverse bars on infinite white paper',
                'linear ADC coverage and all along blur are ASSUMED',
                'no finite paper edge, lateral visibility, wheel slip or Webots',
                'not a replay of school 5 Hz MQTT or a false-positive rate']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        report = sweep(Reader(directory))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(f"{report['gate']}: {report['count']} cases, "
          f"{report['accepted']} accepted, {len(report['failures'])} gate failures")
    return int(bool(report['failures']))


if __name__ == '__main__':
    raise SystemExit(main())
