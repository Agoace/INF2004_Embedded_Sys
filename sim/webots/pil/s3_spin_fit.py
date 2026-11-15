"""Offline boot-5209 joint fit. Standard library; not a runtime pose estimator."""
import csv
import hashlib
import json
import math
from pathlib import Path

ENDPOINTS = {'GP27': (172, 2487), 'GP26': (158, 2611)}


def coverage(offset, sigma):
    return .5*(math.erf((9-offset)/(math.sqrt(2)*sigma))-
               math.erf((-9-offset)/(math.sqrt(2)*sigma)))


def parameters(sigma):
    c0 = coverage(0, sigma)
    power = math.log(.39)/math.log(coverage(7.25, sigma)/c0)
    return c0, power


def normalized(offset, sigma, c0, power):
    return (coverage(offset, sigma)/c0)**power


def fit(path):
    path = Path(path)
    with path.open() as source:
        rows = list(csv.DictReader(source))
    # Left-count extrema separate the three commanded sweeps. Pauses are not
    # recorded. Right counts are absent: each sweep gets its own angle/count
    # and C-centre count; no constant angular-rate or slip claim is made.
    counts = [int(row['count']) for row in rows]
    low = counts.index(min(counts))
    high = counts.index(max(counts))
    groups = [rows[:low+1], rows[low+1:high+1], rows[high+1:]]
    samples = [[(int(r['count']), int(r['L']), int(r['C'])) for r in group]
               for group in groups]

    def loss(p, detail=False):
        sigma = p[0]
        if not 1 <= sigma <= 12:
            return 1e30
        c0, power = parameters(sigma)
        if any(f+(v-f)/c0**power > 4095 for f, v in ENDPOINTS.values()):
            return 1e30
        errors = {'GP27': [], 'GP26': []}
        for j, group in enumerate(samples):
            k, zero = p[1+2*j:3+2*j]
            if not .0003 <= k <= .006:
                return 1e30
            for count, left, centre in group:
                angle = k*(count-zero)
                dc = 119*math.sin(angle)
                dl = dc-14.5*math.cos(angle)
                for ch, distance, observed in [('GP27', dl, left),
                                                ('GP26', dc, centre)]:
                    floor, peak = ENDPOINTS[ch]
                    raw = floor+(peak-floor)*normalized(distance, sigma, c0, power)
                    errors[ch].append(raw-observed)
        if detail:
            return errors
        return sum(e*e for values in errors.values() for e in values)/(2*len(rows))

    # Fixed starts, bounded deterministic coordinate descent; all rows retained.
    fits = []
    for initial_sigma in (2., 4., 6., 8.):
        p = [initial_sigma, .0018, -40., .0018, 45., .0018, 30.]
        steps = [1., .0003, 10., .0003, 10., .0003, 10.]
        best = loss(p)
        for _ in range(240):
            improved = False
            for i, step in enumerate(steps):
                for sign in (-1, 1):
                    candidate = p.copy()
                    candidate[i] += sign*step
                    value = loss(candidate)
                    if value < best:
                        p, best, improved = candidate, value, True
            if not improved:
                steps = [s*.5 for s in steps]
                if steps[0] < 1e-5:
                    break
        fits.append((best, p))
    best, p = min(fits)
    c0, power = parameters(p[0])
    errors = loss(p, True)
    stats = {}
    for ch, values in errors.items():
        ordered = sorted(abs(e) for e in values)
        stats[ch] = {'rmse_adc': math.sqrt(sum(e*e for e in values)/len(values)),
                     'bias_adc': sum(values)/len(values),
                     'max_abs_adc': max(ordered),
                     'p95_abs_adc': ordered[math.ceil(.95*len(ordered))-1]}
    pair_max = max(normalized(x/100, p[0], c0, power)+
                   normalized(x/100-14.5, p[0], c0, power)
                   for x in range(-10000, 10001))
    return {'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'rows': len(rows), 'sweep_end_indices': [low, high],
            'sigma_mm': p[0], 'power': power,
            'line_levels_adc': {ch: f+(v-f)/c0**power
                                for ch, (f, v) in ENDPOINTS.items()},
            'pose_parameters': p[1:], 'rmse_adc': math.sqrt(best),
            'residuals': stats, 'midpoint_norm': .39,
            'pair_max_0_01mm_grid': pair_max,
            'starts_mse_adc2': [value for value, _ in fits]}


if __name__ == '__main__':
    import sys
    print(json.dumps(fit(sys.argv[1]), indent=2))
