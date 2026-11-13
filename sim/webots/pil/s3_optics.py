"""Explicit reference-height optical response through the frozen first_hit API.

The finite-ray 2D sampler is a host reference, NOT a demonstrated 1 kHz
implementation. stripe_coverage is the exact 1D Gaussian integral used for
capture fitting. Neither path changes firmware thresholds or distances.
"""
from dataclasses import dataclass, field
import math
import random
from types import MappingProxyType
from typing import Mapping, Optional, Protocol, Tuple

from .s3_geometry import MATERIALS, MAT_WHITE, MAT_LINE, PAPER_WHITE, PAPER_INK


class FirstHitScene(Protocol):
    def first_hit(self, origin_mm, direction, max_distance_mm):
        """Return structural SurfaceHit, including explicit invalid/miss."""


@dataclass(frozen=True)
class SensorProfile:
    name: str
    reference_height_mm: float
    sigma_x_mm: float
    sigma_y_mm: float
    levels_adc: Mapping[str, float]
    noise_sigma_adc: float = 0.0  # POST eight-conversion averaging
    assumptions: Tuple[str, ...] = ()
    evidence_status: str = 'assumed'
    paper_response_power: float = 1.0
    mat_response_power: float = 1.0

    def __post_init__(self):
        values = (self.reference_height_mm, self.sigma_x_mm, self.sigma_y_mm,
                  self.noise_sigma_adc, self.paper_response_power,
                  self.mat_response_power, *self.levels_adc.values())
        if not all(math.isfinite(x) for x in values):
            raise ValueError('nonfinite sensor profile')
        if min(self.reference_height_mm, self.sigma_x_mm, self.sigma_y_mm) <= 0:
            raise ValueError('height/footprint must be positive')
        if min(self.paper_response_power, self.mat_response_power) < 1:
            raise ValueError('response powers must be at least one')
        if self.noise_sigma_adc < 0 or set(self.levels_adc) != set(MATERIALS):
            raise ValueError('profile needs four material levels and valid noise')
        if any(v < 0 or v > 4095 for v in self.levels_adc.values()):
            raise ValueError('material ADC level outside 12-bit range')
        if (self.levels_adc[MAT_LINE] <= self.levels_adc[MAT_WHITE] or
                self.levels_adc[PAPER_INK] <= self.levels_adc[PAPER_WHITE]):
            raise ValueError('black-HIGH material polarity required')
        object.__setattr__(self, 'levels_adc',
                           MappingProxyType(dict(self.levels_adc)))
        object.__setattr__(self, 'assumptions', tuple(self.assumptions))


@dataclass(frozen=True)
class OpticalSample:
    adc: Optional[int]
    valid: bool
    reason: str
    mean_adc: Optional[float] = None
    standoff_mm: Optional[float] = None
    material_fractions: Mapping[str, float] = field(default_factory=dict)
    assumptions: Tuple[str, ...] = ()
    clipped: bool = False
    rays: int = 0


def _unit(v):
    if len(v) != 3 or not all(math.isfinite(x) for x in v):
        raise ValueError('invalid 3D vector')
    length = math.sqrt(sum(x*x for x in v))
    if length == 0:
        raise ValueError('zero vector')
    return tuple(x/length for x in v)


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def _cdf(value):
    return .5 * (1 + math.erf(value/math.sqrt(2)))


def stripe_coverage(position_mm, intervals_mm, sigma_mm):
    """1D convolution of disjoint printed bars, with no spatial raster error."""
    if not math.isfinite(position_mm) or not math.isfinite(sigma_mm) or sigma_mm < 0:
        raise ValueError('invalid optical coordinate/blur')
    if sigma_mm == 0:
        return float(any(a <= position_mm <= b for a, b in intervals_mm))
    value = sum(_cdf((b-position_mm)/sigma_mm) -
                _cdf((a-position_mm)/sigma_mm) for a, b in intervals_mm)
    return max(0.0, min(1.0, value))


def _quadrature(count):
    """Midpoint rays; each cell has its exact Gaussian probability mass.

    Truncate at +/-4 sigma and renormalize. Resolution is explicit because
    small counts alias material edges; it is not an extra measured noise term.
    """
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ValueError('quadrature must be a positive integer')
    if count > 257:
        raise ValueError('reference sampler resolution is bounded at 257')
    step = 8/count
    norm = _cdf(4)-_cdf(-4)
    return tuple((-4+(i+.5)*step,
                  (_cdf(-4+(i+1)*step)-_cdf(-4+i*step))/norm)
                 for i in range(count))


def mean_adc(profile, fractions):
    """Phenomenological ADC response of normalized physical material coverage."""
    if (any(k not in profile.levels_adc or not math.isfinite(v) or v < 0
            for k, v in fractions.items()) or
            abs(sum(fractions.values())-1) > 1e-7):
        raise ValueError('material fractions must be known, nonnegative and sum to one')
    paper = fractions.get(PAPER_WHITE, 0) + fractions.get(PAPER_INK, 0)
    mat = fractions.get(MAT_WHITE, 0) + fractions.get(MAT_LINE, 0)
    return (paper*profile.levels_adc[PAPER_WHITE] +
            mat*profile.levels_adc[MAT_WHITE] +
            (profile.levels_adc[PAPER_INK]-profile.levels_adc[PAPER_WHITE]) *
            fractions.get(PAPER_INK, 0)**profile.paper_response_power +
            (profile.levels_adc[MAT_LINE]-profile.levels_adc[MAT_WHITE]) *
            fractions.get(MAT_LINE, 0)**profile.mat_response_power)


def sample_sensor(scene: FirstHitScene, profile: SensorProfile, origin_mm,
                  direction=(0, 0, -1), axis_u=(1, 0, 0), *, sample_us=0,
                  seed=0, max_distance_mm=100.0, quadrature=17,
                  bias_adc=0.0, allow_extrapolation=False):
    """Consume nearest surfaces only; unknown/missing hits never become white.

    Valid means a model sample was evaluated, NOT hardware calibration PASS.
    Missing measurements remain in assumptions/evidence_status. Outside the
    reference height or normal incidence, explicit opt-in is required.
    """
    d, u = _unit(direction), _unit(axis_u)
    if abs(sum(a*b for a, b in zip(d, u))) > 1e-7:
        raise ValueError('footprint axis must be perpendicular to optical axis')
    v = _cross(u, d)
    if len(origin_mm) != 3 or not all(math.isfinite(x) for x in origin_mm):
        raise ValueError('invalid origin')
    if not math.isfinite(bias_adc) or not math.isfinite(max_distance_mm) or max_distance_mm <= 0:
        raise ValueError('invalid bias/ray range')
    if not isinstance(sample_us, int) or sample_us < 0:
        raise ValueError('sample_us must be a nonnegative integer')
    nodes = _quadrature(quadrature)
    central = scene.first_hit(tuple(origin_mm), d, max_distance_mm)
    if not central.valid:
        return OpticalSample(None, False, central.reason)
    incidence = -sum(a*b for a, b in zip(d, central.normal))
    outside = (abs(central.distance_mm-profile.reference_height_mm) > .001 or
               incidence < 1 - 1e-6)
    assumptions = profile.assumptions
    if outside:
        if not allow_extrapolation:
            return OpticalSample(None, False, 'uncalibrated_height_or_tilt',
                                 standoff_mm=central.distance_mm)
        assumptions += ('uncalibrated_height_or_tilt',)
    fractions = {}
    rays = 1
    for x, wx in nodes:
        for y, wy in nodes:
            ray = tuple(d[i] + u[i]*x*profile.sigma_x_mm/profile.reference_height_mm +
                        v[i]*y*profile.sigma_y_mm/profile.reference_height_mm
                        for i in range(3))
            hit = scene.first_hit(tuple(origin_mm), ray, max_distance_mm)
            rays += 1
            if not hit.valid:
                return OpticalSample(None, False, hit.reason, rays=rays)
            if hit.material_id not in profile.levels_adc:
                return OpticalSample(None, False, 'unknown_material', rays=rays)
            axial = sum((p-a)*b for p, a, b in zip(hit.point_mm, origin_mm, d))
            tilted = -sum(a*b for a, b in zip(d, hit.normal)) < 1 - 1e-6
            if abs(axial-profile.reference_height_mm) > .001 or tilted:
                if not allow_extrapolation:
                    return OpticalSample(None, False,
                                         'uncalibrated_height_or_tilt', rays=rays)
                if 'uncalibrated_height_or_tilt' not in assumptions:
                    assumptions += ('uncalibrated_height_or_tilt',)
            weight = wx*wy
            fractions[hit.material_id] = fractions.get(hit.material_id, 0) + weight
    # A phenomenological material response, not visible-RGB averaging.
    # Pure-material anchors remain exact. Paper nonlinearity is fit using
    # fixed print dimensions; mat power and mixed-material response are assumed.
    mean = mean_adc(profile, fractions)
    # A per-time/per-channel seed avoids dependence on query ordering or misses.
    noise = random.Random(f'{seed}:{profile.name}:{sample_us}').gauss(
        0, profile.noise_sigma_adc)
    raw = round(mean + bias_adc + noise)
    clipped = raw < 0 or raw > 4095
    return OpticalSample(max(0, min(4095, raw)), True, '', mean,
                         central.distance_mm, MappingProxyType(fractions),
                         assumptions, clipped, rays)
