"""Opt-in, labelled SIL line assumptions. Never a measured-profile fallback.

Footprint fitted to boot-5209 spin recording; transverse isotropy assumed.
Only MAT_WHITE/MAT_LINE on flat straight strips are supported. Paper levels
exist solely to satisfy SensorProfile's four-material schema, not as evidence.
Mount x=forward,y=left; z is flat clearance, NOT axle-relative height.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .s3_optics import SensorProfile

ASSUMED_LINE_PATH = (Path(__file__).resolve().parents[3]/
                     'calibration/s3_assumed_line_profiles.json')
LABEL = 'ASSUMED_SIL_STRAIGHT_ONLY'


@dataclass(frozen=True)
class AssumedLineMount:
    forward_mm: float
    left_mm: float
    flat_clearance_mm: float
    forward_assumed: bool
    height_assumed: bool
    spacing_evidence: str
    forward_evidence: str
    height_evidence: str


def _read(channel, path):
    data = json.loads(Path(path).read_text())
    if (data.get('schema_version') != 1 or data.get('classification') != LABEL or
            data.get('supported_materials') != ['MAT_WHITE', 'MAT_LINE']):
        raise ValueError('not a labelled ASSUMED straight-line profile record')
    if channel not in ('GP26', 'GP27') or channel not in data.get('channels', {}):
        raise ValueError('ASSUMED line profile requires GP26 or GP27')
    params = data.get('profile_parameters', {})
    if (params.get('evidence_status') != LABEL or
            'NOT_MEASURED_CALIBRATION' not in params.get('assumptions', ()) or
            'FITTED_TO_BOOT_5209_SPIN_RECORDING' not in params.get('assumptions', ()) or
            'PAPER_LEVELS_SCHEMA_PLACEHOLDERS_NOT_SUPPORTED' not in params.get('assumptions', ())):
        raise ValueError('missing ASSUMED profile labels')
    return data, data['channels'][channel]


def load_assumed_line_profile(channel, path=ASSUMED_LINE_PATH):
    """Explicit opt-in only. Does not alter load_profile or the frozen JSON."""
    data, _ = _read(channel, path)
    try:
        profile = SensorProfile(name=channel, **data['profile_parameters'])
    except (TypeError, KeyError) as error:
        raise ValueError('invalid ASSUMED profile fields') from error
    if profile.noise_sigma_adc != 0:
        raise ValueError('ASSUMED straight-line profile must be noise-free')
    return profile


def load_assumed_line_mount(channel, path=ASSUMED_LINE_PATH):
    """Axle body XY plus flat clearance; subtract wheel radius once for Z."""
    _, entry = _read(channel, path)
    try:
        mount = AssumedLineMount(**entry['mount'])
    except (TypeError, KeyError) as error:
        raise ValueError('invalid ASSUMED mount fields') from error
    if (not all(type(v) in (float, int) and math.isfinite(v) for v in
                (mount.forward_mm, mount.left_mm, mount.flat_clearance_mm)) or
            mount.flat_clearance_mm <= 0 or mount.forward_assumed is not False or
            mount.forward_evidence != 'MEASURED_OPERATOR_2026_11_15' or
            mount.height_assumed is not False or
            mount.height_evidence != 'MEASURED_OPERATOR_APPROX_2026_11_15' or
            not mount.spacing_evidence):
        raise ValueError('invalid or unlabelled ASSUMED mount')
    return mount
