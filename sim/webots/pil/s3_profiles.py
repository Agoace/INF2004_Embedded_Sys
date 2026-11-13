"""Load explicit S3 profiles or freeze both recorded replay variants.

UNMEASURED channels fail explicitly. There is no GP28-to-GP26/27 fallback.
"""
import argparse
import json
from pathlib import Path
from .s3_optics import SensorProfile

DEFAULT_PATH = Path(__file__).resolve().parents[3]/'calibration/s3_sensor_profiles.json'


def load_profile(channel='GP28', path=DEFAULT_PATH):
    bundle = json.loads(Path(path).read_text())
    if bundle.get('schema_version') != 1:
        raise ValueError('unsupported S3 profile schema')
    entry = bundle.get('channels', {}).get(channel)
    if entry is None:
        raise ValueError(f'unknown S3 channel: {channel}')
    if entry.get('profile') is None:
        raise ValueError(f'{channel} is UNMEASURED; explicit synthetic assumptions required')
    try:
        return SensorProfile(**entry['profile'])
    except TypeError as error:
        raise ValueError('invalid S3 profile fields') from error


def main():
    # Imported lazily so a runtime profile load never runs the fitter.
    from .s3_replay import read_capture, observations, profile_bundle
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-report', type=Path, required=True)
    parser.add_argument('--mean-report', type=Path, required=True)
    parser.add_argument('--training-capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sensor = json.loads(args.sensor_report.read_text())
    mean = json.loads(args.mean_report.read_text())
    capture = read_capture(args.training_capture)
    if capture.sha256 != sensor['training']['sha256']:
        parser.error('training capture hash differs from recorded fit')
    rows, meta = observations(capture, 'sensor_x')
    if meta['flagged_adc'] or meta['saturated_adc']:
        parser.error('cannot freeze flagged/saturated ADC as loss-free evidence')
    bundle = profile_bundle(sensor, mean, rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle, indent=2)+'\n')


if __name__ == '__main__':
    main()
