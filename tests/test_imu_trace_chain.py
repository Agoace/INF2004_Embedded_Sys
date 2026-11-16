#!/usr/bin/env python3
"""End-to-end check: USB log -> converter -> replay analyser -> detector.

This proves the analysis path a captured trace will use, so that when a real
lift capture exists the loop is known to work. The fixture is SYNTHETIC; a
pass here is not lift-detection evidence.

Usage: test_imu_trace_chain.py <path-to-test_replay_imu_safety>
"""

import pathlib
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent
CONVERTER = REPO / "scripts" / "imu_trace_to_csv.py"
LOG = REPO / "tests" / "fixtures" / "imu_trace_log_synthetic.txt"


def main(argv):
    if len(argv) != 2:
        print("usage: test_imu_trace_chain.py <replay-binary>", file=sys.stderr)
        return 2
    replay = pathlib.Path(argv[1])
    if not replay.exists():
        print(f"replay binary not found: {replay}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory() as tmp:
        csv_path = pathlib.Path(tmp) / "trace.csv"

        converted = subprocess.run(
            [sys.executable, str(CONVERTER), str(LOG), str(csv_path)],
            capture_output=True, text=True)
        if converted.returncode != 0:
            print(f"converter failed: {converted.stderr}", file=sys.stderr)
            return 1
        if not csv_path.exists():
            print("converter wrote no CSV", file=sys.stderr)
            return 1

        text = csv_path.read_text()
        if "t_ms,ax_mg,ay_mg,az_mg,valid,calibrated,wheels_still" not in text:
            print("CSV header missing", file=sys.stderr)
            return 1

        replayed = subprocess.run(
            [str(replay), str(csv_path)], capture_output=True, text=True)
        if replayed.returncode != 0:
            print(f"replay failed: {replayed.stderr}", file=sys.stderr)
            return 1

        # The synthetic log has two low-g observations after two quiet ones,
        # so LIFT must confirm on the second low-g sample at t_ms 30. Pinning
        # the exact onset means a rule or column change cannot pass quietly.
        expected = ("first_reason=lift first_ms=30 latched=lift stops=1")
        if expected not in replayed.stdout:
            print("expected outcome not reported\n"
                  f"  want: {expected}\n  got : {replayed.stdout.strip()}",
                  file=sys.stderr)
            return 1

        print("chain ok: log -> csv -> detector reported lift at t_ms 30, "
              "one latch")
        return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
