"""Generate size-only box cases; optionally run the unchanged host target.

Width is world y. Depth is world x. The front face stays at x=740 mm.
Run from any directory: python3 sim/box_sweep.py [--run]
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess

SIM = Path(__file__).resolve().parent
CASES = [(120, 120), (120, 240), (120, 480), (240, 120),
         (240, 240), (240, 480), (360, 480)]


def name(width, depth):
    if (width, depth) == (120, 120):
        return "box_straight"
    if (width, depth) == (240, 480):
        return "box_wide_deep"
    return f"box_w{width}_d{depth}"


def generate():
    baseline = json.loads((SIM / "scenarios/box_straight.json").read_text())
    paths = []
    for width, depth in CASES:
        scenario = copy.deepcopy(baseline)
        scenario["name"] = name(width, depth)
        box = scenario["obstacles"][0]
        box["pose"]["value"]["x_mm"] = 740 + depth / 2
        box["size_mm"]["value"] = [depth, width]
        path = SIM / "scenarios" / (scenario["name"] + ".json")
        # Preserve the baseline file byte-for-byte, including its formatting.
        if (width, depth) != (120, 120):
            path.write_text(json.dumps(scenario, indent=2) + "\n")
        assert box["pose"]["value"]["x_mm"] - depth / 2 == 740
        paths.append(path)
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="Run all seven scenarios after generation")
    parser.add_argument("--launcher", type=Path, default=SIM / "build/robot_sim")
    parser.add_argument("--output", type=Path, default=SIM / "output/bigbox")
    args = parser.parse_args()
    for path in generate():
        print(path)
        if args.run:
            output = args.output / path.stem
            output.mkdir(parents=True, exist_ok=True)
            with (output / "console.txt").open("w") as log:
                subprocess.run([str(args.launcher.resolve()), "--scenario", str(path),
                                "--output", str(output)], check=True, stdout=log)


if __name__ == "__main__":
    main()
