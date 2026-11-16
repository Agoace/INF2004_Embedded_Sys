#!/usr/bin/env python3
"""Convert a captured IMU USB log into the replay analyser's CSV format.

The TEST/IMU_STREAM observer prints one line per observation:

    [S4] t_ms <ms> source_us <us> valid <u> motion <u>
         raw_mg <x> <y> <z> cal_mg <x> <y> <z> ... cal <u> faults <n> ...

``cal_mg`` is ``imu_core_output_t.accel_mg``: already mapped to
X right / Y forward / Z up and with the bias removed. That is exactly what
the detector consumes, so this converter only extracts columns; it never
re-derives a mapping or a bias, and it cannot silently analyse a different
signal.

Output columns, in detector order:

    t_ms,ax_mg,ay_mg,az_mg,valid,calibrated,wheels_still

LIMITS, state them when reporting a result:

- ``wheels_still`` is not on the [S4] line, so it is emitted as 0. That
  field only feeds the all-clear window, never a trigger decision, so a
  zero here cannot invent or hide a lift or overturn.
- The observer samples at 10 Hz (50 Hz with IMU_STREAM) while the detector
  runs at 100 Hz. A replay therefore shows whether the bounds WOULD fire on
  the recorded motion; it does not reproduce the detector's exact timing.
- A capture only contains a transient if the observer caught it.

Exit codes: 0 success, 1 no observations found, 2 usage or I/O error.
"""

import argparse
import pathlib
import sys

COLUMNS = "t_ms,ax_mg,ay_mg,az_mg,valid,calibrated,wheels_still"
MARKER = "[S4]"


def _int_after(tokens, key):
    """Return the integer token that follows ``key``, or None."""
    try:
        index = tokens.index(key)
    except ValueError:
        return None
    if index + 1 >= len(tokens):
        return None
    try:
        return int(tokens[index + 1])
    except ValueError:
        return None


def parse_line(line):
    """Parse one [S4] line into a row, or return None if it is not one."""
    if MARKER not in line:
        return None
    tokens = line.split()
    if "cal_mg" not in tokens:
        return None
    t_ms = _int_after(tokens, "t_ms")
    if t_ms is None:
        return None
    try:
        base = tokens.index("cal_mg") + 1
        ax, ay, az = (int(tokens[base]), int(tokens[base + 1]),
                      int(tokens[base + 2]))
    except (ValueError, IndexError):
        return None
    valid = _int_after(tokens, "valid")
    calibrated = _int_after(tokens, "cal")
    return (t_ms, ax, ay, az, 1 if valid else 0, 1 if calibrated else 0, 0)


def convert(text):
    """Return (rows, skipped_lines)."""
    rows = []
    skipped = 0
    for line in text.splitlines():
        if MARKER not in line:
            continue
        row = parse_line(line)
        if row is None:
            skipped += 1
            continue
        rows.append(row)
    return rows, skipped


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert an IMU USB log to the safety replay CSV.")
    parser.add_argument("log", help="captured USB log (text)")
    parser.add_argument("out", help="CSV path to write")
    parser.add_argument("--min-rows", type=int, default=2,
                        help="fail if fewer rows are found (default 2)")
    args = parser.parse_args(argv)

    try:
        text = pathlib.Path(args.log).read_text(errors="replace")
    except OSError as error:
        print(f"cannot read {args.log}: {error}", file=sys.stderr)
        return 2

    rows, skipped = convert(text)
    if len(rows) < args.min_rows:
        print(f"only {len(rows)} observations found in {args.log}; "
              f"need {args.min_rows}", file=sys.stderr)
        return 1

    last = None
    backwards = 0
    for row in rows:
        if last is not None and row[0] < last:
            backwards += 1
        last = row[0]

    lines = [f"# Converted from {pathlib.Path(args.log).name} by "
             f"imu_trace_to_csv.py",
             "# wheels_still is not on the [S4] line and is emitted as 0.",
             f"# observations {len(rows)}; skipped {skipped}; "
             f"non-monotonic {backwards}",
             COLUMNS]
    lines.extend(",".join(str(value) for value in row) for row in rows)

    try:
        pathlib.Path(args.out).write_text("\n".join(lines) + "\n")
    except OSError as error:
        print(f"cannot write {args.out}: {error}", file=sys.stderr)
        return 2

    print(f"wrote {args.out}: rows={len(rows)} skipped={skipped} "
          f"non_monotonic={backwards}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
