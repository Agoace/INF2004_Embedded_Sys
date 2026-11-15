"""Use the production dispatch condition in the register/kernel test double."""
from pathlib import Path
import sys

source, destination = map(Path, sys.argv[1:])
lines = source.read_text().splitlines()
output = ['/* Generated from production cpu_status.h; do not edit. */']
for name in ('BEGIN_CRITICAL_SECTION', 'END_CRITICAL_SECTION'):
    starts = [i for i, line in enumerate(lines)
              if line.split(maxsplit=2)[:2] == ['#define', name]]
    if len(starts) != 1:
        raise SystemExit(f'expected one production macro: {name}')
    index = starts[0]
    while True:
        line = lines[index]
        output.append(line)
        index += 1
        if not line.rstrip().endswith('\\'):
            break
destination.write_text('\n'.join(output) + '\n')
