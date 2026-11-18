#!/usr/bin/env python3
"""Check the documented template-path deviation list; never read secrets."""
from pathlib import Path
import re
import subprocess
import sys
ROOT = Path(__file__).resolve().parents[1]
reference = sys.argv[1] if len(sys.argv) > 1 else 'c3b0954'
prefixes = ('kernel/', 'lib/', 'device/', 'include/', 'config/', 'etc/')
text = (ROOT / 'docs/architecture/TEMPLATE_CHANGES.md').read_text()
allowed = set(re.findall(r'^\| \[((?:kernel|lib|device|include|config|etc)/[^\]]+)\]', text, re.M))
actual = set(subprocess.check_output(
    ['git', 'diff', '--name-only', reference, '--', *prefixes], cwd=ROOT, text=True).splitlines())
if actual != allowed:
    for name in sorted(actual - allowed):
        print('ERROR unrecorded template path:', name)
    for name in sorted(allowed - actual):
        print('ERROR stale template record:', name)
    raise SystemExit(1)
print(f'PASS: {len(actual)} recorded template deviations against {reference}; content review still required')
