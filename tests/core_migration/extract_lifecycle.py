#!/usr/bin/env python3
"""Extract the B2 task lifecycle, unchanged, for isolated kernel-state tests."""
from pathlib import Path
import re
import sys
from extract_lock_sections import clean, function

root = Path(sys.argv[1])
source = clean((root / 'app_program/platform/app_lock_stress.c').read_text())
constants = re.findall(r'^#define IL_\w+[^\n]*', source, re.M)
variables = re.findall(
    r'^static (?:volatile )?(?:UW|T_SPLOCK) '
    r'(?:stress_lock|stress_errors|il_\w+);', source, re.M)
assert len(constants) == 4
assert len(variables) == 6
pieces = ['/* Generated from production source. Do not edit. */',
          *constants, *variables]
for name in ('il_holder', 'interleave_wait_dormant', 'interleave_delete',
             'interleave_run'):
    pieces.append(function(source, name))
Path(sys.argv[2]).write_text('\n\n'.join(pieces) + '\n')
