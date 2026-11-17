#!/usr/bin/env python3
"""LINE_KD make guards: bad values stop make; 85 adds nothing; n adds _kdn."""
import pathlib
import subprocess
import sys

MK = pathlib.Path(__file__).resolve().parents[1] / "build_make"


def make(*args):
    return subprocess.run(
        ["make", "-n", "-p", "SMP=0", "WIFI=none", *args],
        cwd=MK, capture_output=True, text=True, check=False)


def var(out, name):
    for line in out.splitlines():
        if line.startswith(name + " := "):
            return line.split(" := ", 1)[1]
    return None


fail = 0
for bad in ("86", "-1", "x", "08", "3 4", ""):
    r = make("LINE_KD=" + bad)
    if r.returncode == 0 or "LINE_KD" not in r.stderr:
        print("FAIL: LINE_KD=%r accepted" % bad)
        fail += 1
r = make("LINE_KD=30", "BENCH_KERNEL=1")
if r.returncode == 0 or "LINE_KD" not in r.stderr:
    print("FAIL: LINE_KD=30 with BENCH_KERNEL=1 accepted")
    fail += 1
base = make()
d85 = make("LINE_KD=85")
d30 = make("LINE_KD=30")
d0 = make("LINE_KD=0")
for r in (base, d85, d30, d0):
    if r.returncode != 0:
        print("FAIL: make -n:", r.stderr[-300:])
        sys.exit(1)
exe = var(base.stdout, "EXE_FILE")
if var(d85.stdout, "EXE_FILE") != exe or "line_kd.h" in d85.stdout:
    print("FAIL: LINE_KD=85 changes the build")
    fail += 1
if var(d30.stdout, "EXE_FILE") != exe + "_kd30":
    print("FAIL: suffix", var(d30.stdout, "EXE_FILE"))
    fail += 1
if var(d0.stdout, "EXE_FILE") != exe + "_kd0":
    print("FAIL: suffix", var(d0.stdout, "EXE_FILE"))
    fail += 1
if "-DLINE_KD=30 -include ../app_program/vehicle/line_kd.h" not in d30.stdout:
    print("FAIL: vehicle_task.o rule missing")
    fail += 1
if var(d30.stdout, "PROFILE_ID") == var(base.stdout, "PROFILE_ID"):
    print("FAIL: PROFILE_ID does not change, stale objects can be reused")
    fail += 1
print("test_line_kd_make:", "ok" if fail == 0 else "%d failure(s)" % fail)
sys.exit(1 if fail else 0)
