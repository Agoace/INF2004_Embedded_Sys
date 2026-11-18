"""Exercise the opt-in ROBOT capture flags without compiling firmware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
(ROOT / "build/wip").mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix="failcap-make-", dir=ROOT / "build/wip") as tmp:
    tree = Path(tmp)
    for name in subprocess.check_output(
        ["git", "ls-files", "build_make"], cwd=ROOT, text=True
    ).splitlines():
        if name.endswith(".mk") or name.endswith("/makefile"):
            target = tree / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, target)
    build = tree / "build_make"
    (build / "probe.mk").write_text(
        'include pico_rp2040.mk\n.PHONY: clean\nclean:\n'
        '\t@echo "image=$(EXE_FILE)"\n'
        '\t@echo "profile=$(PROFILE_ID)"\n'
        '\t@echo "flags=$(CFLAGS)"\n'
    )

    def probe(**options):
        return subprocess.run(
            ["make", "-s", "-f", "probe.mk", "clean", "CONSOLE=uart", "WIFI=none",
             *[f"{k}={v}" for k, v in options.items()]],
            cwd=build, text=True, capture_output=True,
        )

    default = probe()
    assert default.returncode == 0, default.stderr
    assert "BARCODE_FAIL_CAPTURE" not in default.stdout
    assert "bcfail" not in default.stdout
    explicit = probe(BARCODE_FAIL_CAPTURE=0)
    for label in ("image=", "profile=", "flags="):
        assert next(x for x in default.stdout.splitlines() if x.startswith(label)) == next(
            x for x in explicit.stdout.splitlines() if x.startswith(label))
    dummy = build / "mtkernel_3/old.o"
    for slots in (2, 1):
        dummy.parent.mkdir(parents=True, exist_ok=True)
        dummy.write_bytes(b"old")
        cap = probe(BARCODE_FAIL_CAPTURE=1, BARCODE_FAIL_SLOTS=slots)
        assert cap.returncode == 0, cap.stderr
        assert f"_bcfail{slots}" in cap.stdout and f"-bcfail{slots}" in cap.stdout
        assert "-DBARCODE_FAIL_CAPTURE=1" in cap.stdout
        assert f"-DBCFC_FAILED_SLOTS={slots}" in cap.stdout
        assert not dummy.exists()
    dummy.write_bytes(b"on")
    assert probe().returncode == 0 and not dummy.exists()
    for bad in ("", "on", "2", "0 1"):
        assert probe(BARCODE_FAIL_CAPTURE=bad).returncode != 0
    for bad in ("", "0", "3", "1 2", "01"):
        assert probe(BARCODE_FAIL_CAPTURE=1, BARCODE_FAIL_SLOTS=bad).returncode != 0
    for mode in ({"TEST": 1}, {"BENCH": 1}, {"BENCH_KERNEL": 1}):
        assert probe(BARCODE_FAIL_CAPTURE=1, **mode).returncode != 0
print("Failed capture default-off, ROBOT guard, bounds and stale-object invalidation: PASS")
