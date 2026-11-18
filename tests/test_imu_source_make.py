"""Check real Make option parsing in an isolated tree; no firmware build."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

with tempfile.TemporaryDirectory(prefix="imu-source-make-") as tmp:
    tree = Path(tmp)
    tracked = subprocess.check_output(
        ["git", "ls-files", "build_make"], cwd=ROOT, text=True).splitlines()
    for name in tracked:
        if not (name.endswith(".mk") or name.endswith("/makefile")):
            continue
        target = tree / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    build = tree / "build_make"
    (build / "probe.mk").write_text(
        'include pico_rp2040.mk\n'
        '.PHONY: clean\n'
        'clean:\n'
        '\t@echo "image=$(EXE_FILE)"\n'
        '\t@echo "profile=$(PROFILE_ID)"\n'
        '\t@echo "flags=$(CFLAGS)"\n')

    def probe(value=None):
        args = ["make", "-s", "-f", "probe.mk", "clean", "CONSOLE=uart", "WIFI=none"]
        if value is not None:
            args.append(f"IMU_HUMP_SOURCE={value}")
        return subprocess.run(args, cwd=build, text=True, capture_output=True)

    default = probe()
    assert default.returncode == 0, default.stderr
    assert "-DIMU_HUMP_SOURCE=1" in default.stdout
    assert "-hump_source1" in default.stdout
    assert "image=mtk3pico_smp1_uart\n" in default.stdout
    explicit = probe("1")
    assert explicit.returncode == 0, explicit.stderr
    for prefix in ("image=", "profile=", "flags="):
        a = next(line for line in default.stdout.splitlines() if line.startswith(prefix))
        b = next(line for line in explicit.stdout.splitlines() if line.startswith(prefix))
        assert a == b
    # Mark an old object and an existing default image before switching.
    dummy = build / "mtkernel_3/old.o"
    dummy.write_bytes(b"old object")
    image = build / "mtk3pico_smp1_uart.uf2"
    image.write_bytes(b"default image")
    filtered = probe("2")
    assert filtered.returncode == 0, filtered.stderr
    assert "-DIMU_HUMP_SOURCE=2" in filtered.stdout
    assert "-hump_source2" in filtered.stdout
    assert "image=mtk3pico_smp1_uart_humpfilt\n" in filtered.stdout
    assert not dummy.exists() and image.exists()
    dummy.write_bytes(b"filtered object")
    assert probe("1").returncode == 0 and not dummy.exists()
    for bad in ("", "0", "3", "1 2", "filtered", "IMU_HUMP_SOURCE_FILTERED"):
        result = probe(bad)
        assert result.returncode != 0 and "Unknown IMU_HUMP_SOURCE" in result.stderr, bad

print("Make hump source defaults, validation, suffix and object guard: PASS")
