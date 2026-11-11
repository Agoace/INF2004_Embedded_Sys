"""Exercise real option parsing and profile invalidation without firmware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
(ROOT / "build/wip").mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix="junction-make-", dir=ROOT / "build/wip") as tmp:
    tree = Path(tmp)
    names = subprocess.check_output(
        ["git", "ls-files", "build_make"], cwd=ROOT, text=True).splitlines()
    for name in names:
        if name.endswith(".mk") or name.endswith("/makefile"):
            target = tree / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, target)
    build = tree / "build_make"
    (build / "probe.mk").write_text(
        'include pico_rp2040.mk\n.PHONY: clean\nclean:\n'
        '\t@echo "image=$(EXE_FILE)"\n'
        '\t@echo "profile=$(PROFILE_ID)"\n'
        '\t@echo "flags=$(CFLAGS)"\n')

    def probe(value=None, **options):
        args = ["make", "-s", "-f", "probe.mk", "clean", "CONSOLE=uart", "WIFI=none"]
        if value is not None:
            args.append(f"JUNCTION_TEST_CMD={value}")
        args.extend(f"{key}={value}" for key, value in options.items())
        return subprocess.run(args, cwd=build, text=True, capture_output=True)

    default = probe()
    assert default.returncode == 0, default.stderr
    assert "image=mtk3pico_smp1_uart\n" in default.stdout
    assert "-DJUNCTION_TEST_CMD=0" in default.stdout
    for prefix in ("image=", "profile=", "flags="):
        a = next(x for x in default.stdout.splitlines() if x.startswith(prefix))
        b = next(x for x in probe("NONE").stdout.splitlines() if x.startswith(prefix))
        assert a == b
    image = build / "mtk3pico_smp1_uart.uf2"
    image.write_bytes(b"NONE image")
    dummy = build / "mtkernel_3/old.o"
    for number, name in enumerate("ABCD", 1):
        dummy.write_bytes(b"old object")
        result = probe(name)
        assert result.returncode == 0, result.stderr
        assert f"image=mtk3pico_smp1_uart_jtest{name}\n" in result.stdout
        assert f"-junction{number}" in result.stdout
        assert f"-DJUNCTION_TEST_CMD={number}" in result.stdout
        assert not dummy.exists() and image.exists()
    dummy.write_bytes(b"D object")
    assert probe("NONE").returncode == 0 and not dummy.exists()
    for bad in ("", "0", "1", "a", "E", "A B", "NONE A", "JA_LEFT"):
        result = probe(bad)
        assert result.returncode != 0 and "Unknown JUNCTION_TEST_CMD" in result.stderr, bad
    assert "-DBARCODE_LIVE_ENABLE=0" in default.stdout
    assert "-DBARCODE_MEASURED_CLEAR_MM=0" in default.stdout
    for bad in ("", "2", "on", "0 1"):
        assert probe(BARCODE_LIVE_ENABLE=bad).returncode != 0
    for bad in ("", "-1", "1.5", "yes", "65536", "08", "00"):
        assert probe(BARCODE_MEASURED_CLEAR_MM=bad).returncode != 0
    assert probe(BARCODE_LIVE_ENABLE=1).returncode != 0
    assert probe(BARCODE_LIVE_ENABLE=1, JUNCTION_SHADOW_CLEAR_MM=150).returncode != 0
    assert probe("A", BARCODE_LIVE_ENABLE=1, BARCODE_MEASURED_CLEAR_MM=1).returncode != 0
    permitted = probe(BARCODE_LIVE_ENABLE=1, BARCODE_MEASURED_CLEAR_MM=1)
    assert permitted.returncode == 0, permitted.stderr
    assert "_bclive" in permitted.stdout and "-bclive1-bcclear1" in permitted.stdout
    dummy.write_bytes(b"permitted object")
    shadow = probe(BARCODE_MEASURED_CLEAR_MM=150)
    assert shadow.returncode == 0 and not dummy.exists()
    assert "-DBARCODE_LIVE_ENABLE=0" in shadow.stdout
    assert "-bclive0-bcclear150" in shadow.stdout
    # Direct C definitions cannot bypass the range check.
    for value in ("0", "1", "2", "3", "4", "-1", "5", "A", ""):
        result = subprocess.run(
            ["cc", "-std=c11", "-Werror", "-x", "c", "-fsyntax-only", "-",
             f"-I{ROOT / 'app_program/vehicle'}", f"-DJUNCTION_TEST_CMD={value}"],
            input='#include "junction_source.h"\n', text=True, capture_output=True)
        assert (result.returncode == 0) == (value in ("0", "1", "2", "3", "4")), value
    for flags, valid in (
        (["BARCODE_LIVE_ENABLE=1"], False),
        (["BARCODE_LIVE_ENABLE=2"], False),
        (["BARCODE_LIVE_ENABLE=ON"], False),
        (["BARCODE_MEASURED_CLEAR_MM=x"], False),
        (["BARCODE_MEASURED_CLEAR_MM=65536"], False),
        (["BARCODE_LIVE_ENABLE=1", "BARCODE_MEASURED_CLEAR_MM=1"], True),
        (["BARCODE_LIVE_ENABLE=1", "BARCODE_MEASURED_CLEAR_MM=1",
          "JUNCTION_TEST_CMD=1"], False),
        (["BARCODE_MEASURED_CLEAR_MM=150"], True),
    ):
        result = subprocess.run(
            ["cc", "-std=c11", "-Werror", "-x", "c", "-fsyntax-only", "-",
             f"-I{ROOT / 'app_program/vehicle'}", *[f"-D{x}" for x in flags]],
            input='#include "junction_source.h"\n', text=True, capture_output=True)
        assert (result.returncode == 0) == valid, flags
print("Make fake/shadow defaults, live permission gates and object guard: PASS")
