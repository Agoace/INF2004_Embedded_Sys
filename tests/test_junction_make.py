"""Exercise real option parsing and profile invalidation without firmware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="junction-make-") as tmp:
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

    def probe(value=None):
        args = ["make", "-s", "-f", "probe.mk", "clean", "CONSOLE=uart", "WIFI=none"]
        if value is not None:
            args.append(f"JUNCTION_TEST_CMD={value}")
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
    # Direct C definitions cannot bypass the range check.
    for value in ("0", "1", "2", "3", "4", "-1", "5", "A", ""):
        result = subprocess.run(
            ["cc", "-std=c11", "-Werror", "-x", "c", "-fsyntax-only", "-",
             f"-I{ROOT / 'app_program/vehicle'}", f"-DJUNCTION_TEST_CMD={value}"],
            input='#include "junction_source.h"\n', text=True, capture_output=True)
        assert (result.returncode == 0) == (value in ("0", "1", "2", "3", "4")), value
print("Make junction defaults, four suffixes, validation and object guard: PASS")
