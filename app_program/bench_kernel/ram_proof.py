#!/usr/bin/env python3
"""Prove the BENCH_KERNEL_RAM=1 placement in a linked ELF.

Usage: ram_proof.py ELF LIST [PREFIX]
Checks that every listed symbol is in RAM, then lists the flash functions
that RAM code still calls (through long-branch veneers) and the indirect
calls (blx rN) that cannot be resolved statically. Exit 1 if a listed
symbol is missing or still in flash.
"""
import re
import subprocess
import sys

RAM = (0x20000000, 0x20042000)
FLASH = (0x10000000, 0x11000000)
CALL = re.compile(r"^\s*([0-9a-f]+):\s+(bl|b|b\.n|b\.w)\s+[0-9a-f]+ <([^>+]+)")
BLX = re.compile(r"^\s*([0-9a-f]+):\s+blx\s+r\d+")
HEAD = re.compile(r"^([0-9a-f]+) <([^>]+)>:")


def in_range(addr, rng):
    return rng[0] <= addr < rng[1]


def parse_nm(text):
    """name -> (address, size); a size-less label ends at the next symbol."""
    rows = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4:
            rows.append((int(parts[0], 16), int(parts[1], 16), parts[3]))
        elif len(parts) == 3:
            rows.append((int(parts[0], 16), None, parts[2]))
    rows.sort(key=lambda r: (r[0], r[2]))
    out = {}
    for i, (addr, size, name) in enumerate(rows):
        if size is None:
            nxt = next((a for a, _, _ in rows[i + 1:] if a > addr), addr)
            size = nxt - addr
        out.setdefault(name, (addr, size))
    return out


def placement(syms, names):
    ram, flash, missing = [], [], []
    for name in names:
        if name not in syms:
            missing.append(name)
        elif in_range(syms[name][0], RAM):
            ram.append(name)
        else:
            flash.append(name)
    return ram, flash, missing


def crossings(disasm, syms, names):
    """Flash callees of the named RAM functions, and blx counts."""
    spans = {n: (syms[n][0], syms[n][0] + syms[n][1]) for n in names
             if n in syms}
    calls, indirect = {}, {}

    def owner(addr):
        return next((n for n, (lo, hi) in spans.items() if lo <= addr < hi),
                    None)

    for line in disasm.splitlines():
        m = CALL.match(line)
        if m:
            src = owner(int(m.group(1), 16))
            target = m.group(3)
            v = re.fullmatch(r"__(.+)_veneer", target)
            name = v.group(1) if v else target
            if src and name in syms and in_range(syms[name][0], FLASH):
                calls.setdefault(name, set()).add(src)
            continue
        m = BLX.match(line)
        if m:
            src = owner(int(m.group(1), 16))
            if src:
                indirect[src] = indirect.get(src, 0) + 1
    return {k: sorted(v) for k, v in sorted(calls.items())}, indirect


def listed(path):
    names = []
    for line in open(path, encoding="utf-8"):
        row = line.split()
        if row and not row[0].startswith("#"):
            names.extend(row[2:] or [row[1][len(".text."):]])
    return names


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    elf, path = argv[1], argv[2]
    prefix = argv[3] if len(argv) > 3 else "arm-none-eabi-"
    syms = parse_nm(subprocess.check_output(
        [prefix + "nm", "-S", "--defined-only", elf], text=True))
    names = listed(path)
    ram, flash, missing = placement(syms, names)
    disasm = subprocess.check_output(
        [prefix + "objdump", "-D", "--no-show-raw-insn", "-j", ".data", elf],
        text=True)
    calls, indirect = crossings(disasm, syms, ram)
    ram_bytes = sum(syms[n][1] for n in ram)
    ok = not flash and not missing
    print(f"RAM_PROOF {'PASS' if ok else 'FAIL'} listed {len(names)} "
          f"in_ram {len(ram)} bytes {ram_bytes}")
    for name in flash:
        print(f"  STILL_IN_FLASH {name}")
    for name in missing:
        print(f"  MISSING {name}")
    print(f"  flash_callees {len(calls)}")
    for name, srcs in calls.items():
        print(f"    {name} <- {','.join(srcs)}")
    print(f"  indirect_blx {sum(indirect.values())} in "
          f"{','.join(sorted(indirect)) or '-'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
