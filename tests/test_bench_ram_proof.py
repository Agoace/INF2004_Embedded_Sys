#!/usr/bin/env python3
"""Parser checks for the RAM placement proof (synthetic tool output)."""
from pathlib import Path
import importlib.util
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "ram_proof", ROOT / "app_program/bench_kernel/ram_proof.py")
proof = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(proof)

NM = """\
20000100 00000010 T tk_wup_tsk
20000110 00000008 t knl_make_ready
10001000 00000020 T tk_slp_tsk
10002000 00000004 T knl_ready_queue_top
20000118 00000008 t __knl_ready_queue_top_veneer
20000120 T knl_dispatch_entry
20000140 T l_after
20000140 00000004 T same_address_sized
"""

DIS = """\
20000100 <tk_wup_tsk>:
20000100:\tbl\t20000110 <knl_make_ready>
20000104:\tbl\t20000118 <__knl_ready_queue_top_veneer>
20000108:\tblx\tr3
2000010c:\tbl\t20000118 <__knl_ready_queue_top_veneer>
20000110 <knl_make_ready>:
20000110:\tbx\tlr
20000120 <knl_dispatch_entry>:
20000120:\tbl\t20000118 <__knl_ready_queue_top_veneer>
"""


class Proof(unittest.TestCase):
    def test_placement_split(self):
        syms = proof.parse_nm(NM)
        ram, flash, missing = proof.placement(
            syms, ["tk_wup_tsk", "knl_make_ready", "tk_slp_tsk", "nope"])
        self.assertEqual(ram, ["tk_wup_tsk", "knl_make_ready"])
        self.assertEqual(flash, ["tk_slp_tsk"])
        self.assertEqual(missing, ["nope"])

    def test_size_less_label_bounded_by_next_symbol(self):
        syms = proof.parse_nm(NM)
        self.assertEqual(syms["knl_dispatch_entry"], (0x20000120, 0x20))

    def test_flash_callees_through_veneers(self):
        syms = proof.parse_nm(NM)
        calls, indirect = proof.crossings(
            DIS, syms, ["tk_wup_tsk", "knl_make_ready",
                        "knl_dispatch_entry"])
        self.assertEqual(calls, {"knl_ready_queue_top":
                                 ["knl_dispatch_entry", "tk_wup_tsk"]})
        self.assertEqual(indirect, {"tk_wup_tsk": 1})


if __name__ == "__main__":
    unittest.main()
