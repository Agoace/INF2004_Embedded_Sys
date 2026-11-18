"""Host-only validator tests; no reader policy changes."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fail_analyze", ROOT / "scripts/barcode_fail_analyze.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

WIDTHS = [3.11, 20.74, 2.02, 9.20, 14.57, 9.08, 14.14, 9.52, 2.44]


def fixture(drop=None):
    lines = ["=== connect", "[BC_FAIL_STATUS] slots 2 edges 64 samples 512 pre 64 div_us 4000 missed_lifetime 0",
             "[BC_REASON_COUNTS] epoch 1 attempts 1 ok 0 failed 1 dropped 0 BW0..10 0",
             "[BC_FAIL_CAPTURE] epoch 1 index 2 start_us 10000 terminal_us 700000 reason 5 window_reason 5 elements 9 rise 326 fall 251 lead_cmm 4263 fit_valid 0 n_cmm 0 w_cmm 0 bias_cmm 0 space_cmm 0 pre 1 adc 1 edge 10 adc_lost 0 edge_lost 0 div_us 4000"]
    pos = 0
    for row in range(10):
        if row: pos += round(WIDTHS[row - 1] * 100)
        lines.append(f"[BC_FAIL_EDGE] epoch 1 index 2 row {row} adc_us {10000+row*10000} raw 400 black {1-row%2} phase 2 mm_cmm {pos} bracket_us {11000+row*10000} counts 1/1")
    for row in range(2):
        if row != drop:
            lines.append(f"[BC_FAIL_ADC] epoch 1 index 2 row {row} us {6000+row*8000} raw 170 flags 0 pre {1-row}")
    lines += ["[BC_EDGES] total 0 retained 0 lost 0", "[robot] end tm_usb_dropped 123 (dump delta 0)"]
    return "\n".join(lines)


class AnalyzeTests(unittest.TestCase):
    def test_complete_and_rounded_star(self):
        report = m.analyze([fixture()])
        self.assertTrue(report["complete"])
        star = report["captures"][0]["star"]
        self.assertEqual(star["mask"], "0x094")
        self.assertEqual(star["decision"], "start_gates_pass")  # 42.63 >= 40 mm policy
        self.assertAlmostEqual(star["legacy_10n_leading_mm"], 58.95)
        self.assertIsNone(report["captures"][0]["context"])
        self.assertEqual(m.star_check(WIDTHS, 39.99)["decision"], "leading_white_short")
        self.assertAlmostEqual(star["narrow_mm"], 5.895)
        self.assertAlmostEqual(star["ratio"], 2.9868061446)

    def test_delta_zero_does_not_hide_missing_adc(self):
        report = m.analyze([fixture(0)])
        self.assertFalse(report["complete"])
        self.assertEqual(report["captures"][0]["missing_adc"], [0])

    def test_repeated_dump_not_double_counted(self):
        report = m.analyze([fixture()+"\n"+fixture()])
        self.assertEqual(report["dump_count"], 2)
        self.assertEqual(len(report["captures"]), 1)

    def test_explicit_union_only_and_conflict(self):
        self.assertFalse(m.analyze([fixture(0), fixture(1)])["complete"])
        self.assertTrue(m.analyze([fixture(0), fixture(1)], merge=True)["complete"])
        bad = fixture().replace("raw 170 flags", "raw 171 flags")
        self.assertFalse(m.analyze([fixture(), bad], merge=True)["complete"])

    def test_never_union_different_run_epochs(self):
        report = m.analyze([fixture(), fixture().replace("epoch 1", "epoch 3")], merge=True)
        self.assertFalse(report["complete"])
        self.assertTrue(any("different run epochs" in error for error in report["errors"]))

    def test_malformed_line_and_barcode_only(self):
        bad = fixture().replace("epoch 1 index 2 row 0 us", "ep_FAIL_ADC] epoch 1 index 2 row 0 us")
        self.assertFalse(m.analyze([bad])["complete"])
        self.assertFalse(m.analyze([fixture().split("[robot] end")[0]])["complete"])

    def test_context_line(self):
        line = "[BC_FAIL_CONTEXT] epoch 1 index 2 enable_us 4000 reset_us 0 reset_reason 0"
        text = fixture().replace("[BC_EDGES]", line + "\n[BC_EDGES]")
        c = m.analyze([text])["captures"][0]
        self.assertEqual(c["context"]["enable_before_start_ms"], 6.0)
        self.assertIsNone(c["context"]["reset_us"])
        bad = text.replace("enable_us 4000", "enable_us 4001")
        self.assertFalse(m.analyze([text, bad], merge=True)["complete"])

    def test_nonstar_and_wrap(self):
        star = m.star_check([15.97,8.63,13.49,9.54,3.22,9.48,2.12,7.15,14.83],46.44)
        self.assertEqual(star["mask"], "0x141")
        self.assertEqual(star["decision"], "not_star")
        self.assertEqual(m.relative_us(100, 0xFFFFFF00), 356)


if __name__ == "__main__":
    unittest.main()
