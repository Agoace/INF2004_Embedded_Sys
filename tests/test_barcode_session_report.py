"""Synthetic GP20 logs for scripts/barcode_session_report.py."""
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import barcode_session_report as rep  # noqa: E402

START = 10000  # run start, uptime ms
CPM = 12414


def kr(t_ms, raw_l, raw_c, v):
    w = (t_ms // 20) | (1 << 16) | (1 << 18) | (raw_l << 21) | (raw_c << 33)
    w |= (v & 1023) << 73 | (v & 1023) << 83
    return f"KR {t_ms // 20} {w.to_bytes(16, 'little').hex()}"


def counts(t_ms, v=100):
    return round(t_ms * v / 1000 * CPM / 1000)


def run_lines(dark=((3000, 3060), (4000, 4060)), delta=0, end_ms=5000,
              evidence_seq=1, edge_ms=3000, kr1="rows 250 events 4"):
    up = lambda ms: (START + ms) * 1000  # noqa: E731
    end = counts(2000)
    lines = [
        "[JUNCTION_COUNTS] epoch 1 busy 0 guard 0 unarmed 0 stop 1 edge 2 confirmed 2",
        "[BC_FRAME] epoch 1 seq 1 letter F cmd 0 reason 1 offered 1 ages_ms 9/0 x",
        f"[BC_REF] epoch 1 seq 1 start_us {up(1000)} edge_us {up(1990)} bracket_us "
        f"{up(2000)} end {end}/{end} eligible_us {up(2005)} read_ms {START + 2010} "
        f"counts {end + 5}/{end + 5} terminal_us {up(2010)}",
        "[BC_FRAME] epoch 1 seq 0 letter - cmd 0 reason 101 offered 0 ages_ms 1/0 x",
        f"[BC_REF] epoch 1 seq 0 start_us {up(1400)} edge_us 0 bracket_us 0 end 0/0 "
        f"eligible_us 0 read_ms 0 counts 0/0 terminal_us {up(1500)}",
        "[BC_FRAME] epoch 1 seq 0 letter - cmd 0 reason 6 offered 0 ages_ms 9/0 x",
        f"[BC_REF] epoch 1 seq 0 start_us {up(4300)} edge_us 0 bracket_us 0 end 0/0 "
        f"eligible_us 0 read_ms 0 counts 0/0 terminal_us {up(4400)}",
        f"[BC_FAIL_CAPTURE] epoch 1 index 1 start_us {up(4300)} terminal_us {up(4400)} "
        "reason 6 window_reason 6 elements 1 rise 350 fall 275 lead_cmm 15000 x",
        f"[BC_FAIL_EDGE] epoch 1 index 1 row 0 adc_us {up(4300)} raw 900 black 1 phase 2 "
        f"mm_cmm 15000 bracket_us {up(4300)} counts 1/1",
        f"[BC_FAIL_EDGE] epoch 1 index 1 row 1 adc_us {up(4600)} raw 200 black 0 phase 2 "
        f"mm_cmm 18250 bracket_us {up(4600)} counts 2/2",
        f"[BC_FAIL_ADC] epoch 1 index 1 row 0 us {up(4350)} raw 1200 flags 0 pre 0",
        f"[robot] KR1 reset power/RUN; 90 s time limit {kr1} lost_telemetry 0",
        "[robot] counts L/R 1/1; line L floor/line 200/3200 C 200/3200; sampled 1",
        f"[timeline] epoch 1 frozen 0 records 128 end_us {up(end_ms) + 638}",
        "VE 0 2 0 1 1", "VE 1500 1 1 0 37", "VE 1500 2 1 2 1",
        f"VE {end_ms} 3 1 0 0",
    ]
    for t in range(0, end_ms, 20):
        black = any(a <= t < b for a, b in dark)
        lines.append(kr(t, 3000 if black else 300, 3000 if black else 250, 100))
    lines.append(f"[robot] end tm_usb_dropped 5 (dump delta {delta})")
    lines += [
        f"[BC_ADMIT] epoch 1 seq {evidence_seq} reason 11 letter 70 cmd 0 valid 0 x",
        f"[BC_EVIDENCE] epoch 1 seq {evidence_seq} read_ms {START + 2010} read_counts "
        f"{end + 5},{end + 5} frame_end {end},{end} captured_rev 9",
        f"[BC_CONTROL] epoch 1 rev 20 cut_ms {START + end_ms} ack 1 mode 0 clear_counts 1 "
        f"edge {counts(edge_ms)},{counts(edge_ms)} edge_us {up(edge_ms)} max_delivery_ms 2 x",
    ]
    return lines


def run(lines):
    with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as f:
        f.write("\n".join(lines) + "\n")
    dumps = rep.parse_dumps(open(f.name, encoding="utf-8"))
    return [rep.report_run(d, n, 0.5) | {"dark_mark": rep.dark_mark(d)}
            for d, n in rep.pick_runs(dumps)]


def basic():
    [r] = run(run_lines())
    assert r["run_start_uptime_ms"] == START
    a1, a2, a3 = r["attempts"]
    assert a1["letter"] == "F" and a1["result"] == "BW_OK" and a1["seq"] == 1
    assert abs(a1["speed_mm_s"] - 100) < 1e-9
    assert abs(a1["d_cross_derived_mm"] - 100) < 1e-9 and a1["dark_pair_ms"] == 3000
    assert a2["result"] == "BP_DISABLED" and a2["end_event"] == "LOST RIGHT at 1500"
    assert a3["result"] == "BW_WIDTH" and a3["capture"] == 1
    dc = r["d_cross"]
    assert dc["seq"] == 1 and abs(dc["mm"] - 100) < 0.1
    assert dc["next_junction"].startswith("consistent")
    [m] = r["dark_mark"]
    assert m["black_widths_mm"] == [32.5] and m["lead_mm"] == 150 and m["peak_raw"] == 1200


def not_next():
    # A dark pair between the frame end and the reported edge.
    [r] = run(run_lines(dark=((2500, 2560), (3000, 3060)), edge_ms=3000))
    assert r["d_cross"]["next_junction"].startswith("NOT the next junction")
    assert r["attempts"][0]["dark_pair_ms"] == 2500


def edge_before():
    [r] = run(run_lines(edge_ms=1800))
    assert r["d_cross"] is None and any("before seq 1" in n for n in r["notes"])


def no_record():
    [r] = run(run_lines(evidence_seq=0))
    assert r["d_cross"] is None and any("seq 0" in n for n in r["notes"])


def repeats_and_runs():
    # Same run dumped twice (lossy first): the delta-0 dump is used, no union.
    lossy = run_lines(delta=7)
    lossy = [x for x in lossy if not x.startswith("[BC_FAIL_EDGE]")]
    good = run_lines(delta=0)
    other = run_lines(kr1="rows 300 events 9", end_ms=6000)
    runs = run(lossy + good + other)
    assert len(runs) == 2
    assert runs[0]["repeats"] == 2 and runs[0]["dump_delta"] == 0
    assert runs[0]["dark_mark"][0]["edges"] == 2
    assert runs[1]["repeats"] == 1 and runs[1]["run_start_uptime_ms"] == START


def garbled_frame():
    # A lost/garbled FRAME must not lend its reason to the next BC_REF.
    lines = run_lines()
    i = next(k for k, x in enumerate(lines) if "reason 101" in x)
    lines[i] = "[BC_FRAME] epoch 1 seq 0 letter - cmd 0 reason 101 offer epoch 1 index 2"
    j = next(k for k, x in enumerate(lines) if "reason 6 offered" in x)
    del lines[j]
    [r] = run(lines)
    assert [a["result"] for a in r["attempts"]] == ["BW_OK", "?", "BW_WIDTH"]


def missing_ref():
    lines = run_lines()
    lines.insert(1, "[BC_REASON_COUNTS] epoch 1 attempts 3 ok 1 failed 2 dropped 0 x")
    k = next(i for i, x in enumerate(lines) if "start_us 14300000" in x and "BC_REF" in x)
    del lines[k]
    [r] = run(lines)
    assert any("lists 2 of 3" in n for n in r["notes"])
    assert any("capture 1 (BW_WIDTH" in n for n in r["notes"])


def stop_event():
    # VE kind 4 (STOP) sets the run start even when later VE rows exist.
    lines = run_lines()
    lines.insert(lines.index("VE 5000 3 1 0 0"), "VE 4997 4 1 1 0")
    [r] = run(lines)
    assert r["run_start_uptime_ms"] == START + 3


for case in (basic, garbled_frame, stop_event, missing_ref, not_next, edge_before, no_record, repeats_and_runs):
    case()
print("barcode_session_report synthetic logs: PASS")
