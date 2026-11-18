#!/usr/bin/env python3
"""Per-attempt barcode report and D_cross from a GP20 dump log.

Reads one robot USB log (one or more runs, repeated dumps allowed). For each
run it uses ONE dump: the most complete one (smallest dump delta, then the
latest). It never unions different dumps.

Per attempt it prints the letter, the result, the KR wheel speed, the
LINE->SEARCH event at the end (if any), and the dark-pair rows of the
line sensors after the attempt.

D_cross (mm) is printed in two forms, labelled:
  MEASURED  ((edgeL-endL)+(edgeR-endR))/2 x 1000/12414, from [BC_EVIDENCE]
            frame_end and [BC_CONTROL] edge. The firmware keeps only the LAST
            record and the LAST confirmed junction, so this exists for at
            most one attempt per run, and only when the edge is after it.
  DERIVED   KR wheel-speed integral from the frame end (BC_REF bracket_us)
            to the first later KR row where both line channels are dark.
            20 ms rows; the dark threshold is a diagnostic, not the
            firmware junction detector.

No dependencies beyond the standard library and robot_log_decode.py.
"""
import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_log_decode import decode  # noqa: E402

COUNTS_PER_M = 12414
KR_ROW_MS = 20
REASONS = dict(enumerate(("BW_WAIT", "BW_OK", "BW_CALIBRATION", "BW_LOSS",
                          "BW_MOTION", "BW_START", "BW_WIDTH", "BW_GAP",
                          "BW_EXTENT", "BW_TAIL", "BW_NOT_LETTER")))
REASONS.update(dict(enumerate(("BP_CONTROL", "BP_DISABLED", "BP_CONTEXT",
                               "BP_CUT", "BP_CALIBRATION", "BP_RECORD_AGE",
                               "BP_MOTOR_AGE", "BP_CLOCK"), 100)))
VE_LOST = {5: "LOST(side unknown)", 21: "LOST LEFT", 37: "LOST RIGHT"}

FRAME = re.compile(r"\[BC_FRAME\] epoch (\d+) seq (\d+) letter (\S) cmd (\d+) "
                   r"reason (\d+) offered (\d+) .*")
REF = re.compile(r"\[BC_REF\] epoch (\d+) seq (\d+) start_us (\d+) edge_us (\d+) "
                 r"bracket_us (\d+) end (-?\d+)/(-?\d+) eligible_us (\d+) "
                 r"read_ms (\d+) counts (-?\d+)/(-?\d+) terminal_us (\d+)\s*")
EVIDENCE = re.compile(r"\[BC_EVIDENCE\] epoch (\d+) seq (\d+) read_ms (\d+) "
                      r"read_counts (-?\d+),(-?\d+) frame_end (-?\d+),(-?\d+) "
                      r"captured_rev (\d+)\s*")
CONTROL = re.compile(r"\[BC_CONTROL\] epoch (\d+) rev (\d+) cut_ms (\d+) ack (\d+) "
                     r"mode (\d+) clear_counts (-?\d+) edge (-?\d+),(-?\d+) "
                     r"edge_us (\d+) .*")
ADMIT = re.compile(r"\[BC_ADMIT\] epoch (\d+) seq (\d+) reason (\d+) letter (\d+) .*")
CAPTURE = re.compile(r"\[BC_FAIL_CAPTURE\] epoch (\d+) index (\d+) start_us (\d+) "
                     r"terminal_us (\d+) reason (\d+) window_reason (\d+) "
                     r"elements (\d+) rise (\d+) fall (\d+) lead_cmm (\d+) .*")
FEDGE = re.compile(r"\[BC_FAIL_EDGE\] epoch (\d+) index (\d+) row (\d+) adc_us (\d+) "
                   r"raw (\d+) black (\d) phase (\d+) mm_cmm (-?\d+) "
                   r"bracket_us (\d+) counts (-?\d+)/(-?\d+)\s*")
FADC = re.compile(r"\[BC_FAIL_ADC\] epoch (\d+) index (\d+) row (\d+) us (\d+) "
                  r"raw (\d+) flags (\d+) pre (\d+)\s*")
TIMELINE = re.compile(r"\[timeline\] epoch (\d+) frozen \d+ records \d+ end_us (\d+)\s*")
KR1 = re.compile(r"\[robot\] KR1 .*rows (\d+) events (\d+) .*")
END = re.compile(r"\[robot\] end .*\(dump delta (\d+)\)\s*")
CAL = re.compile(r"\[robot\] counts .*line L floor/line (\d+)/(\d+) C (\d+)/(\d+);.*")
KR = re.compile(r"KR (\d+) ([0-9a-fA-F]{32})\s*")
COUNTS_LINE = re.compile(r"\[BC_REASON_COUNTS\] epoch (\d+) attempts (\d+) .*")
VE = re.compile(r"VE (\d+) (\d+) (\d+) (\d+) (\d+)\s*")


def mm(counts):
    return counts * 1000.0 / COUNTS_PER_M


def parse_dumps(lines):
    """Split into dumps. A dump ends at '[robot] end'. The stop report
    ([BC_ADMIT]/[BC_EVIDENCE]/[BC_CONTROL]) printed after it belongs to it."""
    dumps, cur, last = [], None, None
    previous = None  # last parsed tag; a BC_REF pairs only with an adjacent BC_FRAME

    def new():
        return {"frames": [], "refs": [], "captures": {}, "fedges": {},
                "fadc": {}, "kr": {}, "ve": [], "end_us": None, "key": None,
                "delta": None, "cal": None, "evidence": None, "control": None,
                "admit": None, "lines": 0, "attempts": None}
    for raw in lines:
        line = raw.rstrip("\n")
        tail = re.match(r"\[BC_(ADMIT|EVIDENCE|CONTROL)\]", line)
        if tail and cur is None and last is not None:
            target = last
        else:
            if cur is None:
                cur = new()
            target = cur
        target["lines"] += 1
        tag = None
        if m := FRAME.fullmatch(line):
            tag = "frame"
            target["frames"].append({"epoch": int(m[1]), "seq": int(m[2]),
                                     "letter": m[3], "cmd": int(m[4]),
                                     "reason": int(m[5])})
        elif m := REF.fullmatch(line):
            v = list(map(int, m.groups()))
            target["refs"].append({"epoch": v[0], "seq": v[1], "start_us": v[2],
                                   "edge_us": v[3], "bracket_us": v[4],
                                   "end": (v[5], v[6]), "eligible_us": v[7],
                                   "read_ms": v[8], "counts": (v[9], v[10]),
                                   "terminal_us": v[11],
                                   "frame": target["frames"][-1]
                                   if previous == "frame" else None})
        elif m := CAPTURE.fullmatch(line):
            v = list(map(int, m.groups()))
            target["captures"][v[1]] = {"index": v[1], "start_us": v[2],
                                        "terminal_us": v[3], "reason": v[4],
                                        "elements": v[6], "rise": v[7],
                                        "fall": v[8], "lead_mm": v[9] / 100}
        elif m := FEDGE.fullmatch(line):
            v = list(map(int, m.groups()))
            target["fedges"].setdefault(v[1], {})[v[2]] = {
                "adc_us": v[3], "raw": v[4], "black": v[5], "mm": v[7] / 100}
        elif m := FADC.fullmatch(line):
            v = list(map(int, m.groups()))
            target["fadc"].setdefault(v[1], {})[v[2]] = {"us": v[3], "raw": v[4]}
        elif m := COUNTS_LINE.fullmatch(line):
            target["attempts"] = int(m[2])
        elif m := TIMELINE.fullmatch(line):
            target["end_us"] = int(m[2])
        elif m := KR1.fullmatch(line):
            target["key"] = (int(m[1]), int(m[2]))
        elif m := CAL.fullmatch(line):
            target["cal"] = tuple(map(int, m.groups()))
        elif m := KR.fullmatch(line):
            try:
                target["kr"][int(m[1])] = decode(m[2])
            except ValueError:
                pass
        elif m := VE.fullmatch(line):
            target["ve"].append(tuple(map(int, m.groups())))
        elif m := EVIDENCE.fullmatch(line):
            v = list(map(int, m.groups()))
            target["evidence"] = {"epoch": v[0], "seq": v[1], "read_ms": v[2],
                                  "frame_end": (v[5], v[6])}
        elif m := CONTROL.fullmatch(line):
            v = list(map(int, m.groups()))
            target["control"] = {"epoch": v[0], "cut_ms": v[2],
                                 "edge": (v[6], v[7]), "edge_us": v[8]}
        elif m := ADMIT.fullmatch(line):
            target["admit"] = {"epoch": int(m[1]), "seq": int(m[2]),
                               "reason": int(m[3]), "letter": int(m[4])}
        elif line.startswith("[BC_FRAME]"):
            tag = "bad_frame"
        elif m := END.fullmatch(line):
            target["delta"] = int(m[1])
            dumps.append(target)
            last, cur = target, None
        if line.startswith("[BC_") or tag:
            previous = tag
    if cur is not None and (cur["refs"] or cur["kr"]):
        dumps.append(cur)
    return dumps


def pick_runs(dumps):
    """One dump per run key; never union. Prefer delta 0, then the latest."""
    runs = {}
    order = []
    for d in dumps:
        key = (d["key"], d["end_us"])
        if key not in runs:
            order.append(key)
            runs[key] = []
        runs[key].append(d)
    picked = []
    for key in order:
        group = runs[key]
        best = min(range(len(group)), key=lambda i: (
            group[i]["delta"] if group[i]["delta"] is not None else 10**9, -i))
        picked.append((group[best], len(group)))
    return picked


def run_start_ms(d):
    """end_us of the frozen timeline is the STOP; its VE has the same
    run-relative time. Returns None when either is missing."""
    if d["end_us"] is None or not d["ve"]:
        return None
    stops = [t for t, kind, *_ in d["ve"] if kind == 4]  # VE kind 4 = STOP
    return d["end_us"] // 1000 - (stops[-1] if stops else max(t for t, *_ in d["ve"]))


def kr_rows(d):
    rows = [d["kr"][i] for i in sorted(d["kr"])]
    return rows


def speed_between(rows, a_ms, b_ms):
    sel = [r for r in rows if a_ms <= r[0] < b_ms]
    if not sel:
        return None
    return sum((r[9] + r[10]) / 2 for r in sel) / len(sel)


def travel_between(rows, a_ms, b_ms):
    return sum((r[9] + r[10]) / 2 * KR_ROW_MS / 1000 for r in rows
               if a_ms <= r[0] < b_ms)


def dark(row, cal, threshold):
    fl, ll, fc, lc = cal
    left = (row[4] - fl) / max(1, ll - fl)
    centre = (row[5] - fc) / max(1, lc - fc)
    return min(left, centre) >= threshold


def dark_clusters(rows, cal, threshold):
    """Start times of runs of KR rows with both channels dark (gap > 60 ms
    starts a new cluster)."""
    starts, prev = [], None
    for r in rows:
        if dark(r, cal, threshold):
            if prev is None or r[0] - prev > 60:
                starts.append(r[0])
            prev = r[0]
    return starts


def end_event(d, t_ms):
    """A LINE->SEARCH loss within 3 ms of the attempt end."""
    for t, kind, _frm, _to, word in d["ve"]:
        if kind == 1 and abs(t - t_ms) <= 3 and word in VE_LOST:
            return f"{VE_LOST[word]} at {t}"
    return ""


def report_run(d, repeats, threshold):
    start = run_start_ms(d)
    rows = kr_rows(d)
    out = {"key": d["key"], "dump_delta": d["delta"], "repeats": repeats,
           "run_start_uptime_ms": start, "attempts": [], "d_cross": None,
           "notes": []}
    clusters = dark_clusters(rows, d["cal"], threshold) if d["cal"] and rows else []
    out["dark_pair_clusters"] = clusters
    for i, ref in enumerate(d["refs"], 1):
        frame = ref["frame"] or {}
        reason = frame.get("reason")
        if reason is None:  # FRAME line lost or garbled: use the capture if any
            reason = next((c["reason"] for c in d["captures"].values()
                           if c["start_us"] == ref["start_us"]), None)
            if reason is None:  # a published record (seq > 0) is BW_OK
                reason = 1 if ref["seq"] else "?"
        a = {"n": i, "seq": ref["seq"], "letter": frame.get("letter", "?"),
             "reason": reason, "result": REASONS.get(reason, str(reason)),
             "start_us": ref["start_us"], "terminal_us": ref["terminal_us"]}
        if start is not None:
            s = ref["start_us"] // 1000 - start
            e = ref["terminal_us"] // 1000 - start
            a.update(start_ms=s, end_ms=e, speed_mm_s=speed_between(rows, s, max(e, s + 20)),
                     end_event=end_event(d, e))
            if reason == 1 and ref["bracket_us"]:
                f = ref["bracket_us"] // 1000 - start
                nxt = next((c for c in clusters if c > f), None)
                if nxt is not None:
                    a["d_cross_derived_mm"] = travel_between(rows, f, nxt)
                    a["dark_pair_ms"] = nxt
        for cap in d["captures"].values():
            if cap["start_us"] == ref["start_us"]:
                a["capture"] = cap["index"]
        out["attempts"].append(a)
    if d["attempts"] is not None and d["attempts"] != len(d["refs"]):
        out["notes"].append(f"this dump lists {len(d['refs'])} of {d['attempts']} "
                            "attempts (BC_REF lines lost or garbled)")
    for cap in d["captures"].values():
        if not any(r["start_us"] == cap["start_us"] for r in d["refs"]):
            out["notes"].append(f"capture {cap['index']} ({REASONS.get(cap['reason'])}, "
                                f"start_us {cap['start_us']}) has no BC_REF in this dump")
    ev, ctl = d["evidence"], d["control"]
    if ev and ctl and ev["seq"]:
        ref = next((r for r in d["refs"] if r["seq"] == ev["seq"] and
                    r["epoch"] == ev["epoch"]), None)
        if ref is None:
            out["notes"].append(f"[BC_EVIDENCE] seq {ev['seq']} has no BC_REF in this dump")
        elif ref["end"] != ev["frame_end"]:
            out["notes"].append("BC_EVIDENCE frame_end differs from BC_REF end")
        elif ctl["edge_us"] <= ref["terminal_us"]:
            out["notes"].append(f"[BC_CONTROL] edge is before seq {ev['seq']}: no D_cross")
        else:
            dl = ctl["edge"][0] - ev["frame_end"][0]
            dr = ctl["edge"][1] - ev["frame_end"][1]
            dc = {"seq": ev["seq"], "mm": mm((dl + dr) / 2),
                  "left_mm": mm(dl), "right_mm": mm(dr)}
            if start is not None:
                f = ref["bracket_us"] // 1000 - start
                edge = ctl["edge_us"] // 1000 - start
                between = [c for c in clusters if f < c < edge - 60]
                dc.update(frame_end_ms=f, edge_ms=edge, dark_pair_between=between,
                          next_junction="consistent (no dark-pair rows between)"
                          if not between else
                          f"NOT the next junction: {len(between)} dark-pair clusters between")
            out["d_cross"] = dc
    elif ev is None or ctl is None:
        out["notes"].append("no [BC_EVIDENCE]/[BC_CONTROL] stop report for this run")
    else:
        out["notes"].append("stop report has no record (seq 0): no D_cross")
    return out


def dark_mark(d):
    """BW_WIDTH captures: element widths from capture edges (reader mm)."""
    marks = []
    for idx, cap in sorted(d["captures"].items()):
        if cap["reason"] != 6:
            continue
        edges = [d["fedges"].get(idx, {})[k] for k in sorted(d["fedges"].get(idx, {}))]
        widths = [round(b["mm"] - a["mm"], 2) for a, b in zip(edges, edges[1:])
                  if a["black"] == 1 and b["black"] == 0]
        adc = d["fadc"].get(idx, {})
        peak = max((v["raw"] for v in adc.values()), default=None)
        marks.append({"index": idx, "lead_mm": cap["lead_mm"],
                      "elements": cap["elements"], "black_widths_mm": widths,
                      "edges": len(edges), "adc_rows": len(adc), "peak_raw": peak,
                      "rise": cap["rise"], "fall": cap["fall"]})
    return marks


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log")
    p.add_argument("--json", action="store_true")
    p.add_argument("--dark", type=float, default=0.5,
                   help="dark-pair fraction of the printed line range (DIAGNOSTIC)")
    args = p.parse_args(argv)
    with open(args.log, encoding="utf-8", errors="replace") as f:
        dumps = parse_dumps(f)
    result = []
    for d, repeats in pick_runs(dumps):
        r = report_run(d, repeats, args.dark)
        r["dark_mark"] = dark_mark(d)
        result.append(r)
    if args.json:
        json.dump(result, sys.stdout, indent=1, default=str)
        print()
        return 0
    for r in result:
        print(f"RUN KR1 rows/events {r['key']} start_uptime_ms {r['run_start_uptime_ms']} "
              f"dump_delta {r['dump_delta']} dumps {r['repeats']} "
              f"dark_pair_clusters {len(r['dark_pair_clusters'])}")
        print("  #  seq letter result          start_ms  end_ms  v_mm_s  end_event            "
              "D_cross_derived_mm")
        for a in r["attempts"]:
            v = a.get("speed_mm_s")
            dd = a.get("d_cross_derived_mm")
            print(f"  {a['n']:<2} {a['seq']:<3} {a['letter']:<6} {a['result']:<15} "
                  f"{a.get('start_ms', '-'):>8} {a.get('end_ms', '-'):>7} "
                  f"{'-' if v is None else f'{v:6.0f}':>7}  {a.get('end_event', ''):<20} "
                  f"{'-' if dd is None else f'{dd:.0f} (DERIVED)'}")
        dc = r["d_cross"]
        if dc:
            print(f"  D_cross MEASURED seq {dc['seq']}: {dc['mm']:.1f} mm "
                  f"(L {dc['left_mm']:.1f} R {dc['right_mm']:.1f}); "
                  f"{dc.get('next_junction', 'next junction not checked')}")
        for n in r["notes"]:
            print(f"  note: {n}")
        for m in r["dark_mark"]:
            print(f"  BW_WIDTH capture {m['index']}: lead {m['lead_mm']} mm, "
                  f"black widths {m['black_widths_mm']} mm (reader projection), "
                  f"edges {m['edges']}, peak raw {m['peak_raw']}, rise {m['rise']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
