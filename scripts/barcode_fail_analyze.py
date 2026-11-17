#!/usr/bin/env python3
"""Validate stopped BC_FAIL dumps; export measured edges/ADC, never invent gaps.

--merge-repeats asserts that ALL inputs are the same powered RAM run. Header
and overlapping row values must agree. Rounded edge model checks are diagnostic,
not a bit-exact reader replay and not proof of physical barcode placement.
"complete" means declared capture rows arrived, NOT KR integrity/full-rate ADC.
"""
import argparse
import copy
import csv
import json
from pathlib import Path
import re
import sys

TAG = re.compile(r"\[(BC_FAIL_[A-Z]+)\](.*)$")
REQUIRED = {
    "STATUS": "slots edges samples pre div_us missed_lifetime",
    "CAPTURE": "epoch index start_us terminal_us reason window_reason elements rise fall lead_cmm fit_valid n_cmm w_cmm bias_cmm space_cmm pre adc edge adc_lost edge_lost div_us",
    "EDGE": "epoch index row adc_us raw black phase mm_cmm bracket_us counts",
    "ADC": "epoch index row us raw flags pre",
    "REF": "epoch index start_us terminal_us reason",
    "CONTEXT": "epoch index enable_us reset_us reset_reason",
}
QUIET_MIN_MM = 40.0  # Reader policy from mtk3-bc-quiet; older images used 10n.


def relative_us(value, start):
    return ((value - start + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


def fields(body, required):
    parts = body.split()
    if len(parts) % 2:
        raise ValueError("odd field/value count")
    out = {}
    for key, value in zip(parts[::2], parts[1::2]):
        if key in out or not re.fullmatch(r"[a-z][a-z0-9_]*", key):
            raise ValueError("invalid/duplicate field")
        if key == "counts":
            left, right = value.split("/")
            out[key] = [int(left), int(right)]
        else:
            out[key] = int(value)
    if not set(required.split()) <= out.keys():
        raise ValueError("missing required fields")
    return out


def star_check(widths, leading):
    """Mirror the current nine-run model on rounded diagnostic widths."""
    out = {"rounded_diagnostic_only": True, "widths_mm": widths}
    if len(widths) != 9 or min(widths) <= 0:
        return dict(out, decision="insufficient_positive_widths")
    ordered = sorted(widths)
    if ordered[6] <= ordered[5]:
        return dict(out, decision="no_rank_split")
    split = (ordered[5] + ordered[6]) / 2
    wide = [x > split for x in widths]
    mask = sum(0x100 >> i for i, value in enumerate(wide) if value)
    reverse = int(f"{mask:09b}"[::-1], 2)
    out.update(mask=f"0x{mask:03x}", reverse_mask=f"0x{reverse:03x}",
               pattern="".join(("W" if b else "N") if i % 2 == 0 else
                               ("w" if b else "n") for i, b in enumerate(wide)))
    narrow_bar = [x for i, x in enumerate(widths) if not wide[i] and i % 2 == 0]
    narrow_space = [x for i, x in enumerate(widths) if not wide[i] and i % 2 == 1]
    if not narrow_bar or not narrow_space:
        return dict(out, decision="missing_narrow_polarity")
    bar = sum(narrow_bar) / len(narrow_bar)
    space = sum(narrow_space) / len(narrow_space)
    physical, bias = (bar + space) / 2, (space - bar) / 2
    corrected = [x + (bias if i % 2 == 0 else -bias) for i, x in enumerate(widths)]
    ns = [x for x, w in zip(corrected, wide) if not w]
    ws = [x for x, w in zip(corrected, wide) if w]
    n, w = sum(ns) / 6, sum(ws) / 3
    if n <= 0 or max(ns) <= 0:
        return dict(out, decision="invalid_corrected_widths")
    gates = {
        "bias": abs(bias) <= physical * .8,
        "narrow_4_to_7": 4 <= n <= 7,
        "ratio_1_6_to_3_4": 1.6 <= w / n <= 3.4,
        "class_separation": min(ws) / max(ns) >= 1.4,
        "narrow_spread": min(ns) >= n * .65 and max(ns) <= n * 1.5,
        "wide_spread": min(ws) >= w * .65 and max(ws) <= w * 1.5,
    }
    out.update(narrow_mm=physical, wide_mm=w, bias_mm=bias, ratio=w / n,
               corrected_mm=corrected, gates=gates, leading_mm=leading,
               required_leading_mm=QUIET_MIN_MM,
               legacy_10n_leading_mm=10 * physical,
               leading_in_narrow_units=leading / physical)
    decision = "size_gate" if not all(gates.values()) else (
        "not_star" if mask != 0x094 and reverse != 0x094 else (
            "leading_white_short" if leading < QUIET_MIN_MM else "start_gates_pass"))
    return dict(out, decision=decision)


def pre_black_runs(rows, header):
    """Observed Schmitt runs before candidate time; never bridge missing rows."""
    result, active, previous, black = [], None, None, False
    for a in sorted(rows, key=lambda r: r["row"]):
        if not a["pre"] or relative_us(a["us"], header["start_us"]) >= 0:
            continue
        gap = previous is not None and a["row"] != previous + 1
        if gap:
            if active:
                active["end_censored"] = True
                result.append(active)
            active, black = None, False
        now_black = a["raw"] > header["fall"] if black else a["raw"] >= header["rise"]
        if now_black and not black:
            active = {"start_sample_us": a["us"], "last_black_sample_us": a["us"],
                      "peak_raw": a["raw"], "start_row": a["row"],
                      "start_censored": gap or previous is None}
        if now_black and active:
            active["last_black_sample_us"] = a["us"]
            active["peak_raw"] = max(active["peak_raw"], a["raw"])
        if black and not now_black and active:
            active["first_fall_sample_us"] = a["us"]
            active["end_censored"] = False
            result.append(active)
            active = None
        black, previous = now_black, a["row"]
    if active:
        active["end_censored"] = True
        result.append(active)
    return result


def parse(text, source):
    dumps = []
    current = None
    for number, line in enumerate(text.splitlines(), 1):
        match = TAG.search(line)
        if match:
            kind = match[1].removeprefix("BC_FAIL_")
            if kind == "STATUS" or current is None:
                current = {"source": source, "line": number, "status": None,
                           "histogram": None, "captures": {}, "malformed": [],
                           "barcode_end": False, "full_end": False}
                dumps.append(current)
            try:
                row = fields(match[2], REQUIRED[kind])
            except (KeyError, ValueError):
                current["malformed"].append({"line": number, "text": line})
                continue
            if kind == "STATUS":
                current["status"] = row
            elif kind in ("CAPTURE", "EDGE", "ADC", "CONTEXT"):
                key = (row["epoch"], row["index"])
                c = current["captures"].setdefault(key, {"header": None, "edges": [], "adc": [], "issues": []})
                if kind == "CONTEXT":
                    if c.get("context") not in (None, row):
                        c["issues"].append("conflicting capture context")
                    c["context"] = row
                elif kind == "CAPTURE":
                    if c["header"] is not None:
                        c["issues"].append("duplicate capture header in one dump")
                    c["header"] = row
                else:
                    c["edges" if kind == "EDGE" else "adc"].append(row)
        elif current:
            if "[BC_REASON_COUNTS]" in line:
                prefix = line.split("[BC_REASON_COUNTS]", 1)[1].split("BW0..10", 1)[0]
                try:
                    current["histogram"] = fields(prefix, "epoch attempts failed")
                except ValueError:
                    pass
            if "[BC_EDGES]" in line:
                current["barcode_end"] = True
            if "[robot] end tm_usb_dropped" in line:
                current["full_end"] = True
                current["end_text"] = line
    return dumps


def unique_rows(rows, issues, kind, merge):
    out = {}
    for row in rows:
        i = row["row"]
        if i in out:
            if row != out[i]:
                issues.append(f"conflicting {kind} row {i}")
            elif not merge:
                issues.append(f"duplicate {kind} row {i}")
        else:
            out[i] = row
    return out


def analyze(texts, merge=False, sources=None):
    sources = sources or [str(i) for i in range(len(texts))]
    dumps = [d for text, source in zip(texts, sources) for d in parse(text, source)]
    if not dumps:
        return {"complete": False, "errors": ["no BC_FAIL_STATUS/capture block"], "dump_count": 0, "captures": []}
    last = dumps[-1]
    selected = dumps if merge else [last]
    captures, errors = {}, []
    epochs = {d["histogram"]["epoch"] for d in selected if d["histogram"]}
    if merge and len(epochs) > 1:
        errors.append("refuse repeat union across different run epochs; separate the input runs")
    for dump in selected:
        for key, value in dump["captures"].items():
            c = captures.setdefault(key, {"header": None, "edges": [], "adc": [], "issues": []})
            if c["header"] and value["header"] and c["header"] != value["header"]:
                c["issues"].append("conflicting immutable capture header (same boot/run required)")
            if value["header"]:
                c["header"] = copy.deepcopy(value["header"])
            if value.get("context"):
                if c.get("context") not in (None, value["context"]):
                    c["issues"].append("conflicting capture context (same boot/run required)")
                c["context"] = value["context"]
            c["edges"].extend(value["edges"])
            c["adc"].extend(value["adc"])
            c["issues"].extend(value["issues"])
    if not last["status"]: errors.append("last dump status missing/malformed")
    if not last["histogram"]: errors.append("cannot verify expected capture count: reason histogram missing")
    if not last["barcode_end"]: errors.append("last barcode section lacks BC_EDGES boundary")
    if not last["full_end"]: errors.append("last dump lacks robot end marker")
    result = []
    for key, c in sorted(captures.items()):
        h = c["header"]
        if not h:
            errors.append(f"capture {key} missing header")
            continue
        issues = list(c["issues"])
        edges = unique_rows(c["edges"], issues, "edge", merge)
        adc = unique_rows(c["adc"], issues, "ADC", merge)
        ne, na = h["edge"], h["pre"] + h["adc"]
        if not (0 <= ne <= 64 and 0 <= h["pre"] <= 64 and 0 <= h["adc"] <= 448):
            issues.append("header capacity out of range")
            ne, na = min(max(ne, 0), 64), min(max(na, 0), 512)
        missing_e = sorted(set(range(ne)) - edges.keys())
        missing_a = sorted(set(range(na)) - adc.keys())
        if missing_e: issues.append("missing edge rows")
        if missing_a: issues.append("missing ADC rows")
        if set(edges) - set(range(ne)) or set(adc) - set(range(na)):
            issues.append("row outside declared bounds")
        for i, row in adc.items():
            if row["pre"] != int(i < h["pre"]): issues.append(f"ADC row {i} pre flag mismatch")
        if 0 in edges and edges[0]["adc_us"] != h["start_us"]:
            issues.append("first edge does not match original start")
        star = None
        if h["elements"] >= 9 and all(i in edges for i in range(10)):
            if all(edges[i]["black"] == 1-i%2 for i in range(10)):
                widths = [(edges[i+1]["mm_cmm"]-edges[i]["mm_cmm"])/100 for i in range(9)]
                star = star_check(widths, h["lead_cmm"]/100)
            else: issues.append("nonalternating first ten edges")
        ctx = c.get("context")
        context = None  # Older images print no context; absent is not zero.
        if ctx:
            context = {"enable_us": ctx["enable_us"] or None, "reset_us": ctx["reset_us"] or None,
                       "reset_reason": ctx["reset_reason"] if ctx["reset_us"] else None}
            for name in ("enable", "reset"):
                if context[f"{name}_us"] is not None:
                    context[f"{name}_before_start_ms"] = -relative_us(context[f"{name}_us"], h["start_us"]) / 1000
        result.append(dict(header=h, context=context, edges=list(edges.values()), adc=list(adc.values()),
                           missing_edges=missing_e, missing_adc=missing_a, issues=issues,
                           star=star, pre_black_runs=pre_black_runs(list(adc.values()), h),
                           storage_truncated=bool(h["adc_lost"] or h["edge_lost"]),
                           complete=not issues))
    hist = last["histogram"]
    if hist and last["status"]:
        expected = min(last["status"]["slots"], hist["failed"])
        observed = sum(c["header"]["epoch"] == hist["epoch"] for c in result)
        if observed != expected: errors.append(f"expected {expected} captures for epoch {hist['epoch']}, found {observed}")
    return {"complete": not errors and all(c["complete"] for c in result),
            "merged_same_run_assertion": merge, "dump_count": len(dumps),
            "errors": errors, "last_status": last["status"], "last_histogram": hist,
            "end_text": last.get("end_text"), "captures": result,
            "dumps": [{k: v for k, v in d.items() if k != "captures"} for d in dumps]}


def export_csv(report, directory):
    directory.mkdir(parents=True, exist_ok=True)
    for kind in ("edges", "adc"):
        rows = []
        for c in report["captures"]:
            for row in sorted(c[kind], key=lambda r: r["row"]):
                r = dict(row)
                r["relative_us"] = relative_us(r.get("adc_us", r.get("us")), c["header"]["start_us"])
                if "counts" in r: r["left"], r["right"] = r.pop("counts")
                rows.append(r)
        if rows:
            with (directory / f"{kind}.csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--merge-repeats", action="store_true", help="assert all logs/dumps are the SAME powered RAM run; union agreeing rows")
    parser.add_argument("--json", type=Path)
    parser.add_argument("--csv-dir", type=Path)
    args = parser.parse_args()
    report = analyze([p.read_text(errors="replace") for p in args.logs], args.merge_repeats, [str(p) for p in args.logs])
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2)+"\n")
    if args.csv_dir: export_csv(report, args.csv_dir)
    print(f"dumps={report['dump_count']} capture_delivery_complete={report['complete']} merged={args.merge_repeats}")
    for c in report["captures"]:
        h = c["header"]
        print(f"epoch={h['epoch']} index={h['index']} reason={h['reason']} edges={len(c['edges'])}/{h['edge']} ADC={len(c['adc'])}/{h['pre']+h['adc']} missing={c['missing_adc']}")
        if c["star"]: print(json.dumps(c["star"], sort_keys=True))
    for error in report["errors"]: print(error, file=sys.stderr)
    return 0 if report["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
