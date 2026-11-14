#!/usr/bin/env python3
"""No-device whole-call observer microbenchmark, not a scheduling bound."""
import json
from pathlib import Path
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.gate1 import RealClock
from pil.gate1_host_observer import HostSpans
from pil.gate1_trial import labels


def main():
    c=RealClock();s=HostSpans(c);samples=[];baseline=[]
    for _ in range(20000):
        a=time.perf_counter_ns();c.now_us();baseline.append(time.perf_counter_ns()-a)
        a=time.perf_counter_ns();token=s.begin('select');s.end(token)
        samples.append(time.perf_counter_ns()-a)
    def stats(v):
        v=sorted(v)
        return dict(p50_ns=statistics.median(v),p95_ns=v[int(len(v)*.95)-1],
                    p99_ns=v[int(len(v)*.99)-1],max_ns=max(v))
    print(json.dumps(dict(labels(True),samples=len(samples),whole_span=stats(samples),
        single_raw_clock_baseline=stats(baseline),
        scope='microbenchmark only; includes begin/end/sample writes, no syscall body; not a hard bound'),indent=2))

if __name__=='__main__':main()
