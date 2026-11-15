#!/usr/bin/env python3
"""No-device C4 CPU benchmark: original bitwise versus table CRC32C."""
import json
from pathlib import Path
import random
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.wire import crc32c


def bitwise(data):
    crc=0xffffffff
    for byte in data:
        crc^=byte
        for _ in range(8):crc=(crc>>1)^(0x82f63b78 if crc&1 else 0)
    return crc^0xffffffff


def main():
    rng=random.Random(20261115);results=[]
    for size in(0,24,56,68,96):
        data=bytes(rng.randrange(256)for _ in range(size));samples={}
        assert crc32c(data)==bitwise(data)
        for name,fn in(('bitwise',bitwise),('table',crc32c)):
            values=[]
            for _ in range(10):
                start=time.thread_time_ns()
                for _ in range(1000):fn(data)
                values.append((time.thread_time_ns()-start)/1000)
            samples[name]={'median_cpu_ns_per_call':statistics.median(values),
                           'max_batch_mean_cpu_ns_per_call':max(values)}
        results.append(dict(size=size,**samples))
    print(json.dumps(dict(scope='CPU microbenchmark only; no hardware, no future bound',
                         calls_per_algorithm_per_size=10000,results=results),indent=2))

if __name__=='__main__':main()
