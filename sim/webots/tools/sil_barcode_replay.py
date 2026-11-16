#!/usr/bin/env python3
"""C component comparison only; never starts Webots or hardware."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.sil_barcode_replay import compare

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--capture-reference',action='store_true')
    args=p.parse_args();result=compare(args.project,args.output,capture_reference=args.capture_reference)
    for label,events in result['results'].items():
        for e in events:print(label,e['sim_ms'],e['reason_name'],e['letter_name'],e['motion_truth'])
