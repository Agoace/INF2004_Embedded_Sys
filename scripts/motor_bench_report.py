#!/usr/bin/env python3
"""Report for a motor_ctl_bench run (docs/s2-motion/MOTION.md, floor bench).

Usage: python3 scripts/motor_bench_report.py build/steer_logs/text_<boot>.txt

Per speed step and wheel: time to 90 % (50 ms average), peak, overshoot,
ripple and mean speed over the second half, duty range; then the steady
(speed, duty) points and a straight-line feed-forward fit per wheel and
direction (counts per 10 ms tick, permille).
"""
import sys
rows={}
for l in open(sys.argv[1]):
    l=l.strip()
    if l.startswith("pico> "): l=l[6:]
    if not l.startswith("MB "): continue
    v=l.split()[1:]
    if len(v)!=11: continue
    try: v=list(map(int,v))
    except: continue
    rows[v[0]]=v
idx=sorted(rows); print("rows",len(idx),"missing",[i for i in range(idx[0],idx[-1]+1) if i not in rows][:20])
names=["rest","fwd 150","rest","rev 150","rest","fwd 300","rest","rev 300","rest","spin L 20","rest","spin R 20","rest","fwd 40","rest","rev 40","rest"]
from collections import defaultdict
seg=defaultdict(list)
for i in idx: seg[rows[i][1]].append(rows[i])
def ma(xs,n=5):
    return [sum(xs[max(0,k-n+1):k+1])/len(xs[max(0,k-n+1):k+1]) for k in range(len(xs))]
print("step       W  tgt  t90ms  peak  ovs%  sd2nd  mean2nd  duty2nd")
for s in sorted(seg):
    r=seg[s]
    if names[s]=="rest": continue
    for w,(ti,si,di) in enumerate([(2,4,6),(3,5,7)]):
        tgt=r[0][ti]; sp=[x[si] for x in r]; du=[x[di] for x in r]
        m=ma(sp); sg=1 if tgt>0 else -1
        t90=next((k*10 for k,x in enumerate(m) if x*sg>=0.9*abs(tgt)),None)
        peak=max(x*sg for x in m)
        h=len(r)//2; sec=m[h:]; mean=sum(sec)/len(sec); sd=(sum((x-mean)**2 for x in sec)/len(sec))**.5
        print(f"{names[s]:10s} {'LR'[w]}  {tgt:+4d}  {t90}  {peak:5.0f}  {100*(peak-abs(tgt))/abs(tgt):4.0f}  {sd:5.1f}  {mean:+6.0f}  {min(du[h:]):+4d}..{max(du[h:]):+4d}")
print("\nsteady points (2nd half mean speed cpt, mean duty) per wheel/dir:")
pts={(w,d):[] for w in (0,1) for d in (0,1)}
for s in sorted(seg):
    r=seg[s]
    if names[s]=="rest": continue
    h=len(r)//2
    for w in (0,1):
        sp=sum(x[4+w] for x in r[h:])/len(r[h:]); du=sum(x[6+w] for x in r[h:])/len(r[h:])
        cpt=abs(sp)*12414*0.01/1000
        d=0 if sp>0 else 1
        pts[(w,d)].append((cpt,abs(du),names[s]))
for k,p in pts.items():
    n=len(p); sx=sum(a for a,_,_ in p); sy=sum(b for _,b,_ in p); sxx=sum(a*a for a,_,_ in p); sxy=sum(a*b for a,b,_ in p)
    sl=(n*sxy-sx*sy)/(n*sxx-sx*sx); off=(sy-sl*sx)/n
    print(["L","R"][k[0]],["fwd","rev"][k[1]],[(round(a,2),round(b),c) for a,b,c in p],f"fit offset {off:.0f} slope {sl:.2f} (q8 {sl*256:.0f})")
