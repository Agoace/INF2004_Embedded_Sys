"""Opt-in host diagnostics. No scheduler/power setters; never estimator input."""
from array import array
import csv
import json
import os
from pathlib import Path
import platform
import resource
import threading
import time


class HostSpans:
    KINDS = ('select', 'read', 'sleep', 'backend_read')
    FIELDS = ('serial','kind','loop','raw_start_us','raw_end_us','wall_us',
              'thread_cpu_ns','nvcsw','nivcsw','minflt','majflt')
    def __init__(self,clock,capacity=32768,*,cpu=time.thread_time_ns,usage=None,labels=None):
        if type(capacity)is not int or capacity<1:raise ValueError('invalid span capacity')
        self.clock,self.capacity,self.cpu=clock,capacity,cpu
        self.usage=usage or (lambda:resource.getrusage(resource.RUSAGE_THREAD))
        self.data=array('q',[0])*(capacity*11)
        self.total=self.overwritten=0
        self.labels=labels or {}
        self.errors=0

    def counters(self):
        try:
            r=self.usage()
            return (r.ru_nvcsw,r.ru_nivcsw,r.ru_minflt,r.ru_majflt)
        except (AttributeError,OSError,ValueError):
            self.errors+=1
            return (-1,)*4

    def begin(self,kind,loop=0):
        # CPU/rusage enclose the RAW wall bracket; their sampling costs are not
        # subtracted or used for timing credit. No procfs or serialization here.
        counts=self.counters();cpu=self.cpu();raw=self.clock.now_us()
        return (self.KINDS.index(kind),loop,raw,cpu,counts)

    def end(self,start):
        raw=self.clock.now_us();cpu=self.cpu();counts=self.counters()
        kind,loop,first,cpu_first,old=start
        delta=tuple(b-a if a>=0 and b>=a else -1 for a,b in zip(old,counts))
        values=(self.total,kind,loop,first,raw,raw-first,cpu-cpu_first,*delta)
        pos=(self.total%self.capacity)*11
        for i,v in enumerate(values):self.data[pos+i]=v
        self.total+=1;self.overwritten=max(0,self.total-self.capacity)

    def finish(self,directory,*,serial_closed=True):
        with (directory/'host-spans.csv').open('x',newline='')as f:
            w=csv.writer(f);w.writerow(self.FIELDS+('qualification',))
            for serial in range(max(0,self.total-self.capacity),self.total):
                p=(serial%self.capacity)*11;row=list(self.data[p:p+11]);row[1]=self.KINDS[row[1]]
                w.writerow(row+[self.labels.get('qualification','DIAGNOSTIC ONLY')])
        meta=dict(self.labels,schema=1,capacity=self.capacity,total=self.total,
            overwritten=self.overwritten,rusage_errors=self.errors,
            clock='CLOCK_MONOTONIC_RAW for hardware; injected in tests',
            cpu_clock='CLOCK_THREAD_CPUTIME_ID via time.thread_time_ns',
            scope='diagnostic tail only; CPU/rusage enclose wall bracket; no timing credit',
            missing_counter_value=-1,serialized_after_serial_close=serial_closed)
        (directory/'host-spans.json').write_text(json.dumps(meta,indent=2)+'\n')


def _read(path):
    try:
        with Path(path).open()as f:text=f.read(262145)
        return {'status':'KNOWN','text':text[:262144],'truncated':len(text)>262144}
    except OSError as e:return {'status':'UNKNOWN','reason':str(e)}


def host_snapshot():
    """Outside the control loop only. Missing files never mean zero activity."""
    def call(fn):
        try:return {'status':'KNOWN','value':fn()}
        except (AttributeError,OSError,ValueError)as e:return {'status':'UNKNOWN','reason':str(e)}
    result={'schema':1,'pid':os.getpid(),'native_tid':threading.get_native_id(),
        'uname':platform.uname()._asdict(),'python':platform.python_version(),
        'policy':call(lambda:os.sched_getscheduler(0)),
        'nice':call(lambda:os.getpriority(os.PRIO_PROCESS,0)),
        'affinity':call(lambda:sorted(os.sched_getaffinity(0))),
        'files':{p:_read(p) for p in('/proc/thread-self/status','/proc/thread-self/schedstat',
        '/proc/self/cgroup','/proc/interrupts','/proc/softirqs','/proc/sys/kernel/sched_schedstats')}}
    cpu=Path('/sys/devices/system/cpu')
    paths=[]
    policies=sorted((cpu/'cpufreq').glob('policy*'))
    idle=sorted(cpu.glob('cpu[0-9]*/cpuidle/state*'))
    result['power_discovery']={'cpufreq_status':'KNOWN' if policies else 'UNKNOWN',
        'cpuidle_status':'KNOWN' if idle else 'UNKNOWN',
        'policy_count_found':len(policies),'idle_count_found':len(idle),
        'policy_limit':256,'idle_limit':1024,'truncated':len(policies)>256 or len(idle)>1024}
    for p in policies[:256]:
        paths += [p/n for n in ('scaling_driver','scaling_governor','scaling_min_freq',
            'scaling_max_freq','scaling_cur_freq','affected_cpus','related_cpus')]
    paths += [cpu/'cpufreq/boost',cpu/'intel_pstate/no_turbo']
    for p in idle[:1024]:
        paths += [p/n for n in ('name','latency','residency','usage','time','disable')]
    result['power_files']={str(p):_read(p)for p in paths}
    result['power_scope']='bounded discovery; absent cpufreq/cpuidle data unknown; no causal inference'
    try:
        lines=Path('/proc/self/cgroup').read_text().splitlines()
        v2=next(x.split(':',2)[2]for x in lines if x.startswith('0::'))
        if '..'in Path(v2).parts:raise ValueError('invalid cgroup path')
        root=Path('/sys/fs/cgroup')/v2.lstrip('/')
        result['cgroup_cpu']={n:_read(root/n)for n in ('cpu.max','cpu.stat','cpuset.cpus.effective')}
        result['cgroup_scope']='visible direct v2 cgroup only; ancestor/hidden limits unknown'
    except (OSError,StopIteration,ValueError)as e:result['cgroup_cpu']={'status':'UNKNOWN','reason':str(e)}
    return result
