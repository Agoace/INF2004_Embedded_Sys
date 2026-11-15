"""Bounded numeric diagnostic tail. Never a clock input or safety credit."""
from array import array
import csv
import gc
import json


class AttributionRing:
    KINDS = ('loop', 'sleep_enter', 'wake', 'read_enter', 'read_exit',
             'receive_exit', 'drain_exit', 'sync_exit', 'tick_exit',
             'capture_enter', 'capture_exit', 'serialize_enter',
             'serialize_exit', 'flush_enter', 'flush_exit', 'gc_start', 'gc_stop')

    def __init__(self, clock, capacity=65536):
        if type(capacity) is not int or capacity < 1:
            raise ValueError('invalid attribution capacity')
        self.clock, self.capacity = clock, capacity
        self.data = array('q', [0]) * (capacity*5)
        self.total = self.overwritten = self.loop = 0
        self.cost_count = self.cost_sum = self.cost_max = self.nested = 0
        self.writing = False
        self.installed = False
        self.qualification = None

    def mark(self, kind, loop=None, value=0):
        if self.writing:
            self.nested += 1
            return
        self.writing = True
        try:
            before = self.clock.now_us()
            pos = (self.total % self.capacity)*5
            self.data[pos] = self.total
            self.data[pos+1] = self.KINDS.index(kind)
            self.data[pos+2] = self.loop if loop is None else loop
            self.data[pos+3] = before
            self.data[pos+4] = value
            self.total += 1
            self.overwritten = max(0, self.total-self.capacity)
            cost = max(0, self.clock.now_us()-before)
            self.cost_count += 1
            self.cost_sum += cost
            self.cost_max = max(self.cost_max, cost)
        finally:
            self.writing = False

    def callback(self, phase, info):
        self.mark('gc_start' if phase == 'start' else 'gc_stop',
                  value=info.get('generation', -1))

    def install(self):
        gc.callbacks.append(self.callback)
        self.installed = True

    def detach(self):
        if self.installed:
            gc.callbacks.remove(self.callback)
            self.installed = False

    def finish(self, directory, *, serial_closed=True):
        self.detach()
        with (directory/'attribution.csv').open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('serial', 'kind', 'loop', 'pc_us', 'value') +
                            (('qualification',) if self.qualification else ()))
            for serial in range(max(0, self.total-self.capacity), self.total):
                pos = (serial % self.capacity)*5
                row = self.data[pos:pos+5]
                writer.writerow((row[0], self.KINDS[row[1]], row[2], row[3], row[4]) +
                                ((self.qualification,) if self.qualification else ()))
        meta = {'schema': 1, 'pc_clock_domain': getattr(self.clock, 'domain', 'INJECTED_TEST_CLOCK'),
                'scope': 'diagnostic tail only; not full-run evidence or safety credit',
                'capacity_rows': self.capacity, 'total_rows': self.total,
                'overwritten_rows': self.overwritten, 'nested_callbacks_not_recorded': self.nested,
                'serialized_after_serial_close': serial_closed,
                'observer_cost': {'sample_count': self.cost_count,
                    'array_bracket_sum_us': self.cost_sum, 'array_bracket_max_us': self.cost_max,
                    'scope': 'RAW/us-quantized internal array bracket; excludes first clock call and cost counters; whole-call benchmark required'}}
        if getattr(self, 'trial_tolerate_late', False):
            from .gate1_trial import labels
            meta.update(labels(True, True))
        if self.qualification:
            meta.update(qualification=self.qualification, trial_clock_400_sync20=True,
                        physical_gate1_pass=False)
        (directory/'attribution.json').write_text(json.dumps(meta, indent=2)+'\n')
