"""Single-producer bounded evidence queue. File work belongs to the writer.

CPython deque append/popleft are atomic; the producer never waits on a lock,
condition or full queue. A rejected submission has its own reserved slot.
"""
from collections import deque
from dataclasses import dataclass
import csv
import json
import threading
import time


class EvidenceFailure(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class Fields:
    items: tuple


def freeze(value):
    if isinstance(value, dict):
        return Fields(tuple((str(k), freeze(v)) for k, v in value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(freeze(v) for v in value)
    if isinstance(value, bytearray):
        return bytes(value)
    if value is None or isinstance(value, (str, bytes, int, float, bool)):
        return value
    raise EvidenceFailure('unsupported evidence snapshot type')


def thaw(value):
    if isinstance(value, Fields):
        return {k: thaw(v) for k, v in value.items}
    if isinstance(value, tuple):
        return [thaw(v) for v in value]
    if isinstance(value, bytes):
        return value.hex()
    return value


@dataclass(frozen=True, slots=True)
class Snapshot:
    event: str
    pc_us: object
    details: object
    loop: int
    flush: bool = False
    serial: int = 0


class QueuedEventWriter:
    def __init__(self, path, capacity=8192, start=True, reserve_capacity=4096):
        if capacity < 1 or reserve_capacity < 1:
            raise ValueError('invalid evidence queue capacity')
        self.stream = path.open('x', newline='')
        self.csv = csv.writer(self.stream)
        self.capacity, self.reserve_capacity = capacity, reserve_capacity
        self.queue, self.reserve = deque(), []
        self.failed_record = self.fault_record = None
        self.failure = self.worker_error = None
        self.accepting, self.stopping, self.close_ack = True, False, False
        self.submitted = self.written = self.reserve_overflows = 0
        self.attribution = None
        self.started = False
        self.dirty = False
        self.thread = threading.Thread(target=self._run, name='gate1-evidence-writer', daemon=True)
        if start:
            try:
                self.start()
            except BaseException:
                self.stream.close()
                raise

    def start(self):
        if not self.started:
            self.started = True
            self.thread.start()

    def _submit(self, snapshot):
        self.submitted += 1
        if self.stopping:
            # This point record cannot be displaced by ordinary stop traffic.
            if snapshot.event == 'source_capture_rejected' and self.fault_record is None:
                self.fault_record = snapshot
            elif len(self.reserve) < self.reserve_capacity:
                self.reserve.append(snapshot)
            else:
                self.reserve_overflows += 1
                self.failure = self.failure or 'reserved stop evidence queue full'
                raise EvidenceFailure(self.failure)
            return
        if self.failure or self.worker_error or len(self.queue) >= self.capacity:
            self.failure = self.failure or self.worker_error or 'evidence queue full'
            if self.failed_record is None:
                self.failed_record = snapshot
            else:
                self.reserve_overflows += 1
            raise EvidenceFailure(self.failure)
        self.queue.append(snapshot)

    def record(self, event, pc_us, details, loop=0):
        self._submit(Snapshot(event, pc_us, freeze(details), loop, serial=self.submitted))
        self.dirty = True

    def flush(self, loop=0):
        if self.failure or self.worker_error:
            if not self.stopping:
                raise EvidenceFailure(self.failure or self.worker_error)
        if self.dirty:
            self._submit(Snapshot('', None, None, loop, flush=True, serial=self.submitted))
            self.dirty = False

    def begin_stop(self):
        self.stopping = True

    def _mark(self, name, loop):
        ring = self.attribution
        if ring is not None:
            ring.mark(name, loop=loop)

    def _emit(self, snapshot):
        if snapshot.flush:
            self._mark('flush_enter', snapshot.loop)
            try:
                self.stream.flush()
            finally:
                self._mark('flush_exit', snapshot.loop)
        else:
            self._mark('serialize_enter', snapshot.loop)
            try:
                self.csv.writerow((snapshot.event, snapshot.pc_us,
                                   json.dumps(thaw(snapshot.details), sort_keys=True)))
                self.written += 1
            finally:
                self._mark('serialize_exit', snapshot.loop)

    def _run(self):
        try:
            self.csv.writerow(('event', 'pc_us', 'details_json'))
            batch = 0
            while True:
                try:
                    snapshot = self.queue.popleft()
                except IndexError:
                    if not self.accepting:
                        break
                    time.sleep(.001)
                    continue
                self._emit(snapshot)
                batch += 1
                if batch == 4:
                    # Give the control thread a chance; no priority policy change.
                    time.sleep(0)
                    batch = 0
            if self.failed_record is not None:
                self._emit(self.failed_record)
            tail = list(self.reserve)
            if self.fault_record is not None:
                tail.append(self.fault_record)
            for snapshot in sorted(tail, key=lambda item: item.serial):
                self._emit(snapshot)
            self._emit(Snapshot('', None, None, 0, flush=True))
            self.stream.close()
            self.close_ack = True
        except BaseException as error:
            self.worker_error = type(error).__name__+': '+str(error)
            try:
                self.stream.close()
            except BaseException:
                pass

    def finish(self, timeout=5):
        self.begin_stop()
        self.accepting = False
        self.start()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.failure = self.failure or 'evidence writer close acknowledgement timeout'
        return self.status()

    def status(self):
        return {'schema': 1, 'capacity': self.capacity,
                'reserve_capacity': self.reserve_capacity,
                'failure': self.failure, 'worker_error': self.worker_error,
                'close_ack': self.close_ack, 'writer_alive': self.thread.is_alive(),
                'submitted': self.submitted, 'rows_written': self.written,
                'pending_snapshots': len(self.queue), 'reserved_snapshots': len(self.reserve),
                'failed_submission_retained': self.failed_record is not None,
                'reserved_overflows': self.reserve_overflows}
