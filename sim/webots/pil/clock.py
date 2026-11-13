"""Conservative fake-clock-testable PIL bounds; not hardware qualification.

Times are integer microseconds. Bounds never use assumed USB delay or sqrt(N).
A fixed apply epoch is not changed by updating the PC/Pico clock estimate.
"""

from dataclasses import dataclass
import math

from .wire import MAX_SOURCE_US


class ClockError(ValueError):
    pass


def unwrap32(value, near):
    if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
        raise ClockError("expected uint32 time")
    return near + ((value - near + (1 << 31)) % (1 << 32) - (1 << 31))


@dataclass(frozen=True)
class Interval:
    lower: int
    upper: int

    @property
    def midpoint(self):
        return (self.lower + self.upper) / 2

    @property
    def halfwidth(self):
        return (self.upper - self.lower) / 2


class ClockBounds:
    MAX_HALFWIDTH_US = 250
    MAX_REFRESH_US = 1_000_000
    DRIFT_PPM = 100

    def __init__(self):
        self.interval = None
        self.updated_pc_us = None
        self.pico_anchor = None
        self.observations = 0

    def _growth(self, elapsed):
        return (elapsed * self.DRIFT_PPM + 999_999) // 1_000_000

    def bounds_at(self, pc_us, *, qualified=True):
        if type(pc_us) is not int or self.interval is None:
            raise ClockError("clock has no observations")
        age = pc_us - self.updated_pc_us
        if age < 0:
            raise ClockError("PC monotonic time moved backwards")
        growth = self._growth(age)
        result = Interval(self.interval.lower - growth, self.interval.upper + growth)
        if qualified and (self.observations < 8 or age > self.MAX_REFRESH_US or
                          result.halfwidth > self.MAX_HALFWIDTH_US):
            raise ClockError("clock not qualified or uncertainty headroom expired")
        return result

    def observe(self, host_tx_us, host_rx_us, pico_rx32, pico_tx32,
                *, proven_min_tx_us=0, proven_min_rx_us=0):
        times = (host_tx_us, host_rx_us, proven_min_tx_us, proven_min_rx_us)
        if any(type(x) is not int or x < 0 for x in times) or host_rx_us < host_tx_us:
            raise ClockError("invalid timestamp observation")
        if self.updated_pc_us is not None and host_rx_us < self.updated_pc_us:
            raise ClockError("SYNC receive time moved backwards")
        anchor = pico_rx32 if self.pico_anchor is None else self.pico_anchor
        receive = unwrap32(pico_rx32, anchor)
        transmit = unwrap32(pico_tx32, receive)
        if transmit < receive:
            raise ClockError("Pico time moved backwards")
        # Optional minima need separate hardware/baud proof. Defaults subtract NOTHING.
        observation_growth = self._growth(host_rx_us - host_tx_us)
        result = Interval(transmit - host_rx_us + proven_min_rx_us - observation_growth,
                          receive - host_tx_us - proven_min_tx_us + observation_growth)
        if self.interval is not None:
            previous = self.bounds_at(host_rx_us, qualified=False)
            result = Interval(max(previous.lower, result.lower),
                              min(previous.upper, result.upper))
        if result.lower > result.upper:
            raise ClockError("empty clock-offset intersection")
        self.interval, self.updated_pc_us = result, host_rx_us
        self.pico_anchor = transmit
        self.observations += 1
        return result

    def pico_range(self, pc_us):
        offset = self.bounds_at(pc_us)
        return Interval(pc_us + offset.lower, pc_us + offset.upper)

    def refresh_deadline_pc_us(self):
        if self.interval is None:
            raise ClockError("clock uninitialized")
        headroom = self.MAX_HALFWIDTH_US - self.interval.halfwidth
        if headroom < 0:
            return self.updated_pc_us
        # Round down: this is a deadline, not a promise that a late reply is safe.
        lifetime = math.floor(headroom) * 1_000_000 // self.DRIFT_PPM
        return self.updated_pc_us + min(self.MAX_REFRESH_US, lifetime)


@dataclass(frozen=True)
class Timeline:
    epoch_pico_us: int

    def apply(self, source_us):
        if type(source_us) is not int or not 0 <= source_us <= MAX_SOURCE_US or source_us % 1000:
            raise ClockError("invalid source step")
        return self.epoch_pico_us + source_us

    def source_target(self, source_us):
        return self.apply(source_us) - 4000

    def validate_capture(self, source_us, captured_pc_us, clock):
        interval = clock.pico_range(captured_pc_us)
        # Midpoint phase error and uncertainty are recorded SEPARATELY, then added.
        phase = abs(interval.midpoint - self.source_target(source_us))
        if phase > 250:
            raise ClockError("source phase error exceeds250us")
        return phase, interval.halfwidth

    def validate_available(self, source_us, captured_pc_us, available_pico_us, clock):
        """Reference scheduler check using actual fake/device availability time.

        The PC cannot infer a real Pico RX timestamp from a successful write.
        Gate1 needs device diagnostics; this routine is NOT a UART latency proof.
        """
        interval = clock.pico_range(captured_pc_us)
        if available_pico_us < interval.lower:
            raise ClockError("availability predates source")
        oldest_age = available_pico_us - interval.lower
        if available_pico_us > self.apply(source_us) - 1000 or oldest_age > 3000:
            raise ClockError("endpoint missed preceding-slot lookahead deadline")
        return oldest_age
