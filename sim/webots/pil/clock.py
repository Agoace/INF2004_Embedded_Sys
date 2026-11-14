"""Conservative fake-clock-testable PIL bounds; not hardware qualification.

Times are integer microseconds. Bounds never use assumed USB delay or sqrt(N).
A fixed apply epoch is not changed by updating the PC/Pico clock estimate.
"""

from dataclasses import dataclass

from .wire import MAX_SOURCE_US


# Gate1 UART0 8N1, CLK_PERI125MHz, IBRD8/FBRD31. Count only33
# complete request chars and41 reply chars: do NOT credit the final stop bit.
# Causal stamps, PL011 tolerance/100ppm and extra1us margin are proved in
# docs/simulation/WEBOTS_PIL.md#gate-1-proven-directional-minima.
# CONDITIONAL until stopped D1 proves zero bad/UART errors and this divisor.
GATE1_MAX_REQUEST_BAUD = 985000
GATE1_MAX_REPLY_BAUD = 921000
GATE1_PROVEN_MIN_TX_US = 33 * 10_000_000 // GATE1_MAX_REQUEST_BAUD - 1  #334us
GATE1_PROVEN_MIN_RX_US = 41 * 10_000_000 // GATE1_MAX_REPLY_BAUD - 1  #444us


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
        # Exact offset cones in millionths of a microsecond. All lower cones
        # have slope-rho; upper cones slope+rho, so two intercepts retain ALL
        # SYNC constraints in O(1) space. Never age a previously rounded bound.
        self._lower_intercept = self._upper_intercept = None
        self._rho = None

    def _interval_at(self, pc_us):
        scale = 1_000_000
        lower = (self._lower_intercept - self._rho * pc_us) // scale
        upper = -(-(self._upper_intercept + self._rho * pc_us) // scale)
        return Interval(lower, upper)

    def bounds_at(self, pc_us, *, qualified=True):
        if type(pc_us) is not int or self.interval is None:
            raise ClockError("clock has no observations")
        age = pc_us - self.updated_pc_us
        if age < 0:
            raise ClockError("PC monotonic time moved backwards")
        result = self._interval_at(pc_us)
        if qualified and (self.observations < 8 or age > self.MAX_REFRESH_US or
                          result.halfwidth > self.MAX_HALFWIDTH_US):
            raise ClockError("clock not qualified or uncertainty headroom expired")
        return result

    def observe(self, host_tx_us, host_rx_us, pico_rx32, pico_tx32,
                *, proven_min_tx_us=0, proven_min_rx_us=0):
        times = (host_tx_us, host_rx_us, proven_min_tx_us, proven_min_rx_us)
        if any(type(x) is not int or x < 0 for x in times) or host_rx_us < host_tx_us:
            raise ClockError("invalid timestamp observation")
        if proven_min_tx_us + proven_min_rx_us > host_rx_us - host_tx_us:
            raise ClockError('proven directional minima exceed observed round trip')
        if self.updated_pc_us is not None and host_rx_us < self.updated_pc_us:
            raise ClockError("SYNC receive time moved backwards")
        anchor = pico_rx32 if self.pico_anchor is None else self.pico_anchor
        receive = unwrap32(pico_rx32, anchor)
        transmit = unwrap32(pico_tx32, receive)
        if transmit < receive:
            raise ClockError("Pico time moved backwards")
        # Optional minima need separate hardware/baud proof. Defaults subtract NOTHING.
        rho, scale = self.DRIFT_PPM, 1_000_000
        if type(rho) is not int or rho < 0 or self._rho is not None and rho != self._rho:
            raise ClockError('clock drift assumption changed or invalid')
        growth = (host_rx_us - host_tx_us + 1) * rho
        # Floor-to-us quantization remains a distinct outward1us. Preserve
        # fractional drift internally, then round ONCE at the requested time.
        lower = (transmit - host_rx_us + proven_min_rx_us - 1) * scale - growth + rho * host_rx_us
        upper = (receive - host_tx_us - proven_min_tx_us + 1) * scale + growth - rho * host_rx_us
        if self._lower_intercept is not None:
            lower = max(lower, self._lower_intercept)
            upper = min(upper, self._upper_intercept)
        if lower - rho * host_rx_us > upper + rho * host_rx_us:
            raise ClockError("empty clock-offset intersection")
        self._lower_intercept, self._upper_intercept, self._rho = lower, upper, rho
        result = self._interval_at(host_rx_us)
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
        if self.interval.halfwidth > self.MAX_HALFWIDTH_US:
            return self.updated_pc_us
        # Exact integer-time deadline, including outward rounding. Width is
        # monotone between observations. Rate history gives NO future credit:
        # instantaneous skew may change anywhere inside the outer100ppm guard.
        lo, hi = 0, self.MAX_REFRESH_US
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self._interval_at(self.updated_pc_us + mid).halfwidth <= self.MAX_HALFWIDTH_US:
                lo = mid
            else:
                hi = mid - 1
        return self.updated_pc_us + lo


@dataclass(frozen=True)
class Timeline:
    epoch_pico_us: int

    def apply(self, source_us):
        if type(source_us) is not int or not 0 <= source_us <= MAX_SOURCE_US or source_us % 1000:
            raise ClockError("invalid source step")
        return self.epoch_pico_us + source_us

    def source_target(self, source_us):
        return self.apply(source_us) - 4000

    def validate_capture(self, source_us, captured_pc_us, clock, capture_end_pc_us=None):
        end = captured_pc_us if capture_end_pc_us is None else capture_end_pc_us
        if type(end) is not int or end < captured_pc_us:
            raise ClockError('invalid source capture bracket')
        first, last = clock.pico_range(captured_pc_us), clock.pico_range(end)
        # Bound pacing across the actual capture bracket, separate from offset
        # uncertainty. Include the upper endpoint's floor-to-us quantization.
        target = self.source_target(source_us)
        phase = max(abs(first.midpoint - target), abs(last.midpoint + 1 - target))
        if phase > 250:
            error = ClockError("source phase error exceeds250us")
            error.capture_rejection = {
                'source_us': source_us, 'intended_pico_us': target,
                'capture_start_pc_us': captured_pc_us, 'capture_end_pc_us': end,
                'start_lower_pico_us': first.lower, 'start_upper_pico_us': first.upper,
                'end_lower_pico_us': last.lower, 'end_upper_pico_us': last.upper,
                'start_midpoint_error_us': first.midpoint-target,
                'end_midpoint_error_plus_quantization_us': last.midpoint+1-target,
                'start_phase_us': abs(first.midpoint-target),
                'end_phase_us': abs(last.midpoint+1-target),
                'endpoint_quantization_us': 1,
                'phase_us': phase, 'limit_us': 250, 'excess_us': phase-250,
                'uncertainty_us': max(first.halfwidth, last.halfwidth),
                'last_sync_pc_us': clock.updated_pc_us,
                'reason': str(error)}
            raise error
        return phase, max(first.halfwidth, last.halfwidth)

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
