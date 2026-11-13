"""Duty-driven plant; no navigation, firmware PI or firmware rise-limit copy.

Steady speeds are DM1 loaded VIN observations (7a65c0e, boot5d51).
80--100 percent observations had wheelies/unsettled speed: excluded. Dynamics,
static breakaway, torque and mass remain profile assumptions, not DM1 findings.
"""

import math

from .robot_profile import RobotProfile

DM1_SOURCE = "docs/archive/s2-motion/DUTY_MAP.md#run-dm1"
DUTIES = (0, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 18, 20, 25, 30, 35, 40, 45, 50, 60, 70)
SPEEDS_MM_S = (
    ((0, 0, 0, 0, 24, 34, 43, 52, 60, 78, 96, 114, 129, 168, 211, 253, 296,
      332, 376, 464, 561),
     (0, 0, 0, 4, 29, 39, 48, 56, 65, 82, 98, 116, 131, 171, 214, 256, 299,
      335, 379, 463, 548)),
    ((0, 0, 0, 0, 24, 33, 42, 51, 59, 77, 95, 114, 129, 171, 214, 257, 299,
      334, 378, 463, 554),
     (0, 0, 0, 1, 26, 36, 45, 53, 63, 80, 99, 117, 134, 175, 217, 262, 304,
      342, 380, 462, 543)),
)


def round_away(value):
    if not math.isfinite(value):
        raise ValueError("nonfinite sample")
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def signed32(value):
    return (value + (1 << 31)) % (1 << 32) - (1 << 31)


def steady_speed_mm_s(wheel, duty, *, assumed_limit=0.7):
    """Interpolate measured RUNNING speeds, with explicitly opted-in extrapolation.

    Above70% is NOT measured calibration: extension uses the60--70% slope only
    when a profile explicitly declares its assumed_limit. Never use wheelie rows.
    """
    if type(wheel) is not int or wheel not in (0, 1) or not math.isfinite(duty):
        raise ValueError("invalid wheel/duty")
    percent = abs(duty) * 100
    if not math.isfinite(assumed_limit) or not 0.7 <= assumed_limit <= 1:
        raise ValueError("invalid assumed high-duty limit")
    if percent > assumed_limit * 100 + 1e-10:
        raise ValueError("duty exceeds declared model support")
    curve = SPEEDS_MM_S[wheel][1 if duty < 0 else 0]
    if percent > 70:
        speed = curve[-1] + (percent - 70) * (curve[-1] - curve[-2]) / 10
        return math.copysign(speed, duty)
    for i in range(1, len(DUTIES)):
        if percent <= DUTIES[i] + 1e-10:
            fraction = (percent - DUTIES[i - 1]) / (DUTIES[i] - DUTIES[i - 1])
            speed = curve[i - 1] + fraction * (curve[i] - curve[i - 1])
            return math.copysign(speed, duty)
    raise ValueError("unsupported duty")


class MotorPlant:
    """Produce torque for Webots force control, never set ideal wheel velocity.

    Assumed reflected inertia includes half the body load. At zero drive the
    damping is PASSIVE coast, not a requested electrical brake. Static breakaway
    threshold is separate from the measured after-kick running map.
    """

    def __init__(self, profile: RobotProfile, wheel: int):
        if type(wheel) is not int or wheel not in (0, 1):
            raise ValueError("wheel must be0/1")
        self.profile, self.wheel = profile, wheel
        self.radius_m = profile["wheel_diameter_mm"] / 2000
        self.reflected_inertia = (profile["wheel_inertia_kg_m2"] +
                                  profile["total_mass_kg"] * self.radius_m ** 2 / 2)

    def torque_nm(self, compare: int, omega_rad_s: float) -> float:
        if type(compare) is not int or not -6250 <= compare <= 6250:
            raise ValueError("PWM compare out of range")
        if not math.isfinite(omega_rad_s):
            raise ValueError("wheel velocity must be finite")
        duty = compare / 6250
        target = steady_speed_mm_s(
            self.wheel, duty, assumed_limit=self.profile["max_assumed_duty_fraction"]
        ) / (1000 * self.radius_m)
        stopped = abs(omega_rad_s) < self.profile["stopped_speed_rad_s"]
        if stopped and abs(duty) < self.profile["start_duty_fraction"]:
            target = 0.0
        tau = self.profile["coast_tau_ms" if compare == 0 else "drive_tau_ms"] / 1000
        torque = self.reflected_inertia * (target - omega_rad_s) / tau
        limit = self.profile["stall_torque_nm"]
        return max(-limit, min(limit, torque))


class EncoderAccumulator:
    """Absolute joint angle to counts. Initial half-count phase is an assumption.

    Nearest/ties-away quantization retains fractional phase without per-step
    rounding drift. Wheel slip does not alter this wheel-rotation observation.
    """

    def __init__(self, counts_per_revolution=2535):
        if type(counts_per_revolution) is not int or counts_per_revolution <= 0:
            raise ValueError("invalid encoder ratio")
        self.ratio = counts_per_revolution / (2 * math.pi)
        self.origins = None
        self.previous = (0, 0)

    def reset(self, angles_rad):
        values = tuple(float(x) for x in angles_rad)
        if len(values) != 2 or not all(math.isfinite(x) for x in values):
            raise ValueError("two finite joint angles required")
        self.origins = values
        self.previous = (0, 0)

    def sample(self, angles_rad):
        if self.origins is None:
            raise ValueError("reset at run origin before sampling")
        values = tuple(float(x) for x in angles_rad)
        if len(values) != 2 or not all(math.isfinite(x) for x in values):
            raise ValueError("two finite joint angles required")
        counts = tuple(round_away((x - origin) * self.ratio)
                       for x, origin in zip(values, self.origins))
        if any(abs(x - previous) > 16 for x, previous in zip(counts, self.previous)):
            raise ValueError("encoder endpoint exceeds16 counts/slot")
        self.previous = counts
        return tuple(signed32(x) for x in counts)


class ServoPlant:
    def __init__(self, slew_deg_s):
        if not math.isfinite(slew_deg_s) or slew_deg_s <= 0:
            raise ValueError("positive finite servo slew required")
        self.angle_deg = 0.0
        self.pulse_us = 1500
        self.slew_deg_s = slew_deg_s

    def set_pulse(self, pulse_us):
        if type(pulse_us) is not int or not 1100 <= pulse_us <= 1900:
            raise ValueError("servo pulse outside mounted clamp")
        self.pulse_us = pulse_us

    def step(self, dt_s):
        if not math.isfinite(dt_s) or not 0 < dt_s <= 0.001:
            raise ValueError("servo model steps must be <=1ms")
        target = (self.pulse_us - 1500) * 36 / 400
        error = target - self.angle_deg
        increment = self.slew_deg_s * dt_s
        self.angle_deg += max(-increment, min(increment, error))
        return self.angle_deg
