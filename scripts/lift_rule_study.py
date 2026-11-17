#!/usr/bin/env python3
"""Model the lift-rule decision boundary. This is a MODEL, not evidence.

It answers one question before any threshold is changed: for a vertical
acceleration event, which rule fires, and how does that depend on the event
peak and duration?

The model has no measured input. Every profile below is an assumption with a
stated basis. A pass here does not show that a lift is detected or that a hump
does not false stop. Only a recorded accelerometer trace can show that.

Rules modelled, from app_program/subsystem4/imu_safety.h and .c:

  Current lift rule, two paths:
    free fall:   |a| < 400 mg for 2 consecutive observations
    vertical:    |vertical change| > 500 mg for 3 consecutive observations,
                 and |a| more than 200 mg away from 1 g (the near-1 g gate)

  Proposed plus-minus Z rule (direction guard, no magnitude gate):
    |a_z - g_z| > T for 3 consecutive observations, while the horizontal
    acceleration stays small. A tilt is excluded by direction, not by |a|.

Both rules run on 10 ms observations. Each observation is the mean of about
four 400 Hz samples, so a pulse shorter than about 10 ms is attenuated and the
model does not capture that.
"""
import math

G = 9.80665
STEP_MS = 10.0
FREEFALL_MG = 400.0
VERTICAL_MG = 500.0
NEAR_ONE_G_MG = 200.0
OVERTURN_DEG = 75.0
CLEAR_MS = 1000


def proper_accel_mg(vertical_accel_mps2):
    """Measured |a| in mg for a pure vertical kinematic acceleration."""
    return 1000.0 * (G + vertical_accel_mps2) / G


def vertical_change_mg(vertical_accel_mps2):
    """Signed change against the gravity reference, in mg."""
    return 1000.0 * vertical_accel_mps2 / G


def observations(vertical_accel_mps2, duration_ms):
    """Repeat one acceleration level for a number of 10 ms observations."""
    return [vertical_accel_mps2] * max(1, int(round(duration_ms / STEP_MS)))


def raised_cosine(height_m, length_m, speed_mps):
    """Vertical acceleration of a wheel following a smooth bump.

    y(t) = h/2 (1 - cos(2*pi*t/T)), T = L/v
    a(t) = h/2 * (2*pi/T)^2 * cos(2*pi*t/T)
    Peak magnitude at t = 0: 2*pi^2*h/T^2.
    The tyre is assumed rigid. Compliance and the foam tile smooth this, so
    the peak here is an upper bound, not a prediction.
    """
    if speed_mps <= 0.0 or length_m <= 0.0:
        return []
    period_s = length_m / speed_mps
    samples = max(2, int(round(period_s * 1000.0 / STEP_MS)))
    peak = 2.0 * math.pi ** 2 * height_m / period_s ** 2
    return [peak * math.cos(2.0 * math.pi * i / samples) for i in range(samples)]


def hand_lift(distance_m, duration_s):
    """Trapezoidal velocity profile: accelerate, cruise, decelerate.

    Peak acceleration = 2*distance/duration^2 for a symmetric profile with
    equal acceleration and deceleration thirds.
    """
    accel = 2.0 * distance_m / duration_s ** 2
    third = max(1, int(round(duration_s * 1000.0 / 3.0 / STEP_MS)))
    return ([accel] * third) + ([0.0] * third) + ([-accel] * third)


def release_drop(fall_ms):
    """Free fall, then a catch. During free fall the measured |a| is 0."""
    fall = [(-G) + 0.0] * max(1, int(round(fall_ms / STEP_MS)))
    catch = [0.0]
    return fall + catch


def fired_current(profile):
    """Number of consecutive observations that satisfy a current-rule path."""
    best = 0
    run = 0
    for accel in profile:
        a_mg = proper_accel_mg(accel)
        v_mg = vertical_change_mg(accel)
        near_one_g = abs(a_mg - 1000.0) <= NEAR_ONE_G_MG
        if a_mg < FREEFALL_MG:
            run += 1
        elif (not near_one_g) and abs(v_mg) > VERTICAL_MG:
            run += 1
        else:
            run = 0
        best = max(best, run)
    return best


def fired_z(profile, threshold_mg):
    """Consecutive observations over a plus-minus Z threshold."""
    best = 0
    run = 0
    for accel in profile:
        if abs(vertical_change_mg(accel)) > threshold_mg:
            run += 1
        else:
            run = 0
        best = max(best, run)
    return best


def report(name, profile, basis):
    peak = max((abs(vertical_change_mg(a)) for a in profile), default=0.0)
    duration = len(profile) * STEP_MS
    cur = fired_current(profile)
    z250 = fired_z(profile, 250.0)
    z300 = fired_z(profile, 300.0)
    z500 = fired_z(profile, 500.0)
    print(f"{name:34s} {peak:8.0f} {duration:7.0f} "
          f"{'FIRE' if cur >= 2 else '-':6s} "
          f"{'FIRE' if z250 >= 3 else '-':6s} "
          f"{'FIRE' if z300 >= 3 else '-':6s} "
          f"{'FIRE' if z500 >= 3 else '-':6s}")
    print(f"    basis: {basis}")


def main():
    print("peak_mg  = largest |vertical change| in the profile")
    print("dur_ms   = profile length")
    print("current  = free fall for 2 observations, or 500 mg vertical for 3")
    print("z250/z300/z500 = plus-minus Z with a direction guard, 3 observations")
    print()
    print(f"{'profile':34s} {'peak_mg':>8s} {'dur_ms':>7s} "
          f"{'current':6s} {'z250':6s} {'z300':6s} {'z500':6s}")

    report("gentle hand lift 0.30 m / 1.0 s", hand_lift(0.30, 1.0),
           "slow lift, consistent with the operator session-2 lift")
    report("brisk hand lift 0.30 m / 0.5 s", hand_lift(0.30, 0.5),
           "faster operator lift")
    report("hard yank 0.30 m / 0.3 s", hand_lift(0.30, 0.3),
           "fast yank; upper bound for a hand")
    report("release and catch, 150 ms", release_drop(150),
           "the robot is let go and caught; measured |a| is 0 in the fall")
    report("release and catch, 80 ms", release_drop(80),
           "short release")

    for height_mm, length_mm, speed in ((10, 40, 0.30), (10, 40, 0.50),
                                        (10, 60, 0.30), (10, 60, 0.50),
                                        (5, 40, 0.30), (5, 40, 0.50)):
        name = f"hump {height_mm} mm / {length_mm} mm / {speed:.2f} m/s"
        report(name, raised_cosine(height_mm / 1000.0, length_mm / 1000.0, speed),
               "rigid wheel over a raised cosine; an UPPER BOUND, tyre "
               "compliance smooths it")

    print()
    print("Reading the table")
    print("  A 10 mm hump is modelled well above 500 mg, so the CURRENT rule "
          "can fire on a hump.")
    print("  A gentle hand lift is modelled near 100 mg, below every rule here.")
    print("  A plus-minus Z rule at 250 mg fires on both the gentle lift region "
          "and the hump region.")
    print("  Therefore the threshold question cannot be settled by this model. "
          "It needs two recorded traces: one real lift, one real hump.")


if __name__ == "__main__":
    main()
