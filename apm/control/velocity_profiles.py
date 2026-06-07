"""Velocity profiles for smooth speed transitions. Used as setpoint filters upstream of a controller (e.g. PID)."""

import math
import time


class LinearRamp:
    """A linear velocity profile with constant acceleration / deceleration. Reaches target in fixed time."""

    def __init__(self, t_accel: float):
        self.current = 0.0
        self._v0 = 0.0
        self._v1 = 0.0
        self._t_elapsed = 0.0
        self._t_accel = t_accel

    def update(self, target: float, dt: float = 1.0) -> float:
        if abs(target - self._v1) > 1e-9: # Close enough
            self._v0 = self.current
            self._v1 = target
            self._t_elapsed = 0.0

        self._t_elapsed += dt
        if self._t_elapsed >= self._t_accel:
            self.current = self._v1
        else:
            accel = (self._v1 - self._v0) / self._t_accel
            self.current = self._v0 + accel * self._t_elapsed

        return self.current

    def reset(self, speed: float = 0.0):
        self.current = speed
        self._v0 = speed
        self._v1 = speed
        self._t_elapsed = 0.0


class ExponentialRamp:
    """An exponential (first-order low-pass) velocity profile for smooth acceleration."""

    def __init__(self, alpha: float):
        self.alpha = alpha
        self.current = 0.0
        self.last_update_time = time.monotonic()

    def update(self, target: float, dt: float = 1.0) -> float:
        effective_alpha = 1.0 - math.exp(-self.alpha * dt)
        self.current += effective_alpha * (target - self.current)
        return self.current

    def reset(self, speed: float = 0.0):
        self.current = speed


class SigmoidRamp:
    """Velocity profile using a logistic sigmoid function. Asymptotic at both ends. 
    Approaches but never exactly reaches the target.

    Adapted from the 2024 Master's thesis "Development of an Autonomous Rabbit
    for Running on a Track" by Kvamme et al.

    Args:
        k: Steepness [1/s]. Higher = faster transition through the middle.
        s: Inflection point [s]. Time at which the transition is halfway to the setpoint (steepest part of the curve).
           At s*2, the profile is 
           Also controls how close to v0 the profile starts: sigmoid(0) ≈ 0 when k*s >> 1.
    """

    def __init__(self, k: float, s: float):
        self.k = k
        self.s = s
        self.current = 0.0
        self._v0 = 0.0
        self._v1 = 0.0
        self._t_elapsed = 0.0

    def update(self, target: float, dt: float = 1.0) -> float:
        if abs(target - self._v1) > 1e-9: # "Close enough"
            self._v0 = self.current
            self._v1 = target
            self._t_elapsed = 0.0

        self._t_elapsed += dt
        sigmoid = 1.0 / (1.0 + math.exp(-self.k * (self._t_elapsed - self.s)))
        self.current = self._v0 + (self._v1 - self._v0) * sigmoid
        return self.current

    def reset(self, speed: float = 0.0):
        self.current = speed
        self._v0 = speed
        self._v1 = speed
        self._t_elapsed = 0.0