"""Trapezoidal wrist transfers for the real-piano score coordinates."""
from dataclasses import dataclass
from math import ceil, radians

from openarm_piano_origin import WristPoint
from openarm_trajectory import Trajectory

US_PER_S = 1_000_000
LIFT_DEG = 2.
LIFT_RISE = .25
LIFT_DESCENT = .60
LIFT_END = .90
MIN_MOVE_US = 120_000  # Three 40 ms arm cycles, including a sample at the lift plateau.


def _trapezoid(u):
    """Position and velocity in normalized time, with quarter-length ramps."""
    peak = 4 / 3
    if u < .25:
        return 2 * peak * u**2, 4 * peak * u
    if u < .75:
        return peak * (u - .125), peak
    remaining = 1 - u
    return 1 - 2 * peak * remaining**2, 4 * peak * remaining


def _lift(u):
    if u < LIFT_RISE:
        blend, rate = _trapezoid(u / LIFT_RISE)
        return blend, rate / LIFT_RISE
    if u < LIFT_DESCENT:
        return 1., 0.
    if u < LIFT_END:
        tail = LIFT_END - LIFT_DESCENT
        blend, rate = _trapezoid((u - LIFT_DESCENT) / tail)
        return 1 - blend, -rate / tail
    return 0., 0.


@dataclass(frozen=True)
class WristProfile:
    lift_rad: float
    speed_override: tuple = ()

    @classmethod
    def between(cls, start_deg, end_deg, j7_max, speed_override=()):
        # +J7 raises the four Aero pads in the unified V1 model on both arms.
        room = j7_max - radians(max(start_deg[-1], end_deg[-1]))
        return cls(max(0., min(radians(LIFT_DEG), room)), speed_override)

    def _displacements(self, q0, q1, order):
        displacement = [abs(b-a) for a, b in zip(q0, q1)]
        displacement[-1] += self.lift_rad / LIFT_RISE**order
        return displacement

    def duration_us(self, start_deg, end_deg, limits):
        displacement = self._displacements(tuple(map(radians, start_deg)), tuple(map(radians, end_deg)), 1)
        peak_time = (4 / 3) * max(delta / limit for delta, limit in zip(displacement, limits))
        return max(MIN_MOVE_US, ceil(peak_time * US_PER_S))


@dataclass(frozen=True)
class WristTransfer:
    start_us: int
    finish_us: int
    press_us: int
    target: WristPoint
    profile: WristProfile | None


@dataclass(frozen=True)
class WristPath:
    points: tuple
    transfers: tuple = ()
    approach: WristProfile | None = None


@dataclass(frozen=True)
class WristTrajectory(Trajectory):
    profiles: dict
    faster_intervals: tuple = ()

    def interpolate(self, start, q0, q1, u, span):
        profile = self.profiles.get(start)
        if profile is None:
            return super().interpolate(start, q0, q1, u, span)
        blend, rate = _trapezoid(u)
        q = [a + (b-a)*blend for a, b in zip(q0, q1)]
        v = [(b-a)*rate/span for a, b in zip(q0, q1)]
        lift, lift_rate = _lift(u)
        q[-1] += profile.lift_rad*lift
        v[-1] += profile.lift_rad*lift_rate/span
        return tuple(q), tuple(v)

    def peak_bounds(self, start, q0, q1, span):
        profile = self.profiles.get(start)
        if profile is None:
            return super().peak_bounds(start, q0, q1, span)
        return ((4 / 3)*max(profile._displacements(q0, q1, 1))/span,
                (16 / 3)*max(profile._displacements(q0, q1, 2))/span**2)

    def speeds_at(self, elapsed):
        for start, end, speeds in self.faster_intervals:
            if start <= elapsed <= end:
                return dict(zip(self.keys, speeds))
        return {}
