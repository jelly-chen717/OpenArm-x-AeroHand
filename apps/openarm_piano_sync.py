"""One score clock for measured finger release, wrist transfer and key press."""
from collections import defaultdict
from dataclasses import dataclass
from math import degrees, dist
from threading import Lock

from openarm_piano_wrist import US_PER_S

FINGER_HOME_DEG = 2.
WRIST_DISTANCE_MM = 5.
WRIST_J7_DEG = 1.5
ARRIVAL_SAMPLES = 2


@dataclass(frozen=True)
class Cue:
    time_us: int
    fingers: tuple
    wrists: dict

    @property
    def time_s(self):
        return self.time_us / US_PER_S


class PianoClock:
    def __init__(self, plan, endpoints):
        info = plan["piano_trial"]
        sides = plan["trial_fingers"].sides
        lift_time = round(plan["steps"][0][1] * US_PER_S)
        lifts = defaultdict(list, {lift_time: list(sides)})
        arrivals = defaultdict(dict)
        first_press = round(info["prepare_s"] * US_PER_S)
        for side in sides:
            arrivals[first_press][side] = info["first_wrist_targets"][side]
            for move in plan["side_paths"][side].transfers:
                lifts[move.start_us].append(side)
                point = move.target
                arrivals[move.press_us][side] = dict(note=point.note, xyz_mm=point.xyz, joints_deg=point.deg)
        self.lift_times = tuple(lifts)
        self.cues = tuple(Cue(t, tuple(lifts.get(t, ())), arrivals.get(t, {})) for t in sorted(lifts.keys() | arrivals.keys()))
        self.endpoints = endpoints
        self.lock = Lock()
        self.origin = None
        self.index = 0
        self.finger_error = {}
        self.arrival_samples = 0
        self.wait_logged = False
        self.paused_s = 0.

    def start(self, now):
        with self.lock:
            self.origin = now

    def _sample(self, now):
        elapsed = now - self.origin
        if self.index < len(self.cues) and elapsed >= self.cues[self.index].time_s:
            cue = self.cues[self.index]
            return cue.time_s, cue
        return elapsed, None

    def sample(self, now):
        with self.lock:
            return self._sample(now)

    def hand_state(self, stamp, now):
        with self.lock:
            elapsed, cue = self._sample(now)
            blocked = cue is not None and stamp >= cue.time_us
            probe = ()
            if blocked and stamp == cue.time_us:
                probe = self._fingers_pending(cue)
            return elapsed, blocked, probe

    def _fingers_pending(self, cue):
        return tuple(side for side in cue.fingers if self.finger_error.get(side, float("inf")) > FINGER_HOME_DEG)

    def report_home(self, stamp, errors):
        with self.lock:
            if self.index < len(self.cues) and self.cues[self.index].time_us == stamp:
                self.finger_error.update(errors)

    def observe(self, cue, measured, now):
        wrist_errors = {}
        if cue.wrists:
            endpoints = self.endpoints(measured)
            for side, target in cue.wrists.items():
                xyz = endpoints[side]["wrist"]
                j7 = degrees(measured[side, 7]) - target["joints_deg"][-1]
                wrist_errors[side] = dict(note=target["note"], distance_mm=dist(xyz, target["xyz_mm"]),
                                          y_mm=xyz[1] - target["xyz_mm"][1], j7_deg=j7)
        wrists_ready = all(row["distance_mm"] <= WRIST_DISTANCE_MM and abs(row["j7_deg"]) <= WRIST_J7_DEG
                           for row in wrist_errors.values())
        with self.lock:
            if self.index == len(self.cues) or self.cues[self.index] is not cue:
                return None, False
            self.arrival_samples = self.arrival_samples + 1 if wrists_ready else 0
            fingers = self._fingers_pending(cue)
            ready = not fingers and (not wrist_errors or self.arrival_samples >= ARRIVAL_SAMPLES)
            delay = max(0., now - self.origin - cue.time_s)
            status = dict(time_us=cue.time_us, waiting=not ready, waiting_s=delay,
                          total_wait_s=self.paused_s + delay, fingers=list(fingers),
                          wrist_errors=wrist_errors, finger_error_deg=self.finger_error.copy())
            changed = ready or not self.wait_logged
            self.wait_logged = True
            if ready:
                self._resume(delay)
                self.index += 1
            return status, changed

    def _resume(self, delay):
        self.origin += delay
        self.paused_s += delay
        self.finger_error.clear()
        self.arrival_samples = 0
        self.wait_logged = False

    def hand_finished(self, now):
        # A serial failure ends finger dispatch. The arm retains its current
        # target and proceeds along the remaining curve using the same clock.
        with self.lock:
            if self.origin is not None:
                _, cue = self._sample(now)
                if cue is not None:
                    self._resume(now - self.origin - cue.time_s)
            self.index = len(self.cues)
