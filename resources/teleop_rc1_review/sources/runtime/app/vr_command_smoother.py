"""Local Ruckig retargeting of 14 arm joints; no ROS, IK, hardware or queues.

Streaming mode lets each joint pursue its goal without waiting for its arm's
slowest joint. Retargeting preserves commanded position/velocity/acceleration;
terminal velocity is zero. No input scaling or prediction is applied.
"""
import math
from pathlib import Path
import sys
from collections import deque
from vr_joint_limits import named_limits


def finite_vector(values, size, label):
    result = [float(v) for v in values]
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(label + " must contain finite values of the expected size")
    return result


class SourcePositionFilter:
    """Short convex filter for original IK publication cadence, without OTG.

    The same newest-first weights apply to every arm joint. There is no target
    prediction, displacement scaling, velocity cap or accumulating pursuit
    queue. Unlike the upstream weighted filter's duplicate suppression, equal
    samples advance the history so a stationary goal is reached exactly.
    This reduces command steps; it does not certify V/A/jerk or collision safety.
    """
    def __init__(self, names, weights=(.6,.3,.1), history_max_age_s=.08):
        self.names=list(names)
        self.weights=finite_vector(weights,3,'source weights')
        if min(self.weights)<0 or abs(sum(self.weights)-1)>1e-9:
            raise ValueError('Source weights must be nonnegative and sum to one')
        if not .04<=history_max_age_s<=.1:
            raise ValueError('Source filter history must be 40-100ms')
        self.max_age_ns=int(history_max_age_s*1e9)
        self.arm_indices=[self.names.index(f'openarm_{side}_joint{i}') for side in ('left','right') for i in range(1,8)]
        self.history=deque(maxlen=3)
        self.last_stamp=None

    def reset(self, seed=None, now_ns=None):
        self.history.clear();self.last_stamp=None
        if seed is not None:
            seed=finite_vector(seed,len(self.names),'source seed')
            if now_ns is None:raise ValueError('Seed timestamp is required')
            self.history.extend([(now_ns,seed[:]),(now_ns,seed[:])])

    def update(self, goal, now_ns):
        goal=finite_vector(goal,len(self.names),'source IK goal')
        if self.last_stamp is not None and now_ns<=self.last_stamp:
            raise ValueError('Nonincreasing source filter timestamp')
        self.last_stamp=now_ns
        while self.history and now_ns-self.history[0][0]>self.max_age_ns:
            self.history.popleft()
        self.history.append((now_ns,goal[:]))
        samples=list(reversed(self.history))
        weights=self.weights[:len(samples)];scale=sum(weights)
        if scale<=0:raise ValueError('No weight for fresh source samples')
        result=goal[:]
        for i in self.arm_indices:
            result[i]=sum(w*q[i] for w,(_,q) in zip(weights,samples))/scale
        age_ms=sum(w*(now_ns-stamp)/1e6 for w,(stamp,_) in zip(weights,samples))/scale
        return result,age_ms


class ArmSmoother:
    def __init__(self, names, profile, period_s=.01, tracking=None):
        # Dependency is pinned in a separate persistent vendor directory. Nothing
        # is installed into or upgraded in the upstream teleop-xr environment.
        dependency = Path(__file__).resolve().parents[1] / "vendor" / "vr_output_deps"
        if dependency.is_dir() and str(dependency) not in sys.path:
            sys.path.insert(0, str(dependency))
        from ruckig import Ruckig, InputParameter, OutputParameter, Synchronization
        self.names = list(names)
        self.groups = [[self.names.index(f"openarm_{side}_joint{i}") for i in range(1, 8)]
                       for side in ("left", "right")]
        self.period_s = period_s
        self.limits = named_limits(profile)
        self.tracking = dict(tracking or {})
        self.check_extrema = bool(self.tracking.get("check_trajectory_extrema", False))
        self.engines = []
        for group in self.groups:
            inp = InputParameter(7)
            for key in ("max_velocity", "max_acceleration", "max_jerk"):
                vector = finite_vector([self.limits[self.names[i]][key] for i in group], 7, key)
                if min(vector) <= 0:
                    raise ValueError(key + " must be positive")
                setattr(inp, key, vector)
            inp.target_velocity = [0.0]*7
            inp.target_acceleration = [0.0]*7
            if self.tracking.get("independent_joints", False):
                inp.synchronization = Synchronization.No
            self.engines.append((Ruckig(7, period_s), inp, OutputParameter(7)))
        self.q = None
        self.v = None
        self.a = None
        self.target = None
        self._safe_goals = []
        self.extrema_fallbacks = 0

    def validate_q(self, q):
        q = finite_vector(q, len(self.names), "q")
        for group in self.groups:
            for index in group:
                limit = self.limits[self.names[index]]
                if not limit["min_position"]-1e-8 <= q[index] <= limit["max_position"]+1e-8:
                    raise ValueError("Arm goal outside loaded model limits: "+self.names[index])
        return q

    def reset(self, q, velocity=None, acceleration=None):
        self.q = self.validate_q(q)
        n = len(self.names)
        self.v = finite_vector(velocity if velocity is not None else [0.0]*n, n, "velocity")
        self.a = finite_vector(acceleration if acceleration is not None else [0.0]*n, n, "acceleration")
        self.target = list(self.q)
        self._safe_goals = [[self.q[i] for i in group] for group in self.groups]
        for group, (engine, inp, out) in zip(self.groups, self.engines):
            engine.reset()
            inp.current_position = [self.q[i] for i in group]
            inp.current_velocity = [self.v[i] for i in group]
            inp.current_acceleration = [self.a[i] for i in group]
            inp.target_position = list(inp.current_position)

    def retarget(self, q):
        self.target = self.validate_q(q)
        for group, (_, inp, _) in zip(self.groups, self.engines):
            inp.target_position = [self.target[i] for i in group]

    def step(self):
        if self.q is None:
            raise RuntimeError("Smoother has no initial state")
        for g, (group, (engine, inp, out)) in enumerate(zip(self.groups, self.engines)):
            goal = [self.target[i] for i in group]
            choices = [goal, self._safe_goals[g]] if self.check_extrema else [goal]
            for attempt, candidate in enumerate(choices):
                inp.target_position = candidate
                result = engine.update(inp, out)
                if int(result) < 0:
                    raise RuntimeError("Ruckig rejected trajectory: " + str(result))
                extrema = out.trajectory.position_extrema if self.check_extrema else []
                if all(self.limits[self.names[i]]["min_position"]-1e-8 <= e.min
                       and e.max <= self.limits[self.names[i]]["max_position"]+1e-8
                       for i, e in zip(group, extrema)):
                    self._safe_goals[g] = list(candidate)
                    if attempt:
                        self.extrema_fallbacks += 1
                    break
            else:
                raise RuntimeError("No model-bounded trajectory from current commanded state")
            for j, index in enumerate(group):
                self.q[index] = float(out.new_position[j])
                self.v[index] = float(out.new_velocity[j])
                self.a[index] = float(out.new_acceleration[j])
            out.pass_to_input(inp)
        # Community OTG does not enforce position extrema: fail closed on an
        # overshoot; never clip q independently (which would break continuity).
        self.validate_q(self.q)
        return list(self.q), list(self.v), list(self.a)
