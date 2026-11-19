from dataclasses import dataclass
from pathlib import Path
import numpy as np
from openarm_app.validation import vector, positive
from .quintic import quintic
from .timing import interpolation_steps as validate_steps


@dataclass(frozen=True)
class Trajectory:
    time: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray

    def save_csv(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        header = ["time_s"] + [f"{field}{i}_{unit}" for field, unit in
                  (("q", "rad"), ("dq", "rad_s"), ("ddq", "rad_s2"))
                  for i in range(1, self.q.shape[1]+1)]
        np.savetxt(path, np.column_stack((self.time, self.q, self.dq, self.ddq)),
                   delimiter=",", header=",".join(header), comments="")


class JointTrajectoryPlanner:
    def __init__(self, lower_limits, upper_limits, *, joint_count=7):
        if isinstance(joint_count, bool) or joint_count not in (7, 14):
            raise ValueError("joint_count must be 7 or 14")
        self.joint_count = int(joint_count)
        self.lower = vector(lower_limits, self.joint_count, "lower_limits")
        self.upper = vector(upper_limits, self.joint_count, "upper_limits")
        if np.any(self.lower >= self.upper):
            raise ValueError("Each lower limit must be below its upper limit")

    def plan(self, q_start, q_goal, velocity_limits, acceleration_limits,
             sample_period=0.05, duration=None, *, interpolation_steps=None):
        q0, q1 = (vector(q, self.joint_count, name) for q, name in
                  ((q_start, "q_start"), (q_goal, "q_goal")))
        v = vector(velocity_limits, self.joint_count, "velocity_limits")
        a = vector(acceleration_limits, self.joint_count, "acceleration_limits")
        if np.any(v <= 0) or np.any(a <= 0):
            raise ValueError("Velocity and acceleration limits must be positive")
        if np.any(np.minimum(q0, q1) < self.lower) or np.any(np.maximum(q0, q1) > self.upper):
            raise ValueError("Joint endpoint exceeds model limits")
        dt = positive(sample_period, "sample_period")
        requested = dt if duration is None else positive(duration, "duration")
        delta = q1-q0
        # Exact continuous-time extrema: max s'=15/8; max |s''|=10/sqrt(3).
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            minimum = max(dt, float(np.max(1.875*np.abs(delta)/v)),
                        float(np.max(np.sqrt((10/np.sqrt(3))*np.abs(delta)/a))))
        if interpolation_steps is not None:
            count = validate_steps(interpolation_steps)
            if duration is not None:
                raise ValueError("duration 与 interpolation_steps 不能同时指定")
            total = count * dt
            if total < minimum * (1 - 1e-12):
                required = int(np.ceil(minimum / dt))
                raise ValueError(f"插补步数不足：当前 {count} 步（{total:.3f} s），"
                                 f"满足速度/加速度限制至少需要 {required} 步；未执行动作")
        else:
            total = max(requested, minimum)
            count = np.ceil(total/dt)
        if not np.isfinite(count) or count > 1_000_000:
            raise ValueError("Trajectory requires too many samples")
        times = np.linspace(0, total, int(count)+1)
        s, ds, dds = quintic(times/total)
        q = q0+s[:, None]*delta
        q[0], q[-1] = q0, q1
        dq, ddq = ds[:, None]*delta/total, dds[:, None]*delta/total**2
        if not all(np.all(np.isfinite(x)) for x in (q, dq, ddq)):
            raise ValueError("Non-finite trajectory")
        if np.any(np.abs(dq) > v*(1+1e-10)) or np.any(np.abs(ddq) > a*(1+1e-10)):
            raise ValueError("Trajectory derivative limit exceeded")
        return Trajectory(times, q, dq, ddq)
