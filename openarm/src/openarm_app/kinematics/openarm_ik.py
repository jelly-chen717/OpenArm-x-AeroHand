import numpy as np
from openarm_control import Kinematics, IKParams
from openarm_app.validation import vector, positive
from .model_adapter import load_v1


class IKConvergenceError(RuntimeError):
    """No acceptable endpoint was found; no partial joint target is returned."""


def normalized_pose(value):
    pose = vector(value, 7, "pose [x,y,z,qw,qx,qy,qz]")
    norm = np.linalg.norm(pose[3:])
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Quaternion must have a finite nonzero norm")
    pose[3:] /= norm
    return pose


def pose_error(actual, target):
    a, b = normalized_pose(actual), normalized_pose(target)
    return float(np.linalg.norm(a[:3]-b[:3])), float(
        2*np.arccos(np.clip(abs(np.dot(a[3:], b[3:])), 0, 1)))


class OpenArmIK:
    """Single-threaded offline wrapper; all poses relative to arm_origin, wxyz."""
    def __init__(self, xml=None, *, side="right", model=None):
        self.setup, self.limits, self.xml = load_v1(xml, side=side, model=model)
        self.side = side
        self._slice = slice(0, 7) if side == "right" else slice(8, 15)
        resolver = self.setup.joint_resolver
        right, rg = resolver.get_driver(self.setup.data.qpos, "right")
        left, lg = resolver.get_driver(self.setup.data.qpos, "left")
        self.home = (right if side == "right" else left).copy()
        self._home16 = np.r_[right, rg, left, lg]
        self.gripper = float(rg if side == "right" else lg)
        # Offline endpoint solving: disable tracking-only shaping and nullspace drift.
        params = IKParams(dt=0.1, max_iters=5, nullspace_cost=0,
            singularity_max_approach_rate=0, kinetic_energy_cost=0,
            frame_position_error_limit=0, frame_orientation_error_limit=0)
        self.kin = Kinematics(self.setup, params)
        self.kin.set_gripper("right", float(rg))
        self.kin.set_gripper("left", float(lg))

    def _joints(self, value):
        q = vector(value, 7, "joints")
        if np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
            raise ValueError("Joint angles outside V1 model limits")
        return q

    def forward(self, q):
        return normalized_pose(self.kin.fk(self.side, np.r_[self._joints(q), self.gripper]))

    def _solver_joints(self, value):
        # The official solver returns float32 states. Permit only roundoff at
        # the boundary, then project back INSIDE the unchanged model limits.
        # User inputs/seeds still go through the strict _joints validator.
        q = vector(value, 7, "IK joints")
        tolerance = 1e-6  # rad; smaller than 0.00006 degrees
        if np.any(q < self.limits[:, 0]-tolerance) or np.any(q > self.limits[:, 1]+tolerance):
            raise ValueError("Joint angles outside V1 model limits")
        return np.clip(q, self.limits[:, 0], self.limits[:, 1])

    def inverse(self, target_pose, seed_q, *, max_iterations=300,
                position_tolerance=0.002, orientation_tolerance=np.deg2rad(2)):
        target, q = normalized_pose(target_pose), self._joints(seed_q)
        pt = positive(position_tolerance, "position_tolerance")
        ot = positive(orientation_tolerance, "orientation_tolerance")
        if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
        state = self._home16.copy()
        state[self._slice] = q
        self.kin.sync(state)
        for step in range(max_iterations+1):
            pe, oe = pose_error(self.forward(q), target)
            if pe <= pt and oe <= ot:
                return q.copy()
            if step == max_iterations:
                break
            self.kin.set_target(self.side, target)
            try:
                result = self.kin.solve()
            except Exception as exc:
                raise IKConvergenceError(f"Official IK failed at iteration {step}: {exc}") from exc
            if result is None:
                raise IKConvergenceError(f"Official IK returned no solution at iteration {step}")
            try:
                values = vector(result, 16, "IK state")
                q = self._solver_joints(values[self._slice])
            except ValueError as exc:
                raise IKConvergenceError(f"Invalid IK result: {exc}") from exc
        raise IKConvergenceError(f"IK did not converge after {max_iterations} iterations: "
                                 f"position={pe:.6f} m, orientation={np.rad2deg(oe):.4f} deg")
