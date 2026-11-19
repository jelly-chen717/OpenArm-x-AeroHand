"""One optional, soft clutch-elbow reference for the standard OpenArm solver.

World-frame Cartesian targets, original costs, joint bounds, collision geometry
and the 15-iteration algorithm remain unchanged. This module has no ROS/CAN IO.
"""
import jax
import jax.numpy as jnp
import jaxls
import numpy as np
from teleop_xr.ik.solver import PyrokiSolver


def _direction(fk, indices):
    shoulder, elbow, forearm = (fk[indices[:, i], 4:] for i in range(3))
    line = forearm - shoulder
    line = line / jnp.sqrt(jnp.sum(line * line, axis=-1, keepdims=True) + 1e-10)
    offset = elbow - shoulder
    return line, offset - jnp.sum(offset * line, axis=-1, keepdims=True) * line


def _clutch_elbow_residual(values, robot, joint_var, indices, reference, weights):
    line, current = _direction(robot.forward_kinematics(values[joint_var]), indices)
    _, initial = _direction(robot.forward_kinematics(reference), indices)
    desired = initial - jnp.sum(initial * line, axis=-1, keepdims=True) * line
    current = current / jnp.sqrt(jnp.sum(current * current, axis=-1, keepdims=True) + 1e-8)
    desired = desired / jnp.sqrt(jnp.sum(desired * desired, axis=-1, keepdims=True) + 1e-8)
    confidence = jnp.minimum(1., jnp.linalg.norm(initial, axis=-1, keepdims=True) / .03)
    return ((current - desired) * confidence * weights[:, None]).flatten()


_clutch_elbow_cost = jaxls.Cost.factory(_clutch_elbow_residual)


def inward_residual(values,joint_var,indices,reference,confidence,strength):
    # URDF shoulder abduction is left -J2 and right +J2. Allow 10 degrees
    # of inward motion relative to engagement before a soft cost appears.
    signs=jnp.array([-1.,1.])
    inward=signs*(reference[indices]-values[joint_var][indices])-jnp.deg2rad(10.)
    return jnp.maximum(inward,0.)*confidence*strength

inward_cost=jaxls.Cost.factory(inward_residual)

class ClutchElbowSolver:
    """Standard fallback plus a frozen-reference preference during dual-arm VR."""
    MAX_ELBOW_WEIGHT = .25
    SHOULDER_INWARD_WEIGHT = 1.5

    def __init__(self, robot):
        self.robot = robot
        self._base = PyrokiSolver(robot)
        links = robot.robot.links.names
        self._indices = jnp.array([[links.index('openarm_'+side+'_link'+str(n))
            for n in [2,4,5]] for side in ['left','right']], dtype=jnp.int32)
        self._elbow_joints = [robot.actuated_joint_names.index('openarm_'+s+'_joint4') for s in ['left','right']]
        self._reference = None
        self._weights = jnp.zeros(2, dtype=jnp.float32)
        self._shoulder_joints = jnp.array([robot.actuated_joint_names.index('openarm_'+side+'_joint2') for side in ['left','right']], dtype=jnp.int32)
        self._shoulder_weight = float(self.SHOULDER_INWARD_WEIGHT)
        self._shoulder_strength = jnp.asarray(self._shoulder_weight, dtype=jnp.float32)
        self._jit = jax.jit(self._solve_preference)
        self._jit_shoulder = jax.jit(self._solve_shoulder)
        self.warmed_target_patterns = self._base.warmed_target_patterns
        self.warmup_complete = False
        self.warmup_error = self._base.warmup_error
        self.preference_warmup_complete = False
        self.shoulder_warmup_complete = False
        try:
            q = jnp.asarray(robot.get_default_config())
            poses = robot.forward_kinematics(q)
            warm = self._jit(poses['left'], poses['right'], q, q, jnp.ones(2, dtype=jnp.float32))
            jax.block_until_ready(warm)
            self.preference_warmup_complete = True
            warm = self._jit_shoulder(poses['left'], poses['right'], q, q, jnp.ones(2, dtype=jnp.float32), self._shoulder_strength)
            jax.block_until_ready(warm)
            self.shoulder_warmup_complete = True
            self.warmup_complete = bool(self._base.warmup_complete)
        except Exception as error:
            self.warmup_error = 'Clutch elbow preference warmup: '+str(error)

    def clear_reference(self):
        self._reference = None
        self._weights = jnp.zeros(2, dtype=jnp.float32)

    def set_reference_config(self, q):
        q = np.array(q, copy=True)
        if q.shape != (len(self.robot.actuated_joint_names),) or not np.all(np.isfinite(q)):
            raise ValueError('Invalid engagement joint reference')
        # A bend plane is unreliable near extension. Freeze this eligibility
        # at engagement; it is a cost weight, never an enable/motion gate.
        degrees = np.abs(q[self._elbow_joints]) * 180 / np.pi
        weights = (self.MAX_ELBOW_WEIGHT * np.clip((degrees - 30.) / 15., 0., 1.)).astype(np.float32)
        self._reference = jnp.asarray(q)
        self._weights = jnp.asarray(weights)

    def _solve_preference(self, left, right, current, reference, weights):
        costs = self.robot.build_costs(left, right, None, current)
        JointVar = self.robot.joint_var_cls
        costs.append(_clutch_elbow_cost(self.robot.robot, JointVar(0), self._indices, reference, weights))
        variable = JointVar(jnp.array([0]))
        initial = jaxls.VarValues.make([variable.with_value(current[jnp.newaxis,:])])
        result = jaxls.LeastSquaresProblem(costs,[variable]).analyze().solve(
            initial_vals=initial, verbose=False, linear_solver='dense_cholesky',
            termination=jaxls.TerminationConfig(max_iterations=15))
        return result[variable][0]

    def _solve_shoulder(self,left,right,current,reference,weights,strength):
            robot=self.robot;JointVar=robot.joint_var_cls
            costs=robot.build_costs(left,right,None,current)
            costs.append(_clutch_elbow_cost(robot.robot,JointVar(0),self._indices,reference,weights))
            costs.append(inward_cost(JointVar(0),self._shoulder_joints,reference,weights/self.MAX_ELBOW_WEIGHT,strength))
            variable=JointVar(jnp.array([0]))
            initial=jaxls.VarValues.make([variable.with_value(current[jnp.newaxis,:])])
            result=jaxls.LeastSquaresProblem(costs,[variable]).analyze().solve(
                initial_vals=initial,verbose=False,linear_solver='dense_cholesky',
                termination=jaxls.TerminationConfig(max_iterations=15))
            return result[variable][0]

    def solve(self, left, right, head, current):
        if (self._reference is None or not self.preference_warmup_complete
                or left is None or right is None or head is not None
                or not bool(np.any(np.asarray(self._weights)))):
            return self._base.solve(left,right,head,current)
        if self._shoulder_weight > 0 and self.shoulder_warmup_complete:
            return self._jit_shoulder(left,right,current,self._reference,self._weights,self._shoulder_strength)
        return self._jit(left,right,current,self._reference,self._weights)

    def preference_status(self):
        return {'kind':'clutch_posture_v2' if self._shoulder_weight > 0 else 'clutch_elbow_plane_v1','reference_active':self._reference is not None,
                'arm_weights':np.asarray(self._weights).tolist(),
                'max_weight':self.MAX_ELBOW_WEIGHT,
                'shoulder_inward_weight':self._shoulder_weight,
                'shoulder_inward_soft_start_deg':10.,
                'shoulder_warmup_complete':self.shoulder_warmup_complete,
                'ramp_reference_bend_deg':[30.,45.],
                'rotation_mapping':'unchanged_world',
                'preference_warmup_complete':self.preference_warmup_complete}
