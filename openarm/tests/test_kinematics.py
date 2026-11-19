import mujoco
import numpy as np
import pytest
from openarm_app.kinematics import OpenArmIK, IKConvergenceError, pose_error


@pytest.fixture(scope="module")
def arm():
    return OpenArmIK()


def test_model_mapping_and_fk(arm):
    assert arm.xml.parts[-2:] == ("v1", "scene.xml")
    assert arm.setup.sides == ["right"]
    assert arm.setup.model.nq == 18
    np.testing.assert_array_equal(arm.setup.joint_resolver.arm_qpos_indices("right"), np.arange(9,16))
    pose = arm.forward(arm.home)
    assert pose.shape == (7,)
    assert np.linalg.norm(pose[3:]) == pytest.approx(1)
    # Independent direct MuJoCo frame transformation checks origin convention.
    data, model = arm.setup.data, arm.setup.model
    ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_ee_control_point")
    origin = arm.setup.origin_id
    rot = data.site_xmat[origin].reshape(3, 3)
    np.testing.assert_allclose(pose[:3], rot.T@(data.site_xpos[ee]-data.site_xpos[origin]), atol=1e-7)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, (rot.T@data.site_xmat[ee].reshape(3, 3)).ravel())
    assert abs(np.dot(pose[3:], quat)) == pytest.approx(1, abs=1e-7)


@pytest.mark.parametrize("scale", [.3, 1., -1.])
def test_nontrivial_ik_roundtrip(arm, scale):
    goal = arm.home+scale*np.array([.15,.1,-.1,.12,.08,.08,-.1])
    target = arm.forward(goal)
    q = arm.inverse(target, arm.home)
    pe, oe = pose_error(arm.forward(q), target)
    assert pe <= .002 and oe <= np.deg2rad(2)
    state = arm.setup.data.qpos
    left, lg = arm.setup.joint_resolver.get_driver(state, "left")
    np.testing.assert_allclose(left, arm._home16[8:15])
    assert lg == arm._home16[15]
    assert arm.setup.joint_resolver.get_driver(state, "right")[1] == arm.gripper


def test_unreachable_fails(arm):
    with pytest.raises(IKConvergenceError, match="converge|solution|Invalid"):
        arm.inverse([10, 10, 10, 1, 0, 0, 0], arm.home, max_iterations=10)


def test_quaternion_and_validation(arm):
    pose = arm.forward(arm.home)
    other = pose.copy()
    other[3:] *= -3
    assert pose_error(pose, other)[1] < 1e-7
    np.testing.assert_allclose(arm.inverse(other, arm.home), arm.home)
    with pytest.raises(ValueError):
        arm.inverse([0]*7, arm.home)
    with pytest.raises(ValueError):
        arm.forward([100]*7)
    with pytest.raises(ValueError):
        arm.inverse(pose, [0]*6)


def test_solver_failure_is_explicit(arm, monkeypatch):
    target = arm.forward(arm.home + np.array([.2,.1,0,0,0,0,0]))
    monkeypatch.setattr(arm.kin, "solve", lambda: None)
    with pytest.raises(IKConvergenceError, match="no solution"):
        arm.inverse(target, arm.home)


def test_solver_roundoff_is_projected_but_inputs_and_real_violations_rejected(arm, monkeypatch):
    goal = arm.home.copy()
    goal[2] = arm.limits[2, 0]
    target = arm.forward(goal)
    rounded = goal.copy()
    rounded[2] -= 5e-7
    # Rounding tolerance is only for official solver output, never input.
    with pytest.raises(ValueError, match="outside"):
        arm.inverse(target, rounded)
    result = arm._home16.copy()
    result[arm._slice] = rounded
    monkeypatch.setattr(arm.kin, "solve", lambda: result)
    q = arm.inverse(target, arm.home)
    np.testing.assert_array_equal(q, goal)
    assert np.all(q >= arm.limits[:, 0]) and np.all(q <= arm.limits[:, 1])
    result[2] = arm.limits[2, 0] - 2e-6
    with pytest.raises(IKConvergenceError, match="outside"):
        arm.inverse(target, arm.home)


@pytest.fixture(scope="module")
def left(arm):
    return OpenArmIK(arm.xml, side="left", model=arm.setup.model)


def test_left_mapping_fk_and_shared_model_isolation(arm, left):
    assert left.setup.model is arm.setup.model
    assert left.setup.data is not arm.setup.data
    assert left.setup.sides == ["left"]
    np.testing.assert_array_equal(left.setup.joint_resolver.arm_qpos_indices("left"), np.arange(7))
    before = arm.setup.data.qpos.copy()
    pose = left.forward(left.home + [.12, .06, -.08, -.1, .05, -.04, .07])
    np.testing.assert_array_equal(arm.setup.data.qpos, before)
    data, model = left.setup.data, left.setup.model
    ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_ee_control_point")
    origin = left.setup.origin_id
    rot = data.site_xmat[origin].reshape(3, 3)
    np.testing.assert_allclose(pose[:3], rot.T@(data.site_xpos[ee]-data.site_xpos[origin]), atol=1e-7)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, (rot.T@data.site_xmat[ee].reshape(3, 3)).ravel())
    assert abs(np.dot(pose[3:], quat)) == pytest.approx(1, abs=1e-7)


@pytest.mark.parametrize("scale", [.3, 1., -1.])
def test_left_ik_roundtrip_and_gripper_mapping(left, scale):
    target = left.forward(left.home + scale*np.array([.12,.08,-.08,-.1,.04,-.06,.07]))
    q = left.inverse(target, left.home)
    pe, oe = pose_error(left.forward(q), target)
    assert pe <= .002 and oe <= np.deg2rad(2)
    right, rg = left.setup.joint_resolver.get_driver(left.setup.data.qpos, "right")
    np.testing.assert_allclose(right, left._home16[:7])
    assert rg == left._home16[7]
    assert left.setup.joint_resolver.get_driver(left.setup.data.qpos, "left")[1] == left.gripper


def test_left_unreachable_and_bad_side(left):
    with pytest.raises(IKConvergenceError):
        left.inverse([10,10,10,1,0,0,0], left.home, max_iterations=10)
    with pytest.raises(ValueError, match="side"):
        OpenArmIK(side="both")
