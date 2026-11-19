import numpy as np
import pytest
from openarm_app.trajectory import JointTrajectoryPlanner
from openarm_app.trajectory.quintic import quintic


@pytest.fixture
def planner():
    return JointTrajectoryPlanner([-3]*7, [3]*7)


def test_endpoints_extrema_and_csv(planner, tmp_path):
    goal = np.array([1, -2, 0, .3, -.1, .5, 2.5])
    v, a = np.linspace(.1, .7, 7), np.linspace(.2, 1, 7)
    tr = planner.plan(np.zeros(7), goal, v, a, sample_period=.07)
    np.testing.assert_array_equal(tr.q[0], np.zeros(7))
    np.testing.assert_array_equal(tr.q[-1], goal)
    np.testing.assert_array_equal(tr.dq[[0, -1]], 0)
    np.testing.assert_array_equal(tr.ddq[[0, -1]], 0)
    assert np.all(np.diff(tr.time) > 0)
    assert np.max(np.diff(tr.time)) <= .07+1e-12
    t = tr.time[-1]
    assert np.all(1.875*np.abs(goal)/t <= v*(1+1e-12))
    assert np.all(10/np.sqrt(3)*np.abs(goal)/t**2 <= a*(1+1e-12))
    assert np.all(tr.q >= -3) and np.all(tr.q <= 3)
    path = tmp_path / "trajectory.csv"
    tr.save_csv(path)
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    assert data.shape == (len(tr.time), 22)
    np.testing.assert_allclose(data[:, 1:8], tr.q)


def test_zero_move_and_duration(planner):
    tr = planner.plan([1]*7, [1]*7, [1]*7, [1]*7, duration=2)
    np.testing.assert_allclose(tr.q, 1)
    np.testing.assert_allclose(tr.dq, 0)
    assert tr.time[-1] == 2


@pytest.mark.parametrize("field,value", [
    ("q_start", [0]*6), ("q_goal", [np.nan]*7), ("q_goal", [4]*7),
    ("velocity_limits", [0]*7), ("acceleration_limits", [-1]*7),
    ("sample_period", 0), ("sample_period", np.inf), ("duration", -1)])
def test_reject_invalid(planner, field, value):
    args = dict(q_start=[0]*7, q_goal=[1]*7, velocity_limits=[1]*7, acceleration_limits=[1]*7)
    args[field] = value
    with pytest.raises(ValueError):
        planner.plan(**args)


def test_polynomial_derivatives():
    u = np.linspace(.01, .99, 100)
    h = 1e-5
    s, ds, dds = quintic(u)
    np.testing.assert_allclose((quintic(u+h)[0]-quintic(u-h)[0])/(2*h), ds, atol=1e-8)
    np.testing.assert_allclose((quintic(u+h)[1]-quintic(u-h)[1])/(2*h), dds, atol=1e-7)


def test_fixed_period_count_and_continuous_limits(planner):
    args = dict(q_start=[0]*7, q_goal=[.1]*7, velocity_limits=[.2]*7,
                acceleration_limits=[.4]*7, sample_period=1/60)
    for count in (120, 300):
        tr = planner.plan(**args, interpolation_steps=count)
        assert len(tr.time) == count + 1
        assert tr.time[-1] == count/60
        np.testing.assert_allclose(np.diff(tr.time), 1/60, atol=1e-14)
        np.testing.assert_array_equal(tr.q[-1], [.1]*7)
    # Acceleration is limiting: sqrt((10/sqrt(3))*.1/.4) -> 73 steps.
    with pytest.raises(ValueError, match="至少需要 73 步"):
        planner.plan(**args, interpolation_steps=72)
    assert len(planner.plan(**args, interpolation_steps=73).time) == 74
    # Velocity is limiting: 1.875*.1/.01 -> 1125 steps.
    args["velocity_limits"] = [.01]*7
    with pytest.raises(ValueError, match="至少需要 1125 步"):
        planner.plan(**args, interpolation_steps=1124)
    assert planner.plan(**args, interpolation_steps=1125).time[-1] == 18.75
    for bad in (True, 0, -1, 2.5, "120", 100001):
        with pytest.raises(ValueError):
            planner.plan(**args, interpolation_steps=bad)
    with pytest.raises(ValueError, match="不能同时指定"):
        planner.plan(**args, interpolation_steps=1200, duration=20)


def test_fourteen_axes_synchronize_and_export(tmp_path):
    planner = JointTrajectoryPlanner([-3]*14, [3]*14, joint_count=14)
    goal = np.linspace(-.4, .9, 14)
    v, a = np.full(14, .2), np.full(14, .4)
    v[10] = .015  # A slow left-arm axis must lengthen the entire motion.
    tr = planner.plan(np.zeros(14), goal, v, a)
    total = tr.time[-1]
    assert total == pytest.approx(1.875*abs(goal[10])/v[10])
    moving = goal != 0
    np.testing.assert_allclose(tr.q[:, moving]/goal[moving],
                               np.tile(quintic(tr.time/total)[0][:, None], (1, np.count_nonzero(moving))))
    np.testing.assert_array_equal(tr.q[:, ~moving], 0)
    np.testing.assert_array_equal(tr.q[-1], goal)
    np.testing.assert_array_equal(tr.dq[[0, -1]], 0)
    np.testing.assert_array_equal(tr.ddq[[0, -1]], 0)
    assert np.all(1.875*np.abs(goal)/total <= v*(1+1e-12))
    assert np.all(10/np.sqrt(3)*np.abs(goal)/total**2 <= a*(1+1e-12))
    path = tmp_path/"dual.csv"
    tr.save_csv(path)
    assert np.loadtxt(path, delimiter=",", skiprows=1).shape == (len(tr.time), 43)
    assert "q14_rad" in path.read_text().splitlines()[0]
    with pytest.raises(ValueError):
        planner.plan(np.zeros(14), goal, [.2]*7, a)
