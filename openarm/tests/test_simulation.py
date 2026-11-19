import numpy as np
import pytest
from openarm_app.kinematics import OpenArmIK
from openarm_app.trajectory import JointTrajectoryPlanner
from openarm_app.simulation import Simulation, load_trajectory, run


@pytest.fixture(scope="module")
def trajectory_path(tmp_path_factory):
    arm = OpenArmIK()
    target = arm.forward(arm.home + np.array([.1,.1,-.1,.1,.05,.05,-.05]))
    end = arm.inverse(target, arm.home)
    trajectory = JointTrajectoryPlanner(arm.limits[:, 0], arm.limits[:, 1]).plan(
        arm.home, end, [.2]*7, [.4]*7)
    path = tmp_path_factory.mktemp("sim")/"trajectory.csv"
    trajectory.save_csv(path)
    return path


def test_physics_is_not_fk_teleportation(trajectory_path):
    sim = Simulation(trajectory_path, mode="dynamic")
    state = sim.data.qpos.copy()
    sim.arm.forward(sim.positions[-1])
    np.testing.assert_array_equal(state, sim.data.qpos)
    for _ in range(100):
        sim.step()
    assert not np.array_equal(sim.data.qpos, state)  # Original gravity drives real dynamics.
    assert sim.data.time == pytest.approx(.1)
    sim.status = "STOPPED"
    sim.step()
    assert sim.data.time == pytest.approx(.1)


@pytest.mark.parametrize("compensated,expected", [(False, "NOT_REACHED"), (True, "PASSED")])
def test_dynamic_acceptance_and_logs(trajectory_path, tmp_path, compensated, expected):
    sim = Simulation(trajectory_path, gravity_compensation=compensated, mode="dynamic")
    run(sim, True)
    report = sim.save(tmp_path)
    assert report["status"] == expected
    assert report["gravity_compensation"] == compensated
    assert not report["joint_limit_violation"]
    data = np.loadtxt(tmp_path/"tracking.csv", delimiter=",", skiprows=1)
    assert data.shape[1] == 40
    assert np.all(np.diff(data[:, 0]) > 0)
    if compensated:
        assert report["final_position_error_mm"] <= 2
        assert report["final_orientation_error_deg"] <= 2


@pytest.mark.parametrize("kind", ["header", "time", "nan", "limits"])
def test_bad_csv_rejected(trajectory_path, tmp_path, kind):
    text = trajectory_path.read_text()
    header, rest = text.split("\n", 1)
    values = np.loadtxt(trajectory_path, delimiter=",", skiprows=1)
    if kind == "header":
        header = "wrong"
    elif kind == "time":
        values[1, 0] = 0
    elif kind == "nan":
        values[1, 1] = np.nan
    else:
        values[1, 1] = 100
    path = tmp_path/"invalid.csv"
    np.savetxt(path, values, delimiter=",", header=header, comments="")
    with pytest.raises(ValueError):
        load_trajectory(path, np.array([[-4,4]]*7))


def test_default_ideal_replay_exact_positions(trajectory_path, tmp_path, monkeypatch):
    import mujoco
    sim = Simulation(trajectory_path)
    assert sim.mode == "ideal"
    def no_physics(*args):
        raise AssertionError("Ideal replay must not integrate dynamics")
    monkeypatch.setattr(mujoco, "mj_step", no_physics)
    run(sim, True)
    report = sim.save(tmp_path)
    assert report["status"] == "PASSED"
    assert report["mode"] == "ideal" and not report["gravity_compensation"]
    rows = np.asarray(sim.rows)
    np.testing.assert_array_equal(rows[:, 2:9], rows[:, 9:16])
    np.testing.assert_array_equal(sim.peak, np.zeros(7))
    np.testing.assert_array_equal(sim.data.qpos[sim.indices], sim.positions[-1])
    np.testing.assert_array_equal(sim.data.qvel, np.zeros(sim.model.nv))
    left, lg = sim.resolver.get_driver(sim.data.qpos, "left")
    np.testing.assert_array_equal(left, sim.arm._home16[8:15])
    assert lg == sim.arm._home16[15]
    assert report["final_position_error_mm"] < 1e-6


def test_ideal_interpolation_and_stop(trajectory_path):
    sim = Simulation(trajectory_path)
    sim.data.time = sim.warmup+sim.times[-1]*.4
    sim.step()
    t = sim.data.time-sim.warmup
    expected = np.array([np.interp(t, sim.times, sim.positions[:, i]) for i in range(7)])
    np.testing.assert_array_equal(sim.last["actual"], expected)
    assert np.any(np.abs(sim.last["velocity"]) > 0)
    state, clock = sim.data.qpos.copy(), sim.data.time
    sim.status = "STOPPED"
    sim.step()
    np.testing.assert_array_equal(sim.data.qpos, state)
    assert sim.data.time == clock


@pytest.mark.parametrize("kwargs", [{"mode": "wrong"}, {"gravity_compensation": True}])
def test_invalid_modes(kwargs):
    with pytest.raises(ValueError):
        Simulation("unused.csv", **kwargs)
