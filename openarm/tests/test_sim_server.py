import json
import threading
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import numpy as np
import pytest
from openarm_app.sim_server import ArmServerState, create_server
from openarm_app.sim_client import SimClient


@pytest.fixture(scope="module")
def arm_engine():
    return ArmServerState()


@pytest.fixture
def engine(arm_engine):
    e = arm_engine
    for side in e.SIDES:
        e.joints[side] = e.arms[side].home.copy()
        e.goals[side] = e.joints[side].copy()
        e.starts[side] = e.joints[side].copy()
        e.velocities[side][:] = 0
    e.active_arms = []
    e.motion_id = None
    e.elapsed = e.duration = 0
    e.interpolation_steps = None
    e.completed_steps = 0
    e.state = "IDLE"
    e.refresh_model()
    return e


def test_sequential_moves_pause_stop_and_hold(engine):
    goal = engine.q + [.1,.1,0,.1,0,0,0]
    engine.command({"action":"move_joints", "q":goal})
    engine.advance(.2)
    assert not np.array_equal(engine.q, engine.arm.home)
    engine.command({"action":"pause"})
    paused = engine.q.copy()
    elapsed = engine.elapsed
    engine.advance(1)
    np.testing.assert_array_equal(engine.q, paused)
    assert engine.elapsed == elapsed
    with pytest.raises(ValueError, match="busy"):
        engine.command({"action":"home"})
    engine.command({"action":"resume"})
    engine.advance(10)
    assert engine.state == "COMPLETED"
    np.testing.assert_array_equal(engine.q, goal)
    engine.advance(10)
    np.testing.assert_array_equal(engine.q, goal)
    engine.command({"action":"home"})
    engine.advance(.2)
    engine.command({"action":"stop"})
    stopped = engine.q.copy()
    engine.advance(10)
    np.testing.assert_array_equal(engine.q, stopped)
    engine.command({"action":"home"})
    engine.advance(10)
    np.testing.assert_array_equal(engine.q, engine.arm.home)


def test_fixed_steps_dual_sync_pause_stop_and_atomic_rejection(engine):
    from openarm_app.trajectory.timing import INTERPOLATION_PERIOD_S as dt
    start = {s: engine.joints[s].copy() for s in engine.SIDES}
    goal = {s: q + [.1, 0, 0, 0, 0, 0, 0] for s, q in start.items()}
    before = engine.snapshot()
    for steps in (1, None, True, 120.0):
        with pytest.raises(ValueError):
            engine.command({"action": "move_joints", "arm": "both", "q": goal,
                            "interpolation_steps": steps})
        assert engine.snapshot() == before
    state = engine.command({"action": "move_joints", "arm": "both", "q": goal,
                            "interpolation_steps": 120})
    assert state["duration_s"] == 2 and state["completed_steps"] == 0
    with pytest.raises(ValueError, match="fixed interpolation period"):
        engine.advance(1)
    assert engine.completed_steps == 0
    for _ in range(60):
        engine.advance(dt)
    for s in engine.SIDES:
        np.testing.assert_allclose(engine.joints[s], (start[s]+goal[s])/2)
    engine.command({"action": "pause"})
    engine.advance(dt)
    assert engine.completed_steps == 60 and engine.elapsed == 1
    engine.command({"action": "resume"})
    for _ in range(59):
        engine.advance(dt)
    assert engine.state == "EXECUTING" and engine.completed_steps == 119
    engine.advance(dt)
    assert engine.state == "COMPLETED" and engine.elapsed == engine.duration == 2
    for s in engine.SIDES:
        np.testing.assert_array_equal(engine.joints[s], goal[s])
    engine.advance(dt)
    assert engine.completed_steps == 120
    engine.command({"action": "move_joints", "q": start["right"], "interpolation_steps": 120})
    engine.advance(dt)
    stopped = engine.command({"action": "stop"})
    engine.advance(dt)
    assert engine.snapshot() == stopped


def test_pose_preview_move_and_rejection_are_atomic(engine):
    pose = engine.arm.forward(engine.q + [.1,.1,-.1,.1,.05,.05,-.05]).tolist()
    preview = engine.command({"action":"preview_pose", "pose":pose})
    assert engine.state == "IDLE"
    np.testing.assert_array_equal(engine.q, engine.arm.home)
    with pytest.raises(ValueError):
        engine.command({"action":"move_joints", "q":[100]*7})
    assert engine.state == "IDLE"
    engine.command({"action":"move_pose", "pose":pose})
    engine.advance(10)
    assert engine.state == "COMPLETED"
    np.testing.assert_allclose(engine.q, preview["q_goal"], atol=1e-6)


def test_http_control_and_invalid_requests(engine):
    server = create_server(engine, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = SimClient(server.server_port)
    try:
        assert client.state()["state"] == "IDLE"
        assert client.command("move_joints", q=(engine.q+[.1,0,0,0,0,0,0]).tolist())["state"] == "EXECUTING"
        assert client.command("pause")["state"] == "PAUSED"
        assert client.command("stop")["state"] == "STOPPED"
        left_goal = (engine.joints["left"]+[.05,0,0,0,0,0,0]).tolist()
        state = client.command("move_joints", arm="both", q={"right": engine.q.tolist(), "left": left_goal})
        assert state["active_arms"] == ["right", "left"]
        assert state["arms"]["left"]["goal"] == left_goal
        client.command("stop")
        for payload in ({"action":"bogus"}, {"action":"move_joints", "q":[float("nan")]*7}, []):
            request = Request(client.url+"/command", data=json.dumps(payload).encode(), headers={"Content-Type":"application/json"})
            with pytest.raises(HTTPError) as error:
                urlopen(request, timeout=3)
            assert error.value.code == 400
        request = Request(client.url+"/command", data=b'{"action":"home"}', headers={"Content-Type":"application/json", "Origin":"http://example.com"})
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=3)
        assert error.value.code == 403
        assert engine.state == "STOPPED"
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_single_left_then_right_keeps_other_arm_and_grippers(engine):
    left = engine.joints["left"] + [.1,.06,0,-.1,0,0,0]
    original = engine.home_qpos.copy()
    engine.command({"action": "move_joints", "arm": "left", "q": left})
    engine.advance(10)
    np.testing.assert_array_equal(engine.q, engine.arm.home)
    np.testing.assert_array_equal(engine.joints["left"], left)
    np.testing.assert_array_equal(engine.data.qpos[:7], left)
    engine.command({"action": "move_joints", "q": engine.q + [.1,0,0,.1,0,0,0]})
    engine.advance(10)
    np.testing.assert_array_equal(engine.joints["left"], left)
    np.testing.assert_array_equal(engine.data.qpos[9:16], engine.q)
    np.testing.assert_array_equal(engine.data.qpos[[7,8,16,17]], original[[7,8,16,17]])
    state = engine.snapshot()
    assert state["arms"]["left"]["q"] == left.tolist()
    assert state["q"] == state["arms"]["right"]["q"]
    assert state["q14"] == state["q"] + left.tolist()


def test_dual_synchronization_pause_stop_and_home(engine):
    start = {s: engine.joints[s].copy() for s in engine.SIDES}
    goal = {"right": start["right"] + [.1,.05,0,.08,0,0,0],
            "left": start["left"] + [-.3,.08,0,-.2,0,0,0]}
    result = engine.command({"action": "move_joints", "arm": "both", "q": goal,
                             "velocity_limits": [.2]*7+[.05]*7})
    assert result["active_arms"] == ["right", "left"]
    assert engine.duration == pytest.approx(1.875*.3/.05)
    engine.advance(engine.duration/2)
    for side in engine.SIDES:
        np.testing.assert_allclose(engine.joints[side], (start[side]+goal[side])/2)
    engine.command({"action": "pause"})
    paused = engine.snapshot()["q14"]
    engine.advance(10)
    assert engine.snapshot()["q14"] == paused
    for velocity in engine.velocities.values():
        np.testing.assert_array_equal(velocity, 0)
    with pytest.raises(ValueError, match="busy"):
        engine.command({"action": "home", "arm": "left"})
    engine.command({"action": "resume"})
    engine.advance(100)
    assert engine.state == "COMPLETED"
    for side in engine.SIDES:
        np.testing.assert_array_equal(engine.joints[side], goal[side])
    engine.command({"action": "home", "arm": "both"})
    engine.advance(.3)
    stopped = engine.command({"action": "stop"})
    engine.advance(100)
    assert engine.snapshot()["q14"] == stopped["q14"]
    engine.command({"action": "home", "arm": "both"})
    engine.advance(100)
    for side in engine.SIDES:
        np.testing.assert_array_equal(engine.joints[side], start[side])


def test_dual_pose_preview_execution_and_atomic_failure(engine, monkeypatch):
    from openarm_app.kinematics import pose_error, IKConvergenceError
    targets = {s: engine.arms[s].forward(engine.joints[s] + [.1,.05,-.06,.08,.03,-.04,.03])
               for s in engine.SIDES}
    before = engine.snapshot()
    preview = engine.command({"action": "preview_pose", "arm": "both", "pose": targets})
    assert engine.snapshot() == before
    engine.command({"action": "move_pose", "arm": "both", "pose": targets})
    engine.advance(10)
    for side in engine.SIDES:
        np.testing.assert_allclose(engine.joints[side], preview["q_goal"][side], atol=1e-6)
        pe, oe = pose_error(engine.arms[side].forward(engine.joints[side]), targets[side])
        assert pe <= .002 and oe <= np.deg2rad(2)
    before = engine.snapshot()
    def failed(*args, **kwargs):
        raise IKConvergenceError("test left IK failed")
    monkeypatch.setattr(engine.arms["left"], "inverse", failed)
    with pytest.raises(IKConvergenceError):
        engine.command({"action": "move_pose", "arm": "both", "pose": targets})
    assert engine.snapshot() == before


def test_left_pose_endpoint_keeps_right(engine):
    from openarm_app.kinematics import pose_error
    target = engine.arms["left"].forward(engine.joints["left"] + [.1,.05,0,-.1,0,.06,0])
    right = engine.q.copy()
    preview = engine.command({"action": "preview_pose", "arm": "left", "pose": target})
    assert preview["arm"] == "left"
    assert len(preview["q_goal"]) == 7
    engine.command({"action": "move_pose", "arm": "left", "pose": target})
    engine.advance(10)
    np.testing.assert_array_equal(engine.q, right)
    pe, oe = pose_error(engine.arms["left"].forward(engine.joints["left"]), target)
    assert pe <= .002 and oe <= np.deg2rad(2)


@pytest.mark.parametrize("payload", [
    {"action":"home", "arm":"invalid"},
    {"action":"move_joints", "arm":"both", "q":[0]*14},
    {"action":"move_joints", "arm":"both", "q":{"right":[0]*7}},
    {"action":"move_joints", "arm":"both", "q":{"right":[0]*7,"left":[100]*7}},
    {"action":"move_joints", "arm":"left", "q":[float("nan")]*7},
    {"action":"home", "arm":"both", "velocity_limits":[.2]*7},
])
def test_dual_rejects_invalid_without_changing_either_arm(engine, payload):
    before = engine.snapshot()
    with pytest.raises(ValueError):
        engine.command(payload)
    assert engine.snapshot() == before
