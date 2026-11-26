"""Monitor integration tests. The PCAN-mode cases inject a driver fixture."""
from dataclasses import asdict, replace
from io import BytesIO
import json
import math
import struct
import time
from unittest.mock import Mock

import pytest

from openarm_app.kinematics import pose_error
from openarm_driver import DriverError
from openarm_joint_mapping import load_mapping
from openarm_monitor import CHANNELS, ROOT, Conflict, Monitor, create_server
from openarm_monitor_motion import MotionPlanner
from openarm_monitor_modes import MODE_CODES
from openarm_piano_air import attacks, compile_air, load_notes
from openarm_piano_origin import load_origin
from openarm_position_mode import REGISTERS
from test_openarm_execution_hardware import BatchDriver


OPERATOR, OBSERVER = "a" * 32, "b" * 32


class MonitorDriverFixture(BatchDriver):
    instances = []
    events = []
    drift_before_enable = False
    fail_disable = False

    def __init__(self, *args):
        super().__init__(*args)
        self.snapshots = 0
        self.positions[4] = 1.570796

    def status(self):
        return 0

    def snapshot(self):
        self.snapshots += 1
        if self.drift_before_enable and self.snapshots == 2:
            self.positions[7] = .03
        return dict(side=self.side, online=7, bus_status="0x0",
                    motors=[asdict(self.query_motor(j)) for j in range(1, 8)])

    def disable_motor(self, motor_id):
        if self.fail_disable:
            raise DriverError("fixture disable timeout")
        return super().disable_motor(motor_id)


def wait_for(monitor, predicate, client=OPERATOR, *, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = monitor.status(0, client)
        if predicate(snapshot):
            return snapshot
        time.sleep(.02)
    pytest.fail(f"Monitor did not reach expected state: {snapshot['run']}")


def finished(snapshot):
    return snapshot["run"] and not snapshot["run"]["active"]


def holding(snapshot):
    return snapshot["run"] and snapshot["run"]["state"] == "HOLDING_TARGET"


def joint_target(degrees=2, duration=1., speed=20):
    return dict(arm="left", mode="joint", targets={"left": [None] * 6 + [degrees]},
                duration_s=duration, speed_percent=speed)


@pytest.fixture
def monitor_factory(tmp_path, monkeypatch):
    monitors = []
    monkeypatch.setattr(MonitorDriverFixture, "instances", [])
    monkeypatch.setattr(MonitorDriverFixture, "events", [])
    monkeypatch.setattr(MonitorDriverFixture, "bad_feedback_key", None)
    monkeypatch.setattr(MonitorDriverFixture, "send_failure_key", None)

    def start(mode, driver_factory=MonitorDriverFixture, **kwargs):
        monitor = Monitor(mode, CHANNELS, tmp_path, driver_factory=driver_factory, **kwargs)
        monitor.start()
        monitors.append(monitor)
        return monitor

    yield start
    for monitor in monitors:
        monitor.close()


def records(monitor, request_id):
    return [row for line in monitor.journal.path.read_text(encoding="utf-8").splitlines()
            if (row := json.loads(line))["request_id"] == request_id]


def test_piano_air_measured_wrist_anchor_is_used_instead_of_csv(monitor_factory, tmp_path):
    monitor = monitor_factory("virtual")
    calibration = tmp_path / "piano.json"
    calibration.write_text(json.dumps(dict(
        anchor_q_rad={side: arm.home.tolist() for side, arm in monitor.planner.arms.items()},
        key_step_m=[0., -.0235, 0.])), encoding="utf-8")
    air = compile_air(monitor.planner.arms, scope="start", speed=.35, score_id="jin-chords",
                      calibration_path=calibration)
    assert air["source"]["site_calibration_sha256"]
    for side, arm in monitor.planner.arms.items():
        assert air["waypoints"][0][1][side] == pytest.approx(arm.home)


def test_piano_air_reads_last_preparation_hand_positions_from_score(tmp_path):
    score = tmp_path / "score.txt"
    score.write_text("left_arm_move(18,4)\nright_arm_move(29,2)\nsleep(2)\n"
                     "right_arm_move(25,2)\nsleep(2)\nhand(D,7)\nsleep(.5)\nhand(U,7)\n",
                     encoding="utf-8")
    notes, _, _, start_keys = load_notes(score)
    assert start_keys == {"left": 18, "right": 25}
    assert notes[0]["key"] == 25


@pytest.mark.parametrize(
    ("side", "start_deg", "offset_mm"),
    [
        pytest.param("left", (0, 0, -80, 0, 0, 0, 10), (0, 0, 50), id="left-limit-edge"),
        pytest.param("right", (129, -49, 2, 119, -10, 4, 18), (-175, 175, -150),
                     id="right-stalled-iteration"),
    ],
)
def test_wrist_ik_preview_recovers_from_current_seed_failure(side, start_deg, offset_mm):
    version, mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json", hardware=True)
    planner = MotionPlanner(version, mappings, virtual=False)
    arm = planner.arms[side]
    seed = [math.radians(angle) for angle in start_deg]
    current = planner.home | {(side, j): angle for j, angle in enumerate(seed, 1)}
    wrist = planner.endpoints(current)[side]["wrist"]
    desired = [coordinate + offset for coordinate, offset in zip(wrist, offset_mm)]
    pose = arm.forward(seed)
    pose[:3] = [coordinate / 1000 for coordinate in desired]

    plan, _ = planner.prepare(
        dict(arm=side, mode="ik", endpoint="wrist", targets={side: desired},
             duration_s=3, speed_percent=50),
        current,
    )

    expected_keys = tuple((side, j) for j in range(1, 8))
    assert plan["commanded"] == expected_keys
    result, _ = plan["trajectory"].sample(3)
    position_error, orientation_error = pose_error(arm.forward(result), pose)
    assert position_error <= .001
    assert orientation_error <= .003
    home_solution = arm.inverse(pose, arm.home, max_iterations=120, position_tolerance=.001,
                                orientation_tolerance=.003)
    assert math.dist(seed, result) <= math.dist(seed, home_solution) + 1e-9


def test_bilateral_wrist_pose_ik_uses_feedback_for_blank_axes():
    version, mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json", hardware=True)
    planner = MotionPlanner(version, mappings, virtual=False)
    _, anchors, _ = load_origin()
    current = {(side, joint): math.radians(angle) for side, anchor in anchors.items()
               for joint, angle in enumerate(anchor.deg, 1)}
    points, rpy = planner.end_effector_feedback(current)
    right_y = points["right"]["wrist"][1] - 5
    left_roll = rpy["left"][0] + 2.5
    right_yaw = rpy["right"][2] + 2.5
    plan, _ = planner.prepare(
        dict(arm="both", mode="ik", endpoint="wrist", duration_s=1., speed_percent=20,
             targets={"left": [None, None, None, left_roll, None, None],
                      "right": [None, right_y, None, None, None, right_yaw]}),
        current,
    )
    assert len(plan["commanded"]) == 14
    final_points, final_rpy = planner.end_effector_feedback(plan["goal"])
    assert final_points["left"]["wrist"] == pytest.approx(points["left"]["wrist"], abs=1.)
    right_x, _, right_z = points["right"]["wrist"]
    assert final_points["right"]["wrist"] == pytest.approx([right_x, right_y, right_z], abs=1.)
    assert final_rpy["left"] == pytest.approx([left_roll, *rpy["left"][1:]], abs=.2)
    assert final_rpy["right"] == pytest.approx([*rpy["right"][:2], right_yaw], abs=.2)


@pytest.mark.parametrize("run", [1, 2])
def test_legacy_right_trial_uses_current_points_and_preserves_notes(monitor_factory, run):
    from bisect import bisect_right
    from collections import Counter
    from openarm_piano_air import CHORD_SCORE
    from paino_skill.score import WHITES

    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda snapshot: snapshot["can_prepare"])
    speed = .35
    monitor.prepare(dict(mode="piano-trial", anchor_run=run, scope="full", speed=speed), OPERATOR)
    plan = monitor.preview[0]
    info, score, path = plan["piano_trial"], plan["trial_fingers"], plan["trajectory"]
    _, anchors, digest = load_origin()
    assert info["points_sha256"] == digest
    assert set(path.keys) == {("right", j) for j in range(1, 8)}
    assert info["note_count"] == 694
    assert info["origin_targets"]["right"]["joints_deg"] == anchors["right"].deg
    assert path.sample(3.25)[0] == pytest.approx(list(map(math.radians, anchors["right"].deg)))
    notes, _, _, _ = load_notes(CHORD_SCORE)
    intro = round(info["prepare_s"] * 1e6)
    expected = Counter()
    for note in (n for n in notes if n["side"] == "right"):
        start = note["on_us"]
        on = intro + round(start / speed) + bisect_right(info["pause_times_us"], start) * 1_000_000
        expected[on, True, note["midi"]] += 1
        expected[on + round((note["off_us"] - start) / speed), False, note["midi"]] += 1
    actual = Counter()
    for event in score.events["right"][1:]:
        joints = dict(zip(path.keys, path.sample(event.time_us / 1e6)[0]))
        y = monitor.planner.endpoints(joints)["right"]["wrist"][1]
        wrist = anchors["right"].key + round((anchors["right"].xyz[1] - y) / 22.5)
        for channel, down in event.changes:
            actual[event.time_us, down, WHITES[wrist + channel - 4]] += 1
    assert actual == expected


def test_calibrated_trial_held_start_and_stop_release(monitor_factory, profile_path):
    from test_openarm_monitor_hands import FixtureHand

    m = monitor_factory("hardware", ProfileDriverFixture, profile_path=profile_path)
    m.hands.factory = FixtureHand
    wait_for(m, lambda s: s["can_enable_all"])
    m.switch_mode("POS_VEL")
    wait_for(m, lambda s: s["can_enable_all"] and s["drive_mode"] == "POS_VEL")
    session = m.hands.action(dict(action="connect", side="right", demo=True))["session"]
    bus = m.hands.links["right"].bus
    m.enable_all(OPERATOR)
    wait_for(m, holding)
    old_left = {k: q for k, q in m.job.plan["goal"].items() if k[0] == "left"}
    args = dict(mode="piano-trial", anchor_run=2, scope="start", speed=1.)
    start = m.prepare(args, OPERATOR)
    assert all(all(target[k] == q for k, q in old_left.items()) for _, _, target in m.preview[0]["steps"])
    m.start_motion(start["request_id"], OPERATOR)
    wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == start["request_id"])
    assert not bus.packets
    neighbour = m.prepare(dict(args, key=25), OPERATOR)
    assert neighbour["piano_trial"]["wrist_keys"] == [25]
    assert len(m.preview[0]["steps"]) == 1
    endpoint = m.planner.endpoints(m.preview[0]["goal"])["right"]["wrist"]
    assert endpoint == pytest.approx([250.25, -77.5, -170.00], abs=.02)
    preview = m.prepare(dict(args, scope="first"), OPERATOR)
    assert preview["piano_trial"]["from_score_start"] and preview["duration_s"] == pytest.approx(2.25)
    m.start_motion(preview["request_id"], OPERATOR, {"right": session})
    wait_for(m, lambda s: len(bus.packets) >= 2)
    assert [j for j, (a,b) in enumerate(zip(*bus.packets[:2])) if a != b] == [3,5]
    m.stop_motion(preview["request_id"])
    wait_for(m, lambda s: finished(s) and not s["hand_score"]["active"])
    assert bus.packets[-1] == bus.packets[0]


def test_trial_without_hand_session_still_runs_the_arm(monitor_factory):
    m = monitor_factory("virtual")
    wait_for(m, lambda s: s["can_prepare"])
    m.enable_all(OPERATOR)
    wait_for(m, holding)
    start = m.prepare(dict(mode="piano-trial", anchor_run=2, scope="start", speed=1.), OPERATOR)
    m.start_motion(start["request_id"], OPERATOR)
    wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == start["request_id"])
    preview = m.prepare(dict(mode="piano-trial", anchor_run=2, scope="first", speed=1.), OPERATOR)
    assert preview["piano_trial"]["from_score_start"]
    # No hand session at all: the arm still plays the wrist path, fingers are skipped.
    m.start_motion(preview["request_id"], OPERATOR)
    done = wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    assert not done["hand_score"]["active"]
    assert done["run"]["piano_trial"]["finger_notice"]
    assert any(row["event"] == "hand.score.skipped" for row in m.journal.events)
    m.stop_motion(done["run"]["session_id"])
    wait_for(m, finished)
    assert not m.hands.snapshot()["left"]["connected"]


@pytest.mark.parametrize(("score_id", "scope", "count"), [("jin-chords", "full", 1300), ("jin-full", "prelude", 37),
                                                        ("jin-full", "full", 1469)])
def test_dual_trial_keeps_physical_keys_and_note_times(monitor_factory, score_id, scope, count):
    from bisect import bisect_right
    from collections import Counter
    from openarm_piano_performance import CHANNEL, fingering_preview
    from openarm_start_pose import START_DEG

    monitor = monitor_factory("virtual")
    speed = .35
    current = {(side, j): math.radians(q) for side in ("left", "right") for j, q in enumerate(START_DEG, 1)}
    plan, _ = monitor.planner.prepare_piano_trial(dict(anchor_run=3, score_id=score_id, scope=scope, speed=speed),
                                                  current, {"goal": current})
    score, trajectory = plan["trial_fingers"], plan["trajectory"]
    source = fingering_preview(scope, score_id, monitor.mappings)
    points, anchors, _ = load_origin()
    assert score.note_count == count
    assert plan["duration"] == pytest.approx(score.duration_us / 1e6, abs=1e-6)
    assert trajectory.sample(3.25)[0] == pytest.approx(
        [math.radians(anchors[side].deg[j-1]) for side, j in trajectory.keys])
    intro_us = round(plan["piano_trial"]["prepare_s"] * 1e6)
    pauses = plan["piano_trial"]["pause_times_us"]
    first = source["rows"][0]["time_us"]
    expected = Counter()
    for row in source["rows"]:
        relative = row["time_us"] - first
        on = intro_us + round(relative / speed) + bisect_right(pauses, relative) * 1_000_000
        for side, notes in row["fingers"].items():
            for note in notes:
                off = on + round((note["off_us"] - row["time_us"]) / speed)
                channel = CHANNEL[note["finger"]]
                expected[side, on, channel, True] += 1
                expected[side, off, channel, False] += 1
                q = [math.radians(deg) for deg in points[side][row["wrists"][side]].deg]
                for stamp in (on, (on + off) // 2, off):
                    actual = [value for key, value in zip(trajectory.keys, trajectory.sample(stamp / 1e6)[0])
                              if key[0] == side]
                    assert actual == pytest.approx(q, abs=1e-8)
    assert Counter((side, e.time_us, ch, down) for side in score.sides
                   for e in score.events[side][1:] for ch, down in e.changes) == expected


def test_dual_trial_http_stop_lifts_fingers_and_keeps_hands_connected(monitor_factory):
    from test_openarm_monitor_hands import FixtureHand

    monitor = monitor_factory("virtual")
    monitor.hands.factory = FixtureHand
    sessions = {side: monitor.hands.action(dict(action="connect", side=side, demo=True))["session"]
                for side in ("left", "right")}
    buses = {side: monitor.hands.links[side].bus for side in sessions}
    wait_for(monitor, lambda s: s["can_prepare"])
    points, _, _ = load_origin()
    first_pose = monitor.prepare(dict(arm="both", mode="joint", duration_s=.12, speed_percent=20,
                                      targets={"left": list(points["left"][22].deg),
                                               "right": list(points["right"][24].deg)}), OPERATOR)
    monitor.start_motion(first_pose["request_id"], OPERATOR)
    wait_for(monitor, holding)
    trial = monitor.prepare(dict(mode="piano-trial", anchor_run=3, scope="first", speed=1.,
                                 score_id="jin-chords"), OPERATOR)
    assert trial["piano_trial"]["from_score_start"]
    assert trial["piano_trial"]["prepare_s"] == .25
    assert all(not bus.packets for bus in buses.values())
    monitor.start_motion(trial["request_id"], OPERATOR, sessions)
    wait_for(monitor, lambda s: all(len(bus.packets) >= 2 for bus in buses.values()) and
             any(row["event"] == "hand.score.frame" and row["data"]["time_us"] == 250_000
                 for row in monitor.journal.events))
    for side, channels in (("left", [3, 5]), ("right", [3, 5])):
        assert [ch for ch, (before, after) in enumerate(zip(*buses[side].packets[:2])) if before != after] == channels
    with create_server(monitor, 0) as server:
        host = f"127.0.0.1:{server.server_port}"
        body = json.dumps({"request_id": trial["request_id"]}).encode()
        headers = (f"POST /api/stop HTTP/1.0\r\nHost: {host}\r\nOrigin: http://{host}\r\n"
                   f"X-OpenArm-Client: {OPERATOR}\r\nContent-Type: application/json\r\n"
                   f"Content-Length: {len(body)}\r\n\r\n").encode()
        request = Mock()
        request.makefile.return_value = BytesIO(headers + body)
        server.RequestHandlerClass(request, ("127.0.0.1", 0), server)
        response = b"".join(call.args[0] for call in request.sendall.call_args_list)
        assert response.startswith(b"HTTP/1.0 200"), response.decode()
    stopped = wait_for(monitor, lambda s: finished(s) and not s["hand_score"]["active"])
    assert stopped["run"]["stop_confirmed"]
    assert all(arm["disabled"] for arm in stopped["arms"].values())
    assert all(bus.packets[-1] == bus.packets[0] for bus in buses.values())
    for side, session in sessions.items():
        assert stopped["hands"][side]["connected"] and stopped["hands"][side]["session"] == session
        previous_frames = len(buses[side].packets)
        hand = monitor.hands.action(dict(side=side, session=session, action="move", channel=3, degrees=20))
        assert len(buses[side].packets) == previous_frames + 1
        assert hand["feedback"]["q"][3] == pytest.approx(20, abs=.1)


def test_play_piano_linked_arms_and_hands_after_two_second_wait(monitor_factory):
    from openarm_piano_performance import PRIORITY_SCORES, SCORES, performance_options
    from test_openarm_monitor_hands import FixtureHand

    monitor = monitor_factory("virtual")
    monitor.hands.factory = FixtureHand
    sessions = {side: monitor.hands.action(dict(action="connect", side=side, demo=True))["session"]
                for side in ("left", "right")}
    wait_for(monitor, lambda s: s["can_prepare"])
    _, anchors, _ = load_origin()
    targets = {side: list(point.deg) for side, point in anchors.items()}
    home = monitor.prepare(dict(arm="both", mode="joint", targets=targets, duration_s=.12, speed_percent=20), OPERATOR)
    monitor.start_motion(home["request_id"], OPERATOR)
    snapshot = wait_for(monitor, holding)
    assert list(snapshot["piano_performance_scores"]) == list(SCORES)
    assert list(SCORES)[:5] == list(PRIORITY_SCORES)
    assert "千本樱" in SCORES and "处理汇总" not in SCORES
    assert snapshot["piano_performance_scores"]["jin-full"]["available"]
    assert performance_options()["jin-full"]["available"]
    assert performance_options()["little-apple"]["wait_s"] == 2.
    # 控制台据此提示缺哪只手，必须与该谱面实际写出的左右手一致。
    for score_id, option in performance_options().items():
        notes = load_notes(SCORES[score_id])[0]
        assert option["sides"] == [side for side in ("left", "right")
                                   if any(note["side"] == side for note in notes)]

    run = monitor.play_piano(dict(score_id="jin-chords", sessions=sessions), OPERATOR)
    info = run["piano_trial"]
    assert info["speed"] == 1 and info["wait_s"] == 2 and info["prepare_s"] == 5.25
    expected = {"left": ((3, True), (5, True)), "right": ((3, True), (5, True))}
    wait_for(monitor, lambda s: any(row["request_id"] == run["request_id"] and
                                  row["event"] == "hand.score.frame" and row["data"]["changes"] == expected
                                  for row in monitor.journal.events), timeout=8)
    rows = [row["data"] for row in monitor.journal.events if row["request_id"] == run["request_id"]
            and row["event"] == "hand.score.frame"]
    assert [r["time_us"] for r in rows[:2]] == [0, 5_250_000]

    buses = {side: monitor.hands.links[side].bus for side in sessions}
    assert all(bus.packets[-1] != bus.packets[0] for bus in buses.values())
    with create_server(monitor, 0) as server:
        host = f"127.0.0.1:{server.server_port}"
        body = json.dumps({"request_id": run["request_id"]}).encode()
        headers = (f"POST /api/piano/exit HTTP/1.0\r\nHost: {host}\r\nOrigin: http://{host}\r\n"
                   f"X-OpenArm-Client: {OPERATOR}\r\nContent-Type: application/json\r\n"
                   f"Content-Length: {len(body)}\r\n\r\n").encode()
        request = Mock()
        request.makefile.return_value = BytesIO(headers + body)
        server.RequestHandlerClass(request, ("127.0.0.1", 0), server)
        response = b"".join(call.args[0] for call in request.sendall.call_args_list)
        assert response.startswith(b"HTTP/1.0 200"), response.decode()
    exited = wait_for(monitor, lambda s: s["run"]["piano_exited"] and not s["hand_score"]["active"])
    assert holding(exited) and exited["all_enabled"] and exited["can_prepare"]
    assert exited["run"]["stop_reason"] is None
    assert all(bus.packets[-1] == bus.packets[0] for bus in buses.values())
    assert {side: hand["session"] for side, hand in exited["hands"].items()} == sessions

    for side, session in sessions.items():
        monitor.hands.action(dict(side=side, session=session, action="press", channel=3))
    replay = monitor.play_piano(dict(score_id="jin-chords", sessions=sessions), OPERATOR)
    assert replay["session_id"] == run["session_id"]
    with pytest.raises(Conflict):
        monitor.exit_piano(run["request_id"])
    monitor.exit_piano(replay["request_id"])
    exited_again = wait_for(monitor, lambda s: s["run"]["request_id"] == replay["request_id"]
                            and s["run"]["piano_exited"] and not s["hand_score"]["active"])
    for side in sessions:
        assert exited_again["run"]["target_deg"][side] == pytest.approx(exited["run"]["target_deg"][side], abs=1e-4)
        assert buses[side].packets[-1] == buses[side].packets[0]
    assert exited_again["all_enabled"]
    monitor.stop_motion(replay["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_piano_waits_for_fingers_and_late_wrist_without_failing_motion(monitor_factory, monkeypatch, tmp_path):
    from threading import Event
    from openarm_piano_performance import SCORES
    from test_openarm_monitor_hands import FixtureHand

    score = tmp_path / "two-chords.txt"
    score.write_text("left_arm_move(19, .2)\nright_arm_move(24, .2)\nsleep(4)\n"
                     "hand(D,1234789x)\nsleep(.25)\nhand(U,1234789x)\n"
                     "right_arm_move(25, .16)\nsleep(.16)\n"
                     "hand(D,1234789x)\nsleep(.25)\nhand(U,1234789x)\nsleep(.2)\n", encoding="ascii")
    monkeypatch.setitem(SCORES, "little-apple", score)

    wrist_lag, finger_lag = Event(), Event()
    frozen = {}

    class SlowWrist(BaseSpecialDriverFixture):
        def query_motor(self, j):
            state = super().query_motor(j)
            if self.side == "right" and wrist_lag.is_set():
                return replace(state, position_rad=frozen[j])
            return state

    class SlowFinger(FixtureHand):
        def read(self, **kw):
            result = super().read(**kw)
            if finger_lag.is_set() and len(self.packets) >= 3:
                result["q"][3] += 6.
            return result

    monitor = monitor_factory("hardware", SlowWrist)
    monitor.hands.factory = SlowFinger
    sessions = {s: monitor.hands.action(dict(action="connect", side=s, demo=True))["session"]
                for s in ("left", "right")}
    wait_for(monitor, lambda s: s["can_prepare"] and s["drive_mode"] == "POS_VEL")
    run = monitor.play_piano(dict(score_id="little-apple", sessions=sessions), OPERATOR)
    request_id = run["request_id"]
    target = run["piano_trial"]["first_wrist_targets"]["right"]["joints_deg"]
    frozen.update({j: monitor.mappings["right", j].to_motor(math.radians(q))
                   for j, q in enumerate(target, 1)})

    def frame_rows():
        return [row["data"] for row in monitor.journal.events
                if row["request_id"] == request_id and row["event"] == "hand.score.frame"]

    def presses():
        return [row for row in frame_rows() if any(d for changes in row["changes"].values() for _, d in changes)]

    def wait_state(snapshot):
        return (snapshot["run"]["sample"] or {}).get("piano_sync") or {}

    wait_for(monitor, lambda s: presses(), timeout=9)
    assert all(len(changes) == 4 and not any(d for _, d in changes)
               for changes in frame_rows()[0]["changes"].values())
    finger_lag.set()
    wrist_lag.set()
    lifting = wait_for(monitor, lambda s: wait_state(s).get("finger_error_deg", {}).get("right", 0) > 5)
    assert lifting["run"]["sample"]["command_deg"]["right"] == pytest.approx(target)
    assert lifting["all_enabled"] and len(presses()) == 1

    finger_lag.clear()
    arriving = wait_for(monitor, lambda s: wait_state(s).get("wrist_errors", {}).get("right", {}).get("distance_mm", 0) > 20)
    assert arriving["all_enabled"] and len(presses()) == 1
    right_driver = monitor.drivers["right"]
    assert next(speed for joint, _, speed in reversed(right_driver.frames) if joint == 1) == pytest.approx(4.4)

    wrist_lag.clear()
    complete = wait_for(monitor, lambda s: holding(s) and not s["hand_score"]["active"])
    assert complete["all_enabled"] and len(presses()) == 2
    assert set(presses()[1]["changes"]) == {"left", "right"}
    ready = next(row["data"] for row in monitor.journal.events if row["request_id"] == request_id
                 and row["event"] == "piano.sync.ready" and row["data"]["time_us"] == presses()[1]["time_us"])
    assert ready["wrist_errors"]["right"]["distance_mm"] <= 5.
    assert ready["wall_elapsed_s"] <= presses()[1]["sent_s"]
    monitor.stop_motion(request_id)
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_little_apple_preparation_from_current_and_held_poses():
    from openarm_piano_performance import PERFORMANCE_SPEED, compile_performance

    version, mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json", hardware=True)
    _, anchors, _ = load_origin()
    current = {(side, j): math.radians(q) for side, point in anchors.items()
               for j, q in enumerate(point.deg, 1)}
    request = dict(mode="piano-performance", score_id="little-apple")
    planner = MotionPlanner(version, mappings, virtual=False)
    plan, _ = planner.prepare_piano_trial(request, current, None)
    info = plan["piano_trial"]
    assert info["approach_s"] == 3 and info["wait_s"] == 2 and info["prepare_s"] == 8.25
    assert info["speed"] == PERFORMANCE_SPEED["little-apple"]
    assert info["note_count"] == 259 and info["pause_times_us"] == []

    first_pose = {(side, j): math.radians(q) for side, target in info["first_wrist_targets"].items()
                  for j, q in enumerate(target["joints_deg"], 1)}
    held, _ = planner.prepare_piano_trial(request, current, {"goal": first_pose})
    assert not held["piano_trial"]["from_score_start"]
    assert held["piano_trial"]["prepare_s"] == 8.25
    path = held["trajectory"]
    assert path.sample(3.25)[0] == pytest.approx([current[k] for k in path.keys])
    assert path.sample(6.25)[0] == pytest.approx([first_pose[k] for k in path.keys])
    assert held["trial_fingers"].duration_us - 8_250_000 == round(63_904_297 / info["speed"])
    right_curve = next(arm for arm in path.arms if arm.keys[0][0] == "right")
    adjacent = next(move for move in held["side_paths"]["right"].transfers if move.profile.speed_override)
    begin, end = adjacent.start_us / 1e6, adjacent.finish_us / 1e6
    start_q = right_curve.sample(begin)[0]
    end_q = right_curve.sample(end)[0]
    midpoint = right_curve.sample((begin + end) / 2)[0]
    assert midpoint[-1] - (start_q[-1] + end_q[-1]) / 2 == pytest.approx(math.radians(2))
    before_end = right_curve.sample(begin + (end - begin) * .95)[0]
    axis = max(range(6), key=lambda j: abs(end_q[j] - start_q[j]))
    progress = (before_end[axis] - start_q[axis]) / (end_q[axis] - start_q[axis])
    assert before_end[-1] == pytest.approx(start_q[-1] + progress * (end_q[-1] - start_q[-1]))
    assert progress < 1

    # A slower joint must get extra approach time without changing the wait.
    key = "left", 4
    mappings[key] = replace(mappings[key], scale=.001)
    steps, slow, _, _ = compile_performance("little-apple", None, mappings)
    assert steps[2].duration_s == slow["approach_s"] > 3
    assert slow["prepare_s"] == pytest.approx(.25 + 3 + slow["approach_s"] + 2, abs=1e-6)


def test_full_performance_compiles_and_reports_skipped_notes(monitor_factory):
    from openarm_piano_performance import SCORES
    from openarm_start_pose import START_DEG

    monitor = monitor_factory("virtual")
    current = {(side, j): math.radians(q) for side in ("left", "right")
               for j, q in enumerate(START_DEG, 1)}
    score_id = "克罗地亚狂想曲"
    notes, _, _, _ = load_notes(SCORES[score_id])
    plan, _ = monitor.planner.prepare_piano_trial(
        dict(mode="piano-performance", score_id=score_id), current, None)
    info = plan["piano_trial"]
    assert info["score_id"] == score_id
    assert info["note_count"] + info["skipped_notes"] == len(notes)
    assert info["note_count"] == 964 and info["skipped_notes"] == 8
    assert {n["reason"] for n in info["skipped_details"]} == {"outside_physical_keyboard"}
    assert "已跳过" in info["skip_notice"]


def test_piano_origin_and_neighbour_reach_calibrated_targets(monitor_factory, profile_path):
    m = monitor_factory("hardware", ProfileDriverFixture, profile_path=profile_path)
    wait_for(m, lambda s: s["can_enable_all"])
    m.switch_mode("POS_VEL")
    wait_for(m, lambda s: s["can_enable_all"] and s["drive_mode"] == "POS_VEL")
    m.enable_all(OPERATOR)
    wait_for(m, holding)
    motion = m.start_pose_motion("piano-origin", OPERATOR)
    feedback = wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == motion["request_id"])["arms"]
    assert feedback["left"]["piano_finger_keys"] == {"1": 19, "2": 20, "3": 21, "4": 22}
    assert feedback["right"]["piano_finger_keys"] == {"7": 24, "8": 25, "9": 26, "x": 27}
    points, anchors, _ = load_origin()
    for side, ref, base in [("left", 22, (250.19, 56.1, -170.00)), ("right", 24, (250.25, -55.0, -170.00))]:
        assert motion["target_deg"][side] == pytest.approx(anchors[side].deg)
        assert anchors[side].key == ref and anchors[side].xyz == base
        assert feedback[side]["endpoints_mm"]["wrist"] == pytest.approx(base, abs=.02)
        other = "left" if side == "right" else "right"
        adjacent = ref + 1
        args = dict(mode="piano-trial", anchor_run=3, side=side, key=adjacent, scope="start")
        p = m.prepare(args, OPERATOR)
        assert p["target_deg"][side] == pytest.approx(points[side][adjacent].deg)
        assert p["target_deg"][other] == pytest.approx(anchors[other].deg)
        assert p["piano_trial"]["physical_key"] == adjacent-9
        assert m.preview[0]["trial_fingers"] is None
    with pytest.raises(ValueError, match="有固定腕姿逆解"):
        m.prepare(dict(args, key=45), OPERATOR)
    m.stop_motion(motion["request_id"])
    wait_for(m, finished)


def test_bilateral_fingering_preview_respects_hand_split_and_pending_wrists():
    from openarm_piano_origin import OFFSETS
    from openarm_piano_air import CHORD_SCORE
    from openarm_piano_performance import fingering_preview
    _, mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json", hardware=True)
    result = fingering_preview("full", "jin-chords", mappings)
    notes, duration, _, _ = load_notes(CHORD_SCORE)
    assert len(result["rows"]) == 347 and result["duration_us"] == duration
    assert result["note_count"] == len(notes) == 1300
    points, _, _ = load_origin()
    for (onset, chord), row in zip(attacks(notes), result["rows"], strict=True):
        assert [(onset, n["key"], n["midi"], n["off_us"]) for n in chord] == [
            (row["time_us"], n["key"], n["midi"], n["off_us"]) for n in row["notes"]]
        assigned = []
        for side in ("left", "right"):
            key = row["wrists"][side]
            lo, hi = (10, 23) if side == "left" else (24, 45)
            assert lo <= key <= hi
            assert (side in row["pending_wrists"]) == (key not in points[side])
            for f in row["fingers"][side]:
                assert lo <= f["key"] <= hi
                assert OFFSETS[side][f["key"] - key] == f["finger"]
                assigned.append((f["midi"], f["off_us"]))
        assert sorted(assigned) == sorted((n["midi"], n["off_us"]) for n in chord)
        assert (row["wrist_distance_mm"] is None) == bool(row["pending_wrists"])
    assert result["unassigned"] == 0
    assert result["pending_wrist_groups"] == sum(bool(r["pending_wrists"]) for r in result["rows"]) == 0
    first = result["rows"][0]
    assert first["wrists"] == {"left": 22, "right": 24}
    assert first["pending_wrists"] == []
    assert [(f["midi"], f["finger"]) for f in first["fingers"]["right"]] == [(60, "index"), (64, "ring")]
    assert fingering_preview("first", "jin-chords", mappings)["rows"] == result["rows"][:1]
    assert fingering_preview("prelude", "jin-chords", mappings)["rows"] == [
        r for r in result["rows"] if r["time_us"] < 16_000_000]


def test_full_piano_score_preview_calculates_only_one_first_attack():
    from openarm_piano_performance import SCORES, fingering_preview

    _, mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json", hardware=True)
    notes, duration, digest, _ = load_notes(SCORES["jin-full"])
    assert notes[4]["on_us"] == 500_000
    first = fingering_preview("first", "jin-full", mappings)
    assert first["score_sha256"] == digest
    assert len(first["rows"]) == 1 and first["note_count"] == 4
    row = first["rows"][0]
    assert row["time_us"] == 0 and row["wrists"] == {"left": 22, "right": 24}
    assert row["pending_wrists"] == [] and first["unassigned"] == 0
    assert [(n["midi"], n["finger"]) for n in row["fingers"]["left"]] == [(53, "ring"), (57, "index")]
    assert [(n["midi"], n["finger"]) for n in row["fingers"]["right"]] == [(60, "index"), (64, "ring")]
    points, _, _ = load_origin()
    for side, key in row["wrists"].items():
        target = first["first_wrist_targets"][side]
        assert target["xyz_mm"] == points[side][key].xyz
        assert target["joints_deg"] == points[side][key].deg

    full = fingering_preview("full", "jin-full", mappings)
    assert full["duration_us"] == duration
    assert full["note_count"] == len(notes) == 1469
    assert full["rows"][0] == row


def test_virtual_ik_has_distinct_targets_and_observations(monitor_factory):
    monitor = monitor_factory("virtual")
    status = wait_for(monitor, lambda s: s["can_prepare"])
    points = status["arms"]["left"]["endpoints_mm"]
    assert points["wrist"] != points["index"]
    desired = points["index"].copy()
    desired[0] += 5
    preview = monitor.prepare(dict(arm="left", mode="ik", endpoint="index",
                                   targets={"left": desired}, duration_s=1., speed_percent=20), OPERATOR)
    assert monitor.status(0, OPERATOR)["arms"]["left"]["endpoints_mm"]["index"] == points["index"]
    monitor.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(monitor, holding)
    assert result["run"]["active"] and not result["run"]["stop_confirmed"]
    assert result["arms"]["left"]["endpoints_mm"]["index"] == pytest.approx(desired, abs=2)
    rows = records(monitor, preview["request_id"])
    sample = next(row["data"] for row in rows if row["event"] == "motion.sample")
    assert len(sample["feedback"]) == 14 and sample["error"] == [0] * len(sample["error"])
    assert any(row["event"] == "drive.holding" for row in rows)
    monitor.stop_motion(preview["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_hardware_path_reuses_mapping_and_reports_confirmed_disable(monitor_factory):
    monitor = monitor_factory("hardware")
    status = wait_for(monitor, lambda s: s["can_prepare"])
    assert set(status["arms"]["left"]["endpoints_mm"]) == {"wrist"}
    # The former speed/acceleration percentage refusal is gone: 6 degrees in
    # 0.5 s is accepted and planned with whatever velocity/acceleration it needs.
    fast = monitor.prepare(joint_target(6, .5), OPERATOR)
    assert fast["duration_s"] == .5
    assert fast["target_deg"]["left"][6] == pytest.approx(6)
    preview = monitor.prepare(joint_target(15, duration=2), OPERATOR)
    assert all(not arm.gains for arm in MonitorDriverFixture.instances)
    with pytest.raises(Conflict):
        monitor.start_motion(preview["request_id"], OBSERVER)
    monitor.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(monitor, holding)
    assert result["arms"]["left"]["joints"][6]["position_deg"] == pytest.approx(15, abs=.03)
    assert result["arms"]["left"]["joints"][6]["mos_c"] == 25
    left, right = MonitorDriverFixture.instances
    assert left.positions[7] < 0  # logical left J7 goes in the opposite motor direction
    assert set(left.gains) == {7} and not right.gains
    assert left.enabled_ids == {7}
    monitor.stop_motion(preview["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]
    assert not left.enabled_ids
    with pytest.raises(Conflict):
        monitor.start_motion(preview["request_id"], OPERATOR)
    assert any(row["event"] == "motion.after" for row in records(monitor, preview["request_id"]))


def test_start_pose_drift_is_adopted_before_enable(monitor_factory, monkeypatch):
    monkeypatch.setattr(MonitorDriverFixture, "drift_before_enable", True)
    monitor = monitor_factory("hardware")
    wait_for(monitor, lambda s: s["can_prepare"])
    preview = monitor.prepare(joint_target(), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(monitor, holding)
    assert result["run"]["state"] == "HOLDING_TARGET"
    rows = records(monitor, preview["request_id"])
    preflight = next(row["data"] for row in rows if row["event"] == "motion.preflight")
    # Left J7 maps logical model +q to negative motor radians, so a +.03 motor
    # drift is adopted as -.03 model radians at execution time.
    assert preflight["trajectory"]["initial_model_position"][0] == pytest.approx(-.03)
    assert not any(row["event"] == "motion.fault" for row in rows)
    monitor.stop_motion(preview["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_axis_resting_on_hard_stop_does_not_refuse_the_target(monitor_factory):
    # J4's lower model limit is exactly 0 deg, so an axis held on that stop
    # reads about one encoder count below it. The measured start pose used to
    # reject the whole command, even a 90 deg target well inside the range.
    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    key = ("right", 4)
    monitor.positions[key] = -.0001907377737087046
    wait_for(monitor, lambda s: s["arms"]["right"]["joints"][3]["position_deg"] < 0)
    payload = dict(arm="right", mode="joint", duration_s=1, speed_percent=20,
                   targets={"right": [None, None, None, 90, None, None, None]})
    preview = monitor.prepare(payload, OPERATOR)
    assert preview["target_deg"]["right"][3] == pytest.approx(90)
    monitor.start_motion(preview["request_id"], OPERATOR)
    done = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    assert done["arms"]["right"]["joints"][3]["position_deg"] == pytest.approx(90, abs=.1)
    rows = records(monitor, preview["request_id"])
    assert not any(row["event"] == "motion.fault" for row in rows)
    preflight = next(row["data"] for row in rows if row["event"] == "motion.preflight")
    assert preflight["trajectory"]["initial_model_position"][0] == pytest.approx(
        -.0001907377737087046)


@pytest.mark.parametrize("lag_at_goal", [False, True], ids=["catches_up", "misses_endpoint"])
def test_hardware_tracking_lag_is_advisory_and_holds(monitor_factory, lag_at_goal):
    class LaggingDriver(MonitorDriverFixture):
        def send(self, motor_id, payload):
            super().send(motor_id, payload)
            if self.side == "left" and motor_id == 7:
                position = self.positions[motor_id]
                if position < -.10 and (lag_at_goal or position > -.16):
                    self.positions[motor_id] += .06

    monitor = monitor_factory("hardware", LaggingDriver)
    wait_for(monitor, lambda s: s["can_prepare"])
    preview = monitor.prepare(joint_target(15, duration=2), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(monitor, holding)["run"]

    rows = records(monitor, preview["request_id"])
    samples = [row["data"] for row in rows if row["event"] == "motion.sample"]
    assert any(abs(row["error"][0]) > .05 for row in samples if row["t"] < 2)
    assert samples[-1]["t"] >= 2
    assert result["state"] == "HOLDING_TARGET"
    assert not any(row["event"] == "motion.fault" for row in rows)
    if lag_at_goal:
        # Endpoint deviation is recorded for the operator but no longer aborts.
        assert any(row["event"] == "motion.advisory" and row["data"]["kind"] == "endpoint_gap"
                   for row in rows)
    monitor.stop_motion(preview["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_fast_target_records_required_limits(monitor_factory):
    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    preview = monitor.prepare(joint_target(45, .1, 100), OPERATOR)
    row = next(row for row in records(monitor, preview["request_id"])
               if row["event"] == "motion.preview")
    limits = row["data"]["trajectory"]["limits"]
    # The old 1 rad/s^2 and speed-percentage caps were replaced by the
    # trajectory's actual requirement.
    assert limits["velocity_rad_s"] > 1
    assert limits["acceleration_rad_s2"] > 1


def test_preview_accepts_speed_above_rated_cap(monitor_factory):
    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    preview = monitor.prepare(joint_target(45, .1, 100), OPERATOR)
    # 45 degrees in 0.1 s is far above the old 1 rad/s^2 planning cap.
    assert preview["peak_speed_deg_s"] > 400
    monitor.start_motion(preview["request_id"], OPERATOR)
    assert wait_for(monitor, holding)["run"]["state"] == "HOLDING_TARGET"
    monitor.stop_motion(preview["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_lost_operator_cancels_and_unconfirmed_disable_is_visible(monitor_factory, monkeypatch):
    monkeypatch.setattr("openarm_monitor.HEARTBEAT_TIMEOUT", .35)
    monkeypatch.setattr(MonitorDriverFixture, "fail_disable", True)
    monitor = monitor_factory("hardware")
    wait_for(monitor, lambda s: s["can_prepare"])
    preview = monitor.prepare(joint_target(duration=3), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    wait_for(monitor, lambda s: s["run"]["sample"] is not None)
    result = wait_for(monitor, finished, OBSERVER)
    assert result["run"]["stop_reason"] == "操控页面连接超时"
    assert result["run"]["state"] == "STOP_UNCONFIRMED"
    assert not result["run"]["stop_confirmed"] and not result["can_prepare"]
    rows = records(monitor, preview["request_id"])
    assert any(not record["confirmed"] for row in rows if row["event"] == "motion.disable_evidence"
               for record in row["data"])


def test_missing_device_clears_feedback_and_disables_previews(monitor_factory):
    def offline(*_):
        raise DriverError("fixture adapter disconnected")

    monitor = monitor_factory("hardware", offline)
    status = wait_for(monitor, lambda s: all("disconnected" in arm["reason"]
                                           for arm in s["arms"].values()))
    assert not status["can_prepare"]
    assert all(arm["joints"] == [None] * 7 and not arm["endpoints_mm"] for arm in status["arms"].values())
    assert all(arm["wrist_rpy_deg"] is None and not arm["piano_finger_keys"] for arm in status["arms"].values())
    with pytest.raises(Conflict):
        monitor.prepare(joint_target(), OPERATOR)


@pytest.mark.parametrize("mode", ["MIT", "POS_VEL", "VEL", "POS_FORCE", "hardware"])
def test_enable_all_and_retarget_without_disabling(monitor_factory, mode):
    monitor = monitor_factory("hardware" if mode == "hardware" else "virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    old = monitor.prepare(joint_target(), OPERATOR)
    if mode == "hardware":
        with pytest.raises(Conflict):
            monitor.switch_mode("POS_VEL")
    else:
        assert monitor.switch_mode(mode)["drive_mode"] == mode
        with pytest.raises(Conflict):
            monitor.start_motion(old["request_id"], OPERATOR)
    enabled = monitor.enable_all(OPERATOR)
    initial = wait_for(monitor, holding)
    assert all(row["state"] == 1 for arm in initial["arms"].values() for row in arm["joints"])
    with pytest.raises(Conflict):
        monitor.switch_mode("VEL")
    with pytest.raises(Conflict):
        monitor.prepare(joint_target(), OBSERVER)
    preview = monitor.prepare(joint_target(), OPERATOR)
    assert preview["command_joints"] == ["left J7"]
    monitor.start_motion(preview["request_id"], OPERATOR)
    after = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    for side, arm in after["arms"].items():
        for i, row in enumerate(arm["joints"]):
            expected = 2 if (side, i) == ("left", 6) else initial["arms"][side]["joints"][i]["position_deg"]
            assert row["state"] == 1 and row["position_deg"] == pytest.approx(expected, abs=.05)
    if mode == "hardware":
        assert all(len(arm.enabled_ids) == 7 for arm in MonitorDriverFixture.instances)
        assert not any(row["event"] == "motion.after" for row in records(monitor, enabled["request_id"]))
    elif mode != "MIT":
        sample = next(row["data"] for row in records(monitor, preview["request_id"])
                      if row["event"] == "motion.sample")
        can_id, payload = sample["dispatch"]["simulated_frames"]["left:7"]
        assert can_id == {"POS_VEL": 0x107, "VEL": 0x207, "POS_FORCE": 0x307}[mode]
        assert len(bytes.fromhex(payload)) == (4 if mode == "VEL" else 8)
    monitor.stop_motion(preview["request_id"])
    stopped = wait_for(monitor, finished)
    assert stopped["run"]["stop_confirmed"] and all(a["disabled"] for a in stopped["arms"].values())


def test_lost_operator_releases_all_held_axes(monitor_factory, monkeypatch):
    monkeypatch.setattr("openarm_monitor.HEARTBEAT_TIMEOUT", .4)
    monitor = monitor_factory("hardware")
    wait_for(monitor, lambda s: s["can_enable_all"])
    monitor.enable_all(OPERATOR)
    wait_for(monitor, holding)
    result = wait_for(monitor, finished, OBSERVER)
    assert result["run"]["stop_reason"] == "操控页面连接超时"
    assert result["run"]["stop_confirmed"] and all(a["disabled"] for a in result["arms"].values())


class ProfileDriverFixture(MonitorDriverFixture):
    fail_write = None
    mismatch = None

    def __init__(self, *args):
        super().__init__(*args)
        self.modes = dict.fromkeys(range(1, 8), 1)

    def read_parameter(self, j, reg):
        if reg in (13, 14, 36):
            return 98 if (self.side, j) == self.mismatch else 99
        if reg == 10:
            return self.modes[j]
        return super().read_parameter(j, reg)

    def write_disabled_mode(self, j, code):
        assert not self.enabled_ids
        self.modes[j] = code
        if (self.side, j, code) == self.fail_write:
            raise DriverError("fixture ACK timeout after write")

    def send(self, can_id, data):
        j = can_id & 255
        code = self.modes[j]
        if data in (b"\xff" * 7 + b"\xfc", b"\xff" * 7 + b"\xfd"):
            assert can_id == (j if self.side == "right" else j + (code - 1) * 256)
            if data[-1] == 0xfc:
                self.enabled_ids.add(j)
            else:
                self.enabled_ids.discard(j)
        else:
            assert can_id == j + (code - 1) * 256
            if code == 3:
                self.positions[j] += struct.unpack("<f", data)[0] * .04
            else:
                self.positions[j] = struct.unpack_from("<f", data)[0]

    def wait_motor_state(self, j, expected):
        state = self.query_motor(j)
        assert state.state == expected
        return state


class BaseSpecialDriverFixture(ProfileDriverFixture):
    """No commissioned profile: mode-offset targets, base-ID special frames."""

    def __init__(self, *args):
        super().__init__(*args)
        self.modes = dict.fromkeys(range(1, 8), 2)  # Motors already in POS_VEL.
        self.specials = []
        self.frames = []

    def send(self, can_id, data):
        j = can_id & 255
        if data in (b"\xff" * 7 + b"\xfc", b"\xff" * 7 + b"\xfd"):
            self.specials.append((j, can_id, data[-1]))
            if data[-1] == 0xfc:
                self.enabled_ids.add(j)
            else:
                self.enabled_ids.discard(j)
            return
        # The OpenArm CAN library addresses POS_VEL targets at base + 0x100.
        assert can_id == j + 0x100
        position, speed = struct.unpack("<ff", data)
        assert speed >= 0
        self.frames.append((j, position, speed))
        self.positions[j] = position

    def enable_motor(self, motor_id, **kwargs):
        raise AssertionError("POS_VEL enable must not use the MIT frame path")


def test_piano_exit_during_approach_holds_encoder_pose_with_pos_vel(monitor_factory):
    encoders = {}
    wrist = ("left", 7)

    class LoadedDriver(BaseSpecialDriverFixture):
        def query_motor(self, j):
            state = super().query_motor(j)
            return replace(state, position_rad=encoders.get((self.side, j), state.position_rad))

    monitor = monitor_factory("hardware", LoadedDriver)
    wait_for(monitor, lambda s: s["can_prepare"] and s["drive_mode"] == "POS_VEL")
    run = monitor.play_piano(dict(score_id="jin-chords"), OPERATOR)
    wait_for(monitor, lambda s: s["run"]["sample"] is not None)
    encoders[wrist] = monitor.mappings[wrist].to_motor(math.radians(23.))
    moving = wait_for(monitor, lambda s: s["arms"]["left"]["joints"][6]["position_deg"] == pytest.approx(23.))
    assert moving["run"]["sample"]["elapsed_s"] < run["piano_trial"]["prepare_s"]
    assert moving["run"]["sample"]["command_deg"]["left"][6] != pytest.approx(23.)

    monitor.exit_piano(run["request_id"])
    exited = wait_for(monitor, lambda s: s["run"]["piano_exited"])
    assert exited["all_enabled"] and exited["can_prepare"]
    assert exited["run"]["target_deg"]["left"][6] == pytest.approx(23.)
    started_holding = exited["run"]["sample"]["elapsed_s"]
    held = wait_for(monitor, lambda s: s["run"]["sample"]["elapsed_s"] >= started_holding + .15)
    assert holding(held) and held["all_enabled"]
    assert held["run"]["sample"]["command_deg"] == exited["run"]["target_deg"]
    for driver in LoadedDriver.instances:
        assert driver.enabled_ids == set(range(1, 8))
        assert all(code == 0xfc for _, _, code in driver.specials)
        assert len(driver.specials) == 7
    monitor.stop_motion(run["request_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


@pytest.fixture
def profile_path(tmp_path):
    sides = {s: dict(bus_status="0x0", motors=[dict(motor_id=j, online=True, state=0)
                                               for j in range(1, 8)]) for s in CHANNELS}
    axes = [dict(side=s, motor_id=j, channel=ch, read_errors={},
                 parameters={r: dict(value=99) for r in REGISTERS})
            for s, ch in CHANNELS.items() for j in range(1, 8)]
    inventory = dict(kind="position_mode_read_only_inventory", complete=True,
                     mapping_version="hardware-unified-forward-inward-v2-20261116",
                     before=sides, after=sides, axes=axes)
    (tmp_path / "inventory.json").write_text(json.dumps(inventory))
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(dict(inventory="inventory.json", disabled_preload_verified=True,
                                    volatile_mode_write_verified=True, pos_force_current_fraction=.1,
                                    special_frame_address=dict(left=["mode"]*7, right=["base"]*7))))
    return path


@pytest.mark.parametrize("mode", ["VEL", "POS_FORCE", "POS_VEL"])
def test_commissioned_modes_hold_and_stop(monitor_factory, profile_path, mode):
    m = monitor_factory("hardware", ProfileDriverFixture, profile_path=profile_path)
    assert wait_for(m, lambda s: s["can_switch_mode"])["drive_mode"] == "MIT"
    m.switch_mode(mode)
    wait_for(m, lambda s: s["can_enable_all"] and s["drive_mode"] == mode)
    assert all(set(d.modes.values()) == {MODE_CODES[mode]} for d in m.drivers.values())
    m.enable_all(OPERATOR)
    wait_for(m, holding)
    preview = m.prepare(joint_target(), OPERATOR)
    m.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    assert result["arms"]["left"]["joints"][-1]["position_deg"] == pytest.approx(2, abs=.15)
    m.stop_motion(preview["request_id"])
    stopped = wait_for(m, finished)
    assert stopped["run"]["stop_confirmed"] and all(a["disabled"] for a in stopped["arms"].values())


def test_pos_vel_enable_and_hold_without_drive_profile(monitor_factory, profile_path):
    """POS_VEL enables and moves from the 14-axis readback alone."""
    inventory = profile_path.parent / "inventory.json"
    m = monitor_factory("hardware", BaseSpecialDriverFixture, inventory_path=inventory)
    assert m.drive_profile is None
    status = wait_for(m, lambda s: s["motion_supported"] and s["drive_mode"] == "POS_VEL")
    assert status["can_enable_all"] and status["can_prepare"] and status["mode_error"] is None
    m.enable_all(OPERATOR)
    enabled = wait_for(m, holding)
    assert all(row["state"] == 1 for arm in enabled["arms"].values() for row in arm["joints"])
    left = m.drivers["left"]
    # Enable/disable specials stay on the base motor ID; targets use base + 0x100.
    assert [can_id for _, can_id, _ in left.specials] == [j for j in range(1, 8)]
    assert all(control == 0xfc for _, _, control in left.specials)
    preview = m.prepare(joint_target(duration=1.5), OPERATOR)
    before_motion = len(left.frames)
    m.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    assert result["arms"]["left"]["joints"][-1]["position_deg"] == pytest.approx(2, abs=.15)
    rows = records(m, preview["request_id"])
    assert not any(row["event"] == "motion.fault" for row in rows)
    # The continuation swaps the enable hold speed for the planned POS_VEL limit.
    speeds = [speed for _, _, speed in left.frames[before_motion:]]
    assert speeds[-1] > .1 and len({round(speed, 6) for speed in speeds[-3:]}) == 1
    m.stop_motion(preview["request_id"])
    stopped = wait_for(m, finished)
    assert stopped["run"]["stop_confirmed"]
    assert len([control for _, _, control in left.specials if control == 0xfd]) == 7
    assert all(not d.enabled_ids for d in m.drivers.values())


@pytest.mark.parametrize("mode", ["MIT", "POS_VEL"])
def test_loaded_joint_keeps_its_setpoint_across_commands(monitor_factory, profile_path, mode):
    # Each fixture decodes actual outbound frames into motor-space positions.
    # Override selected encoder readings independently of the commanded position.
    driver = MonitorDriverFixture if mode == "MIT" else ProfileDriverFixture
    encoders, sent, power_events = {}, [], []

    class LoadedDriver(driver):
        def query_motor(self, j):
            state = super().query_motor(j)
            return replace(state, position_rad=encoders.get((self.side, j), state.position_rad))

        def send(self, can_id, data):
            super().send(can_id, data)
            j = can_id & 255
            if data in (b"\xff" * 7 + b"\xfc", b"\xff" * 7 + b"\xfd"):
                power_events.append((self.side, j, data[-1]))
            else:
                sent.append(((self.side, j), self.positions[j]))

        def enable_motor(self, j, **kwargs):
            power_events.append((self.side, j, 0xfc))
            return super().enable_motor(j, **kwargs)

        def disable_motor(self, j):
            power_events.append((self.side, j, 0xfd))
            return super().disable_motor(j)

    options = dict(profile_path=profile_path) if mode == "POS_VEL" else {}
    m = monitor_factory("hardware", LoadedDriver, **options)
    wait_for(m, lambda s: s["can_enable_all"])
    if mode == "POS_VEL":
        m.switch_mode(mode)
        wait_for(m, lambda s: s["can_enable_all"] and s["drive_mode"] == mode)
    m.enable_all(OPERATOR)
    wait_for(m, holding)
    assert len(power_events) == 14
    wrist, opposite, moving = ("right", 6), ("left", 4), ("right", 3)
    opposite_setpoint = m.mappings[opposite].to_motor(m.job.plan["goal"][opposite])
    encoders[wrist] = math.radians(-6.35)
    encoders[opposite] = opposite_setpoint + .04
    wait_for(m, lambda s: s["arms"]["right"]["joints"][5]["position_deg"] == pytest.approx(-6.35))
    first_frame = len(sent)
    for angle, wrist_input in [(2, None), (4, 0), (4.00001, 0)]:
        payload = dict(arm="right", mode="joint", duration_s=.2, speed_percent=20,
                       targets={"right": [None, None, angle, None, None, wrist_input, None]})
        preview = m.prepare(payload, OPERATOR)
        request_id = preview["request_id"]
        plan = m.preview[0]
        wrist_index, moving_index = plan["keys"].index(wrist), plan["keys"].index(moving)
        assert plan["trajectory"].points[0][1][wrist_index] == 0
        # Change the moving axis reading after preview so revalidation is observable.
        start = math.radians(angle - 1)
        encoders[moving] = m.mappings[moving].to_motor(start)
        wait_for(m, lambda s: s["arms"]["right"]["joints"][2]["position_deg"] == pytest.approx(angle - 1))
        m.start_motion(request_id, OPERATOR)
        wait_for(m, lambda s: holding(s) and s["run"]["request_id"] == request_id)
        assert plan["trajectory"].points[0][1][moving_index] == pytest.approx(start)
        with m.journal.lock:
            samples = [r["data"] for r in records(m, request_id) if r["event"] == "motion.sample"]
        assert samples and all(s["target"][wrist_index] == s["target_velocity"][wrist_index] == 0
                               for s in samples)
        assert samples[-1]["measured"][wrist_index] == pytest.approx(encoders[wrist])
        last_move = next(q for key, q in reversed(sent) if key == moving)
        assert last_move == pytest.approx(m.mappings[moving].to_motor(math.radians(angle)), abs=.0004)
    for key, expected in [(wrist, 0.), (opposite, opposite_setpoint)]:
        frames = [q for axis, q in sent[first_frame:] if axis == key]
        assert frames and all(abs(q - expected) < .0004 for q in frames)
    assert len(power_events) == 14
    m.stop_motion(request_id)
    assert wait_for(m, finished)["run"]["stop_confirmed"]
    m.enable_all(OPERATOR)
    wait_for(m, holding)
    assert m.job.plan["goal"][wrist] == pytest.approx(encoders[wrist])


@pytest.mark.parametrize("failure", ["fail_write", "mismatch"])
def test_mode_failure_restores_writes_and_recovers_uniform_readback(monitor_factory, profile_path, monkeypatch, failure):
    m = monitor_factory("hardware", ProfileDriverFixture, profile_path=profile_path)
    wait_for(m, lambda s: s["can_switch_mode"])
    monkeypatch.setattr(ProfileDriverFixture, failure, ("right", 4, 2) if failure == "fail_write" else ("right", 4))
    m.switch_mode("POS_VEL")
    status = wait_for(m, lambda s: any(r["event"] == "drive.mode_failed" for r in s["events"]))
    assert all(set(d.modes.values()) == {1} and not d.enabled_ids for d in m.drivers.values())
    if failure == "fail_write":
        # A uniform 14-axis readback is authoritative: it clears the stale mode
        # failure so enable/preview recover without another mode re-selection.
        status = wait_for(m, lambda s: s["can_prepare"] and s["can_enable_all"])
        assert status["drive_mode"] == "MIT" and status["mode_error"] is None
    else:
        # The firmware readback itself remains inconsistent, so motion stays off.
        assert not status["can_prepare"] and not status["can_enable_all"] and status["drive_mode"] is None


def test_start_pose_enables_axes_and_quick_returns(monitor_factory, monkeypatch):
    for name in ("ZERO_SECONDS", "J1_J2_SECONDS", "J1_J4_SECONDS", "START_SECONDS"):
        monkeypatch.setattr(f"openarm_start_pose.{name}", .2)
    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    status = monitor.status(0, OPERATOR)
    # A prior “全部使能” is not required: the recipe enables all 14 axes itself.
    assert not status["all_enabled"] and status["can_start_pose"]

    result = monitor.start_pose_motion("start-return", OPERATOR)
    done = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == result["request_id"])
    for side in ("left", "right"):
        assert [row["position_deg"] for row in done["arms"][side]["joints"]] == pytest.approx(
            [-30, 0, 0, 120, 0, 0, 15], abs=.03)
    assert all(row["state"] == 1 for arm in done["arms"].values() for row in arm["joints"])
    plan = monitor.job.plan
    assert plan["source"] == "start-return"
    assert [name for name, _, _ in plan["steps"]] == [
        "0位（J1～J6 为 0°，J7 +15°）", "J1 +15° / J2 −45°",
        "J1 −30° / J4 +90°", "起始位（J1 −30° / J2 0° / J4 +120° / J7 +15°）"]
    keys = plan["keys"]
    right_index = [keys.index(("right", j)) for j in range(1, 8)]
    left_index = [keys.index(("left", j)) for j in range(1, 8)]
    points = plan["trajectory"].points
    assert [t for t, _ in points] == pytest.approx([0, .2, .4, .6, .8], abs=1e-6)
    expected = [
        ([0, 0, 0, 0, 0, 0, 15], [0, 0, 0, 0, 0, 0, 15]),
        ([15, -45, 0, 0, 0, 0, 15], [15, -45, 0, 0, 0, 0, 15]),
        ([-30, -45, 0, 90, 0, 0, 15], [-30, -45, 0, 90, 0, 0, 15]),
        ([-30, 0, 0, 120, 0, 0, 15], [-30, 0, 0, 120, 0, 0, 15]),
    ]
    for point, (right, left) in zip(points[1:], expected):
        assert [math.degrees(point[1][i]) for i in right_index] == pytest.approx(right, abs=.01)
        assert [math.degrees(point[1][i]) for i in left_index] == pytest.approx(left, abs=.01)

    preview = monitor.prepare(dict(arm="both", mode="joint",
                                   targets={"left": [5.]*7, "right": [5.]*7},
                                   duration_s=.2, speed_percent=20), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    shortcut = monitor.start_pose_motion("start-pose", OPERATOR)
    done = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == shortcut["request_id"])
    for side in ("left", "right"):
        assert [row["position_deg"] for row in done["arms"][side]["joints"]] == pytest.approx(
            [-30, 0, 0, 120, 0, 0, 15], abs=.03)
    assert monitor.job.plan["duration"] == pytest.approx(.2)
    monitor.stop_motion(done["run"]["session_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_zero_pose_quick_returns_both_arms(monitor_factory, monkeypatch):
    monkeypatch.setattr("openarm_start_pose.ZERO_SECONDS", .2)
    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    monitor.enable_all(OPERATOR)
    wait_for(monitor, holding)
    preview = monitor.prepare(dict(arm="both", mode="joint",
                                   targets={"left": [5.]*7, "right": [5.]*7},
                                   duration_s=.2, speed_percent=20), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])
    result = monitor.start_pose_motion("zero-pose", OPERATOR)
    done = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == result["request_id"])
    for side in ("left", "right"):
        assert [row["position_deg"] for row in done["arms"][side]["joints"]] == pytest.approx(
            [0, 0, 0, 0, 0, 0, 15], abs=.03)
    assert monitor.job.plan["source"] == "zero-pose"
    assert monitor.job.plan["steps"][0][0] == "0位（J1～J6 为 0°，J7 +15°）"
    assert monitor.job.plan["duration"] == pytest.approx(.2)
    monitor.stop_motion(done["run"]["session_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_zero_return_mirrors_the_start_recipe():
    """Each reversed segment withdraws exactly the travel of its forward counterpart."""
    from openarm_start_pose import return_start_steps, return_zero_steps
    forward, backward = return_start_steps(), return_zero_steps()
    # The first reversed step starts at the measured pose; the waypoints after it are fixed.
    assert backward[0].right_deg == forward[-1].right_deg == backward[0].left_deg
    assert backward[-1].right_deg == forward[0].right_deg == backward[-1].left_deg
    assert [step.right_deg for step in backward[1:]] == [step.right_deg for step in reversed(forward[:-1])]
    assert [step.right_deg for step in backward] == [step.left_deg for step in backward]
    assert [step.duration_s for step in backward] == [step.duration_s for step in reversed(forward)]

    def moves(steps):
        return [[round(b-a, 6) for a, b in zip(before.right_deg, after.right_deg)]
                for before, after in zip(steps, steps[1:])]
    assert moves(backward) == [[-delta for delta in row] for row in reversed(moves(forward))]


def test_zero_return_quick_returns_to_zero_enabled(monitor_factory, monkeypatch):
    for name in ("ZERO_SECONDS", "J1_J2_SECONDS", "J1_J4_SECONDS", "START_SECONDS"):
        monkeypatch.setattr(f"openarm_start_pose.{name}", .2)
    monitor = monitor_factory("virtual")
    wait_for(monitor, lambda s: s["can_prepare"])
    monitor.enable_all(OPERATOR)
    wait_for(monitor, holding)
    preview = monitor.prepare(dict(arm="both", mode="joint",
                                   targets={"left": [5.]*7, "right": [5.]*7},
                                   duration_s=.2, speed_percent=20), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == preview["request_id"])

    result = monitor.start_pose_motion("zero-return", OPERATOR)
    done = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == result["request_id"])
    for side in ("left", "right"):
        assert [row["position_deg"] for row in done["arms"][side]["joints"]] == pytest.approx(
            [0, 0, 0, 0, 0, 0, 15], abs=.03)
        assert all(row["state"] == 1 for row in done["arms"][side]["joints"])

    plan = monitor.job.plan
    assert plan["source"] == "zero-return"
    assert [name for name, _, _ in plan["steps"]] == [
        "当前位置 → 起始位（J1 −30° / J2 0° / J4 +120° / J7 +15°）",
        "J1 −30° / J2 −45° / J4 +90°",
        "J1 +15° / J2 −45°",
        "0位（J1～J6 为 0°，J7 +15°）"]
    assert [t for t, _ in plan["trajectory"].points] == pytest.approx([0, .2, .4, .6, .8], abs=1e-6)
    keys = plan["keys"]
    right_index = [keys.index(("right", j)) for j in range(1, 8)]
    expected = [[-30, 0, 0, 120, 0, 0, 15],
                [-30, -45, 0, 90, 0, 0, 15],
                [15, -45, 0, 0, 0, 0, 15],
                [0, 0, 0, 0, 0, 0, 15]]
    for point, right in zip(plan["trajectory"].points[1:], expected):
        assert [math.degrees(point[1][i]) for i in right_index] == pytest.approx(right, abs=.01)
    monitor.stop_motion(done["run"]["session_id"])
    assert wait_for(monitor, finished)["run"]["stop_confirmed"]


def test_start_pose_sequence_on_hardware_fixture(monitor_factory, monkeypatch):
    for name in ("ZERO_SECONDS", "J1_J2_SECONDS", "J1_J4_SECONDS", "START_SECONDS"):
        monkeypatch.setattr(f"openarm_start_pose.{name}", .15)
    monitor = monitor_factory("hardware")
    wait_for(monitor, lambda s: s["can_enable_all"])
    monitor.enable_all(OPERATOR)
    wait_for(monitor, holding)
    result = monitor.start_pose_motion("start-return", OPERATOR)
    done = wait_for(monitor, lambda s: holding(s) and s["run"]["request_id"] == result["request_id"])
    for side in ("left", "right"):
        assert [row["position_deg"] for row in done["arms"][side]["joints"]] == pytest.approx(
            [-30, 0, 0, 120, 0, 0, 15], abs=.05)
    assert set(monitor.job.plan["keys"]) == {("left", j) for j in range(1, 8)} | {
        ("right", j) for j in range(1, 8)}
    assert not any(row["event"] == "motion.fault" for row in records(monitor, result["request_id"]))
    monitor.stop_motion(done["run"]["session_id"])
    stopped = wait_for(monitor, finished)
    assert stopped["run"]["stop_confirmed"]
    assert all(arm["disabled"] for arm in stopped["arms"].values())



def test_register_console_reads_flash_mode_and_handles_mixed_axes(monitor_factory, profile_path):
    class SavedModes(ProfileDriverFixture):
        def __init__(self, *args):
            super().__init__(*args)
            self.modes = dict.fromkeys(range(1, 8), 2)
            if self.side == "left":
                self.modes[1] = 1

    m = monitor_factory("hardware", SavedModes, inventory_path=profile_path.parent / "inventory.json")
    status = wait_for(m, lambda s: s["can_switch_mode"])
    assert status["drive_mode"] is None and status["axis_modes"]["left"] == [1] + [2]*6
    assert not status["can_enable_all"]
    with pytest.raises(Conflict):
        m.switch_mode("VEL", {"left": [1]*7, "right": [1]*7})
    m.switch_mode("POS_VEL", status["axis_modes"])
    status = wait_for(m, lambda s: s["drive_mode"] == "POS_VEL" and s["can_switch_mode"])
    assert status["axis_modes"] == {"left": [2]*7, "right": [2]*7}
    # Mode readback alone opens enable and preview; no drive profile is required.
    assert status["can_enable_all"] and status["can_prepare"] and status["motion_supported"]
    m.switch_mode("MIT", status["axis_modes"])
    assert wait_for(m, lambda s: s["can_prepare"])["drive_mode"] == "MIT"
    assert all(not d.enabled_ids for d in m.drivers.values())
