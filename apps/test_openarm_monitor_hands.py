from unittest.mock import Mock
import time

import pytest

from openarm_monitor_hands import AeroTransport, ConsoleHands
from openarm_monitor import OperatorWatch


class FixtureHand(AeroTransport):
    """Exercise real-mode routing with the SDK simulator as the device."""
    def __init__(self, demo):
        super().__init__(demo=True)
        self.packets = []

    def write_position_frame(self, raw):
        self.packets.append(tuple(raw))
        super().write_position_frame(raw)


def test_manual_control_port_ownership_and_reconnect(tmp_path):
    hands = ConsoleHands(True, Mock(), tmp_path, FixtureHand)
    hands.defaults["left"]["real"]["3"]["press"] = ""
    right_press = hands.defaults["right"]["real"]["3"]["press"]
    connect = dict(side="left", action="connect", port="COM9", demo=False)
    try:
        first = hands.action(connect)
        link = hands.links["left"]
        baseline = tuple(link.bus.raw)
        assert link.bus.packets == []
        with pytest.raises(ValueError, match="同一个"):
            hands.action(dict(connect, side="right", port="com9"))
        cmd = dict(side="left", channel=3, session=first["session"])
        with pytest.raises(ValueError):
            hands.action(dict(cmd, action="press"))
        hands.action(dict(cmd, action="move", degrees=20))
        hands.action(dict(cmd, action="save_press"))
        hands.action(dict(cmd, action="move", channel=4, degrees=28))
        assert len(link.bus.packets) == 2
        a, b = link.bus.packets
        assert a[:3] == baseline[:3] and a[4:] == baseline[4:]
        assert a[:4] == b[:4] and a[5:] == b[5:]
        assert first["calibration"]["3"]["press"] == ""
        hands.action(dict(cmd, action="disconnect"))
        second = hands.action(connect)
        assert second["calibration"]["3"]["press"] == pytest.approx(20, abs=.1)
        with pytest.raises(ValueError, match="已变更"):
            hands.action(dict(cmd, action="press"))
        hands.action(dict(cmd, session=second["session"], action="press"))
        assert len(hands.links["left"].bus.packets) == 1
        right = hands.action(dict(connect, side="right", port="COM8"))
        assert right["calibration"]["3"]["press"] == right_press
    finally:
        hands.close()
    assert not any(row["connected"] for row in hands.snapshot().values())


def test_piano_pose_sends_one_frame_to_the_selected_hand(tmp_path):
    hands = ConsoleHands(True, Mock(), tmp_path, FixtureHand)
    try:
        connect = dict(action="connect", demo=False)
        left = hands.action(dict(connect, side="left", port="COM9"))
        right = hands.action(dict(connect, side="right", port="COM8"))
        left_bus = hands.links["left"].bus
        right_bus = hands.links["right"].bus
        assert left["piano_pose_available"] and right["piano_pose_available"]
        assert left_bus.packets == right_bus.packets == []

        row = hands.action(dict(side="left", session=left["session"], action="piano_pose"))
        assert left_bus.packets == [(65535, 63805, 28823, 11649, 10193, 12741, 7281)]
        assert row["feedback"]["q"] == pytest.approx([100, 53.02, 35.73, 16, 14, 17.5, 10], abs=.01)
        assert row["calibration"] == left["calibration"]
        assert right_bus.packets == []

        with pytest.raises(ValueError, match="已变更"):
            hands.action(dict(side="right", session=left["session"], action="piano_pose"))
        row = hands.action(dict(side="right", session=right["session"], action="piano_pose"))
        assert right_bus.packets == [(65535, 63805, 28823, 7896, 3718, 5058, 7281)]
        assert len(left_bus.packets) == 1
        assert row["feedback"]["q"] == pytest.approx([100, 53.02, 35.73, 10.845, 5.107, 6.947, 10], abs=.01)
        assert row["calibration"] == right["calibration"]
        assert list(tmp_path.iterdir()) == []

        hands.action(dict(side="left", session=left["session"], action="move", channel=3, degrees=20))
        preset, index_move = left_bus.packets
        assert index_move[:3] == preset[:3] and index_move[4:] == preset[4:]
    finally:
        hands.close()


@pytest.mark.parametrize("finish", ["complete", "stop", "disconnect", "heartbeat"])
def test_console_score_owns_existing_sessions_and_releases_chord(tmp_path, monkeypatch, finish):
    from openarm_monitor_hands import JIN_SCORE, compile_score

    original = compile_score(JIN_SCORE.read_text(encoding="utf-8-sig"))
    assert original.note_count == 1300
    for side in original.sides:
        first = original.events[side][0]
        assert (first.time_us, first.changes) == (4_000_000, ((3, True), (5, True)))

    score_file = tmp_path / "short.txt"
    hold = ".15" if finish == "complete" else "8"
    score_file.write_text(f"left_arm_move(19, 1)\nright_arm_move(24, 1)\nsleep(1)\n"
                          f"hand(D,2479)\nsleep({hold})\nhand(U,2479)\n", encoding="utf-8")
    monkeypatch.setattr("openarm_monitor_hands.JIN_SCORE", score_file)
    hands = ConsoleHands(True, Mock(), tmp_path, FixtureHand)
    watch = OperatorWatch()
    try:
        sessions = {s: hands.action(dict(side=s, action="connect", demo=True))["session"]
                    for s in original.sides}
        buses = {s: hands.links[s].bus for s in sessions}
        with pytest.raises(ValueError, match="连接"):
            hands.start_score({"sessions": {}}, "owner", watch)
        run = hands.start_score({"sessions": sessions}, "owner", watch)
        with pytest.raises(ValueError, match="排练"):
            hands.start_score({"sessions": sessions}, "owner", watch)
        def wait_until(predicate):
            deadline = time.monotonic() + 4
            while not predicate() and time.monotonic() < deadline:
                time.sleep(.005)
            assert predicate()
        wait_until(lambda: all(b.packets for b in buses.values()))
        if finish == "stop":
            hands.stop_score(run["id"])
        elif finish == "disconnect":
            hands.action(dict(side="right", action="disconnect", session=sessions["right"]))
        elif finish == "heartbeat":
            watch.last_seen -= 10
            hands.score_status("observer")
        wait_until(lambda: not hands.score_status("observer")["active"])
        for bus in buses.values():
            down, up = bus.packets
            assert [i for i in range(7) if down[i] != up[i]] == [3, 5]
        assert hands.snapshot()["left"]["connected"]
        assert hands.snapshot()["right"]["connected"] == (finish != "disconnect")
        if finish == "heartbeat":
            assert "超时" in hands.score_status("observer")["message"]
    finally:
        hands.close()
    assert not any(row["connected"] for row in hands.snapshot().values())


def test_offline_console_cannot_open_serial_and_saves_separately(tmp_path):
    hands = ConsoleHands(False, Mock(), tmp_path)
    try:
        with pytest.raises(ValueError, match="离线"):
            hands.action(dict(side="left", action="connect", demo=False, port="COM8"))
        row = hands.action(dict(side="left", action="connect", demo=True))
        cmd = dict(side="left", channel=4, session=row["session"])
        hands.action(dict(cmd, action="move", degrees=12))
        saved = hands.action(dict(cmd, action="save_home"))
        assert saved["calibration"]["4"]["home"] == pytest.approx(12, abs=.1)
        assert [path.name for path in tmp_path.iterdir()] == ["aero_monitor_left_simulation.json"]
        with pytest.raises(ValueError):
            hands.action(dict(cmd, channel=0, action="move", degrees=5))
    finally:
        hands.close()
