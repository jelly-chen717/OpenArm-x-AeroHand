"""Monitor integration tests. The PCAN-mode cases inject a driver fixture."""
from dataclasses import asdict
import json
import struct
import time

import pytest

from openarm_driver import DriverError
from openarm_monitor import CHANNELS, Conflict, Monitor
from openarm_monitor_modes import MODE_CODES
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
            self.positions[6] = .03
        return dict(side=self.side, online=7, bus_status="0x0",
                    motors=[asdict(self.query_motor(j)) for j in range(1, 8)])

    def disable_motor(self, motor_id):
        if self.fail_disable:
            raise DriverError("fixture disable timeout")
        return super().disable_motor(motor_id)


def wait_for(monitor, predicate, client=OPERATOR):
    deadline = time.monotonic() + 6
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
    with pytest.raises(ValueError, match="超限"):
        monitor.prepare(joint_target(6, .5), OPERATOR)
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


def test_start_pose_rechecked_before_enable(monitor_factory, monkeypatch):
    monkeypatch.setattr(MonitorDriverFixture, "drift_before_enable", True)
    monitor = monitor_factory("hardware")
    wait_for(monitor, lambda s: s["can_prepare"])
    preview = monitor.prepare(joint_target(), OPERATOR)
    monitor.start_motion(preview["request_id"], OPERATOR)
    result = wait_for(monitor, finished)
    assert result["run"]["state"] == "FAULT"
    assert "起始姿态已变化" in result["run"]["message"]
    assert all(not arm.gains for arm in MonitorDriverFixture.instances)
    assert not any(event[0] == "send" for event in MonitorDriverFixture.events)


@pytest.mark.parametrize("lag_at_goal", [False, True], ids=["catches_up", "misses_endpoint"])
def test_hardware_tracking_lag_is_logged_until_endpoint(monitor_factory, lag_at_goal):
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
    result = wait_for(monitor, finished if lag_at_goal else holding)["run"]

    rows = records(monitor, preview["request_id"])
    samples = [row["data"] for row in rows if row["event"] == "motion.sample"]
    assert any(abs(row["error"][0]) > .05 for row in samples if row["t"] < 2)
    assert samples[-1]["t"] >= 2
    if lag_at_goal:
        assert result["stop_confirmed"]
        assert result["state"] == "FAULT"
        assert "Waypoint tolerance exceeded" in result["message"]
    else:
        assert result["state"] == "HOLDING_TARGET"
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
    assert not wait_for(m, lambda s: s["can_switch_mode"])["can_enable_all"]
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


@pytest.mark.parametrize("failure", ["fail_write", "mismatch"])
def test_mode_failure_blocks_enable_and_restores_partial_writes(monitor_factory, profile_path, monkeypatch, failure):
    monkeypatch.setattr(ProfileDriverFixture, failure, ("right", 4, 2) if failure == "fail_write" else ("right", 4))
    m = monitor_factory("hardware", ProfileDriverFixture, profile_path=profile_path)
    wait_for(m, lambda s: s["can_switch_mode"])
    m.switch_mode("POS_VEL")
    status = wait_for(m, lambda s: any(r["event"] == "drive.mode_failed" for r in s["events"]))
    assert not status["can_prepare"] and not status["can_enable_all"] and status["drive_mode"] is None
    assert all(set(d.modes.values()) == {1} and not d.enabled_ids for d in m.drivers.values())
