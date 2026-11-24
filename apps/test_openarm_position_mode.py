import json
import math
import struct

import pytest

from openarm_driver import DriverError, LIMITS, OpenArmDriver, motor_type
from openarm_monitor_modes import position_frame
from openarm_position_mode import DEFAULT_CONFIG, check_config, encode_target, load_config, read_inventory
from test_openarm_driver import FakeBus


def test_config_routes_all_position_mode_axes():
    config, mappings = load_config()
    result = check_config(config, mappings)
    assert result["ctrl_mode"] == 2
    for side, channel in (("left", "0x51"), ("right", "0x52")):
        assert result["arms"][side] == {
            "channel": channel, "target_can_ids": [hex(0x100 + j) for j in range(1, 8)]
        }
    mapping = mappings["left", 1]
    target = math.radians(-2)
    can_id, payload = encode_target(mapping, target, .15)
    assert can_id == 0x101
    assert struct.unpack("<ff", payload) == pytest.approx((math.radians(2), .15))
    for speed in (-.15, 0, float("nan"), float("inf"), 46):
        with pytest.raises(ValueError):
            encode_target(mapping, target, speed)
    with pytest.raises(ValueError, match="Position"):
        encode_target(mapping, mapping.limits[1] + .01, .15)


def test_position_frames_saturate_speed_and_current_at_protocol_limits():
    _, mappings = load_config()
    mapping = mappings["left", 1]
    target = mapping.to_motor(math.radians(-2))
    _, speed_range, _ = LIMITS[motor_type(mapping.motor_id)]

    can_id, payload = position_frame("POS_VEL", mapping, target, 100., .1)
    encoded_target, encoded_speed = struct.unpack("<ff", payload)
    assert can_id == 0x101
    assert encoded_target == pytest.approx(target)
    assert encoded_speed == pytest.approx(speed_range)

    can_id, payload = position_frame("POS_FORCE", mapping, target, 100., 1.5)
    _, speed_code, current_code = struct.unpack("<fHH", payload)
    assert can_id == 0x301
    assert speed_code == 10000
    assert current_code == 10000


def test_config_rejects_missing_axes(tmp_path):
    config = json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    config["arms"]["right"]["motor_ids"][-1] = 6
    path = tmp_path / "mode.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    with pytest.raises(ValueError, match="expected J1"):
        load_config(path)


@pytest.mark.parametrize("right_enabled", [False, True])
def test_inventory_only_reads_and_reports_actual_modes(right_enabled):
    config, _ = load_config()
    buses = {side: FakeBus() for side in config["arms"]}
    buses["right"].mode = 2
    buses["right"].enabled = right_enabled

    def factory(side, channel, timeout):
        buses[side].channel = channel
        return OpenArmDriver(side, channel, timeout, bus=buses[side])

    report = read_inventory(config, driver_factory=factory)
    assert report["complete"] is (not right_enabled)
    assert report["axes_matching_target"] == (0 if right_enabled else 7)
    assert len(report["axes"]) == (0 if right_enabled else 14)
    if not right_enabled:
        ctrl_mode = report["axes"][0]["parameters"]["CTRL_MODE"]
        assert ctrl_mode == {"register": 10, "value": 1, "raw_reply": "0100330a01000000"}
    assert all(not bus.opened for bus in buses.values())
    assert all(can_id == 0x7ff and payload[2] in (0x33, 0xcc)
               for bus in buses.values() for can_id, payload in bus.sent)


def test_inventory_releases_first_bus_if_second_open_fails():
    config, _ = load_config()
    left_bus = FakeBus()

    def factory(side, channel, timeout):
        if side == "right":
            raise DriverError("CAN channel already in use")
        return OpenArmDriver(side, channel, timeout, bus=left_bus)

    with pytest.raises(DriverError, match="already in use"):
        read_inventory(config, driver_factory=factory)
    assert not left_bus.opened
    assert not left_bus.sent
