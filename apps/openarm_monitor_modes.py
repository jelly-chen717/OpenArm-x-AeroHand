"""Commissioned monitor drive modes. The default monitor never writes a mode register."""
import json
import struct
import time

from openarm_driver import DriverError
from openarm_position_mode import REGISTERS, _fully_disabled, encode_target

DRIVE_MODES = ("MIT", "POS_VEL", "VEL", "POS_FORCE")
MODE_CODES = {mode: index for index, mode in enumerate(DRIVE_MODES, 1)}
KEYS = tuple((side, j) for side in ("left", "right") for j in range(1, 8))
VERSION_REGISTERS = ("hw_ver", "sw_ver", "sub_ver")


def position_frame(mode, mapping, p, speed, current_fraction):
    if mode == "POS_VEL":
        return encode_target(mapping, mapping.to_model(p), speed)
    if mode != "POS_FORCE":
        raise ValueError("Expected a position mode")
    limit = int(abs(mapping.velocity_to_motor(speed)) * 100)
    if not 1 <= limit <= 10000 or not 0 < current_fraction <= 1:
        raise ValueError("Invalid POS_FORCE speed or current limit")
    current_code = int(current_fraction * 10000)
    if current_code == 0:
        raise ValueError("POS_FORCE current limit rounds to zero")
    payload = struct.pack("<fHH", p, limit, current_code)
    q = mapping.to_model(struct.unpack_from("<f", payload)[0])
    if not mapping.limits[0] <= q <= mapping.limits[1]:
        raise ValueError("Encoded position outside mapped joint range")
    return mapping.motor_id + 0x300, payload


def velocity_frame(joint, position_error, feedforward, limit):
    # An outer position loop converts the console's position reference to motor rad/s.
    speed = max(-limit, min(limit, feedforward + 5.0 * position_error))
    return joint + 0x200, struct.pack("<f", speed)


class DriveProfile:
    def __init__(self, path, version, channels):
        profile = json.loads(path.read_text(encoding="utf-8-sig"))
        inventory = json.loads((path.parent / profile["inventory"]).read_text(encoding="utf-8-sig"))
        if (inventory["kind"] != "position_mode_read_only_inventory"
                or inventory["complete"] is not True or inventory["mapping_version"] != version
                or not all(_fully_disabled(inventory[phase][s]) for phase in ("before", "after")
                           for s in channels)
                or profile["disabled_preload_verified"] is not True
                or profile["volatile_mode_write_verified"] is not True):
            raise ValueError("需要完整的 C02 盘点及已核实的首目标预装载和易失模式写入语义。")
        self.versions, self.addressing = {}, {}
        if any(len(profile["special_frame_address"][s]) != 7 for s in channels):
            raise ValueError("每侧需要七个已核实的特殊帧地址。")
        self.current = float(profile["pos_force_current_fraction"])
        if not .0001 <= self.current <= 1:
            raise ValueError("POS_FORCE 电流标幺上限须为 0.0001–1。")
        for axis in inventory["axes"]:
            side, j = key = axis["side"], axis["motor_id"]
            if (key in self.versions or key not in KEYS or axis["channel"] != channels[side]
                    or axis["read_errors"] or not REGISTERS.keys() <= axis["parameters"].keys()):
                raise ValueError("盘点中的电机或通道不匹配。")
            self.versions[key] = tuple(axis["parameters"][r]["value"] for r in VERSION_REGISTERS)
            address = profile["special_frame_address"][side][j - 1]
            if address not in ("base", "mode"):
                raise ValueError("特殊帧地址须按该轴固件核实为 base 或 mode。")
            self.addressing[key] = address
        if self.versions.keys() != set(KEYS):
            raise ValueError("需要左右臂全部 14 轴的盘点记录。")

    def read_modes(self, arms):
        modes = {}
        for key in KEYS:
            side, j = key
            arm = arms[side]
            versions = tuple(arm.read_parameter(j, REGISTERS[r]) for r in VERSION_REGISTERS)
            if versions != self.versions[key]:
                raise DriverError(f"{side} J{j} 固件与盘点记录不符。")
            mode = arm.read_parameter(j, 10)
            if mode not in MODE_CODES.values():
                raise DriverError(f"{side} J{j} 模式读回无效。")
            modes[key] = mode
        return modes

    @staticmethod
    def require_stationary(arms):
        for side, j in KEYS:
            arm = arms[side]
            state = arm.query_motor(j)
            if (arm.bus.status() or state.state != 0 or not abs(state.velocity_rad_s) <= .1
                    or not 0 <= time.monotonic() - state.received_at <= .15):
                raise DriverError(f"{side} J{j} 需要静止、失能且总线正常。")

    def switch(self, arms, name, record, closing):
        self.require_stationary(arms)
        before = self.read_modes(arms)
        code = MODE_CODES[name]
        attempted = []
        try:
            for (side, j), previous in before.items():
                if closing.is_set():
                    raise DriverError("服务关闭，已中止切换。")
                if previous == code:
                    continue
                record("drive.mode_axis", f"{side} J{j} 模式 {previous} → {code}")
                attempted.append((side, j))
                arms[side].write_disabled_mode(j, code)
            after = self.read_modes(arms)
            if any(actual != code for actual in after.values()):
                raise DriverError("双臂模式回读不一致。")
            self.require_stationary(arms)
            return [dict(side=side, motor_id=j, previous=old, confirmed=after[side, j])
                    for (side, j), old in before.items()]
        except Exception as exc:
            failed = []
            for side, j in reversed(attempted):
                try:
                    original = before[side, j]
                    if arms[side].read_parameter(j, 10) != original:
                        arms[side].write_disabled_mode(j, original)
                except (DriverError, OSError) as rollback:
                    failed.append(f"{side} J{j}: {rollback}")
            detail = "；".join(failed) if failed else "已核对尝试写入轴的原模式"
            raise DriverError(f"{exc}。回退结果：{detail}") from exc

    def special_id(self, key, mode):
        offset = (MODE_CODES[mode] - 1) * 0x100 if self.addressing[key] == "mode" else 0
        return key[1] + offset
