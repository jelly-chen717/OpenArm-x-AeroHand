"""Device contracts and deterministic simulation. No import-time device access."""
import math
from pathlib import Path
import struct
import sys
import time

from .planner import FINGERS, finite, validate_config


class Clock:
    def now(self):
        return time.monotonic()

    def sleep(self, seconds):
        if seconds > 0:
            time.sleep(seconds)


class VirtualClock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += max(0, seconds)


class SimHand:
    def __init__(self, clock):
        self.clock = clock
        self.released = True
        self.events = []

    def connect(self):
        pass

    def read_pose(self):
        return None  # No invented physical angles.

    def release(self, duration):
        self.events.append((self.clock.now(), "release"))
        self.clock.sleep(duration)
        self.released = True

    def press(self, pitch, duration):
        if not self.released:
            raise RuntimeError("前一个音尚未释放")
        self.released = False
        self.events.append((self.clock.now(), "press", pitch))
        self.clock.sleep(duration)

    def close(self):
        pass


class SimArm:
    def __init__(self, clock, hand):
        self.clock, self.hand = clock, hand
        self.group = None
        self.pending = None
        self.stopped = False
        self.events = []

    def connect(self):
        pass

    def move_to(self, group, duration):
        if not self.hand.released:
            raise RuntimeError("琴键未释放，禁止换位")
        self.pending = (group, self.clock.now() + duration)
        self.events.append((self.clock.now(), "move", group))

    def wait_until_arrived(self, group, timeout):
        if self.pending is None or self.pending[0] != group:
            raise RuntimeError("没有对应的移动请求")
        remaining = self.pending[1] - self.clock.now()
        if remaining > timeout + 1e-8:
            raise TimeoutError("机械臂模拟到位超时")
        self.clock.sleep(remaining)
        self.group = group
        self.pending = None

    def stop(self):
        self.stopped = True
        self.pending = None

    def close(self):
        pass


class UnconfiguredOpenArm:
    """A real adapter must implement feedback-based arrival, NOT sleep-based arrival."""
    def connect(self):
        raise RuntimeError("OpenArm 真实控制接口尚未接入；未打开机械臂或机械手连接")

    def move_to(self, group, duration):
        raise NotImplementedError("待接入真实轨迹控制")

    def wait_until_arrived(self, group, timeout):
        raise NotImplementedError("待接入真实到位反馈")

    def stop(self):
        raise NotImplementedError("待接入真实停止接口")

    def close(self):
        pass


def validate_hardware(cfg):
    validate_config(cfg)
    missing = []
    if not isinstance(cfg["hand"]["port"], str) or not cfg["hand"]["port"].strip():
        missing.append("hand.port")
    if any(v is None for v in cfg["hand"]["rest_pose"]):
        missing.append("hand.rest_pose")
    missing.extend(f"notes.{n}.press_degrees" for n, data in cfg["notes"].items() if data["press_degrees"] is None)
    missing.extend(f"arm.poses.{g}" for g in "ABC" if cfg["arm"]["poses"].get(g) is None)
    if cfg["arm"]["timing_is_measured"] is not True:
        missing.append("机械臂实测换位时间")
    detail = "；缺少标定：" + ", ".join(missing) if missing else ""
    # Deliberately unconditional: filling JSON cannot activate an absent adapter.
    raise RuntimeError("OpenArm 真实适配器尚未实现，run 已阻止，未打开任何设备" + detail)


class SerialHand:
    """Prepared hand backend; CLI cannot reach it until OpenArm is integrated.

    Construction only converts calibrated poses. connect() is the sole serial opener.
    """
    def __init__(self, cfg, clock=None):
        validate_config(cfg)
        if any(v is None for v in cfg["hand"]["rest_pose"]) or any(n["press_degrees"] is None for n in cfg["notes"].values()):
            raise ValueError("机械手姿态尚未标定")
        sdk = Path(__file__).resolve().parents[2] / "aero-hand-open" / "sdk" / "src"
        if str(sdk) not in sys.path:
            sys.path.insert(0, str(sdk))
        from aero_open_sdk.aero_hand_constants import AeroHandConstants
        from aero_open_sdk.joints_to_actuations import JointsToActuationsModel
        self.constants = AeroHandConstants()
        model = JointsToActuationsModel()

        def convert(pose):
            expanded = pose[:3] + [pose[2]] + [v for a in pose[3:] for v in (a, a, a)]
            result = list(model.hand_actuations(expanded))
            for i, (v, lo, hi) in enumerate(zip(result, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)):
                finite(float(v), f"电机 {i}", lo, hi)
            return result

        self.cfg, self.clock = cfg, clock or Clock()
        rest = list(cfg["hand"]["rest_pose"])
        self.targets = {"rest": convert(rest)}
        for pitch, item in cfg["notes"].items():
            pose = rest.copy()
            pose[FINGERS[item["finger"]]] = item["press_degrees"]
            self.targets[int(pitch)] = convert(pose)
        self.ser = None
        self.pose = None
        self.released = False

    def connect(self):
        from serial import Serial
        h = self.cfg["hand"]
        self.ser = Serial(h["port"], h["baudrate"], timeout=1, write_timeout=0.3)

    def packet(self, opcode, words):
        packet = struct.pack("<2B7H", opcode, 0, *words)
        if self.ser.write(packet) != len(packet):
            raise IOError("串口写入不完整")

    def read_pose(self):
        self.ser.reset_input_buffer()
        self.packet(0x22, [0] * 7)
        data = self.ser.read(16)
        if len(data) != 16 or data[:2] != bytes((0x22, 0)):
            raise IOError("未读到有效当前位置，取消运动")
        words = struct.unpack("<2B7H", data)[2:]
        self.pose = [lo + w / 65535 * (hi - lo) for w, lo, hi in zip(words, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)]
        return self.pose.copy()

    def send(self, pose):
        words = []
        for i, (v, lo, hi) in enumerate(zip(pose, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)):
            finite(float(v), f"电机 {i}", lo, hi)
            words.append(round((v - lo) / (hi - lo) * 65535))
        self.packet(0x11, words)

    def move(self, target, duration):
        if self.pose is None:
            raise RuntimeError("必须先读取有效机械手位置")
        origin, start = self.pose.copy(), self.clock.now()
        count = max(1, math.ceil(duration * self.cfg["timing"]["update_hz"]))
        for step in range(1, count + 1):
            self.clock.sleep(start + duration * step / count - self.clock.now())
            if self.clock.now() > start + duration * step / count + self.cfg["timing"]["late_tolerance_seconds"]:
                raise TimeoutError("机械手插值超时")
            f = step / count
            w = f * f * (3 - 2 * f)
            pose = [a + (b - a) * w for a, b in zip(origin, target)]
            self.send(pose)
            self.pose = pose

    def press(self, pitch, duration):
        if not self.released:
            raise RuntimeError("前一个音尚未释放")
        self.released = False
        self.move(self.targets[pitch], duration)

    def release(self, duration):
        self.move(self.targets["rest"], duration)
        self.released = True  # Command state only; not a piano-key sensor reading.

    def close(self):
        if self.ser is not None:
            self.ser.close()
