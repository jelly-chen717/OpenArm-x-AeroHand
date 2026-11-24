"""Single-owner serial transport and interruptible piano test sequences.

Uses aero-open-sdk 0.1.0.dev1 for protocol and tendon mapping. No homing,
torque/speed settings, motor IDs or arm commands are issued here.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import random
import threading
import time
from typing import Callable

FINGERS = {3: "食指", 4: "中指", 5: "无名指", 6: "小指"}
CHANNELS = ("拇指外展", "拇指屈曲", "拇指腱", *FINGERS.values())
MODES = ("全部一起", "随机组合", "逐指轮流")


def number(value, name, minimum, maximum):
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum}–{maximum} 之间")
    return value


@dataclass(frozen=True)
class Finger:
    channel: int
    home: float
    press: float
    transition: float = 0.5

    def __post_init__(self):
        if self.channel not in FINGERS:
            raise ValueError("只允许食指、中指、无名指、小指")
        number(self.home, "抬起角度", 0, 90)
        number(self.press, "按下角度", 0, 90)
        number(self.transition, "过渡时间", 0, 10)


@dataclass(frozen=True)
class Plan:
    fingers: tuple[Finger, ...]
    mode: str = MODES[0]
    press_dwell: float = 0.3
    home_dwell: float = 0.3
    cycles: int = 10

    def __post_init__(self):
        if not self.fingers or len({f.channel for f in self.fingers}) != len(self.fingers):
            raise ValueError("至少选择一指，且不能重复")
        if self.mode not in MODES:
            raise ValueError("未知循环模式")
        number(self.press_dwell, "按下停留", 0, 60)
        number(self.home_dwell, "抬起停留", 0, 60)
        if not isinstance(self.cycles, int) or not 0 <= self.cycles <= 100000:
            raise ValueError("循环次数必须是 0–100000 的整数；0 为持续循环")
        if any(f.home == f.press for f in self.fingers):
            raise ValueError("循环前请分别设置抬起和按下位置，不能相同")
        if self.press_dwell == self.home_dwell == 0 and all(f.transition == 0 for f in self.fingers):
            raise ValueError("直接目标模式至少设置一段停留时间，不能无间隔反转目标")


def groups(plan, rng=random):
    channels = [f.channel for f in plan.fingers]
    n = 0
    while plan.cycles == 0 or n < plan.cycles:
        if plan.mode == "全部一起":
            yield set(channels)
        elif plan.mode == "随机组合":
            yield set(rng.sample(channels, rng.randint(1, len(channels))))
        else:
            yield {channels[n % len(channels)]}
        n += 1


class AeroTransport:
    """All methods must be called by one worker, never by the Tk thread."""

    def __init__(self, demo=False):
        self.demo = demo
        self.hand = None
        self.last = None
        self.raw = None
        self.fault = False

    @property
    def connected(self):
        return self.hand is not None

    def connect(self, port):
        if self.connected:
            raise RuntimeError("已经连接")
        from aero_open_sdk.aero_hand_constants import AeroHandConstants
        from aero_open_sdk.joints_to_actuations import JointsToActuationsModel
        from aero_open_sdk.actuations_to_joints import ActuationsToJointsModelCompact
        self.constants = AeroHandConstants()
        self.forward = JointsToActuationsModel()
        self.inverse = ActuationsToJointsModelCompact()
        self.fault = False
        if self.demo:
            self.hand = object()
            self.raw = [12000, 14000, 18000, 5000, 5000, 5000, 5000]
        else:
            from aero_open_sdk.aero_hand import AeroHand
            from serial import SerialTimeoutException

            class CheckedHand(AeroHand):
                # SDK public setters otherwise print and swallow write timeouts.
                def _send_data(self, header, payload=None):
                    try:
                        return super()._send_data(header, [0] * 7 if payload is None else payload)
                    except SerialTimeoutException as exc:
                        raise OSError("Aero 串口写入超时") from exc

            self.hand = CheckedHand(port=port, baudrate=921600)
            self.hand.ser.timeout = 0.05
            self.hand.ser.write_timeout = 0.05
        try:
            return self.read()
        except Exception:
            self.close()
            raise

    def close(self):
        hand, self.hand = self.hand, None
        self.last = None
        self.raw = None
        if hand is not None and not self.demo:
            hand.ser.close()

    @staticmethod
    def _seven(values, label):
        if values is None or len(values) != 7 or not all(math.isfinite(float(x)) for x in values):
            raise OSError(f"{label}反馈缺失/无效；已中止指令，请检查后重新连接")
        return list(map(float, values))

    def encode(self, actuations):
        return [round((v - low) / (high - low) * 65535) for v, low, high in zip(
            actuations, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)]

    def decode(self, raw):
        return [low + v / 65535 * (high - low) for v, low, high in zip(
            raw, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)]

    def read(self, *, telemetry=True):
        if not self.connected or self.fault:
            raise OSError("未连接或故障已锁定；请重新连接")
        try:
            if self.demo:
                act = self.decode(self.raw)
                currents, temps, speeds = [0.] * 7, [25.] * 7, [0.] * 7
            else:
                act = self._seven(self.hand.get_actuations(), "位置")
                if telemetry or self.last is None:
                    currents = self._seven(self.hand.get_actuator_currents(), "电流")
                    temps = self._seven(self.hand.get_actuator_temperatures(), "温度")
                    speeds = self._seven(self.hand.get_actuator_speeds(), "转速")
                else:
                    currents, temps, speeds = (self.last[k] for k in ("currents", "temperatures", "speeds"))
            q = self._seven([math.degrees(v) for v in self.inverse.hand_joints(
                [math.radians(v) for v in act])], "估算关节")
            self.raw = self.encode(act)
            self.last = dict(time=time.monotonic(), q=q, actuations=act,
                             currents=currents, temperatures=temps, speeds=speeds)
            return self.last
        except Exception:
            self.fault = True
            raise

    def target(self, channel, degrees):
        if channel not in FINGERS:
            raise ValueError("禁止发送拇指目标")
        degrees = number(degrees, "关节指令角度", 0, 90)
        compact = [0.] * 7
        compact[channel] = degrees
        full = [compact[0], compact[1], compact[2], compact[2]]
        for q in compact[3:]:
            full.extend([q] * 3)
        # The official SDK owns the mapping; no copy of its model here.
        act = self.forward.hand_actuations(full)
        raw = self.encode(act)[channel]
        if not 0 <= raw <= 65535:
            raise ValueError("官方映射结果超出驱动范围")
        return raw

    def write(self, raw):
        if not self.connected or self.fault or self.last is None:
            raise OSError("当前连接不可执行")
        if time.monotonic() - self.last["time"] > 1.0:
            self.fault = True
            raise OSError("反馈超过 1 秒未更新；停止发送")
        self.write_position_frame(raw)

    def write_position_frame(self, raw):
        """Write one position packet using the transport's existing connection."""
        if not self.connected or self.fault or self.last is None:
            raise OSError("当前连接不可执行")
        if len(raw) != 7 or any(type(v) is not int or not 0 <= v <= 65535 for v in raw):
            raise ValueError("无效驱动目标")
        try:
            if self.demo:
                self.raw = list(raw)
            else:
                from aero_open_sdk.aero_hand import CTRL_POS
                # Send through official framing. Unselected channels retain exact
                # encoder counts, avoiding inverse-model/rounding-induced motion.
                self.hand._send_data(CTRL_POS, list(raw))
        except Exception:
            self.fault = True
            raise


class Interrupted(Exception):
    pass


class Runner:
    def __init__(self, bus, stop: threading.Event, emit: Callable):
        self.bus, self.stop, self.emit = bus, stop, emit
        self.base = None
        self.last_poll = 0.

    def begin(self):
        feedback = self.bus.read()
        self.emit("feedback", feedback)
        self.base = list(self.bus.raw)
        self.last_poll = time.monotonic()

    def poll(self):
        if time.monotonic() - self.last_poll >= 0.2:
            self.emit("feedback", self.bus.read())
            self.last_poll = time.monotonic()
            return True
        return False

    def pause(self, duration, interrupt=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if interrupt and self.stop.is_set():
                raise Interrupted()
            self.poll()
            remaining = min(0.02, max(0, deadline - time.monotonic()))
            if interrupt:
                self.stop.wait(remaining)
            else:
                time.sleep(remaining)

    def move(self, goals, durations, interrupt=True):
        if not goals or set(goals) != set(durations):
            raise ValueError("每个目标必须有对应过渡时间")
        durations = {ch: number(value, "过渡时间", 0, 10) for ch, value in durations.items()}
        if interrupt and self.stop.is_set():
            raise Interrupted()
        if self.base is None:
            raise RuntimeError("动作开始前必须读取反馈")
        # begin() reads once; retain periodic health checks rather than doing four
        # serial request/reply round trips before every press and release.
        refreshed = self.poll()
        start = list(self.bus.raw if refreshed else self.base)
        targets = {ch: self.bus.target(ch, q) for ch, q in goals.items()}
        total = max(durations.values())
        begun = previous = time.monotonic()
        while True:
            if interrupt and self.stop.is_set():
                raise Interrupted()
            now = time.monotonic()
            if now - previous > 0.3:
                self.bus.fault = True
                raise OSError("调度延迟超过 0.3 秒，已中止；不会补发动作")
            previous = now
            elapsed = now - begun
            raw = list(self.base)
            for ch, target in targets.items():
                t = 1. if durations[ch] == 0 else min(1., elapsed / durations[ch])
                raw[ch] = round(start[ch] + (target - start[ch]) * t)
            self.bus.write(raw)
            self.base = list(raw)
            if elapsed >= total:
                return
            self.poll()
            # At most 100 Hz interpolation, with no artificial wait at the end
            # and no catch-up burst after blocking IO. Zero sends one endpoint.
            delay = min(0.01, max(0., total - (time.monotonic() - begun)))
            if interrupt:
                self.stop.wait(delay)
            else:
                time.sleep(delay)

    def run(self, plan):
        self.begin()
        homes = {f.channel: f.home for f in plan.fingers}
        durations = {f.channel: f.transition for f in plan.fingers}
        try:
            # Explicit Start first positions selected fingers at configured HOME.
            self.move(homes, durations)
            for n, selected in enumerate(groups(plan), 1):
                goals = {f.channel: f.press if f.channel in selected else f.home for f in plan.fingers}
                self.move(goals, durations)
                self.pause(plan.press_dwell)
                self.move(homes, durations)
                self.pause(plan.home_dwell)
                self.emit("cycle", (n, sorted(selected)))
        except Interrupted:
            # Only user Stop returns home. IO/feedback errors never cause blind
            # follow-up motion; they latch the fault in the worker instead.
            self.move(homes, durations, interrupt=False)
            self.emit("log", "已停止循环并发送所选手指的抬起目标；请观察实物回弹。")
