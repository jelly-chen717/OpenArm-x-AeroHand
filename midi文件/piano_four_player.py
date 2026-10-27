"""Four-finger fixed-key Aero Hand exercise. No hardware access without --run."""
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "aero-hand-open" / "sdk" / "src"))
NAMES = ("thumb_abduction", "thumb_flexion", "thumb_tendon", "index", "middle", "ring", "pinky")
FINGERS = ("index", "middle", "ring", "pinky")
NOTES = {"index": "do", "middle": "re", "ring": "mi", "pinky": "fa"}


def number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label}: 请填写有限数值，当前为 {value!r}")
    if not low <= value <= high:
        raise ValueError(f"{label}: 应在 {low}..{high} 范围内")
    return float(value)


def validate(cfg, hardware=False):
    if cfg["position_mode"] != "joint_degrees_7":
        raise ValueError("仅支持 joint_degrees_7；不要填入电机角度或腱绳位移")
    timing = cfg["timing"]
    for key in ("down_seconds", "up_seconds", "startup_seconds", "stop_seconds"):
        number(timing[key], key, 0.02, 30)
    for key in ("hold_seconds", "gap_seconds"):
        number(timing[key], key, 0, 30)
    number(timing["update_hz"], "update_hz", 10, 100)
    if hardware:
        if not isinstance(cfg["port"], str) or not cfg["port"].strip():
            raise ValueError("请先填写 port，例如 COM5")
        if cfg["baudrate"] != 921600:
            raise ValueError("当前程序使用官方默认波特率 921600")
    missing = []
    for i, name in enumerate(NAMES):
        value = cfg["rest_pose"][name]
        if value is None:
            missing.append(f"rest_pose.{name}")
        else:
            number(value, name, 0, (100, 55, 90, 90, 90, 90, 90)[i])
    for name in FINGERS:
        value = cfg["press"][name]
        if value is None:
            missing.append(f"press.{name}")
        else:
            number(value, f"press.{name}", 0, 90)
            if value == cfg["rest_pose"][name]:
                raise ValueError(f"{name}: 按下与复位值不能相同")
    if hardware and missing:
        raise ValueError("位置尚未填写：" + ", ".join(missing))
    return missing


def build_targets(cfg):
    from aero_open_sdk.aero_hand_constants import AeroHandConstants
    from aero_open_sdk.joints_to_actuations import JointsToActuationsModel
    constants = AeroHandConstants()
    model = JointsToActuationsModel()

    def convert(p):
        expanded = [p[0], p[1], p[2], p[2]] + [v for a in p[3:] for v in (a, a, a)]
        result = list(model.hand_actuations(expanded))
        for i, value in enumerate(result):
            number(float(value), f"转换后的电机 {i}", constants.actuation_lower_limits[i], constants.actuation_upper_limits[i])
        return result

    rest = [cfg["rest_pose"][name] for name in NAMES]
    targets = {"rest": convert(rest)}
    for name in FINGERS:
        pose = rest.copy()
        pose[NAMES.index(name)] = cfg["press"][name]
        targets[name] = convert(pose)
    return targets, constants


class Transport:
    """Official 16-byte position protocol; propagate write errors instead of swallowing them."""
    def __init__(self, cfg, constants):
        from serial import Serial
        self.constants = constants
        self.ser = Serial(cfg["port"], cfg["baudrate"], timeout=1, write_timeout=0.3)

    def packet(self, opcode, values):
        packet = struct.pack("<2B7H", opcode, 0, *values)
        if self.ser.write(packet) != len(packet):
            raise IOError("串口写入不完整")

    def read_pose(self):
        self.ser.reset_input_buffer()
        self.packet(0x22, [0] * 7)
        response = self.ser.read(16)
        if len(response) != 16 or response[:2] != bytes((0x22, 0)):
            raise IOError("未收到有效当前位置，取消启动")
        words = struct.unpack("<2B7H", response)[2:]
        return [lo + w / 65535 * (hi - lo) for w, lo, hi in zip(words, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)]

    def send(self, pose):
        words = []
        for i, (value, lo, hi) in enumerate(zip(pose, self.constants.actuation_lower_limits, self.constants.actuation_upper_limits)):
            number(float(value), f"电机 {i}", lo, hi)
            words.append(round((value - lo) / (hi - lo) * 65535))
        self.packet(0x11, words)

    def close(self):
        self.ser.close()


class Motion:
    def __init__(self, send, pose, hz, clock=time.monotonic, sleep=time.sleep):
        self.send, self.pose, self.hz = send, list(pose), hz
        self.clock, self.sleep = clock, sleep

    def until(self, deadline):
        remaining = deadline - self.clock()
        if remaining > 0:
            self.sleep(remaining)

    def move(self, target, duration):
        start, origin = self.clock(), self.pose.copy()
        steps = max(1, math.ceil(duration * self.hz))
        for step in range(1, steps + 1):
            self.until(start + duration * step / steps)
            fraction = min(1.0, max(step / steps, (self.clock() - start) / duration))
            weight = fraction * fraction * (3 - 2 * fraction)
            pose = [a + (b - a) * weight for a, b in zip(origin, target)]
            self.send(pose)
            self.pose = pose
            if fraction >= 1:
                break


def play(motion, targets, timing, sequence, cycles):
    period = sum(timing[k] for k in ("down_seconds", "hold_seconds", "up_seconds", "gap_seconds"))
    deadline = motion.clock()
    count = 0
    while cycles == 0 or count < cycles:
        for finger in sequence:
            print(f"第 {count + 1} 轮: {finger} / {NOTES[finger]}", flush=True)
            motion.move(targets[finger], timing["down_seconds"])
            motion.sleep(timing["hold_seconds"])
            motion.move(targets["rest"], timing["up_seconds"])
            deadline += period
            # If late, preserve release time; never accelerate to catch up.
            deadline = max(deadline, motion.clock() + timing["gap_seconds"])
            motion.until(deadline)
        count += 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="Aero Hand do-re-mi-fa 循环；默认仅模拟")
    parser.add_argument("--config", type=Path, default=ROOT / "piano_four_config.json")
    parser.add_argument("--run", action="store_true", help="连接并实际驱动机械手")
    parser.add_argument("--cycles", type=int, default=3, help="轮数；0 为持续运行")
    parser.add_argument("--finger", choices=FINGERS, help="仅试弹一根手指")
    args = parser.parse_args(argv)
    if args.cycles < 0:
        parser.error("cycles 不能为负数")
    transport = motion = targets = None
    failed = False
    try:
        cfg = json.loads(args.config.read_text(encoding="utf-8-sig"))
        missing = validate(cfg, args.run)
        sequence = (args.finger,) if args.finger else FINGERS
        timing = cfg["timing"]
        if not args.run:
            print("模拟模式：不打开串口、不驱动机械手。位置占位值：", ", ".join(missing) or "无")
            for name in sequence:
                print(f"{name}: {cfg['rest_pose'][name]} -> {cfg['press'][name]} -> {cfg['rest_pose'][name]}")
            # Abstract values represent motion phases only, never physical angles.
            targets = {"rest": [0.0] * 7}
            for name in FINGERS:
                targets[name] = [float(i == NAMES.index(name)) for i in range(7)]
            motion = Motion(lambda pose: None, targets["rest"], timing["update_hz"])
        else:
            targets, constants = build_targets(cfg)
            transport = Transport(cfg, constants)
            current = transport.read_pose()
            motion = Motion(transport.send, current, timing["update_hz"])
            print("已读取当前位置，缓慢进入准备姿态。Ctrl+C 停止并尝试复位。", flush=True)
            motion.move(targets["rest"], timing["startup_seconds"])
        play(motion, targets, timing, sequence, args.cycles)
    except KeyboardInterrupt:
        print("\n已停止循环，尝试复位。")
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        failed = True
    finally:
        if transport is not None:
            try:
                if motion is not None:
                    motion.move(targets["rest"], cfg["timing"]["stop_seconds"])
                    print("复位指令已发送；请观察琴键是否实际释放。")
            except (Exception, KeyboardInterrupt) as exc:
                print(f"复位未完成，请手动检查并释放琴键：{exc}", file=sys.stderr)
                failed = True
            finally:
                transport.close()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
