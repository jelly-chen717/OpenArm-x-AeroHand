"""Strict declarative piano text parser. Never evaluates source as code."""
from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation, localcontext
import re

SEQUENCES = {4: {"left": "1234", "right": "789x"},
             5: {"left": "12345", "right": "6789x"}}
PHYSICAL = {4: {"left": ("pinky", "ring", "middle", "index"),
                "right": ("index", "middle", "ring", "pinky")},
            5: {"left": ("pinky", "ring", "middle", "index", "thumb"),
                "right": ("thumb", "index", "middle", "ring", "pinky")}}
WHITES = tuple(m for m in range(21, 109) if m % 12 in (0, 2, 4, 5, 7, 9, 11))
NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
MOVE = re.compile(rf"(left|right)_arm_move\(\s*([+-]?\d+)\s*,\s*({NUMBER})\s*\)")
HAND = re.compile(r"hand\(\s*([DU])\s*,\s*([0-9x]+)\s*\)")
SLEEP = re.compile(rf"sleep\(\s*({NUMBER})\s*\)")
MAX_US = 24 * 3600 * 1_000_000


@dataclass(frozen=True)
class Score:
    finger_count: int
    actions: tuple
    cursor_us: int
    duration_us: int

    def summary(self):
        return {**asdict(self), "action_count": len(self.actions),
                "note_count": sum(len(a["fingers"]) for a in self.actions
                                  if a["kind"] == "hand" and a["direction"] == "D"),
                "sample_midis": sorted({f["midi"] for a in self.actions if a["kind"] == "hand"
                                        for f in a["fingers"] if f["midi"] is not None})}


def parse_score(text, *, finger_count):
    if type(finger_count) is not int or finger_count not in SEQUENCES:
        raise ValueError("必须显式指定 finger_count = 4 或 5")
    if not isinstance(text, str) or len(text) > 512_000:
        raise ValueError("文本必须小于 512000 字符")
    cursor, actions = 0, []
    indices = {"left": None, "right": None}
    ends = {"left": 0, "right": 0}
    held = set()
    fingers = {char: (side, i, PHYSICAL[finger_count][side][i])
               for side, chars in SEQUENCES[finger_count].items() for i, char in enumerate(chars)}

    def fail(line, message):
        raise ValueError(f"第 {line} 行（{cursor/1e6:.6f} s）：{message}")

    def microseconds(token, line, positive=False):
        try:
            if len(token) > 64:
                raise ValueError
            value = Decimal(token)
            if not value.is_finite() or value < 0 or value > MAX_US // 1_000_000:
                raise ValueError
            if value and value < Decimal("0.000001"):
                raise ValueError
            with localcontext() as context:
                context.prec = 80
                scaled = value * 1_000_000 if value else Decimal(0)
            if scaled != scaled.to_integral_value() or (positive and scaled == 0):
                raise ValueError
            return int(scaled)
        except (ValueError, InvalidOperation):
            fail(line, "时间必须有效、精确到整数微秒，移动时间大于 0，等待时间不小于 0")

    for line, raw in enumerate(text.lstrip("\ufeff").splitlines(), 1):
        source = raw.split("#", 1)[0].strip()
        if not source:
            continue
        match = SLEEP.fullmatch(source)
        if match:
            cursor += microseconds(match[1], line)
            if cursor > MAX_US:
                fail(line, "总时间超过 24 小时")
            continue
        match = MOVE.fullmatch(source)
        if match:
            side, index = match[1], int(match[2])
            duration = microseconds(match[3], line, positive=True)
            lo, hi = (1, 23) if side == "left" else (24, 52)
            if not lo <= index <= hi-finger_count+1:
                fail(line, f"{side} 覆盖范围 {index}–{index+finger_count-1} 超出 {lo}–{hi}")
            if ends[side] > cursor:
                fail(line, f"{side} 上次移动尚未完成")
            if any(fingers[f][0] == side for f in held):
                fail(line, f"{side} 移动前必须抬起所有手指")
            if cursor + duration > MAX_US:
                fail(line, "移动结束时间超过 24 小时")
            indices[side], ends[side] = index, cursor+duration
            actions.append({"time_us": cursor, "line": line, "kind": "move", "side": side,
                            "index": index, "duration_us": duration})
        else:
            match = HAND.fullmatch(source)
            if not match:
                fail(line, "未知指令；只允许 left_arm_move / right_arm_move / hand / sleep")
            direction, chars = match[1], match[2]
            if len(set(chars)) != len(chars):
                fail(line, "同一指令不能重复列出手指")
            resolved = []
            for char in chars:
                if char not in fingers:
                    fail(line, f"{finger_count} 指模式不允许手指 {char}")
                side, offset, physical = fingers[char]
                if direction == "D":
                    if indices[side] is None:
                        fail(line, f"{side} 尚未指定 index")
                    if ends[side] > cursor:
                        fail(line, f"{side} 移动未完成，不能按键")
                    if char in held:
                        fail(line, f"手指 {char} 尚未抬起，不能重复按下")
                    held.add(char)
                else:
                    held.discard(char)  # Already released: idempotent release.
                key = indices[side]+offset if indices[side] is not None else None
                resolved.append({"id": char, "side": side, "finger": physical,
                                 "key": key, "midi": WHITES[key-1] if key is not None else None})
            actions.append({"time_us": cursor, "line": line, "kind": "hand",
                            "direction": direction, "fingers": resolved})
        if len(actions) > 20000:
            fail(line, "动作数超过 20000")
    if held:
        raise ValueError("文件结束时仍有手指按下："+"".join(sorted(held)))
    if not actions:
        raise ValueError("文件没有动作")
    return Score(finger_count, tuple(actions), cursor, max(cursor, *ends.values()))
