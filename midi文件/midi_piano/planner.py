"""Deterministic, hardware-free schedule construction."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FINGERS = {"index": 3, "middle": 4, "ring": 5}
MAPPING = {60: ("A", "index"), 62: ("A", "middle"), 64: ("A", "ring"),
           65: ("B", "index"), 67: ("B", "middle"), 69: ("B", "ring"), 71: ("C", "index")}


def finite(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} 必须是 {low}..{high} 内的有限数值")
    return float(value)


def read_config(path=ROOT / "config.json"):
    cfg = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    t = cfg["timing"]
    for key in ("down_seconds", "up_seconds", "min_hold_seconds", "startup_seconds", "stop_seconds"):
        finite(t[key], key, 0.02, 30)
    finite(t["gap_seconds"], "gap_seconds", 0.01, 30)
    finite(t["articulation"], "articulation", 0.1, 0.95)
    finite(t["update_hz"], "update_hz", 10, 100)
    finite(t["late_tolerance_seconds"], "late_tolerance_seconds", 0.01, 1)
    finite(cfg["arm"]["initial_move_seconds"], "initial_move_seconds", 0.02, 120)
    for a in "ABC":
        for b in "ABC":
            if a != b:
                finite(cfg["arm"]["move_seconds"][f"{a}-{b}"], f"换位 {a}-{b}", 0.02, 120)
    hand = cfg["hand"]
    if hand["position_mode"] != "joint_degrees_7" or hand["baudrate"] != 921600:
        raise ValueError("机械手仅支持 joint_degrees_7 / 921600")
    if len(hand["rest_pose"]) != 7:
        raise ValueError("rest_pose 必须包含七个关节角度")
    for i, v in enumerate(hand["rest_pose"]):
        if v is not None:
            finite(v, f"rest_pose[{i}]", 0, (100, 55, 90, 90, 90, 90, 90)[i])
    if set(cfg["notes"]) != {str(n) for n in MAPPING}:
        raise ValueError("音符映射必须恰好包含 C4–B4 七个自然音")
    for pitch, (group, finger) in MAPPING.items():
        entry = cfg["notes"][str(pitch)]
        if (entry["group"], entry["finger"]) != (group, finger):
            raise ValueError(f"音符 {pitch} 必须映射到 {group}/{finger}")
        if entry["press_degrees"] is not None:
            finite(entry["press_degrees"], f"{pitch} press_degrees", 0, 90)
            if entry["press_degrees"] == hand["rest_pose"][FINGERS[finger]]:
                raise ValueError(f"音符 {pitch} 的按压角度与释放角度相同")


def build_plan(song, cfg, speed=1.0):
    validate_config(cfg)
    finite(speed, "speed", 0.01, 4)
    for n in song.notes:
        if n.pitch not in MAPPING:
            raise ValueError(f"{n.start:.3f}s 音符 MIDI {n.pitch} 超出七音范围；可显式 --transpose，不能自动删音")
    t, arm = cfg["timing"], cfg["arm"]
    groups = [MAPPING[n.pitch][0] for n in song.notes]
    holds = []
    scale = 1 / speed
    for i, note in enumerate(song.notes):
        hold = note.duration
        if i + 1 < len(song.notes):
            delta = song.notes[i + 1].start - note.start
            if delta <= 0 or note.duration > delta + 1e-8:
                raise ValueError("只支持无重叠的单旋律")
            hold = min(hold, delta * t["articulation"])
            travel = arm["move_seconds"][f"{groups[i]}-{groups[i + 1]}"] if groups[i] != groups[i + 1] else 0
            required = t["up_seconds"] + t["gap_seconds"] + travel + t["down_seconds"]
            scale = max(scale, required / (delta - hold))
        if hold <= 0:
            raise ValueError("音符时值必须大于零")
        scale = max(scale, t["min_hold_seconds"] / hold)
        holds.append(hold)
    lead = t["startup_seconds"] + arm["initial_move_seconds"] + t["down_seconds"]
    actions = []

    def action(kind, start, duration, **kw):
        actions.append(dict(kind=kind, start=start, duration=duration, **kw))

    action("prepare", 0, t["startup_seconds"])
    action("move", t["startup_seconds"], arm["initial_move_seconds"], group=groups[0])
    rows = []
    for i, (note, hold) in enumerate(zip(song.notes, holds)):
        group, finger = MAPPING[note.pitch]
        onset = lead + note.start * scale
        release = onset + hold * scale
        action("press", onset - t["down_seconds"], t["down_seconds"], pitch=note.pitch, group=group, finger=finger)
        action("release", release, t["up_seconds"])
        rows.append(dict(index=i + 1, pitch=note.pitch, name=cfg["notes"][str(note.pitch)]["name"],
                         source_start=note.start, source_duration=note.duration, velocity=note.velocity,
                         onset=onset, hold=hold * scale, group=group, finger=finger))
        if i + 1 < len(groups) and group != groups[i + 1]:
            action("move", release + t["up_seconds"], arm["move_seconds"][f"{group}-{groups[i + 1]}"], group=groups[i + 1])
    for a, b in zip(actions, actions[1:]):
        if a["start"] + a["duration"] > b["start"] + 1e-8:
            raise ValueError("内部错误：动作时间发生重叠")
    total = max(actions[-1]["start"] + actions[-1]["duration"], lead + song.end_seconds * scale)
    warnings = song.warnings + [f"采用断奏；相邻音保持时长最多为音符起始间隔的 {t['articulation']:.0%}。",
                               "力度仅显示，不控制实物触键力度。"]
    if not arm["timing_is_measured"]:
        warnings.append("机械臂换位时间为模拟估计，不能作为实物性能证明。")
    return dict(schema_version=1, source=song.to_dict(), requested_speed=speed,
                effective_speed=1 / scale, time_scale=scale, lead_seconds=lead,
                total_seconds=total, group_changes=sum(a != b for a, b in zip(groups, groups[1:])),
                tempos=[dict(at_seconds=lead + e["at_seconds"] * scale, bpm=e["bpm"] / scale) for e in song.tempos],
                warnings=warnings, notes=rows, actions=actions)
