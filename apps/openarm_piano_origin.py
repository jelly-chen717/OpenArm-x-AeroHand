"""Measured index anchors and outward wrist targets for the 61-key piano."""
import csv
from dataclasses import dataclass
from decimal import Decimal
from functools import lru_cache
import hashlib
import io
import math
import re

from openarm_joint_mapping import CONVENTION_VERSION, finite
from openarm_piano_air import CHORD_SCORE, FINGERS, PIANO, PREPARE_S, PREVIEW_US, attacks, parse_notes
from openarm_start_pose import PoseStep

POINTS = PIANO / "双臂钢琴原位_白键点位.csv"
FINGERING_SCORES = {
    "jin-chords": CHORD_SCORE,
    "jin-full": PIANO / "music_data/槿（60 BPM 完整钢琴版）.txt",
}
SIDES = ("left", "right")
# Each offset counts white keys from that hand's index. FINGERS uses the lowest finger as zero.
OFFSETS = {side: {n - next(i for i, f in fingers.items() if f == "index"): name
                  for n, name in fingers.items()} for side, fingers in FINGERS.items()}


@dataclass(frozen=True)
class WristPoint:
    key: int
    physical_key: int
    note: str
    xyz: tuple
    deg: tuple


@lru_cache(maxsize=1)
def _parse_points(raw):
    points = {s: {} for s in SIDES}
    anchors = {}
    for row in csv.DictReader(io.StringIO(raw.decode("utf-8-sig"))):
        side = row["arm"]
        if row["coordinate_convention"] != CONVENTION_VERSION or side not in points or row["finger"] != "index":
            raise ValueError("钢琴原位点表的手侧、参考手指或关节坐标版本不匹配。")
        if row["status"] == "no_converged_ik":
            continue
        key = int(row["key"])
        p = WristPoint(key, int(row["physical_white_key"]), row["note"],
                       tuple(finite(float(row[c+"_mm"])) for c in "xyz"),
                       tuple(finite(float(row[f"J{i}_deg"])) for i in range(1, 8)))
        if key in points[side] or p.physical_key != key-9:
            raise ValueError("钢琴点位表含重复或不一致的白键编号。")
        points[side][key] = p
        if row["status"] == "screenshot_feedback":
            if side in anchors or int(row["anchor_key"]) != key:
                raise ValueError("钢琴点位表原位记录有误。")
            anchors[side] = p
        elif row["status"] != "offline_ik":
            raise ValueError("钢琴点位表含未知逆解状态。")
    if set(anchors) != set(SIDES):
        raise ValueError("钢琴点位表缺少一侧原位反馈。")
    return points, anchors, hashlib.sha256(raw).hexdigest()


def load_origin():
    return _parse_points(POINTS.read_bytes())


def origin_step():
    _, anchors, _ = load_origin()
    label = f"钢琴原位 · 左 {anchors['left'].note} / 右 {anchors['right'].note}"
    return PoseStep(label, anchors["right"].deg, anchors["left"].deg, 3.)


def position_trial(payload):
    side, key = payload.get("side", "right"), payload.get("key")
    points, anchors, digest = load_origin()
    if side not in SIDES or type(key) is not int or key not in points[side]:
        raise ValueError("请选择该侧有固定腕姿逆解的实琴白键。")
    p = points[side][key]
    target = {s: p.deg if s == side else (None,)*7 for s in SIDES}
    steps = [PoseStep(f"{side} {p.note}", target["right"], target["left"], 3.)]
    info = dict(anchor_run=3, scope="start", side=side, physical_key=p.physical_key, note=p.note,
                target_key=key, start_key=anchors[side].key, points_sha256=digest,
                wrist_keys=[key], from_score_start=False, prepare_s=3., note_count=0)
    return steps, info, None


def score_wrist_positions(text):
    """Read the real-piano index targets printed beside each chord's onset."""
    positions = {}
    for stamp, parts in re.findall(r"^# 实琴 ([\d.]+)s｜(.+)$", text, re.MULTILINE):
        onset = int(Decimal(stamp) * 1_000_000) - round(PREPARE_S * 1_000_000)
        positions[onset] = {
            "left" if side == "左" else "right": int(physical_key) + 9
            for side, physical_key in re.findall(r"([左右])（食指位 [A-G]\d\[(\d+)\]", parts)
        }
    return positions


def fingering_preview(scope, score_id="jin-chords"):
    if scope not in ("first", "prelude", "full"):
        raise ValueError("请选择首和弦、前奏或全曲。")
    if not isinstance(score_id, str) or score_id not in FINGERING_SCORES:
        raise ValueError("请选择已接入的双手分指琴谱。")
    points, anchors, digest = load_origin()
    raw = FINGERING_SCORES[score_id].read_bytes()
    notes, duration, score_digest, _ = parse_notes(raw)
    positions = score_wrist_positions(raw.decode("utf-8-sig")) if score_id == "jin-chords" else None
    first_onset = notes[0]["on_us"]
    first_end = next((n["on_us"] for n in notes if n["on_us"] > first_onset), duration)
    limit = {"first": first_end, "prelude": PREVIEW_US, "full": duration}[scope]
    active = {side: [] for side in SIDES}
    preferred = {side: anchors[side].key for side in SIDES}
    rows = []
    for onset, chord in attacks(notes):
        if onset >= limit:
            break
        original = [dict(key=n["key"], midi=n["midi"], off_us=n["off_us"]) for n in chord]
        wrists = positions.get(onset, {}) if positions is not None else {}
        if positions is None:
            for side in SIDES:
                active[side] = [n for n in active[side] if n["off_us"] > onset]
                active[side].extend(n for n in chord if n["side"] == side)
                if active[side]:
                    choices = [key for key in points[side]
                               if all(n["key"] - key in OFFSETS[side] for n in active[side])]
                    if choices:
                        wrists[side] = preferred[side] = min(
                            choices, key=lambda key: (abs(key - preferred[side]), key))
        fingers = {side: [] for side in SIDES}
        for n in chord:
            side = n["side"]
            key = wrists.get(side)
            finger = OFFSETS[side].get(n["key"] - key) if key is not None else None
            fingers[side].append(dict(key=n["key"], physical_key=n["key"] - 9, midi=n["midi"],
                                      finger=finger, off_us=n["off_us"]))
        pending = [side for side in SIDES if fingers[side] and wrists.get(side) not in points[side]]
        distance = None
        if all(wrists.get(s) in points[s] for s in SIDES):
            distance = math.dist(points["left"][wrists["left"]].xyz, points["right"][wrists["right"]].xyz)
        rows.append(dict(time_us=onset, notes=original, wrists=wrists, fingers=fingers,
                         pending_wrists=pending, wrist_distance_mm=distance))
    first_targets = {}
    if rows:
        for side, key in rows[0]["wrists"].items():
            point = points[side].get(key)
            if point is not None:
                first_targets[side] = dict(note=point.note, physical_key=point.physical_key,
                                           xyz_mm=point.xyz, joints_deg=point.deg)
    return dict(scope=scope, score_id=score_id, score_sha256=score_digest, points_sha256=digest, duration_us=limit,
                first_wrist_targets=first_targets,
                note_count=sum(len(r["notes"]) for r in rows),
                unassigned=sum(any(f["finger"] is None for part in r["fingers"].values() for f in part)
                               for r in rows),
                pending_wrist_groups=sum(bool(r["pending_wrists"]) for r in rows), rows=rows)
