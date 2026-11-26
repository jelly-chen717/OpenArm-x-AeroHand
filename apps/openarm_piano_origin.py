"""Calibrated index anchors and wrist targets for the 61-key piano."""
import csv
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import io
import json
import math

from openarm_joint_mapping import CONVENTION_VERSION, finite
from openarm_piano_air import FINGERS, PIANO
from openarm_start_pose import PoseStep
from paino_skill.score import PHYSICAL, SEQUENCES, WHITES

POINTS = PIANO / "双臂钢琴原位_白键点位.csv"
CALIBRATION = PIANO.parents[2] / "configs/openarm_piano_white_keys.json"
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


@dataclass(frozen=True)
class PianoFingerProjection:
    score_keys: range
    step_y_mm: float
    anchors: dict

    @classmethod
    def load(cls, anchors=None):
        config = json.loads(CALIBRATION.read_text(encoding="utf-8"))
        first = config["score_key_at_first"]
        keys = range(first, first + config["white_key_count"])
        pitch = finite(config["white_key_pitch_mm"])
        direction = config["higher_note_y_sign"]
        if (config["frame"] != "arm_origin" or config["coordinate_convention"] != CONVENTION_VERSION
                or not 1 <= keys.start < keys.stop <= len(WHITES) + 1
                or pitch <= 0 or direction not in (-1, 1)):
            raise ValueError("白键估算所需的实琴标定无效。")
        if anchors is None:
            _, anchors, _ = load_origin()
        return cls(keys, pitch * direction, {side: (point.key, point.xyz[1]) for side, point in anchors.items()})

    def keys_at(self, side, wrist_y_mm, *, offsets=None):
        """Project measured wrist Y onto the calibrated white-key axis."""
        key, anchor_y = self.anchors[side]
        index_key = key + math.floor((wrist_y_mm - anchor_y) / self.step_y_mm + .5)
        by_finger = {finger: index_key + offset for offset, finger in (offsets or OFFSETS[side]).items()}
        return {code: by_finger[finger]
                for code, finger in zip(SEQUENCES[4][side], PHYSICAL[4][side])
                if by_finger[finger] in self.score_keys}


@lru_cache(maxsize=1)
def _parse_points(raw):
    points = {s: {} for s in SIDES}
    anchors = {}
    for row in csv.DictReader(io.StringIO(raw.decode("utf-8-sig"))):
        side = row["arm"]
        if row["coordinate_convention"] != CONVENTION_VERSION or side not in points or row["finger"] != "index":
            raise ValueError("钢琴原位点表的手侧、参考手指或关节坐标版本不匹配。")
        status = row["status"]
        if status == "no_converged_ik":
            continue
        key = int(row["key"])
        p = WristPoint(key, int(row["physical_white_key"]), row["note"],
                       tuple(finite(float(row[c+"_mm"])) for c in "xyz"),
                       tuple(finite(float(row[f"J{i}_deg"])) for i in range(1, 8)))
        if key in points[side] or p.physical_key != key-9:
            raise ValueError("钢琴点位表含重复或不一致的白键编号。")
        points[side][key] = p
        is_ik_anchor = status == "offline_ik" and row.get("direction") == "anchor"
        if status == "screenshot_feedback" or is_ik_anchor:
            if side in anchors or int(row["anchor_key"]) != key:
                raise ValueError("钢琴点位表原位记录有误。")
            anchors[side] = p
        elif status != "offline_ik":
            raise ValueError("钢琴点位表含未知逆解状态。")
    if set(anchors) != set(SIDES):
        raise ValueError("钢琴点位表缺少一侧原位。")
    return points, anchors, hashlib.sha256(raw).hexdigest()


def load_origin():
    return _parse_points(POINTS.read_bytes())


def origin_step(anchors=None):
    if anchors is None:
        _, anchors, _ = load_origin()
    label = f"钢琴原位 · 左 {anchors['left'].note} / 右 {anchors['right'].note}"
    return PoseStep(label, anchors["right"].deg, anchors["left"].deg, 3.)


def position_trial(payload):
    side = payload.get("side", "right")
    points, anchors, digest = load_origin()
    if side not in SIDES:
        raise ValueError("请选择左手或右手。")
    key = payload.get("key", anchors[side].key)
    if type(key) is not int or key not in points[side]:
        raise ValueError("请选择该侧有固定腕姿逆解的实琴白键。")
    p = points[side][key]
    target = {s: p.deg if s == side else (None,)*7 for s in SIDES}
    steps = [PoseStep(f"{side} {p.note}", target["right"], target["left"], 3.)]
    info = dict(anchor_run=3, scope="start", side=side, physical_key=p.physical_key, note=p.note,
                target_key=key, start_key=anchors[side].key, points_sha256=digest,
                wrist_keys=[key], from_score_start=False, prepare_s=3., note_count=0)
    return steps, info, None
