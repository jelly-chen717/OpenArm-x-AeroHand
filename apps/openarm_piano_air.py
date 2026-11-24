"""Compile the score and CSV wrist targets for the console's arm-only playback."""
import csv
from dataclasses import dataclass
from hashlib import sha256
from itertools import groupby
import json
import math
from pathlib import Path

import numpy as np

from openarm_app.kinematics.openarm_ik import IKConvergenceError, normalized_pose, pose_error
from openarm_joint_mapping import finite
from openarm_trajectory import Trajectory
from paino_skill.score import parse_score

PIANO = Path(__file__).resolve().parents[1] / "openarm/skills/paino_skill"
POINTS = PIANO / "points.csv"
MELODY_SCORE = PIANO / "music_data/槿（实琴分手·主旋律）.txt"
CHORD_SCORE = PIANO / "music_data/槿（实琴分手·纯和弦）.txt"
CHORUS_SCORE = PIANO / "music_data/槿（实琴分手·副歌纯旋律）.txt"
AIR_SCORES = {"jin": MELODY_SCORE, "jin-chords": CHORD_SCORE, "jin-chorus": CHORUS_SCORE}
CSV_ANCHORS = {"left": 19, "right": 24}
RANGES = {"left": range(1, 24), "right": range(24, 37)}
# Left index is four white-key pitches from the little finger. The two inner
# fingers use provisional spacing until the installed hand is measured.
FINGERS = {"left": {0: "pinky", 1: "ring", 2: "middle", 4: "index"},
           "right": {0: "index", 1: "middle", 2: "ring", 3: "pinky"}}
# Key 22 is played from the key-18 wrist with the left index. That keeps the
# left wrist out of the narrow space next to the right key-24 wrist.
LEFT_WRIST_KEYS = frozenset({18, 19})
POSE_FIELDS = ("x_m", "y_m", "z_m", "qw", "qx", "qy", "qz")
JOINT_FIELDS = tuple(f"q{i}" for i in range(1, 8))
PREPARE_S = 4.
PREVIEW_US = 16_000_000


@dataclass(frozen=True)
class Wrist:
    pose: tuple
    q: tuple
    position_error_m: float
    orientation_error_rad: float


def load_notes(path):
    return parse_notes(path.read_bytes())


def parse_notes(raw):
    text = raw.decode("utf-8-sig")
    score = parse_score(text, finger_count=4)
    first = round(PREPARE_S * 1e6)
    start_keys = {action["side"]: action["index"] for action in score.actions
                  if action["kind"] == "move" and action["time_us"] + action["duration_us"] <= first}
    if start_keys.keys() != RANGES.keys():
        raise ValueError("琴谱需要在起拍前给出左右臂的起始手位。")
    by_onset, active, section = {}, {}, {}
    label = ""
    for line, source in enumerate(text.splitlines(), 1):
        if source.startswith(("# 第 ", "# 前奏第 ")) or (source.startswith("# ") and " | " in source):
            label = source[2:].strip()
        section[line] = label
    for action in score.actions:
        if action["kind"] != "hand":
            continue
        line, time_us = action["line"], action["time_us"] - first
        for finger in action["fingers"]:
            fid = finger["id"]
            if action["direction"] == "D":
                active[fid] = dict(on_us=time_us, key=finger["key"], midi=finger["midi"],
                                   line=line, section=section[line], side=finger["side"])
            elif fid in active:
                note = active.pop(fid)
                note["off_us"] = time_us
                by_onset.setdefault(note["on_us"], []).append(note)
    notes = [note for _, simultaneous in sorted(by_onset.items())
             for note in sorted(simultaneous, key=lambda note: note["key"])]
    if not notes or notes[0]["on_us"] < 0 or any(
            n["key"] not in RANGES[n["side"]] or n["off_us"] <= n["on_us"] for n in notes):
        raise ValueError("琴谱起拍、时值或键号超出当前空弹谱范围。")
    return notes, score.duration_us - first, sha256(raw).hexdigest(), start_keys


def load_wrist_targets(arms, path, calibration_path, keys):
    raw = path.read_bytes()
    rows = {}
    for row in csv.DictReader(raw.decode("utf-8-sig").splitlines()):
        entry = row["arm"], int(row["key"])
        if entry[0] not in CSV_ANCHORS or entry in rows:
            raise ValueError(f"点位表手侧或键号重复：{entry}")
        rows[entry] = row
    calibration = None
    calibration_sha = None
    config = json.loads((PIANO / "calibration.json").read_bytes())
    key_step = np.array([0., -finite(config["white_pitch_m"]), 0.])
    if calibration_path:
        calibration_raw = calibration_path.read_bytes()
        try:
            data = json.loads(calibration_raw)
            calibration = data["anchor_q_rad"]
            key_step = np.array([finite(value) for value in data["key_step_m"]])
            if set(calibration) != set(CSV_ANCHORS):
                raise ValueError
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("钢琴标定需要左右 anchor_q_rad 和三维 key_step_m。") from exc
        calibration_sha = sha256(calibration_raw).hexdigest()
        if key_step.shape != (3,) or np.linalg.norm(key_step) == 0:
            raise ValueError("key_step_m 应为朝高音方向的三维每键位移。")
    result, failed = {}, {}
    for side, arm in arms.items():
        anchor = CSV_ANCHORS[side]
        reference = rows[side, anchor]
        reference_pose = normalized_pose([float(reference[field]) for field in POSE_FIELDS])
        branch_seed = np.array([finite(float(reference[field])) for field in JOINT_FIELDS])
        site_pose = None
        if calibration is not None:
            try:
                branch_seed = np.array([finite(q) for q in calibration[side]])
            except TypeError as exc:
                raise ValueError(f"{side} 的 anchor_q_rad 应为七轴角度数组。") from exc
            site_pose = arm.forward(branch_seed)
        ready = {key: row for (owner, key), row in rows.items()
                 if owner == side and row["status"] == "ready"}
        requested = keys[side] | {anchor}
        for key in sorted(requested, key=lambda k: (abs(k - anchor), k)):
            row = rows.get((side, key))
            if site_pose is not None:
                target = site_pose.copy()
                target[:3] += (key - anchor) * key_step
            elif row:
                target = normalized_pose([float(row[field]) for field in POSE_FIELDS])
            else:
                target = reference_pose.copy()
                target[:3] += (key - anchor) * key_step
            nearest = sorted(ready, key=lambda k: abs(k - key))[:2]
            already = sorted((k for owner, k in result if owner == side), key=lambda k: abs(k - key))
            seeds = ([result[side, already[0]].q] if already else []) + [branch_seed]
            csv_seeds = [[float(ready[k][field]) for field in JOINT_FIELDS] for k in nearest]
            seeds = csv_seeds + seeds if site_pose is None else seeds + csv_seeds
            for seed in seeds:
                try:
                    q = arm.inverse(target, seed, position_tolerance=.001, orientation_tolerance=.003)
                except (IKConvergenceError, ValueError):
                    continue
                pos, angle = pose_error(arm.forward(q), target)
                result[side, key] = Wrist(tuple(target), tuple(q), pos, angle)
                break
            else:
                failed[side, key] = "官方 IK 未找到该腕位的关节解"
    manifest = json.loads((PIANO / "points.json").read_bytes())
    csv_sha = sha256(raw).hexdigest()
    return result, failed, dict(points_sha256=csv_sha, original_manifest_matches=csv_sha == manifest["csv_sha256"],
                                site_calibration_sha256=calibration_sha, key_step_m=key_step.tolist())


def attacks(notes):
    """Yield all notes sharing an onset as one attack."""
    return ((onset, tuple(group)) for onset, group in groupby(notes, key=lambda note: note["on_us"]))


def wrist_keys(side, notes):
    fingers = FINGERS[side]
    first_key = notes[0]["key"]
    return {first_key - offset for offset in fingers
            if first_key - offset in RANGES[side]
            and all(note["key"] - (first_key - offset) in fingers for note in notes)}


def wrist_path(side, groups, wrists, start_key):
    states = {start_key: (0., [])}
    previous_us = 0
    for onset, simultaneous in groups:
        available = wrist_keys(side, simultaneous)
        if onset == 0:
            available &= {start_key}
        available = sorted(key for key in available if (side, key) in wrists)
        if not available:
            keys = "、".join(str(note["key"]) for note in simultaneous)
            raise ValueError(f"第 {simultaneous[0]['line']} 行，{side} 白键 {keys} 无共同腕位。")
        gap_s = max((onset - previous_us) / 1e6, .0625)
        updated = {}
        for key in available:
            target = wrists[side, key].q
            options = []
            for previous, (cost, path) in states.items():
                delta = max(abs(a - b) for a, b in zip(wrists[side, previous].q, target))
                move_cost = 0. if previous == key else .08 + delta * (1. + .25 / gap_s)
                options.append((cost + move_cost, previous, path))
            cost, _, path = min(options, key=lambda option: option[:2])
            updated[key] = cost, path + [
                dict(note, anchor_key=key, finger=FINGERS[side][note["key"] - key])
                for note in simultaneous]
        states = updated
        previous_us = onset
    return min(states.values(), key=lambda value: value[0])[1]


def _arm_waypoints(side, notes, wrists, speed, duration_s, anchor):
    points = [(PREPARE_S, wrists[side, anchor].q)]
    previous_on = previous_off = PREPARE_S
    overlaps = 0
    for onset, simultaneous in attacks(notes):
        arrival = PREPARE_S + onset / 1e6 / speed
        next_key = simultaneous[0]["anchor_key"]
        if next_key != anchor:
            # Allow an airborne shift during the prior note if its written
            # release leaves little or no gap. Preserve the finger events.
            available = arrival - previous_on
            start = max(previous_on, min(previous_off, arrival - .6 * available))
            if start < previous_off - 1e-9:
                overlaps += 1
            if start > points[-1][0]:
                points.append((start, points[-1][1]))
            points.append((arrival, wrists[side, next_key].q))
            anchor = next_key
        previous_on = arrival
        previous_off = PREPARE_S + max(note["off_us"] for note in simultaneous) / 1e6 / speed
    if duration_s > points[-1][0]:
        points.append((duration_s, points[-1][1]))
    return points, overlaps


def compile_air(arms, *, scope, speed, score_id="jin", points_path=POINTS, calibration_path=None):
    speed = finite(speed)
    if scope not in ("start", "prelude", "full") or not .35 <= speed <= 1.:
        raise ValueError("请选择起位、开头 16 秒或整份琴谱，倍率须在 0.35×–1× 之间。")
    if not isinstance(score_id, str) or score_id not in AIR_SCORES:
        raise ValueError("请选择已接入的空弹琴谱。")
    is_chord = score_id == "jin-chords"
    notes, score_us, score_sha, start_keys = load_notes(AIR_SCORES[score_id])
    # Keep the left index on the score's key with the wider physical finger spacing.
    start_keys["left"] += 3 - max(FINGERS["left"])
    if is_chord:
        # Reassign the opening notes to the current spacing.
        for side, anchor in start_keys.items():
            opening = [note for note in notes if note["side"] == side and note["on_us"] == 0]
            if opening:
                start_keys[side] = min(wrist_keys(side, opening),
                                       key=lambda key: (abs(key - anchor), key))
    if scope == "start":
        notes, score_us = [], 0
    elif scope == "prelude":
        score_us = PREVIEW_US
        notes = [dict(note, off_us=min(note["off_us"], score_us))
                 for note in notes if note["on_us"] < score_us]
    groups = {side: list(attacks(note for note in notes if note["side"] == side)) for side in RANGES}
    requested = {side: {key for _, simultaneous in hand for key in wrist_keys(side, simultaneous)}
                 for side, hand in groups.items()}
    if not is_chord:
        requested["left"] &= LEFT_WRIST_KEYS
    for side, key in start_keys.items():
        requested[side].add(key)
    wrists, failed, sources = load_wrist_targets(arms, points_path, calibration_path, requested)
    if any((side, key) not in wrists for side, key in start_keys.items()):
        raise ValueError("琴谱起位在当前坐标下无解。")
    scheduled = {side: wrist_path(side, hand, wrists, start_keys[side])
                 for side, hand in groups.items()}
    duration = PREPARE_S + score_us / 1e6 / speed
    arm_paths = {side: _arm_waypoints(side, scheduled[side], wrists, speed, duration, start_keys[side])
                 for side in RANGES}
    arm_curves = {side: Trajectory("", tuple(range(1, 8)), tuple(points), 0.)
                  for side, (points, _) in arm_paths.items()}
    times = sorted({stamp for points, _ in arm_paths.values() for stamp, _ in points})
    waypoints = [(stamp, {side: curve.sample(stamp)[0] for side, curve in arm_curves.items()})
                 for stamp in times]
    stages = {side: len({n["anchor_key"] for n in scheduled[side]} | {anchor})
              for side, anchor in start_keys.items()}
    finger_events = sorted((note for part in scheduled.values() for note in part), key=lambda n: n["on_us"])
    return dict(score_id=score_id, scope=scope, speed=speed, duration_s=duration,
                prepare_s=PREPARE_S, score_us=score_us,
                source=dict(sources, score_sha256=score_sha),
                anchors={"left": {"pinky": start_keys["left"], "index": start_keys["left"] + 4},
                         "right": {"index": start_keys["right"]}},
                hand_positions=stages, finger_events=finger_events, waypoints=waypoints,
                wrist_targets={f"{side}:{key}": vars(wrist) for (side, key), wrist in wrists.items()},
                unsolved_wrist_targets={f"{side}:{key}": reason for (side, key), reason in failed.items()},
                air_only_shifts=sum(overlap for _, overlap in arm_paths.values()))
