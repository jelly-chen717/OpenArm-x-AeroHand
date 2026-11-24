"""Right-hand chord part using the two commissioned white-key wrist tables."""
import csv
import hashlib
import io

from openarm_joint_mapping import CONVENTION_VERSION, finite
from openarm_piano_air import CHORD_SCORE, PIANO, PREPARE_S, PREVIEW_US
from openarm_start_pose import PoseStep
from paino_skill.score import parse_score
from openarm_monitor_hands import CHANNEL, Event, FingerScore

POINTS = PIANO / "右手食指_实机白键点位与关节角_20261122.csv"
COARSE = (8., 0., 0., 90., 0., 0., 2.)
PASSIVE = (None,) * 7
APPROACH_S = 3.


def compile_trial(payload, held_plan):
    run, scope = payload.get("anchor_run"), payload.get("scope")
    if type(run) is int and run == 3:
        from openarm_piano_origin import position_trial
        if scope == "start":
            return position_trial(payload)
        raise ValueError("原位双手分指使用「预览双手指法」。逐键定位可单独执行。")
    if type(run) is not int or run not in (1, 2) or scope not in ("start", "first", "prelude", "full"):
        raise ValueError("请选择两组现场腕姿之一，以及要定位或试弹的范围。")
    speed = finite(payload.get("speed", .35))
    if not .35 <= speed <= 1.:
        raise ValueError("试弹倍率范围为 0.35 至 1。")
    raw = POINTS.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    wrists = {}
    for row in csv.DictReader(io.StringIO(raw.decode("utf-8-sig"))):
        if row["arm"] != "right" or int(row["anchor_run"]) != run or row["status"] == "no_converged_ik":
            continue
        if row["coordinate_convention"] != CONVENTION_VERSION or row["finger"] != "index":
            raise ValueError("右腕点位坐标版本或参考手指不匹配。")
        if row["status"] not in ("offline_ik", "logged_ik_command"):
            raise ValueError("未知的右腕逆解状态。")
        key = int(row["key"])
        if key in wrists:
            raise ValueError(f"右腕点位表重复记录白键 {key}。")
        wrists[key] = tuple(finite(float(row[f"J{i}_deg"])) for i in range(1, 8))
    source = CHORD_SCORE.read_bytes()
    parsed = parse_score(source.decode("utf-8-sig"), finger_count=4)
    origin = round(PREPARE_S * 1e6)
    start_key = next(a["index"] for a in reversed(parsed.actions) if a["kind"] == "move"
                     and a["side"] == "right" and a["time_us"] + a["duration_us"] <= origin)
    target_key = payload.get("key", start_key) if scope == "start" else start_key
    if type(target_key) is not int:
        raise ValueError("白键号须为整数。")

    def at(key):
        if key not in wrists:
            raise ValueError(f"第 {run} 组缺少白键 {key} 的右腕关节解。")
        return wrists[key]

    prior = (held_plan or {}).get("piano_trial", {})
    held_key = (prior.get("target_key") if prior.get("scope") == "start"
                and prior.get("anchor_run") == run and prior.get("points_sha256") == digest else None)
    from_start = scope != "start" and held_key == start_key
    if from_start:
        steps = [PoseStep("24 键抬指准备", at(start_key), PASSIVE, .25)]
    elif held_key is not None:
        steps = [PoseStep(f"移至白键 {target_key}", at(target_key), PASSIVE, APPROACH_S)]
    else:
        steps = [PoseStep("右臂粗定位", COARSE, PASSIVE, APPROACH_S),
                 PoseStep(f"右食指白键 {start_key}", at(start_key), PASSIVE, APPROACH_S)]
        if target_key != start_key:
            steps.append(PoseStep(f"移至白键 {target_key}", at(target_key), PASSIVE, APPROACH_S))
    intro = sum(s.duration_s for s in steps)
    info = dict(anchor_run=run, scope=scope, speed=speed, points_sha256=digest,
                score_sha256=hashlib.sha256(source).hexdigest(), start_key=start_key,
                from_score_start=from_start, held_key=held_key, target_key=target_key, prepare_s=intro, note_count=0,
                wrist_keys=list(dict.fromkeys((held_key or start_key, target_key))))
    if scope == "start":
        return steps, info, None

    end = {"first": 2_000_000, "prelude": PREVIEW_US, "full": parsed.duration_us-origin}[scope]
    key, cursor = start_key, 0
    events, used = [], set()
    for a in parsed.actions:
        t = a["time_us"] - origin
        if not 0 <= t < end:
            continue
        if a["kind"] == "move":
            if a["side"] != "right" or a["index"] == key or t + a["duration_us"] > end:
                continue
            if t > cursor:
                steps.append(PoseStep(f"保持白键 {key}", at(key), PASSIVE, (t-cursor)/1e6/speed))
            key = a["index"]
            steps.append(PoseStep(f"移至白键 {key}", at(key), PASSIVE, a["duration_us"]/1e6/speed))
            cursor = t + a["duration_us"]
            if key not in info["wrist_keys"]:
                info["wrist_keys"].append(key)
        elif a["kind"] == "hand":
            down = a["direction"] == "D"
            changes = tuple((CHANNEL[f["finger"]], down) for f in a["fingers"] if f["side"] == "right")
            if changes:
                events.append(Event(round(intro*1e6 + t/speed), (a["line"],), changes))
                used.update(ch for ch, _ in changes)
                info["note_count"] += len(changes) if down else 0
    if end > cursor:
        steps.append(PoseStep(f"保持白键 {key}", at(key), PASSIVE, (end-cursor)/1e6/speed))
    channels = tuple(sorted(used))
    duration_us = round(intro*1e6 + end/speed)
    events.insert(0, Event(0, (), tuple((ch, False) for ch in channels)))
    score = FingerScore({"right": tuple(events)}, {"right": channels, "left": ()},
                        duration_us, 0, info["note_count"])
    return steps, info, score
