"""Offline keyboard targets: official IK, fixed orientation, lateral translation."""
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np

from openarm_app.kinematics import OpenArmIK
from openarm_app.joint_convention import CONVENTION_VERSION, SIGNS
from openarm_app.kinematics.openarm_ik import IKConvergenceError, pose_error
from paino_skill.scene import PIANO

CALIBRATION = PIANO / "calibration.json"
CSV_PATH = PIANO / "points.csv"
MANIFEST = PIANO / "points.json"
FIELDS = ["arm", "key", "offset_steps", "delta_y_m", "status", "position_error_m",
          "orientation_error_rad", *[f"q{i}" for i in range(1, 8)],
          "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz", "error"]


def calibration():
    cfg = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    if cfg.get('coordinate_convention') != CONVENTION_VERSION:
        raise ValueError('钢琴中点角度坐标版本不匹配，请先换算')
    if cfg["numbering"] != "white_1_based":
        raise ValueError("当前点位生成仅支持从 1 开始的白键编号")
    if not 0 < cfg["white_pitch_m"] < .1 or not 1 <= cfg["steps_each_direction"] <= 14:
        raise ValueError("键距或点位数量无效")
    return cfg


def generate():
    cfg = calibration()
    right = OpenArmIK()
    arms = {"right": right, "left": OpenArmIK(right.xml, side="left", model=right.setup.model)}
    rows = []
    for side, arm in arms.items():
        middle = arm._joints(cfg["center_q_rad"][side])
        pose = arm.forward(middle)
        anchor = cfg["anchor_keys"][side]
        for direction in (0, -1, 1):
            seed = middle.copy()
            offsets = [0] if not direction else [direction*i for i in range(1, cfg["steps_each_direction"]+1)]
            for offset in offsets:
                key = anchor + offset
                target = pose.copy()
                target[1] -= offset*cfg["white_pitch_m"]
                row = {"arm": side, "key": key, "offset_steps": offset,
                       "delta_y_m": -offset*cfg["white_pitch_m"], "status": "unreachable", "error": ""}
                row.update(zip(["x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"], target.tolist()))
                if not 1 <= key <= cfg.get("white_key_count", 52):
                    row["error"] = "超出白键范围"
                else:
                    candidates = [seed, middle]
                    alternate = middle.copy()
                    alternate[1] = SIGNS[side][1] * (.6 if side == "right" else -.6)
                    candidates.append(alternate)
                    for candidate in candidates:
                        try:
                            q = middle.copy() if offset == 0 else arm.inverse(
                                target, candidate, max_iterations=300,
                                position_tolerance=.0002, orientation_tolerance=np.deg2rad(.1))
                            pe, oe = pose_error(arm.forward(q), target)
                            if pe > .0002 or oe > np.deg2rad(.1):
                                raise IKConvergenceError("FK 验证未通过")
                            row.update(status="ready", position_error_m=pe,
                                       orientation_error_rad=oe, error="")
                            row.update({f"q{i+1}": float(v) for i, v in enumerate(q)})
                            seed = q.copy()
                            break
                        except (ValueError, IKConvergenceError) as exc:
                            row["error"] = str(exc)
                rows.append(row)
                print(f"{side} key {key}: {row['status']}", flush=True)
    rows.sort(key=lambda r: (r["arm"], r["key"]))
    buf = io.StringIO(newline="")
    writer = csv.DictWriter(buf, fieldnames=FIELDS)
    writer.writeheader()
    writer.writerows(rows)
    raw = buf.getvalue().encode("utf-8-sig")
    metadata = {"coordinate_convention": CONVENTION_VERSION,
                "calibration": cfg, "csv_sha256": hashlib.sha256(raw).hexdigest(),
                "calibration_sha256": hashlib.sha256(CALIBRATION.read_bytes()).hexdigest(),
                "scene_sha256": hashlib.sha256((PIANO / "scene.json").read_bytes()).hexdigest(),
                "counts": {s: sum(r["status"] == "ready" for r in rows if r["arm"] == s) for s in arms},
                "collision_checked": False}
    tmp = CSV_PATH.with_suffix(".csv.tmp")
    tmp.write_bytes(raw)
    tmp.replace(CSV_PATH)
    tmp = MANIFEST.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(MANIFEST)
    return metadata


def load_table():
    metadata = json.loads(MANIFEST.read_text(encoding="utf-8"))
    if metadata.get('coordinate_convention') != CONVENTION_VERSION:
        raise ValueError('钢琴点位角度坐标版本不匹配，请先换算或重新解算')
    raw = CSV_PATH.read_bytes()
    if hashlib.sha256(raw).hexdigest() != metadata["csv_sha256"]:
        raise ValueError("点位 CSV 已改变，请重新解算")
    for path, field in ((CALIBRATION, "calibration_sha256"), (PIANO / "scene.json", "scene_sha256")):
        if hashlib.sha256(path.read_bytes()).hexdigest() != metadata[field]:
            raise ValueError("场景或中点配置已改变，请重新解算点位")
    rows = []
    for item in csv.DictReader(io.StringIO(raw.decode("utf-8-sig"))):
        row = {"arm": item["arm"], "key": int(item["key"]), "offset_steps": int(item["offset_steps"]),
               "delta_y_m": float(item["delta_y_m"]), "status": item["status"], "error": item["error"]}
        if row["status"] == "ready":
            row["q"] = [float(item[f"q{i}"]) for i in range(1, 8)]
            if not np.all(np.isfinite(row["q"])):
                raise ValueError("CSV 关节数据无效")
            row["position_error_m"] = float(item["position_error_m"])
        rows.append(row)
    return {**metadata, "rows": rows}


if __name__ == "__main__":
    print(json.dumps(generate(), ensure_ascii=False, indent=2))
