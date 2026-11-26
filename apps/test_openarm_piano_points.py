"""Offline regressions for the physical white-key table and its sequence cost."""
import csv
import json
import math

import numpy as np
import pytest

from build_openarm_piano_points import CALIBRATION, smooth_path
from openarm_app.kinematics import OpenArmIK
from openarm_app.kinematics.openarm_ik import pose_error
from openarm_piano_origin import POINTS, PianoFingerProjection, load_origin


def test_path_uses_the_next_key_to_resolve_redundancy():
    def layer(angles):
        q = np.zeros((len(angles), 7))
        q[:, 4] = np.deg2rad(angles)
        return q

    limits = np.tile([-np.pi, np.pi], (7, 1))
    layers = [layer([0]), layer([0, 4]), layer([8]), layer([12])]
    path = smooth_path(layers, np.zeros(7), limits)
    assert np.rad2deg(np.array(path)[:, 4]) == pytest.approx([0, 4, 8, 12])


def test_all_physical_white_keys_match_their_fixed_wrist_pose():
    cfg = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    with POINTS.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    points, anchors, _ = load_origin()
    finger_projection = PianoFingerProjection.load()
    assert finger_projection.keys_at("left", points["left"][10].xyz[1]) == {"4": 10}
    assert finger_projection.keys_at("right", points["right"][42].xyz[1]) == {
        "7": 42, "8": 43, "9": 44, "x": 45}
    assert [int(r["physical_white_key"]) for r in rows] == list(range(1, 37))
    assert [(anchors[s].note, anchors[s].physical_key) for s in ("left", "right")] == [("A3", 13), ("C4", 15)]
    assert points["left"][23].xyz == (250.19, 33.6, -170.00)
    assert {r["note"] for r in rows if r["status"] == "no_converged_ik"} == {"A6", "B6", "C7"}
    for side, anchor in cfg["anchors"].items():
        arm = OpenArmIK(side=side)
        part = [r for r in rows if r["arm"] == side]
        assert [int(r["physical_white_key"]) for r in part] == (list(range(1,15)) if side == "left" else list(range(15,37)))
        assert anchors[side].xyz == tuple(anchor["wrist_xyz_mm"])
        reference = anchor["reference_joints_deg"]
        quat = arm.forward(np.deg2rad(reference))[3:]
        assert finger_projection.keys_at(side, anchor["wrist_xyz_mm"][1]) == (
            {"1": 19, "2": 20, "3": 21, "4": 22} if side == "left" else {"7": 24, "8": 25, "9": 26, "x": 27})
        for row in part:
            key = int(row["physical_white_key"])
            assert float(row["key_distance_from_c2_mm"]) == pytest.approx((key - 1) * 22.5)
            offset = key - anchor["physical_white_key"]
            expected_direction = "inward" if side == "left" and offset > 0 else "outward"
            assert row["direction"] == (expected_direction if offset else "anchor")
            assert (row["suggested_for_single_index"] == "1") == (row["status"] != "no_converged_ik")
            xyz = np.array(anchor["wrist_xyz_mm"]) + [0., -22.5 * offset, 0.]
            target = np.array([float(row[c + "_mm"]) for c in "xyz"])
            assert target == pytest.approx(xyz)
            index = "4" if side == "left" else "7"
            assert finger_projection.keys_at(side, target[1])[index] == int(row["key"])
            assert [float(row[c]) for c in ("qw", "qx", "qy", "qz")] == pytest.approx(quat, abs=1e-7)
            score_key = int(row["key"])
            if row["status"] == "no_converged_ik":
                assert score_key not in points[side]
                assert all(row[f"J{i}_deg"] == "" for i in range(1, 8))
                continue
            point = points[side][score_key]
            q = np.deg2rad(point.deg)
            assert [float(row[f"q{i}"]) for i in range(1, 8)] == pytest.approx(q)
            pe, oe = pose_error(arm.forward(q), np.r_[target / 1000, quat])
            assert pe == pytest.approx(float(row["position_error_m"]), abs=2e-7)
            assert oe == pytest.approx(float(row["orientation_error_rad"]), abs=5e-7)
            assert pe < 1e-5 and oe < 1e-4
            if offset:
                near = points[side].get(score_key - int(np.sign(offset)))
                if near is not None:
                    assert float(row["step_max_deg"]) == pytest.approx(max(abs(np.array(point.deg) - near.deg)))
            else:
                assert row["status"] == "offline_ik"
                assert row["step_max_deg"] == ""
                assert math.dist([float(row["fk_" + c + "_mm"]) for c in "xyz"], target) < .01


def test_uncoverable_group_is_skipped_not_refused():
    from openarm_piano_origin import OFFSETS
    from openarm_piano_performance import Passage, _select_wrists

    points, _, _ = load_origin()
    group = Passage(0, 1_000_000, ({"key": 1, "side": "left"},))
    kept, skipped = _select_wrists([group], points["left"], OFFSETS["left"], min(points["left"]), {})
    assert kept == [] and skipped == [group]


def test_passage_does_not_change_wrist_during_held_notes():
    from openarm_piano_origin import OFFSETS
    from openarm_piano_performance import _passages, _select_wrists

    points, anchors, _ = load_origin()
    notes = [dict(key=20, side="left", on_us=0, off_us=1_000_000),
             dict(key=22, side="left", on_us=250_000, off_us=1_250_000)]
    passages = _passages(notes, "left", 0)
    assert len(passages) == 1 and passages[0].end == 1_250_000
    table = {22: points["left"][22]}
    kept, skipped = _select_wrists(passages, table, OFFSETS["left"], anchors["left"].key, {(22, 22): 0})
    assert kept == [(passages[0], 22)] and not skipped


def test_dual_trial_without_playable_notes_holds_origin(monkeypatch, tmp_path):
    from openarm_joint_mapping import load_mapping
    from openarm_piano_performance import SCORES
    from openarm_piano_trial import compile_trial

    path = tmp_path / "outside-keyboard.txt"
    path.write_text("left_arm_move(4,4)\nsleep(4)\nhand(D,4)\nsleep(.1)\nhand(U,4)\n", encoding="utf-8")
    monkeypatch.setitem(SCORES, "outside-keyboard", path)
    _, mappings = load_mapping(CALIBRATION.with_name("openarm_joint_mapping.json"), hardware=True)
    steps, info, score, paths = compile_trial(dict(score_id="outside-keyboard", scope="first"), None, mappings)
    assert info["note_count"] == 0 and info["skipped_notes"] == 1
    assert info["skipped_details"][0]["reason"] == "outside_physical_keyboard"
    assert score.note_count == 0 and not score.sides
    _, anchors, _ = load_origin()
    assert steps[1].left_deg == anchors["left"].deg and steps[1].right_deg == anchors["right"].deg
    assert all(q == anchors[side].deg for side, path in paths.items() for _, q in path.points)


def test_high_unreachable_index_keys_are_covered_by_other_fingers():
    from openarm_joint_mapping import load_mapping
    from openarm_piano_performance import fingering_preview

    points, _, _ = load_origin()
    _, mappings = load_mapping(CALIBRATION.with_name("openarm_joint_mapping.json"), hardware=True)
    result = fingering_preview("full", "不再犹豫", mappings)
    alternatives = {(row["wrists"]["right"], note["key"], note["finger"])
                    for row in result["rows"] for note in row["fingers"]["right"] if note["key"] >= 43}
    assert {key for _, key, _ in alternatives} == {43, 44, 45}
    assert all(wrist in points["right"] and finger in ("middle", "ring", "pinky")
               for wrist, _, finger in alternatives)
    assert all(key not in points["right"] for key in (43, 44, 45))
