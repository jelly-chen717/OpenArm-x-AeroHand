"""Offline regressions for the physical white-key table and its sequence cost."""
import csv
import json

import numpy as np
import pytest

from build_openarm_piano_points import CALIBRATION, smooth_path
from openarm_app.kinematics import OpenArmIK
from openarm_app.kinematics.openarm_ik import pose_error
from openarm_piano_origin import POINTS, load_origin


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
    assert [int(r["physical_white_key"]) for r in rows] == list(range(1, 37))
    assert [(anchors[s].note, anchors[s].physical_key) for s in ("left", "right")] == [("B3",14),("C4",15)]
    for side, anchor in cfg["anchors"].items():
        arm = OpenArmIK(side=side)
        part = [r for r in rows if r["arm"] == side]
        assert [int(r["physical_white_key"]) for r in part] == (list(range(1,15)) if side == "left" else list(range(15,37)))
        assert anchors[side].xyz == tuple(anchor["wrist_xyz_mm"])
        assert anchors[side].deg == tuple(anchor["joints_deg"])
        quat = arm.forward(np.deg2rad(anchor["joints_deg"]))[3:]
        for row in part:
            key = int(row["physical_white_key"])
            assert float(row["key_distance_from_c2_mm"]) == pytest.approx((key - 1) * 22.5)
            offset = key - anchor["physical_white_key"]
            assert row["direction"] == ("outward" if offset else "anchor")
            assert (row["suggested_for_single_index"] == "1") == (row["status"] != "no_converged_ik")
            xyz = np.array(anchor["wrist_xyz_mm"]) + [0., -22.5 * offset, 0.]
            target = np.array([float(row[c + "_mm"]) for c in "xyz"])
            assert target == pytest.approx(xyz)
            assert [float(row[c]) for c in ("qw", "qx", "qy", "qz")] == pytest.approx(quat)
            score_key = int(row["key"])
            if row["status"] == "no_converged_ik":
                assert score_key not in points[side]
                assert all(row[f"J{i}_deg"] == "" for i in range(1, 8))
                continue
            point = points[side][score_key]
            q = np.deg2rad(point.deg)
            assert [float(row[f"q{i}"]) for i in range(1, 8)] == pytest.approx(q)
            pe, oe = pose_error(arm.forward(q), np.r_[target / 1000, quat])
            # The physical anchor preserves the two separately rounded screen readings.
            assert pe == pytest.approx(float(row["position_error_m"]), abs=2e-7)
            assert oe == pytest.approx(float(row["orientation_error_rad"]), abs=5e-7)
            if offset:
                near = points[side].get(score_key - int(np.sign(offset)))
                if near is not None:
                    assert float(row["step_max_deg"]) == pytest.approx(max(abs(np.array(point.deg) - near.deg)))
            else:
                assert row["step_max_deg"] == ""
