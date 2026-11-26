"""Build the physical piano wrist table offline with the official V1 IK."""
import argparse
import csv
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np

from openarm_app.joint_convention import CONVENTION_VERSION
from openarm_app.kinematics import OpenArmIK
from openarm_app.kinematics.openarm_ik import IKConvergenceError, pose_error


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = ROOT / "configs/openarm_piano_white_keys.json"
NOTE_NAMES = "CDEFGAB"
SEARCH_FRACTIONS = np.random.default_rng(23).uniform(.05, .95, (20, 7))
STEP_UNIT = np.deg2rad(10.)
POSTURE_UNIT = np.deg2rad(45.)
MARGIN_UNIT = np.deg2rad(12.)


def note_name(first, key):
    octave, degree = divmod(NOTE_NAMES.index(first[0]) + key - 1, 7)
    return f"{NOTE_NAMES[degree]}{int(first[1:]) + octave}"


class WristModel:
    """Double-precision verification, independent of the IK package's FK."""

    def __init__(self, arm):
        self.model = arm.setup.model
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site(f"{arm.side}_ee_control_point").id
        self.base = self.model.site("arm_origin").id
        joints = [self.model.joint(f"openarm_{arm.side}_joint{i}") for i in range(1, 8)]
        self.qpos = [int(joint.qposadr[0]) for joint in joints]
        self.dofs = [int(joint.dofadr[0]) for joint in joints]

    def _update(self, q):
        self.data.qpos[self.qpos] = q
        mujoco.mj_kinematics(self.model, self.data)

    def forward(self, q):
        self._update(q)
        rotation = self.data.site_xmat[self.base].reshape(3, 3).T
        xyz = rotation @ (self.data.site_xpos[self.site] - self.data.site_xpos[self.base])
        matrix = rotation @ self.data.site_xmat[self.site].reshape(3, 3)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, matrix.ravel())
        return np.r_[xyz, quat]

    def redundant_direction(self, q):
        self._update(q)
        mujoco.mj_comPos(self.model, self.data)
        jacobian = np.zeros((6, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacobian[:3], jacobian[3:], self.site)
        _, _, right = np.linalg.svd(jacobian[:, self.dofs])
        # Changing q in this direction preserves the pose to first order.
        return right[-1] / np.max(np.abs(right[-1]))


def solve_from(arm, target, seed):
    try:
        return arm.inverse(target, seed, max_iterations=85,
                           position_tolerance=3e-6, orientation_tolerance=3e-5)
    except IKConvergenceError:
        return None


def explore_redundancy(arm, wrist, target, start):
    lower, upper = arm.limits.T
    for sign in (-1, 1):
        last = start
        direction = sign * wrist.redundant_direction(start)
        for _ in range(60):
            tangent = wrist.redundant_direction(last)
            if tangent @ direction < 0:
                tangent = -tangent
            seed = np.clip(last + np.deg2rad(2.5) * tangent, lower, upper)
            q = solve_from(arm, target, seed)
            if q is None or (q - last) @ tangent < np.deg2rad(.35):
                break
            yield q
            last, direction = q, tangent


def posture_cost(q, anchor, limits):
    margin = np.minimum(q - limits[:, 0], limits[:, 1] - q)
    displacement = (q - anchor) / POSTURE_UNIT
    proximity = np.maximum(0., 1. - margin / MARGIN_UNIT)
    return .12 * np.sum(displacement**2, axis=-1) + .65 * np.sum(proximity**2, axis=-1)


def candidates_for_key(arm, wrist, target, anchor, last, prediction):
    lower, upper = arm.limits.T
    swivel = np.array([0, 0, 1, 0, -1, 0, 0])
    seeds = [last, prediction, anchor, arm.home, (lower + upper) / 2]
    seeds.extend(last + swivel * a for a in np.deg2rad([-55., -25., 25., 55.]))
    seeds.extend(lower + SEARCH_FRACTIONS * (upper - lower))
    candidates = []

    def add(q):
        if q is not None and not any(np.max(np.abs(q - other)) < np.deg2rad(.35)
                                     for other in candidates):
            candidates.append(q)

    unique = dict.fromkeys(tuple(np.clip(seed, lower, upper)) for seed in seeds)
    for seed in unique:
        add(solve_from(arm, target, seed))
    if candidates:
        closest = min(candidates, key=lambda q: np.linalg.norm(q - last))
        for q in explore_redundancy(arm, wrist, target, closest):
            add(q)
    return np.asarray(candidates).reshape(-1, 7)


def step_cost(delta):
    return np.sum(delta**2, axis=-1) + 1.5 * np.max(np.abs(delta), axis=-1)**2


def smooth_path(layers, anchor, limits):
    """Minimum of the stated cost over all candidates, including second differences."""
    nodes = [posture_cost(layer, anchor, limits) for layer in layers]
    if len(layers) == 1:
        return [layers[0][np.argmin(nodes[0])]]
    delta = (layers[1][None, :, :] - layers[0][:, None, :]) / STEP_UNIT
    costs = nodes[0][:, None] + nodes[1][None, :] + step_cost(delta)
    parents = []
    for i in range(2, len(layers)):
        next_delta = (layers[i][None, :, :] - layers[i-1][:, None, :]) / STEP_UNIT
        next_costs = step_cost(next_delta) + nodes[i][None, :]
        back = np.empty(next_costs.shape, dtype=int)
        for j, d in enumerate(next_delta.transpose(1, 0, 2)):
            totals = costs + 2.5 * np.sum((d[None, :, :] - delta)**2, axis=-1)
            back[:, j] = np.argmin(totals, axis=0)
            next_costs[:, j] += np.min(totals, axis=0)
        parents.append(back)
        costs, delta = next_costs, next_delta
    a, b = np.unravel_index(np.argmin(costs), costs.shape)
    indices = [a, b]
    for back in reversed(parents):
        indices.insert(0, back[indices[0], indices[1]])
    return [layer[i] for layer, i in zip(layers, indices, strict=True)]


def solve_side(config, side):
    arm = OpenArmIK(side=side)
    wrist = WristModel(arm)
    data = config["anchors"][side]
    base = data["physical_white_key"]
    count = config["white_key_count"]
    first = config["white_key_first"]
    if note_name(first, base) != data["note"] or note_name(first, count) != config["white_key_last"]:
        raise ValueError("Inconsistent physical key numbering")
    measured_angles = data.get("joints_deg")
    reference_angles = measured_angles if measured_angles is not None else data["reference_joints_deg"]
    reference_q = np.deg2rad(reference_angles)
    xyz0 = np.array(data["wrist_xyz_mm"])
    rotation = wrist.forward(reference_q)[3:]
    pitch = config["higher_note_y_sign"] * config["white_key_pitch_mm"]
    split = config["right_first_physical_white_key"]
    owned = range(1, split) if side == "left" else range(split, count + 1)
    if base not in owned:
        raise ValueError(f"{side} anchor outside its assigned range")
    targets = {i: np.r_[(xyz0 + [0., pitch * (i - base), 0.]) / 1000, rotation]
               for i in owned}
    q0 = reference_q
    if measured_angles is None:
        q0 = solve_from(arm, targets[base], reference_q)
        if q0 is None:
            raise ValueError(f"{side} {data['note']}: fixed-orientation origin IK did not converge")
    layers = {base: q0[None, :]}
    selected = {base: q0}
    for travel in (range(base - 1, owned.start - 1, -1), range(base + 1, owned.stop)):
        previous = last = q0
        run = [base]
        for key in travel:
            layer = candidates_for_key(arm, wrist, targets[key], q0, last, 2 * last - previous)
            layers[key] = layer
            if len(layer):
                run.append(key)
                previous, last = last, min(layer, key=lambda q: np.linalg.norm(q - last))
            elif run:
                path = smooth_path([layers[i] for i in run], q0, arm.limits)
                selected.update(zip(run, path, strict=True))
                run = []
        if run:
            path = smooth_path([layers[i] for i in run], q0, arm.limits)
            selected.update(zip(run, path, strict=True))

    rows = []
    for key, target in targets.items():
        offset = key - base
        q = selected.get(key)
        score_offset = config["score_key_at_first"] - 1
        status = "no_converged_ik" if q is None else "offline_ik"
        if key == base and measured_angles is not None:
            status = "screenshot_feedback"
        direction = "anchor" if not offset else "outward" if (offset > 0) == (side == "right") else "inward"
        row = dict(arm=side, finger="index", key=key + score_offset, physical_white_key=key,
                   note=note_name(first, key), anchor_key=base + score_offset, anchor_run=3,
                   offset_steps=offset, delta_y_m=pitch * offset / 1000,
                   key_distance_from_c2_mm=(key - 1) * config["white_key_pitch_mm"],
                   status=status,
                   coordinate_convention=CONVENTION_VERSION, direction=direction,
                   candidate_count=len(layers[key]), suggested_for_single_index=int(q is not None))
        for j, c in enumerate("xyz"):
            row[c + "_m"] = target[j]
            row[c + "_mm"] = round(target[j] * 1000, 6)
        row.update(zip(("qw", "qx", "qy", "qz"), rotation, strict=True))
        if q is not None:
            actual = wrist.forward(q)
            pe, oe = pose_error(actual, target)
            if not np.all((q >= arm.limits[:, 0]) & (q <= arm.limits[:, 1])):
                raise ValueError(f"{side} {key}: solution outside model limits")
            if status == "offline_ik" and (pe > 1e-5 or oe > 1e-4):
                raise ValueError(f"{side} {key}: independent FK disagrees with target")
            row.update(position_error_m=pe, orientation_error_rad=oe,
                       limit_margin_deg=np.rad2deg(np.min(np.minimum(
                           q - arm.limits[:, 0], arm.limits[:, 1] - q))))
            angles = measured_angles if status == "screenshot_feedback" else np.rad2deg(q)
            for j, angle in enumerate(angles, 1):
                row[f"q{j}"], row[f"J{j}_deg"] = q[j-1], angle
            for j, c in enumerate("xyz"):
                row["fk_" + c + "_mm"] = 1000 * actual[j]
                row["fk_d" + c + "_mm"] = 1000 * (actual[j] - target[j])
            neighbour = selected.get(key - int(np.sign(offset))) if offset else None
            if neighbour is not None:
                row["step_max_deg"] = np.rad2deg(np.max(np.abs(q - neighbour)))
        rows.append(row)
    print(f"{side}: {len(selected)}/{len(targets)} key positions", flush=True)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Destination CSV, written after both sides finish")
    args = parser.parse_args()
    config = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    if config["frame"] != "arm_origin" or config["coordinate_convention"] != CONVENTION_VERSION:
        raise ValueError("Calibration frame or joint convention disagrees with the model")
    rows = [row for side in ("left", "right") for row in solve_side(config, side)]
    fields = ["arm", "finger", "key", "physical_white_key", "note", "anchor_key", "anchor_run",
              "offset_steps", "delta_y_m", "status", "position_error_m", "orientation_error_rad",
              *(f"q{i}" for i in range(1, 8)), "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
              *(f"J{i}_deg" for i in range(1, 8)), "x_mm", "y_mm", "z_mm", "coordinate_convention",
              "direction", "candidate_count", "step_max_deg", "limit_margin_deg",
              "fk_x_mm", "fk_y_mm", "fk_z_mm", "fk_dx_mm", "fk_dy_mm", "fk_dz_mm",
              "suggested_for_single_index", "key_distance_from_c2_mm"]
    output = tempfile.NamedTemporaryFile(dir=args.output.parent, delete=False, suffix=".tmp", mode="w",
                                         encoding="utf-8-sig", newline="")
    stage = Path(output.name)
    try:
        with output:
            writer = csv.DictWriter(output, fields)
            writer.writeheader()
            writer.writerows(rows)
        stage.replace(args.output)
    finally:
        stage.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
