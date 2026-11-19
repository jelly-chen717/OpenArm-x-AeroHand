"""Offline, reproducible evidence for the logical-coordinate migration."""
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'openarm/src'))
from openarm_app.joint_convention import CONVENTION_VERSION, JOINT_SIGNS
from openarm_app.kinematics.model_adapter import load_v1

OUT = Path(__file__).resolve().parent
original = load_v1(unified=False)[0].model
logical = load_v1()[0].model
mapping = json.loads((ROOT/'configs/openarm_joint_mapping.json').read_text(encoding='utf-8'))
rows = []
for row in mapping['joints']:
    side, number, name = row['side'], row['motor_id'], row['joint_name']
    old, joint = original.joint(name), logical.joint(name)
    sign = JOINT_SIGNS[name]
    np.testing.assert_allclose(joint.range, np.sort(sign * old.range), atol=1e-12)
    np.testing.assert_allclose(row['limits_rad'], joint.range, atol=1e-12)
    assert row['hardware_mapping']['direction'] == sign
    data = mujoco.MjData(logical)
    mujoco.mj_forward(logical, data)
    body, hand = int(logical.jnt_bodyid[joint.id]), logical.body(f'openarm_{side}_hand').id
    axis = data.xaxis[joint.id].copy()
    before, rotation = data.xpos[hand].copy(), data.xmat[body].reshape(3, 3).copy()
    data.qpos[joint.qposadr[0]] = np.deg2rad(1)
    mujoco.mj_forward(logical, data)
    delta = data.xpos[hand] - before
    rotvec = Rotation.from_matrix(data.xmat[body].reshape(3, 3) @ rotation.T).as_rotvec()
    if number in (1, 4, 7):
        passed, direction = delta[0] > 0, 'forward'
    elif number in (2, 6):
        passed, direction = delta[1] * (-1 if side == 'left' else 1) > 0, 'inward'
    else:
        passed, direction = rotvec[2] < 0, 'clockwise viewed from above'
    assert passed, name
    rows.append(dict(joint=name, official_to_logical_sign=sign,
                     positive_direction=direction, limits_deg=np.rad2deg(joint.range).tolist(),
                     world_axis_at_zero=axis.tolist(), hand_delta_m_for_plus_1deg=delta.tolist(),
                     body_rotation_vector_rad=rotvec.tolist(), passed=bool(passed)))

rng = np.random.default_rng(20261116)
a, b = mujoco.MjData(original), mujoco.MjData(logical)
max_position_error = max_rotation_error = 0.0
for _ in range(12):
    for name, sign in JOINT_SIGNS.items():
        old, new = original.joint(name), logical.joint(name)
        value = rng.uniform(*old.range)
        a.qpos[old.qposadr[0]], b.qpos[new.qposadr[0]] = value, sign * value
    mujoco.mj_forward(original, a)
    mujoco.mj_forward(logical, b)
    max_position_error = max(max_position_error, float(np.max(np.abs(a.xpos-b.xpos))))
    max_rotation_error = max(max_rotation_error, float(np.max(np.abs(a.xmat-b.xmat))))
assert max(max_position_error, max_rotation_error) < 1e-12

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

migration = json.loads((OUT/'migration.json').read_text(encoding='utf-8'))
for entry in migration['files']:
    assert sha(ROOT/entry['file']) == entry['after_sha256'], entry['file']
    assert sha(OUT/'before'/entry['file']) == entry['before_sha256'], entry['file']
vendor = ROOT/'openarm/vendor/openarm_mujoco/v1/openarm_bimanual.xml'
snapshot = ROOT/'logs/openarm/limit_audit_20261116/mujoco_v1.xml'
result = dict(date='2026-11-16', convention=CONVENTION_VERSION,
              mapping_version=mapping['version'], hardware_accessed=False,
              hardware_motion_tested=False, joints=rows,
              random_full_pose_samples=12, max_link_position_error_m=max_position_error,
              max_link_rotation_matrix_error=max_rotation_error,
              migrated_files_and_backups_sha256_verified=len(migration['files']),
              vendor_sha256=sha(vendor), downloaded_official_sha256=sha(snapshot),
              vendor_matches_downloaded_official=sha(vendor) == sha(snapshot),
              vendor_text_matches_after_newline_normalization=(
                  vendor.read_text(encoding='utf-8') == snapshot.read_text(encoding='utf-8')))
(OUT/'validation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k != 'joints'}, ensure_ascii=False))
print('14/14 positive directions passed')
