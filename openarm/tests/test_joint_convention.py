import json
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from openarm_app.joint_convention import CONVENTION_VERSION, JOINT_SIGNS, SIGNS
from openarm_app.kinematics.model_adapter import load_v1


@pytest.fixture(scope='module')
def models():
    return load_v1(unified=False)[0].model, load_v1()[0].model


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('number', range(1, 8))
def test_each_positive_joint_has_requested_direction(models, side, number):
    _, model = models
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    joint = model.joint(f'openarm_{side}_joint{number}')
    body = int(model.jnt_bodyid[joint.id])
    hand = model.body(f'openarm_{side}_hand').id
    before = data.xpos[hand].copy()
    rotation = data.xmat[body].reshape(3, 3).copy()
    data.qpos[joint.qposadr[0]] = np.deg2rad(1)
    mujoco.mj_forward(model, data)
    displacement = data.xpos[hand] - before
    rotvec = Rotation.from_matrix(data.xmat[body].reshape(3, 3) @ rotation.T).as_rotvec()
    if number in (1, 4, 7):
        assert displacement[0] > 0
    elif number in (2, 6):
        assert displacement[1] * (-1 if side == 'left' else 1) > 0
    else:
        assert rotvec[2] < 0  # Clockwise viewed from above in the zero pose.


def test_reparameterization_preserves_every_link_pose_and_physical_limits(models):
    original, logical = models
    a, b = mujoco.MjData(original), mujoco.MjData(logical)
    rng = np.random.default_rng(20261116)
    for _ in range(12):
        for name, sign in JOINT_SIGNS.items():
            old, new = original.joint(name), logical.joint(name)
            position = rng.uniform(*old.range)
            a.qpos[old.qposadr[0]] = position
            b.qpos[new.qposadr[0]] = sign * position
            np.testing.assert_allclose(new.range, np.sort(sign*old.range), atol=1e-12)
            np.testing.assert_allclose(new.axis, sign*old.axis, atol=1e-12)
        mujoco.mj_forward(original, a)
        mujoco.mj_forward(logical, b)
        np.testing.assert_allclose(a.xpos, b.xpos, atol=1e-12)
        np.testing.assert_allclose(a.xmat, b.xmat, atol=1e-12)
    for name, sign in JOINT_SIGNS.items():
        old, new = original.joint(name), logical.joint(name)
        np.testing.assert_allclose(logical.key_qpos[:, new.qposadr[0]],
                                   sign*original.key_qpos[:, old.qposadr[0]])
        actuator_name = name.replace('openarm_', '') + '_ctrl'
        old_a, new_a = original.actuator(actuator_name), logical.actuator(actuator_name)
        np.testing.assert_allclose(new_a.ctrlrange, np.sort(sign*old_a.ctrlrange))
        np.testing.assert_allclose(logical.key_ctrl[:, new_a.id], sign*original.key_ctrl[:, old_a.id])


def test_piano_and_ik_share_axes_and_migrated_points_preserve_targets(models):
    from paino_skill.scene import build_scene
    from paino_skill.points import load_table
    original, logical = models
    piano, _, _ = build_scene()
    for name in JOINT_SIGNS:
        np.testing.assert_allclose(piano.joint(name).axis, logical.joint(name).axis)
        np.testing.assert_allclose(piano.joint(name).range, logical.joint(name).range)
    table = load_table()
    assert table['coordinate_convention'] == CONVENTION_VERSION
    a, b = mujoco.MjData(original), mujoco.MjData(logical)
    for row in table['rows']:
        if row['status'] != 'ready':
            continue
        for i, (q, sign) in enumerate(zip(row['q'], SIGNS[row['arm']]), 1):
            name = f"openarm_{row['arm']}_joint{i}"
            a.qpos[original.joint(name).qposadr[0]] = sign*q
            b.qpos[logical.joint(name).qposadr[0]] = q
        mujoco.mj_forward(original, a)
        mujoco.mj_forward(logical, b)
        np.testing.assert_allclose(a.xpos, b.xpos, atol=1e-12)
        np.testing.assert_allclose(a.xmat, b.xmat, atol=1e-12)


def test_mapping_and_model_use_same_convention_and_limits(models):
    _, model = models
    config = json.loads((Path(__file__).resolve().parents[2]/'configs/openarm_joint_mapping.json').read_text(encoding='utf-8'))
    assert config['coordinate_convention'] == CONVENTION_VERSION
    for row in config['joints']:
        assert row['hardware_mapping']['direction'] == JOINT_SIGNS[row['joint_name']]
        np.testing.assert_allclose(row['limits_rad'], model.joint(row['joint_name']).range)
