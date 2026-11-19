"""Offline 14-joint positive-direction audit; no hardware modules imported."""
from pathlib import Path
import hashlib
import json
import math
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'openarm/src'))
from openarm_app.kinematics.model_adapter import load_v1


def main():
    setup, _, scene = load_v1()
    model = setup.model
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    mapping_path = ROOT / 'configs/openarm_joint_mapping.json'
    mappings = json.loads(mapping_path.read_text(encoding='utf-8'))
    mapping_rows = {r['joint_name']: r for r in mappings['joints']}
    expected = {
        'left': [(0, 1, 0), (-1, 0, 0), (0, 0, -1), (0, -1, 0),
                 (0, 0, -1), (1, 0, 0), (0, 1, 0)],
        'right': [(0, -1, 0), (-1, 0, 0), (0, 0, -1), (0, -1, 0),
                  (0, 0, -1), (1, 0, 0), (0, -1, 0)],
    }
    motions = {
        'left': ['整臂向后', '整臂向里（向机器人右侧）', '上臂轴转：俯视顺时针',
                 '前臂向前屈肘', '前臂轴转：俯视顺时针', '末端向外（向机器人左侧）', '末端向后'],
        'right': ['整臂向前', '整臂向外（向机器人右侧）', '上臂轴转：俯视顺时针',
                  '前臂向前屈肘', '前臂轴转：俯视顺时针', '末端向里（向机器人左侧）', '末端向前'],
    }
    rows = []
    for side in ('left', 'right'):
        hand_id = model.body(f'openarm_{side}_hand').id
        for number in range(1, 8):
            name = f'openarm_{side}_joint{number}'
            joint = model.joint(name)
            qi = int(joint.qposadr[0])
            body_id = int(model.jnt_bodyid[joint.id])
            assert abs(data.qpos[qi]) < 1e-12
            axis = data.xaxis[joint.id].copy()
            anchor = data.xanchor[joint.id].copy()
            hand0 = data.xpos[hand_id].copy()
            rotation0 = data.xmat[body_id].reshape(3, 3).copy()
            assert np.allclose(axis, expected[side][number-1], atol=1e-5)
            mapped = mapping_rows[name]['hardware_mapping']
            assert mapped == dict(direction=1, scale=1., p_ref=0., q_ref=0.)
            probe = 1e-6
            data.qpos[qi] = probe
            mujoco.mj_forward(model, data)
            numeric = (data.xpos[hand_id] - hand0) / probe
            analytic = np.cross(axis, hand0 - anchor)
            derivative_error = float(np.linalg.norm(numeric - analytic))
            assert derivative_error < 1e-6
            delta = math.radians(1)
            assert model.jnt_range[joint.id, 0] <= delta <= model.jnt_range[joint.id, 1]
            data.qpos[qi] = delta
            mujoco.mj_forward(model, data)
            rotvec = Rotation.from_matrix(data.xmat[body_id].reshape(3, 3) @ rotation0.T).as_rotvec()
            rotation_error = float(np.linalg.norm(rotvec - axis * delta))
            assert rotation_error < 1e-10
            rows.append(dict(side=side, motor_id=number, name=name,
                             local_axis=model.jnt_axis[joint.id].tolist(),
                             zero_world_axis=axis.tolist(),
                             hardware_mapping=mapped,
                             zero_motion=motions[side][number-1],
                             positive_probe_deg=1,
                             hand_origin_delta_mm=((data.xpos[hand_id]-hand0)*1000).tolist(),
                             body_rotation_vector_rad=rotvec.tolist(),
                             analytic_velocity_m_per_rad=analytic.tolist(),
                             finite_difference_error=derivative_error,
                             rotation_axis_error_rad=rotation_error))
            data.qpos[qi] = 0
            mujoco.mj_forward(model, data)
    assert len(rows) == 14
    paths = [scene, scene.parent/'openarm_bimanual.xml', mapping_path,
             ROOT/'apps/openarm_joint_mapping.py', ROOT/'apps/drive_openarm_from_sim.py',
             ROOT/'apps/openarm_execution.py', ROOT/'apps/openarm_driver.py']
    result = dict(date='2026-11-16', basis='All 14 arm angles zero; one joint at a time',
                  frame='+X robot forward, +Y robot left, +Z up',
                  clockwise_view='For J3/J5: above the zero-pose arm looking down toward the hand',
                  hardware_access=False, joints_checked=len(rows), rows=rows,
                  sources=[dict(path=str(p.relative_to(ROOT)), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                           for p in paths])
    (OUT/'audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(joints_checked=14, hardware_access=False,
                         max_derivative_error=max(r['finite_difference_error'] for r in rows),
                         max_rotation_error_rad=max(r['rotation_axis_error_rad'] for r in rows)), indent=2))


if __name__ == '__main__':
    main()
