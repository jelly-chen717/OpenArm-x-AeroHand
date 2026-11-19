"""Offline V1 coordinate audit. Never creates a CAN bus or sends hardware commands.

Reads the rc1 Xacro/YAML arm definitions and compares their rigid transforms
with the compiled official MuJoCo scene. This is a source-level arm-chain
audit, not a replacement Xacro processor or a hardware calibration tool.
"""
from pathlib import Path
import ctypes
import hashlib
import importlib.metadata
import json
import math
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'openarm/src'))
import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from openarm_app.kinematics.model_adapter import load_v1
import openarm_driver as driver

X = '{http://www.ros.org/wiki/xacro}'
DESC = ROOT / 'resources/teleop_rc1_review/sources/runtime/ros2_ws/install/openarm_description/share/openarm_description'
V1 = DESC / 'assets/robot/openarm_v1.0'
OUT = ROOT / 'logs/openarm/sync/offline_model_audit_20261115'


def transform(xyz, rotation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = xyz
    return t


def rotation_error(a, b):
    return float(Rotation.from_matrix(a.T @ b).magnitude())


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = [V1 / 'config/arm' / n for n in
             ('kinematics.yaml', 'kinematics_offset.yaml', 'joint_limits.yaml')]
    kin, offsets, limits = [yaml.safe_load(p.read_text(encoding='utf-8')) for p in files]
    arm_file = V1 / 'urdf/arm/openarm_arm.xacro'
    macro_file = V1 / 'urdf/arm/openarm_macro.xacro'
    top_file = V1 / 'urdf/openarm_v10.urdf.xacro'
    arm = ET.parse(arm_file).getroot()
    joints = arm.findall('.//joint[@type="revolute"]')
    assert len(joints) == 7
    # Explicitly pin the source equations used below; no evaluation of arbitrary Xacro.
    macro_text = macro_file.read_text(encoding='utf-8')
    for text in ('${limits.lower * reflect + offset}',
                 '${reflect*(kinematics.roll + offset.roll)}',
                 '${kinematics.x + offset.x}'):
        assert text in macro_text, 'Source macro changed: ' + text
    arm_text = arm_file.read_text(encoding='utf-8')
    for text in ("${-2.094396 if arm_prefix=='left_' else 0}", '${pi/2}', '${-pi/2}'):
        assert text in arm_text, 'Source arm changed: ' + text
    top = ET.parse(top_file).getroot()
    args = {n.attrib['name']: n.attrib['default'] for n in top.findall(X+'arg')}
    setup, _, scene = load_v1(side='right')
    left_setup, _, _ = load_v1(side='left', model=setup.model)
    model, data = setup.model, setup.data
    rows, chain_results, mounts = [], [], []
    all_equal = True
    for side in ('left', 'right'):
        reflect = 1 if side == 'right' else -1
        source_chain, ids = [], []
        for i, node in enumerate(joints, 1):
            assert node.attrib['name'] == '${prefix}joint'+str(i)
            name = f'openarm_{side}_joint{i}'
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            assert jid >= 0 and model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE
            ids.append(jid)
            bid = int(model.jnt_bodyid[jid])
            assert model.body(bid).name == f'openarm_{side}_link{i}'
            assert model.body(int(model.body_parentid[bid])).name == f'openarm_{side}_link{i-1}'
            call = node.find(X+'openarm-kinematics')
            assert call.attrib['name'] == f'joint{i}'
            reflected = reflect if call.attrib.get('reflect') == '${reflect}' else 1
            k = kin[f'joint{i}']['kinematic']
            o = offsets[f'joint{i}']['kinematic_offset'] if 'offset' in call.attrib else {}
            xyz = [k[a]+o.get(a, 0) for a in ('x','y','z')]
            rpy = [reflected*(k[a]+o.get(a, 0)) for a in ('roll','pitch','yaw')]
            rot = Rotation.from_euler('xyz', rpy).as_matrix()
            axis = [float(reflect) if a == '${reflect}' else float(a)
                    for a in node.find('axis').attrib['xyz'].split()]
            source_chain.append((transform(xyz, rot), np.array(axis)))
            lc = node.find(X+'openarm-limits')
            lr = reflect if lc.attrib.get('reflect') == '${reflect}' else 1
            lo = (-2.094396 if side == 'left' else 0) if i == 1 else (
                reflect*math.pi/2 if i == 2 else 0)
            lim = limits[f'joint{i}']['limit']
            source_range = sorted([lr*lim['lower']+lo, lr*lim['upper']+lo])
            mrot = Rotation.from_quat(model.body_quat[bid][[1,2,3,0]]).as_matrix()
            checks = {
                'axis_equal': bool(np.allclose(axis, model.jnt_axis[jid], atol=1e-12)),
                'origin_position_equal': bool(np.allclose(xyz, model.body_pos[bid], atol=1e-10)),
                'origin_rotation_error_rad': rotation_error(rot, mrot),
                'range_equal': bool(np.allclose(source_range, model.jnt_range[jid], atol=1e-10)),
                'joint_anchor_zero': bool(np.linalg.norm(model.jnt_pos[jid]) < 1e-12),
                'qpos_reference_zero': bool(abs(model.qpos0[model.jnt_qposadr[jid]]) < 1e-12),
            }
            equal = all(v for k,v in checks.items() if isinstance(v, bool)) and checks['origin_rotation_error_rad'] < 1e-10
            all_equal &= equal
            rows.append(dict(side=side,joint=name,motor_id=i,feedback_id=hex(0x10+i),
                motor_type=driver.MOTOR_TYPES[i-1],pcan_candidate=hex(driver.DEFAULT_CHANNELS[side]),
                pcan_physical_mapping_verified=False,qpos_index=int(model.jnt_qposadr[jid]),
                dof_index=int(model.jnt_dofadr[jid]),axis=axis,source_range=source_range,
                mujoco_range=model.jnt_range[jid].tolist(),checks=checks,
                model_mapping_equal=equal,source_to_mujoco_candidate={'sign':1,'scale':1,'offset_rad':0} if equal else None,
                hardware_calibration_verified=False))
        # Compare every link at 33 reproducible poses, including zero and joint-wise perturbations.
        ranges = model.jnt_range[ids]
        middle = ranges.mean(axis=1)
        poses = [np.zeros(7), middle]
        for i in range(7):
            for sign in (-1,1):
                q=middle.copy();q[i]+=sign*.05;poses.append(q)
        rng = np.random.default_rng(20261115)
        poses.extend(rng.uniform(ranges[:,0],ranges[:,1],size=(17,7)))
        max_pos = max_rot = 0.
        root_bid = model.body(f'openarm_{side}_link0').id
        for q in poses:
            data.qpos[model.jnt_qposadr[ids]] = q
            mujoco.mj_forward(model,data)
            inv_base=np.linalg.inv(transform(data.xpos[root_bid],data.xmat[root_bid].reshape(3,3)))
            t=np.eye(4)
            for i, ((origin,axis),value) in enumerate(zip(source_chain,q),1):
                t=t@origin@transform([0,0,0],Rotation.from_rotvec(axis*value).as_matrix())
                b=model.body(f'openarm_{side}_link{i}').id
                measured=inv_base@transform(data.xpos[b],data.xmat[b].reshape(3,3))
                max_pos=max(max_pos,float(np.linalg.norm(t[:3,3]-measured[:3,3])))
                max_rot=max(max_rot,rotation_error(t[:3,:3],measured[:3,:3]))
        chain_results.append(dict(side=side,poses=len(poses),links_per_pose=7,
            max_position_error_m=max_pos,max_rotation_error_rad=max_rot))
        source_xyz=np.array([float(v) for v in args[side+'_arm_base_xyz'].split()])
        source_rot=Rotation.from_euler('xyz',[float(v) for v in args[side+'_arm_base_rpy'].split()]).as_matrix()
        b=model.body(root_bid)
        mounts.append(dict(side=side,source_base_xyz=source_xyz.tolist(),mujoco_base_xyz=b.pos.tolist(),
            delta_xyz_m=(b.pos-source_xyz).tolist(),rotation_error_rad=rotation_error(source_rot,
                Rotation.from_quat(b.quat[[1,2,3,0]]).as_matrix())))
    # DLL load/symbol resolution only: do not initialize or enumerate channels.
    dll=ctypes.WinDLL('PCANBasic.dll')
    for symbol in ('CAN_InitializeFD','CAN_Uninitialize','CAN_GetStatus','CAN_SetValue',
                   'CAN_ReadFD','CAN_WriteFD','CAN_GetErrorText'):
        getattr(dll,symbol)
    path_buffer=ctypes.create_unicode_buffer(32768)
    ctypes.windll.kernel32.GetModuleFileNameW(ctypes.c_void_p(dll._handle),path_buffer,len(path_buffer))
    source_files=files+[arm_file,macro_file,top_file,scene,scene.parent/'openarm_bimanual.xml',
                       ROOT/'apps/openarm_driver.py']
    result=dict(python=sys.executable,python_version=sys.version,
        versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','openarm-control')},
        scene=str(scene),dll_path=path_buffer.value,dll_symbols_resolved=True,
        hardware_bus_opened=False,xacro_installed=False,
        method='Explicit source arm macro equations, not full Xacro expansion; local link0 frames',
        all_14_model_coordinates_equal=bool(all_equal),rows=rows,chain_checks=chain_results,
        base_mount_comparison=mounts,
        sources=[dict(path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in source_files])
    assert all_equal, 'Model coordinate mismatch; inspect report'
    assert all(c['max_position_error_m']<1e-9 and c['max_rotation_error_rad']<1e-9 for c in chain_results)
    assert len({r['qpos_index'] for r in rows})==14
    assert len({r['dof_index'] for r in rows})==14
    output=OUT/'audit.json'
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('python','versions','dll_path','all_14_model_coordinates_equal',
        'chain_checks','base_mount_comparison','hardware_bus_opened')},ensure_ascii=False,indent=2))
    print('Evidence:',output)


if __name__=='__main__':
    main()
