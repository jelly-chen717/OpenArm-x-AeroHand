"""Offline V1 model-only torque/speed audit. Never imports any hardware driver."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'openarm/src'))
import mujoco
import numpy as np
from openarm_app.kinematics.model_adapter import load_v1

STEPS = [(7, -15., .8), (6, -30., 1.2), (5, 45., 1.5), (4, 3., .6),
         (3, -45., 1.5), (2, -45., 4.0), (1, -45., 1.8)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', type=Path, help='JSON object of logical radian starts, keyed left:1 etc')
    parser.add_argument('--snapshot', type=Path, help='Read-only status JSONL; convert raw positions with current hardware mapping')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('dynamics_estimate.json'))
    args = parser.parse_args()
    setup, _, path = load_v1()
    model, data = setup.model, setup.data
    keys = [(s, j) for s in ('left', 'right') for j in range(1, 8)]
    names = [f'openarm_{s}_joint{j}' for s, j in keys]
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    qidx = model.jnt_qposadr[joints]
    didx = model.jnt_dofadr[joints]
    initial = json.loads(args.start.read_text(encoding='utf-8')) if args.start else {}
    if args.snapshot:
        mapping = json.loads((ROOT/'configs/openarm_joint_mapping.json').read_text(encoding='utf-8'))
        lookup = {(j['side'],j['motor_id']):j['hardware_mapping'] for j in mapping['joints']}
        for line in args.snapshot.read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            for motor in row['motors']:
                key = (row['side'], motor['motor_id'])
                mp = lookup[key]
                initial[f'{key[0]}:{key[1]}'] = mp['q_ref'] + mp['direction']*mp['scale']*(motor['position_rad']-mp['p_ref'])
        if len(initial) != 14:
            raise ValueError('Need complete 14-axis start snapshot')
    start = np.array([initial.get(f'{s}:{j}', 0.) for s,j in keys])
    data.qpos[qidx] = start
    data.qvel[:] = 0
    data.qacc[:] = 0
    limits = np.array([20. if j < 3 else 9. if j < 5 else 3. for s,j in keys])
    max_all = np.zeros(14)
    max_hold = np.zeros(14)
    contacts = {}
    rows = []
    matrix = np.zeros((model.nv, model.nv))
    contact_samples = 0
    pose = start.copy()
    elapsed = 0.
    for joint, delta_deg, duration in STEPS:
        active = [i for i,k in enumerate(keys) if k[1] == joint]
        delta = np.zeros(14)
        delta[active] = math.radians(delta_deg)
        peak = np.zeros(14)
        minimum_margin = float('inf')
        world_axes = {}
        ee_motion = {}
        mj_inverse_max = np.zeros(14)
        for u in np.linspace(0, 1, 501):
            b = 10*u**3 - 15*u**4 + 6*u**5
            rate = 30*u*u*(1-u)**2/duration
            accel = (60*u-180*u*u+120*u**3)/duration**2
            data.qpos[qidx] = pose + delta*b
            data.qvel[:] = 0
            data.qvel[didx] = delta*rate
            data.qacc[:] = 0
            prescribed_acc = np.zeros(model.nv)
            prescribed_acc[didx] = delta*accel
            mujoco.mj_forward(model, data)
            if u == 0:
                world_axes = {keys[i][0]:data.xaxis[joints[i]].tolist() for i in active}
                for i in active:
                    side = keys[i][0]
                    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'openarm_{side}_hand_tcp')
                    jacp, jacr = np.zeros((3,model.nv)), np.zeros((3,model.nv))
                    mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
                    ee_motion[side] = (jacp[:,didx[i]]*math.copysign(math.pi/180,delta_deg)*1000).tolist()
            mujoco.mj_fullM(model, data, matrix)
            # Newton-Euler equation with declared model damping and Coulomb friction.
            # Contact constraint forces are excluded; detect contacts separately below.
            torque = (matrix @ prescribed_acc + data.qfrc_bias - data.qfrc_passive
                      + model.dof_frictionloss * np.sign(data.qvel))[didx]
            peak = np.maximum(peak, np.abs(torque))
            minimum_margin = min(minimum_margin, float(np.min(limits-np.abs(torque))))
            if data.ncon:
                contact_samples += 1
            for c in data.contact[:data.ncon]:
                pair = tuple(sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or str(g) for g in (c.geom1,c.geom2)))
                contacts[' | '.join(pair)] = min(float(c.dist), contacts.get(' | '.join(pair), float('inf')))
            data.qacc[:] = prescribed_acc
            mujoco.mj_inverse(model, data)
            mj_inverse_max = np.maximum(mj_inverse_max, np.abs(data.qfrc_inverse[didx]))
        pose += delta
        data.qpos[qidx] = pose
        data.qvel[:] = 0
        data.qacc[:] = 0
        mujoco.mj_forward(model, data)
        hold = data.qfrc_bias[didx].copy()
        max_all = np.maximum(max_all, peak)
        max_hold = np.maximum(max_hold, np.abs(hold))
        rows.append(dict(joint=joint, delta_deg=delta_deg, move_s=duration, arrival_hold_s=1.5,
                         nominal_move_start_s=elapsed, nominal_move_end_s=elapsed+duration,
                         peak_velocity_deg_s=1.875*abs(delta_deg)/duration,
                         peak_velocity_rad_s=1.875*abs(math.radians(delta_deg))/duration,
                         peak_acceleration_rad_s2=10/math.sqrt(3)*abs(math.radians(delta_deg))/duration**2,
                         active_axes_peak_torque_nm={keys[i][0]:float(peak[i]) for i in active},
                         all_axes_peak_torque_nm={f'{s}:{j}':float(v) for (s,j),v in zip(keys,peak)},
                         all_axes_end_hold_gravity_torque_nm={f'{s}:{j}':float(v) for (s,j),v in zip(keys,hold)},
                         raw_mj_inverse_peak_torque_nm={f'{s}:{j}':float(v) for (s,j),v in zip(keys,mj_inverse_max)},
                         all_axes_min_margin_nm=minimum_margin, active_world_positive_axes=world_axes,
                         hand_tcp_initial_displacement_mm_per_degree_in_requested_direction=ee_motion))
        elapsed += duration+1.5
    result = dict(model_path=str(path), model_sha256=hashlib.sha256((path.parent/'openarm_bimanual.xml').read_bytes()).hexdigest(),
                  start_source=str(args.snapshot or args.start) if (args.snapshot or args.start) else 'calibrated logical zero assumption; not live feedback',
                  start_logical_rad={f'{s}:{j}':float(v) for (s,j),v in zip(keys,start)},
                  coordinate_convention='unified-forward-inward-v1-20261116',
                  trajectory='quintic rest-to-rest, 501 samples per move, simultaneous bilateral same joint',
                  torque_model='M(q) qddot + bias(q,qdot) - passive(q,qdot) + frictionloss sign(qdot); no environmental contact',
                  caveats=['Model uses original V1 grippers and their CAD inertias; actual hands/payload unknown.',
                           'Model comments mark rotor inertia and gearbox damping as approximate/unconfirmed.',
                           'Numerical torque estimates are not hardware measurements or guaranteed upper bounds.',
                           'Static holding friction is excluded from gravity estimate; it may add or subtract up to declared frictionloss.',
                           'Only modeled collision meshes are checked; no external table, cables, piano or people.',
                           'Planning duration excludes feedback-based settling; 1.5 s hold starts after arrival.'],
                  nominal_total_s=elapsed, move_total_s=sum(t for j,d,t in STEPS), rows=rows,
                  all_axes_max_torque_nm={f'{s}:{j}':float(v) for (s,j),v in zip(keys,max_all)},
                  all_axes_max_static_gravity_torque_nm={f'{s}:{j}':float(v) for (s,j),v in zip(keys,max_hold)},
                  all_axes_final_gravity_torque_nm={f'{s}:{j}':float(v) for (s,j),v in zip(keys,hold)},
                  final_logical_deg={f'{s}:{j}':float(math.degrees(v)) for (s,j),v in zip(keys,pose)},
                  joint_limit_check={f'{s}:{j}':dict(start_deg=float(math.degrees(start[i])), end_deg=float(math.degrees(pose[i])), lower_deg=float(math.degrees(model.jnt_range[joints[i],0])), upper_deg=float(math.degrees(model.jnt_range[joints[i],1])), within=bool(model.jnt_range[joints[i],0] <= min(start[i],pose[i]) <= max(start[i],pose[i]) <= model.jnt_range[joints[i],1])) for i,(s,j) in enumerate(keys)},
                  all_axes_model_torque_within_rated=bool(np.all(max_all<=limits)),
                  collision_contact_sample_count=contact_samples, collision_pairs_min_distance_m=contacts,
                  model_damping={f'{s}:{j}':float(v) for (s,j),v in zip(keys,model.dof_damping[didx])},
                  model_armature={f'{s}:{j}':float(v) for (s,j),v in zip(keys,model.dof_armature[didx])})
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k in ('nominal_total_s','all_axes_max_torque_nm','all_axes_final_gravity_torque_nm','all_axes_model_torque_within_rated','collision_contact_sample_count','collision_pairs_min_distance_m')}, ensure_ascii=False, indent=2))
    for r in rows:
        print(r['joint'], r['move_s'], r['peak_velocity_deg_s'], r['peak_acceleration_rad_s2'], r['active_axes_peak_torque_nm'])


if __name__ == '__main__':
    main()
