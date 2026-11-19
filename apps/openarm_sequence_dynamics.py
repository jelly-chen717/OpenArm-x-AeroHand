"""Offline V1 estimates for absolute model-space, rest-to-rest trajectories.

No transport, CAN library, motor commands or authorization are involved.
The returned torques are sampled model estimates, not hardware guarantees.
"""
from pathlib import Path
import hashlib
import math
import sys

from openarm_joint_mapping import CONVENTION_VERSION, finite, load_mapping


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_KEYS = {(side, joint) for side in ('left', 'right') for joint in range(1, 8)}


def _parse(request, samples_per_move):
    if type(samples_per_move) is not int or samples_per_move < 3:
        raise ValueError('samples_per_move must be an integer >= 3')
    if any(type(row['motor_id']) is not int for row in request['joints']):
        raise ValueError('Motor IDs must be integers')
    keys = tuple((row['side'], row['motor_id']) for row in request['joints'])
    if len(keys) != 14 or set(keys) != EXPECTED_KEYS:
        raise ValueError('Dynamics requires 14 unique left/right arm joints')
    points = [(finite(row['t']), tuple(finite(q) for q in row['q']))
              for row in request['points']]
    if len(points) < 2 or points[0][0] != 0 or any(len(q) != 14 for _, q in points):
        raise ValueError('Need 14-axis points starting at time zero')
    if any(b[0] <= a[0] for a, b in zip(points, points[1:])):
        raise ValueError('Point times must strictly increase')
    dwell = finite(request.get('sequence', {}).get('dwell_s', 0.))
    if dwell < 0:
        raise ValueError('Negative dwell time')
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    if request.get('mapping_version') != version:
        raise ValueError('Mapping version mismatch')
    return keys, points, dwell, mappings


def _violations(keys, points, limits, source):
    # Each quintic segment is monotone in every coordinate. Checking ALL knots
    # therefore checks its exact extrema, including intermediate reversals.
    return [dict(source=source, point_index=p, time_s=t, side=key[0],
                 motor_id=key[1], value_deg=math.degrees(q[i]),
                 lower_deg=math.degrees(limits[i][0]),
                 upper_deg=math.degrees(limits[i][1]))
            for p, (t, q) in enumerate(points) for i, key in enumerate(keys)
            if not limits[i][0] <= q[i] <= limits[i][1]]


def analyze_dynamics(request: dict, samples_per_move: int = 201) -> dict:
    """Estimate every moving/holding axis along the request, never opening CAN.

    Expiration and authorization are deliberately irrelevant to offline analysis;
    the hardware executor remains responsible for both. Absolute joint limits
    and coordinate convention are checked here before any torque is estimated.
    """
    keys, points, dwell, mappings = _parse(request, samples_per_move)
    limits = [mappings[key].limits for key in keys]
    violations = _violations(keys, points, limits, 'joint_mapping')
    if violations:
        return dict(status='BLOCKED_LIMITS', violations=violations,
                    torque_estimated=False,
                    limit_check='All knots; monotone quintic interval extrema')

    # Delay heavy simulation imports, so invalid requests fail without a model.
    import numpy as np
    import mujoco
    source_path = str(ROOT/'openarm/src')
    if source_path not in sys.path:
        sys.path.insert(0, source_path)
    from openarm_app.kinematics.model_adapter import load_v1

    setup, _, model_path = load_v1()
    model, data = setup.model, setup.data
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                              f'openarm_{side}_joint{joint}') for side, joint in keys]
    if min(joints) < 0 or not np.all(model.jnt_limited[joints]):
        raise ValueError('Missing bounded joint in V1 model')
    model_limits = model.jnt_range[joints]
    violations = _violations(keys, points, model_limits, 'compiled_v1_model')
    if violations:
        return dict(status='BLOCKED_LIMITS', violations=violations, torque_estimated=False)
    qidx, didx = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    labels = [f'{s}:{j}' for s, j in keys]
    as_axes = lambda values: {key: float(v) for key, v in zip(labels, values)}
    rated = np.array([20. if j <= 2 else 9. if j <= 4 else 3. for _, j in keys])
    matrix = np.zeros((model.nv, model.nv))
    max_all = np.zeros(14)
    max_hold = np.zeros(14)
    contacts, rows = {}, []
    contact_samples = 0
    sample_count = 0

    def set_state(q, velocity, acceleration):
        data.qpos[qidx] = q
        data.qvel[:] = 0.
        data.qvel[didx] = velocity
        data.qacc[:] = 0.
        prescribed = np.zeros(model.nv)
        prescribed[didx] = acceleration
        mujoco.mj_forward(model, data)
        return prescribed

    def record_contacts(stage_contacts=None):
        nonlocal contact_samples, sample_count
        sample_count += 1
        if data.ncon:
            contact_samples += 1
        for contact in data.contact[:data.ncon]:
            pair = ' | '.join(sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM,
                                                     int(g)) or str(g)
                                     for g in (contact.geom1, contact.geom2)))
            contacts[pair] = min(float(contact.dist), contacts.get(pair, float('inf')))
            if stage_contacts is not None:
                stage_contacts[pair] = min(float(contact.dist), stage_contacts.get(pair, float('inf')))

    # Initial and each endpoint hold count toward the all-axis load estimate.
    set_state(np.array(points[0][1]), np.zeros(14), np.zeros(14))
    initial_hold = data.qfrc_bias[didx].copy()
    initial_contact_count = int(data.ncon)
    max_all = np.maximum(max_all, np.abs(initial_hold))
    max_hold = np.maximum(max_hold, np.abs(initial_hold))
    record_contacts()
    for index, ((t0, q0), (t1, q1)) in enumerate(zip(points, points[1:]), 1):
        duration = t1-t0
        start, end = np.array(q0), np.array(q1)
        delta = end-start
        active = np.flatnonzero(np.abs(delta) > 1e-12)
        peak = np.zeros(14)
        contact_count_before = contact_samples
        stage_contacts = {}
        first_contact_u = None
        last_contact_u = None
        for u in np.linspace(0., 1., samples_per_move):
            blend = 10*u**3-15*u**4+6*u**5
            rate = 30*u*u*(1-u)**2/duration
            accel = (60*u-180*u*u+120*u**3)/duration**2
            prescribed = set_state(start+delta*blend, delta*rate, delta*accel)
            mujoco.mj_fullM(model, data, matrix)
            # Unconstrained load: include model passive damping and dynamic
            # Coulomb friction; never substitute inverse constraint reactions.
            torque = (matrix@prescribed + data.qfrc_bias-data.qfrc_passive
                      + model.dof_frictionloss*np.sign(data.qvel))[didx]
            if not np.all(np.isfinite(torque)):
                raise ValueError('Non-finite model torque')
            peak = np.maximum(peak, np.abs(torque))
            record_contacts(stage_contacts)
            if data.ncon and first_contact_u is None:
                first_contact_u = float(u)
            if data.ncon:
                last_contact_u = float(u)
        set_state(end, np.zeros(14), np.zeros(14))
        hold = data.qfrc_bias[didx].copy()
        peak = np.maximum(peak, np.abs(hold))
        max_all = np.maximum(max_all, peak)
        max_hold = np.maximum(max_hold, np.abs(hold))
        rows.append(dict(stage_index=index, move_s=duration, arrival_hold_s=dwell,
                         nominal_move_start_s=t0+(index-1)*dwell,
                         nominal_move_end_s=t1+(index-1)*dwell,
                         delta_deg=as_axes(np.degrees(delta)),
                         peak_velocity_deg_s=as_axes(1.875*np.abs(np.degrees(delta))/duration),
                         peak_velocity_rad_s=as_axes(1.875*np.abs(delta)/duration),
                         peak_acceleration_rad_s2=as_axes(10/math.sqrt(3)*np.abs(delta)/duration**2),
                         active_axes_peak_torque_nm={labels[i]:float(peak[i]) for i in active},
                         all_axes_peak_torque_nm=as_axes(peak),
                         all_axes_end_hold_gravity_torque_nm=as_axes(hold),
                         all_axes_min_margin_nm=float(np.min(rated-peak)),
                         collision_contact_sample_count=contact_samples-contact_count_before,
                         collision_pairs_min_distance_m=stage_contacts,
                         first_contact_fraction=first_contact_u,
                         last_contact_fraction=last_contact_u))
    sources = [model_path, model_path.parent/'openarm_bimanual.xml',
               ROOT/'configs/openarm_joint_convention.json',
               ROOT/'openarm/src/openarm_app/joint_convention.py']
    hashes = {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sources}
    return dict(status='MODEL_ESTIMATE', torque_estimated=True, violations=[],
                model_path=str(model_path), source_sha256=hashes,
                coordinate_convention=CONVENTION_VERSION,
                start_logical_rad=as_axes(points[0][1]), final_logical_deg=as_axes(np.degrees(points[-1][1])),
                trajectory='Rest-to-rest quintic, absolute logical angles',
                limit_check='All knots; monotone quintic interval extrema',
                samples_per_move=samples_per_move, sample_count=sample_count,
                torque_model='M(q) qdd + bias(q,qd) - passive(q,qd) + frictionloss sign(qd)',
                move_total_s=points[-1][0], nominal_dwell_total_s=dwell*len(rows),
                nominal_total_s=points[-1][0]+dwell*len(rows), rows=rows,
                all_axes_max_torque_nm=as_axes(max_all),
                all_axes_max_static_gravity_torque_nm=as_axes(max_hold),
                all_axes_initial_gravity_torque_nm=as_axes(initial_hold),
                all_axes_final_gravity_torque_nm=as_axes(hold),
                all_axes_rated_torque_nm=as_axes(rated),
                all_axes_model_torque_within_rated=bool(np.all(max_all <= rated)),
                modeled_path_contact_free=contact_samples == 0,
                initial_pose_contact_count=initial_contact_count,
                collision_contact_sample_count=contact_samples,
                collision_pairs_min_distance_m=contacts,
                caveats=[
                    'Sampled model estimates are not measured torques or guaranteed upper bounds.',
                    'V1 original gripper CAD masses/inertias and approximate motor/gearbox inertia/damping are used.',
                    'External cable loads and unmodeled payloads are excluded; prior real load exceeded this model.',
                    'Static hold figures are gravity only, excluding uncertain static friction and external loads.',
                    'Contacts cover model collision meshes only, excluding cables, tables and people.',
                    'When contacts are detected, reported torques exclude contact forces and cannot validate that path.',
                    'Timing excludes measured settling; dwell begins after actual arrival.',
                ])
