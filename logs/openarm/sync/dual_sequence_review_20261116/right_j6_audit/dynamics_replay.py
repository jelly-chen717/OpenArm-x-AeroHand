"""Offline replay of existing feedback into V1 dynamics; no hardware imports."""
from pathlib import Path
import hashlib
import json
import math
import sys
import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0, str(ROOT/'openarm/src'))
from openarm_app.kinematics.model_adapter import load_v1

SOURCE = HERE.parent/'execution_20261116_retry1/execution.jsonl'
REQUEST = SOURCE.with_name('request.authorized.json')
rows = [json.loads(line) for line in SOURCE.read_text(encoding='utf-8').splitlines()]
samples = [r for r in rows if r.get('type') == 'sample']
session = next(r for r in rows if r['type'] == 'session')
keys = [tuple(k) for k in session['keys']]
request = json.loads(REQUEST.read_text(encoding='utf-8'))
settings = {(r['side'],r['motor_id']):r for r in json.loads(SOURCE.with_name('settings.json').read_text(encoding='utf-8'))}
mp = json.loads((ROOT/'configs/openarm_joint_mapping.json').read_text(encoding='utf-8'))
mapping = {(r['side'],r['motor_id']):r['hardware_mapping'] for r in mp['joints']}
setup, _, model_path = load_v1()
model, data = setup.model, setup.data
joints = [mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,f'openarm_{s}_joint{j}') for s,j in keys]
qi, vi = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
idx6, idx2 = keys.index(('right',6)), keys.index(('right',2))
focus = [('left',6),('right',6),('left',2),('right',2)]
focus_indices = [keys.index(k) for k in focus]
tt = np.array([r['t'] for r in samples])
vv = np.array([r['measured_velocity'] for r in samples])
qq = np.array([r['measured'] for r in samples])
aa_raw = np.gradient(vv, tt, axis=0)
aa_smooth = np.zeros_like(vv)
for n in range(len(samples)):
    use = np.arange(max(0,n-3),min(len(samples),n+4))
    # Offline centered local linear regression, at most ~0.24 s window.
    times = tt[use]-tt[n]
    design = np.column_stack([np.ones(len(use)), times])
    aa_smooth[n] = np.linalg.lstsq(design,vv[use],rcond=None)[0][1]
stage_starts = {r['stage']:r['t'] for r in rows if r['type']=='stage_start'}
stage_ends = {r['stage']:r['t'] for r in rows if r['type']=='stage_complete'}
j2_start = stage_starts['J2']
matrix = np.zeros((model.nv,model.nv))
records = []

def descendants(body):
    answer=[]
    for b in range(model.nbody):
        parent=b
        while parent and parent != body:
            parent=int(model.body_parentid[parent])
        if parent==body:
            answer.append(b)
    return answer

distal=descendants(int(model.jnt_bodyid[joints[idx6]]))
distal_mass=float(sum(model.body_mass[b] for b in distal))
body_rows=[dict(name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b),mass_kg=float(model.body_mass[b]),com_in_body_m=model.body_ipos[b].tolist()) for b in distal]

for n,s in enumerate(samples):
    data.qpos[qi]=qq[n]
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    gravity=data.qfrc_bias.copy()
    gravity6_by_body=[]
    # Sum the torque required to oppose m*g about the logical J6 world axis.
    axis=data.xaxis[joints[idx6]].copy()
    anchor=data.xanchor[joints[idx6]].copy()
    for b in distal:
        tau=-float(np.dot(axis,np.cross(data.xipos[b]-anchor,model.body_mass[b]*model.opt.gravity)))
        gravity6_by_body.append(dict(body=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b),gravity_hold_nm=tau))
    com=sum((model.body_mass[b]*data.xipos[b] for b in distal),start=np.zeros(3))/distal_mass
    jacp,jacr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
    hand=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'openarm_right_hand_tcp')
    mujoco.mj_jacBody(model,data,jacp,jacr,hand)
    unit_payload_gravity=-float(np.dot(jacp[:,vi[idx6]],model.opt.gravity))
    data.qvel[vi]=vv[n]
    mujoco.mj_forward(model,data)
    bias=data.qfrc_bias.copy()
    passive=data.qfrc_passive.copy()
    mujoco.mj_fullM(model,data,matrix)
    raw_acc=np.zeros(model.nv); raw_acc[vi]=aa_raw[n]
    smooth_acc=np.zeros(model.nv); smooth_acc[vi]=aa_smooth[n]
    planned_acc=np.zeros(model.nv)
    stage_index=next(i for i,x in enumerate(request['sequence']['stages']) if x['name']==s['stage'])
    p0,p1=request['points'][stage_index:stage_index+2]
    duration=p1['t']-p0['t']
    approximate_command_t=s['t']-s.get('dispatch',{}).get('cycle_s',0.)
    u=(approximate_command_t-stage_starts[s['stage']])/duration
    if s['phase']=='moving' and 0<u<1:
        planned_acc[vi]=(np.array(p1['q'])-np.array(p0['q']))*(60*u-180*u*u+120*u**3)/duration**2
    inertia_raw=matrix@raw_acc
    inertia_smooth=matrix@smooth_acc
    inertia_planned=matrix@planned_acc
    friction=model.dof_frictionloss*np.sign(data.qvel)
    known=bias-passive+friction
    feedback={(m['side'],m['motor_id']):m for m in s['motor_feedback']}
    record=dict(t=s['t'],stage=s['stage'],phase=s['phase'],since_j2_s=s['t']-j2_start,
                right_j6_world_positive_axis=axis.tolist(),right_j6_anchor_m=anchor.tolist(),
                right_j6_distal_com_world_m=com.tolist(),right_j6_gravity_by_body=gravity6_by_body,
                right_j6_effective_horizontal_moment_arm_m=float(abs(gravity[vi[idx6]])/(distal_mass*np.linalg.norm(model.opt.gravity))),
                right_j6_tcp_extra_mass_gravity_nm_per_kg=unit_payload_gravity,
                right_j6_M66_kgm2=float(matrix[vi[idx6],vi[idx6]]),
                right_j6_M62_kgm2=float(matrix[vi[idx6],vi[idx2]]),
                right_j6_inertia_from_right_j2_raw_nm=float(matrix[vi[idx6],vi[idx2]]*raw_acc[vi[idx2]]),
                right_j6_inertia_from_right_j2_smoothed_nm=float(matrix[vi[idx6],vi[idx2]]*smooth_acc[vi[idx2]]),
                right_j6_inertia_from_right_j2_planned_nm=float(matrix[vi[idx6],vi[idx2]]*planned_acc[vi[idx2]]),
                modeled_contact_count=int(data.ncon),joints={})
    for i in focus_indices:
        key=keys[i]; dof=vi[i]; mpv=mapping[key]; fb=feedback[key]; gains=settings[key]
        # Power-conjugate torque mapping; current scale is 1 on all 14 axes.
        measured_tau=fb['torque_nm']/(mpv['direction']*mpv['scale'])
        error=qq[n,i]-s['target'][i]
        pd_estimate=-gains['kp']*error+gains['kd']*(s['target_velocity'][i]-vv[n,i])
        record['joints'][f'{key[0]}:{key[1]}']=dict(q_deg=float(math.degrees(qq[n,i])),q_target_deg=math.degrees(s['target'][i]),error_deg=float(math.degrees(error)),
            velocity_rad_s=float(vv[n,i]),target_velocity_rad_s=s['target_velocity'][i],
            feedback_torque_motor_nm=fb['torque_nm'],feedback_torque_logical_nm=measured_tau,
            pd_from_logged_position_velocity_nm=float(pd_estimate),
            model_static_gravity_nm=float(gravity[dof]),model_coriolis_nm=float(bias[dof]-gravity[dof]),
            model_viscous_nm=float(-passive[dof]),model_coulomb_assumed_nm=float(friction[dof]),
            qddot_raw_diff_rad_s2=float(raw_acc[dof]),qddot_smoothed_rad_s2=float(smooth_acc[dof]),qddot_plan_rad_s2=float(planned_acc[dof]),
            model_inertia_raw_nm=float(inertia_raw[dof]),model_inertia_smoothed_nm=float(inertia_smooth[dof]),model_inertia_planned_nm=float(inertia_planned[dof]),
            model_total_raw_nm=float(known[dof]+inertia_raw[dof]),model_total_smoothed_nm=float(known[dof]+inertia_smooth[dof]),model_total_plan_acc_at_measured_pose_nm=float(known[dof]+inertia_planned[dof]),
            feedback_minus_model_smoothed_nm=float(measured_tau-known[dof]-inertia_smooth[dof]))
    assert abs(sum(b['gravity_hold_nm'] for b in gravity6_by_body)-gravity[vi[idx6]])<1e-8
    records.append(record)

windows={
    'after_J6_arrival_dwell':[r for r in records if r['stage']=='J6' and r['phase']=='settling'],
    'before_J2_last_second':[r for r in records if j2_start-1<=r['t']<=j2_start],
    'during_J2':[r for r in records if r['stage']=='J2'],
}
def stats(values):
    values=np.asarray(values,dtype=float)
    return dict(min=float(np.min(values)),max=float(np.max(values)),mean=float(np.mean(values)),abs_max=float(np.max(np.abs(values))))
summaries={}
for name,window in windows.items():
    summaries[name]=dict(sample_count=len(window),first_t=window[0]['t'],last_t=window[-1]['t'],joints={})
    for key in ('left:6','right:6','left:2','right:2'):
        fields=['error_deg','velocity_rad_s','feedback_torque_logical_nm','model_static_gravity_nm','model_coriolis_nm','model_viscous_nm','model_inertia_raw_nm','model_inertia_smoothed_nm','model_inertia_planned_nm','model_total_raw_nm','model_total_smoothed_nm','model_total_plan_acc_at_measured_pose_nm','feedback_minus_model_smoothed_nm']
        summaries[name]['joints'][key]={field:stats([r['joints'][key][field] for r in window]) for field in fields}
    summaries[name]['right_j6_coupling']={field:stats([r[field] for r in window]) for field in ['right_j6_M66_kgm2','right_j6_M62_kgm2','right_j6_inertia_from_right_j2_raw_nm','right_j6_inertia_from_right_j2_smoothed_nm','right_j6_inertia_from_right_j2_planned_nm']}
result=dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),analysis='OFFLINE_EXISTING_LOG_MODEL_REPLAY_NO_CAN',
    model_path=str(model_path.relative_to(ROOT)),model_xml_sha256=hashlib.sha256((model_path.parent/'openarm_bimanual.xml').read_bytes()).hexdigest(),model_gravity_m_s2=model.opt.gravity.tolist(),model_right_j6_downstream_mass_kg=distal_mass,model_right_j6_downstream_bodies=body_rows,
    model_damping_right_j6=float(model.dof_damping[vi[idx6]]),model_armature_right_j6=float(model.dof_armature[vi[idx6]]),
    prior_planned_right_j6_whole_sequence_peak_nm=json.loads(SOURCE.with_name('dynamics.json').read_text(encoding='utf-8'))['all_axes_max_torque_nm']['right:6'],
    user_confirmed_context=['Right wrist has the original gripper, so an Aero/different-tool explanation is unsupported.',
      'User did not observe cable tension or structural contact during outward J2 motion; this observation does not conclusively exclude external force.'],
    caveats=['Model uses original V1 gripper CAD mass/COM, matching the confirmed tool type; physical mass, COM, mounting/gravity alignment and external forces were not measured.',
      'Model armature, damping and gearbox contributions are approximate; modeled friction is not identified from this motor.',
      'Feedback torque is a decoded motor-reported value; no independent joint torque sensor calibration was supplied.',
      'Position/velocity feedback in a cycle is asynchronous and quantized. Acceleration from finite differences is a noisy diagnostic, not ground truth.',
      'Smoothed acceleration is a local linear fit to reported velocity over up to seven samples (~0.24 s); final estimates are one-sided.',
      'Planned acceleration uses approximate command time from cycle-end timestamp minus cycle duration, evaluated at measured pose; it is not measured acceleration.',
      'No static plateau exists at the later failing J2 pose: J2 is still moving at shutdown.',
      '40 ms feedback cannot exclude intersample vibration or acceleration; small replay inertia terms are not guaranteed bounds on physical transients.',
      'Near zero velocity, the sign of quantized velocity is not reliable evidence for Coulomb-friction direction; static gravity is reported separately.',
      'Residual differences cannot uniquely identify payload, mounting/gravity mismatch, friction, cable force, interference, or feedback/model error.'],
    next_hardware_trial_plan_not_authorized=[
      'Confirm the current physical pose and support before any new trial; keep all existing position/torque/temperature protection thresholds.',
      'After a separately approved recovery/approach, hold the pre-J2 pose with every joint fixed for at least 5 s of settled feedback.',
      'Use small separately reviewed outward J2 increments, with slow approach and at least 5 s of fully static plateau at each pose; keep J6 target fixed.',
      'Stop at the existing first protection violation; do not widen 2 degree holding error or 3 Nm wrist threshold to collect a later plateau.',
      'Compare angle-dependent steady torque/error against approach transients and verify actual gripper mass/COM and torque-report units before gain or gravity-feedforward changes.'],
    windows=summaries,
    checkpoints=dict(before_j2=windows['before_J2_last_second'][-1],first_j2=windows['during_J2'][0],mid_j2=min(windows['during_J2'],key=lambda r:abs(r['since_j2_s']-.7)),last_enabled=records[-1]),
    samples=records)
out=HERE/'dynamics_replay.json'
out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(downstream_mass_kg=distal_mass,gravity=model.opt.gravity.tolist(),checkpoints={name:{'t':r['t'],'R6':r['joints']['right:6'],'R2':r['joints']['right:2'],'M62':r['right_j6_M62_kgm2']} for name,r in result['checkpoints'].items()}),ensure_ascii=False,indent=2))
