"""Offline direction audit from confirmed feedback. No hardware imports."""
from pathlib import Path
import sys,json,math
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'openarm/src'))
from openarm_app.kinematics.model_adapter import load_v1
OUT=Path(__file__).resolve().parent
req=json.loads((OUT/'reconfirmed_original/request.draft.json').read_text(encoding='utf-8'))
setup,_,_=load_v1();m,d=setup.model,setup.data
keys=[(x['side'],x['motor_id']) for x in req['joints']]
ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,f'openarm_{s}_joint{j}') for s,j in keys]
qidx=m.jnt_qposadr[ids]
baseline=np.array(req['initial_model_position'])

def pose(q):
    d.qpos[qidx]=q;mujoco.mj_forward(m,d)
    out={}
    for side,sign in [('left',1),('right',-1)]:
        j7=ids[keys.index((side,7))];j5=ids[keys.index((side,5))]
        tcp=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,f'openarm_{side}_hand_tcp')
        pivot=d.xanchor[j7].copy();p=d.xpos[tcp].copy();v=p-pivot
        out[side]=dict(tcp_m=p.tolist(),j7_anchor_m=pivot.tolist(),j7_to_tcp_m=v.tolist(),
                       j5_world_positive_axis=d.xaxis[j5].tolist(),j7_world_positive_axis=d.xaxis[j7].tolist(),
                       outward_tcp_projection_from_j7_m=float(sign*v[1]),
                       joint5_logical_deg=math.degrees(q[keys.index((side,5))]),joint7_logical_deg=math.degrees(q[keys.index((side,7))]))
    return out

cases=[]
for vals in [(-88,-88),(-88,88),(88,-88)]:
    q=baseline.copy()
    for side,val in zip(['left','right'],vals):q[keys.index((side,5))]+=math.radians(val)
    before=pose(q)
    for side in ['left','right']:q[keys.index((side,7))]+=math.radians(88)
    after=pose(q)
    motion={}
    for side,sign in [('left',1),('right',-1)]:
        delta=np.array(after[side]['tcp_m'])-np.array(before[side]['tcp_m'])
        motion[side]=dict(tcp_displacement_m=delta.tolist(),outward_motion_m=float(sign*delta[1]),
                          classification='outward' if sign*delta[1]>0 else 'inward')
    cases.append(dict(j5_test_relative_deg=dict(zip(['left','right'],vals)),j7_test_relative_target_deg=88,
                      before_j7=before,after_j7=after,j7_motion=motion))
full=[]
for i,point in enumerate(req['points']):full.append(dict(point=i,trajectory_time_s=point['t'],pose=pose(np.array(point['q']))))
result=dict(source_request=str((OUT/'reconfirmed_original/request.draft.json').relative_to(ROOT)),
            world_axes='X forward, Y robot left, Z up; outward = +Y left arm, -Y right arm',
            tcp_definition='body openarm_{side}_hand_tcp; ee_control_point is J7 pivot and is unsuitable for detecting bend',
            baseline_logical_deg={f'{s}:{j}':math.degrees(v) for (s,j),v in zip(keys,baseline)},comparisons=cases,
            original_sequence_endpoints=full,
            caveat='Offline geometric comparisons only; no changes to mapping or motion requests, no new authorization, no hardware access.')
(OUT/'direction_geometry_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(baseline=result['baseline_logical_deg'],comparisons=[dict(j5=x['j5_test_relative_deg'],motion=x['j7_motion'],after={s:x['after_j7'][s]['j7_to_tcp_m'] for s in ['left','right']}) for x in cases],full=[dict(point=x['point'],left=x['pose']['left']['j7_to_tcp_m'],right=x['pose']['right']['j7_to_tcp_m']) for x in full]),ensure_ascii=False))
