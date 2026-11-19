"""Build the frozen review package from read-only feedback. Never opens CAN."""
from pathlib import Path
import copy
import hashlib
import json
import math
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'apps'))
from openarm_joint_mapping import load_mapping
from openarm_execution import PcanTransport, rated_limits
from openarm_trajectory import preflight
from drive_openarm_from_sim import validate_sequence

STEPS = [(7,-15.,.8,'向后'),(6,-30.,1.2,'向外'),(5,45.,1.5,'俯视顺时针'),
         (4,3.,.6,'向前'),(3,-45.,1.5,'俯视逆时针'),(2,-45.,1.8,'向外'),(1,-45.,1.8,'向后')]
GAINS = {1:(300.,5.),2:(300.,5.),3:(80.,3.),4:(200.,3.),5:(60.,2.),6:(60.,2.),7:(60.,2.)}
version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
snap_path = HERE/'status_confirmed_zero.jsonl'
snapshot = [json.loads(line) for line in snap_path.read_text(encoding='utf-8').splitlines()]
origins = {}
for report in snapshot:
    if report['online'] != 7 or report['bus_status'] != '0x0':
        raise ValueError('Incomplete or unhealthy baseline')
    for motor in report['motors']:
        if motor['state'] != 0:
            raise ValueError('Baseline must be disabled')
        origins[report['side'],motor['motor_id']] = motor['position_rad']
keys = [(s,j) for j,_,_,_ in STEPS for s in ('left','right')]
initial = [mappings[k].to_model(origins[k]) for k in keys]
current = dict(zip(keys, initial))
points = [dict(t=0., q=initial.copy())]
q = initial.copy()
t = 0.
stages = []
settings = []
for index, (j,delta,duration,direction) in enumerate(STEPS,1):
    for i,k in enumerate(keys):
        if k[1] == j:
            q[i] += math.radians(delta)
    t = round(t+duration,8)
    points.append(dict(t=t,q=q.copy()))
    stage = dict(name=f'J{j}',endpoint_index=index)
    if j == 4:
        stage['endpoint_tolerances_rad'] = [math.radians(.5 if k[1] == 4 else 2.) for k in keys]
    stages.append(stage)
    for s in ('left','right'):
        kp,kd = GAINS[j]
        settings.append(dict(side=s,motor_id=j,kp=kp,kd=kd,max_travel=math.radians(abs(delta)+5.),
                             **rated_limits(j),max_temperature=50.,max_age=.15))
sequence = dict(schema_version=1,dwell_s=1.5,settle_timeout_s=8.,settle_speed_rad_s=.03,
                endpoint_tolerances_rad=[math.radians(2.)]*14,stages=stages)
request = dict(request_id='dual-sequence-review-20261116-NOT-AUTHORIZED',mapping_version=version,
               expires_at=0.,authorization_status='AWAITING_USER_REVIEW',
               source='User seven-stage bilateral MIT test; frozen read-only baseline; expires_at=0 blocks execution.',
               joints=[dict(side=s,motor_id=j) for s,j in keys],initial_model_position=initial,
               initial_motor_position=[origins[k] for k in keys],
               limits=dict(velocity_rad_s=1.,acceleration_rad_s2=2.5,start_error_rad=.01),
               sequence=sequence,points=points)
# An in-memory time-limited clone verifies math only. The saved draft stays expired.
validation_request = copy.deepcopy(request)
validation_request['expires_at'] = time.time()+300
trajectory = preflight(validation_request,version,mappings,current,time.time(),set())
validate_sequence(trajectory,sequence)
joint_settings = {(r['side'],r['motor_id']):{k:v for k,v in r.items() if k not in ('side','motor_id')} for r in settings}
transport = PcanTransport(mappings,keys,dict(left=0x51,right=0x52),joint_settings=joint_settings)
transport.origins = origins
transport.validate_trajectory(trajectory)  # no context manager, no drivers, no CAN
dynamics = json.loads((HERE/'dynamics_confirmed_zero.json').read_text(encoding='utf-8'))
baseline = dict(source=str(snap_path.relative_to(ROOT)),sha256=hashlib.sha256(snap_path.read_bytes()).hexdigest(),
                zero_confirmation='User explicitly selected current pose as new test zero after initial-pose mismatch; movement approval still pending.',
                snapshot_time_unix=max(r['time_unix'] for r in snapshot),
                zero_policy='q_test = q_logical - q_logical_at_snapshot; no persistent zero/calibration changes',
                joints=[dict(side=k[0],motor_id=k[1],motor_rad=origins[k],logical_rad=current[k],
                             logical_deg=math.degrees(current[k]),final_logical_deg=math.degrees(q[i]),
                             raw_motor_delta_deg=math.degrees(mappings[k].to_motor(q[i])-origins[k])) for i,k in enumerate(keys)])
review = dict(status='READY_FOR_REVIEW_NOT_AUTHORIZED',mapping_version=version,
              channels=dict(left='0x51',right='0x52'),period_s=.04,max_lateness_s=.08,
              max_tracking_error_rad=math.radians(5),hold_after=True,batch_mit=True,
              motor_enabled_this_preparation=False,nominal_total_s=19.7,move_total_s=9.2,
              nominal_dwell_total_s=10.5,extra_time='Measured settling and control scheduling; 8s post-motion timeout per stage includes dwell.',
              initial_all_axes_hold=True,baseline=baseline,sequence=sequence,
              rows=[dict(joint=j,delta_deg=delta,direction=direction,move_s=duration,dwell_s=1.5,
                         peak_velocity_deg_s=1.875*abs(delta)/duration,
                         peak_velocity_rad_s=math.radians(1.875*abs(delta)/duration),
                         peak_acceleration_rad_s2=10/math.sqrt(3)*abs(math.radians(delta))/duration**2,
                         torque_peak_model_whole_sequence_nm=max(dynamics['all_axes_max_torque_nm'][f'{s}:{j}'] for s in ('left','right')),
                         **rated_limits(j),kp=GAINS[j][0],kd=GAINS[j][1]) for j,delta,duration,direction in STEPS],
              offline_validation=dict(trajectory=True,sequence_schema=True,hardware_limits_without_connection=True,
                                      model_all_limits=all(v['within'] for v in dynamics['joint_limit_check'].values()),
                                      model_torque=dynamics['all_axes_model_torque_within_rated'],
                                      model_contact_samples=dynamics['collision_contact_sample_count']),
              torque_note='MIT tau_ff=0; motor internally applies Kp*(qdes-q)+Kd*(vdes-v). Rated thresholds are measured-feedback trips, not firmware current clamps. Model torque is not measured or guaranteed.',
              before_authorized_execution=['Receive user approval of this concrete plan.','Read current status without motion, verify 14 disabled online joints and exclusive bus ownership.',
                  'Keep frozen review origins; if any differs by >0.01rad, stop rather than silently rebase.','Refresh request_id and expires_at only after approval; hardware preflight must pass before execute.',
                  'Use dedicated executor with --sequence --allow-dual-arm --batch-mit --hold-after and cancel file.'],
              uncertainties=dynamics['caveats'])
for filename,payload in [('request.draft.json',request),('settings.json',settings),('baseline.json',baseline),('review.json',review)]:
    (HERE/filename).write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(status=review['status'],nominal_total_s=review['nominal_total_s'],checks=review['offline_validation']),ensure_ascii=False))
