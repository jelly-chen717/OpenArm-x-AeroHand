"""Read-only hardware-log analysis; writes only this run's analysis.json."""
import hashlib
import json
import math
from pathlib import Path

RUN = Path(__file__).resolve().parent
path = RUN/'execution.jsonl'
rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]
request = json.loads((RUN/'request.authorized.json').read_text(encoding='utf-8'))
settings = json.loads((RUN/'settings.json').read_text(encoding='utf-8'))
keys = [tuple(j) for j in next(r for r in rows if r['type'] == 'session')['keys']]
config = {(r['side'],r['motor_id']):r for r in settings}
baseline = request['initial_model_position']
samples = [r for r in rows if r['type']=='sample']
last = samples[-1]


def feedback(row, key):
    return next(m for m in row['motor_feedback'] if (m['side'],m['motor_id'])==key)


def stats(values):
    values=sorted(values)
    def percentile(f):
        x=f*(len(values)-1)
        a,b=math.floor(x),math.ceil(x)
        return values[a]+(values[b]-values[a])*(x-a)
    return dict(n=len(values), min=min(values), p50=percentile(.5),p95=percentile(.95),max=max(values))


joint_stats=[]
for i,key in enumerate(keys):
    motor=[feedback(r,key) for r in samples]
    joint_stats.append(dict(side=key[0],motor_id=key[1],
        last_enabled_delta_deg=math.degrees(last['measured'][i]-baseline[i]),
        last_command_delta_deg=math.degrees(last['target'][i]-baseline[i]),
        last_enabled_tracking_error_deg=math.degrees(last['error'][i]),
        last_enabled_velocity_rad_s=last['measured_velocity'][i],
        last_enabled_torque_nm=motor[-1]['torque_nm'],
        max_abs_tracking_error_deg=max(abs(math.degrees(r['error'][i])) for r in samples),
        max_abs_velocity_rad_s=max(abs(r['measured_velocity'][i]) for r in samples),
        rated_speed_rad_s=config[key]['max_speed'],
        max_abs_torque_nm=max(abs(m['torque_nm']) for m in motor),
        rated_torque_nm=config[key]['max_torque'],
        rated_speed_exceed_count=sum(abs(r['measured_velocity'][i])>config[key]['max_speed'] for r in samples),
        rated_torque_exceed_count=sum(abs(m['torque_nm'])>config[key]['max_torque'] for m in motor),
        max_temperature_c=max(max(m['mos_c'],m['rotor_c']) for m in motor),
        states_during_samples=sorted({m['state'] for m in motor})))

stages=[]
for done in (r for r in rows if r['type']=='stage_complete'):
    selected=[r for r in samples if r['stage']==done['stage']]
    accepted=max((r for r in selected if r['t']<=done['t']+1e-9),key=lambda r:r['t'])
    motor_id=int(done['stage'][1:])
    stages.append(dict(stage=done['stage'],completed_at_s=done['t'],
        actual_move_s=done['actual_move_s'],planned_move_s=done['planned_move_s'],
        arrival_wait_s=done['arrival_wait_s'],continuous_dwell_s=done['dwell_s'],
        all_axes_max_settled_speed_rad_s=max(map(abs,done['measured_velocity'])),
        accepted_axes=[dict(side=key[0],motor_id=key[1],
            delta_deg=math.degrees(accepted['measured'][i]-baseline[i]),
            error_deg=math.degrees(done['error'][i]),
            speed_rad_s=done['measured_velocity'][i],
            torque_nm=feedback(accepted,key)['torque_nm'])
            for i,key in enumerate(keys) if key[1]==motor_id]))

j2=[r for r in samples if r['stage']=='J2']
i=keys.index(('right',6))
trend=[dict(t=r['t'],error_deg=math.degrees(r['error'][i]),
            velocity_rad_s=r['measured_velocity'][i],torque_nm=feedback(r,('right',6))['torque_nm'],
            right_j2_delta_deg=math.degrees(r['measured'][keys.index(('right',2))]-baseline[keys.index(('right',2))]))
       for r in j2]
over_2=[r for r in trend if abs(r['error_deg'])>2.]
over_1=[r for r in trend if abs(r['error_deg'])>1.]
nondecreasing=sum(b['error_deg']>=a['error_deg']-1e-9 for a,b in zip(trend,trend[1:]))
j2_details=[]
for side in ('left','right'):
    key=(side,2)
    index=keys.index(key)
    motor=[feedback(r,key) for r in j2]
    series=[dict(t=r['t'],command_delta_deg=math.degrees(r['target'][index]-baseline[index]),
                 measured_delta_deg=math.degrees(r['measured'][index]-baseline[index]),
                 tracking_error_deg=math.degrees(r['error'][index]),
                 command_speed_deg_s=math.degrees(r['target_velocity'][index]),
                 measured_speed_deg_s=math.degrees(r['measured_velocity'][index]),
                 torque_nm=m['torque_nm']) for r,m in zip(j2,motor)]
    j2_details.append(dict(side=side,peak_measured_speed_deg_s=max(abs(r['measured_speed_deg_s']) for r in series),
        peak_measured_speed_rad_s=max(abs(r['measured_velocity'][index]) for r in j2),
        last=series[-1],peak_abs_torque_nm=max(abs(m['torque_nm']) for m in motor),
        rated_torque_nm=config[key]['max_torque'],peak_torque_ratio=max(abs(m['torque_nm']) for m in motor)/config[key]['max_torque'],
        last_6_abs_torques_nm=[abs(m['torque_nm']) for m in motor[-6:]],
        last_6_torques_strictly_increasing=all(abs(b['torque_nm'])>abs(a['torque_nm']) for a,b in zip(motor[-6:],motor[-5:])),
        series=series))
disable=next(r for r in rows if r['type']=='disable_evidence')
after=next(r for r in rows if r['type']=='after')
motors=[dict(side=report['side'],**m) for report in after['reports'] for m in report['motors']]
dispatch=[r['dispatch'] for r in samples]

analysis=dict(source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    analysis_kind='offline_log_only_no_control_or_CAN_changes',
    result='Holding error exceeded: right J6 at 2.088689 degrees during J2',
    sample_count=len(samples),last_enabled_sample_t_s=last['t'],
    stages_started=[r['stage'] for r in rows if r['type']=='stage_start'],
    stages_completed=stages,
    holding_target_entered=any(r.get('state')=='HOLDING_TARGET' for r in rows),
    failure_events=[r for r in rows if r['type'] in ('fault','hold_tolerance_exceeded')],
    right_j6=dict(trend_during_j2=trend,
        first_j2_error_deg=trend[0]['error_deg'],last_error_deg=trend[-1]['error_deg'],
        first_j2_torque_nm=trend[0]['torque_nm'],last_torque_nm=trend[-1]['torque_nm'],
        over_2deg_samples_before_shutdown=len(over_2),
        over_2deg_sample_duration_s=over_2[-1]['t']-over_2[0]['t'],
        first_over_1deg_t_s=over_1[0]['t'] if over_1 else None,
        nondecreasing_error_transitions=nondecreasing,total_j2_transitions=len(trend)-1,
        note='Only one >2-degree sample exists because the first violation stopped execution; post-threshold persistence is unobserved. The preceding rising error and torque are not an isolated random spike.'),
    joints=joint_stats,
    j2=j2_details,
    j2_slower_candidate=dict(status='proposed_only_no_execution',old_move_s=1.8,new_move_s=4.,
        delta_deg=-45.,old_peak_speed_deg_s=1.875*45/1.8,new_peak_speed_deg_s=1.875*45/4.,
        old_peak_acceleration_rad_s2=(10/math.sqrt(3))*math.radians(45)/(1.8**2),
        new_peak_acceleration_rad_s2=(10/math.sqrt(3))*math.radians(45)/(4.**2),
        other_motion_parameters='unchanged candidate',
        limitation='Lower dynamic speed/acceleration does not establish that right-J6 static holding torque or error will be acceptable.'),
    no_rated_speed_or_torque_exceeded_in_recorded_samples=all(j['rated_speed_exceed_count']==0 and j['rated_torque_exceed_count']==0 for j in joint_stats),
    timing=dict(sample_interval_s=stats([b['t']-a['t'] for a,b in zip(samples,samples[1:])]),
                cycle_s=stats([d['cycle_s'] for d in dispatch]),
                send_span_s=stats([d['send_span_s'] for d in dispatch]),
                pair_send_skew_s={name:stats([d['pair_send_skew_s'][name] for d in dispatch]) for name in dispatch[0]['pair_send_skew_s']}),
    shutdown=dict(stopped_event=next(r for r in rows if r.get('state')=='STOPPED'),
        confirmed_disable_count=len({(m['side'],m['motor_id']) for m in disable['records'] if m.get('confirmed')}),
        all_14_after_disabled=len(motors)==14 and all(m['state']==0 for m in motors),
        buses=[dict(side=r['side'],online=r['online'],bus_status=r['bus_status']) for r in after['reports']],
        after_motors=motors),
    recommendations=[
        'Treat the completed first five stages as passed at their recorded acceptance times; J2 and J1 remain unaccepted.',
        'The revised 0.05 rad/s settling criterion allowed the first five stages to complete with the required continuous dwell.',
        'Do not classify the right-J6 error as mere noise: error grew from about 0.36 to 2.09 degrees while its absolute torque rose from about 0.42 to 2.40 Nm as J2 moved.',
        'A short persistence window/hysteresis is appropriate for isolated small excursions, but this monotone load-correlated trend needs separate evaluation; no data proves it would settle after the next sample.',
        'Right J6 already uses about 80% of its 3 Nm rating. Further J2 motion may increase static load; a blanket wider position threshold or larger Kp is not justified by this stopped trace alone.',
        'If planning another authorized attempt, first evaluate slower J2 motion to reduce inertial disturbance and examine right-J6 cable/payload/gravity effects. Slowing motion does not remove static holding torque.',
        'The user identified J2 speed as excessive. The proposed 4.0-second movement reduces planned peak speed from 46.875 to 21.09375 deg/s and peak acceleration from 1.39954 to 0.28341 rad/s^2; it has not been executed.',
        'Right J2 reached 18.210989 Nm, 91.05% of its 20 Nm rating. Its final six torque magnitudes strictly increased while commanded velocity was already falling; this trace shows no torque recovery before shutdown.',
    ])
(RUN/'analysis.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(stages=stages,j2_and_failed_axis=[j for j in joint_stats if j['motor_id']==2 or (j['side'],j['motor_id'])==('right',6)],
    rated_ok=analysis['no_rated_speed_or_torque_exceeded_in_recorded_samples'],
    max_torque_ratio=max(j['max_abs_torque_nm']/j['rated_torque_nm'] for j in joint_stats),
    right_j6={k:v for k,v in analysis['right_j6'].items() if k!='trend_during_j2'},
    shutdown={k:v for k,v in analysis['shutdown'].items() if k!='after_motors'}),ensure_ascii=False,indent=2))
