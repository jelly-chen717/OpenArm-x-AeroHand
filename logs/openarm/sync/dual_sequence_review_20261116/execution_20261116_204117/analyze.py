"""Offline analysis of this completed run only; imports no hardware modules."""
import collections
import hashlib
import json
import math
from pathlib import Path

RUN = Path(__file__).resolve().parent
LOG = RUN / 'execution.jsonl'
rows = [json.loads(line) for line in LOG.read_text(encoding='utf-8').splitlines() if line.strip()]
request = json.loads((RUN / 'request.authorized.json').read_text(encoding='utf-8'))
session = next(row for row in rows if row['type'] == 'session')
keys = [tuple(key) for key in session['keys']]
samples = [row for row in rows if row['type'] == 'sample']
settling = [row for row in samples if row['phase'] == 'settling']
sequence = next(row for row in rows if row['type'] == 'sequence_settings')
speed_threshold = sequence['settle_speed_rad_s']
tolerances = sequence['endpoint_tolerances_rad']
baseline = request['initial_model_position']


def percentiles(values):
    values = sorted(values)
    def at(fraction):
        index = fraction * (len(values)-1)
        lo, hi = math.floor(index), math.ceil(index)
        return values[lo] + (values[hi]-values[lo]) * (index-lo)
    return dict(n=len(values), min=min(values), p50=at(.5), p95=at(.95), max=max(values)) if values else None


def histogram(values):
    return [{'value': value, 'count': count} for value, count in sorted(collections.Counter(values).items())]


episodes = []
episode = None
blocked_samples = []
for row in settling:
    speed_blockers = [i for i, v in enumerate(row['measured_velocity']) if abs(v) > speed_threshold]
    position_blockers = [i for i, e in enumerate(row['error']) if abs(e) > tolerances[i]]
    good = not speed_blockers and not position_blockers
    if good:
        if episode is None:
            episode = dict(start_s=row['t'], last_good_s=row['t'], good_samples=0)
        episode['last_good_s'] = row['t']
        episode['good_samples'] += 1
    else:
        blocked_samples.append(dict(t=row['t'],
            speed=[dict(side=keys[i][0], motor_id=keys[i][1], velocity_rad_s=row['measured_velocity'][i])
                   for i in speed_blockers],
            position=[dict(side=keys[i][0], motor_id=keys[i][1], error_deg=math.degrees(row['error'][i]))
                      for i in position_blockers]))
        if episode is not None:
            episode['reset_at_s'] = row['t']
            episode['timer_when_reset_s'] = row['t']-episode['start_s']
            episodes.append(episode)
            episode = None
if episode is not None:
    episode['reset_at_s'] = None
    episode['timer_when_reset_s'] = None
    episodes.append(episode)
for episode in episodes:
    episode['confirmed_good_sample_span_s'] = episode['last_good_s']-episode['start_s']

joint_stats = []
for i, key in enumerate(keys):
    feedback = [next(m for m in row['motor_feedback'] if (m['side'], m['motor_id']) == key) for row in samples]
    velocity = [row['measured_velocity'][i] for row in samples]
    error = [row['error'][i] for row in samples]
    settle_velocity = [row['measured_velocity'][i] for row in settling]
    settle_error = [row['error'][i] for row in settling]
    positions = [row['measured'][i] for row in samples]
    joint_stats.append(dict(side=key[0], motor_id=key[1],
        active_in_run=key[1] == 7,
        last_enabled_delta_deg=math.degrees(positions[-1]-baseline[i]),
        last_enabled_error_deg=math.degrees(error[-1]),
        max_abs_tracking_error_deg=math.degrees(max(map(abs, error))),
        rms_tracking_error_deg=math.degrees(math.sqrt(sum(e*e for e in error)/len(error))),
        settling_max_abs_error_deg=math.degrees(max(map(abs, settle_error))),
        measured_position_span_deg=math.degrees(max(positions)-min(positions)),
        max_abs_velocity_rad_s=max(map(abs, velocity)),
        min_velocity_rad_s=min(velocity), max_velocity_rad_s=max(velocity),
        settling_max_abs_velocity_rad_s=max(map(abs, settle_velocity)),
        settling_speed_blocked_samples=sum(abs(v) > speed_threshold for v in settle_velocity),
        settling_position_blocked_samples=sum(abs(e) > tolerances[i] for e in settle_error),
        settling_velocity_histogram=histogram(settle_velocity),
        max_abs_torque_nm=max(abs(m['torque_nm']) for m in feedback),
        last_enabled_torque_nm=feedback[-1]['torque_nm'],
        max_temperature_c=max(max(m['mos_c'], m['rotor_c']) for m in feedback),
        enabled_sample_states=sorted(set(m['state'] for m in feedback))))

dispatch = [row['dispatch'] for row in samples]
after = next(row for row in rows if row['type'] == 'after')
disable = next(row for row in rows if row['type'] == 'disable_evidence')
after_motors = [dict(side=report['side'], **m) for report in after['reports'] for m in report['motors']]
analysis = dict(
    source=str(LOG), source_sha256=hashlib.sha256(LOG.read_bytes()).hexdigest(),
    analysis_kind='offline_completed_run_no_hardware_access',
    result='FAULT: Sequence settling timeout: J7',
    sample_count=len(samples), settling_sample_count=len(settling),
    last_enabled_sample_t_s=samples[-1]['t'],
    stages_started=[r['stage'] for r in rows if r['type'] == 'stage_start'],
    stages_completed=[r['stage'] for r in rows if r['type'] == 'stage_complete'],
    holding_target_entered=any(r.get('state') == 'HOLDING_TARGET' for r in rows),
    events=[r for r in rows if r['type'] in ('stage_motion_complete','stage_timeout','fault','state')],
    gate=dict(speed_threshold_rad_s=speed_threshold, position_threshold_deg=math.degrees(tolerances[0]),
        required_continuous_dwell_s=sequence['dwell_s'],
        good_samples=len(settling)-len(blocked_samples), blocked_samples=len(blocked_samples),
        all_position_checks_passed=all(not row['position'] for row in blocked_samples),
        arrival_events=sum(r['type'] == 'stage_arrived' for r in rows),
        dwell_reset_events=sum(r['type'] == 'stage_dwell_reset' for r in rows),
        longest_confirmed_good_episode=max(episodes, key=lambda e:e['confirmed_good_sample_span_s']),
        max_timer_at_disqualifying_sample_s=max(e['timer_when_reset_s'] or 0. for e in episodes),
        episodes=episodes, blocked_sample_details=blocked_samples),
    joints=joint_stats,
    timing=dict(sample_interval_s=percentiles([b['t']-a['t'] for a,b in zip(samples,samples[1:])]),
        cycle_s=percentiles([d['cycle_s'] for d in dispatch]),
        send_span_s=percentiles([d['send_span_s'] for d in dispatch]),
        feedback_collection_s=percentiles([d['feedback_collection_s'] for d in dispatch]),
        pair_send_skew_s={name:percentiles([d['pair_send_skew_s'][name] for d in dispatch])
                          for name in dispatch[0]['pair_send_skew_s']},
        all_dispatch_complete=all(d['complete'] for d in dispatch),
        basis='Host API send timestamps; not device simultaneous arrival'),
    shutdown=dict(stopped_event=next(r for r in rows if r.get('state') == 'STOPPED'),
        confirmed_disable_keys=sorted([[m['side'],m['motor_id']] for m in disable['records'] if m.get('confirmed')]),
        all_14_disable_confirmed=len({(m['side'],m['motor_id']) for m in disable['records'] if m.get('confirmed')}) == 14,
        after_all_14_disabled=len(after_motors) == 14 and all(m['state'] == 0 for m in after_motors),
        buses=[dict(side=r['side'], online=r['online'], bus_status=r['bus_status']) for r in after['reports']],
        after_j7=[m for m in after_motors if m['motor_id'] == 7]),
    interpretation=[
        'All settling position samples met the 2-degree criterion; failure was the all-14-axis speed gate.',
        'J1/J2 near-zero reported speeds frequently reached +/-0.032967 rad/s, exceeding 0.03 by 0.002967.',
        'J1/J2 held positions moved at most one encoder position count or were unchanged. This is consistent with near-zero velocity quantization/noise; the log alone does not prove the physical source.',
        'J7 reached near its requested angle but did not pass the required uninterrupted 1.5-second stage gate.',
        'Disabled readback proves state=0, not mechanical immobility. J7 moved after disable in the immediate after snapshot.',
    ])
(RUN/'analysis.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(samples=analysis['sample_count'], gate={k:v for k,v in analysis['gate'].items()
                  if k not in ('episodes','blocked_sample_details')},
                  j7=[j for j in joint_stats if j['motor_id'] == 7],
                  held=[{k:j[k] for k in ('side','motor_id','settling_speed_blocked_samples','settling_max_abs_velocity_rad_s','settling_max_abs_error_deg','max_abs_torque_nm')}
                        for j in joint_stats if j['motor_id'] != 7],
                  timing=analysis['timing'],shutdown={k:v for k,v in analysis['shutdown'].items() if k!='after_j7'}),
                 ensure_ascii=False,indent=2))
