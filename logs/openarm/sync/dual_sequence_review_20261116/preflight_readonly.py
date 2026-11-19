"""Validate the reviewed request against live disabled hardware; never enable."""
from dataclasses import asdict
from pathlib import Path
import json
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'apps'))
from openarm_joint_mapping import load_mapping
from openarm_execution import PcanTransport
from openarm_trajectory import preflight
from drive_openarm_from_sim import validate_sequence

request = json.loads((HERE/'request.draft.json').read_text(encoding='utf-8'))
request['expires_at'] = time.time()+300  # memory only; disk draft remains expired
version,mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json',hardware=True)
keys = [(j['side'],j['motor_id']) for j in request['joints']]
settings = {(r['side'],r['motor_id']):{k:v for k,v in r.items() if k not in ('side','motor_id')}
            for r in json.loads((HERE/'settings.json').read_text(encoding='utf-8'))}
with PcanTransport(mappings,keys,dict(left=0x51,right=0x52),joint_settings=settings,batch_mit=True) as bus:
    initial = bus.read(keys)
    deviations = {f'{s}:{j}':initial[s,j]-q for (s,j),q in zip(keys,request['initial_model_position'])}
    check = dict(time_unix=time.time(),before=bus.before,no_enable=True,no_MIT_commands=True,
                 start_deviation_rad=deviations,start_deviation_deg={k:v*180/3.141592653589793 for k,v in deviations.items()})
    (HERE/'latest_start_check.json').write_text(json.dumps(check,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(start_deviation_deg=check['start_deviation_deg'])))
    trajectory = preflight(request,version,mappings,initial,time.time(),set())
    validate_sequence(trajectory,request['sequence'])
    bus.validate_trajectory(trajectory)
    cycles = []
    for n in range(30):
        began = time.perf_counter()
        for key in keys:
            state = bus.arms[key[0]].query_motor(key[1])
            bus.check(key,state,expected_state=0)
            bus.states[key] = state
        cycles.append(time.perf_counter()-began)
        time.sleep(.02)
    after = bus.require_disabled()
    result = dict(time_unix=time.time(),kind='hardware_preflight_read_only',passed=True,
                  no_enable=True,no_MIT_commands=True,no_mode_or_calibration_changes=True,
                  before=bus.before,after=after,all_parameters_verified=True,
                  start_deviation_rad={f'{s}:{j}':initial[s,j]-q for (s,j),q in zip(keys,request['initial_model_position'])},
                  effective_settings=[dict(side=k[0],motor_id=k[1],**v) for k,v in bus.joint_settings.items()],
                  batch_feedback_cycle_median_s=statistics.median(cycles),batch_feedback_cycle_max_s=max(cycles),
                  batch_feedback_cycle_p95_s=sorted(cycles)[int(.95*(len(cycles)-1))],
                  note='Read-only feedback timing does not measure actual 14-command batch transmission or enabled tracking.')
(HERE/'hardware_preflight_readonly.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('passed','no_enable','no_MIT_commands','batch_feedback_cycle_median_s','batch_feedback_cycle_max_s')}))
