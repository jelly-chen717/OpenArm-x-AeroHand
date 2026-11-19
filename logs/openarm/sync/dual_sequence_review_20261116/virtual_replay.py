"""Replay the exact reviewed seven-stage request without hardware imports/connection."""
from pathlib import Path
import json
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'apps'))
from drive_openarm_from_sim import VirtualTransport,execute_sequence
from openarm_joint_mapping import load_mapping
from openarm_trajectory import preflight

request = json.loads((HERE/'request.draft.json').read_text(encoding='utf-8'))
request['expires_at'] = time.time()+300
version,mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json',hardware=True)
keys = [(j['side'],j['motor_id']) for j in request['joints']]
initial = dict(zip(keys,request['initial_model_position']))
trajectory = preflight(request,version,mappings,initial,time.time(),set())
bus = VirtualTransport(mappings,initial)
cancel = threading.Event()
events = []
hold_samples = 0
with (HERE/'virtual_replay.jsonl').open('w',encoding='utf-8') as stream:
    def log(row):
        global hold_samples
        row['feedback_kind'] = 'synthetic_virtual'
        stream.write(json.dumps(row)+'\n')
        if row['type'] != 'sample':
            events.append(row)
        if row.get('phase') == 'holding_target':
            hold_samples += 1
            if hold_samples == 5:
                cancel.set()
    outcome = execute_sequence(trajectory,mappings,bus,log,sequence=request['sequence'],
                               period=.04,max_lateness=.08,max_error=.08726646259971647,
                               cancel=cancel,hold_after=True)
stages = [r for r in events if r['type'] == 'stage_complete']
assert [r['stage'] for r in stages] == ['J7','J6','J5','J4','J3','J2','J1']
assert all(r['dwell_s'] >= 1.5 for r in stages)
assert hold_samples == 5 and outcome == 'CANCELLED' and not bus.enabled
summary = dict(passed=True,kind='synthetic_virtual_not_hardware',stage_completion=stages,
               nominal_total_s=19.7,actual_virtual_completion_s=stages[-1]['t'],
               holding_target_samples=hold_samples,planned_virtual_cancel_confirmed=True,
               note='Exact request timing and mapping replay; no physical tracking/torque validation.')
(HERE/'virtual_replay_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps({k:summary[k] for k in ('passed','kind','actual_virtual_completion_s','holding_target_samples')}))
