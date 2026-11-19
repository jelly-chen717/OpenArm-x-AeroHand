"""Offline dense model check and ideal-feedback replay; no hardware access."""
from pathlib import Path
import copy
import json
import math
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent/'outward_review'
sys.path.insert(0, str(ROOT/'apps'))
from openarm_sequence_dynamics import analyze_dynamics
from openarm_joint_mapping import load_mapping
from openarm_trajectory import preflight
from drive_openarm_from_sim import VirtualTransport, execute_sequence

request = json.loads((HERE/'request.draft.json').read_text(encoding='utf-8'))
assert request['offline_only'] is True and request['expires_at'] == 0.
dynamics = analyze_dynamics(request, samples_per_move=1001)
assert dynamics['collision_contact_sample_count'] == 0
assert dynamics['all_axes_model_torque_within_rated']
with (HERE/'dynamics_dense.json').open('x', encoding='utf-8') as stream:
    json.dump(dynamics, stream, ensure_ascii=False, indent=2)

version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
keys = [(j['side'], j['motor_id']) for j in request['joints']]
initial = dict(zip(keys, request['initial_model_position']))
validation = copy.deepcopy(request)
validation['expires_at'] = time.time()+120
trajectory = preflight(validation, version, mappings, initial, time.time(), set())

class Clock:
    now = 100.
    cancelled = False
    def is_set(self):
        return self.cancelled
    def wait(self, seconds):
        self.now += max(seconds, .000001)
        return self.cancelled

clock = Clock()
original_clock = time.perf_counter
time.perf_counter = lambda: clock.now
bus = VirtualTransport(mappings, initial)
events, held = [], []

def log(row):
    if row['type'] != 'sample':
        events.append(row)
    else:
        assert bus.enabled and len(row['target']) == 14
        if row['phase'] == 'holding_target':
            assert row['target'] == trajectory.points[-1][1]
            held.append(row)
            if len(held) == 5:
                clock.cancelled = True

try:
    result = execute_sequence(trajectory, mappings, bus, log, sequence=request['sequence'],
                              period=.04, max_lateness=.08, max_error=math.radians(5),
                              cancel=clock, hold_after=True)
finally:
    time.perf_counter = original_clock

complete = [r for r in events if r['type'] == 'stage_complete']
assert len(complete) == 6 and all(r['dwell_s'] >= 1. for r in complete)
assert [r['stage'] for r in complete] == [s['name'] for s in request['sequence']['stages']]
assert result == 'CANCELLED' and len(held) == 5 and not bus.enabled
summary = dict(passed=True, feedback_kind='synthetic_virtual_not_hardware',
               stage_completion=complete, nominal_total_s=dynamics['nominal_total_s'],
               virtual_completion_s=complete[-1]['t'], final_enabled_hold_samples=len(held),
               planned_virtual_cancel_confirmed=True, model_sample_count=dynamics['sample_count'],
               model_contact_samples=dynamics['collision_contact_sample_count'],
               hardware_motion_authorized=False, hardware_accessed=False)
with (HERE/'virtual_replay_summary.json').open('x', encoding='utf-8') as stream:
    json.dump(summary, stream, ensure_ascii=False, indent=2)
print(json.dumps({k:summary[k] for k in ('passed','nominal_total_s','virtual_completion_s','model_sample_count','model_contact_samples')}))
for row in json.loads((HERE/'review.json').read_text(encoding='utf-8'))['rows']:
    j = row['joint']
    print(row['stage'], row['target_test_deg'], row['move_s'], row['peak_velocity_deg_s'],
          max(dynamics['all_axes_max_torque_nm'][f'{s}:{j}'] for s in ('left','right')))
