"""Read-only hardware preparation. Never enables or commands a motor."""
import json
import math
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'apps'))
from openarm_joint_mapping import load_mapping
from openarm_execution import PcanTransport
from openarm_trajectory import preflight

OUT = Path(__file__).resolve().parent
version, mappings = load_mapping(ROOT / 'configs/openarm_joint_mapping.json', hardware=True)
keys = (('left', 4), ('left', 5), ('left', 6), ('left', 7))
settings = {}
for row in json.loads((OUT / 'settings.json').read_text(encoding='utf-8')):
    settings[(row['side'], row['motor_id'])] = {k: v for k, v in row.items() if k not in ('side', 'motor_id')}
with PcanTransport(mappings, keys, dict(left=0x51, right=0x52), joint_settings=settings) as transport:
    initial = transport.read(keys)
    q = [initial[k] for k in keys]
    points = [dict(t=0, q=q.copy())]
    t = 0
    for index, degrees, move, hold in ((3, 25, 4, 2), (2, 25, 4, 3), (1, 45, 7, 3), (0, 15, 4, 3)):
        q[index] += math.radians(degrees)
        t += move
        points.append(dict(t=t, q=q.copy()))
        t += hold
        points.append(dict(t=t, q=q.copy()))
    request = dict(request_id='left-replan-' + uuid.uuid4().hex, mapping_version=version,
                   expires_at=time.time()+300,
                   joints=[dict(side=k[0], motor_id=k[1]) for k in keys],
                   initial_model_position=[initial[k] for k in keys],
                   limits=dict(velocity_rad_s=.22, acceleration_rad_s2=.17, start_error_rad=.005),
                   points=points, source='user-approved-left-J7plus25-J6plus25-J5plus45-J4plus15-30s-then-hold')
    trajectory = preflight(request, version, mappings, initial, time.time(), set())
    transport.validate_trajectory(trajectory)
    stamp = str(time.time_ns())
    (OUT / ('request_' + stamp + '.json')).write_text(json.dumps(request, indent=2), encoding='utf-8')
    (OUT / 'request.json').write_text(json.dumps(request, indent=2), encoding='utf-8')
    evidence = dict(action='query_and_preflight_only', enabled=False, before=transport.before,
                    request_id=request['request_id'], duration_s=trajectory.duration,
                    start_deg=[math.degrees(initial[k]) for k in keys],
                    target_deg=[math.degrees(v) for v in q],
                    settings=[dict(side=k[0], motor_id=k[1], **v) for k, v in settings.items()])
    (OUT / ('preflight_' + stamp + '.json')).write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in evidence.items() if k not in ('before', 'settings')}))
