"""Offline audit only: does not enter a transport or open CAN channels."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps'))
from openarm_execution import PcanTransport, rated_limits
from openarm_joint_mapping import load_mapping

version, mappings = load_mapping(ROOT / 'configs/openarm_joint_mapping.json', hardware=True)
keys = tuple((side, motor) for side in ('left', 'right') for motor in range(1, 8))
bus = PcanTransport(mappings, keys, dict(left=0x51, right=0x52))
assert len(bus.joint_settings) == 14
example = json.loads((ROOT / 'configs/openarm_execution_right_wrist.json').read_text('utf-8'))
for row in example:
    for name, value in rated_limits(row['motor_id']).items():
        assert abs(row[name] - value) < 1e-12

historical_path = ROOT / 'logs/openarm/sync/left_four_retry2nm_20261116/settings.json'
historical_bytes = historical_path.read_bytes()
historical = json.loads(historical_bytes)
settings = {(row['side'], row['motor_id']): {k: v for k, v in row.items()
            if k not in ('side', 'motor_id')} for row in historical}
loaded = PcanTransport(mappings, tuple(settings), dict(left=0x51, right=0x52),
                       joint_settings=settings)
for key, values in loaded.joint_settings.items():
    assert values == dict(settings[key], **rated_limits(key[1]))
assert historical_path.read_bytes() == historical_bytes

result = dict(date='2026-11-16', hardware_accessed=False, mapping_version=version,
              limit_profile='rated', voltage_v=24,
              effective_defaults=[dict(side=k[0], motor_id=k[1], **v)
                                  for k, v in bus.joint_settings.items()],
              historical_file=str(historical_path.relative_to(ROOT)),
              historical_sha256=hashlib.sha256(historical_bytes).hexdigest(),
              historical_loaded_effective=[dict(side=k[0], motor_id=k[1], **v)
                                           for k, v in loaded.joint_settings.items()],
              checks=dict(all_14_defaults=True, current_wrist_example=True,
                          actual_historical_settings_overridden=True, historical_unchanged=True))
out = Path(__file__).with_name('verification.json')
out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('PASS: 14 rated defaults, wrist example, actual old four-axis settings; no hardware accessed')
