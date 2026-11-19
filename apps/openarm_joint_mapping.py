"""Validated coordinate mappings; model candidates never authorize hardware execution."""
from dataclasses import dataclass
import json
import math
from pathlib import Path

CONVENTION_VERSION = json.loads((Path(__file__).resolve().parents[1] /
    'configs/openarm_joint_convention.json').read_text(encoding='utf-8'))['version']


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('Expected finite numeric value')
    return float(value)


@dataclass(frozen=True)
class JointMapping:
    side: str
    motor_id: int
    joint_name: str
    direction: int
    scale: float
    p_ref: float
    q_ref: float
    limits: tuple
    coordinate_convention: str = 'official-v1'

    def __post_init__(self):
        if self.side not in ('left', 'right') or type(self.motor_id) is not int or not 1 <= self.motor_id <= 7:
            raise ValueError('Invalid side or arm motor ID')
        if self.joint_name != f'openarm_{self.side}_joint{self.motor_id}':
            raise ValueError('Joint name does not match side and motor ID')
        if type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError('Direction must be integer +/-1')
        if finite(self.scale) <= 0:
            raise ValueError('Scale must be positive')
        finite(self.p_ref)
        finite(self.q_ref)
        if len(self.limits) != 2 or finite(self.limits[0]) >= finite(self.limits[1]):
            raise ValueError('Invalid joint limits')

    def to_model(self, p):
        return finite(self.q_ref + self.direction * self.scale * (finite(p) - self.p_ref))

    def to_motor(self, q):
        return finite(self.p_ref + (finite(q) - self.q_ref) / (self.direction * self.scale))

    def velocity_to_model(self, v):
        return finite(self.direction * self.scale * finite(v))

    def velocity_to_motor(self, v):
        return finite(finite(v) / (self.direction * self.scale))


def load_mapping(path, *, hardware=False, sides=('left', 'right')):
    sides = tuple(sides)
    if not sides or len(set(sides)) != len(sides) or any(s not in ('left', 'right') for s in sides):
        raise ValueError('Invalid selected sides')
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if config.get('schema_version') != 1 or not config.get('version'):
        raise ValueError('Mapping schema/version missing')
    convention = config.get('coordinate_convention', 'official-v1')
    if convention not in ('official-v1', CONVENTION_VERSION):
        raise ValueError('Unsupported mapping coordinate convention')
    rows = config['joints']
    result = {}
    for row in rows:
        if row['side'] not in sides:
            continue
        if hardware:
            if row.get('hardware_verified') is not True or not row.get('evidence'):
                raise ValueError('Hardware mapping not verified: ' + row['joint_name'])
            params = row['hardware_mapping']
        else:
            params = row['model_candidate']
        mapping = JointMapping(row['side'], row['motor_id'], row['joint_name'],
                               limits=tuple(row['limits_rad']), coordinate_convention=convention, **params)
        key = (mapping.side, mapping.motor_id)
        if key in result:
            raise ValueError('Duplicate side/motor ID')
        result[key] = mapping
    if set(result) != {(s, i) for s in sides for i in range(1, 8)}:
        raise ValueError(f'Expected all {7 * len(sides)} selected arm joints')
    return config['version'], result
