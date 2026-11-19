"""Reparameterize V1 joint coordinates without changing physical geometry."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

CONFIG = Path(__file__).resolve().parents[3] / 'configs/openarm_joint_convention.json'
_config = json.loads(CONFIG.read_text(encoding='utf-8'))
CONVENTION_VERSION = _config['version']
SIGNS = _config['official_to_logical_signs']
if set(SIGNS) != {'left', 'right'} or any(
        len(v) != 7 or any(type(s) is not int or s not in (-1, 1) for s in v)
        for v in SIGNS.values()):
    raise ValueError('Joint convention requires seven +/-1 signs per arm')
JOINT_SIGNS = {f'openarm_{side}_joint{i}': sign
               for side, signs in SIGNS.items() for i, sign in enumerate(signs, 1)}


def from_official(side, values):
    """Convert positions, velocities or accelerations; also its own inverse."""
    if side not in SIGNS or len(values) != 7:
        raise ValueError('Expected one named arm with seven coordinates')
    return [sign * value for sign, value in zip(SIGNS[side], values)]


def apply_v1_convention(root):
    """Transform a private XML tree before compilation; never edit vendor files."""
    marker = root.find("custom/text[@name='openarm_joint_convention']")
    if marker is not None:
        if marker.get('data') != CONVENTION_VERSION:
            raise ValueError('Unknown model coordinate convention')
        return root
    joints = list(root.findall('worldbody//joint'))
    named = {j.get('name'): j for j in joints}
    if not set(JOINT_SIGNS) <= set(named):
        raise ValueError('V1 convention requires all fourteen named arm joints')
    # V1 has only scalar hinges/slides; keyframe ordering follows this traversal.
    if any(j.get('type', 'hinge') not in ('hinge', 'slide') for j in joints):
        raise ValueError('V1 coordinate adapter expects scalar joints')
    for name, sign in JOINT_SIGNS.items():
        if sign == 1:
            continue
        joint = named[name]
        joint.set('axis', ' '.join(str(-float(v)) for v in joint.get('axis').split()))
        joint.set('range', ' '.join(str(v) for v in sorted(-float(v) for v in joint.get('range').split())))
        for field in ('ref', 'springref'):
            if field in joint.attrib:
                joint.set(field, str(-float(joint.get(field))))
    actuators = list(root.find('actuator')) if root.find('actuator') is not None else []
    for actuator in actuators:
        if JOINT_SIGNS.get(actuator.get('joint'), 1) == -1:
            for field in ('ctrlrange', 'forcerange'):
                if field in actuator.attrib:
                    actuator.set(field, ' '.join(str(v) for v in sorted(-float(v) for v in actuator.get(field).split())))
    for key in root.findall('keyframe/key'):
        for field, signs in (
            ('qpos', [JOINT_SIGNS.get(j.get('name'), 1) for j in joints]),
            ('qvel', [JOINT_SIGNS.get(j.get('name'), 1) for j in joints]),
            ('ctrl', [JOINT_SIGNS.get(a.get('joint'), 1) for a in actuators]),
        ):
            if field in key.attrib:
                values = list(map(float, key.get(field).split()))
                if len(values) != len(signs):
                    raise ValueError('V1 keyframe dimension mismatch')
                key.set(field, ' '.join(str(s*v) for s, v in zip(signs, values)))
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'text', name='openarm_joint_convention', data=CONVENTION_VERSION)
    return root
