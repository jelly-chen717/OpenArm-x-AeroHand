"""Export an offline POS_VEL review plan from the current piano simulator."""

import argparse
from hashlib import sha256
import json
from math import isfinite, sqrt
from pathlib import Path

import mujoco
import numpy as np
from openarm_control.config import ARM_JOINT_VELOCITY_LIMITS_RAD_S
from openarm_app.joint_convention import CONVENTION_VERSION
from openarm_app.sim_server import ArmServerState

from openarm_execution import rated_limits
from openarm_joint_mapping import load_mapping


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'openarm/skills/paino_skill/scene.json'
MAPPING = ROOT / 'configs/openarm_joint_mapping.json'
MICROSECONDS = 1_000_000
QUINTIC_SPEED = 1.875
QUINTIC_ACCELERATION = 10 / sqrt(3)


def positive(value):
    value = float(value)
    if not isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('Expected a finite positive number')
    return value


def source_record(path):
    return {'path': path.resolve().as_posix(), 'sha256': sha256(path.read_bytes()).hexdigest()}


def catalog_positions(engine, positions):
    data = mujoco.MjData(engine.model)
    result = []
    for (side, index), position in sorted(positions.items()):
        data.qpos[:] = engine.home_qpos
        q = position['q']
        limits = engine.arms[side].limits
        data.qpos[engine.scene_qindices[side]] = q
        mujoco.mj_forward(engine.model, data)
        palm = data.body(f'aero_{side}_palm')
        result.append({
            'side': side, 'index': index, 'q_model_rad': q.tolist(),
            'joint_limit_margin_rad': np.minimum(q - limits[:, 0], limits[:, 1] - q).tolist(),
            'palm_world_m': palm.xpos.tolist(), 'palm_world_quat_wxyz': palm.xquat.tolist(),
            'sim_finger_press_amount': position['strokes'],
        })
    return result


def motion_window(actions, i, side):
    """Find the nearest constraints on a move if the hand timeline stays fixed."""
    previous = {}
    for action in reversed(actions[:i]):
        if action['kind'] == 'move' and action['side'] == side:
            previous.setdefault('move', (action['time_us'] + action['duration_us'], action['line']))
        elif action['kind'] == 'hand' and any(f['side'] == side for f in action['fingers']):
            previous.setdefault('hand', (action['time_us'], action['line']))
        if len(previous) == 2:
            break
    earliest, previous_line = max(previous.values(), default=(0, None), key=lambda item: item[0])

    for action in actions[i+1:]:
        if action['kind'] == 'move':
            constrains = action['side'] == side
        else:
            constrains = action['direction'] == 'D' and any(f['side'] == side for f in action['fingers'])
        if constrains:
            return earliest, previous_line, action['time_us'], action['line'], action['kind']
    return earliest, previous_line, None, None, None


def timing_for_move(action, endpoints, rated, sdk, acceleration, overhead, candidate_rate):
    duration = action['duration_us'] / MICROSECONDS
    delta = np.abs(endpoints['q1'] - endpoints['q0'])
    peak_velocity = QUINTIC_SPEED * delta / duration
    peak_acceleration = QUINTIC_ACCELERATION * delta / duration**2
    rated_time = QUINTIC_SPEED * delta / rated
    sdk_time = QUINTIC_SPEED * delta / sdk
    min_time = np.maximum.reduce((rated_time, sdk_time, np.sqrt(QUINTIC_ACCELERATION * delta / acceleration)))
    required_with_reserve = float(np.max(min_time)) + overhead
    return {
        'peak_velocity_1x_rad_s': peak_velocity.tolist(),
        'peak_acceleration_1x_rad_s2': peak_acceleration.tolist(),
        'rated_exceeded_joint_ids': (np.flatnonzero(peak_velocity > rated) + 1).tolist(),
        'sdk_cap_exceeded_joint_ids': (np.flatnonzero(peak_velocity > sdk) + 1).tolist(),
        'rated_speed_only_bottleneck_joint': int(np.argmax(rated_time)) + 1,
        'rated_speed_only_min_s': float(np.max(rated_time)),
        'sdk_speed_only_min_s': float(np.max(sdk_time)),
        'trial_motion_min_s': float(np.max(min_time)),
        'trial_total_min_s': required_with_reserve,
        'trial_candidate_slack_s': duration / candidate_rate - required_with_reserve,
    }


def finger_timing(events):
    down, up = {}, {}
    shortest_hold, shortest_restrike = None, None
    for event in events:
        stamp = event['time_us']
        for finger in event['fingers']:
            key = finger['side'], finger['finger']
            if event['direction'] == 'D':
                if key in up:
                    gap = stamp - up[key]
                    shortest_restrike = gap if shortest_restrike is None else min(shortest_restrike, gap)
                down[key] = stamp
            elif key in down:
                duration = stamp - down.pop(key)
                shortest_hold = duration if shortest_hold is None else min(shortest_hold, duration)
                up[key] = stamp
    return {'shortest_press_us': shortest_hold, 'shortest_same_finger_restrike_gap_us': shortest_restrike}


def build_review(args, overrides=()):
    engine = ArmServerState(scene='piano')
    for position in overrides:
        side, index = position['side'], position['index']
        q = np.array(position['q_model_rad'], dtype=float)
        engine.score.position_cache[side, index] = engine.score._validate_position(side, index, q)
    checked = engine.score.prepare(args.score.read_text(encoding='utf-8-sig'), 4, automatic_motion=True)
    prepared = engine.score.prepared
    mapping_version, mappings = load_mapping(MAPPING)
    if any(mapping.coordinate_convention != CONVENTION_VERSION for mapping in mappings.values()):
        raise ValueError('Trajectory and hardware mapping use different model coordinates')

    sdk = np.asarray(ARM_JOINT_VELOCITY_LIMITS_RAD_S)
    specs = [rated_limits(i) for i in range(1, 8)]
    rated = np.array([spec['max_speed'] for spec in specs])
    overhead = args.release_s + args.settle_s + args.press_s
    rows, fingers = [], []
    previous_index = dict.fromkeys(engine.SIDES)
    for i, action in enumerate(checked['actions']):
        if action['kind'] == 'hand':
            fingers.append(action)
            continue
        side = action['side']
        endpoints = prepared['plans'][i]
        limits = engine.arms[side].limits
        mapping_limits = [mappings[side, joint].limits for joint in range(1, 8)]
        if not np.allclose(limits, mapping_limits, atol=1e-6, rtol=0):
            raise ValueError(f'{side} model and mapping limits differ')
        q0, q1 = endpoints['q0'], endpoints['q1']
        if np.any(np.minimum(q0, q1) < limits[:, 0]) or np.any(np.maximum(q0, q1) > limits[:, 1]):
            raise ValueError(f'Joint limit violated at score line {action["line"]}')
        earliest, previous_line, deadline, next_line, next_kind = motion_window(checked['actions'], i, side)
        planned_start = round(action['time_us'] / args.candidate_rate + args.release_s * MICROSECONDS)
        planned_end = round((action['time_us'] + action['duration_us']) / args.candidate_rate
                            - (args.settle_s + args.press_s) * MICROSECONDS)
        rows.append({
            **action, 'from_index': previous_index[side],
            'q0_model_rad': q0.tolist(), 'q1_model_rad': q1.tolist(),
            'earliest_start_us': earliest, 'previous_line': previous_line,
            'next_constraint_us': deadline, 'next_constraint_line': next_line, 'next_constraint_kind': next_kind,
            'trial_candidate_move_start_us': planned_start,
            'trial_candidate_move_end_us': planned_end,
            **timing_for_move(action, endpoints, rated, sdk, args.acceleration, overhead, args.candidate_rate),
        })
        previous_index[side] = action['index']
    if not rows:
        raise ValueError('No arm moves to review')

    def fastest_rate(minimum_key):
        rates = (row['duration_us'] / MICROSECONDS / row[minimum_key]
                 for row in rows if row[minimum_key] > 0)
        return min(rates, default=None)

    return {
        'schema': 'piano_posvel_offline_review_v1', 'offline_only': True, 'expires_at': 0,
        'requested_control_mode': 'POS_VEL', 'ctrl_mode': 2,
        'sources': {'score': source_record(args.score), 'piano_scene': source_record(SCENE),
                    'joint_mapping': source_record(MAPPING), 'mapping_version': mapping_version,
                    'coordinate_convention': CONVENTION_VERSION},
        'duration_us': checked['duration_us'], 'press_count': checked['note_count'],
        'trial_duration_us': round(checked['duration_us'] / args.candidate_rate),
        'model_joint_limits_rad': {side: engine.arms[side].limits.tolist() for side in engine.SIDES},
        'rated_speed_24v_rad_s': rated.tolist(), 'rated_torque_reference_nm': [row['max_torque'] for row in specs],
        'optional_sdk_speed_cap_rad_s': sdk.tolist(),
        'trial_budget_unmeasured': {'candidate_rate': args.candidate_rate, 'acceleration_rad_s2': args.acceleration,
                                    'release_s': args.release_s, 'settle_s': args.settle_s, 'press_s': args.press_s},
        'rated_speed_only_max_rate': fastest_rate('rated_speed_only_min_s'),
        'sdk_speed_only_max_rate': fastest_rate('sdk_speed_only_min_s'),
        'trial_max_rate': fastest_rate('trial_total_min_s'),
        'candidate_min_slack_s': min(row['trial_candidate_slack_s'] for row in rows),
        'sample_timing': finger_timing(fingers),
        'hand_positions': catalog_positions(engine, prepared['positions']),
        'moves': rows, 'finger_events': fingers,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('score', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate-poses', type=Path,
                        help='Optional JSON hand positions to reverify alongside the original simulation')
    parser.add_argument('--candidate-rate', type=positive, default=.25)
    parser.add_argument('--acceleration', type=positive, default=20., help='Unmeasured trial value in rad/s²')
    parser.add_argument('--release-s', type=positive, default=.05)
    parser.add_argument('--settle-s', type=positive, default=.05)
    parser.add_argument('--press-s', type=positive, default=.05)
    args = parser.parse_args()
    if args.output.resolve() in {args.score.resolve(), SCENE.resolve(), MAPPING.resolve()}:
        parser.error('Output would overwrite an input')
    review = build_review(args)
    if args.candidate_poses:
        source = json.loads(args.candidate_poses.read_text(encoding='utf-8'))
        if 'candidate' in source:
            source = source['candidate']
        candidate = build_review(args, source['hand_positions'])
        if candidate['finger_events'] != review['finger_events']:
            raise RuntimeError('Pose overrides changed the score timeline')
        review['candidate'] = {key: candidate[key] for key in (
            'hand_positions', 'moves', 'rated_speed_only_max_rate', 'sdk_speed_only_max_rate',
            'trial_max_rate', 'candidate_min_slack_s',
        )}
    args.output.write_text(json.dumps(review, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(f'{len(review["moves"])} moves, {len(review["hand_positions"])} hand positions, {review["press_count"]} presses')
    rated_rate = review['rated_speed_only_max_rate']
    if rated_rate is not None:
        print(f'Rated speed-only rate: {rated_rate:.4f}x')
    print(f'Trial rate with unmeasured timing budget: {review["trial_max_rate"]:.4f}x')


if __name__ == '__main__':
    main()
