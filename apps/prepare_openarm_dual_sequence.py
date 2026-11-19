"""Build an expired MIT review candidate offline; never connects to hardware.

Targets are absolute offsets from a frozen test baseline, not cumulative moves.
Each target can be one angle for both arms or exact left/right angles.
Without --snapshot, logical zero is an explicit model-only assumption.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

from drive_openarm_from_sim import validate_sequence
from openarm_joint_mapping import finite, load_mapping
from openarm_trajectory import preflight

ROOT = Path(__file__).resolve().parents[1]
KEYS = tuple((side, joint) for joint in range(1, 8) for side in ('left', 'right'))
GAINS = {1: (300., 5.), 2: (300., 5.), 3: (80., 3.), 4: (200., 3.),
         5: (60., 2.), 6: (60., 2.), 7: (60., 2.)}
# Planning speeds, not motor ratings. Shoulders use the slower prior review range.
SPEED_DEG_S = {1: 20., 2: 20., 3: 30., 4: 30., 5: 45., 6: 45., 7: 45.}
ACCEL_RAD_S2 = {1: .35, 2: .35, 3: .7, 4: .7, 5: 1., 6: 1., 7: 1.}
OUTPUT_FILES = {'request': 'request.draft.json', 'settings': 'settings.json',
                'review': 'review.json', 'dynamics': 'dynamics.json'}


def motion_duration(joint, displacement_deg):
    distance = abs(finite(displacement_deg))
    if distance == 0:
        raise ValueError('Each stage must move its joint')
    minimum = max(.6, 1.875 * distance / SPEED_DEG_S[joint],
                  math.sqrt((10 / math.sqrt(3)) * math.radians(distance) / ACCEL_RAD_S2[joint]))
    return math.ceil(minimum * 10 - 1e-12) / 10


def bilateral_targets(value):
    if isinstance(value, dict):
        if set(value) != {'left', 'right'}:
            raise ValueError('A bilateral target requires exactly left and right')
        return {side: finite(value[side]) for side in ('left', 'right')}
    target = finite(value)
    return dict(left=target, right=target)


def scalar_if_equal(values):
    """Keep legacy scalar reports for matching arms; never hide differing signs."""
    return values['left'] if values['left'] == values['right'] else values.copy()


def read_baseline(path, mappings):
    """Read a saved disabled snapshot; this function does not obtain live feedback."""
    path = Path(path)
    reports = [json.loads(line) for line in path.read_text(encoding='utf-8-sig').splitlines() if line.strip()]
    if len(reports) != 2 or {r['side'] for r in reports} != {'left', 'right'}:
        raise ValueError('Snapshot requires exactly one report per arm')
    origins, timestamps = {}, []
    for report in reports:
        if report['online'] != 7 or report['bus_status'] != '0x0' or len(report['motors']) != 7:
            raise ValueError('Snapshot requires seven healthy motors per arm')
        timestamps.append(finite(report['time_unix']))
        for motor in report['motors']:
            key = report['side'], motor['motor_id']
            if type(key[1]) is not int or key not in mappings or key in origins or motor['state'] != 0:
                raise ValueError('Snapshot requires 14 unique disabled motors')
            origins[key] = finite(motor['position_rad'])
    if set(origins) != set(KEYS):
        raise ValueError('Incomplete snapshot')
    logical = {key: mappings[key].to_model(origins[key]) for key in KEYS}
    return logical, dict(kind='saved_feedback_not_live', source=str(path.resolve()),
                         sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                         snapshot_time_unix=max(timestamps),
                         zero_policy='Test-relative reference only; persistent calibration is unchanged')


def build_package(plan, version, mappings, baseline=None, provenance=None):
    if plan.get('schema_version') != 1 or plan.get('target_mode') != 'absolute_from_test_zero':
        raise ValueError('Expected schema 1 with explicit absolute_from_test_zero targets')
    if not isinstance(plan.get('name'), str) or not plan['name'].strip():
        raise ValueError('Plan name required')
    if not isinstance(plan.get('steps'), list) or not plan['steps']:
        raise ValueError('Nonempty ordered steps required')
    assumed_zero = baseline is None
    baseline = dict.fromkeys(KEYS, 0.) if assumed_zero else dict(baseline)
    if set(baseline) != set(KEYS):
        raise ValueError('Baseline requires exactly 14 axes')
    initial = [finite(baseline[k]) for k in KEYS]
    provenance = provenance or dict(kind='assumed_logical_zero_not_measured' if assumed_zero
                                   else 'caller_supplied_baseline_not_live')
    q, offsets = initial.copy(), dict.fromkeys(KEYS, 0.)
    points, stages, rows = [dict(t=0., q=q.copy())], [], []
    elapsed = 0.
    tolerance = math.radians(finite(plan['arrival_tolerance_deg']))
    for index, step in enumerate(plan['steps'], 1):
        joint = step['joint']
        if type(joint) is not int or joint not in range(1, 8):
            raise ValueError('Step joint must be an integer from 1 to 7')
        targets = bilateral_targets(step['target_deg'])
        starts = {side: offsets[side, joint] for side in ('left', 'right')}
        displacements = {side: targets[side] - starts[side] for side in ('left', 'right')}
        # Same joint limits apply to each side. The larger displacement sets
        # the shared duration; a stationary side keeps its current target.
        distance = max(abs(value) for value in displacements.values())
        duration = motion_duration(joint, distance)
        for i, key in enumerate(KEYS):
            if key[1] == joint:
                q[i] = initial[i] + math.radians(targets[key[0]])
        name = f'{index:02d}_J{joint}'  # repeated joint names must remain distinct stages
        elapsed = round(elapsed + duration, 8)
        points.append(dict(t=elapsed, q=q.copy()))
        stages.append(dict(name=name, endpoint_index=index))
        peak_velocities = {side: 1.875 * abs(value) / duration for side, value in displacements.items()}
        peak_accelerations = {side: (10 / math.sqrt(3)) * abs(math.radians(value)) / duration**2
                              for side, value in displacements.items()}
        rows.append(dict(stage=name, joint=joint, start_test_deg=scalar_if_equal(starts),
                         target_test_deg=scalar_if_equal(targets), displacement_deg=scalar_if_equal(displacements),
                         start_test_deg_by_side=starts, target_test_deg_by_side=targets,
                         displacement_deg_by_side=displacements, move_s=duration,
                         dwell_s=finite(plan['dwell_s']),
                         peak_velocity_deg_s=max(peak_velocities.values()),
                         peak_acceleration_rad_s2=max(peak_accelerations.values()),
                         peak_velocity_deg_s_by_side=peak_velocities,
                         peak_acceleration_rad_s2_by_side=peak_accelerations,
                         raw_motor_displacement_deg={s: math.degrees(mappings[s, joint].velocity_to_motor(math.radians(displacements[s]))) for s in ('left', 'right')}))
        offsets.update({(side, joint): target for side, target in targets.items()})
    sequence = dict(schema_version=1, dwell_s=finite(plan['dwell_s']),
                    settle_timeout_s=finite(plan['settle_timeout_s']),
                    settle_speed_rad_s=finite(plan['settle_speed_rad_s']),
                    endpoint_tolerances_rad=[tolerance] * 14, stages=stages)
    request = dict(request_id=plan['name'] + '-NOT-AUTHORIZED', mapping_version=version,
                   expires_at=0., offline_only=True,
                   authorization_status='AWAITING_FIXED_ZERO_AND_USER_REVIEW',
                   source='Offline review candidate; never refresh expiry before final user authorization.',
                   joints=[dict(side=s, motor_id=j) for s, j in KEYS],
                   initial_model_position=initial,
                   initial_motor_position=[mappings[k].to_motor(baseline[k]) for k in KEYS],
                   limits=dict(velocity_rad_s=math.radians(max(SPEED_DEG_S.values())),
                               acceleration_rad_s2=max(ACCEL_RAD_S2.values()), start_error_rad=.01),
                   points=points, sequence=sequence)
    violations, margins = [], []
    for i, key in enumerate(KEYS):
        lower, upper = mappings[key].limits
        values = [p['q'][i] for p in points]
        margins.append(dict(side=key[0], motor_id=key[1],
                            min_margin_deg=math.degrees(min(min(values)-lower, upper-max(values))),
                            lower_deg=math.degrees(lower), upper_deg=math.degrees(upper)))
        for point_index, value in enumerate(values):
            if not lower <= value <= upper:
                violations.append(dict(side=key[0], motor_id=key[1], point_index=point_index,
                                       target_deg=math.degrees(value), lower_deg=math.degrees(lower), upper_deg=math.degrees(upper)))
    # These are intervals of possible logical baseline angles, not new zero calibrations.
    allowed_baselines = []
    for i, key in enumerate(KEYS):
        lo, hi = mappings[key].limits
        relative = [p['q'][i]-initial[i] for p in points]
        allowed_baselines.append(dict(side=key[0], motor_id=key[1],
                                      minimum_deg=math.degrees(lo-min(relative)),
                                      maximum_deg=math.degrees(hi-max(relative))))
    settings = []
    # Import only for the shared rating constants and static transport validation;
    # neither constructor nor validate_trajectory opens the PCAN device.
    from openarm_execution import PcanTransport, rated_limits
    for i, (side, joint) in enumerate(KEYS):
        travel = max(abs(mappings[side, joint].to_motor(p['q'][i]) - request['initial_motor_position'][i]) for p in points)
        kp, kd = GAINS[joint]
        settings.append(dict(side=side, motor_id=joint, kp=kp, kd=kd,
                             max_travel=travel + math.radians(5), **rated_limits(joint),
                             max_temperature=50., max_age=.15))
    errors = []
    try:
        # Only this private in-memory clone has a synthetic validity window.
        validation = copy.deepcopy(request)
        validation['expires_at'] = 1.
        trajectory = preflight(validation, version, mappings, baseline, 0., set())
        validate_sequence(trajectory, sequence)
        joint_settings = {(r['side'], r['motor_id']): {k: v for k, v in r.items() if k not in ('side', 'motor_id')} for r in settings}
        transport = PcanTransport(mappings, KEYS, dict(left=0x51, right=0x52), joint_settings=joint_settings)
        transport.origins = dict(zip(KEYS, request['initial_motor_position']))
        transport.validate_trajectory(trajectory)
    except (ValueError, KeyError) as exc:
        errors.append(str(exc))
    for name in ('period_s', 'max_lateness_s', 'max_tracking_error_deg'):
        if finite(plan[name]) <= 0:
            raise ValueError(f'{name} must be positive')
    report = dict(status='BLOCKED_OFFLINE_VALIDATION' if errors else 'OFFLINE_CANDIDATE_AWAITING_REVIEW',
                  hardware_accessed=False, hardware_execution_allowed=False,
                  baseline=provenance, target_mode=plan['target_mode'], rows=rows,
                  move_total_s=elapsed, dwell_total_s=sequence['dwell_s'] * len(stages),
                  nominal_total_s=round(elapsed + sequence['dwell_s'] * len(stages), 8),
                  extra_time='Measured settling and control-period alignment',
                  validation_errors=errors, limit_violations=violations, joint_limit_margins=margins,
                  allowed_logical_baseline_intervals=allowed_baselines,
                  settings=dict(period_s=plan['period_s'], max_lateness_s=plan['max_lateness_s'],
                                max_tracking_error_rad=math.radians(plan['max_tracking_error_deg']),
                                hold_after=True, batch_mit=True, all_14_axes_commanded=True),
                  pending=['Wiring completion and current disabled 14-axis snapshot',
                           'Recompute using frozen measured baseline and review model torque/contact results',
                           'User approval of the final ordered target, timing and load table',
                           'Dedicated hardware preflight with unchanged approved targets and baseline'])
    return dict(request=request, settings=settings, review=report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=ROOT/'configs/openarm_dual_mit_test_20261116.json')
    parser.add_argument('--mapping', type=Path, default=ROOT/'configs/openarm_joint_mapping.json')
    parser.add_argument('--snapshot', type=Path, help='Saved status JSONL; never queries motors')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dynamics', action='store_true', help='Estimate torque with the local MuJoCo model')
    args = parser.parse_args(argv)
    existing = [str(args.output/name) for name in OUTPUT_FILES.values() if (args.output/name).exists()]
    if existing:
        raise FileExistsError('Use a new output directory; refusing to mix or overwrite review evidence: '
                              + ', '.join(existing))
    plan = json.loads(args.plan.read_text(encoding='utf-8-sig'))
    version, mappings = load_mapping(args.mapping, hardware=True)
    baseline, provenance = read_baseline(args.snapshot, mappings) if args.snapshot else (None, None)
    package = build_package(plan, version, mappings, baseline, provenance)
    if args.dynamics:
        from openarm_sequence_dynamics import analyze_dynamics
        package['dynamics'] = analyze_dynamics(package['request'])
        dynamics = package['dynamics']
        if dynamics['status'] != 'MODEL_ESTIMATE' or not dynamics['all_axes_model_torque_within_rated'] or dynamics['collision_contact_sample_count']:
            package['review']['status'] = 'BLOCKED_MODEL_REVIEW'
            package['review']['validation_errors'].append('Model limits, torque or contacts require review')
    serialized = {OUTPUT_FILES[key]: json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
                  for key, value in package.items()}
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, contents in serialized.items():
        with (args.output/filename).open('x', encoding='utf-8') as stream:
            stream.write(contents)
    print(json.dumps({k: package['review'][k] for k in ('status', 'hardware_accessed', 'nominal_total_s', 'validation_errors')}, ensure_ascii=False))
    return 2 if package['review']['validation_errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
