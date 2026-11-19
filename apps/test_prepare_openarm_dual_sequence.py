"""Offline candidate preparation and ideal-feedback execution semantics."""
import copy
import json
import math
import time

import pytest

from drive_openarm_from_sim import VirtualTransport, execute_sequence
from openarm_joint_mapping import load_mapping
from openarm_trajectory import preflight
from prepare_openarm_dual_sequence import (
    ACCEL_RAD_S2, KEYS, ROOT, SPEED_DEG_S, build_package, main, read_baseline,
)


@pytest.fixture
def prepared(monkeypatch):
    import openarm_execution
    def no_device(*args, **kwargs):
        raise AssertionError('Offline preparation must never enter a hardware transport')
    monkeypatch.setattr(openarm_execution.PcanTransport, '__enter__', no_device)
    plan = json.loads((ROOT/'configs/openarm_dual_mit_test_20261116.json').read_text(encoding='utf-8'))
    # Preserve scalar-target compatibility cases independently of the current
    # user's mirrored outward plan, tested explicitly below.
    plan['steps'][0]['target_deg'] = -88.
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    return plan, version, mappings, build_package(plan, version, mappings)


def test_current_plan_implements_user_outward_left_counterclockwise_right_clockwise():
    plan = json.loads((ROOT/'configs/openarm_dual_mit_test_20261116.json').read_text(encoding='utf-8'))
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    package = build_package(plan, version, mappings)
    rows = package['review']['rows']
    assert [r['joint'] for r in rows] == [5, 7, 5, 4, 1, 2]
    assert rows[0]['displacement_deg_by_side'] == {'left': -88., 'right': 88.}
    assert rows[1]['target_test_deg_by_side'] == {'left': 88., 'right': 88.}
    assert rows[2]['displacement_deg_by_side'] == {'left': 88., 'right': -88.}
    assert rows[2]['target_test_deg_by_side'] == {'left': 0., 'right': 0.}


def test_repeated_j5_returns_to_test_zero_without_cumulative_target_error(prepared):
    plan, version, mappings, package = prepared
    request, review = package['request'], package['review']
    assert len(request['joints']) == len(set(KEYS)) == len(package['settings']) == 14
    assert [r['joint'] for r in review['rows']] == [5, 7, 5, 4, 1, 2]
    assert [r['displacement_deg'] for r in review['rows']] == [-88., 88., 88., 30., -15., -15.]
    assert request['points'][1]['q'][8:10] == pytest.approx([math.radians(-88)]*2)
    assert request['points'][3]['q'][8:10] == [0., 0.]
    assert all(p['q'][10:12] == [0., 0.] for p in request['points'])  # J6 never moves
    assert len({s['name'] for s in request['sequence']['stages']}) == 6
    assert review['validation_errors'] == []


def test_quintic_timing_and_raw_directions(prepared):
    _, _, _, package = prepared
    rows = package['review']['rows']
    assert [r['move_s'] for r in rows] == [3.7, 3.7, 3.7, 2.1, 2.1, 2.1]
    assert package['review']['nominal_total_s'] == pytest.approx(23.4)
    for row in rows:
        assert row['peak_velocity_deg_s'] <= SPEED_DEG_S[row['joint']]
        assert row['peak_acceleration_rad_s2'] <= ACCEL_RAD_S2[row['joint']]
    assert rows[1]['raw_motor_displacement_deg'] == pytest.approx({'left': -88., 'right': 88.})
    assert rows[4]['raw_motor_displacement_deg'] == pytest.approx({'left': 15., 'right': -15.})
    assert rows[5]['raw_motor_displacement_deg'] == pytest.approx({'left': -15., 'right': 15.})


def test_saved_candidate_is_expired_and_has_no_hardware_authorization(prepared):
    _, version, mappings, package = prepared
    assert package['request']['expires_at'] == 0.
    assert package['request']['offline_only'] is True
    assert not package['review']['hardware_accessed']
    assert not package['review']['hardware_execution_allowed']
    assert package['review']['baseline']['kind'] == 'assumed_logical_zero_not_measured'
    with pytest.raises(ValueError, match='Expired'):
        preflight(package['request'], version, mappings, dict.fromkeys(KEYS, 0.), time.time(), set())


def test_original_mistyped_j6_is_reported_without_clamping(prepared):
    plan, version, mappings, _ = prepared
    plan['steps'][1]['joint'] = 6
    package = build_package(plan, version, mappings)
    assert package['review']['status'] == 'BLOCKED_OFFLINE_VALIDATION'
    assert {r['motor_id'] for r in package['review']['limit_violations']} == {6}
    assert math.degrees(package['request']['points'][2]['q'][10]) == pytest.approx(88.)
    assert package['request']['expires_at'] == 0.


@pytest.mark.parametrize('offset', [-3., 3.])
def test_actual_baseline_shift_cannot_expand_absolute_limits(prepared, offset):
    plan, version, mappings, _ = prepared
    # Exercise both travel extrema even though the latest user sequence returns to zero.
    plan['steps'][2]['target_deg'] = 88.
    baseline = dict.fromkeys(KEYS, 0.)
    baseline['left', 5] = math.radians(offset)
    before = baseline.copy()
    package = build_package(plan, version, mappings, baseline)
    assert baseline == before
    assert package['review']['status'] == 'BLOCKED_OFFLINE_VALIDATION'
    assert any(r['side'] == 'left' and r['motor_id'] == 5 for r in package['review']['limit_violations'])
    interval = next(r for r in package['review']['allowed_logical_baseline_intervals'] if (r['side'], r['motor_id']) == ('left', 5))
    assert interval['minimum_deg'] == pytest.approx(-2., abs=.0001)
    assert interval['maximum_deg'] == pytest.approx(2., abs=.0001)


def test_explicit_positive_j5_target_remains_a_176_degree_reversal(prepared):
    plan, version, mappings, _ = prepared
    plan['steps'][2]['target_deg'] = 88.
    package = build_package(plan, version, mappings)
    assert package['review']['rows'][2]['displacement_deg'] == 176.
    assert package['review']['rows'][2]['move_s'] == 7.4


def test_mirrored_j5_targets_return_to_each_frozen_baseline(prepared):
    plan, version, mappings, _ = prepared
    plan['steps'][0]['target_deg'] = dict(left=-88., right=88.)
    plan['steps'][2]['target_deg'] = 0.
    baseline = dict.fromkeys(KEYS, 0.)
    baseline['left', 5], baseline['right', 5] = math.radians(.5), math.radians(-.5)
    package = build_package(plan, version, mappings, baseline)
    points, rows = package['request']['points'], package['review']['rows']
    assert package['review']['validation_errors'] == []
    assert points[1]['q'][8:10] == pytest.approx([math.radians(-87.5), math.radians(87.5)])
    assert points[3]['q'][8:10] == pytest.approx([baseline['left', 5], baseline['right', 5]])
    assert rows[0]['target_test_deg_by_side'] == {'left': -88., 'right': 88.}
    assert rows[2]['displacement_deg_by_side'] == {'left': 88., 'right': -88.}
    assert rows[2]['raw_motor_displacement_deg'] == pytest.approx({'left': 88., 'right': -88.})
    assert rows[2]['start_test_deg'] == {'left': -88., 'right': 88.}
    assert rows[2]['target_test_deg'] == 0.
    assert len(package['request']['joints']) == len(package['settings']) == 14
    assert package['request']['offline_only'] is True


@pytest.mark.parametrize('right_target', [44., 0.])
def test_unequal_arm_displacements_share_the_longer_required_duration(prepared, right_target):
    plan, version, mappings, _ = prepared
    plan['steps'][0]['target_deg'] = dict(left=-88., right=right_target)
    plan['steps'][2]['target_deg'] = dict(left=0., right=0.)
    package = build_package(plan, version, mappings)
    rows = package['review']['rows']
    assert package['review']['validation_errors'] == []
    for row in (rows[0], rows[2]):
        assert row['move_s'] == 3.7
        speeds = row['peak_velocity_deg_s_by_side']
        assert speeds['left'] == pytest.approx(1.875 * 88. / 3.7)
        assert speeds['right'] == pytest.approx(1.875 * right_target / 3.7)
        assert row['peak_velocity_deg_s'] == speeds['left']
    for stage_index, step in enumerate(plan['steps'], 1):
        before = package['request']['points'][stage_index-1]['q']
        after = package['request']['points'][stage_index]['q']
        assert len(before) == len(after) == 14
        assert all(after[i] == before[i] for i, key in enumerate(KEYS) if key[1] != step['joint'])


@pytest.mark.parametrize('target', [{}, {'left': -88.}, {'right': 88.},
                                  {'left': -88., 'right': 88., 'other': 0.},
                                  {'left': -88., 'other': 88.},
                                  {'left': -88., 'right': math.nan}])
def test_bilateral_target_requires_exact_finite_left_and_right(prepared, target):
    plan, version, mappings, _ = prepared
    plan['steps'][0]['target_deg'] = target
    with pytest.raises(ValueError):
        build_package(plan, version, mappings)


def test_snapshot_uses_motor_mapping_and_preserves_evidence(prepared, tmp_path):
    _, _, mappings, _ = prepared
    reports = [dict(side=s, online=7, bus_status='0x0', time_unix=123.,
                    motors=[dict(motor_id=j, position_rad=.001*j, state=0) for j in range(1, 8)])
               for s in ('left', 'right')]
    path = tmp_path/'snapshot.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in reports), encoding='utf-8')
    baseline, source = read_baseline(path, mappings)
    assert baseline['left', 7] == pytest.approx(-.007)
    assert baseline['right', 7] == pytest.approx(.007)
    assert source['kind'] == 'saved_feedback_not_live'
    assert source['snapshot_time_unix'] == 123.
    assert len(source['sha256']) == 64
    reports[1]['motors'][0] = reports[1]['motors'][1].copy()
    path.write_text('\n'.join(json.dumps(r) for r in reports), encoding='utf-8')
    with pytest.raises(ValueError, match='unique'):
        read_baseline(path, mappings)


@pytest.mark.parametrize('fault', ['incremental', 'zero_move', 'nonfinite', 'joint_bool'])
def test_invalid_plan_is_rejected(prepared, fault):
    plan, version, mappings, _ = prepared
    if fault == 'incremental':
        plan['target_mode'] = 'incremental'
    elif fault == 'zero_move':
        plan['steps'][0]['target_deg'] = 0.
    elif fault == 'nonfinite':
        plan['steps'][0]['target_deg'] = math.nan
    else:
        plan['steps'][0]['joint'] = True
    with pytest.raises(ValueError):
        build_package(plan, version, mappings)


@pytest.mark.parametrize('mirrored', [False, True])
def test_ideal_feedback_replay_preserves_14_axis_hold_and_one_second_gates(prepared, monkeypatch, mirrored):
    # Kinematic executor contract only. This is not a physical collision simulation.
    plan, version, mappings, package = prepared
    if mirrored:
        plan['steps'][0]['target_deg'] = dict(left=-88., right=88.)
        plan['steps'][2]['target_deg'] = 0.
        package = build_package(plan, version, mappings)
    request = copy.deepcopy(package['request'])
    request['expires_at'] = time.time()+120
    trajectory = preflight(request, version, mappings, dict.fromkeys(KEYS, 0.), time.time(), set())
    class Clock:
        now = 100.
        cancelled = False
        def is_set(self):
            return self.cancelled
        def wait(self, seconds):
            self.now += max(seconds, .000001)
            return self.cancelled
    clock = Clock()
    monkeypatch.setattr(time, 'perf_counter', lambda: clock.now)
    bus = VirtualTransport(mappings, dict.fromkeys(KEYS, 0.))
    records, holds = [], []
    def log(row):
        records.append(row)
        if row['type'] == 'sample':
            assert bus.enabled and len(row['target']) == 14
            stage = int(row['stage'][:2])
            joint = package['review']['rows'][stage-1]['joint']
            for i, key in enumerate(KEYS):
                if key[1] != joint:
                    assert row['target'][i] == trajectory.points[stage-1][1][i]
            if row['phase'] == 'holding_target':
                assert row['target'] == trajectory.points[-1][1]
                holds.append(row)
                if len(holds) == 5:
                    clock.cancelled = True
    outcome = execute_sequence(trajectory, mappings, bus, log, sequence=request['sequence'],
                               period=.04, max_lateness=.08, max_error=math.radians(5),
                               cancel=clock, hold_after=True)
    complete = [r for r in records if r['type'] == 'stage_complete']
    assert [r['stage'] for r in complete] == [r['name'] for r in request['sequence']['stages']]
    assert all(r['dwell_s'] >= 1. for r in complete)
    assert len(holds) == 5 and outcome == 'CANCELLED'
    assert not bus.enabled and records[-1]['state'] == 'STOPPED'


def test_cli_writes_expired_candidate_without_model_or_hardware(prepared, tmp_path):
    assert main(['--output', str(tmp_path)]) == 0
    request = json.loads((tmp_path/'request.draft.json').read_text(encoding='utf-8'))
    assert request['expires_at'] == 0.
    assert len(json.loads((tmp_path/'settings.json').read_text(encoding='utf-8'))) == 14


def test_cli_refuses_to_mix_new_request_with_old_dynamics(prepared, tmp_path):
    old = tmp_path/'dynamics.json'
    old.write_text('{"old_evidence": true}', encoding='utf-8')
    with pytest.raises(FileExistsError, match='new output directory'):
        main(['--output', str(tmp_path)])
    assert not (tmp_path/'request.draft.json').exists()
    assert json.loads(old.read_text(encoding='utf-8')) == {'old_evidence': True}
