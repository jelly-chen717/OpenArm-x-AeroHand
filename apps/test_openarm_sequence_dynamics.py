"""Exercise real offline model estimates and exact path-limit rejection."""
import copy
import math

import pytest

from openarm_joint_mapping import load_mapping
from openarm_sequence_dynamics import ROOT, analyze_dynamics


def request_for(points):
    version, _ = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    return dict(mapping_version=version,
                joints=[dict(side=s, motor_id=j) for j in range(1, 8) for s in ('left', 'right')],
                points=[dict(t=float(i), q=q) for i, q in enumerate(points)],
                sequence=dict(dwell_s=1.))


def test_j6_88_rejected_before_model_import(monkeypatch):
    import builtins
    original = builtins.__import__
    def guard(name, *args, **kwargs):
        if name == 'mujoco':
            raise AssertionError('Out-of-range request must not load model')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guard)
    end = [0.]*14
    end[10:12] = [math.radians(88)]*2
    result = analyze_dynamics(request_for([[0.]*14, end]))
    assert result['status'] == 'BLOCKED_LIMITS'
    assert {row['side'] for row in result['violations']} == {'left', 'right'}
    assert not result['torque_estimated']
    assert 'all_axes_max_torque_nm' not in result


def test_intermediate_reversal_is_not_hidden_by_legal_endpoints():
    middle = [0.]*14
    middle[8:10] = [math.radians(-91)]*2
    result = analyze_dynamics(request_for([[0.]*14, middle, [0.]*14]))
    assert result['status'] == 'BLOCKED_LIMITS'
    assert {row['point_index'] for row in result['violations']} == {1}


def test_real_model_reports_all_axes_and_repeated_joint_segments():
    middle, end = [0.]*14, [0.]*14
    middle[8:10] = [math.radians(-10)]*2
    end[8:10] = [math.radians(10)]*2
    result = analyze_dynamics(request_for([[0.]*14, middle, end]), samples_per_move=11)
    assert result['status'] == 'MODEL_ESTIMATE'
    assert result['nominal_total_s'] == 4.
    assert len(result['all_axes_max_torque_nm']) == 14
    assert result['rows'][1]['delta_deg']['left:5'] == pytest.approx(20.)
    assert result['rows'][1]['peak_velocity_deg_s']['right:5'] == pytest.approx(37.5)
    assert set(result['rows'][1]['active_axes_peak_torque_nm']) == {'left:5', 'right:5'}
    assert all(math.isfinite(v) and v >= 0 for v in result['all_axes_max_torque_nm'].values())
    assert result['sample_count'] == 23
    assert result['source_sha256']
    assert result['caveats']


def test_large_wrist_sweep_reports_contact_even_when_limits_pass():
    q = [0.]*14
    points = [q.copy()]
    for joint, target in [(5, -88.), (7, 88.), (5, 88.)]:
        q[(joint-1)*2:joint*2] = [math.radians(target)]*2
        points.append(q.copy())
    result = analyze_dynamics(request_for(points), samples_per_move=31)
    assert result['status'] == 'MODEL_ESTIMATE'
    assert result['initial_pose_contact_count'] == 0
    assert not result['modeled_path_contact_free']
    assert result['rows'][0]['collision_contact_sample_count'] == 0
    assert result['rows'][1]['collision_contact_sample_count'] == 0
    assert result['rows'][2]['collision_contact_sample_count'] > 0
    assert result['rows'][2]['first_contact_fraction'] < result['rows'][2]['last_contact_fraction']
    assert any('body_link0' in pair for pair in result['collision_pairs_min_distance_m'])


@pytest.mark.parametrize('fault', ['duplicate', 'nonfinite', 'backwards_time', 'version'])
def test_invalid_request_rejected(fault):
    request = request_for([[0.]*14, [0.]*14])
    if fault == 'duplicate':
        request['joints'][1] = copy.deepcopy(request['joints'][0])
    elif fault == 'nonfinite':
        request['points'][1]['q'][0] = math.nan
    elif fault == 'backwards_time':
        request['points'][1]['t'] = 0.
    else:
        request['mapping_version'] = 'old-coordinate-version'
    with pytest.raises(ValueError):
        analyze_dynamics(request)
