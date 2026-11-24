"""Offline measured-arrival checks; these tests never open PCAN."""
import math
import time

import pytest

from drive_openarm_from_sim import (
    ROOT, VirtualTransport, execute_sequence, validate_sequence,
)
from openarm_joint_mapping import load_mapping
from openarm_trajectory import preflight


class Clock:
    def __init__(self):
        self.now = 100.
        self.cancelled = False

    def wait(self, duration):
        self.now += max(.000001, duration)
        return self.cancelled

    def is_set(self):
        return self.cancelled


@pytest.fixture
def setup(monkeypatch):
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    keys = (('left', 4), ('right', 4), ('left', 3), ('right', 3))
    current = dict.fromkeys(keys, 0.)
    request = dict(
        request_id='dual-sequence', mapping_version=version, expires_at=time.time()+60,
        joints=[dict(side=side, motor_id=motor_id) for side, motor_id in keys],
        limits=dict(velocity_rad_s=2., acceleration_rad_s2=100., start_error_rad=.01),
        points=[dict(t=0., q=[0.]*4), dict(t=.2, q=[math.radians(3)]*2+[0., 0.]),
                dict(t=.4, q=[math.radians(3)]*2+[-.1, -.1])],
    )
    trajectory = preflight(request, version, mappings, current, time.time(), set())
    sequence = dict(schema_version=1, dwell_s=1.5, settle_timeout_s=2.5,
                    settle_speed_rad_s=.03,
                    endpoint_tolerances_rad=[math.radians(.5)]*2+[math.radians(2)]*2,
                    stages=[dict(name='J4', endpoint_index=1), dict(name='J3', endpoint_index=2)])
    clock = Clock()
    monkeypatch.setattr(time, 'perf_counter', lambda: clock.now)
    transport = VirtualTransport(mappings, current)
    return trajectory, mappings, transport, sequence, clock


def run(setup, log, **kwargs):
    trajectory, mappings, transport, sequence, clock = setup
    return execute_sequence(trajectory, mappings, transport, log, sequence=sequence,
                            period=.02, max_lateness=.1, max_error=.2,
                            cancel=clock, **kwargs)


def test_each_stage_waits_for_measured_arrival_and_full_dwell(setup):
    trajectory, mappings, transport, sequence, clock = setup
    records = []
    assert run(setup, records.append, hold_after=False) == 'COMPLETED'
    complete = [r for r in records if r['type'] == 'stage_complete']
    starts = [r for r in records if r['type'] == 'stage_start']
    assert [r['stage'] for r in complete] == ['J4', 'J3']
    assert all(r['dwell_s'] >= 1.5 for r in complete)
    assert starts[1]['t'] >= complete[0]['t']
    assert all(r['total_stage_s'] >= .2+1.5 for r in complete)
    assert not transport.enabled


@pytest.mark.parametrize('stalled_keys', [(('left', 4), ('right', 4)), (('right', 4),)])
def test_stationary_j4_or_one_arm_times_out_and_forces_next_stage(setup, stalled_keys):
    trajectory, mappings, transport, sequence, clock = setup
    original = transport.command
    def command(targets):
        original(targets)
        for key in stalled_keys:
            transport.position[key] = mappings[key].to_motor(0.)
    transport.command = command
    records = []
    assert run(setup, records.append, hold_after=False) == 'COMPLETED'
    starts = [r['stage'] for r in records if r['type'] == 'stage_start']
    assert starts == ['J4', 'J3']
    j4 = next(r for r in records if r['type'] == 'stage_complete' and r['stage'] == 'J4')
    assert j4['forced'] is True
    assert all(r['target'][2:] == (0., 0.) for r in records
               if r['type'] == 'sample' and r['stage'] == 'J4')
    assert any(r['type'] == 'stage_timeout' for r in records)
    assert records[-1]['state'] == 'STOPPED'
    assert not transport.enabled


def test_speed_above_settle_threshold_resets_continuous_dwell(setup):
    trajectory, mappings, transport, sequence, clock = setup
    sequence['settle_timeout_s'] = 4.
    original = transport.read_velocity
    def velocity(keys):
        result = original(keys)
        if 100.7 <= clock.now <= 100.9:
            result['right', 4] = .04
        return result
    transport.read_velocity = velocity
    records = []
    run(setup, records.append, hold_after=False)
    resets = [r for r in records if r['type'] == 'stage_dwell_reset']
    first_done = next(r for r in records if r['type'] == 'stage_complete')
    assert len(resets) == 1
    assert first_done['t'] >= .9+1.5
    assert first_done['dwell_s'] >= 1.5


def test_already_completed_joint_hold_error_is_logged_not_raised(setup):
    trajectory, mappings, transport, sequence, clock = setup
    original = transport.command
    def command(targets):
        original(targets)
        if targets['left', 3][0] < -.001:
            transport.position['left', 4] = mappings['left', 4].to_motor(0.)
    transport.command = command
    records = []
    assert run(setup, records.append, hold_after=False) == 'COMPLETED'
    assert [r['stage'] for r in records if r['type'] == 'stage_complete'] == ['J4', 'J3']
    assert any(r['type'] == 'hold_tolerance_exceeded' for r in records)
    assert not transport.enabled


def test_final_hold_logs_global_position_error_and_dispatch_until_cancel(setup):
    trajectory, mappings, transport, sequence, clock = setup
    sequence['endpoint_tolerances_rad'] = [math.radians(2)]*4
    transport.last_dispatch = dict(dispatch_mode='test', pair_send_skew_s={'J4': .0001})
    original = transport.command
    holding = False
    def command(targets):
        original(targets)
        if holding:
            transport.position['right', 4] += math.radians(2.1)
    transport.command = command
    records = []
    held = 0
    def log(row):
        nonlocal holding, held
        records.append(row)
        if row.get('state') == 'HOLDING_TARGET':
            holding = True
        if row['type'] == 'sample' and row['phase'] == 'holding_target':
            held += 1
            if held == 3:
                clock.cancelled = True
    assert run(setup, log) == 'CANCELLED'
    assert any(row['type'] == 'hold_tolerance_exceeded' for row in records)
    assert all(row['dispatch']['pair_send_skew_s']['J4'] == .0001
               for row in records if row['type'] == 'sample')
    assert records[-1]['state'] == 'STOPPED'


def test_final_targets_stay_enabled_until_cancel_and_ignore_request_expiry(setup, monkeypatch):
    trajectory, mappings, transport, sequence, clock = setup
    records = []
    held = []
    def log(row):
        records.append(row)
        if row.get('state') == 'HOLDING_TARGET':
            assert transport.enabled
            monkeypatch.setattr(time, 'time', lambda: trajectory.expires_at+1.)
        if row['type'] == 'sample' and row['phase'] == 'holding_target':
            assert transport.enabled
            assert row['target'] == trajectory.points[-1][1]
            assert row['target_velocity'] == (0.,)*len(trajectory.keys)
            held.append(row)
            if len(held) == 5:
                clock.cancelled = True
    assert run(setup, log) == 'CANCELLED'
    assert len(held) == 5
    assert records[-1]['state'] == 'STOPPED'
    assert not transport.enabled


@pytest.mark.parametrize('problem', ['shape', 'too_large_j4_tolerance', 'stage_order', 'timeout'])
def test_bad_sequence_rejected_before_enable(setup, problem):
    trajectory, mappings, transport, sequence, clock = setup
    if problem == 'shape':
        sequence['endpoint_tolerances_rad'].pop()
    elif problem == 'too_large_j4_tolerance':
        sequence['endpoint_tolerances_rad'][0] = math.radians(5)
    elif problem == 'stage_order':
        sequence['stages'][1]['endpoint_index'] = 1
    else:
        sequence['settle_timeout_s'] = 1.
    with pytest.raises(ValueError):
        run(setup, lambda row: None)
    assert not transport.enabled


def test_cancel_file_exists_does_not_enable(setup, tmp_path):
    cancel = tmp_path/'cancel.stop'
    cancel.write_text('cancel', encoding='utf-8')
    records = []
    assert run(setup, records.append, cancel_file=cancel) == 'CANCELLED'
    assert not any(row.get('state') == 'HOLDING' for row in records)
    assert not setup[2].enabled


def test_logging_fault_during_dwell_still_disables(setup):
    def log(row):
        if row['type'] == 'stage_arrived':
            raise OSError('disk full')
    with pytest.raises(OSError, match='disk full'):
        run(setup, log)
    assert not setup[2].enabled


def test_stage_specific_j4_tolerance_relaxes_only_after_its_own_acceptance(setup):
    trajectory, mappings, transport, sequence, clock = setup
    sequence['endpoint_tolerances_rad'] = [math.radians(2)]*4
    sequence['stages'][0]['endpoint_tolerances_rad'] = [math.radians(.5)]*2+[math.radians(2)]*2
    original = transport.command
    def command(targets):
        original(targets)
        # A later proximal movement changes the J4 gravity load. A one-degree
        # holding offset may pass the 2-degree hold allowance, never its 0.5-degree gate.
        if targets['left', 3][0] < -.001:
            transport.position['left', 4] -= math.radians(1.)
    transport.command = command
    records = []
    assert run(setup, records.append, hold_after=False) == 'COMPLETED'
    done = [r for r in records if r['type'] == 'stage_complete']
    assert abs(done[0]['error'][0]) <= math.radians(.5)
    assert abs(done[1]['error'][0]) == pytest.approx(math.radians(1.))


def test_stage_specific_j4_tolerance_timeout_still_advances(setup):
    trajectory, mappings, transport, sequence, clock = setup
    sequence['endpoint_tolerances_rad'] = [math.radians(2)]*4
    sequence['stages'][0]['endpoint_tolerances_rad'] = [math.radians(.5)]*2+[math.radians(2)]*2
    original = transport.command
    def command(targets):
        original(targets)
        if targets['left', 4][1] == 0. and targets['left', 4][0] > 0.:
            transport.position['left', 4] -= math.radians(1.)
    transport.command = command
    records = []
    assert run(setup, records.append, hold_after=False) == 'COMPLETED'
    j4 = next(r for r in records if r['type'] == 'stage_complete' and r['stage'] == 'J4')
    assert j4['forced'] is True
    assert any(r['type'] == 'stage_timeout' for r in records)


def test_full_14_axis_sequence_preserves_every_completed_target(setup):
    _, mappings, _, _, clock = setup
    version, _ = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    keys = tuple((side, motor_id) for motor_id in range(7, 0, -1) for side in ('left', 'right'))
    deltas = {7: -15., 6: -30., 5: 45., 4: 3., 3: -45., 2: -45., 1: -45.}
    points = [dict(t=0., q=[0.]*14)]
    stages = []
    for step, motor_id in enumerate(range(7, 0, -1), 1):
        q = list(points[-1]['q'])
        for index, key in enumerate(keys):
            if key[1] == motor_id:
                q[index] = math.radians(deltas[motor_id])
        points.append(dict(t=step*1., q=q))
        stages.append(dict(name=f'J{motor_id}', endpoint_index=step))
    request = dict(request_id='all14', mapping_version=version, expires_at=time.time()+60,
                   joints=[dict(side=side, motor_id=motor_id) for side, motor_id in keys],
                   limits=dict(velocity_rad_s=2., acceleration_rad_s2=10., start_error_rad=.01),
                   points=points)
    initial = dict.fromkeys(keys, 0.)
    trajectory = preflight(request, version, mappings, initial, time.time(), set())
    sequence = dict(schema_version=1, dwell_s=1.5, settle_timeout_s=2.5,
                    settle_speed_rad_s=.03, endpoint_tolerances_rad=[math.radians(2)]*14,
                    stages=stages)
    stages[3]['endpoint_tolerances_rad'] = [math.radians(.5 if key[1] == 4 else 2.) for key in keys]
    transport = VirtualTransport(mappings, initial)
    records = []
    def log(row):
        records.append(row)
        if row.get('state') == 'HOLDING_TARGET':
            assert transport.enabled
            assert len(transport.position) == 14
            clock.cancelled = True
    assert execute_sequence(trajectory, mappings, transport, log, sequence=sequence,
                            period=.02, max_error=.1, cancel=clock) == 'CANCELLED'
    done = [r for r in records if r['type'] == 'stage_complete']
    assert [r['stage'] for r in done] == [f'J{i}' for i in range(7, 0, -1)]
    assert all(r['dwell_s'] >= 1.5 for r in done)
    for row in (r for r in records if r['type'] == 'sample'):
        moving_id = int(row['stage'][1:])
        for key, q in zip(keys, row['target']):
            if key[1] > moving_id:
                assert q == pytest.approx(math.radians(deltas[key[1]]))
            elif key[1] < moving_id:
                assert q == 0.
    # Unified logical signs must map to the appropriate opposite raw motor signs.
    assert transport.position['left', 7] == pytest.approx(math.radians(15))
    assert transport.position['right', 7] == pytest.approx(math.radians(-15))
    assert transport.position['left', 1] == pytest.approx(math.radians(45))
    assert transport.position['right', 2] == pytest.approx(math.radians(45))


@pytest.mark.parametrize('allow_dual', [False, True])
def test_cli_requires_explicit_dual_arm_and_preflight_never_enables(setup, tmp_path, monkeypatch, allow_dual):
    import json
    import sys
    import drive_openarm_from_sim
    import openarm_execution
    from test_openarm_execution_hardware import MultiJointDriver

    trajectory, mappings, _, sequence, _ = setup
    version, _ = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    request = dict(request_id='dual-cli', mapping_version=version, expires_at=time.time()+60,
                   initial_model_position=[0.]*len(trajectory.keys),
                   joints=[dict(side=side, motor_id=motor_id) for side, motor_id in trajectory.keys],
                   limits=dict(velocity_rad_s=2., acceleration_rad_s2=100., start_error_rad=.01),
                   points=[dict(t=t, q=q) for t, q in trajectory.points], sequence=sequence)
    request_path, settings_path = tmp_path/'request.json', tmp_path/'settings.json'
    request_path.write_text(json.dumps(request), encoding='utf-8')
    settings_path.write_text(json.dumps([
        dict(side=side, motor_id=motor_id, kp=30., kd=2., max_travel=.2,
             max_speed=2., max_torque=9., max_temperature=50., max_age=.15)
        for side, motor_id in trajectory.keys]), encoding='utf-8')
    MultiJointDriver.instances = []
    MultiJointDriver.fail_enable_id = MultiJointDriver.fail_command_id = None
    real_transport = openarm_execution.PcanTransport
    def make_transport(*args, **kwargs):
        return real_transport(*args, **kwargs, driver_factory=MultiJointDriver)
    monkeypatch.setattr(openarm_execution, 'PcanTransport', make_transport)
    monkeypatch.setattr(sys, 'argv', ['drive_openarm_from_sim.py', str(request_path),
        '--mode', 'hardware', '--action', 'preflight', '--sequence', '--hold-after',
        '--cancel-file', str(tmp_path/'cancel.stop'), '--joint-settings', str(settings_path),
        '--left-channel', '0x51', '--right-channel', '0x52', '--confirm-bus-ownership',
        '--log', str(tmp_path/'preflight.jsonl'), *(['--allow-dual-arm'] if allow_dual else [])])
    if allow_dual:
        drive_openarm_from_sim.main()
        assert len(MultiJointDriver.instances) == 2
        assert all(not arm.gains and not arm.enabled_ids for arm in MultiJointDriver.instances)
        rows = [json.loads(line) for line in (tmp_path/'preflight.jsonl').read_text().splitlines()]
        assert any(row['type'] == 'sequence_settings' for row in rows)
    else:
        with pytest.raises(SystemExit):
            drive_openarm_from_sim.main()
        assert MultiJointDriver.instances == []


@pytest.mark.parametrize('mode', ['virtual', 'hardware'])
def test_sequence_metadata_requires_flag_before_hardware_or_mapping_access(tmp_path, monkeypatch, capsys, mode):
    import json
    import sys
    import drive_openarm_from_sim
    import openarm_execution

    request_path = tmp_path/'request.json'
    request_path.write_text(json.dumps({'sequence': {'schema_version': 1}}), encoding='utf-8')
    def unexpected_access(*args, **kwargs):
        raise AssertionError('Sequence mode mismatch must fail before transport or mapping access')
    monkeypatch.setattr(drive_openarm_from_sim, 'load_mapping', unexpected_access)
    monkeypatch.setattr(openarm_execution, 'PcanTransport', unexpected_access)
    monkeypatch.setattr(sys, 'argv', ['drive_openarm_from_sim.py', str(request_path),
        '--mode', mode, '--action', 'execute', '--allow-dual-arm', '--batch-mit',
        '--left-channel', '0x51', '--right-channel', '0x52', '--confirm-bus-ownership',
        '--log', str(tmp_path/'execution.jsonl')])
    with pytest.raises(SystemExit) as exc:
        drive_openarm_from_sim.main()
    assert exc.value.code == 2
    assert 'Request contains sequence metadata; --sequence is required' in capsys.readouterr().err
    assert not (tmp_path/'execution.jsonl').exists()


@pytest.mark.parametrize('action', ['preflight', 'execute'])
def test_offline_candidate_rejected_before_hardware_even_with_refreshed_expiry(tmp_path, monkeypatch, capsys, action):
    import json
    import sys
    import drive_openarm_from_sim
    import openarm_execution

    request_path = tmp_path/'request.json'
    request_path.write_text(json.dumps(dict(offline_only=True, expires_at=time.time()+120,
                                           authorization_status='USER_EXPLICITLY_AUTHORIZED')), encoding='utf-8')
    def unexpected_access(*args, **kwargs):
        raise AssertionError('Offline candidate must fail before mapping or device access')
    monkeypatch.setattr(drive_openarm_from_sim, 'load_mapping', unexpected_access)
    monkeypatch.setattr(openarm_execution, 'PcanTransport', unexpected_access)
    monkeypatch.setattr(sys, 'argv', ['drive_openarm_from_sim.py', str(request_path),
        '--mode', 'hardware', '--action', action, '--allow-dual-arm', '--batch-mit',
        '--left-channel', '0x51', '--right-channel', '0x52', '--confirm-bus-ownership',
        '--log', str(tmp_path/'execution.jsonl')])
    with pytest.raises(SystemExit) as exc:
        drive_openarm_from_sim.main()
    assert exc.value.code == 2
    assert 'Offline preparation candidate; rebuild after frozen-zero/model review and approval' in capsys.readouterr().err
    assert not (tmp_path/'execution.jsonl').exists()


def test_offline_candidate_allows_virtual_preflight(setup, tmp_path, monkeypatch):
    import json
    import sys
    import drive_openarm_from_sim
    import openarm_execution

    trajectory, _, _, sequence, _ = setup
    version, _ = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    request = dict(offline_only=True, request_id='offline-virtual', mapping_version=version,
                   expires_at=time.time()+120, initial_model_position=[0.]*len(trajectory.keys),
                   joints=[dict(side=s, motor_id=j) for s, j in trajectory.keys],
                   limits=dict(velocity_rad_s=2., acceleration_rad_s2=100., start_error_rad=.01),
                   points=[dict(t=t, q=q) for t, q in trajectory.points], sequence=sequence)
    request_path = tmp_path/'request.json'
    request_path.write_text(json.dumps(request), encoding='utf-8')
    def unexpected_device(*args, **kwargs):
        raise AssertionError('Virtual preflight must not access hardware')
    monkeypatch.setattr(openarm_execution, 'PcanTransport', unexpected_device)
    monkeypatch.setattr(sys, 'argv', ['drive_openarm_from_sim.py', str(request_path),
        '--mode', 'virtual', '--action', 'preflight', '--sequence', '--hold-after',
        '--cancel-file', str(tmp_path/'cancel.stop'), '--log', str(tmp_path/'preflight.jsonl')])
    drive_openarm_from_sim.main()
    rows = [json.loads(line) for line in (tmp_path/'preflight.jsonl').read_text(encoding='utf-8').splitlines()]
    assert any(row['type'] == 'sequence_settings' for row in rows)
