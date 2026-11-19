import threading
import time

import pytest

from calibrate_openarm_mapping import fit_mapping
from drive_openarm_from_sim import VirtualTransport, execute, ROOT
from openarm_joint_mapping import load_mapping
from openarm_trajectory import preflight


def fixture():
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    request = dict(request_id='test', mapping_version=version, expires_at=time.time()+30,
                   joints=[dict(side='right', motor_id=7)],
                   limits=dict(velocity_rad_s=2, acceleration_rad_s2=100, start_error_rad=.01),
                   points=[dict(t=0, q=[0]), dict(t=.1, q=[.01]), dict(t=.2, q=[0])])
    return request, version, mappings, {('right', 7): 0.}


def test_fit_sign_zero_scale_and_validation():
    result = fit_mapping([(0,.2), (.2,-.2), (.4,-.6)],
                         [(.1,0), (.3,-.4)], min_span=.1, max_error=.001)
    assert result['mapping']['direction'] == -1
    assert result['mapping']['scale'] == pytest.approx(2)
    assert result['motor_angle_at_model_zero'] == pytest.approx(.1)
    assert result['residual_check_passed']
    assert result['hardware_verified'] is False
    bad = fit_mapping([(0,0), (.2,.2), (.4,.4)], [(0,0), (.4,.5)],
                      min_span=.1, max_error=.001)
    assert not bad['residual_check_passed']
    with pytest.raises(ValueError):
        fit_mapping([(0,0)]*3, [(0,0), (.1,.1)], min_span=.1, max_error=.01)


@pytest.mark.parametrize('issue', ['expired','duplicate','version','start','velocity','acceleration','limit','nan','time'])
def test_trajectory_rejections(issue):
    request, version, mappings, current = fixture()
    seen = set()
    if issue == 'expired': request['expires_at'] = time.time()-1
    if issue == 'duplicate': seen.add('test')
    if issue == 'version': request['mapping_version'] = 'wrong'
    if issue == 'start': current['right',7] = .5
    if issue == 'velocity': request['limits']['velocity_rad_s'] = .001
    if issue == 'acceleration': request['limits']['acceleration_rad_s2'] = .001
    if issue == 'limit': request['points'][1]['q'] = [10]
    if issue == 'nan': request['points'][1]['q'] = [float('nan')]
    if issue == 'time': request['points'][1]['t'] = 0
    with pytest.raises(ValueError):
        preflight(request, version, mappings, current, time.time(), seen)


def test_quintic_entire_path_and_execution_stop():
    request, version, mappings, current = fixture()
    trajectory = preflight(request, version, mappings, current, time.time(), set())
    assert trajectory.sample(.1)[0] == (.01,)
    assert trajectory.sample(.05)[0] == pytest.approx((.005,))
    assert trajectory.sample(.2)[0] == (0.,)
    records = []
    transport = VirtualTransport(mappings, current)
    seen = set()
    assert execute(trajectory, mappings, transport, records.append, period=.005, seen=seen) == 'COMPLETED'
    assert not transport.enabled
    assert records[-1]['state'] == 'STOPPED'
    assert max(r['target'][0] for r in records if r['type']=='sample') > .009
    with pytest.raises(ValueError, match='Duplicate'):
        execute(trajectory, mappings, transport, records.append, seen=seen)


def test_cancel_and_transport_fault_stop():
    request, version, mappings, current = fixture()
    trajectory = preflight(request, version, mappings, current, time.time(), set())
    transport = VirtualTransport(mappings, current)
    cancel = threading.Event()
    cancel.set()
    assert execute(trajectory, mappings, transport, lambda r: None, cancel=cancel) == 'CANCELLED'
    def fail(targets):
        raise OSError('disconnected')
    transport.command = fail
    records = []
    with pytest.raises(OSError, match='disconnected'):
        execute(trajectory, mappings, transport, records.append)
    assert not transport.enabled
    assert records[-1]['outcome'] == 'FAULT'


def test_logging_failure_still_stops_transport():
    request, version, mappings, current = fixture()
    trajectory = preflight(request, version, mappings, current, time.time(), set())
    transport = VirtualTransport(mappings, current)
    def broken_log(row):
        raise OSError('disk full')
    with pytest.raises(OSError):
        execute(trajectory, mappings, transport, broken_log)
    assert not transport.enabled


def test_coarse_feedback_clock_does_not_repeat_execution_time(monkeypatch):
    request, version, mappings, current = fixture()
    trajectory = preflight(request,version,mappings,current,time.time(),set())
    transport = VirtualTransport(mappings,current)
    monkeypatch.setattr(time,'monotonic',lambda: 100.)
    records = []
    execute(trajectory,mappings,transport,records.append,period=.01)
    times = [r['t'] for r in records if r['type']=='sample']
    assert all(b>a for a,b in zip(times,times[1:]))
    assert times[-1] >= trajectory.duration


def test_stationary_motor_cannot_pass_outbound_checkpoint():
    request,version,mappings,current = fixture()
    trajectory = preflight(request,version,mappings,current,time.time(),set())
    transport = VirtualTransport(mappings,current)
    transport.command = lambda targets: None
    records = []
    with pytest.raises(RuntimeError,match='Waypoint'):
        execute(trajectory,mappings,transport,records.append,period=.005,endpoint_tolerance=.002)
    assert not transport.enabled
    assert records[-1]['outcome'] == 'FAULT'


@pytest.mark.parametrize('fault',['tracking','deadline','expired'])
def test_runtime_faults_stop_transport(fault):
    request,version,mappings,current = fixture()
    if fault == 'expired':
        request['expires_at'] = time.time()+.03
    trajectory = preflight(request,version,mappings,current,time.time(),set())
    transport = VirtualTransport(mappings,current)
    if fault == 'tracking':
        transport.command = lambda targets: None
    if fault == 'deadline':
        transport.command = lambda targets: time.sleep(.03)
    with pytest.raises(RuntimeError):
        execute(trajectory,mappings,transport,lambda r:None,period=.005,
                max_lateness=.005,max_error=.001)
    assert not transport.enabled


def test_cancel_during_execution_stops_future_targets():
    request,version,mappings,current = fixture()
    trajectory = preflight(request,version,mappings,current,time.time(),set())
    transport = VirtualTransport(mappings,current)
    cancel = threading.Event()
    original = transport.command
    def command(targets):
        original(targets)
        cancel.set()
    transport.command = command
    records = []
    assert execute(trajectory,mappings,transport,records.append,cancel=cancel) == 'CANCELLED'
    assert len([r for r in records if r['type']=='sample']) == 1
    assert not transport.enabled


def test_existing_cancel_file_prevents_enable(tmp_path):
    request, version, mappings, current = fixture()
    trajectory = preflight(request, version, mappings, current, time.time(), set())
    transport = VirtualTransport(mappings, current)
    def unexpected_enable():
        raise AssertionError('Existing cancellation must prevent enable')
    transport.enable = unexpected_enable
    cancel_file = tmp_path/'cancel.stop'
    cancel_file.write_text('cancel', encoding='utf-8')
    assert execute(trajectory, mappings, transport, lambda row: None,
                   cancel_file=cancel_file) == 'CANCELLED'


@pytest.mark.parametrize('exit_kind', ['cancel', 'tracking'])
def test_hold_after_keeps_final_target_enabled_and_stops_on_exit(exit_kind, tmp_path, monkeypatch):
    request, version, mappings, current = fixture()
    trajectory = preflight(request, version, mappings, current, time.time(), set())
    transport = VirtualTransport(mappings, current)
    cancel_file = tmp_path/'cancel.stop'
    records = []
    held_samples = []
    def log(row):
        records.append(row)
        if row['type'] == 'state' and row['state'] == 'HOLDING_TARGET':
            assert transport.enabled
            # The execution request expiry does not terminate explicitly requested holding.
            monkeypatch.setattr(time, 'time', lambda: trajectory.expires_at+1)
        elif row['type'] == 'sample' and any(r.get('state') == 'HOLDING_TARGET' for r in records):
            assert transport.enabled
            assert row['target'] == trajectory.points[-1][1]
            assert row['target_velocity'] == (0.,)
            held_samples.append(row)
            if len(held_samples) == 3:
                if exit_kind == 'cancel':
                    cancel_file.write_text('stop')
                else:
                    transport.read = lambda keys: {k: 1. for k in keys}
    if exit_kind == 'tracking':
        with pytest.raises(RuntimeError, match='Tracking error'):
            execute(trajectory, mappings, transport, log, period=.005,
                    hold_after=True, cancel_file=cancel_file, endpoint_tolerance=.01)
        assert records[-1]['outcome'] == 'FAULT'
    else:
        assert execute(trajectory, mappings, transport, log, period=.005,
                       hold_after=True, cancel_file=cancel_file, endpoint_tolerance=.01) == 'CANCELLED'
    assert len(held_samples) >= 3
    assert not transport.enabled
    assert records[-1]['state'] == 'STOPPED'
