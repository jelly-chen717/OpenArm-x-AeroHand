from dataclasses import replace
import json
import math
import sys
import time

import pytest

from drive_openarm_from_sim import ROOT, execute
from openarm_bus_lock import ChannelLease
from openarm_driver import DriverError, LIMITS, MotorState, motor_type
from openarm_execution import PcanTransport, rated_limits
from openarm_joint_mapping import load_mapping
from openarm_trajectory import Trajectory, preflight


class FakeDriver:
    instances = []
    fail_enable = False
    fail_command = False
    fail_disable = False

    def __init__(self, side, channel, timeout):
        self.side = side
        self.enabled = False
        self.closed = False
        self.position = 0.
        self.__class__.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def snapshot(self):
        return dict(side=self.side, online=7, bus_status='0x0',
                    motors=[dict(state=int(self.enabled)) for i in range(7)])

    def read_parameter(self, motor_id, register):
        return dict(zip((10,21,22,23),(1,*LIMITS[motor_type(motor_id)])))[register]

    def query_motor(self, motor_id):
        return MotorState(motor_id,motor_type(motor_id),int(self.enabled),self.position,
                          0.,0.,25,25,time.monotonic(),'fixture')

    def enable_motor(self, motor_id, **kwargs):
        self.enabled = True
        if self.fail_enable:
            raise DriverError('partly enabled')
        return self.query_motor(motor_id)

    def command_mit(self, motor_id,p,v,kp,kd):
        if self.fail_command:
            raise DriverError('disconnected')
        self.position = p
        return self.query_motor(motor_id)

    def disable_motor(self, motor_id):
        if self.fail_disable:
            raise DriverError('disable disconnected')
        self.enabled = False
        return self.query_motor(motor_id)


@pytest.mark.parametrize('key', [('left', 1), ('left', 6), ('left', 7), ('right', 2)])
def test_unified_positive_command_reverses_motor_and_feedback_roundtrips(key):
    FakeDriver.fail_enable = FakeDriver.fail_command = FakeDriver.fail_disable = False
    _, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    with PcanTransport(mappings, [key], dict(left=0x51, right=0x52), driver_factory=FakeDriver) as bus:
        bus.enable()
        mapping = mappings[key]
        bus.command({key: (mapping.to_motor(.01), mapping.velocity_to_motor(.02))})
        assert bus.states[key].position_rad == pytest.approx(-.01)
        assert bus.read([key])[key] == pytest.approx(.01)
        assert mapping.velocity_to_model(mapping.velocity_to_motor(.02)) == pytest.approx(.02)
        assert bus.stop()


def test_old_coordinate_request_is_rejected_before_execution():
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    request = dict(request_id='old-direction', mapping_version='hardware-user-confirmed-v1-20261115')
    with pytest.raises(ValueError, match='Mapping version mismatch'):
        preflight(request, version, mappings, {}, time.time(), set())


@pytest.fixture
def transport():
    FakeDriver.instances = []
    FakeDriver.fail_enable = False
    FakeDriver.fail_command = False
    FakeDriver.fail_disable = False
    version,mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json',hardware=True)
    return version,mappings,PcanTransport(mappings,[('right',7)],dict(left=0x51,right=0x52),
                                         driver_factory=FakeDriver)


def test_channel_lease_conflict_and_release():
    first,second = ChannelLease(0x7ffe),ChannelLease(0x7ffe)
    first.acquire()
    try:
        with pytest.raises(OSError):
            second.acquire()
    finally:
        first.release()
    second.acquire()
    second.release()


def test_hardware_adapter_roundtrip_and_disabled_readback(transport):
    version,mappings,bus = transport
    with bus:
        request = dict(request_id='hardware-fixture',mapping_version=version,expires_at=time.time()+10,
                       joints=[dict(side='right',motor_id=7)],
                       limits=dict(velocity_rad_s=.3,acceleration_rad_s2=10,start_error_rad=.01),
                       points=[dict(t=0,q=[0]),dict(t=.1,q=[.01]),dict(t=.2,q=[0])])
        trajectory = preflight(request,version,mappings,bus.read([('right',7)]),time.time(),set())
        bus.validate_trajectory(trajectory)
        assert execute(trajectory,mappings,bus,lambda r:None,period=.01) == 'COMPLETED'
        assert bus.stop_records[-1]['confirmed']
        assert bus.require_disabled()
    assert all(arm.closed for arm in FakeDriver.instances)


@pytest.mark.parametrize('fault',['enable','command'])
def test_partial_failures_disable_before_close(transport,fault):
    version,mappings,bus = transport
    with pytest.raises(DriverError):
        with bus:
            if fault == 'enable':
                FakeDriver.fail_enable = True
            bus.enable()
            FakeDriver.fail_command = True
            bus.command({('right',7):(.01,0)})
    assert all(not arm.enabled and arm.closed for arm in FakeDriver.instances)
    assert bus.stop_records[-1]['confirmed']


def test_stale_feedback_and_travel_are_advisory(transport):
    version,mappings,bus = transport
    with bus:
        state = bus.states['right',7]
        assert bus.check(('right',7),replace(state,received_at=time.monotonic()-1),
                         expected_state=0) is False
        assert bus.check(('right',7),replace(state,position_rad=.2),expected_state=0) is False
        assert {'stale','travel'} <= {row['kind'] for row in bus.notes}


def test_unconfirmed_stop_keeps_fault_visible(transport):
    version,mappings,bus = transport
    with pytest.raises(DriverError,match='unconfirmed'):
        with bus:
            bus.enable()
            FakeDriver.fail_disable = True
            assert not bus.stop()
    assert bus.attempted
    assert all(arm.closed for arm in FakeDriver.instances)


class MultiJointDriver(FakeDriver):
    fail_enable_id = None
    fail_command_id = None

    def __init__(self, *args):
        super().__init__(*args)
        self.enabled_ids = set()
        self.positions = dict.fromkeys(range(1, 8), 0.)
        self.commands = []
        self.gains = {}

    def snapshot(self):
        return dict(side=self.side, online=7, bus_status='0x0',
                    motors=[dict(state=int(i in self.enabled_ids)) for i in range(1, 8)])

    def query_motor(self, motor_id):
        return MotorState(motor_id, motor_type(motor_id), int(motor_id in self.enabled_ids),
                          self.positions[motor_id], 0., 0., 25, 25, time.monotonic(), 'fixture')

    def enable_motor(self, motor_id, **kwargs):
        self.enabled_ids.add(motor_id)
        self.gains[motor_id] = kwargs
        if motor_id == self.fail_enable_id:
            raise DriverError('partly enabled multi-joint')
        return self.query_motor(motor_id)

    def command_mit(self, motor_id, p, v, kp, kd):
        if motor_id == self.fail_command_id:
            raise DriverError('multi-joint disconnected')
        self.commands.append(motor_id)
        self.positions[motor_id] = p
        return self.query_motor(motor_id)

    def disable_motor(self, motor_id):
        self.enabled_ids.discard(motor_id)
        return self.query_motor(motor_id)


@pytest.fixture
def multi_transport():
    MultiJointDriver.instances = []
    MultiJointDriver.fail_enable_id = None
    MultiJointDriver.fail_command_id = None
    version, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    keys = [('right', 6), ('right', 7)]
    settings = {key: dict(kp=20.+key[1], kd=2., max_travel=.04, max_speed=.2,
                          max_torque=1., max_temperature=50., max_age=.15) for key in keys}
    bus = PcanTransport(mappings, keys, dict(left=0x51, right=0x52),
                        driver_factory=MultiJointDriver, joint_settings=settings, limit_profile='custom')
    return version, mappings, bus


def test_multijoint_roundtrip_and_individual_gains(multi_transport):
    version, mappings, bus = multi_transport
    with bus:
        request = dict(request_id='multi-fixture', mapping_version=version, expires_at=time.time()+10,
                       joints=[dict(side=side, motor_id=i) for side, i in bus.keys],
                       limits=dict(velocity_rad_s=.2, acceleration_rad_s2=10, start_error_rad=.01),
                       points=[dict(t=0, q=[0, 0]), dict(t=.2, q=[.01, -.01]),
                               dict(t=.4, q=[0, 0])])
        trajectory = preflight(request, version, mappings, bus.read(bus.keys), time.time(), set())
        bus.validate_trajectory(trajectory)
        assert execute(trajectory, mappings, bus, lambda row: None, period=.01,
                       endpoint_tolerance=.001) == 'COMPLETED'
        assert bus.arms['right'].gains[6]['kp'] == 26.
        assert bus.arms['right'].gains[7]['kp'] == 27.
        assert {row['motor_id'] for row in bus.stop_records if row['confirmed']} == {6, 7}
        bus.require_disabled()


@pytest.mark.parametrize('fault', ['enable', 'command'])
def test_multijoint_partial_failure_stops_every_attempted_axis(multi_transport, fault):
    _, _, bus = multi_transport
    with pytest.raises(DriverError):
        with bus:
            if fault == 'enable':
                MultiJointDriver.fail_enable_id = 7
            bus.enable()
            MultiJointDriver.fail_command_id = 7
            bus.command({('right', 6): (.01, 0), ('right', 7): (.01, 0)})
    assert not bus.attempted
    assert all(not arm.enabled_ids and arm.closed for arm in MultiJointDriver.instances)


def test_nonfinite_feedback_is_a_note(transport):
    _, _, bus = transport
    with bus:
        state = bus.states['right', 7]
        assert bus.check(('right', 7), replace(state, torque_nm=math.nan),
                         expected_state=0) is False
        assert 'nonfinite' in {row['kind'] for row in bus.notes}


def test_multijoint_command_no_longer_prevalidates_travel(multi_transport):
    _, _, bus = multi_transport
    with bus:
        bus.enable()
        bus.command({('right', 6): (.01, 0), ('right', 7): (.05, 0)})
        assert bus.arms['right'].commands == [6, 7]
        assert 'travel' in {row['kind'] for row in bus.notes}


def test_multijoint_per_axis_feedback_limits_are_notes(multi_transport):
    _, _, bus = multi_transport
    with bus:
        bus.joint_settings['right', 6]['max_torque'] = .1
        assert bus.check(('right', 6), replace(bus.states['right', 6], torque_nm=.2),
                         expected_state=0) is False
        assert 'speed_torque' in {row['kind'] for row in bus.notes}
        with pytest.raises(ValueError, match='all gain'):
            PcanTransport(bus.mappings, bus.keys, bus.channels,
                          joint_settings={key: dict(kp=20.) for key in bus.keys})


@pytest.mark.parametrize('action', ['preflight', 'execute'])
@pytest.mark.parametrize('profile', ['rated', 'custom'])
def test_multijoint_cli_entry(multi_transport, tmp_path, monkeypatch, action, profile):
    import drive_openarm_from_sim
    import openarm_execution

    version, mappings, bus = multi_transport
    request = dict(request_id='cli-multi', mapping_version=version, expires_at=time.time()+10,
                   joints=[dict(side=side, motor_id=i) for side, i in bus.keys],
                   initial_model_position=[0, 0],
                   limits=dict(velocity_rad_s=.2, acceleration_rad_s2=10, start_error_rad=.01),
                   points=[dict(t=0, q=[0, 0]), dict(t=.2, q=[.01, -.01]),
                           dict(t=.4, q=[0, 0])])
    request_path = tmp_path/'request.json'
    request_path.write_text(json.dumps(request), encoding='utf-8')
    settings_path = tmp_path/'settings.json'
    settings_path.write_text(json.dumps([dict(side=key[0], motor_id=key[1], **settings)
                                        for key, settings in bus.joint_settings.items()]), encoding='utf-8')
    log_path = tmp_path/'execution.jsonl'
    real_transport = PcanTransport

    def make_transport(*args, **kwargs):
        return real_transport(*args, **kwargs, driver_factory=MultiJointDriver)

    monkeypatch.setattr(openarm_execution, 'PcanTransport', make_transport)
    monkeypatch.setattr(drive_openarm_from_sim, 'ROOT', tmp_path)
    monkeypatch.setattr(sys, 'argv', ['drive_openarm_from_sim.py', str(request_path),
                                    '--mode', 'hardware', '--action', action,
                                    '--mapping', str(ROOT/'configs/openarm_joint_mapping.json'),
                                    '--joint-settings', str(settings_path), '--period', '.01',
                                    *(['--limit-profile', 'custom'] if profile == 'custom' else []),
                                    '--left-channel', '0x51', '--right-channel', '0x52',
                                    '--confirm-bus-ownership', '--log', str(log_path)])
    drive_openarm_from_sim.main()
    rows = [json.loads(line) for line in log_path.read_text(encoding='utf-8').splitlines()]
    settings_log = next(row for row in rows if row['type'] == 'joint_settings')
    assert len(settings_log['joints']) == 2
    assert settings_log['limit_profile'] == profile
    for row in settings_log['joints']:
        assert row['max_torque'] == (3. if profile == 'rated' else 1.)
        assert row['max_speed'] == pytest.approx(4 * math.pi if profile == 'rated' else .2)
        assert row['max_travel'] == .04
        assert row['kp'] == 20. + row['motor_id']
    assert any(row['type'] == 'sample' for row in rows) == (action == 'execute')
    assert all(not arm.enabled_ids and arm.closed for arm in MultiJointDriver.instances)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('motor_id,rpm,torque', [
    (1, 100, 20), (2, 100, 20), (3, 36, 9), (4, 36, 9),
    (5, 120, 3), (6, 120, 3), (7, 120, 3),
])
def test_rated_limits_are_reported_not_enforced(side, motor_id, rpm, torque):
    FakeDriver.fail_enable = FakeDriver.fail_command = FakeDriver.fail_disable = False
    _, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    key = side, motor_id
    speed = rpm * 2 * math.pi / 60
    with PcanTransport(mappings, [key], dict(left=0x51, right=0x52),
                       driver_factory=FakeDriver) as bus:
        assert bus.joint_settings[key]['max_speed'] == pytest.approx(speed)
        assert bus.joint_settings[key]['max_torque'] == torque
        state = bus.states[key]
        assert bus.check(key, replace(state, velocity_rad_s=speed, torque_nm=torque),
                         expected_state=0) is True
        assert bus.check(key, replace(state, velocity_rad_s=-speed, torque_nm=-torque),
                         expected_state=0) is True
        for field, value in [('velocity_rad_s', speed * 1.001), ('torque_nm', torque * 1.001)]:
            assert bus.check(key, replace(state, **{field: value}), expected_state=0) is False
        assert 'speed_torque' in {row['kind'] for row in bus.notes}
        assert bus.check(key, replace(state, mos_c=50), expected_state=0) is False
        assert 'temperature' in {row['kind'] for row in bus.notes}
        # The trajectory validator only checks joint identity now; no velocity cap.
        for ratio in (.99, 1.01):
            trajectory = Trajectory('rated', (key,), ((0., (0.,)),
                (1.875 * .01 / (speed * ratio), (.01,))), time.time()+10)
            bus.validate_trajectory(trajectory)
        bus.enable()
        bus.command({key: (0., speed)})
        bus.command({key: (0., speed * 1.001)})
        assert bus.stop()


def test_rated_profile_overrides_old_j4_settings_and_custom_retains_them():
    _, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    key = 'left', 4
    old = dict(kp=60., kd=2., max_travel=.35, max_speed=.5, max_torque=2.,
               max_temperature=50., max_age=.15)
    for profile in ('rated', 'custom'):
        bus = PcanTransport(mappings, [key], dict(left=0x51, right=0x52),
                            joint_settings={key: old}, limit_profile=profile)
        expected = dict(old, **rated_limits(4)) if profile == 'rated' else old
        assert bus.joint_settings[key] == expected
    assert old['max_torque'] == 2.  # Loading does not rewrite historical settings.


class BatchDriver(MultiJointDriver):
    events = []
    bad_feedback_key = None
    send_failure_key = None

    def __init__(self, *args):
        super().__init__(*args)
        self.bus = self

    def send(self, motor_id, payload):
        key = self.side, motor_id
        self.events.append(('send', key))
        if key == self.send_failure_key:
            raise DriverError('batch send failed')
        pmax = LIMITS[motor_type(motor_id)][0]
        self.positions[motor_id] = int.from_bytes(payload[:2], 'big')/65535*2*pmax-pmax

    def query_motor(self, motor_id):
        key = self.side, motor_id
        self.events.append(('query', key))
        state = super().query_motor(motor_id)
        if key == self.bad_feedback_key and motor_id in self.enabled_ids:
            state = replace(state, torque_nm=30.)
        return state

    def read_parameter(self, motor_id, register):
        self.events.append(('parameter', (self.side, motor_id, register)))
        return super().read_parameter(motor_id, register)


@pytest.fixture
def batch_transport():
    BatchDriver.instances = []
    BatchDriver.events = []
    BatchDriver.fail_enable_id = BatchDriver.fail_command_id = None
    BatchDriver.bad_feedback_key = BatchDriver.send_failure_key = None
    _, mappings = load_mapping(ROOT/'configs/openarm_joint_mapping.json', hardware=True)
    keys = [(s,j) for j in range(7,0,-1) for s in ('left','right')]
    return PcanTransport(mappings, keys, dict(left=0x51,right=0x52),
                         driver_factory=BatchDriver, batch_mit=True)


def test_batch_sends_all_pairs_before_queries_and_records_host_timings(batch_transport):
    with batch_transport as bus:
        bus.enable()
        BatchDriver.events.clear()
        bus.command({key: (.01, 0.) for key in bus.keys})
        assert BatchDriver.events == ([('send', key) for key in bus.keys]
                                      + [('query', key) for key in bus.keys])
        assert all(state.position_rad == pytest.approx(.01, abs=.0004) for state in bus.states.values())
        record = bus.last_dispatch
        assert record['complete'] is True
        assert record['timing_basis'] == 'host_bus_send_calls_not_device_arrival'
        assert set(record['pair_send_skew_s']) == {f'J{j}' for j in range(1,8)}
        assert len(record['joint_send_offsets_s']) == 14
        assert 0 <= record['send_span_s'] <= record['cycle_s']
        assert record['feedback_collection_s'] >= 0
        assert all(v >= 0 for v in record['pair_send_skew_s'].values())
        json.dumps(record, allow_nan=False)


def test_batch_sends_all_pairs_even_with_bad_feedback(batch_transport):
    with batch_transport as bus:
        bus.enable()
        BatchDriver.events.clear()
        BatchDriver.bad_feedback_key = bus.keys[0]
        targets = {key: (.01, 0.) for key in bus.keys}
        bus.command(targets)
        assert BatchDriver.events == ([('send', key) for key in bus.keys]
                                      + [('query', key) for key in bus.keys])
        assert 'speed_torque' in {row['kind'] for row in bus.notes}


def test_batch_send_failure_execute_cleanup_disables_every_attempted_axis(batch_transport):
    with batch_transport as bus:
        trajectory = Trajectory('batch-fault', bus.keys,
                                ((0., (0.,)*14), (.2, (.01,)*14)), time.time()+10)
        original_enable = bus.enable

        def enable_then_inject():
            original_enable()
            BatchDriver.send_failure_key = bus.keys[3]

        bus.enable = enable_then_inject
        with pytest.raises(DriverError):
            execute(trajectory, bus.mappings, bus, lambda row: None, period=.01)
        assert not bus.attempted
        assert len([r for r in bus.stop_records if r['confirmed']]) == 14
        assert bus.last_dispatch['complete'] is False
        assert all(not arm.enabled_ids for arm in bus.arms.values())


def test_batch_bad_feedback_is_advisory_and_execution_completes(batch_transport):
    with batch_transport as bus:
        trajectory = Trajectory('batch-note', bus.keys,
                                ((0., (0.,)*14), (.2, (.01,)*14)), time.time()+10)
        original_enable = bus.enable

        def enable_then_inject():
            original_enable()
            BatchDriver.bad_feedback_key = bus.keys[0]

        bus.enable = enable_then_inject
        assert execute(trajectory, bus.mappings, bus, lambda row: None, period=.01) == 'COMPLETED'
        assert 'speed_torque' in {row['kind'] for row in bus.notes}
        assert bus.last_dispatch['complete'] is True
        assert all(not arm.enabled_ids for arm in bus.arms.values())
