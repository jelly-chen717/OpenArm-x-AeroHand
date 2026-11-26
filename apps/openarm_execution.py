"""Single-owner PCAN transport for calibrated joint execution.

Laboratory profile (2026-11-19): feedback thresholds, current-relative travel and
protocol-range checks are recorded as notes instead of aborting the run. The
disable path, the per-frame encoding and the request bookkeeping are unchanged.
The removed and retained guards are listed in
docs/project/OpenArm防护删除_总纲_20261125.md.
"""
from contextlib import ExitStack
from dataclasses import asdict
import math
import time

from openarm_driver import DriverError, OpenArmDriver, encode_mit
from openarm_joint_mapping import LIMIT_TOLERANCE, finite

WARN_INTERVAL = 2.  # Seconds between repeats of the same advisory.


def rated_limits(motor_id):
    """24 V output-shaft ratings from 关节详细资料.md, not protocol/peak limits."""
    if type(motor_id) is not int or motor_id not in range(1, 8):
        raise ValueError('Invalid rated-limit motor ID')
    rpm, torque = (100., 20.) if motor_id <= 2 else (36., 9.) if motor_id <= 4 else (120., 3.)
    return dict(max_speed=rpm * 2 * math.pi / 60, max_torque=torque)


class PcanTransport:
    mode_code = 1
    dispatch_mode = 'batch_mit_send_then_query'
    def __init__(self, mappings, keys, channels, *, kp=10., kd=2., timeout=.1,
                 max_travel=.12, max_speed=None, max_torque=None, max_temperature=50.,
                 max_age=.15, driver_factory=OpenArmDriver, joint_settings=None,
                 limit_profile='rated', batch_mit=False, warn=None):
        self.mappings = mappings
        self.keys = tuple(keys)
        if not self.keys or len(set(self.keys)) != len(self.keys):
            raise ValueError('Invalid command joints')
        if any(key not in mappings for key in self.keys):
            raise ValueError('Command joint missing from mapping')
        if set(channels) != {'left', 'right'} or channels['left'] == channels['right']:
            raise ValueError('Two distinct physical arm channels required')
        self.channels = channels
        if limit_profile not in ('rated', 'custom'):
            raise ValueError('Unknown limit profile')
        self.limit_profile = limit_profile
        if type(batch_mit) is not bool:
            raise ValueError('batch_mit must be boolean')
        self.batch_mit = batch_mit
        self.last_dispatch = None
        self.verified_mode_keys = set()
        self.kp, self.kd = finite(kp), finite(kd)
        self.timeout = finite(timeout)
        self.max_travel = finite(max_travel)
        self.max_speed = None if max_speed is None else finite(max_speed)
        self.max_torque = None if max_torque is None else finite(max_torque)
        self.max_temperature, self.max_age = map(finite, (max_temperature,max_age))
        if min(self.timeout, self.max_travel, self.max_temperature, self.max_age) <= 0:
            raise ValueError('Protection thresholds must be positive')
        defaults = dict(kp=self.kp, kd=self.kd, max_travel=self.max_travel,
                        max_speed=self.max_speed, max_torque=self.max_torque,
                        max_temperature=self.max_temperature, max_age=self.max_age)
        if joint_settings is not None and set(joint_settings) != set(self.keys):
            raise ValueError('Joint settings must match command joints')
        self.joint_settings = {}
        for key in self.keys:
            settings = defaults.copy()
            rated = rated_limits(key[1])
            for name, value in rated.items():
                if settings[name] is None:
                    settings[name] = value
            if joint_settings is not None:
                if set(joint_settings[key]) != set(defaults):
                    raise ValueError('Each joint requires all gain and protection fields')
                settings.update({name: finite(value) for name, value in joint_settings[key].items()})
            if min(settings[name] for name in defaults if name not in ('kp', 'kd')) <= 0:
                raise ValueError('Joint protection thresholds must be positive')
            # Rated is the test default, including when loading older task files.
            # Custom explicitly opts into the task's speed/torque thresholds.
            if limit_profile == 'rated':
                settings.update(rated)
            encode_mit(key[1], 0, 0, settings['kp'], settings['kd'], 0)
            self.joint_settings[key] = settings
        self.driver_factory = driver_factory
        self.arms = {}
        self.origins = {}
        self.states = {}
        self.modes = {}
        self.attempted = set()
        self.stop_records = []
        self.notes = []
        self.warn = warn
        self._warned = {}
        self.stack = ExitStack()

    def _note(self, kind, message, key=None):
        """Record a condition that used to abort the run. Never raises."""
        row = dict(time_unix=time.time(), kind=kind,
                   joint=None if key is None else f'{key[0]}:{key[1]}', message=message)
        self.notes.append(row)
        del self.notes[:-400]
        now = time.monotonic()
        if now - self._warned.get((kind, key), -1e9) < WARN_INTERVAL:
            return
        self._warned[(kind, key)] = now
        if self.warn is not None:
            try:
                self.warn(message)
            except Exception:  # Advisory output must never stop a motion.
                pass

    def __enter__(self):
        try:
            for side in ('left', 'right'):
                arm = self.driver_factory(side, self.channels[side], self.timeout)
                self.arms[side] = self.stack.enter_context(arm)
            self.before = self.snapshot_arms()
            for key in self.keys:
                side, motor_id = key
                arm = self.arms[side]
                try:
                    self.modes[key] = arm.read_parameter(motor_id, 10)
                except (DriverError, OSError) as exc:
                    self.modes[key] = None
                    self._note('mode_read', f'{side} J{motor_id} 模式读取失败：{exc}', key)
                self.verified_mode_keys.add(key)
                state = arm.query_motor(motor_id)
                self.origins[key] = state.position_rad
                self.check(key, state, expected_state=0)
                self.states[key] = state
            return self
        except BaseException:
            self.stack.close()
            raise

    def __exit__(self, *args):
        try:
            if self.attempted and not self.stop():
                raise DriverError('Disable remains unconfirmed')
        finally:
            self.stack.close()

    def snapshot_arms(self):
        """Read both arms without judging the result. Callers may still inspect it."""
        return [self.arms[side].snapshot() for side in ('left','right')]

    def require_disabled(self):
        """Evidence helper for the stop path: raise unless all 14 axes read back disabled."""
        reports = self.snapshot_arms()
        for report in reports:
            if (report['online'] != 7 or report['bus_status'] != '0x0'
                    or any(m.get('state') != 0 for m in report['motors'])):
                raise DriverError('Both arms must have seven online disabled joints')
        return reports

    def check(self, key, state, *, expected_state=None):
        """Compare one feedback frame against the configured thresholds.

        Every former `raise DriverError(...)` here is now a recorded note so an
        in-flight motion is not interrupted by a single sample. Returns True when
        the sample is inside every configured threshold.
        """
        settings = self.joint_settings[key]
        values = (state.position_rad, state.velocity_rad_s, state.torque_nm,
                  state.mos_c, state.rotor_c, state.received_at)
        if any(not math.isfinite(v) for v in values):
            self._note('nonfinite', f'{key[0]} J{key[1]} 反馈含非有限值', key)
            return False
        clean = True
        age = time.monotonic()-state.received_at
        if not 0 <= age <= settings['max_age']:
            clean = False
            self._note('stale', f'{key[0]} J{key[1]} 反馈 {age*1000:.0f} ms，超过 {settings["max_age"]*1000:.0f} ms', key)
        if expected_state is not None and state.state != expected_state:
            clean = False
            self._note('state', f'{key[0]} J{key[1]} 电机状态 {state.state}，期望 {expected_state}', key)
        if max(state.mos_c,state.rotor_c) >= settings['max_temperature']:
            clean = False
            self._note('temperature', f'{key[0]} J{key[1]} 温度 {max(state.mos_c,state.rotor_c)} °C，阈值 {settings["max_temperature"]}', key)
        if abs(state.velocity_rad_s) > settings['max_speed'] or abs(state.torque_nm) > settings['max_torque']:
            clean = False
            self._note('speed_torque', f'{key[0]} J{key[1]} 反馈 {state.velocity_rad_s:.4f} rad/s、'
                                       f'{state.torque_nm:.3f} Nm，阈值 {settings["max_speed"]:.4f} / {settings["max_torque"]}', key)
        if abs(state.position_rad-self.origins[key]) > settings['max_travel']:
            clean = False
            self._note('travel', f'{key[0]} J{key[1]} 相对起点 {state.position_rad-self.origins[key]:+.4f} rad，'
                                 f'阈值 {settings["max_travel"]:.4f}', key)
        mapping = self.mappings[key]
        q = mapping.to_model(state.position_rad)
        if not mapping.limits[0] - LIMIT_TOLERANCE <= q <= mapping.limits[1] + LIMIT_TOLERANCE:
            clean = False
            self._note('limit', f'{key[0]} J{key[1]} 反馈 {q:+.4f} rad 超出模型限位', key)
        return clean

    def read(self, keys):
        return {key: self.mappings[key].to_model(self.states[key].position_rad) for key in keys}

    def validate_trajectory(self, trajectory):
        if tuple(trajectory.keys) != self.keys:
            raise ValueError('Trajectory joints differ from transport joints')

    def enable(self, *, check_start=None, cancel=None):
        if check_start is not None:
            check_start(self.snapshot_arms())
        for key in self.keys:
            if cancel is not None and cancel.is_set():
                return
            side, motor_id = key
            state = self.arms[side].query_motor(motor_id)
            # Include even a partly failed enable in final disable attempts.
            self.attempted.add(key)
            state = self._enable_axis(key, state)
            self.check(key, state)
            self.states[key] = state

    def _enable_axis(self, key, state):
        settings = self.joint_settings[key]
        return self.arms[key[0]].enable_motor(key[1], kp=settings['kp'], kd=settings['kd'])

    def _disable_axis(self, key):
        return self.arms[key[0]].disable_motor(key[1])

    def _command_frame(self, key, p, v):
        settings = self.joint_settings[key]
        return key[1], encode_mit(key[1], p, v, settings['kp'], settings['kd'], 0)

    def command(self, targets):
        self.last_dispatch = None
        if set(targets) != set(self.keys) or self.attempted != set(self.keys):
            raise DriverError('Command joints differ from enabled joints')
        payloads = {key: self._command_frame(key, finite(p), finite(v))
                    for key,(p,v) in targets.items()}
        if self.batch_mit:
            self._command_batch(payloads)
            return
        for key in self.keys:
            settings = self.joint_settings[key]
            side,motor_id = key
            p, v = targets[key]
            state = self.arms[side].command_mit(motor_id,p,v,settings['kp'],settings['kd'])
            self.check(key,state)
            self.states[key] = state

    def _command_batch(self, payloads):
        """Submit a shared target batch before collecting any motor replies.

        Timings measure host send calls, not actual device arrival or motion.
        The caller's exception/finally path stops every attempted joint.
        """
        began = time.perf_counter()
        sends = {}
        send_finished = None
        complete = False
        try:
            for key in self.keys:
                sends[key] = time.perf_counter()-began
                self.arms[key[0]].bus.send(*payloads[key])
            send_finished = time.perf_counter()
            for key in self.keys:
                state = self.arms[key[0]].query_motor(key[1])
                self.check(key, state)
                self.states[key] = state
            complete = True
        finally:
            ended = time.perf_counter()
            self.last_dispatch = dict(
                dispatch_mode=self.dispatch_mode,
                timing_basis='host_bus_send_calls_not_device_arrival',
                joint_send_offsets_s={f'{s}:{j}':offset for (s,j),offset in sends.items()},
                pair_send_skew_s={f'J{j}':abs(sends[('left',j)]-sends[('right',j)])
                                  for j in range(1,8)
                                  if ('left',j) in sends and ('right',j) in sends},
                send_span_s=None if send_finished is None else send_finished-began,
                feedback_collection_s=None if send_finished is None else ended-send_finished,
                cycle_s=ended-began, complete=complete)

    def stop(self):
        for key in tuple(self.attempted):
            side,motor_id = key
            for attempt in range(3):
                try:
                    state = self._disable_axis(key)
                    if state.state != 0:
                        raise DriverError('Disabled state not read back')
                    self.stop_records.append(dict(side=side, confirmed=True, **asdict(state)))
                    self.attempted.remove(key)
                    break
                except (DriverError,OSError) as exc:
                    self.stop_records.append(dict(side=side,motor_id=motor_id,
                                                  confirmed=False,attempt=attempt+1,error=str(exc)))
        return not self.attempted
