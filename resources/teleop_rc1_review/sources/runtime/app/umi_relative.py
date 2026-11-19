"""Pure, single-arm UMI translation candidate adapter.

This module opens no device, transport, ROS publisher or motor interface. A
candidate is an IK input, never hardware permission. The station must serialize
its own source admission, reset and output lifecycle around these methods.

Native source: eleven ASCII fields: image_END_plus_shift tx ty tz qx qy qz qw
epoch valid loss_count. Pose is Tcw; it is inverted exactly once here. Host
timestamps must be from the original same-boot CLOCK_MONOTONIC receiver, not
refreshed when a spool file is read. The stereo monitor reference uses the same
device clock. This is delay detection, not absolute hardware clock sync.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
import threading
from typing import Optional


class AdapterError(ValueError):
    """Invalid input/configuration; an engaged adapter must hold and rearm."""


def _vector(value, length, name):
    try:
        result = tuple(float(v) for v in value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise AdapterError(name + '_malformed') from exc
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise AdapterError(name + '_nonfinite_or_shape')
    return result


def _integer(value, name, *, positive=False):
    if not isinstance(value, int) or isinstance(value, bool) or value < int(positive):
        raise AdapterError(name + '_invalid_integer')
    return value


def _scalar(value, name):
    try:
        v = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise AdapterError(name + '_invalid_scalar') from exc
    if not math.isfinite(v):
        raise AdapterError(name + '_nonfinite')
    return v


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


def _mv(matrix, vector):
    return tuple(_dot(row, vector) for row in matrix)


def _unit_quaternion(value):
    q = _vector(value, 4, 'quaternion')
    n = _norm(q)
    if abs(n - 1.0) > .01:
        raise AdapterError('quaternion_not_unit')
    return tuple(v / n for v in q)


def quaternion_matrix(q):
    x, y, z, w = _unit_quaternion(q)
    return ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
            (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)))


def invert_tcw(translation, quaternion):
    """Return (world camera center, Rwc quaternion), meters and xyzw."""
    t = _vector(translation, 3, 'translation')
    q = _unit_quaternion(quaternion)
    inverse_q = (-q[0], -q[1], -q[2], q[3])
    return _mv(quaternion_matrix(inverse_q), tuple(-v for v in t)), inverse_q


def rotation_angle_deg(a, b):
    qa, qb = _unit_quaternion(a), _unit_quaternion(b)
    return math.degrees(2 * math.acos(min(1., max(0., abs(_dot(qa, qb))))))


def validate_rotation(value):
    """Explicit proper R_BW only; reject reflection, shear and scale matrices."""
    try:
        matrix = tuple(_vector(row, 3, 'R_BW_row') for row in value)
    except TypeError as exc:
        raise AdapterError('R_BW_malformed') from exc
    if len(matrix) != 3:
        raise AdapterError('R_BW_shape')
    for i in range(3):
        for j in range(3):
            if abs(_dot(matrix[i], matrix[j]) - int(i == j)) > 1e-6:
                raise AdapterError('R_BW_not_orthonormal')
    if abs(_dot(matrix[0], _cross(matrix[1], matrix[2])) - 1.) > 1e-6:
        raise AdapterError('R_BW_not_proper_rotation')
    return matrix


def rotation_from_axis_observations(world_positive_x, world_positive_y,
                                    min_displacement_m=.02, max_abs_axis_dot=.35):
    """Derive R_BW from operator-labeled robot +X and +Y motions in VIO world.

    The two labels are essential: this function cannot infer robot front or
    left from arbitrary VIO axes. Gram-Schmidt removes small hand alignment
    error; strongly nonorthogonal/short observations are rejected. Scale is not
    fitted. The station must verify the resulting mapping in its dry preview.
    """
    x = _vector(world_positive_x, 3, 'world_positive_x')
    y = _vector(world_positive_y, 3, 'world_positive_y')
    minimum = _scalar(min_displacement_m, 'min_displacement_m')
    tolerance = _scalar(max_abs_axis_dot, 'max_abs_axis_dot')
    if minimum <= 0. or not 0. <= tolerance < 1.:
        raise AdapterError('axis_observation_configuration_invalid')
    if min(_norm(x), _norm(y)) < minimum:
        raise AdapterError('axis_observation_too_short')
    x = tuple(v / _norm(x) for v in x)
    y = tuple(v / _norm(y) for v in y)
    projection = _dot(x, y)
    if abs(projection) > tolerance:
        raise AdapterError('axis_observations_not_orthogonal')
    y = tuple(b - projection*a for a, b in zip(x, y))
    y = tuple(v / _norm(y) for v in y)
    z = _cross(x, y)
    return validate_rotation((x, y, z))


@dataclass(frozen=True)
class NativePose:
    source_time_s: float
    epoch: int
    valid: bool
    loss_count: int
    camera_position_m: Optional[tuple]
    camera_quaternion_xyzw: Optional[tuple]
    serial: str
    stream_id: str
    sequence: int
    received_monotonic_ns: int
    monitor_source_time_s: float
    monitor_received_monotonic_ns: int

    def __post_init__(self):
        # Direct construction must be just as strict as the native parser.
        source = _scalar(self.source_time_s, 'source_time_s')
        monitor = _scalar(self.monitor_source_time_s, 'monitor_source_time_s')
        if min(source, monitor) < 0.:
            raise AdapterError('negative_source_time')
        if self.serial != 's200085' or not isinstance(self.stream_id, str) or not self.stream_id:
            raise AdapterError('unsupported_serial_or_missing_stream')
        for name in ('epoch', 'loss_count'):
            value = _integer(getattr(self, name), name)
            if value >= 2**64:
                raise AdapterError(name + '_out_of_range')
        _integer(self.sequence, 'sequence', positive=True)
        _integer(self.received_monotonic_ns, 'received_monotonic_ns')
        _integer(self.monitor_received_monotonic_ns, 'monitor_received_monotonic_ns')
        if type(self.valid) is not bool:
            raise AdapterError('valid_must_be_boolean')
        object.__setattr__(self, 'source_time_s', source)
        object.__setattr__(self, 'monitor_source_time_s', monitor)
        if self.valid:
            object.__setattr__(self, 'camera_position_m', _vector(self.camera_position_m, 3, 'camera_position'))
            object.__setattr__(self, 'camera_quaternion_xyzw', _unit_quaternion(self.camera_quaternion_xyzw))
        elif self.camera_position_m is not None or self.camera_quaternion_xyzw is not None:
            raise AdapterError('invalid_status_cannot_contain_usable_pose')

    @classmethod
    def from_line(cls, raw, *, stream_id, sequence, received_monotonic_ns,
                  monitor_source_time_s, monitor_received_monotonic_ns,
                  serial='s200085'):
        try:
            fields = raw.decode('ascii').split() if isinstance(raw, bytes) else raw.split()
        except (AttributeError, UnicodeError) as exc:
            raise AdapterError('native_packet_not_ascii') from exc
        if len(fields) != 11:
            raise AdapterError('native_packet_requires_11_fields')
        values = tuple(_scalar(v, 'native_pose') for v in fields[:8])
        if values[0] < 0.:
            raise AdapterError('negative_source_time')
        if not all(re.fullmatch('[0-9]+', f) for f in fields[8:]):
            raise AdapterError('native_status_not_integer')
        epoch, valid, losses = (int(v) for v in fields[8:])
        if epoch >= 2**64 or losses >= 2**64 or valid not in (0, 1):
            raise AdapterError('native_status_out_of_range')
        if not isinstance(stream_id, str) or not stream_id or len(stream_id) > 256:
            raise AdapterError('stream_id_required')
        if serial != 's200085':
            raise AdapterError('unsupported_camera_serial')
        seq = _integer(sequence, 'sequence', positive=True)
        recv = _integer(received_monotonic_ns, 'received_monotonic_ns')
        mon_recv = _integer(monitor_received_monotonic_ns, 'monitor_received_monotonic_ns')
        mon_source = _scalar(monitor_source_time_s, 'monitor_source_time_s')
        if mon_source < 0.:
            raise AdapterError('negative_monitor_time')
        position, quaternion = invert_tcw(values[1:4], values[4:8]) if valid else (None, None)
        return cls(values[0], epoch, bool(valid), losses, position, quaternion,
                   serial, stream_id, seq, recv, mon_source, mon_recv)


@dataclass(frozen=True)
class FeedbackPose:
    """Existing measured robot FK, already expressed in the configured frame."""
    position_m: tuple
    quaternion_xyzw: tuple
    measured_monotonic_ns: int
    side: str
    frame_id: str


@dataclass(frozen=True)
class IKTarget:
    side: str
    frame_id: str
    position_m: tuple
    quaternion_xyzw: tuple
    displacement_m: tuple
    source_time_s: float
    source_sequence: int
    epoch: int
    stream_id: str
    # This pure module cannot know shared motor/source admission state.
    hardware_permission: bool = False


class UmiTranslationAdapter:
    """Clutched, fixed-orientation absolute IK candidates from relative UMI pose.

    All failures hold (candidate=None), clear anchors and require an explicit
    new engage. Valid input recovery never resumes by itself. Disengagement
    does not disable motors, send a goal or command a reset/ready pose.
    """
    def __init__(self, *, side, robot_frame_id, R_BW, scale=1.,
                 max_displacement_m=.02, max_camera_rotation_deg=2.,
                 max_age_s=.25, max_monitor_age_s=.25, max_pose_lag_s=.25,
                 max_source_gap_s=.3, max_delivery_delay_s=.25,
                 max_feedback_age_s=.25, max_source_step_m=.05):
        if side not in ('left', 'right') or not isinstance(robot_frame_id, str) or not robot_frame_id:
            raise AdapterError('explicit_side_and_robot_frame_required')
        self.side, self.robot_frame_id = side, robot_frame_id
        self.R_BW = validate_rotation(R_BW)
        self.scale = _scalar(scale, 'scale')
        self.max_displacement_m = _scalar(max_displacement_m, 'max_displacement_m')
        self.max_camera_rotation_deg = _scalar(max_camera_rotation_deg, 'max_camera_rotation_deg')
        if not 0. < self.scale <= 1. or not 0. < self.max_displacement_m <= .02:
            raise AdapterError('first_test_scale_or_translation_scope_invalid')
        if not 0. < self.max_camera_rotation_deg <= 10.:
            raise AdapterError('camera_rotation_scope_invalid')
        self.max_age_s = _scalar(max_age_s, 'max_age_s')
        self.max_monitor_age_s = _scalar(max_monitor_age_s, 'max_monitor_age_s')
        self.max_pose_lag_s = _scalar(max_pose_lag_s, 'max_pose_lag_s')
        self.max_source_gap_s = _scalar(max_source_gap_s, 'max_source_gap_s')
        self.max_delivery_delay_s = _scalar(max_delivery_delay_s, 'max_delivery_delay_s')
        self.max_feedback_age_s = _scalar(max_feedback_age_s, 'max_feedback_age_s')
        self.max_source_step_m = _scalar(max_source_step_m, 'max_source_step_m')
        if min(self.max_age_s, self.max_monitor_age_s, self.max_pose_lag_s,
               self.max_source_gap_s, self.max_delivery_delay_s,
               self.max_feedback_age_s, self.max_source_step_m) <= 0.:
            raise AdapterError('positive_freshness_and_step_limits_required')
        self._lock = threading.RLock()
        self.latest = None
        self.engaged = False
        self.reason = 'not_engaged'
        self.anchor_sample = self.anchor_feedback = None
        self.last_target = None
        self._best_offset_s = None
        self._cursor = None
        self._retired_streams = set()
        self._last_now_ns = None
        self.accepted_samples = 0

    def _now(self, now_ns):
        try:
            now = _integer(now_ns, 'now_ns')
        except AdapterError:
            self.disengage('invalid_host_clock')
            raise
        if self._last_now_ns is not None and now < self._last_now_ns:
            self.disengage('host_clock_regression')
            raise AdapterError('host_clock_regression')
        self._last_now_ns = now
        return now

    def disengage(self, reason='released'):
        with self._lock:
            self.engaged = False
            self.anchor_sample = self.anchor_feedback = None
            self.last_target = None
            self.reason = str(reason)

    def _sample_problem(self, sample, now):
        if sample is None:
            return 'no_pose'
        if sample.serial != 's200085':
            return 'unsupported_camera_serial'
        if not sample.valid or sample.camera_position_m is None or sample.camera_quaternion_xyzw is None:
            return 'invalid_tracking'
        age = (now - sample.received_monotonic_ns) * 1e-9
        monitor_age = (now - sample.monitor_received_monotonic_ns) * 1e-9
        lag = sample.monitor_source_time_s - sample.source_time_s
        if not 0. <= age <= self.max_age_s:
            return 'pose_host_age'
        if not 0. <= monitor_age <= self.max_monitor_age_s:
            return 'monitor_host_age'
        if not -self.max_source_gap_s <= lag <= self.max_pose_lag_s:
            return 'pose_backlog_against_monitor'
        if self._best_offset_s is not None:
            delay = sample.received_monotonic_ns * 1e-9 - sample.source_time_s - self._best_offset_s
            if delay > self.max_delivery_delay_s:
                return 'increasing_delivery_delay'
        return None

    def ingest(self, sample, now_ns):
        """Accept one original sample; duplicates never renew source age.

        Use ingest_line when a raw packet can be malformed. An explicit stream
        change resets timing association but still clears the engagement.
        """
        with self._lock:
            now = self._now(now_ns)
            if not isinstance(sample, NativePose):
                self.latest = None
                self.disengage('malformed_pose_object')
                return False
            previous = self._cursor
            if previous is not None and sample.stream_id == previous.stream_id:
                if sample.sequence == previous.sequence and sample.source_time_s == previous.source_time_s:
                    payload = lambda v: (v.epoch, v.valid, v.loss_count,
                                         v.camera_position_m, v.camera_quaternion_xyzw)
                    if payload(sample) != payload(previous):
                        self.latest = None
                        self.disengage('conflicting_duplicate_packet')
                    return False
                if sample.sequence <= previous.sequence or sample.source_time_s <= previous.source_time_s:
                    self.latest = None
                    self.disengage('source_clock_or_sequence_regression')
                    return False
                if sample.received_monotonic_ns <= previous.received_monotonic_ns:
                    self.latest = None
                    self.disengage('receive_clock_regression')
                    return False
                if sample.epoch < previous.epoch or sample.loss_count < previous.loss_count:
                    self.latest = None
                    self.disengage('native_status_counter_regression')
                    return False
            if previous is not None and sample.stream_id != previous.stream_id:
                if sample.stream_id in self._retired_streams:
                    self.latest = None
                    self.disengage('retired_stream_replay')
                    return False
                self._retired_streams.add(previous.stream_id)
            identity_changed = previous is not None and (sample.stream_id != previous.stream_id or
                sample.epoch != previous.epoch or sample.loss_count != previous.loss_count)
            if identity_changed:
                self.disengage('stream_epoch_or_loss_changed')
                self._best_offset_s = None
            if previous is not None and sample.stream_id == previous.stream_id and not identity_changed:
                if sample.source_time_s - previous.source_time_s > self.max_source_gap_s:
                    self.disengage('source_gap')
                if sample.valid and previous.valid and sample.camera_position_m and previous.camera_position_m:
                    if _norm(tuple(b-a for a,b in zip(previous.camera_position_m, sample.camera_position_m))) > self.max_source_step_m:
                        self.disengage('source_position_jump')
            self.latest = sample
            self._cursor = sample
            problem = self._sample_problem(sample, now)
            if problem:
                self.disengage(problem)
                return False
            offset = sample.received_monotonic_ns * 1e-9 - sample.source_time_s
            self._best_offset_s = offset if self._best_offset_s is None else min(self._best_offset_s, offset)
            self.accepted_samples += 1
            return True

    def ingest_line(self, raw, *, now_ns, **metadata):
        with self._lock:
            try:
                sample = NativePose.from_line(raw, **metadata)
            except (AdapterError, TypeError, ValueError) as exc:
                self._now(now_ns)
                self.disengage('malformed_native_packet:' + str(exc))
                self.latest = None
                return False
            return self.ingest(sample, now_ns)

    def _feedback(self, feedback, now):
        if not isinstance(feedback, FeedbackPose) or feedback.side != self.side or feedback.frame_id != self.robot_frame_id:
            raise AdapterError('feedback_side_or_frame_mismatch')
        position = _vector(feedback.position_m, 3, 'feedback_position')
        quaternion = _unit_quaternion(feedback.quaternion_xyzw)
        stamp = _integer(feedback.measured_monotonic_ns, 'feedback_stamp')
        if not 0. <= (now - stamp) * 1e-9 <= self.max_feedback_age_s:
            raise AdapterError('feedback_stale_or_future')
        return FeedbackPose(position, quaternion, stamp, feedback.side, feedback.frame_id)

    def engage(self, feedback, now_ns):
        """Explicitly re-anchor camera and measured FK; no initial motion."""
        with self._lock:
            now = self._now(now_ns)
            self.disengage('engage_pending')
            problem = self._sample_problem(self.latest, now)
            if problem:
                self.reason = problem
                return False
            try:
                fk = self._feedback(feedback, now)
            except AdapterError as exc:
                self.reason = str(exc)
                return False
            self.anchor_sample, self.anchor_feedback = self.latest, fk
            self.engaged, self.reason = True, 'engaged'
            self.last_target = None
            return True

    def candidate(self, feedback, now_ns):
        with self._lock:
            now = self._now(now_ns)
            if not self.engaged:
                return None
            problem = self._sample_problem(self.latest, now)
            if problem:
                self.disengage(problem)
                return None
            try:
                self._feedback(feedback, now)
            except AdapterError as exc:
                self.disengage(str(exc))
                return None
            source, anchor, fk = self.latest, self.anchor_sample, self.anchor_feedback
            if source.stream_id != anchor.stream_id or source.epoch != anchor.epoch or source.loss_count != anchor.loss_count:
                self.disengage('anchor_identity_mismatch')
                return None
            if rotation_angle_deg(source.camera_quaternion_xyzw, anchor.camera_quaternion_xyzw) > self.max_camera_rotation_deg:
                self.disengage('camera_rotation_exceeds_fixed_orientation_test')
                return None
            delta = tuple(self.scale*v for v in _mv(self.R_BW,
                tuple(b-a for a,b in zip(anchor.camera_position_m, source.camera_position_m))))
            if _norm(delta) > self.max_displacement_m + 1e-12:
                self.disengage('translation_exceeds_first_test_scope')
                return None
            target = IKTarget(self.side, self.robot_frame_id,
                tuple(a+b for a,b in zip(fk.position_m, delta)), fk.quaternion_xyzw,
                delta, source.source_time_s, source.sequence, source.epoch, source.stream_id)
            self.last_target = target
            return target

    def status(self, now_ns):
        with self._lock:
            now = self._now(now_ns)
            problem = self._sample_problem(self.latest, now)
            if self.engaged and problem:
                self.disengage(problem)
            return {'serial': 's200085', 'side': self.side, 'robot_frame_id': self.robot_frame_id,
                    'engaged': self.engaged, 'rearm_required': not self.engaged,
                    'reason': self.reason, 'input_problem': problem,
                    'source_time_s': self.latest.source_time_s if self.latest else None,
                    'source_sequence': self.latest.sequence if self.latest else None,
                    'stream_id': self.latest.stream_id if self.latest else None,
                    'epoch': self.latest.epoch if self.latest else None,
                    'loss_count': self.latest.loss_count if self.latest else None,
                    'accepted_samples': self.accepted_samples, 'hardware_permission': False}
