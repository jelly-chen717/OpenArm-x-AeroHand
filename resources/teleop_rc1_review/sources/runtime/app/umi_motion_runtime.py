"""UMI85 candidates into the existing station IK worker; no motor interface.

The loopback producer owns the camera. This consumer cannot enable motors,
create a ROS trajectory publisher, reset hardware, or select a source on boot.
All Cartesian poses are in the existing robot FK root, metres, xyzw internally.
"""
import json
import math
import socket
import threading
import time
import uuid
from pathlib import Path

from umi_relative import (NativePose, FeedbackPose, UmiTranslationAdapter,
                          rotation_from_axis_observations, rotation_angle_deg)


class UmiMotionRuntime:
    FRAME = 'robot_model_root'
    SIDE = 'right'
    ENDPOINT = ('127.0.0.1', 15758)
    NATIVE_SHA = 'bd959a05a90310dadc29a192035f10b07ca2923c6b1181594d422f10e1860c0d'
    CONFIG_SHA = '8773250732dc9884381158aebbcc5a1ade28018ac271ae2c8845798a72d5a404'

    def __init__(self, worker, robot, state, node=None, *, start=True, clock=time.monotonic_ns,
                 registration_path='/state/umi_motion_session.json'):
        self.worker, self.robot, self.state, self.node = worker, robot, state, node
        self.clock = clock
        self.registration_path = Path(registration_path)
        self.lock = threading.RLock()
        self.token = uuid.uuid4().hex
        self.revision = 0
        self.envelope = None
        self.sample = None
        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip() if Path('/proc/sys/kernel/random/boot_id').exists() else None
        self.validator = UmiTranslationAdapter(side=self.SIDE, robot_frame_id=self.FRAME,
            R_BW=((1,0,0),(0,1,0),(0,0,1)))
        self.transport_error = 'no_pose_producer'
        self.mapping_points = {}
        self.mapping_identity = None
        self.mapping_rotation = None
        self.adapter = None
        self.active = False
        self.reason = 'paused'
        self.source_epoch = None
        self.hold_q = None
        self.last_submitted_pose = None
        self.pending_sequence = None
        self.bridge_state = None
        self.bridge_future = None
        self.bridge_received_ns = 0
        self.closed = threading.Event()
        if node is not None:
            from std_srvs.srv import Trigger
            self._trigger = Trigger
            self.bridge_client = node.create_client(Trigger, '/archive_control/bridge_state')
            self.bridge_timer = node.create_timer(.2, self._poll_bridge)
        if start:
            self.thread = threading.Thread(target=self._run, name='umi85-ingress', daemon=True)
            self.thread.start()

    def _poll_bridge(self):
        if self.bridge_future is not None and not self.bridge_future.done():
            return
        if not self.bridge_client.service_is_ready():
            return
        self.bridge_future = self.bridge_client.call_async(self._trigger.Request())
        self.bridge_future.add_done_callback(self._bridge_reply)

    def _bridge_reply(self, future):
        try:
            response = future.result()
            value = json.loads(response.message)
            if not response.success or value.get('kind') != 'archive_control_bridge_state_v1':
                raise ValueError('invalid_bridge_state')
            with self.lock:
                previous = self.bridge_state
                self.bridge_state, self.bridge_received_ns = value, self.clock()
                if (previous and previous.get('instance_id') != value.get('instance_id')) or value.get('home_active'):
                    self._pause('original_reset_or_bridge_restart')
        except Exception:
            with self.lock:
                self.bridge_state = None
                self._pause('bridge_state_unavailable')

    def _bridge_problem(self, now):
        value = self.bridge_state
        if not value or not 0 <= (now-self.bridge_received_ns)/1e9 <= .6:
            return 'bridge_status_stale'
        try:
            age = now/1e9 - float(value['heartbeat_monotonic_s'])
            if not math.isfinite(age) or not 0 <= age <= .6:
                return 'bridge_heartbeat_stale'
        except (KeyError, TypeError, ValueError):
            return 'bridge_heartbeat_invalid'
        if value.get('bridge_enabled') is not True:
            return 'manual_motor_enable_required'
        if value.get('home_active') or any(v is not None for v in value.get('home_targets', {}).values()):
            return 'original_reset_active'
        return None

    def _identity(self, sample):
        return (sample.stream_id, sample.epoch, sample.loss_count)

    def _pause(self, reason):
        was_armed = self.active or (self.adapter and self.adapter.engaged)
        self.active = False
        self.reason = reason
        if self.adapter:
            self.adapter.disengage(reason)
        self.source_epoch = None
        self.hold_q = None
        self.last_submitted_pose = None
        self.pending_sequence = None
        if was_armed and self.worker.get_input_source() == 'umi':
            self.worker.pause_umi(reason)

    def _transport_problem(self, now):
        e, s = self.envelope, self.sample
        if self.transport_error:
            return self.transport_error
        if not e or not s:
            return 'no_native_pose'
        if e.get('ba2_ready') is not True:
            return 'vio_ba2_not_ready'
        if not s.valid:
            return 'tracking_invalid'
        if not 0 <= (now-s.received_monotonic_ns)/1e9 <= .25:
            return 'pose_host_age'
        if not 0 <= (now-s.monitor_received_monotonic_ns)/1e9 <= .25:
            return 'monitor_host_age'
        if not -.3 <= s.monitor_source_time_s-s.source_time_s <= .25:
            return 'pose_backlog_against_monitor'
        return None

    def ingest_envelope(self, e):
        """Consume original producer timestamps; polling cannot renew freshness."""
        now = self.clock()
        with self.lock:
            try:
                if (not isinstance(e, dict) or e.get('schema') != 'oa.umi.vio-pose-envelope.v1'
                        or e.get('serial') != 's200085' or e.get('healthy') is not True):
                    raise ValueError(e.get('reason', 'unhealthy_producer') if isinstance(e, dict) else 'bad_envelope')
                if self.boot_id and not str(e.get('stream_id','')).startswith(self.boot_id+':'):
                    raise ValueError('producer_boot_identity_mismatch')
                try:
                    raw = self.registration_path.read_bytes()
                    if len(raw) > 4096:
                        raise ValueError('registration_too_large')
                    registered = json.loads(raw)
                except (OSError, ValueError) as exc:
                    raise ValueError('producer_session_not_registered') from exc
                if (registered.get('schema') != 'oa.umi.producer-registration.v1'
                        or registered.get('active') is not True
                        or registered.get('stream_id') != e['stream_id']
                        or registered.get('native_sha256') != self.NATIVE_SHA
                        or registered.get('config_sha256') != self.CONFIG_SHA
                        or (self.boot_id and registered.get('boot_id') != self.boot_id)):
                    raise ValueError('producer_registration_mismatch')
                s = NativePose.from_line(e['raw'], stream_id=e['stream_id'],
                    sequence=e['pose_sequence'], received_monotonic_ns=e['received_monotonic_ns'],
                    monitor_source_time_s=e['monitor_source_time_s'],
                    monitor_received_monotonic_ns=e['monitor_received_monotonic_ns'])
                old = self.sample
                # Identity changes discard mapping as VIO world may have changed.
                if old and self._identity(old) != self._identity(s):
                    self._pause('vio_identity_changed')
                    self.mapping_rotation = None
                    self.mapping_points.clear()
                    self.mapping_identity = None
                    self.adapter = None
                if old and old.stream_id == s.stream_id and (s.sequence < old.sequence or
                       s.source_time_s < old.source_time_s):
                    raise ValueError('producer_order_regression')
                self.validator.ingest(s, now)
                if self.validator.latest is None or self.validator.status(now)['input_problem']:
                    raise ValueError(self.validator.reason)
                # A duplicate heartbeat retains the original sample and timestamp.
                s = self.validator.latest
                self.envelope, self.sample, self.transport_error = e, s, None
                problem = self._transport_problem(now)
                if self.adapter:
                    self.adapter.ingest(s, now)
                    if self.active and not self.adapter.engaged:
                        problem = self.adapter.reason
                if problem:
                    self._pause(problem)
            except (ValueError, TypeError, KeyError) as exc:
                self.transport_error = str(exc)
                self._pause(self.transport_error)

    def _feedback(self):
        value = self.worker.get_feedback_snapshot(max_age_s=.25)
        pose = value['ee'][self.SIDE]
        q = pose['orientation']
        p = pose['position']
        fk = FeedbackPose(tuple(p[k] for k in ('x', 'y', 'z')),
            tuple(q[k] for k in ('x', 'y', 'z', 'w')), value['received_monotonic_ns'],
            self.SIDE, value['frame_id'])
        return value, fk

    @staticmethod
    def _target_dict(target):
        x,y,z,w = target.quaternion_xyzw
        return {target.side: {'position': dict(zip(('x','y','z'), target.position_m)),
                'orientation': {'w':w,'x':x,'y':y,'z':z}}}

    def _require_source(self):
        if self.worker.get_input_source() != 'umi':
            raise ValueError('select_umi_source_first')

    def _require_pose(self):
        problem = self._transport_problem(self.clock())
        if problem:
            raise ValueError(problem)
        return self.sample

    def action(self, data):
        try:
            return self._action(data)
        except (ValueError, TypeError, KeyError):
            with self.lock:
                self._pause('action_failed_reanchor_required')
            raise

    def _action(self, data):
        with self.lock:
            action = data.get('action')
            if action == 'source':
                source = data.get('source')
                if source not in ('none', 'vr', 'umi'):
                    raise ValueError('unknown_input_source')
                self._pause('source_changed')
                self.worker.select_input_source(source)
                self.reason = 'source_selected:' + source
            elif action == 'pause':
                self._pause('web_manual_pause')
            elif action == 'map_capture':
                self._require_source()
                self._pause('mapping_capture')
                s = self._require_pose()
                point = data.get('point')
                if point not in ('origin', 'x', 'y'):
                    raise ValueError('unknown_mapping_point')
                if point == 'origin':
                    self.mapping_points.clear()
                    self.mapping_rotation = None
                    self.mapping_identity = self._identity(s)
                    self.adapter = None
                if self.mapping_identity != self._identity(s):
                    raise ValueError('capture_mapping_origin_first')
                origin = self.mapping_points.get('origin')
                if origin and rotation_angle_deg(origin.camera_quaternion_xyzw, s.camera_quaternion_xyzw) > 2:
                    raise ValueError('mapping_camera_rotation_exceeds_2deg')
                self.mapping_points[point] = s
            elif action == 'map_apply':
                self._require_source()
                self._pause('mapping_apply')
                s = self._require_pose()
                if self.mapping_identity != self._identity(s):
                    raise ValueError('mapping_world_changed')
                try:
                    p = self.mapping_points
                    origin = p['origin'].camera_position_m
                    x = tuple(b-a for a,b in zip(origin,p['x'].camera_position_m))
                    y = tuple(b-a for a,b in zip(origin,p['y'].camera_position_m))
                except KeyError:
                    raise ValueError('mapping_requires_origin_x_y')
                self.mapping_rotation = rotation_from_axis_observations(x, y)
                self.adapter = UmiTranslationAdapter(side=self.SIDE, robot_frame_id=self.FRAME,
                    R_BW=self.mapping_rotation)
                self.adapter.ingest(s, self.clock())
                self.reason = 'mapping_ready'
            elif action == 'anchor':
                self._require_source()
                self._pause('anchor_requested')
                self._require_pose()
                if not self.adapter:
                    raise ValueError('mapping_required')
                problem = self._bridge_problem(self.clock())
                if problem:
                    raise ValueError(problem)
                feedback, fk = self._feedback()
                self.adapter.ingest(self.sample, self.clock())
                if not self.adapter.engage(fk, self.clock()):
                    raise ValueError(self.adapter.reason)
                self.hold_q = list(feedback['q'])
                self.source_epoch = feedback['epoch']
                self.reason = 'anchored_preview_only'
                self._submit(preview=True)
            elif action == 'preview':
                self._submit(preview=True)
            elif action == 'engage':
                if self.active:
                    raise ValueError('already_following_pause_before_reanchor')
                self._require_source()
                self._require_pose()
                if not self.adapter or not self.adapter.engaged:
                    raise ValueError('anchor_and_preview_first')
                problem = self._bridge_problem(self.clock())
                if problem:
                    raise ValueError(problem)
                preview = self.state.get('umi_preview') or {}
                if preview.get('accepted') is not True or preview.get('source_epoch') != self.source_epoch:
                    raise ValueError('accepted_ik_preview_required')
                if not 0 <= (self.clock()-preview.get('completed_monotonic_ns',0))/1e9 <= 1.:
                    raise ValueError('fresh_ik_preview_required')
                # Engagement anchors again, so no jump from a dry-preview offset.
                feedback, fk = self._feedback()
                if not self.adapter.engage(fk, self.clock()):
                    raise ValueError(self.adapter.reason)
                self.hold_q = list(feedback['q'])
                self.source_epoch = self.worker.arm_umi_anchor(self.source_epoch)
                self.active, self.reason = True, 'following_right_translation'
                self.last_submitted_pose = None
                self.pending_sequence = None
                self._submit(preview=False)
            else:
                raise ValueError('unknown_action')
            self.revision += 1
            return self.status()

    def _submit(self, *, preview):
        self._require_source()
        self._require_pose()
        if not self.adapter or not self.adapter.engaged or self.hold_q is None:
            raise ValueError('anchor_required')
        problem = self._bridge_problem(self.clock())
        if problem:
            raise ValueError(problem)
        if self.pending_sequence is not None:
            report = self.state.get('umi_result') or {}
            if report.get('sequence') != self.pending_sequence:
                return
        feedback, fk = self._feedback()
        if feedback['epoch'] != self.source_epoch:
            raise ValueError('source_epoch_changed_reanchor')
        target = self.adapter.candidate(fk, self.clock())
        if target is None:
            raise ValueError(self.adapter.reason)
        self.pending_sequence = self.worker.submit_umi_targets(self._target_dict(target), source_epoch=self.source_epoch,
            deadline_monotonic_ns=min(self.sample.received_monotonic_ns+250_000_000,
                                      self.sample.monitor_received_monotonic_ns+250_000_000),
            side=self.SIDE, preview=preview, hold_q=self.hold_q)
        self.last_submitted_pose = self.sample.sequence

    def tick(self):
        with self.lock:
            if not self.adapter or not self.adapter.engaged:
                return
            try:
                self._require_source()
                self._require_pose()
                if self.worker.get_feedback_snapshot(max_age_s=.25)['epoch'] != self.source_epoch:
                    raise ValueError('source_epoch_changed_reanchor')
                report = self.state.get('umi_result') or {}
                if (report.get('source_epoch') == self.source_epoch and
                        report.get('sequence') == self.pending_sequence and report.get('accepted') is False):
                    raise ValueError('ik_rejected:' + str(report.get('reason')))
                if self.sample.sequence != self.last_submitted_pose:
                    self._submit(preview=not self.active)
            except (ValueError, TypeError, KeyError) as exc:
                self._pause(str(exc))

    def status(self):
        with self.lock:
            now = self.clock()
            # Report actual measurements, never q predicted by IK.
            try:
                feedback, _ = self._feedback()
                feedback_ok, feedback_error = True, None
            except (ValueError, TypeError, KeyError) as exc:
                feedback, feedback_ok, feedback_error = None, False, str(exc)
            problem = self._transport_problem(now)
            return {'schema':'oa.umi.motion.v1', 'revision':self.revision,
                'csrf_token':self.token, 'input_source':self.worker.get_input_source(),
                'side':self.SIDE, 'serial':'s200085', 'fixed_orientation':True,
                'max_translation_m':.02, 'active':self.active, 'reason':self.reason,
                'feedback_ok':feedback_ok, 'feedback_error':feedback_error,
                'feedback':feedback, 'bridge_problem':self._bridge_problem(now),
                'transport':{'healthy':problem is None, 'ba2_ready':bool(self.envelope and self.envelope.get('ba2_ready')),
                    'reason':problem, 'stream_id':self.sample.stream_id if self.sample else None,
                    'epoch':self.sample.epoch if self.sample else None},
                'mapping':{'ready':self.mapping_rotation is not None, 'points':list(self.mapping_points),
                    'R_BW':self.mapping_rotation, 'frame_id':self.FRAME},
                'adapter':self.adapter.status(now) if self.adapter else None,
                'preview':self.state.get('umi_preview'), 'ik_result':self.state.get('umi_result'),
                'motor_enable':'existing_web_manual_only', 'new_can_writer':False}

    def _run(self):
        try:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            receiver.bind(self.ENDPOINT)
            receiver.settimeout(.02)
            try:
                while not self.closed.is_set():
                    try:
                        raw, peer = receiver.recvfrom(8193)
                        if peer[0] != '127.0.0.1' or len(raw) > 8192:
                            raise ValueError('invalid_envelope_size_or_peer')
                        self.ingest_envelope(json.loads(raw))
                    except socket.timeout:
                        pass
                    except (ValueError, UnicodeDecodeError):
                        with self.lock:
                            self.transport_error = 'invalid_envelope_json'
                            self._pause(self.transport_error)
                    self.tick()
            finally:
                receiver.close()
        except Exception as exc:
            with self.lock:
                self.transport_error = 'receiver_failed:' + type(exc).__name__
                self._pause(self.transport_error)
